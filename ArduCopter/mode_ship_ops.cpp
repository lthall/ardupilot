#include "Copter.h"

#if MODE_SHIP_OPS_ENABLED == ENABLED

/*
 * mode_ship_ops.cpp - Ship take off and landing referenced to a mavlink-enabled system id
 */

// initialise ship operations mode
bool ModeShipOperation::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        // follow not enabled
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }
    if (!g2.follow.have_target()) {
        // follow does not have a target
        gcs().send_text(MAV_SEVERITY_WARNING, "No beacon detected");
        return false;
    }
    if (!g2.follow.offsets_are_set()) {
        // follow does not have a target
        gcs().send_text(MAV_SEVERITY_WARNING, "FOLL_OFS X, Y, Z not set");
        return false;
    }

    pos_control->set_max_speed_xy(wp_nav->get_default_speed_xy());
    pos_control->set_max_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_max_speed_z(wp_nav->get_default_speed_down(),
                                 wp_nav->get_default_speed_up());
    pos_control->set_max_accel_z(wp_nav->get_accel_z());
    pos_control->calc_leash_length_xy();
    pos_control->calc_leash_length_z();
    pos_control->init_pos_vel_xy();

    if (get_alt_hold_state(0.0f) == AltHold_Flying) {
        // we are flying so we will initialise at the climb to RTL altitude.
        _state = ShipOps_ClimbToRTL;
        gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ClimbToRTL");
        ship_takeoff = false;
    } else {
        // we are landed so we will initialise in the Final state.
        _state = ShipOps_LaunchRecovery;
        gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_LaunchRecovery");
        ship_takeoff = true;
    }

    offset.zero();

    return true;
}

// perform cleanup required when leaving ship operations mode
void ModeShipOperation::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeShipOperation::run()
{
    float takeoff_climb_rate;
    float nav_roll;
    float nav_pitch;

    Vector3f desired_velocity_neu_cms;
    float yaw_cd = 0.0f;

    Vector2f perch_offset = Vector2f(g2.ship_perch_radius * 100.0, 0.0f);
    perch_offset.rotate(radians(g2.ship_perch_angle));
    float perch_height = g2.ship_perch_altitude * 100.0;

    // using the small angle approximation for simplicity and a conservative result when maximum acceleration is large
    // this assumes the time taken to achieve the maximum acceleration angle is limited by the angular acceleration rather than maximum angular rate.
    float lean_angle = wp_nav->get_wp_acceleration() / (GRAVITY_MSS * 100.0 * M_PI / 18000.0);
    float angle_accel = MIN(attitude_control->get_accel_pitch_max(), attitude_control->get_accel_roll_max());
    float tc = 2.0 * sqrtf(lean_angle / angle_accel);

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // get pilot desired climb rate if enabled
    if (!copter.failsafe.radio && copter.ap.rc_receiver_present && g.land_repositioning) {
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        if (ship_takeoff) {
            target_climb_rate = g.pilot_speed_up;
        } else {
            target_climb_rate = -get_pilot_speed_dn();
        }
    }

    Vector3f pos_with_ofs;  // vector to lead vehicle + offset
    Vector3f vel_ned;  // velocity of lead vehicle
    bool ship_availible = g2.follow.update_target_pos_and_vel();
    if (ship_availible) {
        g2.follow.get_target_pos_and_vel_ned(pos_with_ofs, vel_ned);
        pos_with_ofs *= 100.0f;
        vel_ned *= 100.0f;

        // calculate vehicle heading
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                Vector3f dist_vec;  // vector to lead vehicle
                g2.follow.get_target_dist_ned(dist_vec);
                const Vector3f dist_vec_xy(dist_vec.x, dist_vec.y, 0.0f);
                if (dist_vec_xy.length() > 1.0f) {
                    yaw_cd = get_bearing_cd(Vector3f(), dist_vec_xy);
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                float target_hdg = 0.0f;
                if (g2.follow.get_target_heading_deg(target_hdg)) {
                    yaw_cd = target_hdg * 100.0f;
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                Vector3f vel_target = pos_control->get_vel_target();
                const Vector3f vel_vec(vel_target.x, vel_target.y, 0.0f);
                if (vel_vec.length() > 100.0f) {
                    yaw_cd = get_bearing_cd(Vector3f(), vel_vec);
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
               break;
        }
    } else {
        _state = ShipOps_ClimbToRTL;
    }

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->init_pos_vel_xy();
        offset = pos_control->get_pos_target() - pos_with_ofs;
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->init_pos_vel_xy();
        offset = pos_control->get_pos_target() - pos_with_ofs;
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(MAX(g.pilot_takeoff_alt, g2.wp_navalt_min * 100), 0.0f, 1000.0f));
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // set position controller targets
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        // tell the position controller that we have limited roll/pitch demand to prevent integrator buildup
        pos_control->init_pos_vel_xy();
        offset = pos_control->get_pos_target() - pos_with_ofs;
        break;

    case AltHold_Flying:
        // define target location
        float target_hdg = 0.0f;
        if (g2.follow.get_target_heading_deg(target_hdg)) {
            perch_offset.rotate(radians(target_hdg));
        }
        switch (_state) {
        case ShipOps_ClimbToRTL:
            // climb to RTL altitude
            offset.z = -(float)g.rtl_altitude;
            pos_control->set_alt_target_with_slew(-offset.z, G_Dt);
            break;
        case ShipOps_ReturnToPerch:
            // move to Perch location at RTL altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = -(float)g.rtl_altitude;
            pos_control->set_alt_target_with_slew(-offset.z, G_Dt);
            break;
            // FALLTHROUGH
        case ShipOps_Perch:
            // move to Perch location and altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = -perch_height;
            pos_control->set_alt_target_with_slew(-offset.z, G_Dt);
            break;
        case ShipOps_OverSpot:
            // move over landing Spot at Perch altitude
            offset.zero();
            offset.z = -perch_height;
            pos_control->set_alt_target_with_slew(-offset.z, G_Dt);
            break;
        case ShipOps_LaunchRecovery:
            // ascend to Perch altitude when throttle high
            // descend to deck when throttle is low
            // calculate pilot repositioning
            float roll_in = 0.0f;
            float pitch_in = 0.0f;
            if (!copter.failsafe.radio && copter.ap.rc_receiver_present && g.land_repositioning) {
                // do not reposition unless valid pilot input is present
                const float scale = 1.0 / ROLL_PITCH_YAW_INPUT_MAX;
                roll_in = channel_roll->get_control_in() * scale;
                pitch_in = channel_pitch->get_control_in() * scale;
                // limit correction speed to 1 second of acceleration
                const float speed_max = wp_nav->get_wp_acceleration();
                Vector2f pos_change(-pitch_in, roll_in);
                pos_change *= G_Dt * speed_max;
                pos_change.rotate(ahrs.yaw);
                offset.x += pos_change.x;
                offset.y += pos_change.y;
            }
            // set reposition state
            copter.ap.land_repo_active = (!is_zero(roll_in) || !is_zero(pitch_in));

            float max_land_descent_velocity;
            if (g.land_speed_high > 0) {
                max_land_descent_velocity = -g.land_speed_high;
            } else {
                max_land_descent_velocity = pos_control->get_max_speed_down();
            }
            max_land_descent_velocity = MIN(max_land_descent_velocity, -abs(g.land_speed));
            float alt_above_deck = fabsf(-pos_with_ofs.z - pos_control->get_pos_target().z);
            if (copter.rangefinder_alt_ok()) {
                // check if range finder detects the deck is closer than expected
                alt_above_deck = MIN(alt_above_deck, copter.rangefinder_state.alt_cm_filt.get());
            }
            // reduce decent rate to land_speed when below land_alt_low
            float cmb_rate = constrain_float(sqrt_controller(MAX(g2.land_alt_low, 100) - alt_above_deck, pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z(), G_Dt), -get_pilot_speed_dn(), -abs(g.land_speed));
            target_climb_rate = constrain_float(target_climb_rate, cmb_rate, g.pilot_speed_up);
            pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            offset.z = -perch_height;
            break;
        }

        // navigate to target location
        switch (_state) {
        case ShipOps_ClimbToRTL:
            // slow to zero velocity and climb to RTL altitude using velocity control
            vel_ned.zero();
            pos_control->input_vel_xy(vel_ned, wp_nav->get_wp_acceleration(), tc);
            break;
        case ShipOps_ReturnToPerch:
            // FALLTHROUGH
        case ShipOps_Perch:
            // FALLTHROUGH
        case ShipOps_OverSpot:
            // FALLTHROUGH
        case ShipOps_LaunchRecovery:
            // move to target position and velocity using position and velocity control
            Vector3f pos = pos_with_ofs;
            pos += offset;
            // relax stop target if we might be landed
            if (copter.ap.land_complete_maybe) {
                loiter_nav->soften_for_landing();
            }
            pos_control->input_pos_vel_xy(pos, vel_ned,
                    wp_nav->get_default_speed_xy(),
                    wp_nav->get_default_speed_xy() + vel_ned.length(),
                    wp_nav->get_wp_acceleration(), tc);

            // reset landing offset to the current stopping point when
            // pilot correction is active.
            if (copter.ap.land_repo_active) {
                offset = (pos - pos_with_ofs);
            }
            break;
        }

        // update state of Ship Operations

        Vector3f pos_error = pos_with_ofs + offset - pos_control->get_pos_target();
        bool pos_check;
        // altitude is less than 5% of the Perch height
        bool alt_check = fabsf(-offset.z - pos_control->get_pos_target().z) < perch_height * 0.05f;
        switch (_state) {
        case ShipOps_ClimbToRTL:
            // check altitude is within 5% of the Perch radius from RTL altitude
            if (ship_availible && (-offset.z - pos_control->get_pos_target().z < perch_height * 0.05f)) {
                _state = ShipOps_ReturnToPerch;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
            }
            break;
        case ShipOps_ReturnToPerch:
            // check that position is within 10% of the Perch radius in x and y
            // if throttle is low then reduce altitude to Perch altitude
            pos_check = Vector2f(pos_error.x, pos_error.y).length() < g2.ship_perch_radius * 10.0f;
            if (pos_check) {
                _state = ShipOps_Perch;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_Perch");
            }
            break;
        case ShipOps_Perch:
            // if altitude is correct and throttle is low then continue landing
            if (alt_check && is_negative(target_climb_rate)) {
                _state = ShipOps_OverSpot;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_OverSpot");
            }
            break;
        case ShipOps_OverSpot:
            // check position is within 10 percent of the Perch height
            // if accent requested then move back to Perch location
            // if decent requested then continue recovery
            pos_check = Vector2f(pos_error.x, pos_error.y).length() < perch_height * 0.1f;
            if (pos_check && is_negative(target_climb_rate)) {
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_LaunchRecovery");
                _state = ShipOps_LaunchRecovery;
            } else if (is_positive(target_climb_rate)) {
                _state = ShipOps_Perch;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
                // optionally deploy landing gear
                copter.landinggear.retract_after_takeoff();
            }
            break;
        case ShipOps_LaunchRecovery:
            // if accent requested and altitude has reached or exceeded the perch altitude then move to Perch
            if (!is_positive(-offset.z - pos_control->get_pos_target().z) && is_positive(target_climb_rate)) {
                _state = ShipOps_Perch;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
                // optionally deploy landing gear
                copter.landinggear.deploy_for_landing();
            }
            break;
        }

        nav_roll = pos_control->get_roll();
        nav_pitch = pos_control->get_pitch();

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(nav_roll, nav_pitch, copter.aparm.angle_max);
#endif
        break;
    }
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(nav_roll, nav_pitch, yaw_cd, true);
}

bool ModeShipOperation::is_landing() const
{
    return is_negative(target_climb_rate);
}

uint32_t ModeShipOperation::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeShipOperation::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeShipOperation::get_wp(Location &loc)
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_SHIP_OPS_ENABLED == ENABLED
