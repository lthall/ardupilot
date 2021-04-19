#include "Copter.h"

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

// brake_init - initialise brake controller
bool ModeBrake::init(bool ignore_checks)
{
    // set target to current position
    init_target();

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->init_pos_vel_accel_z();
    }

    _timeout_ms = 0;

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        init_target();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // update altitude target and call position controller
    // protects heli's from inflight motor interlock disable
    Vector3f vel;
    if (motors->get_desired_spool_state() == AP_Motors::DesiredSpoolState::GROUND_IDLE && !copter.ap.land_complete) {
        vel.z = -abs(g.land_speed);
    }

    // update position controller with new target
    pos_control->input_vel_accel_xy(vel, Vector3f());
    pos_control->input_vel_accel_z(vel, Vector3f());

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    }
}

void ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}

void ModeBrake::init_target()
{
    // initialise position controller
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    pos_control->init_pos_vel_accel_xyz();

    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity().length(), BRAKE_MODE_DECEL_RATE);

    // set target position
    Vector3f stopping_point;
    pos_control->get_stopping_point_xy(stopping_point);
    pos_control->set_target_pos_xy(stopping_point.x, stopping_point.y);
}

#endif
