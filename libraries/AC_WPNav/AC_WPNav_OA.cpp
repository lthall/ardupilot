#include <AP_Math/control.h>
#include <AP_InternalError/AP_InternalError.h>
#include "AC_WPNav_OA.h"

AC_WPNav_OA::AC_WPNav_OA(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    AC_WPNav(inav, ahrs, pos_control, attitude_control)
{
}

// returns object avoidance adjusted wp location using location class
// returns false if unable to convert from target vector to global coordinates
bool AC_WPNav_OA::get_oa_wp_destination(Location& destination) const
{
    // if oa inactive return unadjusted location
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return get_wp_destination_loc(destination);
    }

    // return latest destination provided by oa path planner
    destination = _oa_destination;
    return true;
}

/// set_wp_destination waypoint using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav_OA::set_wp_destination(const Vector3f& destination, bool terrain_alt)
{
    const bool ret = AC_WPNav::set_wp_destination(destination, terrain_alt);

    if (ret) {
        // reset object avoidance state
        _oa_state = AP_OAPathPlanner::OA_NOT_REQUIRED;
    }

    return ret;
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
/// always returns distance to final destination (i.e. does not use oa adjusted destination)
float AC_WPNav_OA::get_wp_distance_to_destination() const
{
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_distance_to_destination();
    }

    // get current location
    const Vector3f &curr = _inav.get_position();
    return norm(_destination_oabak.x-curr.x, _destination_oabak.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
/// always returns bearing to final destination (i.e. does not use oa adjusted destination)
int32_t AC_WPNav_OA::get_wp_bearing_to_destination() const
{
    if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
        return AC_WPNav::get_wp_bearing_to_destination();
    }

    return get_bearing_cd(_inav.get_position(), _destination_oabak);
}

/// true when we have come within RADIUS cm of the waypoint
bool AC_WPNav_OA::reached_wp_destination() const
{
    return (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) && AC_WPNav::reached_wp_destination();
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav_OA::update_wpnav()
{
    // run path planning around obstacles
    AP_OAPathPlanner *oa_ptr = AP_OAPathPlanner::get_singleton();
    Location current_loc;
    if ((oa_ptr != nullptr) && AP::ahrs().get_position(current_loc)) {

        // backup _origin and _destination when not doing oa
        if (_oa_state == AP_OAPathPlanner::OA_NOT_REQUIRED) {
            _origin_oabak = _origin;
            _destination_oabak = _destination;
            _terrain_alt_oabak = _terrain_alt;
        }

        // convert origin and destination to Locations and pass into oa
        // Note: altitudes are incorrect if using terrain altitudes but current path planners do not use altitude anyway
        Location origin_loc(_origin_oabak);
        Location destination_loc(_destination_oabak);
        Location oa_origin_new, oa_destination_new;
        AP_OAPathPlanner::OAPathPlannerUsed path_planner_used = AP_OAPathPlanner::OAPathPlannerUsed::None;
        const AP_OAPathPlanner::OA_RetState oa_retstate = oa_ptr->mission_avoidance(current_loc, origin_loc, destination_loc, oa_origin_new, oa_destination_new, path_planner_used);
        switch (oa_retstate) {
        case AP_OAPathPlanner::OA_NOT_REQUIRED:
            if (_oa_state != oa_retstate) {
                // object avoidance has become inactive so reset target to original destination
                set_wp_destination(_destination_oabak, _terrain_alt_oabak);
                _oa_state = oa_retstate;
            }
            break;
        case AP_OAPathPlanner::OA_PROCESSING:
        case AP_OAPathPlanner::OA_ERROR:
            // during processing or in case of error stop the vehicle
            // by setting the oa_destination to a stopping point
            if ((_oa_state != AP_OAPathPlanner::OA_PROCESSING) && (_oa_state != AP_OAPathPlanner::OA_ERROR)) {
                // calculate stopping point
                Vector3f stopping_point;
                get_wp_stopping_point(stopping_point);
                _oa_destination = Location(stopping_point);
                if (set_wp_destination(stopping_point, false)) {
                    _oa_state = oa_retstate;
                }
            }
            break;
        case AP_OAPathPlanner::OA_SUCCESS:

            // handling of returned destination depends upon path planner used
            switch (path_planner_used) {

            case AP_OAPathPlanner::OAPathPlannerUsed::None:
                // this should never happen.  this means the path planner has returned success but has failed to set the path planner used
                INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                return false;

            case AP_OAPathPlanner::OAPathPlannerUsed::Dijkstras:
                // Dijkstra's.  Action is only needed if path planner has just became active or the target destination's lat or lon has changed
                if ((_oa_state != AP_OAPathPlanner::OA_SUCCESS) || !oa_destination_new.same_latlon_as(_oa_destination)) {
                    Location origin_oabak_loc(_origin_oabak);
                    Location destination_oabak_loc(_destination_oabak);
                    if (_terrain_alt_oabak) {
                        // if original destination is alt-above-terrain correct altitude
                        origin_oabak_loc.set_alt_cm(_origin_oabak.z, Location::AltFrame::ABOVE_TERRAIN);
                        destination_oabak_loc.set_alt_cm(_destination_oabak.z, Location::AltFrame::ABOVE_TERRAIN);
                    }
                    oa_destination_new.linearly_interpolate_alt(origin_oabak_loc, destination_oabak_loc);
                    if (!set_wp_destination_loc(oa_destination_new)) {
                        // trigger terrain failsafe
                        return false;
                    }
                    // if new target set successfully, update oa state and destination
                    _oa_state = oa_retstate;
                    _oa_destination = oa_destination_new;
                }
                break;

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerHorizontal: {
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;

                // altitude target interpolated from current_loc's distance along the original path
                Location target_alt_loc = current_loc;
                target_alt_loc.linearly_interpolate_alt(origin_loc, destination_loc);

                // correct target_alt_loc's alt-above-ekf-origin if using terrain altitudes
                if (_terrain_alt_oabak) {
                    // get terrain offset. positive terr_offset means terrain below vehicle is above ekf origin's altitude
                    float terr_offset;
                    if (!get_terrain_offset(terr_offset)) {
                        // trigger terrain failsafe
                        return false;
                    }
                    target_alt_loc.alt += terr_offset;
                }

                // calculate final destination as an offset from EKF origin in NEU
                Vector2f dest_NE;
                if (!_oa_destination.get_vector_xy_from_origin_NE(dest_NE)) {
                    // this should never happen because we can only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }
                Vector3f dest_NEU{dest_NE.x, dest_NE.y, (float)target_alt_loc.alt};

                // pass the desired position directly to the position controller as an offset from EKF origin in NEU
                set_pos_control_target_NEU(dest_NEU);

                // return success without calling parent AC_WPNav
                return true;
            }

            case AP_OAPathPlanner::OAPathPlannerUsed::BendyRulerVertical:
            {
                _oa_state = oa_retstate;
                _oa_destination = oa_destination_new;

                // calculate final destination as an offset from EKF origin in NEU
                Vector3f dest_NEU;
                if (!_oa_destination.get_vector_from_origin_NEU(dest_NEU)) {
                    // this should never happen because we acan only get here if we have an EKF origin
                    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
                    return false;
                }

                // pass the desired position directly to the position controller as an offset from EKF origin in NEU
                set_pos_control_target_NEU(dest_NEU);

                // return success without calling parent AC_WPNav
                return true;
            }

            } // case AP_OAPathPlanner::OA_SUCCESS
        } // switch (oa_retstate)
    }

    // run the non-OA update
    return AC_WPNav::update_wpnav();
}

/// set position control target in NEU offset in cm from EKF origin
/// used to process results from BendyRuler
void AC_WPNav_OA::set_pos_control_target_NEU(Vector3f &dest_neu)
{
    // pass the desired position directly to the position controller as an offset from EKF origin in NEU
    const float lean_angle = get_wp_acceleration() / (GRAVITY_MSS * 100.0f * M_PI / 18000.0);
    const float angle_accel = MIN(_attitude_control.get_accel_pitch_max(), _attitude_control.get_accel_roll_max());
    const float tc = 2.0f * sqrtf(lean_angle / angle_accel);
    _pos_control.input_pos_vel_xy(dest_neu, Vector3f{},
                                  get_default_speed_xy(),
                                  get_default_speed_xy(),
                                  get_wp_acceleration(), tc);

    // pass z-axis target to position controller
    _pos_control.input_pos_vel_z(dest_neu, Vector3f{},
                                 get_default_speed_up(),
                                 get_default_speed_up(),
                                 get_accel_z(), tc);

    // calculate the yaw angle and rate
    /*float yaw_angle_rad, yaw_rate_rads;
    if (_pos_control.get_yaw_angle_and_rate(yaw_angle_rad, yaw_rate_rads)) {
        set_yaw_cd(degrees(yaw_angle_rad) * 100.0f);
        set_yaw_rate_cds(degrees(yaw_rate_rads) * 100.0f);
    }*/

    AP::logger().Write("BR", "TimeUS,TX,TY,TZ,SXY,AXY,SZ,AZ,tc",
                             "Qffffffff",
                             AP_HAL::micros64(),
                             dest_neu.x * 0.01f,
                             dest_neu.y * 0.01f,
                             dest_neu.z * 0.01f,
                             get_default_speed_xy() * 0.01f,
                             get_wp_acceleration() * 0.01f,
                             get_default_speed_up() * 0.01f,
                             get_accel_z() * 0.01f,
                             tc);

    // @LoggerMessage: DPTH
    // @Description: Depth messages on boats with downwards facing range finder
    // @Field: TimeUS: Time since system startup
    // @Field: Lat: Latitude
    // @Field: Lng: Longitude
    // @Field: Depth: Depth as detected by the sensor

    /*logger.Write("DPTH", "TimeUS,Lat,Lng,Depth",
                        "sDUm", "FGG0", "QLLf",
                        AP_HAL::micros64(),
                        loc.lat,
                        loc.lng,
                        (double)(rangefinder.distance_cm_orient(ROTATION_PITCH_270) * 0.01f));
    struct log_PSC pkt{
        LOG_PACKET_HEADER_INIT(LOG_PSC_MSG),
        time_us         : AP_HAL::micros64(),
        pos_target_x    : pos_target.x * 0.01f,
        pos_target_Y    : pos_target.y * 0.01f,
        position_x      : position.x * 0.01f,
        position_y      : position.y * 0.01f,
        vel_target_x    : vel_target.x * 0.01f,
        vel_target_y    : vel_target.y * 0.01f,
        velocity_x      : velocity.x * 0.01f,
        velocity_y      : velocity.y * 0.01f,
        accel_target_x  : accel_target.x * 0.01f,
        accel_target_y  : accel_target.y * 0.01f,
        accel_x         : accel_x * 0.01f,
        accel_y         : accel_y * 0.01f
    };
    WriteBlock(&pkt, sizeof(pkt));
    { LOG_PSCZ_MSG, sizeof(log_PSCZ), \
      "PSCZ", "Qfffffffff", "TimeUS,TPZ,PZ,DVZ,TVZ,VZ,DAZ,TAZ,AZ,ThO", "smmnnnooo%", "F000000002" }*/
}
