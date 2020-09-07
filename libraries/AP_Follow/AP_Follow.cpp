/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Follow.h"
#include <ctype.h>
#include <stdio.h>

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define AP_FOLLOW_TIMEOUT_MS    3000    // position estimate timeout after 1 second
#define AP_FOLLOW_SYSID_TIMEOUT_MS 10000 // forget sysid we are following if we haave not heard from them in 10 seconds

#define AP_FOLLOW_OFFSET_TYPE_NED       0   // offsets are in north-east-down frame
#define AP_FOLLOW_OFFSET_TYPE_RELATIVE  1   // offsets are relative to lead vehicle's heading

#define AP_FOLLOW_ALTITUDE_TYPE_RELATIVE  1 // relative altitude is used by default   

#define AP_FOLLOW_POS_P_DEFAULT 0.1f    // position error gain default

AP_Follow *AP_Follow::_singleton;

// table of user settable parameters
const AP_Param::GroupInfo AP_Follow::var_info[] = {

    // @Param: _ENABLE
    // @DisplayName: Follow enable/disable
    // @Description: Enabled/disable following a target
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_Follow, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // 2 is reserved for TYPE parameter

    // @Param: _SYSID
    // @DisplayName: Follow target's mavlink system id
    // @Description: Follow target's mavlink system id
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_SYSID", 3, AP_Follow, _sysid, 0),

    // 4 is reserved for MARGIN parameter

    // @Param: _DIST_MAX
    // @DisplayName: Follow distance maximum
    // @Description: Follow distance maximum.  targets further than this will be ignored
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    AP_GROUPINFO("_DIST_MAX", 5, AP_Follow, _dist_max, 100),

    // @Param: _OFS_TYPE
    // @DisplayName: Follow offset type
    // @Description: Follow offset type
    // @Values: 0:North-East-Down, 1:Relative to lead vehicle heading
    // @User: Standard
    AP_GROUPINFO("_OFS_TYPE", 6, AP_Follow, _offset_type, AP_FOLLOW_OFFSET_TYPE_NED),

    // @Param: _OFS_X
    // @DisplayName: Follow offsets in meters north/forward
    // @Description: Follow offsets in meters north/forward.  If positive, this vehicle fly ahead or north of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Y
    // @DisplayName: Follow offsets in meters east/right
    // @Description: Follow offsets in meters east/right.  If positive, this vehicle will fly to the right or east of lead vehicle.  Depends on FOLL_OFS_TYPE
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard

    // @Param: _OFS_Z
    // @DisplayName: Follow offsets in meters down
    // @Description: Follow offsets in meters down.  If positive, this vehicle will fly below the lead vehicle
    // @Range: -100 100
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OFS", 7, AP_Follow, _offset, 0),

#if !(APM_BUILD_TYPE(APM_BUILD_APMrover2))
    // @Param: _YAW_BEHAVE
    // @DisplayName: Follow yaw behaviour
    // @Description: Follow yaw behaviour
    // @Values: 0:None,1:Face Lead Vehicle,2:Same as Lead vehicle,3:Direction of Flight
    // @User: Standard
    AP_GROUPINFO("_YAW_BEHAVE", 8, AP_Follow, _yaw_behave, 1),
#endif

    // @Param: _POS_P
    // @DisplayName: Follow position error P gain
    // @Description: Follow position error P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 0.01 1.00
    // @Increment: 0.01
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos, "_POS_", 9, AP_Follow, AC_P),

#if !(APM_BUILD_TYPE(APM_BUILD_APMrover2)) 
    // @Param: _ALT_TYPE
    // @DisplayName: Follow altitude type
    // @Description: Follow altitude type
    // @Values: 0:absolute, 1: relative
    // @User: Standard
    AP_GROUPINFO("_ALT_TYPE", 10, AP_Follow, _alt_type, AP_FOLLOW_ALTITUDE_TYPE_RELATIVE),
#endif

    // @Param: _PATH_TC
    // @DisplayName: Follow time constant
    // @Description: Time constant used to generate the kinematically consistent path
    // @Units: s
    // @Range: 0.1 10
    // @User: Standard
    AP_GROUPINFO("_PATH_TC", 11, AP_Follow, _path_tc, 1.0f),

    AP_GROUPEND
};

/* 
   The constructor also initialises the proximity sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the proximity sensor
*/
AP_Follow::AP_Follow() :
        _p_pos(AP_FOLLOW_POS_P_DEFAULT)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// restore offsets to zero if necessary, should be called when vehicle exits follow mode
void AP_Follow::clear_offsets_if_required()
{
    if (_offsets_were_zero) {
        _offset = Vector3f();
    }
    _offsets_were_zero = false;
}

// return true if we have a target
bool AP_Follow::have_target(void) const
{
    if (!_enabled) {
        return false;
    }

    // check for timeout
    if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

// get target's estimated location
bool AP_Follow::get_target_location_and_velocity(Location &loc, Vector3f &vel_ned, bool force_absolute_alt) const
{
    // exit immediately if not enabled
    if (!have_target()) {
        return false;
    }

    // calculate time since last actual position update
    const float dt = (AP_HAL::millis() - _last_location_update_ms) * 0.001f;

    // get velocity estimate
    if (!get_velocity_ned(vel_ned, dt)) {
        return false;
    }

    // project the vehicle position
    Location last_loc = _target_location;

    if (force_absolute_alt) {
        // allow caller to request absolute altitude
        last_loc.alt = _target_alt_cm;
        last_loc.relative_alt = false;
    }

    last_loc.offset(vel_ned.x * dt, vel_ned.y * dt);
    last_loc.alt -= vel_ned.z * 100.0f * dt; // convert m/s to cm/s, multiply by dt.  minus because NED

    // return latest position estimate
    loc = last_loc;
    return true;
}

// update the current position and velocity based on the last known location and velocity
bool AP_Follow::update_target_pos_and_vel()
{
    const float dt = (AP_HAL::millis() - _last_location_update_ms) * 0.001f;
    if (dt*1000 > AP_FOLLOW_TIMEOUT_MS) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // get our location
    Location current_loc;
    Vector3f current_location_ned;
    if (!AP::ahrs().get_position(current_loc) || !current_loc.get_vector_from_origin_NED(current_location_ned)) {
        clear_dist_and_bearing_to_target();
         return false;
    }

    // get target location and velocity
    Vector3f pos_ned;
    Vector3f vel_ned = _target_velocity_ned;
    Vector3f acc_ned = _target_accel_ned;
    if (!_target_location.get_vector_from_origin_NED(pos_ned)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    // calculate difference and check for minimum distance
    const Vector3f dist_vec = current_loc.get_distance_NED(_target_location);
    if (is_positive(_dist_max.get()) && (Vector2f(dist_vec.x, dist_vec.y).length() > _dist_max)) {
        // Too far from target
        clear_dist_and_bearing_to_target();
        return false;
    }

    // initialise offsets from distance vector if required
    init_offsets_if_required(dist_vec);

    // get offsets
    Vector3f offsets;
    if (!get_offsets_ned(offsets)) {
        clear_dist_and_bearing_to_target();
        return false;
    }

    pos_ned += offsets;

    // calculate time since last actual position update
    update_pos_vel_accel_xy(pos_ned, vel_ned, acc_ned, dt);
    update_pos_vel_accel_z(pos_ned, vel_ned, acc_ned, dt);

    // calculate time since last path update
    const float path_dt = (AP_HAL::micros() - _last_path_update_us) * 0.000001f;
    _last_path_update_us = AP_HAL::micros();

    update_pos_vel_accel_xy(_path_location_ned, _path_velocity_ned, _path_accel_ned, path_dt);
    update_pos_vel_accel_z(_path_location_ned, _path_velocity_ned, _path_accel_ned, path_dt);

    if ((_last_path_update_us == 0) || (path_dt > MIN(_path_tc, AP_FOLLOW_TIMEOUT_MS * 0.001f))) {
        _path_location_ned = pos_ned;
        _path_velocity_ned = vel_ned;
        _path_accel_ned.zero();
    } else {
        shape_pos_vel_xy(pos_ned, vel_ned, _path_location_ned, _path_velocity_ned, _path_accel_ned, 0.0f, 0.0f, 0.0f, _path_tc, path_dt);
        shape_pos_vel_z(pos_ned, vel_ned, _path_location_ned, _path_velocity_ned, _path_accel_ned, 0.0f, 0.0f, 0.0f, _path_tc, path_dt);
    }

    // record distance and heading for reporting purposes
    _dist_offs_ned = _path_location_ned - current_location_ned;
    if (is_zero(_dist_offs_ned.x) && is_zero(_dist_offs_ned.y)) {
        clear_dist_and_bearing_to_target();
    } else {
        _dist_to_target = safe_sqrt(sq(_dist_offs_ned.x) + sq(_dist_offs_ned.y));
        _bearing_to_target = degrees(atan2f(_dist_offs_ned.y, _dist_offs_ned.x));
    }

    return true;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
void AP_Follow::get_target_vel_ned(Vector3f &vel_ned)
{
    vel_ned = _path_velocity_ned;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
void AP_Follow::get_target_pos_and_vel_ned(Vector3f &pos_with_ofs, Vector3f &vel_ned)
{
    pos_with_ofs = _path_location_ned;
    vel_ned = _path_velocity_ned;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
void AP_Follow::get_target_dist_and_vel_ned(Vector3f &dist_with_offs, Vector3f &vel_ned)
{
    dist_with_offs = _dist_offs_ned;
    vel_ned = _path_velocity_ned;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
void AP_Follow::get_target_dist_ned(Vector3f &dist_with_offs)
{
    dist_with_offs = _dist_offs_ned;
}

// get distance vector to target (in meters) and target's velocity all in NED frame
bool AP_Follow::get_target_dist_target_frame(Vector3f &dist_with_offs)
{
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc) || !have_target()) {
        return false;
    }
    const Vector3f dist_vec = _target_location.get_distance_NED(current_loc);
    Vector2f ofs_body(dist_vec.x, dist_vec.y);
    ofs_body.rotate(-radians(_target_heading));
    dist_with_offs = Vector3f(ofs_body.x, ofs_body.y, dist_vec.z);
    return true;
}

// get target's heading in degrees (0 = north, 90 = east)
bool AP_Follow::get_target_heading_deg(float &heading) const
{
    // exit immediately if not enabled
    if (!have_target()) {
        return false;
    }

    // return latest heading estimate
    heading = _target_heading;
    return true;
}

// handle mavlink DISTANCE_SENSOR messages
bool AP_Follow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!_enabled) {
        return false;
    }

    // skip our own messages
    if (msg.sysid == mavlink_system.sysid) {
        return false;
    }

    // skip message if not from our target
    if (_sysid != 0 && msg.sysid != _sysid) {
        if (_automatic_sysid) {
            // maybe timeout who we were following...
            if ((_last_location_update_ms == 0) || (AP_HAL::millis() - _last_location_update_ms > AP_FOLLOW_SYSID_TIMEOUT_MS)) {
                _sysid.set(0);
            }
        }
        return false;
    }

    // decode global-position-int message
    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        // get estimated location and velocity (for logging)
        Location loc_estimate{};
        Vector3f vel_estimate;
        UNUSED_RESULT(get_target_location_and_velocity(loc_estimate, vel_estimate));

        // decode message
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);

        // ignore message if lat and lon are (exactly) zero
        if ((packet.lat == 0 && packet.lon == 0)) {
            return false;
        }

        _target_location.lat = packet.lat;
        _target_location.lng = packet.lon;

        // remember absolute alt
        _target_alt_cm = packet.alt / 10;

        // select altitude source based on FOLL_ALT_TYPE param
        if (_alt_type == AP_FOLLOW_ALTITUDE_TYPE_RELATIVE) {
            // relative altitude
            _target_location.alt = packet.relative_alt / 10;        // convert millimeters to cm
            _target_location.relative_alt = true;                // set relative_alt flag
        } else {
            // absolute altitude
            _target_location.alt = packet.alt / 10;                 // convert millimeters to cm
            _target_location.relative_alt = false;                // reset relative_alt flag
        }

        _target_velocity_ned.x = packet.vx * 0.01f; // velocity north
        _target_velocity_ned.y = packet.vy * 0.01f; // velocity east
        _target_velocity_ned.z = packet.vz * 0.01f; // velocity down

        // get a local timestamp with correction for transport jitter
        _last_location_update_ms = _jitter.correct_offboard_timestamp_msec(packet.time_boot_ms, AP_HAL::millis());
        if (packet.hdg <= 36000) {                  // heading (UINT16_MAX if unknown)
            _target_heading = packet.hdg * 0.01f;   // convert centi-degrees to degrees
            _last_heading_update_ms = _last_location_update_ms;
        }
        // initialise _sysid if zero to sender's id
        if (_sysid == 0) {
            _sysid.set(msg.sysid);
            _automatic_sysid = true;
        }

        // log lead's estimated vs reported position
        AP::logger().Write("FOLL",
                           "TimeUS,Lat,Lon,Alt,VelN,VelE,VelD,LatE,LonE,AltE,Hdg",  // labels
                           "sDUmnnnDUmd",    // units
                           "F--B000--B0",    // mults
                           "QLLifffLLif",    // fmt
                           AP_HAL::micros64(),
                           _target_location.lat,
                           _target_location.lng,
                           _target_location.alt,
                           _target_velocity_ned.x,
                           _target_velocity_ned.y,
                           _target_velocity_ned.z,
                           loc_estimate.lat,
                           loc_estimate.lng,
                           loc_estimate.alt,
                           _target_heading
            );
        return true;
    }
    return false;
}

// get velocity estimate in m/s in NED frame using dt since last update
bool AP_Follow::get_velocity_ned(Vector3f &vel_ned, float dt) const
{
    vel_ned = _target_velocity_ned + (_target_accel_ned * dt);
    return true;
}

// initialise offsets to provided distance vector to other vehicle (in meters in NED frame) if offset is not set.
void AP_Follow::init_offsets_if_required(const Vector3f &dist_vec_ned)
{
    // return immediately if offsets have already been set
    if (!_offset.get().is_zero()) {
        return;
    }
    _offsets_were_zero = true;

    float target_heading_deg;
    if ((_offset_type == AP_FOLLOW_OFFSET_TYPE_RELATIVE) && get_target_heading_deg(target_heading_deg)) {
        // rotate offsets from north facing to vehicle's perspective
        _offset = rotate_vector(-dist_vec_ned, -target_heading_deg);
        gcs().send_text(MAV_SEVERITY_INFO, "Relative follow offset loaded");
    } else {
        // initialise offset in NED frame
        _offset = -dist_vec_ned;
        // ensure offset_type used matches frame of offsets saved
        _offset_type = AP_FOLLOW_OFFSET_TYPE_NED;
        gcs().send_text(MAV_SEVERITY_INFO, "N-E-D follow offset loaded");
    }
}

// get offsets in meters in NED frame
bool AP_Follow::get_offsets_ned(Vector3f &offset) const
{
    const Vector3f &off = _offset.get();

    // if offsets are zero or type is NED, simply return offset vector
    if (off.is_zero() || (_offset_type == AP_FOLLOW_OFFSET_TYPE_NED)) {
        offset = off;
        return true;
    }

    // offset type is relative, exit if we cannot get vehicle's heading
    float target_heading_deg;
    if (!get_target_heading_deg(target_heading_deg)) {
        return false;
    }

    // rotate offsets from vehicle's perspective to NED
    offset = rotate_vector(off, target_heading_deg);
    return true;
}

// rotate 3D vector clockwise by specified angle (in degrees)
Vector3f AP_Follow::rotate_vector(const Vector3f &vec, float angle_deg) const
{
    // rotate roll, pitch input from north facing to vehicle's perspective
    const float cos_yaw = cosf(radians(angle_deg));
    const float sin_yaw = sinf(radians(angle_deg));
    return Vector3f((vec.x * cos_yaw) - (vec.y * sin_yaw), (vec.y * cos_yaw) + (vec.x * sin_yaw), vec.z);
}

// set recorded distance and bearing to target to zero
void AP_Follow::clear_dist_and_bearing_to_target()
{
    _dist_to_target = 0.0f;
    _bearing_to_target = 0.0f;
}

// get target's estimated location and velocity (in NED), with offsets added, and absolute alt
bool AP_Follow::get_target_location_and_velocity_ofs_abs(Location &loc, Vector3f &vel_ned) const
{
    Vector3f ofs;
    if (!get_offsets_ned(ofs) ||
        !get_target_location_and_velocity(loc, vel_ned, true)) {
        return false;
    }
    // apply offsets
    loc.offset(ofs.x, ofs.y);
    loc.alt -= ofs.z*100;
    return true;
}

namespace AP {

AP_Follow &follow()
{
    return *AP_Follow::get_singleton();
}

}
