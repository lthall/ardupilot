#include <AP_HAL/AP_HAL.h>
#include "AC_Circle.h"
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_Circle::var_info[] = {
    // @Param: RADIUS
    // @DisplayName: Circle Radius
    // @Description: Defines the radius of the circle the vehicle will fly when in Circle flight mode
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("RADIUS",  0,  AC_Circle, _radius, AC_CIRCLE_RADIUS_DEFAULT),

    // @Param: RATE
    // @DisplayName: Circle rate
    // @Description: Circle mode's turn rate in deg/sec.  Positive to turn clockwise, negative for counter clockwise
    // @Units: deg/s
    // @Range: -90 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RATE",    1, AC_Circle, _rate,    AC_CIRCLE_RATE_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_Circle::AC_Circle(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control),
    _yaw(0.0f),
    _angle(0.0f),
    _angle_total(0.0f),
    _angular_vel(0.0f),
    _angular_vel_max(0.0f),
    _angular_accel(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.panorama = false;
}

/// init - initialise circle controller setting center specifically
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init(const Vector3f& center)
{
    _center = center;

    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // calculate velocities
    calc_velocities(true);

    // set start angle from position
    init_start_angle(false);
}

/// init - initialise circle controller setting center using stopping point and projecting out based on the copter's heading
///     caller should set the position controller's x,y and z speeds and accelerations before calling this
void AC_Circle::init()
{
    // initialise position controller (sets target roll angle, pitch angle and I terms based on vehicle current lean angles)
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.set_desired_velocity_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // set initial position target to reasonable stopping point
    _pos_control.set_target_to_stopping_point_xy();
    _pos_control.set_target_to_stopping_point_z();

    // get stopping point
    const Vector3f& stopping_point = _pos_control.get_pos_target();

    // set circle center to circle_radius ahead of stopping point
    _center.x = stopping_point.x + _radius * _ahrs.cos_yaw();
    _center.y = stopping_point.y + _radius * _ahrs.sin_yaw();
    _center.z = stopping_point.z;

    // calculate velocities
    calc_velocities(true);

    // set starting angle from vehicle heading
    init_start_angle(true);
}

/// set_circle_rate - set circle rate in degrees per second
void AC_Circle::set_rate(float deg_per_sec)
{
    if (!is_equal(deg_per_sec, _rate.get())) {
        _rate = deg_per_sec;
        calc_velocities(false);
    }
}

/// update - update circle controller
void AC_Circle::update()
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    if (_velocity_cross < 0.5f * _velocity_max) {
        // jerk limited increase in acceleration
        float time = safe_sqrt(2.0f * _velocity_cross / _jerk_max);
        _velocity_cross = 0.5f * _jerk_max * sq(time + dt);
        _accel_cross = _jerk_max * (time + dt);
    } else if (_accel_cross > 0.0f && _velocity_cross < _velocity_max) {
        // jerk limited decrease in acceleration
        float time = safe_sqrt(2.0f * (_velocity_max - _velocity_cross) / _jerk_max);
        _velocity_cross = _velocity_max - 0.5f * _jerk_max * sq(time - dt);
        _accel_cross = _jerk_max * (time - dt);
    } else {
        // constant velocity
        _velocity_cross = _velocity_max;
        _accel_cross = 0.0f;
    }

    _angular_vel = _velocity_cross / _radius;

    // update the target angle and total angle traveled
    float angle_change = _angular_vel * dt;
    _angle += angle_change;
    _angle = wrap_PI(_angle);
    _angle_total += angle_change;

    float cos_angle = cosf(_angle);
    float sin_angle = sinf(_angle);
    float accel_rad = sq(_angular_vel) * _radius;

    // if the circle_radius is zero we are doing panorama so no need to update loiter target
    if (!is_zero(_radius)) {
        // calculate target position
        Vector3f pos;
        pos.x = _center.x + _radius * cos_angle;
        pos.y = _center.y + _radius * sin_angle;
        pos.z = _pos_control.get_alt_target();

        Vector3f vel;
        vel.x = - _velocity_cross * sin_angle;
        vel.y = _velocity_cross * cos_angle;
        vel.z = 0.0f;

        Vector3f accel;
        accel.x = - accel_rad * cos_angle - _accel_cross * sin_angle;
        accel.y = - accel_rad * sin_angle + _accel_cross * cos_angle;
        accel.z = 0.0f;

        // update position controller target
        _pos_control.set_xy_target(pos.x, pos.y);
        _pos_control.set_desired_velocity_xy(vel.x, vel.y);
        _pos_control.set_desired_accel_xy(accel.x, accel.y);

        // heading is from vehicle to center of circle
//        _yaw = get_bearing_cd(_inav.get_position(), _center);
        _yaw = ToDeg(_angular_vel)*100.0f;

        // write log - save the data.
        AP::logger().Write("LEN2", "TimeUS,vm,vc,am,ac,ar,y,yd", "Qfffffff",
                                               AP_HAL::micros64(),
                                               (double)_velocity_max,
                                               (double)_velocity_cross,
                                               (double)_accel_max,
                                               (double)_accel_cross,
                                               (double)accel_rad,
                                               (double)ToDeg(_ahrs.yaw),
                                               (double)ToDeg(wrap_PI(_angle+M_PI)));
    } else {
        // set target position to center
        Vector3f target;
        target.x = _center.x;
        target.y = _center.y;
        target.z = _pos_control.get_alt_target();

        // update position controller target
        _pos_control.set_xy_target(target.x, target.y);

        // heading is same as _angle but converted to centi-degrees
        _yaw = _angle * DEGX100;
    }

    // update position controller
    _pos_control.update_xy_controller();
}

// get_closest_point_on_circle - returns closest point on the circle
//  circle's center should already have been set
//  closest point on the circle will be placed in result
//  result's altitude (i.e. z) will be set to the circle_center's altitude
//  if vehicle is at the center of the circle, the edge directly behind vehicle will be returned
void AC_Circle::get_closest_point_on_circle(Vector3f &result)
{
    // return center if radius is zero
    if (_radius <= 0) {
        result = _center;
        return;
    }

    // get current position
    Vector3f stopping_point;
    _pos_control.get_stopping_point_xy(stopping_point);

    // calc vector from stopping point to circle center
    Vector2f vec;   // vector from circle center to current location
    vec.x = (stopping_point.x - _center.x);
    vec.y = (stopping_point.y - _center.y);
    float dist = norm(vec.x, vec.y);

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (is_zero(dist)) {
        result.x = _center.x - _radius * _ahrs.cos_yaw();
        result.y = _center.y - _radius * _ahrs.sin_yaw();
        result.z = _center.z;
        return;
    }

    // calculate closest point on edge of circle
    result.x = _center.x + vec.x / dist * _radius;
    result.y = _center.y + vec.y / dist * _radius;
    result.z = _center.z;
}

// calc_velocities - calculate angular velocity max and acceleration based on radius and rate
//      this should be called whenever the radius or rate are changed
//      initialises the yaw and current position around the circle
void AC_Circle::calc_velocities(bool init_velocity)
{
    // if we are doing a panorama set the circle_angle to the current heading

    // circle mode angular velocity
    _angular_vel_max = ToRad(_rate);
    // limit to max slew rate angular velocity
    _angular_vel_max = MIN(_angular_vel_max, _attitude_control.get_slew_yaw_rads());

    if (_radius <= 0) {
        _angular_accel = MAX(fabsf(_angular_vel_max), 0.25f * _attitude_control.get_accel_yaw_max_radss());
    }else{
        // maximum acceleration set to leave 25% for corrections
        _accel_max = 0.75 * _pos_control.get_max_accel_xy();
        // maximum velocity to achieve angular velocity
        _velocity_max = _angular_vel_max * _radius;
        // limit velocity by maximum allowable velocity
        _velocity_max = MIN(_velocity_max, _pos_control.get_max_speed_xy());
        // limit velocity by maximum allowable radial acceleration
        _velocity_max = MIN(_velocity_max, safe_sqrt(_accel_max*_radius));
        // maximum jerk based on 50% roll or pitch angular acceleration
        _jerk_max = 0.5f * MIN(_attitude_control.get_accel_roll_max_radss(), _attitude_control.get_accel_pitch_max_radss()) * GRAVITY_MSS * 100.0f; // cm/s/s
        // limit maximum acceleration by maximum radial jerk
        _accel_max = MIN(_accel_max, _jerk_max * _radius / _velocity_max);
        // limit jerk to ensure maximum acceleration is not exceeded
        _jerk_max = MIN(_jerk_max, sq(_accel_max)/_velocity_max);
        // limit jerk by maximum yaw angular acceleration
        _jerk_max = MIN(_jerk_max, 0.25f * _attitude_control.get_accel_yaw_max_radss() * _radius);
    }

    // initialise angular velocity
    if (init_velocity) {
        _angular_vel = 0.0f;
    }
    _velocity_cross = 0.0f;
    _accel_cross = 0.0f;
}

// init_start_angle - sets the starting angle around the circle and initialises the angle_total
//  if use_heading is true the vehicle's heading will be used to init the angle causing minimum yaw movement
//  if use_heading is false the vehicle's position from the center will be used to initialise the angle
void AC_Circle::init_start_angle(bool use_heading)
{
    // initialise angle total
    _angle_total = 0;

    // if the radius is zero we are doing panorama so init angle to the current heading
    if (_radius <= 0) {
        _angle = _ahrs.yaw;
        return;
    }

    // if use_heading is true
    if (use_heading) {
        _angle = wrap_PI(_ahrs.yaw-M_PI);
    } else {
        // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
        const Vector3f &curr_pos = _inav.get_position();
        if (is_equal(curr_pos.x,_center.x) && is_equal(curr_pos.y,_center.y)) {
            _angle = wrap_PI(_ahrs.yaw-M_PI);
        } else {
            // get bearing from circle center to vehicle in radians
            float bearing_rad = atan2f(curr_pos.y-_center.y,curr_pos.x-_center.x);
            _angle = wrap_PI(bearing_rad);
        }
    }
}
