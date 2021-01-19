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
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include "SplineCurve.h"

extern const AP_HAL::HAL &hal;

#define SPEED_MIN           50.0f   // minimum speed in cm/s
#define ACCEL_MIN           50.0f   // minimum acceleration in cm/s/s
#define LEASH_LENGTH_MIN    100.0f  // minimum leash length in cm
#define TANGENTIAL_ACCEL_SCALER 0.5f    // the proportion of the maximum accel that can be used for tangential acceleration (aka in the direction of travel along the track)
#define LATERAL_ACCEL_SCALER    1.0f    // the proportion of the maximum accel that can be used for lateral acceleration (aka crosstrack acceleration)

// limit the maximum speed along the track to that which will achieve a cornering (aka lateral) acceleration of LATERAL_SPEED_SCALER * acceleration limit

// constructor
SplineCurve::SplineCurve() :
    _speed_xy_cms(SPEED_MIN),
    _speed_up_cms(SPEED_MIN),
    _speed_down_cms(SPEED_MIN),
    _accel_xy_cmss(ACCEL_MIN),
    _accel_z_cmss(ACCEL_MIN)
{
}

// set maximum speed, acceleration and leash lengths
void SplineCurve::set_speed_accel(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                                   float accel_xy_cmss, float accel_z_cmss)
{
    _speed_xy_cms = MAX(speed_xy_cms, SPEED_MIN);
    _speed_up_cms = MAX(speed_up_cms, SPEED_MIN);
    _speed_down_cms = MAX(fabsf(speed_down_cms), SPEED_MIN);
    _accel_xy_cmss = MAX(accel_xy_cmss, ACCEL_MIN);
    _accel_z_cmss = MAX(accel_z_cmss, ACCEL_MIN);
}

// set origin and destination using position vectors (offset from EKF origin in cm
// origin_vel is vehicle velocity at origin (in cm/s in NEU frame)
// destination_vel is vehicle velocity at destination (in cm/s in NEU frame)
void SplineCurve::set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    _origin_vel = origin_vel;
    _destination_vel = destination_vel;
    _reached_destination = false;

    // reset time
    // Note: _time could include left-over from previous waypoint
    _time = 0.0f;

    // code below ensures we don't get too much overshoot when the next segment is short
    const float vel_len = _origin_vel.length() + _destination_vel.length();
    const float pos_len = (_destination - _origin).length() * 4.0f;
    if (vel_len > pos_len) {
        // if total start+stop velocity is more than twice position difference
        // use a scaled down start and stop velocity
        const float vel_scaling = pos_len / vel_len;
        // update spline calculator
        update_solution(_origin, _destination, _origin_vel * vel_scaling, _destination_vel * vel_scaling);
    } else {
        // update spline calculator
        update_solution(_origin, _destination, _origin_vel, _destination_vel);
    }
    Vector3f target_pos;
    Vector3f spline_vel_unit;
    float spline_dt;
    float accel_max;
    calc_dt_speed_max(0.0f, 0.0f, spline_dt, target_pos, spline_vel_unit, _origin_speed_max, accel_max);
    if (_destination_vel.is_zero()) {
        _destination_speed_max = 0.0f;
    } else {
        calc_dt_speed_max(1.0f, 0.0f, spline_dt, target_pos, spline_vel_unit, _destination_speed_max, accel_max);
    }
}

// move target location along track from origin to destination
// target_pos is updated with the target position in cm from EKF origin in NEU frame
// target_vel is updated with the target velocity in cm/s in NEU frame
void SplineCurve::advance_target_along_track(float dt, Vector3f &target_pos, Vector3f &target_vel)
{
    // calculate target position and velocity using spline calculator
    float speed_cms = target_vel.length();
    const float distance_delta = speed_cms * dt;
    float spline_dt;
    Vector3f spline_vel_unit;
    float speed_max;
    float accel_max;

    calc_dt_speed_max(_time, distance_delta, spline_dt, target_pos, spline_vel_unit, speed_max, accel_max);
    speed_cms = constrain_float(speed_max, speed_cms - accel_max * dt, speed_cms + accel_max * dt);
    target_vel = spline_vel_unit * speed_cms;

    _time += spline_dt;

    // we will reach the destination in the next step so set reached_destination flag
    if (_time >= 1.0f) {
        _time = 1.0f;
        _reached_destination = true;
    }
}

// calculate the spline delta time for a given delta distance
// returns the spline position and velocity and maximum speed and acceleration the vehicle can travel without exceeding acceleration limits
void SplineCurve::calc_dt_speed_max(float time, float distance_delta, float &spline_dt, Vector3f &target_pos, Vector3f &spline_vel_unit, float &speed_max, float &accel_max)
{
    // initialise outputs
    spline_dt = 0.0f;
    spline_vel_unit.zero();
    speed_max = 0.0f;
    accel_max = 0.0f;

    // calculate target position and velocity using spline calculator
    Vector3f spline_vel;
    Vector3f spline_accel;
    Vector3f spline_jerk;
    Vector3f spline_accel_norm;
    float spline_vel_length;
    float spline_accel_norm_length;

    calc_target_pos_vel(time, target_pos, spline_vel, spline_accel, spline_jerk);

    // vel, accel and jerk should never all be zero
    if (spline_vel.is_zero() && spline_accel.is_zero() && spline_jerk.is_zero()) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        _reached_destination = true;
        return;
    }

    // aircraft velocity and acceleration along the spline will be defined based on the aircraft kinematic limits
    // aircraft velocity along the spline should be reduced to ensure normal accelerations do not exceed kinematic limits
    spline_vel_length = spline_vel.length();
    if (is_zero(spline_vel_length)) {
        // if spline velocity is zero then direction must be defined by acceleration or jerk
        if (is_zero(spline_accel.length_squared())) {
            // if acceleration is zero then direction must be defined by jerk
            spline_vel_unit = spline_jerk.normalized();
            spline_dt = powf(6.0f * distance_delta / spline_jerk.length(), 1.0f/3.0f);
        } else {
            // all spline acceleration is in the direction of travel
            spline_vel_unit = spline_accel.normalized();
            spline_dt = safe_sqrt(2.0f * distance_delta / spline_accel.length());
        }
    } else {
        spline_vel_unit = spline_vel.normalized();
        spline_dt = distance_delta / spline_vel_length;
    }

    // calculate acceleration normal to the direction of travel
    const float spline_accel_tangent_length = spline_accel.dot(spline_vel_unit);
    spline_accel_norm = spline_accel - (spline_vel_unit * spline_accel_tangent_length);
    spline_accel_norm_length = spline_accel_norm.length();

    // limit the maximum speed along the track to that which will achieve a cornering (aka lateral) acceleration of LATERAL_SPEED_SCALER * acceleration limit
    const float tangential_speed_max = kinematic_limit(spline_vel_unit, _speed_xy_cms, _speed_up_cms, _speed_down_cms);
    const float accel_norm_max = LATERAL_ACCEL_SCALER * kinematic_limit(spline_accel_norm, _accel_xy_cmss, _accel_z_cmss, _accel_z_cmss);

    // sanity check to avoid divide by zero
    if (is_zero(tangential_speed_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        _reached_destination = true;
        return;
    }

    if ((is_positive(accel_norm_max)) && is_positive(spline_accel_norm_length) && is_positive(spline_vel_length) &&
         ((spline_accel_norm_length/accel_norm_max) > sq(spline_vel_length/tangential_speed_max))) {
        speed_max = spline_vel_length / safe_sqrt(spline_accel_norm_length/accel_norm_max);
    } else {
        speed_max = tangential_speed_max;
    }

    // calculate accel max and sanity check
    accel_max = TANGENTIAL_ACCEL_SCALER * kinematic_limit(spline_vel_unit, _accel_xy_cmss, _accel_z_cmss, _accel_z_cmss);
    if (is_zero(accel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        _reached_destination = true;
        return;
    }
    const float dist = (_destination - target_pos).length();
    speed_max = MIN(speed_max, safe_sqrt(2.0f * accel_max * (dist + sq(_destination_speed_max) / (2.0f*accel_max))));
}

// recalculate hermite_solution grid
//     relies on _origin_vel, _destination_vel and _origin and _destination
void SplineCurve::update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel)
{
    _hermite_solution[0] = origin;
    _hermite_solution[1] = origin_vel;
    _hermite_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
}

// calculate target position and velocity from given spline time
// time is a value from 0 to 1
// position is updated with target position as an offset from EKF origin in NEU frame
// velocity is updated with the unscaled velocity
// relies on set_origin_and_destination having been called to update_solution
void SplineCurve::calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity, Vector3f &acceleration, Vector3f &jerk)
{
    const float time_sq = sq(time);
    const float time_cubed = time_sq * time;

    position = _hermite_solution[0] + \
               _hermite_solution[1] * time + \
               _hermite_solution[2] * time_sq + \
               _hermite_solution[3] * time_cubed;

    velocity = _hermite_solution[1] + \
               _hermite_solution[2] * 2.0f * time + \
               _hermite_solution[3] * 3.0f * time_sq;

    acceleration = _hermite_solution[2] * 2.0f + \
               _hermite_solution[3] * 6.0f * time;

    jerk = _hermite_solution[3] * 6.0f;
}

