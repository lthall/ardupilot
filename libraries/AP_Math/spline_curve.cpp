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
#include "spline_curve.h"

extern const AP_HAL::HAL &hal;

#define SPEED_MIN           50.0f   // minimum speed in cm/s
#define ACCEL_MIN           50.0f   // minimum acceleration in cm/s/s
#define LEASH_LENGTH_MIN    100.0f  // minimum leash length in cm

// constructor
spline_curve::spline_curve() :
    _speed_xy_cms(SPEED_MIN),
    _speed_up_cms(SPEED_MIN),
    _speed_down_cms(SPEED_MIN),
    _accel_xy_cmss(ACCEL_MIN),
    _accel_z_cmss(ACCEL_MIN),
    _leash_xy_cm(LEASH_LENGTH_MIN),
    _leash_up_cm(LEASH_LENGTH_MIN),
    _leash_down_cm(LEASH_LENGTH_MIN)
{
}

// set maximum speed, acceleration and leash lengths
void spline_curve::set_speed_accel_leash(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                                         float accel_xy_cms, float accel_z_cms,
                                         float leash_xy_cm, float leash_up_cm, float leash_down_cm)
{
    _speed_xy_cms = MAX(speed_xy_cms, SPEED_MIN);
    _speed_up_cms = MAX(speed_up_cms, SPEED_MIN);
    _speed_down_cms = MAX(fabsf(speed_down_cms), SPEED_MIN);
    _accel_xy_cmss = MAX(accel_xy_cms, ACCEL_MIN);
    _accel_z_cmss = MAX(accel_z_cms, ACCEL_MIN);
    _leash_xy_cm = MAX(leash_xy_cm, LEASH_LENGTH_MIN);
    _leash_up_cm = MAX(leash_up_cm, LEASH_LENGTH_MIN);
    _leash_down_cm = MAX(leash_down_cm, LEASH_LENGTH_MIN);
}

// set origin and destination using position vectors (offset from EKF origin in cm
// origin_vel is vehicle velocity at origin (in cm/s in NEU frame)
// destination_vel is vehicle velocity at destination (in cm/s in NEU frame)
void spline_curve::set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel, float vel_target_length)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    _origin_vel = origin_vel;   // Note: _origin_vel was, "(destination - origin) * dt" instead of being zero
    _destination_vel = destination_vel;
    _vel_scalar = vel_target_length;
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
}

// recalculate hermite_solution grid
//     relies on _origin_vel, _destination_vel and _origin and _destination
void spline_curve::update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel)
{
    _hermite_solution[0] = origin;
    _hermite_solution[1] = origin_vel;
    _hermite_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
}

// move target location along track from origin to destination
// target_pos is updated with the target position in cm from EKF origin in NEU frame
// target_vel is updated with the target velocity in cm/s in NEU frame
void spline_curve::advance_target_along_track(const Vector3f &curr_pos, float dt, Vector3f &target_pos, Vector3f &target_vel)
{
    // return immediately if already reached destination
    if (_reached_destination) {
        target_pos = curr_pos;
        target_vel.zero();
        return;
    }

    // calculate target position and velocity using spline calculator
    Vector3f target_vel_unscaled;
    calc_target_pos_vel(_time, target_pos, target_vel_unscaled);

    // if target velocity is zero the origin and destination must be the same
    // so flag reached destination (and protect against divide by zero)
    const float target_vel_unscaled_len = target_vel_unscaled.length();
    if (is_zero(target_vel_unscaled_len)) {
        _reached_destination = true;
        target_pos = curr_pos;
        target_vel.zero();
        return;
    }

    calc_leash_length(target_vel_unscaled / target_vel_unscaled_len);

    // calculate the horizontal and vertical error
    const Vector3f track_error = curr_pos - target_pos;
    const float track_error_xy = norm(track_error.x, track_error.y);
    const float track_error_z = fabsf(track_error.z);

    // get position control leash lengths
    float leash_xy = _leash_xy_cm;
    float leash_z;
    if (track_error.z >= 0) {
        leash_z = _leash_up_cm;
    } else {
        leash_z = _leash_down_cm;
    }

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    float track_leash_slack = MIN(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_xy)/leash_xy);
    if (track_leash_slack < 0.0f) {
        track_leash_slack = 0.0f;
    }

    // update velocity
    const float dist_to_wp = (_destination - target_pos).length();
    float vel_limit = _speed_xy_cms;
    if (!is_zero(dt)) {
        vel_limit = MIN(vel_limit, track_leash_slack / dt);
    }

    // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
    if (_destination_vel.is_zero() && (dist_to_wp < _slow_down_dist)) {
        _vel_scalar = safe_sqrt(dist_to_wp * 2.0f * _accel_xy_cmss);
    } else if (_vel_scalar < vel_limit) {
        // increase velocity using acceleration
        _vel_scalar += _accel_xy_cmss * dt;
    }

    // constrain target velocity
    _vel_scalar = constrain_float(_vel_scalar, 0.0f, vel_limit);

    // update target velocity
    target_vel = target_vel_unscaled * _vel_scalar;

    // scale the time by the velocity we've calculated vs the velocity that came out of the spline calculator
    const float time_scale = _vel_scalar / target_vel_unscaled_len;

    // advance time to next step
    _time += time_scale * dt;

    // we will reach the destination in the next step so set reached_destination flag
    // To-Do: is this one step too early?
    if (_time >= 1.0f) {
        _reached_destination = true;
    }
}

// calculates horizontal and vertical leash lengths for waypoint controller
// pos_delta_unit should be a unit vector (in NEU frame) pointing from origin to destination
// or pointing in the direction of the latest target velocity
void spline_curve::calc_leash_length(const Vector3f &pos_delta_unit)
{
    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = norm(pos_delta_unit.x, pos_delta_unit.y);
    float pos_delta_unit_z = fabsf(pos_delta_unit.z);

    float speed_z;
    float leash_z;
    if (pos_delta_unit.z >= 0.0f) {
        speed_z = _speed_up_cms;
        leash_z = _leash_up_cm;
    } else {
        speed_z = _speed_down_cms;
        leash_z = _leash_down_cm;
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    float track_accel, track_speed;
    if (is_zero(pos_delta_unit_z) && is_zero(pos_delta_unit_xy)){
        track_accel = 0;
        track_speed = 0;
        _track_leash_length = LEASH_LENGTH_MIN;
    } else if (is_zero(pos_delta_unit.z)){
        track_accel = _accel_xy_cmss/pos_delta_unit_xy;
        track_speed = _speed_xy_cms / pos_delta_unit_xy;
        _track_leash_length = _leash_xy_cm/pos_delta_unit_xy;
    } else if (is_zero(pos_delta_unit_xy)){
        track_accel = _accel_z_cmss/pos_delta_unit_z;
        track_speed = speed_z/pos_delta_unit_z;
        _track_leash_length = leash_z/pos_delta_unit_z;
    } else {
        track_accel = MIN(_accel_z_cmss / pos_delta_unit_z, _accel_xy_cmss / pos_delta_unit_xy);
        track_speed = MIN(speed_z / pos_delta_unit_z, _speed_xy_cms / pos_delta_unit_xy);
        _track_leash_length = MIN(leash_z / pos_delta_unit_z, _leash_xy_cm / pos_delta_unit_xy);
    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(track_speed, track_accel);
}

// calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void spline_curve::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
    // protect against divide by zero
    if (accel_cmss <= 0.0f) {
        _slow_down_dist = 0.0f;
        return;
    }
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

// calculate target position and velocity from given spline time
// time is a value from 0 to 1
// position is updated with target position as an offset from EKF origin in NEU frame
// velocity is updated with the unscaled velocity
// relies on set_origin_and_destination having been called to update_solution
void spline_curve::calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity)
{
    float time_sq = sq(time);
    float time_cubed = time_sq * time;

    position = _hermite_solution[0] + \
               _hermite_solution[1] * time + \
               _hermite_solution[2] * time_sq + \
               _hermite_solution[3] * time_cubed;

    velocity = _hermite_solution[1] + \
               _hermite_solution[2] * 2.0f * time + \
               _hermite_solution[3] * 3.0f * time_sq;
}

