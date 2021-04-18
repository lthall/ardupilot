/*
 * control.cpp
 * Copyright (C) Leonard Hall 2020
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  this module provides common controller functions
 */
#include "AP_Math.h"
#include "vector2.h"
#include "vector3.h"
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>

// control default definitions
#define CONTROL_TIME_CONSTANT_RATIO 4.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation

// update_pos_vel_accel - single axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel(float& pos, float& vel, float accel, float dt,
    bool limit_neg, bool limit_pos,
    float pos_error, float vel_error)
{
    // move position and velocity forward by dt if it does not increase error when limited.
    float delta_pos = vel * dt + accel * 0.5f * sq(dt);
    if (!(limit_neg && is_negative(delta_pos) && is_negative(pos_error)) &&
        !(limit_pos && is_positive(delta_pos) && is_positive(pos_error))  ){
        pos += delta_pos;
    }

    float delta_vel = accel * dt;
    if (!(limit_neg && is_negative(delta_vel) && is_negative(vel_error)) &&
        !(limit_pos && is_positive(delta_vel) && is_positive(vel_error))  ){
        vel += delta_vel;
    }
}

// update_pos_vel_accel_z - single axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel_z(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt,
    bool limit_neg, bool limit_pos,
    float pos_error, float vel_error)
{
    update_pos_vel_accel(pos.z, vel.z, accel.z, dt,
        limit_neg, limit_pos,
        pos_error, vel_error);
}

// update_pos_vel_accel_xy - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
void update_pos_vel_accel_xy(Vector2f& pos, Vector2f& vel, Vector2f& accel, float dt,
    bool limit, Vector2f pos_error, Vector2f vel_error)
{
    // move position and velocity forward by dt.
    Vector2f delta_pos = vel * dt + accel * 0.5f * sq(dt);
    Vector2f delta_vel = accel * dt;

    if(limit && !is_zero(pos_error.length_squared())) {
        // remove component of delta_pos that would increase the position error
        pos_error.normalize();
        delta_pos -= pos_error * (delta_pos*pos_error);
    }

    if(limit && !is_zero(vel_error.length_squared())) {
        // remove component of delta_vel that would increase the velocity error
        vel_error.normalize();
        delta_vel -= delta_vel * (delta_vel*vel_error);
    }

    pos += delta_pos;
    vel += delta_vel;
}

// update_pos_vel_accel_xy - dual axis projection of position and velocity, pos and vel, forwards in time based on a time step of dt and acceleration of accel.
// This function only updates the x and y axis leaving the z axis unchanged.
void update_pos_vel_accel_xy(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt,
    bool limit, const Vector2f& pos_error, const Vector2f& vel_error)
{
    Vector2f pos_2f = Vector2f(pos.x, pos.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);
    update_pos_vel_accel_xy(pos_2f, vel_2f, accel_2f, dt,
        limit, pos_error, vel_error);
    pos.x = pos_2f.x;
    pos.y = pos_2f.y;
    vel.x = vel_2f.x;
    vel.y = vel_2f.y;
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

/* shape_accel calculates a jerk limited path from the current acceleration to an input acceleration.
 The function takes the current acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     acceleration limits - accel_min, accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
*/
void shape_accel(float accel_input, float& accel,
    float accel_min, float accel_max,
    float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = 1.0 / tc;

    // limit acceleration to accel_max
    if (is_negative(accel_min) && is_positive(accel_max)){
        accel = constrain_float(accel, accel_min, accel_max);
    }

    float jerk_max = 0.0;
    if (is_negative(accel_min) && is_positive(accel_max)){
        jerk_max = MAX(-accel_min, accel_max) * KPa;
    }

    // jerk limit acceleration change
    float accel_delta = accel_input - accel;
    if (is_positive(jerk_max)) {
        accel_delta = constrain_float(accel_delta, -jerk_max * dt, jerk_max * dt);
    }
    accel += accel_delta;
}

/* shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by:
     vel_max : maximum velocity
     accel_max : maximum acceleration
     tc : time constant
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
 The accel_max limit can be removed by setting it to zero.
*/
void shape_accel_xy(const Vector2f& accel_input, Vector2f& accel,
    float accel_max, float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    float KPa = 1.0 / tc;
    float jerk_max = accel_max * KPa;

    // jerk limit acceleration change
    Vector2f accel_delta = accel_input - accel;
    if (is_positive(jerk_max)) {
        accel_delta.limit_length(jerk_max * dt);
    }
    accel = accel + accel_delta;

    // limit acceleration to accel_max
    if (is_negative(accel_max)) {
        // we may want to allow this for some applications but call error for now.
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    } else if (is_positive(accel_max)) {
        accel.limit_length(accel_max);
    }
}


/* shape_vel and shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 The accel_max limit can be removed by setting it to zero.
*/
void shape_vel_accel(float vel_input, float accel_input,
    float vel, float& accel,
    float vel_min, float vel_max,
    float accel_min, float accel_max,
    float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPa = 1.0 / tc;

    // limit velocity to vel_max
    if (is_negative(vel_min) && is_positive(vel_max)){
        vel_input = constrain_float(vel_input, vel_min, vel_max);
    }

    // velocity error to be corrected
    float vel_error = vel_input - vel;

    // acceleration to correct velocity
    float accel_target = vel_error * KPa;

    // velocity correction with input velocity
    accel_target += accel_input;

    shape_accel(accel_target, accel, accel_min, accel_max, tc, dt);
}

void shape_vel_accel_z(const Vector3f& vel_input, const Vector3f& accel_input,
    const Vector3f& vel, Vector3f& accel,
    float vel_min, float vel_max,
    float accel_min, float accel_max,
    float tc, float dt)
{
    shape_vel_accel(vel_input.z, accel_input.z,
        vel.z, accel.z,
        vel_min, vel_max,
        accel_min, accel_max,
        tc, dt);
}

/* shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by:
     vel_max : maximum velocity
     accel_max : maximum acceleration
     tc : time constant
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
 The accel_max limit can be removed by setting it to zero.
*/
void shape_vel_accel_xy(Vector2f vel_input, const Vector2f& accel_input,
    const Vector2f& vel, Vector2f& accel, float vel_max, float accel_max, float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // limit velocity to vel_max
    if (is_negative(vel_max)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
    } else if (is_positive(vel_max)) {
        vel_input.limit_length(vel_max);
    }

    // Calculate time constants and limits to ensure stable operation
    float KPa = 1.0 / tc;

    // velocity error to be corrected
    Vector2f vel_error = vel_input - vel;

    // acceleration to correct velocity
    Vector2f accel_target = vel_error * KPa;
    accel_target += accel_input;

    shape_accel_xy(accel_target, accel, accel_max, tc, dt);
}

void shape_vel_accel_xy(const Vector3f& vel_input, const Vector3f& accel_input,
    const Vector3f& vel, Vector3f& accel, float vel_max, float accel_max, float tc, float dt)
{
    Vector2f vel_input_2f = Vector2f(vel_input.x, vel_input.y);
    Vector2f accel_input_2f = Vector2f(accel_input.x, accel_input.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);

    shape_vel_accel_xy(vel_input_2f, accel_input_2f, vel_2f, accel_2f, vel_max, accel_max, tc, dt);
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

/* shape_pos_vel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel_accel(float pos_input, float vel_input, float accel_input,
    float pos, float vel, float& accel,
    float vel_correction_max, float vel_min, float vel_max,
    float accel_min, float accel_max, float tc, float dt)
{
    // sanity check tc
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = 1.0 / (CONTROL_TIME_CONSTANT_RATIO*tc);
    const float accel_tc_max = accel_max*(1-1.0f/CONTROL_TIME_CONSTANT_RATIO);

    // position error to be corrected
    float pos_error = pos_input - pos;

    // velocity to correct position
    float vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity correction to vel_correction_max
    if (is_positive(vel_correction_max)) {
        vel_target = constrain_float(vel_target, -vel_correction_max, vel_correction_max);
    }

    // velocity correction with input velocity
    vel_target += vel_input;

    shape_vel_accel(vel_target, accel_input, vel, accel, vel_min, vel_max, accel_min, accel_max, tc, dt);
}

void shape_pos_vel_accel_z(const Vector3f& pos_input, const Vector3f& vel_input, const Vector3f& accel_input,
    const Vector3f& pos, const Vector3f& vel, Vector3f& accel,
    float vel_correction_max, float vel_min, float vel_max,
    float accel_min, float accel_max, float tc, float dt)
{
    shape_pos_vel_accel(pos_input.z, vel_input.z, accel_input.z,
        pos.z, vel.z, accel.z,
        vel_correction_max, vel_min, vel_max,
        accel_min, accel_max,
        tc, dt);
}

/* shape_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by:
     vel_max : maximum velocity
     accel_max : maximum acceleration
     tc : time constant
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel_accel_xy(const Vector2f& pos_input, const Vector2f& vel_input, const Vector2f& accel_input,
    const Vector2f& pos, const Vector2f& vel, Vector2f& accel,
    float vel_correction_max, float vel_max, float accel_max, float tc, float dt)
{
    if (!is_positive(tc)) {
        return;
    }

    // Calculate time constants and limits to ensure stable operation
    const float KPv = 1.0f / (CONTROL_TIME_CONSTANT_RATIO*tc);
    const float accel_tc_max = accel_max*(1.0f - 1.0f/CONTROL_TIME_CONSTANT_RATIO);

    // position error to be corrected
    Vector2f pos_error = pos_input - pos;

    // velocity to correct position
    Vector2f vel_target = sqrt_controller(pos_error, KPv, accel_tc_max, dt);

    // limit velocity correction to vel_correction_max
    if (is_positive(vel_correction_max)) {
        vel_target.limit_length(vel_correction_max);
    }

    // velocity correction with input velocity
    vel_target = vel_target + vel_input;

    shape_vel_accel_xy(vel_target, accel_input, vel, accel, vel_max, accel_max, tc, dt);
}

/* shape_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by:
     vel_max : maximum velocity
     accel_max : maximum acceleration
     tc : time constant
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 This function operates only on the x and y axis of the Vector2f or Vector3f inputs.
 The vel_max, vel_correction_max, and accel_max limits can be removed by setting the desired limit to zero.
*/
void shape_pos_vel_accel_xy(const Vector3f& pos_input, const Vector3f& vel_input, const Vector3f& accel_input,
    const Vector3f& pos, const Vector3f& vel, Vector3f& accel,
    float vel_max, float vel_correction_max, float accel_max, float tc, float dt)
{
    Vector2f pos_input_2f = Vector2f(pos_input.x, pos_input.y);
    Vector2f vel_input_2f = Vector2f(vel_input.x, vel_input.y);
    Vector2f accel_input_2f = Vector2f(accel_input.x, accel_input.y);
    Vector2f pos_2f = Vector2f(pos.x, pos.y);
    Vector2f vel_2f = Vector2f(vel.x, vel.y);
    Vector2f accel_2f = Vector2f(accel.x, accel.y);

    shape_pos_vel_accel_xy(pos_input_2f, vel_input_2f, accel_input_2f,
        pos_2f, vel_2f, accel_2f, vel_max, vel_correction_max, accel_max, tc, dt);
    accel.x = accel_2f.x;
    accel.y = accel_2f.y;
}

// proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
    float correction_rate;
    if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) {
        // second order limit is zero or negative.
        correction_rate = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error));
        } else if (is_negative(error)) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        const float linear_dist = second_ord_lim / sq(p);
        if (error > linear_dist) {
            correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
        } else {
            correction_rate = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction_rate;
    }
}

// proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt)
{
    const float error_length = error.length();
    if (!is_positive(error_length)) {
        return Vector2f{};
    }

    const float correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    return error * (correction_length / error_length);
}

// inverse of the sqrt controller.  calculates the input (aka error) to the sqrt_controller required to achieve a given output
float inv_sqrt_controller(float output, float p, float D_max)
{
    if (is_positive(D_max) && is_zero(p)) {
        return (output * output) / (2.0f * D_max);
    }
    if ((is_negative(D_max) || is_zero(D_max)) && !is_zero(p)) {
        return output / p;
    }
    if ((is_negative(D_max) || is_zero(D_max)) && is_zero(p)) {
        return 0.0f;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    const float linear_velocity = D_max / p;

    if (fabsf(output) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        return output / p;
    }

    const float linear_dist = D_max / sq(p);
    const float stopping_dist = (linear_dist * 0.5f) + sq(output) / (2.0f * D_max);
    return is_positive(output) ? stopping_dist : -stopping_dist;
}

// calculate the stopping distance for the square root controller based deceleration path
float stopping_distance(float velocity, float p, float accel_max)
{
    return inv_sqrt_controller(velocity, p, accel_max);
}

// calculate the maximum acceleration or velocity in a given direction
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg)
{
    if (is_zero(direction.length_squared())) {
        return 0.0f;
    }

    max_xy = fabsf(max_xy);
    max_z_pos = fabsf(max_z_pos);
    max_z_neg = fabsf(max_z_neg);

    direction.normalize();
    const float xy_length = Vector2f{direction.x, direction.y}.length();

    if (is_zero(xy_length)) {
        return is_positive(direction.z) ? max_z_pos : max_z_neg;
    }

    if (is_zero(direction.z)) {
        return max_xy;
    }

    const float slope = direction.z/xy_length;
    if (is_positive(slope)) {
        if (fabsf(slope) < max_z_pos/max_xy) {
            return max_xy/xy_length;
        }
        return fabsf(max_z_pos/direction.z);
    }

    if (fabsf(slope) < max_z_neg/max_xy) {
        return max_xy/xy_length;
    }
    return fabsf(max_z_neg/direction.z);
}
