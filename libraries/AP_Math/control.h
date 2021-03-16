#pragma once

#include "vector2.h"
#include "vector3.h"

/*
  common controller helper functions
 */

// update_pos_vel_accel projects the position and velocity, pos and vel, forward in time based on a time step of dt and acceleration of accel.
// update_pos_vel_accel - single axis projection.
void update_pos_vel_accel(float& pos, float& vel, float accel, float dt);
void update_pos_vel_accel_z(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt);

// update_pos_vel_accel_xy - dual axis projection operating on the x, y axis of Vector2f or Vector3f inputs.
void update_pos_vel_accel_xy(Vector2f& pos, Vector2f& vel, Vector2f& accel, float dt);
void update_pos_vel_accel_xy(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt);

/* shape_vel calculates a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinematic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
*/
void shape_vel(float& vel_input, float vel, float& accel, float accel_max, float tc, float dt);
void shape_vel_z(Vector3f& vel_input, const Vector3f& vel, Vector3f& accel, float accel_max, float tc, float dt);

/* shape_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
*/
void shape_vel_xy(Vector2f& vel_input, const Vector2f& vel, Vector2f& accel, float accel_max, float tc, float dt);
void shape_vel_xy(Vector3f& vel_input, const Vector3f& vel, Vector3f& accel, float accel_max, float tc, float dt);

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
*/
void shape_pos_vel(float& pos_input, float vel_input, float pos, float vel, float& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt);
void shape_pos_vel_z(Vector3f& pos_input, const Vector3f& vel_input, const Vector3f& pos, const Vector3f& vel, Vector3f& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt);

/* shape_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
 This function operates on the x and y axis of both Vector2f or Vector3f inputs.
*/
void shape_pos_vel_xy(Vector2f& pos_input, const Vector2f& vel_input, const Vector2f& pos, const Vector2f& vel, Vector2f& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt);
void shape_pos_vel_xy(Vector3f& pos_input, const Vector3f& vel_input, const Vector3f& pos, const Vector3f& vel, Vector3f& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt);

// proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

// proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller(const Vector2f& error, float p, float second_ord_lim, float dt);

// proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f sqrt_controller_xy(const Vector3f& error, float p, float second_ord_lim);

// inverse of the sqrt controller.  calculates the input (aka error) to the sqrt_controller required to achieve a given output
float inv_sqrt_controller(float output, float p, float D_max);

// calculate the stopping distance for the square root controller based deceleration path
float stopping_distance(float velocity, float p, float accel_max);

// calculate the stopping distance for the square root controller based deceleration path
Vector2f stopping_distance_xy(const Vector2f &velocity, float p, float accel_max);

// calculate the maximum acceleration or velocity in a given direction
// based on horizontal and vertical limits.
float kinematic_limit(Vector3f direction, float max_xy, float max_z_pos, float max_z_neg);
