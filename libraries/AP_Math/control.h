#pragma once

/*
  common controller helper functions
 */

// update_pos_vel_accel projects the position and velocity, pos and vel, forward in time based on a time step of dt and acceleration of accel.
// update_pos_vel_accel - single axis projection.
void update_pos_vel_accel(float& pos, float& vel, float& accel, float dt);

/* shape_vel calculates a jerk limited path from the current position, velocity and acceleration to an input velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
*/
void shape_vel(float& vel_input, float vel, float& accel, float accel_max, float tc, float dt);

/* shape_pos_vel calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
 The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
 The kinimatic path is constrained by :
     maximum velocity - vel_max,
     maximum acceleration - accel_max,
     time constant - tc.
 The time constant defines the acceleration error decay in the kinimatic path as the system approaches constant acceleration.
 The time constant also defines the time taken to achieve the maximum acceleration.
 The time constant must be positive.
 The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
*/
void shape_pos_vel(float& pos_input, float vel_input, float pos, float vel, float& accel, float vel_max, float vel_correction_max, float accel_max, float tc, float dt);

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(const float& error, float p, float second_ord_lim, float dt);

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
float stopping_point(float velocity, float p, float accel_max);

// limit vector to a given length, returns true if vector was limited
bool limit_vector_length(float& vector_x, float& vector_y, float max_length);
