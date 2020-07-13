#pragma once

/*
 * LOCATION
 */

// control default definitions
#define CONTROL_TIME_CONSTANT_RATIO 4.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation

// shape_pos_vel_accel_xy takes the current pos, vel, accel, vel and accel limits and a time constant.
// The current acceleration is adjusted with jerk limiting to achieve the desired pos and vel.
void update_pos_vel_accel(float& pos, float& vel, float& accel, float dt);
void update_pos_vel_accel_xy(Vector2f& pos, Vector2f& vel, Vector2f& accel, float dt);
void update_pos_vel_accel_xy(Vector3f& pos, Vector3f& vel, Vector3f& accel, float dt);

// shape_pos_vel_accel_xy takes the current pos, vel, accel, vel and accel limits and a time constant.
// The current acceleration is adjusted with jerk limiting to achieve the desired pos and vel.
void shape_vel_accel(float& vel_target, const float& vel, float& accel, float vel_max, float accel_max, float tc, float dt);
void shape_vel_accel_xy(Vector2f& vel_target, const Vector2f& vel, Vector2f& accel, float vel_max, float accel_max, float tc, float dt);
void shape_vel_accel_xy(Vector3f& vel_des, const Vector3f& vel, Vector3f& accel, float vel_max, float accel_max, float tc, float dt);

// shape_pos_vel_accel_xy takes the current pos, vel, accel, vel and accel limits and a time constant.
// The current acceleration is adjusted with jerk limiting to achieve the desired pos and vel.
void shape_pos_vel_accel(float& pos_des, const float& vel_des, const float& pos, const float& vel, float& accel, float vel_max, float accel_max, float tc, float dt);
void shape_pos_vel_accel_xy(Vector2f& pos_des, const Vector2f& vel_des, const Vector2f& pos, const Vector2f& vel, Vector2f& accel, float vel_max, float accel_max, float tc, float dt);
void shape_pos_vel_accel_xy(Vector3f& pos_des, const Vector3f& vel_des, const Vector3f& pos, const Vector3f& vel, Vector3f& accel, float vel_max, float accel_max, float tc, float dt);

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(const float& error, float p, float second_ord_lim, float dt);

/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f sqrt_controller_xy(const Vector2f& error, float p, float second_ord_lim);

/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f sqrt_controller_xy(const Vector3f& error, float p, float second_ord_lim);

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
float stopping_point(float velocity, float p, float accel_max);

// Inverse proportional controller with piecewise sqrt sections to constrain second derivative
Vector2f stopping_point_xy(Vector2f velocity, float p, float accel_max);

/// limit vector to a given length, returns true if vector was limited
bool limit_vector_length(float& vector_x, float& vector_y, float max_length);
