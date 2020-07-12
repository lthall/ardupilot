#pragma once

/*
 * LOCATION
 */

// Proportional controller with piecewise sqrt sections to constrain second derivative
float sqrt_controller(float error, float p, float second_ord_lim, float dt);

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
