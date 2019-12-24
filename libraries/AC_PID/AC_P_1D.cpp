/// @file	AC_P_1D.cpp
/// @brief	Generic P algorithm

#include <AP_Math/AP_Math.h>
#include "AC_P_1D.h"

const AP_Param::GroupInfo AC_P_1D::var_info[] = {
    // @Param: P
    // @DisplayName: P Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_P_1D, _kp, 0),
    AP_GROUPEND
};

// Constructor
AC_P_1D::AC_P_1D(float initial_p, float dt) :
    _dt(dt)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    _kp = initial_p;
    _D_max = 10.0f;
    _D2_max = 2.0f * 9.8f;
    _error = 0.0f;
}

// set_dt - set time step in seconds
void AC_P_1D::set_dt(float dt)
{
    // set dt and calculate the input filter alpha
    _dt = dt;
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and _error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_P_1D::update_all(float &target, float measurement, float min, float max, bool limit_min, bool limit_max)
{
    // calculate distance _error
    float error = target - measurement;

    if (asymetricLimit(error, min, max, limit_min, limit_max )) {
        target = measurement + error;
    }

//    todo: Replace sqrt_controller with optimal acceleration and jerk limited curve
    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
//    return sqrt_controller(error, _kp, _D_max, _D2_max, _dt);
    return error * _kp;
}

float AC_P_1D::get_p() const
{
    return _error * _kp;
}

void AC_P_1D::load_gains()
{
    _kp.load();
}

void AC_P_1D::save_gains()
{
    _kp.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_P_1D::operator()(float initial_p, float dt)
{
    _kp = initial_p;
    _dt = dt;
}

/// asymetricLimit - set limits based on seperate max and min
bool AC_P_1D::asymetricLimit(float &input, float min, float max, bool &limitMin, bool &limitMax )
{
    limitMin = false;
    limitMax = false;
    if (input < min) {
        input = min;
        limitMin = true;
    }
    if (input > max) {
        input = max;
        limitMax = true;
    }

    return limitMin || limitMax;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_P_1D::sqrt_controller(float error, float p, float D_max, float D2_max, float dt)
{
    if(is_positive(D2_max)) {
        // limit the first derivative so as not to exceed the second derivative
        D_max = MIN(D_max, D2_max / p);
    }

    float correction;
    if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        correction = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            correction = safe_sqrt(2.0f * D_max * (error));
        } else if (is_negative(error)) {
            correction = -safe_sqrt(2.0f * D_max * (-error));
        } else {
            correction = 0.0f;
        }
    } else {
        // Both the P and second order limit have been defined.
        float linear_dist = D_max / sq(p);
        if (error > linear_dist) {
            correction = safe_sqrt(2.0f * D_max * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            correction = -safe_sqrt(2.0f * D_max * (-error - (linear_dist / 2.0f)));
        } else {
            correction = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(correction, -fabsf(error) / dt, fabsf(error) / dt);
    } else {
        return correction;
    }
}

