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
    _lim_D_Out = 10.0f;     // maximum first differential of output
}

// update_all - set target and measured inputs to PID controller and calculate outputs
// target and measurement are filtered
// if measurement is further than error_min or error_max (see set_limits_error method)
//   the target is moved closer to the measurement and limit_min or limit_max will be set true
float AC_P_1D::update_all(float &target, float measurement, bool &limit_min, bool &limit_max)
{
    limit_min = false;
    limit_max = false;

    // calculate distance _error
    float error = target - measurement;

    if (error < _lim_err_neg) {
        error = _lim_err_neg;
        limit_min = true;
    }
    if (error > _lim_err_pos) {
        error = _lim_err_pos;
        limit_max = true;
    }

    // ToDo: Replace sqrt_controller with optimal acceleration and jerk limited curve
    // MIN(_Dxy_max, _D2xy_max / _kxy_P) limits the max accel to the point where max jerk is exceeded
    return sqrt_controller(error, _kp, _lim_D_Out, _dt);
}

// set limits on error, output and output from D term
void AC_P_1D::set_limits_error(float lim_err_neg, float lim_err_pos, float lim_out_neg, float lim_out_pos, float lim_D_Out, float lim_D2_Out)
{
    _lim_err_neg = -inv_sqrt_controller(lim_err_neg, _kp, _lim_D_Out);
    _lim_err_pos = inv_sqrt_controller(lim_err_pos, _kp, _lim_D_Out);
    _lim_D_Out = lim_D_Out;
    if (is_positive(lim_D2_Out)) {
        // limit the first derivative so as not to exceed the second derivative
        _lim_D_Out = MIN(_lim_D_Out, lim_D2_Out / _kp);
    }
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_P_1D::sqrt_controller(float error, float p, float D_max, float dt)
{
    float output;
    if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        output = error * p;
    } else if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        if (is_positive(error)) {
            output = safe_sqrt(2.0f * D_max * (error));
        } else if (is_negative(error)) {
            output = -safe_sqrt(2.0f * D_max * (-error));
        } else {
            output = 0.0f;
        }
    } else {
        // both the P and second order limit have been defined.
        const float linear_dist = D_max / sq(p);
        if (error > linear_dist) {
            output = safe_sqrt(2.0f * D_max * (error - (linear_dist / 2.0f)));
        } else if (error < -linear_dist) {
            output = -safe_sqrt(2.0f * D_max * (-error - (linear_dist / 2.0f)));
        } else {
            output = error * p;
        }
    }
    if (!is_zero(dt)) {
        // this ensures we do not get small oscillations by over shooting the error correction in the last time step.
        return constrain_float(output, -fabsf(error) / dt, fabsf(error) / dt);
    }

    return output;
}

// Proportional controller with piecewise sqrt sections to constrain second derivative
float AC_P_1D::inv_sqrt_controller(float output, float p, float D_max)
{
    output = fabsf(output);

    if (!is_positive(D_max) && is_zero(p)) {
        return 0.0f;
    }

    if (!is_positive(D_max)) {
        // second order limit is zero or negative.
        return output / p;
    }

    if (is_zero(p)) {
        // P term is zero but we have a second order limit.
        return sq(output)/(2*D_max);
    }

    // both the P and second order limit have been defined.
    float linear_out = D_max / p;
    if (output > linear_out) {
        return sq(output)/(2*D_max) + D_max/(4*sq(p));
    }

    return output / p;
}
