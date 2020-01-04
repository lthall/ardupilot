#pragma once

/// @file   AC_P_2D.h
/// @brief  Generic PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class  AC_P_2D
/// @brief  Copter PID control class
class AC_P_2D {
public:

    // Constructor for PID
    AC_P_2D(float initial_p, float dt);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    //  update_all - set target and measured inputs to PID controller and calculate outputs
    //  target and error are filtered
    //  the derivative is then calculated and filtered
    //  the integral is then updated based on the setting of the limit flag
    Vector2f update_all(float &target_x, float &target_y, Vector2f measurement, float error_max, float D2_max) WARN_IF_UNUSED;
    Vector2f update_all(float &target_x, float &target_y, Vector3f measurement, float error_max, float D2_max) WARN_IF_UNUSED {
        return update_all(target_x, target_y, Vector2f(measurement.x, measurement.y), error_max, D2_max);
    }

    // get results from p controller
    Vector2f get_p() const WARN_IF_UNUSED;

    // reset_I - reset the integrator
    void reset_I();

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void reset_filter();

    // save gain to eeprom
    void save_gains();

    // get accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }

    // set accessors
    void kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:
    // parameters
    AP_Float    _kp;

    // internal variables
    float _dt;          // time step in seconds
};
