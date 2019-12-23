#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.4f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.7f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.35f   // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // 0 was used for HOVER

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ACC_XY_FILT", 1, AC_PosControl, _accel_NE_filt_hz, POSCONTROL_ACCEL_FILTER_HZ),

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_D, "_POSZ_", 2, AC_PosControl, AC_P_1D),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_pid_vel_D, "_VELZ_", 3, AC_PosControl, AC_PID_1D),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FILT
    // @DisplayName: Acceleration (vertical) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_D, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_NE, "_POSXY_", 5, AC_PosControl, AC_P_2D),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VELXY_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_D_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_NE, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

    // @Param: _ANGLE_MAX
    // @DisplayName: Position Control Angle Max
    // @Description: Maximum lean angle autopilot can request.  Set to zero to use ANGLE_MAX parameter value
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_ANGLE_MAX", 7, AC_PosControl, _lean_angle_max, 0.0f),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_D(POSCONTROL_POS_Z_P, POSCONTROL_DT_50HZ),
    _pid_vel_D(POSCONTROL_VEL_Z_P, 0.0f, 0.0f, 0.0f, 0.0f, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ),
    _pid_accel_D(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, 0.0f, POSCONTROL_ACC_Z_IMAX, 0.0f, POSCONTROL_ACC_Z_FILT_HZ, 0.0f, POSCONTROL_ACC_Z_DT),
    _p_pos_NE(POSCONTROL_POS_XY_P, POSCONTROL_DT_50HZ),
    _pid_vel_NE(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, 0.0f, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ),
    _dt(POSCONTROL_DT_400HZ),
    _speed_down(POSCONTROL_SPEED_DOWN),
    _speed_up(POSCONTROL_SPEED_UP),
    _speed(POSCONTROL_SPEED),
    _accel_z(POSCONTROL_ACCEL_Z),
    _accel(POSCONTROL_ACCEL_XY),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _accel_target_filter(POSCONTROL_ACCEL_FILTER_HZ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _limit.pos_up = true;
    _limit.pos_down = true;
    _limit.vel_up = true;
    _limit.vel_down = true;
    _limit.accel_NE = true;
}

///
/// initialisation functions
///

/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update PID controller dt
    _p_pos_NE.set_dt(_dt);
    _p_pos_D.set_dt(_dt);
    _pid_vel_NE.set_dt(_dt);
    _pid_vel_D.set_dt(_dt);
    _pid_accel_D.set_dt(_dt);

    // update rate z-axis velocity error and accel error filters
    _vel_error_filter.set_cutoff_frequency(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
}

/// set_max_speed_z - set the maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
void AC_PosControl::set_max_speed_D(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = -fabsf(speed_down);

    // only update if there is a minimum of 1cm/s change and is valid
    if (((fabsf(_speed_down - speed_down) > 1.0f) || (fabsf(_speed_up - speed_up) > 1.0f)) && is_positive(speed_up) && is_negative(speed_down) ) {
        _speed_down = speed_down;
        _speed_up = speed_up;
        calc_leash_length_xy();
    }
}

/// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
void AC_PosControl::set_max_accel_D(float accel)
{
    if (fabsf(_accel_z - accel) > 1.0f) {
        _accel_z = accel;
        calc_leash_length_xy();
    }
}

/// set_max_accel_NE - set the maximum horizontal acceleration in cm/s/s
void AC_PosControl::set_max_accel_NE(float accel)
{
    if (fabsf(_accel - accel) > 1.0f) {
        _accel = accel;
        calc_leash_length_xy();
    }
}

/// set_max_speed_NE - set the maximum horizontal speed maximum in cm/s
void AC_PosControl::set_max_speed_NE(float speed)
{
    if (fabsf(_speed - speed) > 1.0f) {
        _speed = speed;
        calc_leash_length_xy();
    }
}

/// init_NE_controller - initialise the NE controller
///     this should be called after setting the position target and the desired velocity and acceleration
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the NE target
void AC_PosControl::init_NE_controller()
{
    // set roll, pitch lean angle targets to current attitude
    Vector3f euler_angle = _attitude_control.get_att_target_euler_cd();
    _roll_target = euler_angle.x;
    _pitch_target = euler_angle.y;
    target_attitude_to_accel(euler_angle, _accel_target.x, _accel_target.y);
    _accel_target_filter.reset(Vector2f(_accel_target.x, _accel_target.y));

    // initialise I terms from lean angles
    _pid_vel_NE.set_integrator(_accel_target - _accel_desired);
    _pid_vel_NE.reset_filter();

    // initialise ekf reset handlers
    init_ekf_NE_reset();
}

/// init_D_controller - initialise the NE controller
///     this should be called after setting the position target and the desired velocity and acceleration
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the NE target
void AC_PosControl::init_D_controller()
{
    set_target_to_vehicle_pos_D();
    set_desired_to_vehicle_vel_D();
    _accel_desired.z = 0.0f;

    // initialise ekf reset handlers
    init_ekf_D_reset();
}

/// standby_NED_reset - resets I terms and removes position error
///     This function will let Loiter and Alt Hold continue to operate
///     in the event that the flight controller is in control of the
///     aircraft when in standby.
void AC_PosControl::standby_NED_reset()
{
    // Set _pid_accel_D integrator to zero.
    _pid_accel_D.set_integrator(0.0f);

    // Set the target position to the current pos.
    _pos_target = get_pos_NED();

    // Set _pid_vel_NED integrators and derivative to zero.
    _pid_vel_NE.reset_filter();

    // initialise ekf NE reset handler
    init_ekf_NE_reset();
}

// write log to dataflash
void AC_PosControl::write_log()
{
    const Vector3f &pos_target = get_target_pos_NED();
    const Vector3f &vel_target = get_target_vel_NED();
    const Vector3f &accel_target = get_accel_target();
    const Vector3f &position = get_pos_NED();
    const Vector3f &velocity = get_vel_NED();
    float accel_x, accel_y;
    attitude_to_accel(accel_x, accel_y);

    AP::logger().Write("PSC",
                       "TimeUS,TPX,TPY,PX,PY,TVX,TVY,VX,VY,TAX,TAY,AX,AY",
                       "smmmmnnnnoooo",
                       "F000000000000",
                       "Qffffffffffff",
                       AP_HAL::micros64(),
                       double(pos_target.x * 0.01f),
                       double(pos_target.y * 0.01f),
                       double(position.x * 0.01f),
                       double(position.y * 0.01f),
                       double(vel_target.x * 0.01f),
                       double(vel_target.y * 0.01f),
                       double(velocity.x * 0.01f),
                       double(velocity.y * 0.01f),
                       double(accel_target.x * 0.01f),
                       double(accel_target.y * 0.01f),
                       double(accel_x * 0.01f),
                       double(accel_y * 0.01f));
}

///
/// Input shaping functions
///


// set target position to the stopping point in m in NED axis


// set desired position in m in NE axis
void AC_PosControl::set_target_pos_NED(Vector3f pos_target)
{
    _pos_target = pos_target;
}

// set desired velocity in m/s in NE axis
void AC_PosControl::set_desired_vel_NED(Vector3f vel_desired)
{
    _vel_desired = vel_desired;
}

// set desired acceleration in m/s/s in NE axis
void AC_PosControl::set_desired_accel_NED(Vector3f accel_desired)
{
    _accel_desired = accel_desired;
}

void AC_PosControl::set_target_to_stopping_pos_NED()
{
    Vector2f stopping_pos_NE;
    get_stopping_pos_NE(stopping_pos_NE);
    _pos_target.x = stopping_pos_NE.x;
    _pos_target.y = stopping_pos_NE.y;
    get_stopping_pos_D(_pos_target.z);
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NED_pos_vel_accel(Vector3f pos_target, Vector3f vel_desired, Vector3f accel_desired)
{
    _pos_target = pos_target;
    _vel_desired = vel_desired;
    _accel_desired = accel_desired;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NED_vel_accel(Vector3f vel_desired, Vector3f accel_desired)
{
    _pos_target = _vel_desired * _dt;
    _vel_desired = vel_desired;
    _accel_desired = accel_desired;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NED_vel_accel(Vector3f accel_desired)
{
    _pos_target = _vel_desired * _dt;
    _vel_desired = _accel_desired * _dt;
    _accel_desired = accel_desired;
}


// set target position to the stopping point in m in NE axis


// set desired position in m in NE axis
void AC_PosControl::set_target_pos_NE(Vector2f pos_target)
{
    _pos_target.x = pos_target.x;
    _pos_target.y = pos_target.y;
}

// set target position to the stopping point in m in D axis
void AC_PosControl::set_target_to_vehicle_pos_NE()
{
    Vector3f vehicle_pos = get_pos_NED();
    set_target_pos_NE(Vector2f(vehicle_pos.x, vehicle_pos.y));
}

// set target position to the stopping point in m in NE axis
void AC_PosControl::set_target_to_stopping_pos_NE()
{
    Vector2f pos_target;
    get_stopping_pos_NE(pos_target);
    _pos_target.x = pos_target.x;
    _pos_target.y = pos_target.y;
}

// set desired velocity in m/s in NE axis
void AC_PosControl::set_desired_vel_NE(Vector2f vel_desired)
{
    _vel_desired.x = vel_desired.x;
    _vel_desired.y = vel_desired.y;
}

// set desired acceleration in m/s/s in NE axis
void AC_PosControl::set_desired_accel_NE(Vector2f accel_desired)
{
    _accel_desired.x = accel_desired.x;
    _accel_desired.y = accel_desired.y;
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos_NE()
{
    // update target position
    _pos_target.x += _vel_desired.x * _dt;
    _pos_target.y += _vel_desired.y * _dt;
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_accel_to_vel_NE()
{
    // update target position
    _pos_target.x += _vel_desired.x * _dt;
    _pos_target.y += _vel_desired.y * _dt;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NE_pos_vel_accel(Vector2f pos_target, Vector2f vel_desired, Vector2f accel_desired)
{
    set_target_pos_NE(pos_target);
    set_desired_vel_NE(vel_desired);
    set_desired_accel_NE(accel_desired);
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NE_vel_accel(Vector2f vel_desired, Vector2f accel_desired)
{
    desired_vel_to_pos_NE();
    set_desired_vel_NE(vel_desired);
    set_desired_accel_NE(accel_desired);
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_NE_vel_accel(Vector2f accel_desired)
{
    desired_vel_to_pos_NE();
    desired_accel_to_vel_NE();
    set_desired_accel_NE(accel_desired);
}


// set target position to the stopping point in m in D axis


// set desired position in m in NE axis
void AC_PosControl::set_target_pos_D(float pos_target)
{
    _pos_target.z = pos_target;
}

// set desired velocity in m/s in NE axis
void AC_PosControl::set_desired_vel_D(float vel_desired)
{
    _vel_desired.z = vel_desired;
}

// set desired acceleration in m/s/s in NE axis
void AC_PosControl::set_desired_accel_D(float accel_desired)
{
    _accel_desired.z = accel_desired;
}

// set target position to the stopping point in m in D axis
void AC_PosControl::set_target_to_vehicle_pos_D()
{
    set_target_pos_D(-_inav.get_altitude());
}

// set target position to the stopping point in m in D axis
void AC_PosControl::set_target_to_stopping_pos_D()
{
    get_stopping_pos_D(_pos_target.z);
}

// set target position to the stopping point in m in D axis
void AC_PosControl::set_desired_to_vehicle_vel_D()
{
    set_desired_vel_D(-_inav.get_velocity_z());
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos_D()
{
    // update target position
    _pos_target.z += _vel_desired.z * _dt;
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_accel_to_vel_D()
{
    // update target position
    _pos_target.z += _vel_desired.z * _dt;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_D_pos_vel_accel(float pos_target, float vel_desired, float accel_desired)
{
    _pos_target.z = pos_target;
    _vel_desired.z = vel_desired;
    _accel_desired.z = accel_desired;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_D_vel_accel(float vel_desired, float accel_desired)
{
    _pos_target.z += _vel_desired.z * _dt;
    _vel_desired.z = vel_desired;
    _accel_desired.z = accel_desired;
}

// Command a Quaternion attitude with feedforward and smoothing
void AC_PosControl::input_D_vel_accel(float accel_desired)
{
    _pos_target.z += _vel_desired.z * _dt + 0.5*_accel_desired.z * sq(_dt);
    _vel_desired.z += _accel_desired.z * _dt;
    _accel_desired.z = accel_desired;
}

///
/// Control functions
///

/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::update_NED_controller()
{
    run_NE_controller();

    run_D_controller();

    // Frame acceleration frame conversion
    run_frame_conversion();
}

void AC_PosControl::update_NE_controller()
{
    run_NE_controller();

    // Frame acceleration frame conversion
    run_frame_conversion();
}

void AC_PosControl::update_D_controller()
{
    run_D_controller();

    // Frame acceleration frame conversion
    run_frame_conversion();
}

void AC_PosControl::run_NE_controller()
{
    float ekfGndSpdLimit, ekfNavVelGainScaler;
    AP::ahrs_navekf().getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    // check time since last cast
    const uint64_t now_us = AP_HAL::micros64();
    if (now_us - _last_update_NE_us > POSCONTROL_ACTIVE_TIMEOUT_US) {
        // incorrect use of position controller
        // initialisation should be done already
        // initialise the NE controller
    }
    _last_update_NE_us = now_us;

    // the following section converts target position in lat/lon directions to velocities in lat/lon frame

    Vector3f vehicle_pos = get_pos_NED();

    // Constrain the maximum length of _vel_target to the maximum position correction velocity
    // TODO: replace the leash length with a user definable maximum position correction
    Vector2f pos_target;
    pos_target.x = _pos_target.x;
    pos_target.y = _pos_target.y;
    Vector2f vel_target = _p_pos_NE.update_all(pos_target, Vector2f(vehicle_pos.y, vehicle_pos.y), 1.0f);
    _pos_target.x = pos_target.x;
    _pos_target.y = pos_target.y;

    _vel_target.x = vel_target.x;
    _vel_target.y = vel_target.y;

    _vel_target.x *= ekfNavVelGainScaler;
    _vel_target.y *= ekfNavVelGainScaler;

    // add velocity feed-forward
    _vel_target.x += _vel_desired.x;
    _vel_target.y += _vel_desired.y;

    // the following section converts target velocities in lat/lon directions to accelerations in lat/lon frame

    // check if vehicle velocity is being overridden
    if (_limit.vehicle_horiz_vel_override) {
        _limit.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_vel = get_vel_NED();
    }

    Vector2f accel_target = _pid_vel_NE.update_all(Vector2f(_vel_target.y, _vel_target.y), Vector2f(_vehicle_vel.y, _vehicle_vel.y), _limit.accel_NE || _motors.limit.throttle_upper);
    _accel_target.x = accel_target.x;
    _accel_target.y = accel_target.y;

    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    _accel_target.x *= ekfNavVelGainScaler;
    _accel_target.y *= ekfNavVelGainScaler;

    // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(MIN(_accel_NE_filt_hz, 5.0f * ekfNavVelGainScaler));
    _accel_target_filter.apply(Vector2f(_accel_target.x, _accel_target.y), _dt);

    // pass the correction acceleration to the target acceleration output
    _accel_target.x = _accel_target_filter.get().x;
    _accel_target.y = _accel_target_filter.get().y;

    // Add feed forward into the target acceleration output
    _accel_target.x += _accel_desired.x;
    _accel_target.y += _accel_desired.y;
}

void AC_PosControl::run_D_controller()
{
    // check time since last cast
    const uint64_t now_us = AP_HAL::micros64();
    if (now_us - _last_update_D_us > POSCONTROL_ACTIVE_TIMEOUT_US) {
        // incorrect use of position controller
        // initialisation should be done already
        // initialise the D controller
    }
    _last_update_D_us = now_us;

    Vector3f vehicle_pos = get_pos_NED();

    _vel_target.z = _p_pos_D.update_all(_pos_target.z, vehicle_pos.z, _leash_down_z, _leash_up_z, _limit.pos_down, _limit.pos_up);

    // add velocity feed-forward
    _vel_target.z += _vel_desired.z;

    _vehicle_vel = get_vel_NED();

    _accel_target.z = _pid_vel_D.update_all(_vel_target.z, _vehicle_vel.z, _limit.accel_z);

    // Add feed forward into the target acceleration output
    _accel_target.z += _accel_desired.z;
}

void AC_PosControl::run_frame_conversion()
{
    // the following section converts desired accelerations provided in lat/lon frame to roll/pitch angles

    // limit acceleration using maximum lean angles
    float angle_max = MIN(_attitude_control.get_althold_lean_angle_max(), get_lean_angle_max_cd());
    float accel_max = MIN(GRAVITY_MSS * 100.0f * tanf(ToRad(angle_max * 0.01f)), POSCONTROL_ACCEL_XY_MAX);
    _limit.accel_NE = limit_vector_length(_accel_target.x, _accel_target.y, accel_max);

    // update angle targets that will be passed to stabilize controller
    accel_to_lean_angles(_accel_target.x, _accel_target.y, _roll_target, _pitch_target);


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float z_accel_meas;         // actual acceleration

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_D.imax()) {
        _pid_accel_D.imax(_motors.get_throttle_hover() * 1000.0f);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        _accel_desired.z = 0.0f;
        const float thr_per_accelzs = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
        // during vibration compensation use feed forward with manually calculated gain
        // ToDo: clear pid_info P, I and D terms for logging
        if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_D.get_i()) && is_negative(_pid_vel_D.get_error())) || (is_negative(_pid_accel_D.get_i()) && is_positive(_pid_vel_D.get_error())))) {
            _pid_accel_D.set_integrator(_pid_accel_D.get_i() + _dt * thr_per_accelzs * 1000.0f * _pid_vel_D.get_error() * _pid_vel_D.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
        }
        thr_out = POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accelzs * _accel_target.z + _pid_accel_D.get_i() * 0.001f;
    } else {
        thr_out = _pid_accel_D.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

    // _speed_down is checked to be non-zero when set
    float error_ratio = _pid_vel_D.get_error()/_speed_down;

    _vel_z_control_ratio += _dt*0.1f*(0.5-error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::accel_to_lean_angles(float accel_x, float accel_y, float& roll_target, float& pitch_target) const
{
    float accel_right, accel_forward;

    // rotate accelerations into body forward-right frame
    // todo: this should probably be based on the desired heading not the current heading
    accel_forward = accel_x * _ahrs.cos_yaw() + accel_y * _ahrs.sin_yaw();
    accel_right = -accel_x * _ahrs.sin_yaw() + accel_y * _ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = atanf(-accel_forward / (GRAVITY_MSS));
    float cos_pitch_target = cosf(pitch_target);
    roll_target = atanf(accel_right * cos_pitch_target / (GRAVITY_MSS));
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::attitude_to_accel(float& accel_x, float& accel_y) const
{
    // rotate our roll, pitch angles into lat/lon frame
    accel_x = GRAVITY_MSS * (-_ahrs.cos_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() - _ahrs.sin_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll() * _ahrs.cos_pitch(), 0.5f);
    accel_y = GRAVITY_MSS * (-_ahrs.sin_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() + _ahrs.cos_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll() * _ahrs.cos_pitch(), 0.5f);
}

void AC_PosControl::target_attitude_to_accel(Vector3f euler_angle, float &accel_x, float &accel_y) const
{
    // convert our predicted attitude to an acceleration vector assuming we are hovering
    const float cos_pitch = cosf(euler_angle.y);
    const float accel_rgt = GRAVITY_MSS*100.0f * tanf(euler_angle.x)/cos_pitch;
    const float accel_fwd = -GRAVITY_MSS*100.0f * tanf(euler_angle.y);

    const float sin_yaw_target = sinf(euler_angle.z);
    const float cos_yaw_target = cosf(euler_angle.z);
    // rotate acceleration vectors input to lat/lon frame
    accel_x = (accel_fwd*cos_yaw_target - accel_rgt*sin_yaw_target);
    accel_y = (accel_fwd*sin_yaw_target + accel_rgt*cos_yaw_target);
}

///
/// private methods
///

/// get_pos_NED - converts NEU from inav to NED
Vector3f AC_PosControl::get_pos_NED() const
{
    const Vector3f& pos_NED = get_pos_NED();
    return Vector3f(pos_NED.x, pos_NED.y, -pos_NED.z);
}

/// get_vel_NED - converts NEU from inav to NED
Vector3f AC_PosControl::get_vel_NED() const
{
    Vector3f vel_NED = _inav.get_velocity();
    return Vector3f(vel_NED.x, vel_NED.y, -vel_NED.z);
}

/// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
float AC_PosControl::get_lean_angle_max_cd() const
{
    if (is_zero(_lean_angle_max)) {
        return _attitude_control.lean_angle_max();
    }
    return _lean_angle_max * 100.0f;
}

/// get_stopping_pos_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_max_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_pos_NE(Vector2f &stopping_pos)
{
    const Vector3f curr_pos = get_pos_NED();
    Vector3f curr_vel = get_vel_NED();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;        // the distance within the vehicle can stop
    float kP = _p_pos_NE.kP();
    Vector2f vel_error = _pid_vel_NE.get_error();
    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += vel_error.x;
        curr_vel.y += vel_error.y;
    }

    // calculate current velocity
    float vel_total = norm(curr_vel.x, curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel <= 0.0f || is_zero(vel_total)) {
        stopping_pos.x = curr_pos.x;
        stopping_pos.y = curr_pos.y;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _accel / kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
        stopping_dist = vel_total / kP;
    } else {
        linear_distance = _accel / (2.0f * kP * kP);
        stopping_dist = linear_distance + (vel_total * vel_total) / (2.0f * _accel);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_pos.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_pos.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_stopping_pos_z - calculates stopping point based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_pos_D(float &stopping_pos) const
{
    const Vector3f curr_pos = get_pos_NED();
    const Vector3f curr_vel = get_vel_NED();

    float linear_distance; // half the distance we swap between linear and sqrt and the distance we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // add current velocity error to avoid sudden jump in acceleration
    // consider case where we are stopping at he moving point
//    curr_vel.z -= _vel_desired.z;

    // avoid divide by zero by using current position if kP is very low or acceleration is zero
    if (_p_pos_D.kP() <= 0.0f || _accel_z <= 0.0f) {
        stopping_pos = curr_pos.z;
        return;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    linear_velocity = _accel_z / _p_pos_D.kP();

    if (fabsf(curr_vel.z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_pos = curr_pos.z + curr_vel.z / _p_pos_D.kP();
    } else {
        linear_distance = _accel_z / (2.0f * _p_pos_D.kP() * _p_pos_D.kP());
        if (curr_vel.z > 0) {
            stopping_pos = curr_pos.z + (linear_distance + curr_vel.z * curr_vel.z / (2.0f * _accel_z));
        } else {
            stopping_pos = curr_pos.z - (linear_distance + curr_vel.z * curr_vel.z / (2.0f * _accel_z));
        }
    }
    stopping_pos = constrain_float(stopping_pos, curr_pos.z - POSCONTROL_STOPPING_DIST_DOWN_MAX, curr_pos.z + POSCONTROL_STOPPING_DIST_UP_MAX);
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl::relax_alt_hold_controllers(float throttle_setting)
{
    set_target_to_vehicle_pos_D();
    _vel_desired.z = 0.0f;
    _vel_target.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _pid_accel_D.set_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f);
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_D.reset_filter();
}

// is_active_NE - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_NE() const
{
    return ((AP_HAL::micros64() - _last_update_NE_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

// is_active_D - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_D() const
{
    return ((AP_HAL::micros64() - _last_update_D_us) <= POSCONTROL_ACTIVE_TIMEOUT_US);
}

bool AC_PosControl::limit_vector_length(float input_x, float input_y, float length_max)
{
    Vector2f input = Vector2f(input_x, input_y);
    const float input_length = input.length();
    if (input_length > length_max) {
        input *= (input_length / length_max);
        input_x = input.x;
        input_y = input.y;
        return true;
    }
    return false;
}

///
/// Alt Hold functions
///

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    float desired_vel_D = -0.01 * climb_rate_cms;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((desired_vel_D < 0.0f && (!_motors.limit.throttle_lower || force_descend)) || (desired_vel_D > 0.0f && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += 0.01f * desired_vel_D * dt;
    }

    // do not use z-axis desired velocity feed forward
    // vel_desired set to desired climb rate for reporting and land-detector
    _vel_desired.z = -0.01f * desired_vel_D;
}

/// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
///     set force_descend to true during landing to allow target to move low enough to slow the motors
void AC_PosControl::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
{
    float desired_vel_D = -0.01 * climb_rate_cms;

    _vel_desired.z += _accel_desired.z * dt;
    _pos_target.z += _vel_desired.z * dt + 0.5*_accel_desired.z * sq(dt);

    // calculated increased maximum acceleration if over speed
    float accel_z = _accel_z;
    if (_vel_desired.z < _speed_down && !is_zero(_speed_down)) {
        accel_z *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down;
    }
    if (_vel_desired.z > _speed_up && !is_zero(_speed_up)) {
        accel_z *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up;
    }
    accel_z = constrain_float(accel_z, 0.0f, 7.5f);

    // jerk_z is calculated to reach full acceleration in 1000ms.
    float jerk_z = accel_z * POSCONTROL_JERK_RATIO;

    float accel_z_max = MIN(accel_z, safe_sqrt(2.0f * fabsf(desired_vel_D - _vel_desired.z) * jerk_z));

    _accel_z_max += jerk_z * dt;
    _accel_z_max = MIN(_accel_z_max, accel_z_max);
    _accel_desired.z = constrain_float((desired_vel_D - _vel_desired.z) / dt, - _accel_z_max, _accel_z_max);
}

/// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     almost no checks are performed on the input
void AC_PosControl::add_takeoff_climb_rate(float climb_rate_cms, float dt)
{
    _pos_target.z += climb_rate_cms * dt;
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
///     should be called whenever the speed, acceleration or position kP is modified
void AC_PosControl::calc_leash_length_xy()
{
        _leash = calc_leash_length(_speed, _accel, _p_pos_NE.kP());
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by update_z_controller if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    _leash_up_z = calc_leash_length(_speed_up, _accel_z, _p_pos_D.kP());
    _leash_down_z = calc_leash_length(-_speed_down, _accel_z, _p_pos_D.kP());
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float AC_PosControl::calc_leash_length(float speed_cms, float accel_cms, float kP) const
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if (speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    } else {
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f * kP * kP)) + (speed_cms * speed_cms / (2.0f * accel_cms));
    }

    // ensure leash is at least 1m long
    if (leash_length < POSCONTROL_LEASH_LENGTH_MIN) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}

/// initialise ekf xy position reset check
void AC_PosControl::init_ekf_NE_reset()
{
    Vector2f pos_shift;
    _ekf_NE_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_NE_reset()
{
    // check for position shift
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_NE_reset_ms) {
        shift_pos_xy_target(pos_shift.x * 100.0f, pos_shift.y * 100.0f);
        _ekf_NE_reset_ms = reset_ms;
    }
}

/// initialise ekf z axis reset check
void AC_PosControl::init_ekf_D_reset()
{
    float alt_shift;
    _ekf_D_reset_ms = _ahrs.getLastPosDownReset(alt_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_D_reset()
{
    // check for position shift
    float alt_shift;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift);
    if (reset_ms != 0 && reset_ms != _ekf_D_reset_ms) {
        shift_alt_target(-alt_shift * 100.0f);
        _ekf_D_reset_ms = reset_ms;
    }
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    const Vector3f& curr_pos = _inav.get_position();

    _pos_target.z = curr_pos.z;

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_accel_D.set_integrator((_attitude_control.get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f);

    // initialise ekf reset handler
    init_ekf_D_reset();
}

bool AC_PosControl::pre_arm_checks(const char *param_prefix,
                                   char *failure_msg,
                                   const uint8_t failure_msg_len)
{
    // validate AC_P members:
    const struct {
        const char *pid_name;
        AC_P_1D &p;
    } ps[] = {
        { "POSZ", get_pos_D_p() },
    };
    for (uint8_t i=0; i<ARRAY_SIZE(ps); i++) {
        // all AC_P's must have a positive P value:
        if (!is_positive(ps[i].p.kP())) {
            hal.util->snprintf(failure_msg, failure_msg_len, "%s_%s_P must be > 0", param_prefix, ps[i].pid_name);
            return false;
        }
    }

    // z-axis acceleration control PID doesn't use FF, so P and I must be positive
    if (!is_positive(get_accel_z_pid().kP())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_P must be > 0", param_prefix);
        return false;
    }
    if (!is_positive(get_accel_z_pid().kI())) {
        hal.util->snprintf(failure_msg, failure_msg_len, "%s_ACCZ_I must be > 0", param_prefix);
        return false;
    }

    return true;
}

