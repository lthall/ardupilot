#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P_2D.h>                 // P library
#include <AC_PID/AC_P_1D.h>                 // P library
#include <AC_PID/AC_PID_2D.h>               // PID library (2-axis)
#include <AC_PID/AC_PID_1D.h>               // PID library (1-axis)
#include <AC_PID/AC_PID.h>                  // PID library
#include <AC_PID/AC_PI_2D.h>           // PI library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library
#include "AC_AttitudeControl.h"             // Attitude control library
#include <AP_Motors/AP_Motors.h>            // motors library
#include <AP_Vehicle/AP_Vehicle.h>          // common vehicle parameters


// position controller default definitions
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.

#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define POSCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate

#define POSCONTROL_ACTIVE_TIMEOUT_US            200000  // position controller is considered active if it has been called within the past 0.2 seconds

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define POSCONTROL_THROTTLE_CUTOFF_FREQ         2.0f    // low-pass filter on accel error (unit: hz)
#define POSCONTROL_ACCEL_FILTER_HZ              2.0f    // low-pass filter on acceleration (unit: hz)
#define POSCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration

#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, AC_AttitudeControl& attitude_control);

    ///
    /// initialisation functions
    ///

    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    ///     updates z axis accel controller's D term filter
    void set_dt(float delta_sec);
    float get_dt() const { return _dt; }

    /// set_max_speed_z - sets maximum climb and descent rates
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     leash length will be recalculated

    // get_leash_NE - returns horizontal leash length in cm
    float get_leash_NE() const { return _leash; }

    /// set the horizontal leash length
    void set_leash_length_NE(float leash) { _leash = leash; }

    // get_leash_down_z, get_leash_up_z - returns vertical leash lengths in cm
    float get_leash_down_D() const { return _leash_down_z; }
    float get_leash_up_D() const { return _leash_up_z; }

    void set_max_speed_D(float speed_down, float speed_up);

    /// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
    ///     leash length will be recalculated
    void set_max_accel_D(float accel);
    void set_max_accel_D_cm(float accel) {set_max_accel_D(0.01f * accel);}

    /// set_max_accel_NE - set the maximum horizontal acceleration in cm/s/s
    ///     leash length will be recalculated
    void set_max_accel_NE(float accel);
    float get_max_accel_NE() const { return _accel; }

    /// set_max_speed_NE - set the maximum horizontal speed maximum in cm/s
    ///     leash length will be recalculated
    void set_max_speed_NE(float speed);
    float get_max_speed_NE() const { return _speed; }

    /// get_target_pos_NED - returns NED target position
    const Vector3f& get_target_pos_NED() { return _pos_target; }

    /// get_desired_velocity - returns NED desired velocity
    const Vector3f& get_desired_vel_NED() { return _vel_desired; }
    const Vector3f& get_target_vel_NED() { return _vel_target; }

    /// get_desired_velocity - returns NED desired velocity
    const Vector3f& get_desired_accel_NED() { return _accel_desired; }
    const Vector3f& get_target_accel_NED() { return _accel_target; }

    /// init_NED_controller - initialise the NE controller
    ///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
    ///     should be called once whenever significant changes to the position target are made
    ///     this does not update the NE target
    void init_NE_controller();

    /// init_NED_controller - initialise the NE controller
    ///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
    ///     should be called once whenever significant changes to the position target are made
    ///     this does not update the NE target
    void init_D_controller();

    /// standby_NED_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_NED_reset();



    ///
    /// Interface Functions primarily for unit transforms and minimising initial code changes.
    ///

    ///
    /// z position controller
    ///

    /// set_max_speed_z - sets maximum climb and descent rates
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    ///     leash length will be recalculated
    void set_max_speed_z(float speed_down, float speed_up) {set_max_speed_D(speed_down * 0.01f, speed_up * 0.01f);}

    /// get_max_speed_up - accessor for current maximum up speed in cm/s
    float get_max_speed_up() const { return _speed_up * 100.0f; }

    /// get_max_speed_down - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down() const { return _speed_down * 100.0f; }

    /// get_vel_target_z - returns current vertical speed in cm/s
    float get_vel_target_z() const { return _vel_target.z * 100.0f; }

    /// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
    ///     leash length will be recalculated
    void set_max_accel_z(float accel_cmss) {_accel_z = accel_cmss * 0.01f;}

    /// get_max_accel_z - returns current maximum vertical acceleration in cm/s/s
    float get_max_accel_z() const { return _accel_z * 100.0f; }

    /// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
    ///     called by update_z_controller if z-axis speed or accelerations are changed
    void calc_leash_length_z();

    /// set_alt_target - set altitude target in cm above home
    void set_alt_target(float alt_cm) { _pos_target.z = alt_cm * 0.01f; }

    /// set_alt_target_with_slew - adjusts target towards a final altitude target
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    void set_alt_target_with_slew(float alt_cm, float dt) {;}
    /// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    virtual void set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend);

    /// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    virtual void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend);

    /// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     almost no checks are performed on the input
    void add_takeoff_climb_rate(float climb_rate_cms, float dt);

    /// set_alt_target_to_current_alt - set altitude target to current altitude
    void set_alt_target_to_current_alt() { set_target_to_vehicle_pos_D(); }

    /// shift altitude target (positive means move altitude up)
    void shift_alt_target(float z_cm) {_pos_target.z += -z_cm;}

    /// relax_alt_hold_controllers - set all desired and targets to measured
    void relax_alt_hold_controllers(float throttle_setting);

    /// get_alt_target - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_alt_target() const { return -_pos_target.z * 100.0f; }

    /// get_alt_error - returns altitude error in cm
    float get_alt_error() const {return _p_pos_D.get_error();}

    /// get_vel_z_error_ratio - returns the proportion of error relative to the maximum request
    float get_vel_z_control_ratio() const { return constrain_float(_vel_z_control_ratio, 0.0f, 1.0f); }

    // returns horizontal error in cm
    float get_horizontal_error() const {return _p_pos_NE.get_error_mag();}

    /// set_target_to_stopping_point_z - sets altitude target to reasonable stopping altitude in cm above home
    void set_target_to_stopping_point_z() {set_target_to_stopping_pos_D();}

    /// init_takeoff - initialises target altitude if we are taking off
    void init_takeoff();

    // is_active - returns true if the z-axis position controller has been run very recently
    bool is_active_z() const {return is_active_D();}

    /// update_z_controller - fly to altitude in cm above home
    void update_z_controller() {update_D_controller();}

    // get_leash_down_z, get_leash_up_z - returns vertical leash lengths in cm
    float get_leash_down_z() const { return _leash_down_z; }
    float get_leash_up_z() const { return _leash_up_z; }

    ///
    /// xy position controller
    ///

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
    float get_lean_angle_max_cd() const;

    /// init_xy_controller - initialise the xy controller
    ///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
    ///     should be called once whenever significant changes to the position target are made
    ///     this does not update the xy target
    void init_xy_controller() {init_NE_controller();}

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset() {standby_NED_reset();}

    /// set_max_accel_xy - set the maximum horizontal acceleration in cm/s/s
    ///     leash length will be recalculated
    void set_max_accel_xy(float accel_cmss){_accel = accel_cmss * 0.01f;}
    float get_max_accel_xy() const { return _accel * 100.0f; }

    /// set_max_speed_xy - set the maximum horizontal speed maximum in cm/s
    ///     leash length will be recalculated
    void set_max_speed_xy(float speed_cms) {_speed = speed_cms * 0.01f;}
    float get_max_speed_xy() const { return _speed * 100.0f; }

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_limit_accel_xy(void) { _limit.accel_NE = true; }

    /// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
    ///     should be called whenever the speed, acceleration or position kP is modified
    void calc_leash_length_xy();

    /// set the horizontal leash length
    void set_leash_length_xy(float leash) { _leash = leash; }

    /// get_pos_target - get target as position vector (from home in cm)
    const Vector3f& get_pos_target() const { return _pos_target; }

    /// set_pos_target in cm from home
    void set_pos_target(const Vector3f& position) {;}

    /// set_xy_target in cm from home
    void set_xy_target(float x, float y) {;}

    /// shift position target target in x, y axis
    void shift_pos_xy_target(float x_cm, float y_cm) {;}

    /// get_desired_velocity - returns xy desired velocity (i.e. feed forward) in cm/s in lat and lon direction
    const Vector3f& get_desired_velocity() { return _vel_desired; }

    /// set_desired_velocity_z - sets desired velocity in cm/s in z axis
    void set_desired_velocity_z(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    // set desired acceleration in cm/s in xy axis
    void set_desired_accel_xy(float accel_lat_cms, float accel_lon_cms) { _accel_desired.x = accel_lat_cms; _accel_desired.y = accel_lon_cms; }

    /// set_desired_velocity_xy - sets desired velocity in cm/s in lat and lon directions
    ///     when update_xy_controller is next called the position target is moved based on the desired velocity and
    ///     the desired velocities are fed forward into the rate_to_accel step
    void set_desired_velocity_xy(float vel_lat_cms, float vel_lon_cms) {_vel_desired.x = vel_lat_cms; _vel_desired.y = vel_lon_cms; }

    /// set_desired_velocity - sets desired velocity in cm/s in all 3 axis
    ///     when update_vel_controller_xyz is next called the position target is moved based on the desired velocity
    void set_desired_velocity(const Vector3f &des_vel) { _vel_desired = des_vel; }

    // overrides the velocity process variable for one timestep
    void override_vehicle_velocity_xy(const Vector2f& vel_xy) { _vehicle_vel.x = vel_xy.x; _vehicle_vel.y = vel_xy.y; _limit.vehicle_horiz_vel_override = true; }

    // is_active_xy - returns true if the xy position controller has been run very recently
    bool is_active_xy() const {return is_active_NE();}

    /// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
    ///     when use_desired_velocity is true the desired velocity (i.e. feed forward) is incorporated at the pos_to_rate step
    void update_xy_controller() {;}

    /// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
    void set_target_to_stopping_point_xy() {set_target_to_stopping_pos_NE();}

    /// get_stopping_point_xyz - calculates stopping point based on current position, velocity, vehicle acceleration
    ///     distance_max allows limiting distance to stopping point
    ///     results placed in stopping_position vector
    ///     set_accel_xy() should be called before this method to set vehicle acceleration
    ///     set_leash_length() should have been called before this method
    void get_stopping_point_xyz(Vector3f &stopping_point) {Vector2f xy; float z = 0.0f; get_stopping_pos_NE(xy);get_stopping_pos_D(z); stopping_point =  Vector3f(xy.x, xy.y, -z) * 100.0f;}

    /// get_distance_to_target - get horizontal distance to position target in cm (used for reporting)
    float get_distance_to_target() const {return _p_pos_NE.get_error_mag();}

    /// get_bearing_to_target - get bearing to target position in centi-degrees
    int32_t get_bearing_to_target() const {return get_bearing_cd(_inav.get_position(), _pos_target);}

    /// xyz velocity controller

    /// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
    void init_vel_controller_xyz() {;}

    /// update_velocity_controller_xy - run the XY velocity controller - should be called at 100hz or higher
    ///     velocity targets should we set using set_desired_velocity_xy() method
    ///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
    ///     throttle targets will be sent directly to the motors
    void update_vel_controller_xy() {;}

    /// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
    ///     velocity targets should we set using set_desired_velocity_xyz() method
    ///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
    ///     throttle targets will be sent directly to the motors
    void update_vel_controller_xyz() {;}

    /// get desired roll, pitch which should be fed into stabilize controllers
    float get_roll() const { return _roll_target; }
    float get_pitch() const { return _pitch_target; }

    // get_leash_xy - returns horizontal leash length in cm
    float get_leash_xy() const { return _leash; }

    /// get pid controllers
    AC_P_2D& get_pos_xy_p() { return _p_pos_NE; }
    AC_P_1D& get_pos_z_p() { return _p_pos_D; }
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_NE; }
    AC_PID_1D& get_vel_z_p() { return _pid_vel_D; }
    AC_PID& get_accel_z_pid() { return _pid_accel_D; }

    /// accessors for reporting
    const Vector3f& get_vel_target() const { return _vel_target; }
    const Vector3f& get_accel_target() const { return _accel_target; }

    // time_since_last_xy_update - returns time in seconds since the horizontal position controller was last run
    float time_since_last_xy_update() const {return time_since_last_NE_update();}


    ///
    /// Control functions
    ///

    // set target position to the stopping point in m in NED axis

    // set desired position in m in NE axis
    void set_target_pos_NED(Vector3f pos_target);

    // set desired velocity in m/s in NE axis
    void set_desired_vel_NED(Vector3f vel_desired);

    // set desired acceleration in m/s/s in NE axis
    void set_desired_accel_NED(Vector3f accel_desired);

    void set_target_to_stopping_pos_NED();

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NED_pos_vel_accel(Vector3f pos_target, Vector3f vel_desired, Vector3f accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NED_vel_accel(Vector3f vel_desired, Vector3f accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NED_vel_accel(Vector3f accel_desired);


    // set target position to the stopping point in m in NE axis

    // set desired position in m in NE axis
    void set_target_pos_NE(Vector2f pos_target);
    void set_target_pos_NE(Vector3f pos_target) {set_target_pos_NE(Vector2f(pos_target.x, pos_target.y));}
    void set_target_to_vehicle_pos_NE();
    void set_target_to_stopping_pos_NE();

    // set desired velocity in m/s in NE axis
    void set_desired_vel_NE(Vector2f vel_desired);
    void set_desired_vel_NE(Vector3f vel_desired) {set_desired_vel_NE(Vector2f(vel_desired.x, vel_desired.y));}
    void set_desired_vel_to_zero_NE() {set_desired_vel_NE(Vector2f());}
    void set_desired_to_vehicle_vel_NE();

    // set desired acceleration in m/s/s in NE axis
    void set_desired_accel_NE(Vector2f accel_desired);
    void set_desired_accel_to_zero_NE() {set_desired_accel_NE(Vector2f());}

    /// desired_vel_to_pos - move position target using desired velocities
    void desired_vel_to_pos_NE();

    /// desired_vel_to_pos - move position target using desired velocities
    void desired_accel_to_vel_NE();

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NE_pos_vel_accel(Vector2f pos_target, Vector2f vel_desired, Vector2f accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NE_vel_accel(Vector2f vel_desired, Vector2f accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_NE_vel_accel(Vector2f accel_desired);


    // set target position to the stopping point in m in D axis

    // set desired position in m in NE axis
    void set_target_pos_D(float pos_target);

    // set target position to the stopping point in m in D axis
    void set_target_to_vehicle_pos_D();

    // set target position to the stopping point in m in D axis
    void set_target_to_stopping_pos_D();

    // set desired velocity in m/s in NE axis
    void set_desired_vel_D(float vel_desired);

    // set target position to the stopping point in m in D axis
    void set_desired_to_vehicle_vel_D();

    // set desired acceleration in m/s/s in NE axis
    void set_desired_accel_D(float accel_desired);

    /// desired_vel_to_pos - move position target using desired velocities
    void desired_vel_to_pos_D();

    /// desired_vel_to_pos - move position target using desired velocities
    void desired_accel_to_vel_D();

    // Command a Quaternion attitude with feedforward and smoothing
    void input_D_pos_vel_accel(float pos_target, float vel_desired, float accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_D_vel_accel(float vel_desired, float accel_desired);

    // Command a Quaternion attitude with feedforward and smoothing
    void input_D_vel_accel(float accel_desired);



    ///
    /// Control functions
    ///

    /// run horizontal position controller correcting position and velocity
    ///     converts position (_pos_target) to target velocity (_vel_target)
    ///     desired velocity (_vel_desired) is combined into final target velocity
    ///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
    ///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
    void update_NED_controller();
    void update_NE_controller();
    void update_D_controller();
    void run_NE_controller();
    void run_D_controller();

    void run_frame_conversion();

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_xs, float accel_ys, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void attitude_to_accel(float& accel_xs, float& accel_ys) const;

    void target_attitude_to_accel(Vector3f euler_angle, float &accel_x, float &accel_y) const;

    ///
    /// private methods
    ///

    /// get_pos_NED - converts NEU from inav to NED
    Vector3f get_pos_NED() const;

    /// get_vel_NED - converts NEU from inav to NED
    Vector3f get_vel_NED() const;

    /// get pid controllers
    AC_P_2D& get_pos_NE_p() { return _p_pos_NE; }
    AC_P_1D& get_pos_D_p() { return _p_pos_D; }
    AC_PID_2D& get_vel_NE_pid() { return _pid_vel_NE; }
    AC_PID_1D& get_vel_D_p() { return _pid_vel_D; }
    AC_PID& get_accel_D_pid() { return _pid_accel_D; }

    // time_since_last_NE_update - returns time in seconds since the horizontal position controller was last run
    float time_since_last_NE_update() const {return AP_HAL::micros64() - _last_update_NE_us;}

    void write_log();

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable or disable high vibration compensation
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    /// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
    ///     distance_max allows limiting distance to stopping point
    ///     results placed in stopping_position vector
    ///     set_accel_xy() should be called before this method to set vehicle acceleration
    ///     set_leash_length() should have been called before this method
    void get_stopping_pos_NE(Vector2f &stopping_point);

    /// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
    void get_stopping_pos_D(float &stopping_pos) const;

    // is_active_D - returns true if the z-axis position controller has been run very recently
    bool is_active_NE() const;

    // is_active_D - returns true if the z-axis position controller has been run very recently
    bool is_active_D() const;

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_limit_accel_NE(void) { _limit.accel_NE = true; }

    Vector3f NEDtoNEUcm(Vector3f NED) const {NED.z = -NED.z; return NED;}

    bool limit_vector_length(float input_x, float input_y, float length_max);

    /// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
    float calc_leash_length(float speed_cms, float accel_cms, float kP) const;

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // limit flags structure
    struct poscontrol_limit_flags {
        bool  pos_up;       // 1 if we have hit the vertical position leash limit while going up
        bool  pos_down;     // 1 if we have hit the vertical position leash limit while going down
        bool  vel_up;       // 1 if we have hit the vertical velocity limit going up
        bool  vel_down;     // 1 if we have hit the vertical velocity limit going down
        bool  accel_NE;     // 1 if we have hit the horizontal accel limit
        bool  accel_z;      // 1 if we have hit the vertical accel limit
        bool  vehicle_horiz_vel_override;      // 1 if we have hit the vertical accel limit
    } _limit;

    ///
    /// z controller private methods
    ///

    /// initialise and check for ekf position resets
    void init_ekf_NE_reset();
    void check_for_ekf_NE_reset();
    void init_ekf_D_reset();
    void check_for_ekf_D_reset();

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    const AP_Motors&            _motors;
    AC_AttitudeControl&         _attitude_control;

    // parameters
    AP_Float    _accel_NE_filt_hz;      // XY acceleration filter cutoff frequency
    AP_Float    _lean_angle_max;        // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AC_P_2D     _p_pos_NE;
    AC_P_1D     _p_pos_D;
    AC_PID_2D   _pid_vel_NE;
    AC_PID_1D   _pid_vel_D;
    AC_PID      _pid_accel_D;

    // Pos Controller internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    uint64_t    _last_update_NE_us;     // system time (in microseconds) since last update_NE_controller call
    uint64_t    _last_update_D_us;      // system time (in microseconds) of last update_z_controller call
    float       _leash;                 // horizontal leash length in cm.  target will never be further than this distance from the vehicle
    float       _leash_down_z;          // vertical leash down in cm.  target will never be further than this distance below the vehicle
    float       _leash_up_z;            // vertical leash up in cm.  target will never be further than this distance above the vehicle

    // Alt Hold internal variables
    float       _speed_down;        // max descent rate in cm/s
    float       _speed_up;          // max climb rate in cm/s
    float       _speed;             // max horizontal speed in cm/s
    float       _accel_z;           // max vertical acceleration in cm/s/s
    float       _accel_z_max;      // max vertical acceleration in cm/s/s
    float       _accel;             // max horizontal acceleration in cm/s/s
    float       _vel_z_control_ratio = 2.0f;   // confidence that we have control in the vertical axis

    // output from controller
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller

    // position controller internal variables
    Vector3f    _pos_target;            // target location in cm from home
    Vector3f    _vel_desired;           // desired velocity in cm/s
    Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
    Vector3f    _vehicle_vel;           // velocity to use if _flags.vehicle_horiz_vel_override is set
    Vector3f    _accel_desired;         // desired acceleration in cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in cm/s/s
    LowPassFilterFloat _vel_error_filter;   // low-pass-filter on z-axis velocity error

    LowPassFilterVector2f _accel_target_filter; // acceleration target filter

    // ekf reset handling
    uint32_t    _ekf_NE_reset_ms;      // system time of last recorded ekf NE position reset
    uint32_t    _ekf_D_reset_ms;       // system time of last recorded ekf altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on
};
