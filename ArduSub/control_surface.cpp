#include "Sub.h"


bool Sub::surface_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    pos_control.set_max_speed_accel_z(wp_nav.get_default_speed_down(), wp_nav.get_default_speed_up(), wp_nav.get_accel_z());

    // initialise position and desired velocity
    pos_control.init_pos_vel_accel_z();

    return true;

}

void Sub::surface_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    // Already at surface, hold depth at surface
    if (ap.at_surface) {
        set_mode(ALT_HOLD, ModeReason::SURFACE_COMPLETE);
    }

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // set target climb rate
    float cmb_rate = constrain_float(fabsf(wp_nav.get_default_speed_up()), 1, pos_control.get_max_speed_up());

    // record desired climb rate for logging
    desired_climb_rate = cmb_rate;

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate_ff(cmb_rate, true);
    pos_control.update_z_controller();

    // pilot has control for repositioning
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
