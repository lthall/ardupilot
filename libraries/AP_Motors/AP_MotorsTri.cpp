/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTri.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsTri.h"

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsTri::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    add_motor_num(AP_MOTORS_MOT_1);
    add_motor_num(AP_MOTORS_MOT_2);

    // set update rate for the 2 motors (but not the servo channels)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the ESCs can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_1] = true;
    motor_enabled[AP_MOTORS_MOT_2] = true;

    // find the yaw servo
    _rear_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor3, AP_MOTORS_CH_TRI_REAR);
    _gvnr_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor4, AP_MOTORS_CH_TRI_GVNR);
    _yaw1_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor5, AP_MOTORS_CH_TRI_YAW1);
    _yaw2_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor6, AP_MOTORS_CH_TRI_YAW2);
    _yaw3_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor7, AP_MOTORS_CH_TRI_YAW3);
    _yaw4_servo = SRV_Channels::get_channel_for(SRV_Channel::k_motor8, AP_MOTORS_CH_TRI_YAW4);
    if (!_rear_servo | !_gvnr_servo | !_yaw1_servo | !_yaw2_servo | !_yaw3_servo | !_yaw4_servo | !_rear_servo) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MotorsTri: unable to setup yaw channel");
        // don't set initialised_ok
        return;
    }

    // allow mapping of motor7
    add_motor_num(AP_MOTORS_CH_TRI_REAR);
    add_motor_num(AP_MOTORS_CH_TRI_GVNR);
    add_motor_num(AP_MOTORS_CH_TRI_YAW1);
    add_motor_num(AP_MOTORS_CH_TRI_YAW2);
    add_motor_num(AP_MOTORS_CH_TRI_YAW3);
    add_motor_num(AP_MOTORS_CH_TRI_YAW4);

    // we set four servos to angle
    _rear_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);
    _gvnr_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);
    _yaw1_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);
    _yaw2_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);
    _yaw3_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);
    _yaw4_servo->set_angle(AP_MOTORS_SERVO_INPUT_RANGE);

    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TRI);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsTri::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TRI);
}

// set update rate to motors - a value in hertz
void AP_MotorsTri::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 2 motors (but not the servo on channels)
    uint32_t mask = 
	    1U << AP_MOTORS_MOT_1 |
	    1U << AP_MOTORS_MOT_2;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsTri::output_to_motors()
{
    switch (_spool_mode) {
        case SHUT_DOWN:
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_1, get_pwm_output_min());
            rc_write(AP_MOTORS_MOT_2, get_pwm_output_min());
            rc_write(AP_MOTORS_CH_TRI_REAR, calc_pwm_output_1to1(-_pitch_radio_passthrough, _rear_servo));
            rc_write(AP_MOTORS_CH_TRI_GVNR, _gvnr_servo->get_output_min());
            rc_write(AP_MOTORS_CH_TRI_YAW1, calc_pwm_output_1to1(_yaw_radio_passthrough, _yaw1_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW2, calc_pwm_output_1to1(_yaw_radio_passthrough, _yaw2_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW3, calc_pwm_output_1to1(-_yaw_radio_passthrough, _yaw3_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW4, calc_pwm_output_1to1(-_yaw_radio_passthrough, _yaw4_servo));
            break;
        case SPIN_WHEN_ARMED:
            // sends output to motors when armed but not flying
            rc_write(AP_MOTORS_MOT_1, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_MOT_2, calc_spin_up_to_pwm());
            rc_write(AP_MOTORS_CH_TRI_REAR, calc_pwm_output_1to1(-_pitch_radio_passthrough * (1.0f - _spin_up_ratio), _rear_servo));
            rc_write(AP_MOTORS_CH_TRI_GVNR, _gvnr_servo->get_trim());
            rc_write(AP_MOTORS_CH_TRI_YAW1, calc_pwm_output_1to1(_yaw_radio_passthrough * (1.0f - _spin_up_ratio), _yaw1_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW2, calc_pwm_output_1to1(_yaw_radio_passthrough * (1.0f - _spin_up_ratio), _yaw2_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW3, calc_pwm_output_1to1(-_yaw_radio_passthrough * (1.0f - _spin_up_ratio), _yaw3_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW4, calc_pwm_output_1to1(-_yaw_radio_passthrough * (1.0f - _spin_up_ratio), _yaw4_servo));
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // set motor output based on thrust requests
            rc_write(AP_MOTORS_MOT_1, calc_thrust_to_pwm(_thrust_right));
            rc_write(AP_MOTORS_MOT_2, calc_thrust_to_pwm(_thrust_left));
            rc_write(AP_MOTORS_CH_TRI_REAR, calc_pwm_output_1to1(_thrust_rear, _rear_servo));
            rc_write(AP_MOTORS_CH_TRI_GVNR, _gvnr_servo->get_trim());
            rc_write(AP_MOTORS_CH_TRI_YAW1, calc_pwm_output_1to1(_deflection_yaw, _yaw1_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW2, calc_pwm_output_1to1(_deflection_yaw, _yaw2_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW3, calc_pwm_output_1to1(-_deflection_yaw, _yaw3_servo));
            rc_write(AP_MOTORS_CH_TRI_YAW4, calc_pwm_output_1to1(-_deflection_yaw, _yaw4_servo));
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsTri::get_motor_mask()
{
    // tri copter uses channels 1,2,4 and 7
    return rc_map_mask((1U << AP_MOTORS_MOT_1) |
                       (1U << AP_MOTORS_MOT_2) |
                       (1U << AP_MOTORS_CH_TRI_REAR) |
                       (1U << AP_MOTORS_CH_TRI_GVNR) |
                       (1U << AP_MOTORS_CH_TRI_YAW1) |
                       (1U << AP_MOTORS_CH_TRI_YAW2) |
                       (1U << AP_MOTORS_CH_TRI_YAW3) |
                       (1U << AP_MOTORS_CH_TRI_YAW4));
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsTri::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   roll_max = 0.0f;            // maximum change in motor value
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   thrust_out;                 // final thrust output to the motors

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in; // this does not use compensation as it is a collective pitch propeller on a governor.
    yaw_thrust = _yaw_in * get_compensation_gain(); // we scale this so a thrust request of 1.0f will ask for full servo deflection at full rear throttle
    throttle_thrust = get_throttle() * get_compensation_gain();

    float thrust_max = 1.0f;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, throttle_thrust, _throttle_thrust_max);

    _thrust_right = -roll_thrust/2.0f;
    _thrust_left = roll_thrust/2.0f;;
    _thrust_rear = -pitch_thrust;

    // calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible room margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // check everything fits
    roll_max = fabs(roll_thrust/2.0f);
    throttle_thrust_best_rpy = MIN(0.5f*thrust_max, _throttle_avg_max);
    if(is_zero(roll_max)){
        rpy_scale = 1.0f;
    } else {
        rpy_scale = constrain_float(throttle_thrust_best_rpy/roll_max, 0.0f, 1.0f);
    }

    // calculate how close the motors can come to the desired throttle
    thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if(rpy_scale < 1.0f){
        // Full range is being used by roll, pitch, and yaw.
        limit.roll_pitch = true;
        if (thr_adj > 0.0f){
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    }else{
        if(roll_max > throttle_thrust_best_rpy+thr_adj){
            // Throttle can't be reduced to desired value
            thr_adj = -(throttle_thrust_best_rpy-roll_max);
        }else if(roll_max > thrust_max - (throttle_thrust_best_rpy+thr_adj)){
            // Throttle can't be increased to desired value
            thr_adj = thrust_max - (throttle_thrust_best_rpy+roll_max);
            limit.throttle_upper = true;
        }
    }

    // calculate the throttle setting for the lift fan
    thrust_out = _throttle_avg_max + thr_adj;

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    _thrust_right = thrust_out + rpy_scale*_thrust_right;
    _thrust_left = thrust_out + rpy_scale*_thrust_left;



    // limit thrust out for calculation of actuator gains
    float thrust_out_actuator = constrain_float(MAX(_throttle_hover*0.5f,thrust_out), 0.1f, 1.0f);

    _deflection_yaw = yaw_thrust/thrust_out_actuator;

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, -1.0f, 1.0f);

    if (fabsf(_deflection_yaw) > 1.0f) {
        limit.yaw = true;
        _deflection_yaw = constrain_float(_deflection_yaw, -1.0f, 1.0f);
    }
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTri::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!armed()) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // front right servo 1
            rc_write(AP_MOTORS_CH_TRI_YAW3, pwm);
            break;
        case 2:
            // front right servo 2
            rc_write(AP_MOTORS_CH_TRI_YAW4, pwm);
            break;
        case 3:
            // right motor
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 4:
            // back servo
            rc_write(AP_MOTORS_CH_TRI_REAR, pwm);
            break;
        case 5:
            // back govner
            rc_write(AP_MOTORS_CH_TRI_GVNR, pwm);
            break;
        case 6:
            // front left motor
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 7:
            // front left servo 2
            rc_write(AP_MOTORS_CH_TRI_YAW1, pwm);
            break;
        case 8:
            // front left servo 1
            rc_write(AP_MOTORS_CH_TRI_YAW2, pwm);
            break;
        default:
            // do nothing
            break;
    }
}

/*
  call vehicle supplied thrust compensation if set. This allows for
  vehicle specific thrust compensation for motor arrangements such as
  the forward motors tilting
*/
void AP_MotorsTri::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        // convert 3 thrust values into an array indexed by motor number
        float thrust[4] { _thrust_right, _thrust_left, 0, _thrust_rear };

        // apply vehicle supplied compensation function
        _thrust_compensation_callback(thrust, 4);

        // extract compensated thrust values
        _thrust_right = thrust[0];
        _thrust_left  = thrust[1];
    }
}
