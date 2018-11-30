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
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500
#define THROTTLE_RANGE       100

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record successful initialisation if what we setup was the desired frame_class
    _flags.initialised_ok = (frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t loop_rate, uint16_t speed_hz) :
    AP_MotorsMulticopter(loop_rate, speed_hz)
{
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleTop, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleBot, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!_flags.initialised_ok) {
        return;
    }
    
    switch (_spool_mode) {
        case SHUT_DOWN:
            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
            _throttle = 0;
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft,  get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTop, get_pwm_output_min());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleBot, get_pwm_output_min());
            _aileron = -_deflection_yaw;
            _elevator = _deflection_pitch;
            _rudder = 0.0f;
            break;
        case SPIN_WHEN_ARMED:
            // set limits flags
            limit.roll_pitch = true;
            limit.yaw = true;
            limit.throttle_lower = true;
            limit.throttle_upper = true;
            // sends output to motors when armed but not flying
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft,  calc_spin_up_to_pwm());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, calc_spin_up_to_pwm());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTop, calc_spin_up_to_pwm());
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleBot, calc_spin_up_to_pwm());
            _aileron = -_deflection_yaw;
            _elevator = _deflection_pitch;
            _rudder = 0.0f;
            break;
        case SPOOL_UP:
        case THROTTLE_UNLIMITED:
        case SPOOL_DOWN:
            // initialize limits flags
            limit.roll_pitch = false;
            limit.yaw = false;
            limit.throttle_lower = false;
            limit.throttle_upper = false;
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft,  calc_thrust_to_pwm(_thrust_left));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, calc_thrust_to_pwm(_thrust_right));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleTop, calc_thrust_to_pwm(_thrust_rear));
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleBot, calc_thrust_to_pwm(_thrust_rear));
            _aileron = -_deflection_yaw;
            _elevator = _deflection_pitch;
            _rudder = 0.0f;
            break;
    }
    // outputs are setup here, and written to the HAL by the plane servos loop
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  _aileron*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _elevator*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder,   _rudder*SERVO_OUTPUT_RANGE);

#if APM_BUILD_TYPE(APM_BUILD_ArduCopter)
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
#endif
}

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust_best_rpy;   // throttle providing maximum roll, pitch and yaw range without climbing
    float   rpy_scale = 1.0f;           // this is used to scale the roll, pitch and yaw to fit within the motor limits
    float   roll_max = 0.0f;            // maximum change in motor value
    float   thr_adj;                    // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   thrust_out;                 // final thrust output to the motors

    // apply voltage and air pressure compensation
    roll_thrust = _roll_in * get_compensation_gain();
    pitch_thrust = _pitch_in; // this does not use compensation as it is a collective pitch propeller on a governor.
    yaw_thrust = _yaw_in; // we scale this so a thrust request of 1.0f will ask for full servo deflection at full rear throttle
    _throttle = get_throttle() * get_compensation_gain();

    float thrust_max = 1.0f;

    // sanity check throttle is above zero and below current limited throttle
    if (_throttle <= 0.0f) {
        _throttle = 0.0f;
        limit.throttle_lower = true;
    }
    if (_throttle >= _throttle_thrust_max) {
        _throttle = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    _throttle_avg_max = constrain_float(_throttle_avg_max, _throttle, _throttle_thrust_max);

//    float thrust_actuator_min = MIN(MAX(fabsf(pitch_thrust), fabsf(yaw_thrust)), _throttle_avg_max);
//    if(_throttle < thrust_actuator_min){
//        _throttle = thrust_actuator_min;
//        limit.throttle_lower = true;
//    }

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
    thr_adj = _throttle - throttle_thrust_best_rpy;
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
    _thrust_right = thrust_out - 0.5f * rpy_scale * roll_thrust;
    _thrust_left = thrust_out + 0.5f * rpy_scale * roll_thrust;
    _thrust_rear = constrain_float(_rear_max * MIN(_throttle, thrust_out) / _throttle_hover, 0.0f, _rear_max) - _rear_max * pitch_thrust;
    _thrust_front = constrain_float(_rear_max * MIN(_throttle, thrust_out) / _throttle_hover, 0.0f, _rear_max) + _rear_max * pitch_thrust;

    // constrain all outputs to 0.0f to 1.0f
    // test code should be run with these lines commented out as they should not do anything
    _thrust_right = constrain_float(_thrust_right, 0.0f, 1.0f);
    _thrust_left = constrain_float(_thrust_left, 0.0f, 1.0f);
    _thrust_rear = constrain_float(_thrust_rear, 0.0f, 1.0f);
    _thrust_front = constrain_float(_thrust_front, 0.0f, 1.0f);

    _deflection_pitch = pitch_thrust * _pitch_scale;

    if (fabsf(_deflection_pitch) > 1.0f) {
        limit.roll_pitch = true;
        _deflection_pitch = constrain_float(_deflection_pitch, -1.0f, 1.0f);
    }

    _deflection_yaw = yaw_thrust;

    float yaw_headroom = MAX(1.0f-fabsf(_deflection_pitch),(float)_yaw_headroom/1000.0f);
    if (fabsf(_deflection_yaw) > yaw_headroom) {
        limit.yaw = true;
        _deflection_yaw = constrain_float(_deflection_yaw, -yaw_headroom, yaw_headroom);
    }
}

