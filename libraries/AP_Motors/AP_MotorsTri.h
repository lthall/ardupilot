/// @file	AP_MotorsTri.h
/// @brief	Motor control class for Tricopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <SRV_Channel/SRV_Channel.h>
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_CH_TRI_REAR    CH_3
#define AP_MOTORS_CH_TRI_GVNR    CH_4
#define AP_MOTORS_CH_TRI_YAW1    CH_5
#define AP_MOTORS_CH_TRI_YAW2    CH_6
#define AP_MOTORS_CH_TRI_YAW3    CH_7
#define AP_MOTORS_CH_TRI_YAW4    CH_8

#define AP_MOTORS_SERVO_INPUT_RANGE    4500    // input of -4500 will cause servos to their minimum (i.e. radio_min), +4500 will move them to their maximum (i.e. radio_max)


/// @class      AP_MotorsTri
class AP_MotorsTri : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsTri(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(loop_rate, speed_hz)
    {
    };

    // init
    void                init(motor_frame_class frame_class, motor_frame_type frame_type);

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type);

    // set update rate to motors - a value in hertz
    void                set_update_rate( uint16_t speed_hz );

    // output_test - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        output_test(uint8_t motor_seq, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    virtual uint16_t    get_motor_mask();

protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing();

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    // parameters

    SRV_Channel     *_rear_servo;  // elevator output channel
    SRV_Channel     *_gvnr_servo;  // elevator govner output channel
    SRV_Channel     *_yaw1_servo; // yaw output channel
    SRV_Channel     *_yaw2_servo; // yaw output channel
    SRV_Channel     *_yaw3_servo; // yaw output channel
    SRV_Channel     *_yaw4_servo; // yaw output channel
    float           _deflection_yaw;
    float           _thrust_right;
    float           _thrust_rear;
    float           _thrust_left;
};
