#include "Copter.h"

/*
 * Init and run calls for systemId, flight mode
 */

const AP_Param::GroupInfo Copter::ModeSystemId::var_info[] = {

    // @Param: _AXIS
    // @DisplayName: System identification axis
    // @Description: Controls which axis are being excited
    // @User: Standard
    // @Values: 0:None, 1:Input Roll Angle, 2:Input Pitch Angle, 3:Input Yaw Angle, 4:Recovery Roll Angle, 5:Recovery Pitch Angle, 6:Recovery Yaw Angle, 7:Disturbance Roll, 8:Disturbance Pitch, 9:Disturbance Yaw, 10:Rate Roll, 11:Rate Pitch, 12:Rate Yaw, 13:Mixer Roll, 14:Mixer Pitch, 15:Mixer Yaw, 16:Mixer Thrust
    AP_GROUPINFO("_AXIS", 1, Copter::ModeSystemId, systemID_axis, 0),

    // @Param: _MAG
    // @DisplayName: Chirp Magnitude
    // @Description: Magnitude of sweep in deg, deg/s and 0-1 for mixer outputs.
    // @User: Standard
    AP_GROUPINFO("_MAG", 2, Copter::ModeSystemId, magnitude, 15),

    // @Param: _F_START_HZ
    // @DisplayName: Start Frequency
    // @Description: Frequency at the start of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_START_HZ", 3, Copter::ModeSystemId, fStart, 0.5f),

    // @Param: _F_STOP_HZ
    // @DisplayName: Stop Frequency
    // @Description: Frequency at the end of the sweep
    // @Range: 0.01 100
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("_F_STOP_HZ", 4, Copter::ModeSystemId, fStop, 40),

    // @Param: _T_FADE_IN
    // @DisplayName: Fade in time
    // @Description: Time to reach maximum amplitude of sweep
    // @Range: 0 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_IN", 5, Copter::ModeSystemId, tFadeIn, 15),

    // @Param: _T_REC
    // @DisplayName: Total Sweep length
    // @Description: Time taken to complete the sweep
    // @Range: 0 255
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_REC", 6, Copter::ModeSystemId, tRec, 70),

    // @Param: _T_FADE_OUT
    // @DisplayName: Fade out time
    // @Description: Time to reach zero amplitude at the end of the sweep
    // @Range: 0 5
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("_T_FADE_OUT", 7, Copter::ModeSystemId, tFadeOut, 2),

    AP_GROUPEND
};

Copter::ModeSystemId::ModeSystemId(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#define SYSTEM_ID_DELAY     1.0f      // speed below which it is always safe to switch to loiter

// systemId_init - initialise systemId controller
bool Copter::ModeSystemId::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }

    orig_bf_feedforward = attitude_control->get_bf_feedforward();
    waveformTime = 0.0f;
    magnitude_scale = 0.0f;
    tConst = 2.0f / fStart; // Two full cycles at the starting frequency
    systemIDState = SystemID_Testing;

    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Starting: axis=%d", (unsigned)systemID_axis);

    AP::logger().Write(
        "SID",
        "TimeUS,Ax,FSt,FSp,TFin,TC,TR,TFout",
        "s--ssssss",
        "F--------",
        "Qdfffffff",
        AP_HAL::micros64(),
        (uint8_t)systemID_axis,
        (double)magnitude,
        (double)fStart,
        (double)fStop,
        (double)tFadeIn,
        (double)tConst,
        (double)tRec,
        (double)tFadeOut);

    return true;
}

// systemId_run - runs the systemId controller
// should be called at 100hz or more
void Copter::ModeSystemId::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    waveformTime += G_Dt;
    waveformSample = waveform(waveformTime - SYSTEM_ID_DELAY);

    switch (systemIDState) {
        case SystemID_Stopped:
            break;
        case SystemID_Testing:
            attitude_control->bf_feedforward(orig_bf_feedforward);

            if(copter.ap.land_complete) {
                systemIDState = SystemID_Stopped;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: Landed");
                break;
            }
            if(attitude_control->lean_angle()*100 > attitude_control->lean_angle_max()) {
                systemIDState = SystemID_Stopped;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: lean=%f max=%f", (double)attitude_control->lean_angle(), (double)attitude_control->lean_angle_max());
                break;
            }
            if(waveformTime > SYSTEM_ID_DELAY + tFadeIn + tConst + tRec + tFadeOut) {
                systemIDState = SystemID_Stopped;
                gcs().send_text(MAV_SEVERITY_INFO, "SystemID Finished");
                break;
            }

            switch (systemID_axis) {
                case NONE:
                    systemIDState = SystemID_Stopped;
                    gcs().send_text(MAV_SEVERITY_INFO, "SystemID Stopped: axis = 0");
                    break;
                case INPUT_ROLL:
                    target_roll += waveformSample*100.0f;
                    break;
                case INPUT_PITCH:
                    target_pitch += waveformSample*100.0f;
                    break;
                case INPUT_YAW:
                    target_yaw_rate += waveformSample*100.0f;
                    break;
                case RECOVER_ROLL:
                    target_roll += waveformSample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case RECOVER_PITCH:
                    target_pitch += waveformSample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case RECOVER_YAW:
                    target_yaw_rate += waveformSample*100.0f;
                    attitude_control->bf_feedforward(false);
                    break;
                case RATE_ROLL:
                    attitude_control->rate_bf_roll_sysid(radians(waveformSample));
                    break;
                case RATE_PITCH:
                    attitude_control->rate_bf_pitch_sysid(radians(waveformSample));
                    break;
                case RATE_YAW:
                    attitude_control->rate_bf_yaw_sysid(radians(waveformSample));
                    break;
                case MIX_ROLL:
                    attitude_control->actuator_roll_sysid(waveformSample);
                    break;
                case MIX_PITCH:
                    attitude_control->actuator_pitch_sysid(waveformSample);
                    break;
                case MIX_YAW:
                    attitude_control->actuator_yaw_sysid(waveformSample);
                    break;
                case MIX_THROTTLE:
                    pilot_throttle_scaled += waveformSample;
                    break;
            }
            break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    log_data();
}

// init_test - initialises the test
void Copter::ModeSystemId::log_data()
{
    int8_t index = copter.EKF2.getPrimaryCoreIMUIndex();
    if(index < 0){
        index = 0;
    }

    Vector3f delta_angle;
    copter.ins.get_delta_angle(index, delta_angle);
    float delta_angle_dt = copter.ins.get_delta_angle_dt(index);

    Vector3f delta_velocity;
    copter.ins.get_delta_velocity(index, delta_velocity);
    float delta_velocity_dt = copter.ins.get_delta_velocity_dt(index);

    AP::logger().Write(
        "SIDI",
        "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az",
        "ss-zkkkooo",
        "F---------",
        "Qfffffffff",
        AP_HAL::micros64(),
        (double)waveformTime,
        (double)waveformSample,
        (double)waveformFreqRads / (2 * M_PI),
        (double)(degrees(delta_angle.x / delta_angle_dt)),
        (double)(degrees(delta_angle.y / delta_angle_dt)),
        (double)(degrees(delta_angle.z / delta_angle_dt)),
        (double)(delta_velocity.x / delta_velocity_dt),
        (double)(delta_velocity.y / delta_velocity_dt),
        (double)(delta_velocity.z / delta_velocity_dt));

    // Full rate logging of attitude, rate and pid loops
    copter.Log_Write_Attitude();
}

// init_test - initialises the test
float Copter::ModeSystemId::waveform(float time)
{
    float wMin = 2 * M_PI * fStart;
    float wMax = 2 * M_PI * fStop;

    float window;
    float output;

    float B = logf(wMax / wMin);

    if(time <= 0.0f) {
        window = 0.0f;
    } else if (time <= tFadeIn) {
        window = 0.5 - 0.5 * cosf(M_PI * time / tFadeIn);
    } else if (time <= tRec - tFadeOut) {
        window = 1.0;
    } else if (time <= tRec) {
        window = 0.5 - 0.5 * cosf(M_PI * (time - (tRec - tFadeOut)) / tFadeOut + M_PI);
    } else {
        window = 0.0;
    }

    if(time <= 0.0f) {
        waveformFreqRads = wMin;
        output = 0.0f;
    } else if (time <= tConst) {
        waveformFreqRads = wMin;
        output = window * magnitude_scale * magnitude * sinf(wMin * time - wMin * tConst);
    } else if (time <= tRec) {
        waveformFreqRads = wMin * expf(B * (time - tConst) / (tRec - tConst));
        output = window * magnitude_scale * magnitude * sinf((wMin * (tRec - tConst) / B) * (expf(B * (time - tConst) / (tRec - tConst)) - 1));
    } else {
        waveformFreqRads = wMax;
        output = 0.0f;
    }
    return output;
}

