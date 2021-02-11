#define ALLOW_DOUBLE_MATH_FUNCTIONS

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

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InternalError/AP_InternalError.h>
#include "scurves.h"

extern const AP_HAL::HAL &hal;

#define SPEED_XY_MIN        50.0f   // minimum horizontal speed in cm/s
#define SPEED_Z_MIN         20.0f   // minimum vertical speed in cm/s
#define ACCEL_XY_MIN        50.0f   // minimum horizontal acceleration in cm/s/s
#define ACCEL_Z_MIN         20.0f   // minimum vertical acceleration in cm/s/s

#define SEG_INIT        0
#define SEG_ACCEL_MAX   4
#define SEG_ACCEL_END   7
#define SEG_CHANGE_END  14
#define SEG_CONST       15
#define SEG_DECEL_END   22

// constructor
scurves::scurves()
{
    otj = 0.0;
    jerk_max = 0.0;
    accel_max = 0.0;
    vel_max = 0.0;
    _t = 0.0;
    num_segs = SEG_INIT;
}

scurves::scurves(float tj, float Jm, float Am, float Vm) :
    otj(tj), jerk_max(Jm), accel_max(Am), vel_max(Vm)
{
    _t = 0.0;
    num_segs = SEG_INIT;
}

// initialise the S-Curve path
void scurves::init()
{
    _t = 0.0f;
    num_segs = SEG_INIT;
    add_segment(num_segs, 0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, 0.0f, 0.0f);
    _track.zero();
    _delta_unit.zero();
}

// set maximum velocity and re-calculate the path using these limits
void scurves::set_speed_accel(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                              float accel_xy_cmss, float accel_z_cmss)
{
    // segment accelerations can not be changed after segment creation.
    float track_speed_max = kinematic_limit(_delta_unit, speed_xy_cms, speed_up_cms, fabsf(speed_down_cms));

    if (is_equal(vel_max, track_speed_max)) {
        // New speed is equal to current speed maximum
        return;
    }

    if (is_zero(vel_max) || is_zero(track_speed_max)) {
        // New or original speeds are set to zero
        return;
    }
    set_speed_max(track_speed_max);

    // Check path has been defined
    if (num_segs != segments_max) {
        return;
    }

    if (_t >= segment[SEG_CONST].end_time) {
        return;
    }

    // re-calculate the s-curve path based on update speeds

    float Pend = segment[SEG_DECEL_END].end_pos;
    float Vend = MIN(vel_max, segment[SEG_DECEL_END].end_vel);
    uint16_t seg;

    if (is_zero(_t)) {
        // Path has not started to we can recompute the path
        float Vstart = MIN(vel_max, segment[SEG_INIT].end_vel);
        num_segs = SEG_INIT;
        add_segment(num_segs, 0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, 0.0f, 0.0f);
        add_segments(Pend);
        set_origin_speed_max(Vstart);
        set_destination_speed_max(Vend);
        return;
    }

    if (_t >= segment[SEG_ACCEL_END].end_time && _t <= segment[SEG_CHANGE_END].end_time) {
        // In the change speed phase
        // Move adjust phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        segment[SEG_INIT].jtype = jtype_t::CONSTANT;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_ACCEL_END].end_time;
        segment[SEG_INIT].end_accel = segment[SEG_ACCEL_END].end_accel;
        segment[SEG_INIT].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_ACCEL_END].end_pos;

        // move change segments to acceleration segments
        for (uint8_t i = SEG_INIT+1; i <= SEG_ACCEL_END; i++) {
            segment[i].jtype = segment[i+7].jtype;
            segment[i].jerk_ref = segment[i+7].jerk_ref;
            segment[i].end_time = segment[i+7].end_time;
            segment[i].end_accel = segment[i+7].end_accel;
            segment[i].end_vel = segment[i+7].end_vel;
            segment[i].end_pos = segment[i+7].end_pos;
        }

        // set change segments to last acceleration speed
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
            segment[i].jtype = jtype_t::CONSTANT;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

    } else if (_t >= segment[SEG_CHANGE_END].end_time && _t <= segment[SEG_CONST].end_time) {
        // In the constant speed phase
        // Move adjust phase to acceleration phase to provide room for further speed adjustments

        // set initial segment to last acceleration segment
        segment[SEG_INIT].jtype = jtype_t::CONSTANT;
        segment[SEG_INIT].jerk_ref = 0.0f;
        segment[SEG_INIT].end_time = segment[SEG_CHANGE_END].end_time;
        segment[SEG_INIT].end_accel = 0.0f;
        segment[SEG_INIT].end_vel = segment[SEG_CHANGE_END].end_vel;
        segment[SEG_INIT].end_pos = segment[SEG_CHANGE_END].end_pos;

        // set acceleration and change segments to current constant speed
        float Jt_out, At_out, Vt_out, Pt_out;
        update(_t, Jt_out, At_out, Vt_out, Pt_out);
        for (uint8_t i = SEG_INIT+1; i <= SEG_CHANGE_END; i++) {
            segment[i].jtype = jtype_t::CONSTANT;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = _t;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = Vt_out;
            segment[i].end_pos = Pt_out;
        }
    }

    // Adjust the INIT and ACCEL segments for new speed
    if ((_t <= segment[SEG_ACCEL_MAX].end_time) && is_positive(segment[SEG_ACCEL_MAX].end_time - segment[SEG_ACCEL_MAX-1].end_time) && (vel_max < segment[SEG_ACCEL_END].end_vel) && is_positive(segment[SEG_ACCEL_MAX].end_accel) ) {
        // Path has not finished constant positive acceleration segment
        // Reduce velocity as close to target velocity as possible

        float Vstart = segment[SEG_INIT].end_vel;

        // minimum velocity that can be obtained by shortening SEG_ACCEL_MAX
        float Vmin = segment[SEG_ACCEL_END].end_vel - segment[SEG_ACCEL_MAX].end_accel * (segment[SEG_ACCEL_MAX].end_time - MAX(_t, segment[SEG_ACCEL_MAX-1].end_time));

        seg = SEG_INIT+1;

        float Jm, t2, t4, t6;
        calculate_path(otj, jerk_max, Vstart, accel_max, MAX(Vmin, vel_max), Pend / 2.0, Jm, t2, t4, t6);

        add_segments_jerk(seg, otj, Jm, t2);
        add_segment_const_jerk(seg, t4, 0.0);
        add_segments_jerk(seg, otj, -Jm, t6);

        // add empty speed adjust segments
        for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CONST; i++) {
            segment[i].jtype = jtype_t::CONSTANT;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_ACCEL_END].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
            segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
        }

        calculate_path(otj, jerk_max, 0.0f, accel_max, MAX(Vmin, vel_max), Pend / 2.0, Jm, t2, t4, t6);

        seg = SEG_CONST +1;
        add_segments_jerk(seg, otj, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0);
        add_segments_jerk(seg, otj, Jm, t2);

        // add to constant velocity segment to end at the correct position
        float dP = (Pend - segment[SEG_DECEL_END].end_pos);
        float t15 =  dP / segment[SEG_CONST].end_vel;
        for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
            segment[i].end_time += t15;
            segment[i].end_pos += dP;
        }
    }

    // Adjust the CHANGE segments for new speed
    // start with empty speed adjust segments
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
        segment[i].jtype = jtype_t::CONSTANT;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }
    if (!is_equal(vel_max, segment[SEG_ACCEL_END].end_vel)) {
        // add velocity adjustment
        // check there is enough time to make velocity change
        // we use the approximation that the time will be distance/max_vel and 8 jerk segments
        float L = segment[SEG_CONST].end_pos - segment[SEG_ACCEL_END].end_pos;
        float Jm = 0;
        float t2 = 0;
        float t4 = 0;
        float t6 = 0;
        if ((vel_max < segment[SEG_ACCEL_END].end_vel) && (otj*12.0f < L/segment[SEG_ACCEL_END].end_vel)) {
            // we have a problem here with small segments.
            calculate_path(otj, jerk_max, vel_max, accel_max, segment[SEG_ACCEL_END].end_vel, L / 2.0, Jm, t6, t4, t2);
            Jm = -Jm;

        } else if ((vel_max > segment[SEG_ACCEL_END].end_vel) && (L/(otj*12.0f) > segment[SEG_ACCEL_END].end_vel)) {
            float Vm = MIN(vel_max, L/(otj*12.0f));
            calculate_path(otj, jerk_max, segment[SEG_ACCEL_END].end_vel, accel_max, Vm, L / 2.0, Jm, t2, t4, t6);
        }

        seg = SEG_ACCEL_END + 1;
        if (!is_zero(Jm) && !is_negative(t2) && !is_negative(t4) && !is_negative(t6)) {
            add_segments_jerk(seg, otj, Jm, t2);
            add_segment_const_jerk(seg, t4, 0.0);
            add_segments_jerk(seg, otj, -Jm, t6);
        }
    }

    // add deceleration segments
    // Earlier check should ensure that we should always have sufficient time to stop
    seg = SEG_CONST;
    Vend = MIN(Vend, segment[SEG_CHANGE_END].end_vel);
    add_segment_const_jerk(seg, 0.0, 0.0);
    if (Vend < segment[SEG_CHANGE_END].end_vel) {
        float Jm, t2, t4, t6;
        calculate_path(otj, jerk_max, Vend, accel_max, segment[SEG_CONST].end_vel, Pend - segment[SEG_CONST].end_pos, Jm, t2, t4, t6);
        add_segments_jerk(seg, otj, -Jm, t6);
        add_segment_const_jerk(seg, t4, 0.0);
        add_segments_jerk(seg, otj, Jm, t2);
    } else {
        // No deceleration is required
        for (uint8_t i = SEG_CONST+1; i <= SEG_DECEL_END; i++) {
            segment[i].jtype = jtype_t::CONSTANT;
            segment[i].jerk_ref = 0.0f;
            segment[i].end_time = segment[SEG_CONST].end_time;
            segment[i].end_accel = 0.0f;
            segment[i].end_vel = segment[SEG_CONST].end_vel;
            segment[i].end_pos = segment[SEG_CONST].end_pos;
        }
    }

    // add to constant velocity segment to end at the correct position
    float dP = (Pend - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
}

// generate a trigonometric S-Curve path in 3D space that moves over a straight line
// between two points defined by the origin and destination.
void scurves::calculate_leg(const Vector3f &origin, const Vector3f &destination,
                   float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                   float accel_xy_cmss, float accel_z_cmss)
{
    init();

    // update speed and acceleration limits along path
    set_kinematic_limits(origin, destination,
                         speed_xy_cms, speed_up_cms, speed_down_cms,
                         accel_xy_cmss, accel_z_cmss);

    // avoid divide-by zeros. Path will be left as a zero length path
    if (!is_positive(otj) || !is_positive(jerk_max) || !is_positive(accel_max) || !is_positive(vel_max)) {
        return;
    }

    _track = destination - origin;
    float track_length = _track.length();
    if (is_zero(track_length)) {
        // avoid possible divide by zero
        _delta_unit.zero();
    } else {
        _delta_unit = _track.normalized();
        add_segments(track_length);
    }
}

// set the maximum vehicle speed at the origin
// returns the expected speed at the origin (will always be equal or lower to speed_cm)
float scurves::set_origin_speed_max(float speed_cms)
{
    // if path is zero length then start speed must be zero
    if (num_segs != segments_max) {
        return 0.0f;
    }

    // check speed is zero or positive
    // avoid re-calculating if unnecessary
    if (is_equal(segment[SEG_INIT].end_vel, speed_cms)) {
        return speed_cms;
    }

    float Vm = segment[SEG_ACCEL_END].end_vel;
    float L = segment[SEG_DECEL_END].end_pos;
    speed_cms = MIN(speed_cms, Vm);

    float Jm, t2, t4, t6;
    calculate_path(otj, jerk_max, speed_cms, accel_max, Vm, L / 2.0, Jm, t2, t4, t6);

    uint16_t seg = SEG_INIT;
    add_segment(seg, 0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, speed_cms, 0.0f);
    add_segments_jerk(seg, otj, Jm, t2);
    add_segment_const_jerk(seg, t4, 0.0);
    add_segments_jerk(seg, otj, -Jm, t6);

    // add empty speed change segments and constant speed segment
    for (uint8_t i = SEG_ACCEL_END+1; i <= SEG_CHANGE_END; i++) {
        segment[i].jtype = jtype_t::CONSTANT;
        segment[i].jerk_ref = 0.0f;
        segment[i].end_time = segment[SEG_ACCEL_END].end_time;
        segment[i].end_accel = 0.0f;
        segment[i].end_vel = segment[SEG_ACCEL_END].end_vel;
        segment[i].end_pos = segment[SEG_ACCEL_END].end_pos;
    }

    calculate_path(otj, jerk_max, 0.0f, accel_max, Vm, L - segment[SEG_CONST].end_pos, Jm, t2, t4, t6);

    seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0, 0.0);

    add_segments_jerk(seg, otj, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0);
    add_segments_jerk(seg, otj, Jm, t2);

    // add to constant velocity segment to end at the correct position
    float dP = (L - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
    return speed_cms;
}

// set the maximum vehicle speed at the destination
void scurves::set_destination_speed_max(float speed_cms)
{
    // if path is zero length then start speed must be zero
    if (num_segs != segments_max) {
        return;
    }

    // avoid re-calculating if unnecessary
    if (is_equal(segment[segments_max-1].end_vel, speed_cms)) {
        return;
    }

    float Vm = segment[SEG_CONST].end_vel;
    float L = segment[SEG_DECEL_END].end_pos;
    speed_cms = MIN(speed_cms, Vm);

    float Jm, t2, t4, t6;
    calculate_path(otj, jerk_max, speed_cms, accel_max, Vm, L / 2.0, Jm, t2, t4, t6);

    uint16_t seg = SEG_CONST;
    add_segment_const_jerk(seg, 0.0, 0.0);

    add_segments_jerk(seg, otj, -Jm, t6);
    add_segment_const_jerk(seg, t4, 0.0);
    add_segments_jerk(seg, otj, Jm, t2);

    // add to constant velocity segment to end at the correct position
    float dP = (L - segment[SEG_DECEL_END].end_pos);
    float t15 =  dP / segment[SEG_CONST].end_vel;
    for (uint8_t i = SEG_CONST; i <= SEG_DECEL_END; i++) {
        segment[i].end_time += t15;
        segment[i].end_pos += dP;
    }
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
void scurves::move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(_t, pos, vel, accel);
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the destination
void scurves::move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(_t, pos, vel, accel);
    pos -= _track;
}

// return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
void scurves::move_from_time_pos_vel_accel(float time, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    float scurve_P1 = 0.0f;
    float scurve_V1, scurve_A1, scurve_J1;
    update(time, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit * scurve_P1;
    vel += _delta_unit * scurve_V1;
    accel += _delta_unit * scurve_A1;
}

// time has reached the end of the sequence
bool scurves::finished() const
{
    if (num_segs != segments_max) {
        return true;
    }
    return _t > time_end();
}

// magnitude of the position vector at the end of the sequence
float scurves::pos_end() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_pos;
}

// time at the end of the sequence
float scurves::time_end() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_time;
}

// time left before sequence will complete
float scurves::get_time_remaining() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_DECEL_END].end_time - _t;
}

// time when acceleration section of the sequence will complete
float scurves::get_accel_finished_time() const
{
    if (num_segs != segments_max) {
        return 0.0f;
    }
    return segment[SEG_ACCEL_END].end_time;
}

// return true if the sequence is braking to a stop
bool scurves::braking() const
{
    if (num_segs != segments_max) {
        return true;
    }
    return _t >= segment[SEG_CONST].end_time;
}

// calculate the jerk, acceleration, velocity and position at the provided time
void scurves::update(float time, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out)
{
    jtype_t Jtype;
    int8_t pnt = num_segs;
    float tj;
    float Jm, T0, A0, V0, P0;

    for (uint8_t i = 0; i < num_segs; i++) {
        if (time < segment[num_segs - 1 - i].end_time) {
            pnt = num_segs - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = jtype_t::CONSTANT;
        Jm = 0.0f;
        T0 = segment[pnt].end_time;
        A0 = segment[pnt].end_accel;
        V0 = segment[pnt].end_vel;
        P0 = segment[pnt].end_pos;
    } else if (pnt == num_segs) {
        Jtype = jtype_t::CONSTANT;
        Jm = 0.0;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    } else {
        Jtype = segment[pnt].jtype;
        Jm = segment[pnt].jerk_ref;
        tj = segment[pnt].end_time - segment[pnt - 1].end_time;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    }

    switch (Jtype) {
    case jtype_t::CONSTANT:
        calc_javp_for_segment_const_jerk(time - T0, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::POSITIVE:
        calc_javp_for_segment_incr_jerk(time - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::NEGATIVE:
        calc_javp_for_segment_decr_jerk(time - T0, tj, Jm, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }
}

// calculate the jerk, acceleration, velocity and position at time t when running the constant jerk time segment
void scurves::calc_javp_for_segment_const_jerk(float time, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    Jt = J0;
    At = A0 + J0 * time;
    Vt = V0 + A0 * time + 0.5 * J0 * (time * time);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (1 / 6.0) * J0 * (time * time * time);
}

// Calculate the jerk, acceleration, velocity and position at time t when running the increasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_incr_jerk(float time, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jm / 2.0;
    float Beta = M_PI / tj;
    Jt = Alpha * (1.0 - cosf(Beta * time));
    At = A0 + Alpha * time - (Alpha / Beta) * sinf(Beta * time);
    Vt = V0 + A0 * time + (Alpha / 2.0) * (time * time) + (Alpha / (Beta * Beta)) * cosf(Beta * time) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (-Alpha / (Beta * Beta)) * time + Alpha * (time * time * time) / 6.0 + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time);
}

// Calculate the jerk, acceleration, velocity and position at time t when running the decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_decr_jerk(float time, float tj, float Jm, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jm / 2.0;
    float Beta = M_PI / tj;
    float AT = Alpha * tj;
    float VT = Alpha * ((tj * tj) / 2.0 - 2.0 / (Beta * Beta));
    float PT = Alpha * ((-1.0 / (Beta * Beta)) * tj + (1 / 6.0) * (tj * tj * tj));
    Jt = Alpha * (1.0 - cosf(Beta * (time + tj)));
    At = (A0 - AT) + Alpha * (time + tj) - (Alpha / Beta) * sinf(Beta * (time + tj));
    Vt = (V0 - VT) + (A0 - AT) * time + 0.5 * Alpha * (time + tj) * (time + tj) + (Alpha / (Beta * Beta)) * cosf(Beta * (time + tj)) - Alpha / (Beta * Beta);
    Pt = (P0 - PT) + (V0 - VT) * time + 0.5 * (A0 - AT) * (time * time) + (-Alpha / (Beta * Beta)) * (time + tj) + (Alpha / 6.0) * (time + tj) * (time + tj) * (time + tj) + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * (time + tj));
}

// debugging messages
void scurves::debug()
{
    hal.console->printf("\n");
    hal.console->printf("num_segs:%u, _t:%4.2f, otj:%4.2f, jerk_max:%4.2f, accel_max:%4.2f, vel_max:%4.2f\n",
                        (unsigned)num_segs, (double)_t, (double)otj, (double)jerk_max, (double)accel_max, (double)vel_max);
    hal.console->printf("T, Jt, J, A, V, P \n");
    for (uint8_t i = 0; i < num_segs; i++) {
        hal.console->printf("i:%u, T:%4.2f, Jtype:%4.2f, J:%4.2f, A:%4.2f, V: %4.2f, P: %4.2f\n",
                            (unsigned)i, (double)segment[i].end_time, (double)segment[i].jtype, (double)segment[i].jerk_ref,
                            (double)segment[i].end_accel, (double)segment[i].end_vel, (double)segment[i].end_pos);
    }
    hal.console->printf("_track x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_track.x, (double)_track.y, (double)_track.z);
    hal.console->printf("_delta_unit x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_delta_unit.x, (double)_delta_unit.y, (double)_delta_unit.z);
    hal.console->printf("\n");
}

// generate the segments for a path of length L
// the path consists of 23 segments
// 1 initial segment
// 7 segments forming the acceleration S-Curve
// 7 segments forming the velocity change S-Curve
// 1 constant velocity S-Curve
// 7 segments forming the deceleration S-Curve
void scurves::add_segments(float L)
{
    if (is_zero(L)) {
        return;
    }

    float Jm, t2, t4, t6;
    calculate_path(otj, jerk_max, 0.0f, accel_max, vel_max, L / 2.0, Jm, t2, t4, t6);

    add_segments_jerk(num_segs, otj, Jm, t2);
    add_segment_const_jerk(num_segs, t4, 0.0);
    add_segments_jerk(num_segs, otj, -Jm, t6);

    // add empty speed adjust segments
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);
    add_segment_const_jerk(num_segs, 0.0, 0.0);

    float t15 = 2.0 * (L / 2.0 - segment[SEG_CHANGE_END].end_pos) / segment[SEG_CHANGE_END].end_vel;
    add_segment_const_jerk(num_segs, t15, 0.0);

    add_segments_jerk(num_segs, otj, -Jm, t6);
    add_segment_const_jerk(num_segs, t4, 0.0);
    add_segments_jerk(num_segs, otj, Jm, t2);
}

// calculate the segment times for the trigonometric S-Curve path defined by:
// tj - duration of the raised cosine jerk profile
// Jm - maximum value of the raised cosine jerk profile
// V0 - initial velocity magnitude
// Am - maximum constant acceleration
// Vm - maximum constant velocity
// L - Length of the path
void scurves::calculate_path(float tj, float Jm, float V0, float Am, float Vm, float L, float &Jm_out, float &t2_out, float &t4_out, float &t6_out) const
{
    // init outputs
    Jm_out = 0.0f;
    t2_out = 0.0f;
    t4_out = 0.0f;
    t6_out = 0.0f;

    // check for invalid arguments
    if (!is_positive(tj) || !is_positive(Jm) || !is_positive(Am) || !is_positive(Vm) || !is_positive(L)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arguments);
        return;
    }

    if (V0 >= Vm) {
        // no velocity change so all segments as zero length
        return;
    }

    Am = MIN(MIN(Am, (Vm - V0) / (2.0 * tj)), (L + 4.0 * V0 * tj) / (4.0 * sq(tj)));
    if (fabsf(Am) < Jm * tj) {
        Jm = Am / tj;
        if ((Vm <= V0 + 2.0 * Am * tj) || (L <= 4.0 * V0 * tj + 4.0 * Am * sq(tj))) {
            // solution = 0 - t6 t4 t2 = 0 0 0
            t2_out = 0.0;
            t4_out = 0.0;
            t6_out = 0.0;
        } else {
            // solution = 2 - t6 t4 t2 = 0 1 0
            t2_out = 0.0;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0 / 2.0) + safe_sqrt((Am * Am * Am * Am) * (1.0 / 4.0) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0 / 4.0) + Am * (Jm * Jm) * L * 2.0 - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0 / 2.0) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0 / 2.0)) / (Am * Jm), ((Am * Am) * (-3.0 / 2.0) - safe_sqrt((Am * Am * Am * Am) * (1.0 / 4.0) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0 / 4.0) + Am * (Jm * Jm) * L * 2.0 - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0 / 2.0) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0 / 2.0)) / (Am * Jm)));
            t6_out = 0.0;
        }
    } else {
        if ((Vm < V0 + Am * tj + (Am * Am) / Jm) || (L < 1.0 / (Jm * Jm) * (Am * Am * Am + Am * Jm * (V0 * 2.0 + Am * tj * 2.0)) + V0 * tj * 2.0 + Am * (tj * tj))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Am = MIN(MIN(Am, MAX(Jm * (tj + safe_sqrt((V0 * -4.0 + Vm * 4.0 + Jm * (tj * tj)) / Jm)) * (-1.0 / 2.0), Jm * (tj - safe_sqrt((V0 * -4.0 + Vm * 4.0 + Jm * (tj * tj)) / Jm)) * (-1.0 / 2.0))), Jm * tj * (-2.0 / 3.0) + ((Jm * Jm) * (tj * tj) * (1.0 / 9.0) - Jm * V0 * (2.0 / 3.0)) * 1.0 / powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0 / 2.0) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0 / 2.7E1) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0) * (1.0 / 3.0) + (Jm * Jm) * V0 * tj, 2.0) - powf((Jm * Jm) * (tj * tj) * (1.0 / 9.0) - Jm * V0 * (2.0 / 3.0), 3.0)) + (Jm * Jm) * L * (1.0 / 2.0) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0 / 2.7E1) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0) * (1.0 / 3.0) - (Jm * Jm) * V0 * tj, 1.0 / 3.0) + powf(safe_sqrt(powf(- (Jm * Jm) * L * (1.0 / 2.0) + (Jm * Jm * Jm) * (tj * tj * tj) * (8.0 / 2.7E1) - Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0) * (1.0 / 3.0) + (Jm * Jm) * V0 * tj, 2.0) - powf((Jm * Jm) * (tj * tj) * (1.0 / 9.0) - Jm * V0 * (2.0 / 3.0), 3.0)) + (Jm * Jm) * L * (1.0 / 2.0) - (Jm * Jm * Jm) * (tj * tj * tj) * (8.0 / 2.7E1) + Jm * tj * ((Jm * Jm) * (tj * tj) + Jm * V0 * 2.0) * (1.0 / 3.0) - (Jm * Jm) * V0 * tj, 1.0 / 3.0));
            t2_out = Am / Jm - tj;
            t4_out = 0;
            t6_out = t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Am / Jm - tj;
            t4_out = MIN(-(V0 - Vm + Am * tj + (Am * Am) / Jm) / Am, MAX(((Am * Am) * (-3.0 / 2.0) + safe_sqrt((Am * Am * Am * Am) * (1.0 / 4.0) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0 / 4.0) + Am * (Jm * Jm) * L * 2.0 - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0 / 2.0) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0 / 2.0)) / (Am * Jm), ((Am * Am) * (-3.0 / 2.0) - safe_sqrt((Am * Am * Am * Am) * (1.0 / 4.0) + (Jm * Jm) * (V0 * V0) + (Am * Am) * (Jm * Jm) * (tj * tj) * (1.0 / 4.0) + Am * (Jm * Jm) * L * 2.0 - (Am * Am) * Jm * V0 + (Am * Am * Am) * Jm * tj * (1.0 / 2.0) - Am * (Jm * Jm) * V0 * tj) - Jm * V0 - Am * Jm * tj * (3.0 / 2.0)) / (Am * Jm)));
            t6_out = t2_out;
        }
    }
    Jm_out = Jm;
}

// generate three time segments forming the jerk profile
void scurves::add_segments_jerk(uint16_t &seg_pnt, float tj, float Jm, float Tcj)
{
    add_segment_incr_jerk(seg_pnt, tj, Jm);
    add_segment_const_jerk(seg_pnt, Tcj, Jm);
    add_segment_decr_jerk(seg_pnt, tj, Jm);
}

// generate constant jerk time segment
void scurves::add_segment_const_jerk(uint16_t &seg_pnt, float tin, float J0)
{
    jtype_t Jtype = jtype_t::CONSTANT;
    float J = J0;
    float T = segment[seg_pnt - 1].end_time + tin;
    float A = segment[seg_pnt - 1].end_accel + J0 * tin;
    float V = segment[seg_pnt - 1].end_vel + segment[seg_pnt - 1].end_accel * tin + 0.5 * J0 * sq(tin);
    float P = segment[seg_pnt - 1].end_pos + segment[seg_pnt - 1].end_vel * tin + 0.5 * segment[seg_pnt - 1].end_accel * sq(tin) + (1 / 6.0) * J0 * powf(tin, 3.0);
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_incr_jerk(uint16_t &seg_pnt, float tj, float Jm)
{
    float Beta = M_PI / tj;
    float Alpha = Jm / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));

    jtype_t Jtype = jtype_t::POSITIVE;
    float J = Jm;
    float T = segment[seg_pnt - 1].end_time + tj;
    float A = segment[seg_pnt - 1].end_accel + AT;
    float V = segment[seg_pnt - 1].end_vel + segment[seg_pnt - 1].end_accel * tj + VT;
    float P = segment[seg_pnt - 1].end_pos + segment[seg_pnt - 1].end_vel * tj + 0.5 * segment[seg_pnt - 1].end_accel * sq(tj) + PT;
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

// generate decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_decr_jerk(uint16_t &seg_pnt, float tj, float Jm)
{
    float Beta = M_PI / tj;
    float Alpha = Jm / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));
    float A2T = Jm * tj;
    float V2T = Jm * sq(tj);
    float P2T = Alpha * ((-1.0 / sq(Beta)) * 2.0 * tj + (4.0 / 3.0) * powf(tj, 3.0));

    jtype_t Jtype = jtype_t::NEGATIVE;
    float J = Jm;
    float T = segment[seg_pnt - 1].end_time + tj;
    float A = (segment[seg_pnt - 1].end_accel - AT) + A2T;
    float V = (segment[seg_pnt - 1].end_vel - VT) + (segment[seg_pnt - 1].end_accel - AT) * tj + V2T;
    float P = (segment[seg_pnt - 1].end_pos - PT) + (segment[seg_pnt - 1].end_vel - VT) * tj + 0.5 * (segment[seg_pnt - 1].end_accel - AT) * sq(tj) + P2T;
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

// add single S-Curve segment
void scurves::add_segment(uint16_t &seg_pnt, float end_time, jtype_t jtype, float jerk_ref, float end_accel, float end_vel, float end_pos)
{
    segment[seg_pnt].end_time = end_time;
    segment[seg_pnt].jtype = jtype;
    segment[seg_pnt].jerk_ref = jerk_ref;
    segment[seg_pnt].end_accel = end_accel;
    segment[seg_pnt].end_vel = end_vel;
    segment[seg_pnt].end_pos = end_pos;
    seg_pnt++;
}

// set speed and acceleration limits for the path
// origin and destination are offsets from EKF origin
// speed and acceleration parameters are given in horizontal, up and down.
void scurves::set_kinematic_limits(const Vector3f &origin, const Vector3f &destination,
                                   float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                                   float accel_xy_cmss, float accel_z_cmss)
{
    // sanity check arguments
    speed_xy_cms = MAX(speed_xy_cms, SPEED_XY_MIN);
    speed_up_cms = MAX(speed_up_cms, SPEED_Z_MIN);
    speed_down_cms = MAX(fabsf(speed_down_cms), SPEED_Z_MIN);
    accel_xy_cmss = MAX(accel_xy_cmss, ACCEL_XY_MIN);
    accel_z_cmss = MAX(accel_z_cmss, ACCEL_Z_MIN);
    // ensure arguments are positive

    Vector3f direction = destination - origin;
    float track_speed_max = kinematic_limit(direction, speed_xy_cms, speed_up_cms, speed_down_cms);
    float track_accel_max = kinematic_limit(direction, accel_xy_cmss, accel_z_cmss, accel_z_cmss);

    set_speed_max(track_speed_max);
    set_accel_max(track_accel_max);
}
