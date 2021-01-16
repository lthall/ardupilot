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
#include "scurves.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

// constructor
scurves::scurves()
{
    otj = 0.0;
    jerk_max = 0.0;
    accel_max = 0.0;
    vel_max = 0.0;
    _t = 0.0;
    num_segs = 0;
}

scurves::scurves(float tj, float Jp, float Ap, float Vp) :
    otj(tj), jerk_max(Jp), accel_max(Ap), vel_max(Vp)
{
    _t = 0.0;
    num_segs = 0;
}

// initialise the S-curve track
bool scurves::init()
{
    _t = 0.0f;
    num_segs = 0;
    add_segment(num_segs, 0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, 0.0f, 0.0f);
    _track.zero();
    _delta_unit.zero();

    return is_positive(otj) && is_positive(jerk_max) && is_positive(accel_max) && is_positive(vel_max);
}

// generate an optimal jerk limited curve in 3D space that moves over a straight line between two points
void scurves::calculate_leg(const Vector3f &origin, const Vector3f &destination)
{
    if (!init()) {
        // Some parameters have been set to zero
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

// Change the starting velocity of the S-curve
float scurves::set_start_vel(float vel)
{
    float tj = otj;
    float Jp = jerk_max;
    float Ap = accel_max;
    float Vp = segment[7].end_vel;
    float Pp = segment[num_segs - 1].end_pos;
    vel = MIN(vel, Vp);

    float t2, t4, t6;
    cal_pos(tj, vel, Jp, Ap, Vp, Pp / 2.0, Jp, t2, t4, t6);

    uint16_t seg = 0;
    add_segment(seg, 0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, vel, 0.0f);
    add_segments_incr_const_decr_jerk(seg, tj, Jp, t2);
    add_segment_const_jerk(seg, t4, 0.0);
    add_segments_incr_const_decr_jerk(seg, tj, -Jp, t6);

    float t8 = (Pp / 2.0 - segment[seg - 1].end_pos) / segment[seg - 1].end_vel;
    float end_time = segment[seg].end_time;
    add_segment_const_jerk(seg, t8, 0.0);
    float delta_time = end_time - segment[seg-1].end_time;
    for (uint8_t i = seg; i < num_segs; i++) {
        segment[i].end_time -= delta_time;
    }
    return vel;
}

// Change the ending velocity of the S-curve
void scurves::set_end_vel(float vel)
{
    float tj = otj;
    float Jp = jerk_max;
    float Ap = accel_max;
    float Vp = segment[7].end_vel;
    float Pp = segment[num_segs - 1].end_pos;
    vel = MIN(vel, Vp);

    float t2, t4, t6;
    cal_pos(tj, vel, Jp, Ap, Vp, Pp / 2.0, Jp, t2, t4, t6);

    uint16_t seg = 9;
    add_segment_const_jerk(seg, 0.0, 0.0);

    add_segments_incr_const_decr_jerk(seg, tj, -Jp, t6);
    add_segment_const_jerk(seg, t4, 0.0);
    add_segments_incr_const_decr_jerk(seg, tj, Jp, t2);

    seg = 9;
    float dP = (Pp - segment[num_segs - 1].end_pos);
    float t8 =  dP / Vp;
    add_segment_const_jerk(seg, t8, 0.0);
    for (uint8_t i = seg; i < num_segs; i++) {
        segment[i].end_time += t8;
        segment[i].end_pos += dP;
    }
}

// Straight implementations

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
void scurves::move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(_t, pos, vel, accel);
}

// return the position, velocity and acceleration vectors relative to the origin
void scurves::move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(dt);
    move_from_time_pos_vel_accel(_t, pos, vel, accel);
    pos -= _track;
}

// return the position, velocity and acceleration vectors relative to the origin
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
    return _t > time_end();
}

// straight segment implementations of pos_end, time_end, time_to_end and braking
float scurves::pos_end() const
{
    return segment[num_segs - 1].end_pos;
}

float scurves::time_end() const
{
    return segment[num_segs - 1].end_time;
}

float scurves::get_time_remaining() const
{
    return segment[num_segs - 1].end_time - _t;
}

float scurves::get_accel_finished_time() const
{
    return segment[6].end_time;
}

bool scurves::braking() const
{
    return _t >= segment[8].end_time;
}

// calculate the jerk, acceleration, velocity and position at time t
void scurves::update(float time, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out)
{
    jtype_t Jtype;
    int8_t pnt = num_segs;
    float tj;
    float Jp, T0, A0, V0, P0;

    for (uint8_t i = 0; i < num_segs; i++) {
        if (time < segment[num_segs - 1 - i].end_time) {
            pnt = num_segs - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = jtype_t::CONSTANT;
        Jp = 0.0f;
        T0 = segment[pnt].end_time;
        A0 = segment[pnt].end_accel;
        V0 = segment[pnt].end_vel;
        P0 = segment[pnt].end_pos;
    } else if (pnt == num_segs) {
        Jtype = jtype_t::CONSTANT;
        Jp = 0.0;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    } else {
        Jtype = segment[pnt].jtype;
        Jp = segment[pnt].jerk_ref;
        tj = segment[pnt].end_time - segment[pnt - 1].end_time;
        T0 = segment[pnt - 1].end_time;
        A0 = segment[pnt - 1].end_accel;
        V0 = segment[pnt - 1].end_vel;
        P0 = segment[pnt - 1].end_pos;
    }

    switch (Jtype) {
    case jtype_t::CONSTANT:
        calc_javp_for_segment_const_jerk(time - T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::POSITIVE:
        calc_javp_for_segment_incr_jerk(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::NEGATIVE:
        calc_javp_for_segment_decr_jerk(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }
}

// calculate the jerk, acceleration, velocity and position at time "time" when running the constant jerk time segment
void scurves::calc_javp_for_segment_const_jerk(float time, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    Jt = J0;
    At = A0 + J0 * time;
    Vt = V0 + A0 * time + 0.5 * J0 * (time * time);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (1 / 6.0) * J0 * (time * time * time);
}

// Calculate the jerk, acceleration, velocity and position at time "time" when running the increasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_incr_jerk(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jp / 2.0;
    float Beta = M_PI / tj;
    Jt = Alpha * (1.0 - cosf(Beta * time));
    At = A0 + Alpha * time - (Alpha / Beta) * sinf(Beta * time);
    Vt = V0 + A0 * time + (Alpha / 2.0) * (time * time) + (Alpha / (Beta * Beta)) * cosf(Beta * time) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (-Alpha / (Beta * Beta)) * time + Alpha * (time * time * time) / 6.0 + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time);
}

// Calculate the jerk, acceleration, velocity and position at time "time" when running the  decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_decr_jerk(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jp / 2.0;
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

// generate time segments for straight segment
void scurves::add_segments(float Pp)
{
    if (is_zero(Pp)) {
        return;
    }

    float tj = otj;
    float Jp = jerk_max;
    float Ap = accel_max;
    float Vp = vel_max;

    float t2, t4, t6;
    cal_posfast(tj, Jp, Ap, Vp, Pp / 2.0, Jp, t2, t4, t6);

    add_segments_incr_const_decr_jerk(num_segs, tj, Jp, t2);
    add_segment_const_jerk(num_segs, t4, 0.0);
    add_segments_incr_const_decr_jerk(num_segs, tj, -Jp, t6);

    float t8 = (Pp / 2.0 - segment[num_segs - 1].end_pos) / segment[num_segs - 1].end_vel;
    add_segment_const_jerk(num_segs, t8, 0.0);
    add_segment_const_jerk(num_segs, t8, 0.0);

    add_segments_incr_const_decr_jerk(num_segs, tj, -Jp, t6);
    add_segment_const_jerk(num_segs, t4, 0.0);
    add_segments_incr_const_decr_jerk(num_segs, tj, Jp, t2);
}

// calculate duration of time segments for basic acceleration and deceleration curve from constant velocity to stationary.
void scurves::cal_pos(float tj, float V0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const
{
    Ap = MIN(MIN(Ap, (Vp - V0) / (2.0 * tj)), (Pp + 4.0 * V0 * tj) / (4.0 * sq(tj)));
    if (fabsf(Ap) < Jp * tj) {
        Jp = Ap / tj;
        if ((Vp <= V0 + 2.0 * Ap * tj) || (Pp <= 4.0 * V0 * tj + 4.0 * Ap * sq(tj))) {
            // solution = 0 - t6 t4 t2 = 0 0 0
            t2_out = 0.0;
            t4_out = 0.0;
            t6_out = 0.0;
        } else {
            // solution = 2 - t6 t4 t2 = 0 1 0
            t2_out = 0.0;
            t4_out = MIN(-(V0 - Vp + Ap * tj + (Ap * Ap) / Jp) / Ap, MAX(((Ap * Ap) * (-3.0 / 2.0) + safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp), ((Ap * Ap) * (-3.0 / 2.0) - safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp)));
            t6_out = 0.0;
        }
    } else {
        if ((Vp < V0 + Ap * tj + (Ap * Ap) / Jp) || (Pp < 1.0 / (Jp * Jp) * (Ap * Ap * Ap + Ap * Jp * (V0 * 2.0 + Ap * tj * 2.0)) + V0 * tj * 2.0 + Ap * (tj * tj))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Ap = MIN(MIN(Ap, MAX(Jp * (tj + safe_sqrt((V0 * -4.0 + Vp * 4.0 + Jp * (tj * tj)) / Jp)) * (-1.0 / 2.0), Jp * (tj - safe_sqrt((V0 * -4.0 + Vp * 4.0 + Jp * (tj * tj)) / Jp)) * (-1.0 / 2.0))), Jp * tj * (-2.0 / 3.0) + ((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0)) * 1.0 / powf(safe_sqrt(powf(- (Jp * Jp) * Pp * (1.0 / 2.0) + (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) - Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) + (Jp * Jp) * V0 * tj, 2.0) - powf((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0), 3.0)) + (Jp * Jp) * Pp * (1.0 / 2.0) - (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) + Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) - (Jp * Jp) * V0 * tj, 1.0 / 3.0) + powf(safe_sqrt(powf(- (Jp * Jp) * Pp * (1.0 / 2.0) + (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) - Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) + (Jp * Jp) * V0 * tj, 2.0) - powf((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0), 3.0)) + (Jp * Jp) * Pp * (1.0 / 2.0) - (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) + Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) - (Jp * Jp) * V0 * tj, 1.0 / 3.0));
            t2_out = Ap / Jp - tj;
            t4_out = 0;
            t6_out = t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Ap / Jp - tj;
            t4_out = MIN(-(V0 - Vp + Ap * tj + (Ap * Ap) / Jp) / Ap, MAX(((Ap * Ap) * (-3.0 / 2.0) + safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp), ((Ap * Ap) * (-3.0 / 2.0) - safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp)));
            t6_out = t2_out;
        }
    }
    Jp_out = Jp;
}

// calculate duration of time segments for basic acceleration and deceleration curve from and to stationary.
void scurves::cal_posfast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const
{
    if ((Vp < Jp * (tj * tj) * 2.0) || (Pp < Jp * (tj * tj * tj) * 4.0)) {
        // solution = 0 - t6 t4 t2 = 0 0 0
        t4_out = MIN(Vp / (2.0 * Jp * tj * tj), Pp / (4.0 * Jp * tj * tj * tj));
        t2_out = 0;
        t4_out = 0;
        t6_out = 0;
    } else if (Ap < Jp * tj) {
        // solution = 2 - t6 t4 t2 = 0 1 0
        Jp = Ap / tj;
        t2_out = 0;
        t4_out = MIN((Vp - Ap * tj * 2.0) / Ap, -3.0 * tj + safe_sqrt((Pp * 2.0) / Ap + tj * tj));
        t6_out = 0;
    } else {
        if ((Vp < Ap * tj + (Ap * Ap) / Jp) || (Pp < Ap * 1.0 / (Jp * Jp) * powf(Ap + Jp * tj, 2.0))) {
            // solution = 5 - t6 t4 t2 = 1 0 1
            Ap = MIN(Ap, MIN(-Jp * tj * (1.0 / 2.0) + safe_sqrt(Jp * Vp * 4.0 + (Jp * Jp) * (tj * tj)) * (1.0 / 2.0), powf(Jp * tj - powf((Jp * Jp) * Pp * (1.0 / 2.0) + safe_sqrt((Jp * Jp * Jp * Jp) * (Pp * Pp) * (1.0 / 4.0) + (Jp * Jp * Jp * Jp * Jp) * Pp * (tj * tj * tj) * (1.0 / 2.7E1)) + (Jp * Jp * Jp) * (tj * tj * tj) * (1.0 / 2.7E1), 1.0 / 3.0) * 3.0, 2.0) * 1.0 / powf((Jp * Jp) * Pp * (1.0 / 2.0) + safe_sqrt((Jp * Jp * Jp * Jp) * (Pp * Pp) * (1.0 / 4.0) + (Jp * Jp * Jp * Jp * Jp) * Pp * (tj * tj * tj) * (1.0 / 2.7E1)) + (Jp * Jp * Jp) * (tj * tj * tj) * (1.0 / 2.7E1), 1.0 / 3.0) * (1.0 / 9.0)));
            t2_out = Ap / Jp - tj;
            t4_out = 0;
            t6_out = t2_out;
        } else {
            // solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Ap / Jp - tj;
            t4_out = MIN(Vp / Ap - (Ap + Jp * tj) / Jp, (Ap * (-3.0 / 2.0) - Jp * tj * (3.0 / 2.0)) / Jp + (safe_sqrt(Ap * Ap * Ap * Ap + (Ap * Ap) * (Jp * Jp) * (tj * tj) + Ap * (Jp * Jp) * Pp * 8.0 + (Ap * Ap * Ap) * Jp * tj * 2.0) * (1.0 / 2.0)) / (Ap * Jp));
            t6_out = t2_out;
        }
    }
    Jp_out = Jp;
}

// generate three time segment raised cosine jerk profile
void scurves::add_segments_incr_const_decr_jerk(uint16_t &seg_pnt, float tj, float Jp, float Tcj)
{
    add_segment_incr_jerk(seg_pnt, tj, Jp);
    add_segment_const_jerk(seg_pnt, Tcj, Jp);
    add_segment_decr_jerk(seg_pnt, tj, Jp);
}

// generate constant jerk time segment
void scurves::add_segment_const_jerk(uint16_t &seg_pnt, float tin, float J0)
{
    enum jtype_t Jtype = jtype_t::CONSTANT;
    float J = J0;
    float T = segment[seg_pnt - 1].end_time + tin;
    float A = segment[seg_pnt - 1].end_accel + J0 * tin;
    float V = segment[seg_pnt - 1].end_vel + segment[seg_pnt - 1].end_accel * tin + 0.5 * J0 * sq(tin);
    float P = segment[seg_pnt - 1].end_pos + segment[seg_pnt - 1].end_vel * tin + 0.5 * segment[seg_pnt - 1].end_accel * sq(tin) + (1 / 6.0) * J0 * powf(tin, 3.0);
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_incr_jerk(uint16_t &seg_pnt, float tj, float Jp)
{
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));

    enum jtype_t Jtype = jtype_t::POSITIVE;
    float J = Jp;
    float T = segment[seg_pnt - 1].end_time + tj;
    float A = segment[seg_pnt - 1].end_accel + AT;
    float V = segment[seg_pnt - 1].end_vel + segment[seg_pnt - 1].end_accel * tj + VT;
    float P = segment[seg_pnt - 1].end_pos + segment[seg_pnt - 1].end_vel * tj + 0.5 * segment[seg_pnt - 1].end_accel * sq(tj) + PT;
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

// generate  decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_decr_jerk(uint16_t &seg_pnt, float tj, float Jp)
{
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));
    float A2T = Jp * tj;
    float V2T = Jp * sq(tj);
    float P2T = Alpha * ((-1.0 / sq(Beta)) * 2.0 * tj + (4.0 / 3.0) * powf(tj, 3.0));

    enum jtype_t Jtype = jtype_t::NEGATIVE;
    float J = Jp;
    float T = segment[seg_pnt - 1].end_time + tj;
    float A = (segment[seg_pnt - 1].end_accel - AT) + A2T;
    float V = (segment[seg_pnt - 1].end_vel - VT) + (segment[seg_pnt - 1].end_accel - AT) * tj + V2T;
    float P = (segment[seg_pnt - 1].end_pos - PT) + (segment[seg_pnt - 1].end_vel - VT) * tj + 0.5 * (segment[seg_pnt - 1].end_accel - AT) * sq(tj) + P2T;
    add_segment(seg_pnt, T, Jtype, J, A, V, P);
}

void scurves::add_segment(uint16_t &seg_pnt, float end_time, enum jtype_t jtype, float jerk_ref, float end_accel, float end_vel, float end_pos)
{
    segment[seg_pnt].end_time = end_time;
    segment[seg_pnt].jtype = jtype;
    segment[seg_pnt].jerk_ref = jerk_ref;
    segment[seg_pnt].end_accel = end_accel;
    segment[seg_pnt].end_vel = end_vel;
    segment[seg_pnt].end_pos = end_pos;
    seg_pnt++;
}
