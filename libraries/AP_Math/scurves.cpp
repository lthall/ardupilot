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

void scurves::calculate_leg(Vector3f origin, Vector3f destination) {
    streight = true;
    _track = destination - origin;
    float track_length = _track.length();
    Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    if (is_zero(track_length)) {
        // avoid possible divide by zero
        _delta_unit_1.x = 0;
        _delta_unit_1.y = 0;
        _delta_unit_1.z = 0;
    } else {
        _delta_unit_1 = _track.normalized();
        Cal_Ps(track_length);
    }

    hal.console->printf("T, Jt, J, A, V, P\n");
    for (uint8_t i = 0; i < num_items; i++) {
        hal.console->printf("i: %4.2f, T: %4.2f, Jtype: %4.2f, J: %4.2f, A: %4.2f, V: %4.2f, P: %4.2f\n", (float)i, oT[i], (float) oJtype[i], oJ[i], oA[i], oV[i], oP[i]);
    }
}

void scurves::calculate_spline_leg(Vector3f origin, Vector3f destination, Vector3f origin_vector, Vector3f destination_vector)
{
    if (_track.is_zero() ) {
        calculate_leg(origin, destination);
        return;
    } else if (origin_vector.is_zero() && destination_vector.is_zero()) {
        calculate_leg(origin, destination);
        return;
    }

    streight = false;
    _track = destination - origin;
    float track_length = _track.length();
    float x = 0.0f;
    float y = 0.0f;
    float cos_phi = 0.0f;
    float cos_alpha = 0.0f;

    Vector3f track_unit = _track.normalized();
    if (origin_vector.is_zero()) {
        origin_vector.normalize();
        cos_phi = track_unit * origin_vector;
        y = cos_phi + safe_sqrt(9.0f - (1 - sq(cos_phi)));
        x = 1.0 / y;
        destination_vector = origin_vector * x - track_unit;
    } else if (destination_vector.is_zero()) {
        destination_vector.normalize();
        cos_alpha = track_unit * destination_vector;
        y = cos_alpha + safe_sqrt(9.0f - (1 - sq(cos_alpha)));
        x = 1.0 / y;
        origin_vector = track_unit - destination_vector * x;
    } else {
        origin_vector.normalize();
        destination_vector.normalize();
        cos_phi = track_unit * origin_vector;
        cos_alpha = track_unit * destination_vector;
        if (cos_phi + cos_alpha >= 2.0f) {
            x = 0.5;
        } else if (is_zero(cos_phi - 1.0f) && is_zero(cos_alpha - 1.0f)) {
            x = 0.25;
        } else {
            Vector3f vector_zero_gap = origin_vector + destination_vector - track_unit * (cos_phi + cos_alpha);
            if (!vector_zero_gap.is_zero()) {
                float zero_gap = vector_zero_gap.length_squared();
                y = (cos_phi + cos_alpha + safe_sqrt(4.0 - zero_gap));
                x = MIN(1.0 / y, 0.5f);
                hal.console->printf("3D Solution: cos_phi %4.2f, cos_alpha %4.2f, zero_gap %4.2f, x %4.2f \n", cos_phi, cos_alpha, zero_gap, x);
            } else {
                float zero_gap = 0.0f;
                y = (cos_phi + cos_alpha + 2.0f);
                x = MIN(1.0 / y, 0.5f);
                hal.console->printf("3D Solution: cos_phi %4.2f, cos_alpha %4.2f, zero_gap %4.2f, x %4.2f \n", cos_phi, cos_alpha, zero_gap, x);
            }
        }
        _delta_unit_1 = origin_vector;
        _delta_unit_3 = destination_vector;
    }

    Vector3f track_2 = _track - (_delta_unit_1 + _delta_unit_3)*(track_length*x);
    _delta_unit_2 = track_2.normalized();
    Cal_Pc(track_length * x, track_2.length());

    hal.console->printf("origin - x %4.2f, y %4.2f, z %4.2f \n", origin.x, origin.y, origin.z);
    hal.console->printf("destination - x %4.2f, y %4.2f, z %4.2f \n", destination.x, destination.y, destination.z);
    hal.console->printf("origin_vector - x %4.2f, y %4.2f, z %4.2f \n", origin_vector.x, origin_vector.y, origin_vector.z);
    hal.console->printf("destination_vector - x %4.2f, y %4.2f, z %4.2f \n", destination_vector.x, destination_vector.y, destination_vector.z);
    hal.console->printf("_track - x %4.2f, y %4.2f, z %4.2f \n", _track.x, _track.y, _track.z);
    hal.console->printf("_delta_unit_1 - x %4.2f, y %4.2f, z %4.2f \n", _delta_unit_1.x, _delta_unit_1.y, _delta_unit_1.z);
    hal.console->printf("_delta_unit_3 - x %4.2f, y %4.2f, z %4.2f \n", _delta_unit_3.x, _delta_unit_3.y, _delta_unit_3.z);
    hal.console->printf("_delta_unit_2 - x %4.2f, y %4.2f, z %4.2f \n", _delta_unit_2.x, _delta_unit_2.y, _delta_unit_2.z);
    hal.console->printf("length %4.2f, length m %4.2f, cos_phi %4.2f, cos_alpha %4.2f, x %4.2f \n", track_length, track_2.length(), cos_phi, cos_alpha, x);

    hal.console->printf("T, Jt, J, A, V, P\n");
    for (uint8_t i = 0; i < num_items; i++) {
        hal.console->printf("i: %4.2f, T: %4.2f, Jtype: %4.2f, J: %4.2f, A: %4.2f, V: %4.2f, P: %4.2f\n", (float) i, oT[i], (float) oJtype[i], oJ[i], oA[i], oV[i], oP[i]);
    }
}

bool scurves::move_to_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    if (streight) {
        return move_to_pva_streight(dt, time_scale, pos, vel, accel);
    } else {
        return move_to_pva_spline(dt, time_scale, pos, vel, accel);
    }
}

bool scurves::move_from_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    if (streight) {
        return move_from_pva_streight(dt, time_scale, pos, vel, accel);
    } else {
        return move_from_pva_spline(dt, time_scale, pos, vel, accel);
    }
}

bool scurves::move_to_time_pos_vel_accel(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    if (streight) {
        return move_to_time_pva_streight(time, time_scale, pos, vel, accel);
    } else {
        return move_to_time_pva_spline(time, time_scale, pos, vel, accel);
    }
}

bool scurves::move_to_pva_streight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    advance_time(time_scale * dt);
    bool finish = move_to_time_pva_streight(_t, time_scale, pos, vel, accel);
    return finish;
}

bool scurves::move_from_pva_streight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    advance_time(time_scale * dt);
    bool finish = move_to_time_pva_streight(_t, time_scale, pos, vel, accel);
    pos -= _track;
    return finish;
}

bool scurves::move_to_time_pva_streight(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    float scurve_P1, scurve_V1, scurve_A1, scurve_J1;
    bool finish = runme(time, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_1 * scurve_P1;
    vel += _delta_unit_1 * scurve_V1 * MIN(time_scale * 1.1f, 1.0f);
    accel += _delta_unit_1 * scurve_A1 * time_scale;
    return finish;
}

bool scurves::move_to_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    advance_time(time_scale * dt);
    bool finish = move_to_time_pva_spline(_t, time_scale, pos, vel, accel);
    return finish;
}

bool scurves::move_from_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    advance_time(time_scale * dt);
    bool finish = move_to_time_pva_spline(_t, time_scale, pos, vel, accel);
    pos -= _track;
    return finish;
}

bool scurves::move_to_time_pva_spline(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel) {
    bool finish;
    float scurve_P1, scurve_V1, scurve_A1, scurve_J1;
    float time_start = oT[7];
    float time_mid = oT[num_items - 1] - oT[11];

    runme(MIN(time, oT[11]), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_1 * scurve_P1;
    vel += _delta_unit_1 * scurve_V1;
    accel += _delta_unit_1 * scurve_A1;

    runme(MAX(time-time_start + oT[11], oT[11]), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_2 * (scurve_P1 - oP[11]);
    vel += _delta_unit_2 * scurve_V1;
    accel += _delta_unit_2 * scurve_A1;

    runme(MIN(oT[11] - (time-time_start*2-time_mid+ oT[11]) , oT[11]), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
//    runme(MIN(oP[11] - (time-oT[num_items - 1]) , oP[11]), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
//    hal.console->printf("scurve_J1 %4.2f, scurve_A1 %4.2f, scurve_V1 %4.2f, scurve_P1 %4.2f \n", scurve_J1, -scurve_A1, scurve_V1, (oP[11] - scurve_P1));
    pos += _delta_unit_3 * (oP[11] - scurve_P1);
    vel += _delta_unit_3 * scurve_V1;
    accel += _delta_unit_3 * (-scurve_A1);

    finish = _t > (time_mid + 2.0*time_start);
    finish = _t > (time_mid + 2.0*time_start);

    vel *=  MIN(time_scale * 1.1f, 1.0f);
    accel *= time_scale;
    return finish;
}

float scurves::pos_end() {
    if (streight) {
        return pos_end_streight();
    } else {
        return pos_end_spline();
    }
}

float scurves::time_end() {
    if (streight) {
        return time_end_streight();
    } else {
        return time_end_spline();
    }
}

float scurves::time_to_end() {
    if (streight) {
        return time_to_end_streight();
    } else {
        return time_to_end_spline();
    }
}

bool scurves::breaking() {
    if (streight) {
        return breaking_streight();
    } else {
        return breaking_spline();
    }
}

float scurves::pos_end_streight() {
    return oP[num_items - 1];
}

float scurves::time_end_streight() {
    return oT[num_items - 1];
}

float scurves::time_to_end_streight() {
    return oT[num_items - 1] - _t;
}

bool scurves::breaking_streight() {
    return _t > oT[9];
}

float scurves::pos_end_spline() {
    return oP[num_items - 1] + oP[11];
}

float scurves::time_end_spline() {
    return oT[num_items - 1] - oT[11] + oT[7]*2.0;
}

float scurves::time_to_end_spline() {
    return time_end_spline() - _t;
}

bool scurves::breaking_spline() {
    return _t > oT[num_items - 1] - oT[11] + oT[7];
}

void scurves::Segment(float T, enum jtype_t Jtype, float J, float A, float V, float P) {
    oT[num_items] = T;
    oJtype[num_items] = Jtype;
    oJ[num_items] = J;
    oA[num_items] = A;
    oV[num_items] = V;
    oP[num_items] = P;
    num_items++;
    //    hal.console->printf("Segment - J %4.2f, T %4.2f, A %4.2f, V %4.2f, P %4.2f\n", J, T, A, V, P);
}

void scurves::Cal_Init(float T0, float J0, float A0, float V0, float P0) {
    _t = 0.0f;
    num_items = 0;
    enum jtype_t Jtype = JTYPE_CONSTANT;
    float J = J0;
    float T = T0;
    float A = A0;
    float V = V0;
    float P = P0;

    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_T(float tin, float J0) {
    enum jtype_t Jtype = JTYPE_CONSTANT;
    float J = J0;
    float T = oT[num_items - 1] + tin;
    float A = oA[num_items - 1] + J0 * tin;
    float V = oV[num_items - 1] + oA[num_items - 1] * tin + 0.5 * J0 * sq(tin);
    float P = oP[num_items - 1] + oV[num_items - 1] * tin + 0.5 * oA[num_items - 1] * sq(tin) + (1 / 6.0) * J0 * powf(tin, 3.0);
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_JS1(float tj, float Jp) {
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));

    enum jtype_t Jtype = JTYPE_POSITIVE;
    float J = Jp;
    float T = oT[num_items - 1] + tj;
    float A = oA[num_items - 1] + AT;
    float V = oV[num_items - 1] + oA[num_items - 1] * tj + VT;
    float P = oP[num_items - 1] + oV[num_items - 1] * tj + 0.5 * oA[num_items - 1] * sq(tj) + PT;
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_JS2(float tj, float Jp) {
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));
    float A2T = Jp * tj;
    float V2T = Jp * sq(tj);
    float P2T = Alpha * ((-1.0 / sq(Beta)) * 2.0 * tj + (4.0 / 3.0) * powf(tj, 3.0));

//    hal.console->printf("Cal_JS2 - Beta %4.2f, Alpha %4.2f, AT %4.2f, VT %4.2f, PT %4.2f, A2T %4.2f, V2T %4.2f, P2T %4.2f\n", Beta, Alpha, AT, VT, PT, A2T, V2T, P2T);

    enum jtype_t Jtype = JTYPE_NEGATIVE;
    float J = Jp;
    float T = oT[num_items - 1] + tj;
    float A = (oA[num_items - 1] - AT) + A2T;
    float V = (oV[num_items - 1] - VT) + (oA[num_items - 1] - AT) * tj + V2T;
    float P = (oP[num_items - 1] - PT) + (oV[num_items - 1] - VT) * tj + 0.5 * (oA[num_items - 1] - AT) * sq(tj) + P2T;
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_tj_Jp_Tcj(float tj, float Jp, float Tcj) {
    Cal_JS1(tj, Jp);
//    hal.console->printf("Cal_JS1 - tj %4.2f, Tcj %4.2f\n", tj, Jp);
    Cal_T(Tcj, Jp);
//    hal.console->printf("Cal_T - Tcj %4.2f, Tcj %4.2f\n", Tcj, Jp);
    Cal_JS2(tj, Jp);
//    hal.console->printf("Cal_JS2 - tj %4.2f, Tcj %4.2f\n", tj, Jp);
}

void scurves::Cal_Ps(float Pp) {
    _t = 0.0f;
//    timer == 40;
    hal.console->printf("Cal_Ps : %4.2f, otj %4.2f, oJp %4.2f, oAp %4.2f, oVp %4.2f\n", Pp, otj, oJp, oAp, oVp);

    if (is_zero(Pp)) {
        return;
    }

    float tj = otj;
    float Jp = oJp;
    float Ap = oAp;
    float Vp = oVp;

    float t2, t4, t6;
    Cal_PosFast(tj, Jp, Ap, Vp, Pp / 2.0, Jp, t2, t4, t6);
//    hal.console->printf("Block1 : Jp %4.2f, t2 %4.2f, t4 %4.2f, t6 %4.2f\n", Jp, t2, t4, t6);
    Cal_tj_Jp_Tcj(tj, Jp, t2);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);

    float Tcv = (Pp / 2.0 - oP[num_items - 1]) / oV[num_items - 1];
    Cal_T(Tcv, 0.0);
    Cal_T(Tcv, 0.0);
//    hal.console->printf("Block3 : tin %4.2f, J0 %4.2f\n", Tcv, 0.0);

//    hal.console->printf("Block4 : Jp %4.2f, t2 %4.2f, t4 %4.2f, t6 %4.2f\n", Jp, t2, t4, t6);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, Jp, t2);
}

void scurves::Cal_Pc(float Pp,  float Pm) {
    hal.console->printf("Cal_Pc : Pp %4.2f, Pm %4.2f, otj %4.2f, oJp %4.2f, oAp %4.2f, oVp %4.2f\n", Pp, Pm, otj, oJp, oAp, oVp);

    _t = 0.0f;

    if (is_zero(Pp)) {
        return;
    }

    float tj = otj;
    float Jp = oJp;
    float Ap = oAp;
    float Vp = oVp;
    float Js;
    float Vs;
    float Ps;
    float tc = 0;
    float Jc = 0;
    float Ac = 0;
    float Vc = 0;
    float Pc = 0;

    Pc = Pp*2.0/3.0;
    Ps = Pp - Pc;
    Vs = MIN(Vp, MIN(MIN(safe_sqrt(Ap*Pc), powf(0.5*Jp*sq(Pc), 1.0/3.0)), Pc/(2*tj)));
//    hal.console->printf("Cal_Pc Ps: %4.2f, Vs %4.2f \n", Ps, Vs);

    float t2, t4, t6;
    Cal_PosFast(tj, Jp, Ap, Vs, Ps, Js, t2, t4, t6);
    Cal_tj_Jp_Tcj(tj, Js, t2);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, -Js, t6);

    Pc = Pp - oP[num_items - 1];
    Vc = oV[num_items - 1];

//    hal.console->printf("T, Jt, J, A, V, P\n");
//    for (uint8_t i = 0; i < num_items; i++) {
//        hal.console->printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\n", oT[i], (float)oJtype[i], oJ[i], oA[i], oV[i], oP[i]);
//    }

    Ac = MIN(MIN(Pc / (4 * tj * tj), Vc * Vc / Pc), powf((Jp * Jp) * Pc, 1.0 / 3.0) * 6.299605249474365E-1);

    Jc = powf(Ac, 3.0 / 2.0) * 1.0 / safe_sqrt(Pc) * 2.0;
    tc = Ac / Jc;
//    hal.console->printf("Cal_Pc Pc: %4.2f, Vc %4.2f, Ac %4.2f, Jc %4.2f, tc %4.2f \n", Pc, Vc, Ac, Jc, tc);

    Cal_JS1(tc, -Jc);
    Cal_JS2(tc, -Jc);
    Cal_JS1(tc, Jc);
    Cal_JS2(tc, Jc);

    Cal_JS1(tc, Jc);
    Cal_JS2(tc, Jc);
    Cal_JS1(tc, -Jc);
    Cal_JS2(tc, -Jc);

    float Tcv = (Pm - 2.0*(oP[num_items - 1]-Pp)) / oV[num_items - 1];
    Cal_T(Tcv, 0.0);

    Cal_JS1(tc, -Jc);
    Cal_JS2(tc, -Jc);
    Cal_JS1(tc, Jc);
    Cal_JS2(tc, Jc);
}

void scurves::Cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) {
    Ap = MIN(MIN(Ap, (Vp - V0) / (2.0 * tj)), (Pp - P0 + 4.0 * V0 * tj) / (4.0 * sq(tj)));
    if (fabsf(Ap) < Jp * tj) {
        Jp = Ap / tj;
        Jp_out = Jp;
        t2_out = 0.0;
        if ((Vp <= V0 + 2.0 * Ap * tj) || (Pp <= P0 + 4.0 * V0 * tj + 4.0 * Ap * sq(tj))) {
            t4_out = 0.0;
        } else {
            t4_out = MIN(-(V0 - Vp + Ap * tj + (Ap * Ap) / Jp) / Ap, MAX(((Ap * Ap) * (-3.0 / 2.0) + safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) - Ap * (Jp * Jp) * P0 * 2.0 + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp), ((Ap * Ap) * (-3.0 / 2.0) - safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) - Ap * (Jp * Jp) * P0 * 2.0 + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp)));
        }
    } else {
        if ((Vp < V0 + Ap * tj + (Ap * Ap) / Jp) || (Pp < P0 + 1.0 / (Jp * Jp) * (Ap * Ap * Ap + Ap * Jp * (V0 * 2.0 + Ap * tj * 2.0)) + V0 * tj * 2.0 + Ap * (tj * tj))) {
            Ap = MIN(MIN(Ap, MAX(Jp * (tj + safe_sqrt((V0 * -4.0 + Vp * 4.0 + Jp * (tj * tj)) / Jp)) * (-1.0 / 2.0), Jp * (tj - safe_sqrt((V0 * -4.0 + Vp * 4.0 + Jp * (tj * tj)) / Jp)) * (-1.0 / 2.0))), Jp * tj * (-2.0 / 3.0) + ((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0)) * 1.0 / powf(safe_sqrt(powf((Jp * Jp) * P0 * (1.0 / 2.0) - (Jp * Jp) * Pp * (1.0 / 2.0) + (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) - Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) + (Jp * Jp) * V0 * tj, 2.0) - powf((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0), 3.0)) - (Jp * Jp) * P0 * (1.0 / 2.0) + (Jp * Jp) * Pp * (1.0 / 2.0) - (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) + Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) - (Jp * Jp) * V0 * tj, 1.0 / 3.0) + powf(safe_sqrt(powf((Jp * Jp) * P0 * (1.0 / 2.0) - (Jp * Jp) * Pp * (1.0 / 2.0) + (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) - Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) + (Jp * Jp) * V0 * tj, 2.0) - powf((Jp * Jp) * (tj * tj) * (1.0 / 9.0) - Jp * V0 * (2.0 / 3.0), 3.0)) - (Jp * Jp) * P0 * (1.0 / 2.0) + (Jp * Jp) * Pp * (1.0 / 2.0) - (Jp * Jp * Jp) * (tj * tj * tj) * (8.0 / 2.7E1) + Jp * tj * ((Jp * Jp) * (tj * tj) + Jp * V0 * 2.0) * (1.0 / 3.0) - (Jp * Jp) * V0 * tj, 1.0 / 3.0));
            t4_out = 0;
        } else {
            t4_out = MIN(-(V0 - Vp + Ap * tj + (Ap * Ap) / Jp) / Ap, MAX(((Ap * Ap) * (-3.0 / 2.0) + safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) - Ap * (Jp * Jp) * P0 * 2.0 + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp), ((Ap * Ap) * (-3.0 / 2.0) - safe_sqrt((Ap * Ap * Ap * Ap) * (1.0 / 4.0) + (Jp * Jp) * (V0 * V0) + (Ap * Ap) * (Jp * Jp) * (tj * tj) * (1.0 / 4.0) - Ap * (Jp * Jp) * P0 * 2.0 + Ap * (Jp * Jp) * Pp * 2.0 - (Ap * Ap) * Jp * V0 + (Ap * Ap * Ap) * Jp * tj * (1.0 / 2.0) - Ap * (Jp * Jp) * V0 * tj) - Jp * V0 - Ap * Jp * tj * (3.0 / 2.0)) / (Ap * Jp)));
        }
        t2_out = Ap / Jp - tj;
    }
    t6_out = t2_out;
}

void scurves::Cal_PosFast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) {
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
//    hal.console->printf("Cal_PosFast Pp : %4.2f, Vp %4.2f, Ap %4.2f, Jp %4.2f, tj %4.2f, Jp_out %4.2f, t2_out %4.2f, t4_out %4.2f, t6_out %4.2f\n", Pp, Vp, Ap, Jp, tj, Jp_out, t2_out, t4_out, t6_out);
}

bool scurves::runme_rev(float time, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
    bool finish = (time > oT[num_items - 1]);
    runme(oT[num_items - 1] - time, Jt_out, At_out, Vt_out, Pt_out);
    Pt_out = oP[num_items - 1] - Pt_out;
    At_out = -At_out;
    return finish;
}

bool scurves::runme(float time, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
    jtype_t Jtype;
    int8_t pnt = num_items;
    float tj;
    float Jp, T0, A0, V0, P0;

    for (uint8_t i = 0; i < num_items; i++) {
        if (time < oT[num_items - 1 - i]) {
            pnt = num_items - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = JTYPE_CONSTANT;
        Jp = 0.0f;
        T0 = oT[pnt];
        A0 = oA[pnt];
        V0 = oV[pnt];
        P0 = oP[pnt];
    } else if (pnt == num_items) {
        Jtype = JTYPE_CONSTANT;
        Jp = 0.0;
        T0 = oT[pnt - 1];
        A0 = oA[pnt - 1];
        V0 = oV[pnt - 1];
        P0 = oP[pnt - 1];
    } else {
        Jtype = oJtype[pnt];
        Jp = oJ[pnt];
        tj = oT[pnt] - oT[pnt - 1];
        T0 = oT[pnt - 1];
        A0 = oA[pnt - 1];
        V0 = oV[pnt - 1];
        P0 = oP[pnt - 1];
    }

    switch (Jtype) {
    case JTYPE_POSITIVE:
        JSegment1(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case JTYPE_NEGATIVE:
        JSegment2(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    default:
        JConst(time - T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }
//    timer++;
//    if(timer == 40 ){
//        timer = 0;
//        hal.console->printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2u\n", t, Jt_out, At_out, Vt_out, Pt_out, pnt);
//    }

    return pnt == num_items;
}

void scurves::JConst(float time, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) {
    Jt = J0;
    At = A0 + J0 * time;
    Vt = V0 + A0 * time + 0.5 * J0 * (time * time);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (1 / 6.0) * J0 * (time * time * time);
}

void scurves::JSegment1(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) {
    float Alpha = Jp / 2.0;
    float Beta = M_PI / tj;
    Jt = Alpha * (1.0 - cosf(Beta * time));
    At = A0 + Alpha * time - (Alpha / Beta) * sinf(Beta * time);
    Vt = V0 + A0 * time + (Alpha / 2.0) * (time * time) + (Alpha / (Beta * Beta)) * cosf(Beta * time) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (-Alpha / (Beta * Beta)) * time + Alpha * (time * time * time) / 6.0 + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time);
}

void scurves::JSegment2(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) {
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
