#pragma once

#include <AP_Common/AP_Common.h>

class scurves {
public:
    scurves() {
        otj = 0.0;
        oJp = 0.0;
        oAp = 0.0;
        oVp = 0.0;
        _t = 0.0;
    }

    // constructor
    scurves(float tj, float Jp, float Ap, float Vp) :
            otj(tj), oJp(Jp), oAp(Ap), oVp(Vp) {
        _t = 0.0;
    }

    void calculate_leg(Vector3f origin, Vector3f destination);
    void calculate_spline_leg(Vector3f origin, Vector3f destination, Vector3f origin_vector, Vector3f destination_vector);

    bool move_to_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_time_pos_vel_accel(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    bool move_to_pva_streight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_pva_streight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_time_pva_streight(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_time_pva_spline(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    Vector3f get_pos_end() { return _track; };

    void Cal_Init(float T0, float J0, float A0, float V0, float P0);
    void Cal_T(float tin, float J0);
    void Cal_JS1(float tj, float Jp);
    void Cal_JS2(float tj, float Jp);
    void Cal_tj_Jp_Tcj(float tj, float Jp, float Tcj);

    void Cal_Ps(float Pp);
    void Cal_Pc(float Pp, float Pm);
    void Cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);
    void Cal_PosFast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);

    void advance_time(float dt) {
        _t += dt;
    }
    bool runme_rev(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    bool runme(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    bool runme(float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
        return runme(_t, Jt_out, At_out, Vt_out, Pt_out);
    }
    void JConst(float t, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    void JSegment1(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    void JSegment2(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);

    float time_now() {
        return _t;
    }

    bool is_streight(){return streight;}

    float pos_end();
    float time_end();
    float time_to_end();
    bool breaking();

    float pos_end_streight();
    float time_end_streight();
    float time_to_end_streight();
    bool breaking_streight();

    float pos_end_spline();
    float time_end_spline();
    float time_to_end_spline();
    bool breaking_spline();


private:

    const static uint16_t array_size_max = 21;

    // scurve segment types
    enum jtype_t {
        JTYPE_CONSTANT, JTYPE_POSITIVE, JTYPE_NEGATIVE
    };
    void Segment(float T, enum jtype_t Jtype, float J, float A, float V, float P);

    // members
    float otj;
    float oJp;
    float oAp;
    float oVp;
    float _t;

    // arrays
    uint16_t num_items;
    float oJ[array_size_max];
    enum jtype_t oJtype[array_size_max];
    float oT[array_size_max];
    float oA[array_size_max];
    float oV[array_size_max];
    float oP[array_size_max];

    uint16_t timer;
    Vector3f _track;
    Vector3f _delta_unit_1;
    Vector3f _delta_unit_2;
    Vector3f _delta_unit_3;
    bool streight;
};
