#pragma once

#include <AP_Common/AP_Common.h>

class scurves {
public:
    scurves() {
        type = EMPTY;
        otj = 0.0;
        oJp = 0.0;
        oAp = 0.0;
        oVp = 0.0;
        _t = 0.0;
        num_items = 0;
    }

    // constructor
    scurves(float tj, float Jp, float Ap, float Vp) :
            otj(tj), oJp(Jp), oAp(Ap), oVp(Vp) {
        _t = 0.0;
        num_items = 0;
        type = EMPTY;
    }

    void calculate_straight_leg(Vector3f origin, Vector3f destination);
    void calculate_spline_leg(Vector3f origin, Vector3f destination, Vector3f origin_vector, Vector3f destination_vector);

    // return the position velocity and acceleration referenced from the origin
    bool move_from_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the position velocity and acceleration referenced to the destination
    bool move_to_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the position velocity and acceleration at a given time referenced from the origin
    bool move_from_time_pos_vel_accel(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // Straight and spline implementations of move_from, move_to, and move_time
    bool move_from_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_time_pva_straight(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_to_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    bool move_from_time_pva_spline(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    Vector3f get_pos_end() { return _track; };

    void debug();
    void cal_Init();

    float time_now() {
        return _t;
    }

    bool is_straight() const {return type == STRAIGHT;}

    // magnitude of the position vector at the end of the sequence
    float pos_end() const;

    // time at the end of the sequence
    float time_end() const;

    // time left before sequence will complete
    float time_to_end() const;

    // return true if the sequence is braking to a stop
    bool braking() const;

    // Straight segment implementations of pos_end, time_end, time_to_end and braking
    float pos_end_straight() const;
    float time_end_straight() const;
    float time_to_end_straight() const;
    bool braking_straight() const;

    // Spline segment implementations of pos_end, time_end, time_to_end and braking
    float pos_end_spline() const;
    float time_end_spline() const;
    float time_to_end_spline() const;
    bool braking_spline() const;

private:
    void cal_T(float tin, float J0);
    void cal_JS1(float tj, float Jp);
    void cal_JS2(float tj, float Jp);
    void cal_tj_Jp_Tcj(float tj, float Jp, float Tcj);

    void cal_Ps(float Pp);
    void cal_Pc(float Pp, float Pm);
    void cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);
    void cal_PosFast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);

    void advance_time(float dt) {
        _t += dt;
    }
    bool update_rev(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    bool update(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    bool update(float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
        return update(_t, Jt_out, At_out, Vt_out, Pt_out);
    }
    void JConst(float t, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    void JSegment1(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    void JSegment2(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);

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

    Vector3f _track;
    Vector3f _delta_unit_1;
    Vector3f _delta_unit_2;
    Vector3f _delta_unit_3;

    // segment types, either straight or spine
    enum SegmentType {
        EMPTY = 0,
        STRAIGHT = 1,
        SPLINE = 2
    };

    SegmentType type;
};
