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

    // generate an optimal jerk limited curve in 3D space that moves over a straight line between two points
    void calculate_straight_leg(Vector3f origin, Vector3f destination);

    // generate jerk limited curve in 3D space approximating a spline between two points
    void calculate_spline_leg(Vector3f origin, Vector3f destination, Vector3f origin_vector, Vector3f destination_vector);

    // increment time pointer and return the position, velocity and acceleration vectors relative to the origin
    bool move_from_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the destination
    bool move_to_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the origin
    bool move_from_time_pos_vel_accel(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);


    // Straight implementations

    // increment time pointer and return the position, velocity and acceleration vectors relative to the origin
    bool move_from_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the destination
    bool move_to_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the origin
    bool move_from_time_pva_straight(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);


    // Spline implementations

    // increment time pointer and return the position, velocity and acceleration vectors relative to the origin
    bool move_from_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the destination
    bool move_to_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the origin
    bool move_from_time_pva_spline(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel);


    // return the final position of the track
    Vector3f get_pos_end() const { return _track; };

    // debugging messages
    void debug();

    // initialise the S-curve track
    void cal_Init();

    // return the current time pointer
    float time_now() {
        return _t;
    }

    // return true if the current segment is a straight segment
    bool is_straight() const {return type == STRAIGHT;}

    // magnitude of the position vector at the end of the sequence
    float pos_end() const;

    // time at the end of the sequence
    float time_end() const;

    // time left before sequence will complete
    float time_to_end() const;

    // return true if the sequence is braking to a stop
    bool braking() const;

    // straight segment implementations of pos_end, time_end, time_to_end and braking
    float pos_end_straight() const;
    float time_end_straight() const;
    float time_to_end_straight() const;
    bool braking_straight() const;

    // spline segment implementations of pos_end, time_end, time_to_end and braking
    float pos_end_spline() const;
    float time_end_spline() const;
    float time_to_end_spline() const;
    bool braking_spline() const;

private:
    // generate constant jerk time segment
    void cal_T(float tin, float J0);
    // generate increasing jerk magnitude time segment based on a raised cosine profile
    void cal_JS1(float tj, float Jp);
    // generate  decreasing jerk magnitude time segment based on a raised cosine profile
    void cal_JS2(float tj, float Jp);
    // generate three time segment raised cosine jerk profile
    void cal_tj_Jp_Tcj(float tj, float Jp, float Tcj);

    // generate time segments for straight segment
    void cal_Ps(float Pp);
    // generate time segments to generate large curved corners
    void cal_Pc(float Pp, float Pm);
    // calculate duration of time segments for basic acceleration and deceleration curve from constant velocity to stationary.
    void cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);
    // calculate duration of time segments for basic acceleration and deceleration curve from and to stationary.
    void cal_PosFast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out);

    // increment the internal time pointer by
    void advance_time(float dt) {
        _t += dt;
    }

    // calculate the jerk, acceleration, velocity and position at time t
    bool update(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    // calculate the jerk, acceleration, velocity and position at time _t
    bool update(float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
        return update(_t, Jt_out, At_out, Vt_out, Pt_out);
    }
    // calculate the jerk, acceleration, velocity and position at time t when running the sequence in reverse
    bool update_rev(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);

    // calculate the jerk, acceleration, velocity and position at time t when running the constant jerk time segment
    void JConst(float t, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    // Calculate the jerk, acceleration, velocity and position at time t when running the increasing jerk magnitude time segment based on a raised cosine profile
    void JSegment1(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);
    // Calculate the jerk, acceleration, velocity and position at time t when running the  decreasing jerk magnitude time segment based on a raised cosine profile
    void JSegment2(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt);

    // maximum number of time segments
    const static uint16_t array_size_max = 21;

    // scurve segment types
    enum jtype_t {
        JTYPE_CONSTANT, JTYPE_POSITIVE, JTYPE_NEGATIVE
    };
    void Segment(float T, enum jtype_t Jtype, float J, float A, float V, float P);

    // members
    float otj; // duration of jerk raised cosine time segment
    float oJp; // maximum jerk magnitude
    float oAp; // maximum acceleration magnitude
    float oVp; // maximum velocity magnitude
    float _t;  // time pointer

    // arrays
    uint16_t num_items; // number of array segments being used
    float oJ[array_size_max]; // initial jerk value for each time segment
    enum jtype_t oJtype[array_size_max]; // time segment type (constant, increasing, decreasing)
    float oT[array_size_max]; // initial time value for each time segment
    float oA[array_size_max]; // initial acceleration value for each time segment
    float oV[array_size_max]; // initial velocity value for each time segment
    float oP[array_size_max]; // initial position value for each time segment

    Vector3f _track;        // total change in position at the end of the track
    Vector3f _delta_unit_1; // reference direction vector for track
    Vector3f _delta_unit_2; // reference direction vector for track
    Vector3f _delta_unit_3; // reference direction vector for track

    // segment types, empty, straight or spine
    enum SegmentType {
        EMPTY = 0,
        STRAIGHT = 1,
        SPLINE = 2
    } type;
};
