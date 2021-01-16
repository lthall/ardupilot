#pragma once

#include <AP_Common/AP_Common.h>

class scurves {

public:

    // constructors
    scurves();
    scurves(float tj, float Jp, float Ap, float Vp);

    // initialise the S-curve track
    bool init();

    // get or set maximum velocity in cm/s
    float get_vel_max() const { return vel_max; }
    void set_vel_max(float velocity_max) { vel_max = velocity_max; }

    // get or set maximum acceleration in cm/s/s
    float get_accel_max() const { return accel_max; }
    void set_accel_max(float acceleration_max) { accel_max = acceleration_max; }

    // generate an optimal jerk limited curve in 3D space that moves over a straight line between two points
    void calculate_leg(const Vector3f &origin, const Vector3f &destination);

    // Change the starting velocity of the S-curve
    float set_start_vel(float vel);

    // Change the ending velocity of the S-curve
    void set_end_vel(float vel);

    // increment time pointer and return the position, velocity and acceleration vectors relative to the origin
    void move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the destination
    void move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);
    // return the position, velocity and acceleration vectors relative to the origin
    void move_from_time_pos_vel_accel(float time, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the change in position from origin to destination
    const Vector3f& get_track() const { return _track; };

    // return the current time pointer
    float get_time_elapsed() const { return _t; }

    // magnitude of the position vector at the end of the sequence
    float pos_end() const;

    // time has reached the end of the sequence
    bool finished() const;

    // time at the end of the sequence
    float time_end() const;

    // time left before sequence will complete
    float get_time_remaining() const;

    // time left before sequence will complete
    float get_accel_finished_time() const;

    // return true if the sequence is braking to a stop
    bool braking() const;

private:

    // increment the internal time pointer by
    void advance_time(float dt) { _t += dt; }

    // calculate the jerk, acceleration, velocity and position at time t
    void update(float t, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out);
    // calculate the jerk, acceleration, velocity and position at time _t
    void update(float &Jt_out, float &At_out, float &Vt_out, float &Pt_out) {
        return update(_t, Jt_out, At_out, Vt_out, Pt_out);
    }

    // calculate the jerk, acceleration, velocity and position at time t when running the constant jerk time segment
    void calc_javp_for_segment_const_jerk(float t, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;
    // Calculate the jerk, acceleration, velocity and position at time t when running the increasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_incr_jerk(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;
    // Calculate the jerk, acceleration, velocity and position at time t when running the decreasing jerk magnitude time segment based on a raised cosine profile
    void calc_javp_for_segment_decr_jerk(float t, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const;

    // debugging messages
    void debug();

    // generate time segments for straight segment
    void add_segments(float Pp);
    // calculate duration of time segments for basic acceleration and deceleration curve from and to stationary.
    void cal_pos(float tj, float V0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const;
    // calculate duration of time segments for basic acceleration and deceleration curve from and to stationary.
    void cal_posfast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const;

    // generate three time segment raised cosine jerk profile
    void add_segments_incr_const_decr_jerk(uint16_t &seg_pnt, float tj, float Jp, float Tcj);
    // generate constant jerk time segment
    void add_segment_const_jerk(uint16_t &seg_pnt, float tin, float J0);
    // generate increasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_incr_jerk(uint16_t &seg_pnt, float tj, float Jp);
    // generate decreasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_decr_jerk(uint16_t &seg_pnt, float tj, float Jp);

    // scurve segment types
    enum class jtype_t {
        CONSTANT,
        POSITIVE,
        NEGATIVE
    };

    void add_segment(uint16_t &seg_pnt, float end_time, enum jtype_t jtype, float jerk_ref, float end_accel, float end_vel, float end_pos);

    // members
    float otj;          // duration of jerk raised cosine time segment
    float jerk_max;     // maximum jerk magnitude
    float accel_max;    // maximum acceleration magnitude
    float vel_max;      // maximum velocity magnitude
    float _t;           // time

    // arrays
    const static uint16_t segments_max = 21;  // maximum number of time segments
    uint16_t num_segs;      // number of time segments being used
    struct {
        float jerk_ref;     // jerk value for time segment
        jtype_t jtype;      // segment type (jerk is constant, increasing or decreasing)
        float end_time;   // initial time value for time segment
        float end_accel;  // initial acceleration value for time segment
        float end_vel;    // initial velocity value for time segment
        float end_pos;    // initial position value for time segment
    } segment[segments_max];

    Vector3f _track;        // total change in position from origin to destination
    Vector3f _delta_unit; // reference direction vector for track
};
