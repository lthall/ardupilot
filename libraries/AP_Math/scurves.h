#pragma once

#include <AP_Common/AP_Common.h>

class scurves {

public:

    // constructors
    scurves();
    scurves(float tj, float Jp, float Ap, float Vp);

    // initialise the S-Curve path
    void init();

    // get or set maximum speed along path
    float get_speed_max() const { return vel_max; }
    void set_speed_max(float speed_cms) { vel_max = speed_cms; }

    // get or set maximum acceleration along path
    float get_accel_max() const { return accel_max; }
    void set_accel_max(float acceleration_max) { accel_max = acceleration_max; }

    // set maximum velocity and re-calculate the path using these limits
    void set_speed_accel(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                         float accel_xy_cmss, float accel_z_cmss);

    // generate a trigonometric S-Curve path in 3D space that moves over a straight line
    // between two points defined by the origin and destination.
    void calculate_leg(const Vector3f &origin, const Vector3f &destination,
                       float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                       float accel_xy_cmss, float accel_z_cmss);

    // set the maximum vehicle speed at the origin
    // returns the expected speed at the origin (will always be equal or lower to speed_cm)
    float set_origin_speed_max(float speed_cms);

    // set the maximum vehicle speed at the destination
    void set_destination_speed_max(float speed_cms);

    // increment time pointer and return the position, velocity and acceleration vectors relative to the origin
    void move_from_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // increment time pointer and return the position, velocity and acceleration vectors relative to the destination
    void move_to_pos_vel_accel(float dt, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the position, velocity and acceleration vectors relative to the origin at a specified time along the path
    void move_from_time_pos_vel_accel(float time, Vector3f &pos, Vector3f &vel, Vector3f &accel);

    // return the change in position from origin to destination
    const Vector3f& get_track() const { return _track; };

    // return the current time pointer
    float get_time_elapsed() const { return _t; }

    // time has reached the end of the sequence
    bool finished() const;

    // magnitude of the position vector at the end of the sequence
    float pos_end() const;

    // time at the end of the sequence
    float time_end() const;

    // time left before sequence will complete
    float get_time_remaining() const;

    // time when acceleration section of the sequence will complete
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

    // calculate the segment times for the trigonometric S-Curve path defined by:
    // V0 - initial velocity magnitude
    // tj - duration of the raised cosine jerk profile
    // Jm - maximum value of the raised cosine jerk profile
    // Am - maximum constant acceleration
    // Vm - maximum constant velocity
    // L - Length of the path
    void cal_pos(float tj, float V0, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const;

    // generate three time segments forming the jerk profile
    void add_segments_incr_const_decr_jerk(uint16_t &seg_pnt, float tj, float Jp, float Tcj);

    // generate constant jerk time segment
    void add_segment_const_jerk(uint16_t &seg_pnt, float tin, float J0);

    // generate increasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_incr_jerk(uint16_t &seg_pnt, float tj, float Jp);

    // generate decreasing jerk magnitude time segment based on a raised cosine profile
    void add_segment_decr_jerk(uint16_t &seg_pnt, float tj, float Jp);

    // set speed and acceleration limits for the path
    // origin and destination are offsets from EKF origin
    // speed and acceleration parameters are given in horizontal, up and down.
    void set_kinematic_limits(const Vector3f &origin, const Vector3f &destination,
                              float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                              float accel_xy_cmss, float accel_z_cmss);

    // S-Curve segment types
    enum class jtype_t {
        CONSTANT,
        POSITIVE,
        NEGATIVE
    };

    // add single S-Curve segment
    void add_segment(uint16_t &seg_pnt, float end_time, jtype_t jtype, float jerk_ref, float end_accel, float end_vel, float end_pos);

    // members
    float otj;          // duration of jerk raised cosine time segment
    float jerk_max;     // maximum jerk magnitude
    float accel_max;    // maximum acceleration magnitude
    float vel_max;      // maximum velocity magnitude
    float _t;           // time pointer to define position on the path

    // arrays
    const static uint16_t segments_max = 23;    // maximum number of time segments
    // segment 0 is the initial segment
    // segments 1 to 7 are the acceleration segments
    // segments 8 to 14 are the speed change segments
    // segment 15 is the constant velocity segment
    // segment 16 to 22 is the deceleration segment
    uint16_t num_segs;    // number of time segments being used
    struct {
        float jerk_ref;   // jerk reference value for time segment
        jtype_t jtype;    // segment type (jerk is constant, increasing or decreasing)
        float end_time;   // final time value for segment
        float end_accel;  // final acceleration value for segment
        float end_vel;    // final velocity value for segment
        float end_pos;    // final position value for segment
    } segment[segments_max];

    Vector3f _track;      // total change in position from origin to destination
    Vector3f _delta_unit; // reference direction vector for path
};
