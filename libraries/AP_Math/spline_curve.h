#pragma once

#include <AP_Common/AP_Common.h>

class spline_curve {

public:

    // constructor
    spline_curve();

    // set maximum speed, acceleration and leash lengths
    void set_speed_accel_leash(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                               float accel_xy_cms, float accel_z_cms,
                               float leash_xy_cm, float leash_up_cm, float leash_down_cm);

    // set origin and destination using position vectors (offset from EKF origin in cm
    // origin_vel is vehicle velocity at origin (in cm/s in NEU frame)
    // destination_vel is vehicle velocity at destination (in cm/s in NEU frame)
    // vel_target_length is latest position controller velocity target length (in cm/s)
    void set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel, float vel_target_length);

    // move target location along track from origin to destination
    // target_pos is updated with the target position in cm from EKF origin in NEU frame
    // target_vel is updated with the target velocity in cm/s in NEU frame
    void advance_target_along_track(const Vector3f &curr_pos, float dt, Vector3f &target_pos, Vector3f &target_vel);

    // returns true if vehicle has reached destination
    bool reached_destination() const { return _reached_destination; }

private:

    // recalculate hermite_spline_solution grid
    void update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel);

    // calculates horizontal and vertical leash lengths for waypoint controller
    // pos_delta_unit should be a unit vector (in NEU frame) pointing from origin to destination
    // or pointing in the direction of the latest target velocity
    // results placed in _track_leash_length
    // also calls calc_slow_down_distance() to update _slow_down_dist
    void calc_leash_length(const Vector3f &pos_delta_unit);

    // calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
    // results placed in _slow_down_dist
    void calc_slow_down_distance(float speed_cms, float accel_cmss);

    // calculate target position and velocity from given spline time
    // time is a value from 0 to 1
    // position is updated with target position as an offset from EKF origin in NEU frame
    // velocity is updated with the unscaled velocity
    // relies on set_origin_and_destination having been called to update_solution
    void calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity);

    // interval variables
    Vector3f    _origin;            // origin offset in cm (in NEU frame) from EKF
    Vector3f    _destination;       // destination offset in cm (in NEU frame) from EKF
    Vector3f    _origin_vel;        // the target velocity vector in cm/s in NEU frame at the origin of the spline segment
    Vector3f    _destination_vel;   // the target velocity vector in cm/s in NEU frame at the destination point of the spline segment
    Vector3f    _hermite_solution[4];   // array describing path between origin and destination
    float       _time;              // current spline time (between 0 and 1) between origin and destination
    float       _speed_xy_cms;      // maximum horizontal speed in cm/s
    float       _speed_up_cms;      // maximum speed upwards in cm/s
    float       _speed_down_cms;    // maximum speed upwards in cm/s
    float       _accel_xy_cmss;     // maximum horizontal acceleration in cm/s/s in NEU frame
    float       _accel_z_cmss;      // maximum vertical acceleration in cm/s/s in NEU frame
    float       _leash_xy_cm;       // horizontal leash length in cm
    float       _leash_up_cm;       // vertical (upwards) leash length in cm
    float       _leash_down_cm;     // vertical (downwards) leash length in cm
    float       _vel_scalar;        // scaling to reduce velocity from the hermite solution to an actual velocity
    float       _track_leash_length;    // leash length in cm which is the shorter of the horizontal and vertical leashes depending upon the direction of the track
    float       _slow_down_dist;    // distance from the destination (in cm) after which vehicle should begin to slowdown
    bool        _reached_destination;   // true once vehicle has reached destination
};
