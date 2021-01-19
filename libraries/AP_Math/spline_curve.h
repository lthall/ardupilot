#pragma once

#include <AP_Common/AP_Common.h>

class spline_curve {

public:

    // constructor
    spline_curve();

    // set maximum speed and acceleration in cm/s and cm/s/s
    void set_speed_accel(float speed_xy_cms, float speed_up_cms, float speed_down_cms,
                         float accel_xy_cmss, float accel_z_cmss);

    // set origin and destination using position vectors (offset from EKF origin in cm
    // origin_vel is vehicle velocity at origin (in cm/s in NEU frame)
    // destination_vel is vehicle velocity at destination (in cm/s in NEU frame)
    // vel_target_length is latest position controller velocity target length (in cm/s)
    void set_origin_and_destination(const Vector3f &origin, const Vector3f &destination, const Vector3f &origin_vel, const Vector3f &destination_vel);

    // move target location along track from origin to destination
    // target_pos is updated with the target position in cm from EKF origin in NEU frame
    // target_vel is updated with the target velocity in cm/s in NEU frame
    void advance_target_along_track(float dt, Vector3f &target_pos, Vector3f &target_vel);

    // returns true if vehicle has reached destination
    bool reached_destination() const { return _reached_destination; }

    // returns the unscaled destination velocity vector
    const Vector3f& get_destination_vel() { return _destination_vel; }

    // returns maximum speed at origin
    float get_origin_speed_max() const { return _origin_speed_max; }

    // get or set maximum speed at destination
    float get_destination_speed_max() const { return _destination_speed_max; }
    void set_destination_speed_max(float destination_speed_max) { _destination_speed_max = MIN(_destination_speed_max, destination_speed_max); }

private:

    // calculate the spline delta time for a given delta distance and
    // returns the spline position and velocity and maximum speed vehicle can travel without exceeding acceleration limits
    void calc_dt_speed_max(float time, float distance_delta, float &spline_dt, Vector3f &target_pos, Vector3f &spline_vel_unit, float &speed_xy_max);

    // recalculate hermite_spline_solution grid
    void update_solution(const Vector3f &origin, const Vector3f &dest, const Vector3f &origin_vel, const Vector3f &dest_vel);

    // calculate target position and velocity from given spline time
    // time is a value from 0 to 1
    // position is updated with target position as an offset from EKF origin in NEU frame
    // velocity is updated with the unscaled velocity
    // relies on set_origin_and_destination having been called to update_solution
    void calc_target_pos_vel(float time, Vector3f &position, Vector3f &velocity, Vector3f &acceleration, Vector3f &jerk);

    // interval variables
    Vector3f    _origin;            // origin offset in cm (in NEU frame) from EKF
    Vector3f    _destination;       // destination offset in cm (in NEU frame) from EKF
    Vector3f    _origin_vel;        // the target velocity vector in cm/s in NEU frame at the origin of the spline segment
    Vector3f    _destination_vel;   // the target velocity vector in cm/s in NEU frame at the destination point of the spline segment
    Vector3f    _hermite_solution[4];   // array describing path between origin and destination
    float       _time;              // current spline time (between 0 and 1) between origin and destination
    float       _speed_xy_cms;      // maximum horizontal speed in cm/s
    float       _speed_up_cms;      // maximum speed upwards in cm/s
    float       _speed_down_cms;    // maximum speed downwards in cm/s
    float       _accel_xy_cmss;     // maximum horizontal acceleration in cm/s/s
    float       _accel_z_cmss;      // maximum vertical acceleration in cm/s/s
    float       _origin_speed_max;  // maximum speed in cm/s at origin
    float       _destination_speed_max; // maximum speed in cm/s at destination
    bool        _reached_destination;   // true once vehicle has reached destination
};
