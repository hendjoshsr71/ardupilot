/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */
#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_NavEKF/AP_Nav_Common.h>              // definitions shared by inertial and ekf nav filters

class AP_InertialNav
{
public:
    // Constructor
    AP_InertialNav(AP_AHRS &ahrs) :
        _ahrs_ekf(ahrs)
        {}

    /**
       update internal state
    */
    void        update(bool high_vibes = false);

    /**
     * get_filter_status - returns filter status as a series of flags
     */
    nav_filter_status get_filter_status() const;

    /**
     * get_position_ned - returns the current position relative to the EKF origin, frame NED in meters.
     * 
     * @return
     */
    const Vector3f&    get_position_ned() const;

    /**
     * get_position_xy - returns the current x-y position relative to the EKF origin, frame NED in meter.
     *
     * @return
     */
    const Vector2f&    get_position_xy() const;

    /**
     * get_position_z_down - returns the current z position relative to the EKF origin, frame NED in meters.
     * @return
     */
    float              get_position_z_down() const;

    /**
     * get_velocity_ned - returns the current velocity relative to the EKF origin, frame NED in m/s.
     *
     * @return velocity vector:
     *      		.x : x-axis velocity in m/s, North
     * 				.y : y-axis velocity in m/s, East
     * 				.z : z-axis velocity in m/s, Down
     */
    const Vector3f&    get_velocity_ned() const;

    /**
     * get_velocity_xy - returns the current x-y velocity relative to the EKF origin, frame NED in m/s.
     *
     * @return
     */
    const Vector2f& get_velocity_xy() const;

    /**
     * get_speed_xy - returns the current horizontal speed relative to the EKF origin in m/s
     *
     * @returns the current horizontal speed relative to the EKF origin in m/s
     */
    float        get_speed_xy() const;

    /**
     * get_velocity_z_down - returns the current z-axis velocity relative to the EKF origin, frame NED in m/s.
     *
     * @return z-axis velocity, frame NED, in m/s
     */
    float       get_velocity_z_down() const;

private:
    Vector3f _relpos;   // NED in meters
    Vector3f _velocity; // NED in meters
    AP_AHRS &_ahrs_ekf;
};
