#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#include "AP_InertialNav.h"

/*
  A wrapper around the AP_AHRS class
 */

/**
   update internal state
*/
void AP_InertialNav::update(bool high_vibes)
{
    // get the NE position relative to the local NED earth frame EKF origin in meters
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE_origin(posNE)) {
        _relpos.xy() = posNE;
    }

    // get the D position relative to the local NED earth frame EKF origin in meters
    float posD;
    if (_ahrs_ekf.get_relative_position_D_origin(posD)) {
        _relpos.z = posD;
    }

    // get the velocity relative to the local NED earth frame EKF origin in meters
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        // during high vibration events use vertical position change
        if (high_vibes) {
            float rate_z;
            if (_ahrs_ekf.get_vert_pos_rate(rate_z)) {
                velNED.z = rate_z;
            }
        }
        _velocity = velNED;
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_position_ned - returns the current position relative to the EKF origin, frame NED in meters.
 * 
 * @return
 */
const Vector3f &AP_InertialNav::get_position_ned(void) const
{
    return _relpos;
}

/**
 * get_position_xy - returns the current x-y position relative to the EKF origin, frame NED in meter.
 *
 * @return
 */
const Vector2f &AP_InertialNav::get_position_xy() const
{
    return _relpos.xy();
}

/**
 * get_position_z_down - returns the current z position relative to the EKF origin, frame NED in meters.
 * @return
 */
float AP_InertialNav::get_position_z_down() const
{
    return _relpos.z;
}

/**
 * get_velocity_ned - returns the current velocity relative to the EKF origin, frame NED in m/s.
 *
 * @return velocity vector:
 *      		.x : x-axis velocity in m/s, North
 * 				.y : y-axis velocity in m/s, East
 * 				.z : z-axis velocity in m/s, Down
 */
const Vector3f &AP_InertialNav::get_velocity_ned() const
{
    return _velocity;
}

/**
 * get_velocity_xy - returns the current x-y velocity relative to the EKF origin, frame NED in m/s.
 *
 * @return
 */
const Vector2f &AP_InertialNav::get_velocity_xy() const
{
    return _velocity.xy();
}

/**
 * get_speed_xy - returns the current horizontal speed relative to the EKF origin in m/s
 *
 * @returns the current horizontal speed relative to the EKF origin in m/s
 */
float AP_InertialNav::get_speed_xy() const
{
    return _velocity.xy().length();
}

/**
 * get_velocity_z_down_cms - returns the current z-axis velocity relative to the EKF origin, frame NED in m/s.
 *
 * @return z-axis velocity, frame NED, in m/s
 */
float AP_InertialNav::get_velocity_z_down() const
{
    return _velocity.z;
}
