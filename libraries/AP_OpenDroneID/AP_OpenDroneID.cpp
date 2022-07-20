/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by:
 *  BlueMark Innovations BV, Roel Schiphorst
 *  Contributors: Tom Pittenger, Josh Henderson
 *  Parts of this code are based on/copied from the Open Drone ID project https://github.com/opendroneid/opendroneid-core-c
 *
 * The code has been tested with the BlueMark DroneBeacon MAVLink transponder running this command in the ArduPlane folder:
 * sim_vehicle.py --console --map -A --serial1=uart:/dev/ttyUSB1:9600
 * (and a DroneBeacon MAVLink transponder connected to ttyUSB1)
 *
 * The Remote ID implementation expects a transponder that caches the received MAVLink messages from ArduPilot
 * and transmits them at the required intervals. So static messages are only send once to the transponder.
 */

#include <stdio.h>
#include "AP_OpenDroneID.h"

#if AP_OPENDRONEID_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>

const AP_Param::GroupInfo AP_OpenDroneID::var_info[] = {
    // @Param: SER_NUM
    // @DisplayName: Serial Number
    // @Description: Serial Number
    // @User: Advanced
    AP_GROUPINFO("SER_NUM", 1, AP_OpenDroneID, _serial_number, 0),

    // @Param: BARO_ACC
    // @DisplayName: Barometer Vertical Accuracy when installed in the vehicle. Note this is dependent upon installation conditions and thus disabled by default.
    // @Description: Serial Number
    // @User: Advanced
    AP_GROUPINFO("BARO_ACC", 2, AP_OpenDroneID, _baro_accuracy, -1.0),

    // FIX ME Add Autofilling in vehicle code for the parameter
    // : autofilled for Plane, Plane_VTOl, Heli, Multicopter, & blimp

    // @Param: UA_TYPE
    // @DisplayName: Unmanned Aircraft Type
    // @Description: unmanned aircraft type
    // @Values: 0:None,1:Plane,2:Helicopter_Or_Multirotor,3:Gyroplane,4:Plane_VTOL,5:Orinthopter,6:Glider,7:Kite,8:Free_Balloon,9:Captive_Balloon,10:Airship,11:Free_Fall_Parachute,12:Rocket,13:Tethered_Powered_Aircraft,14:Ground_Obstacle,15:Other
    // @User: Advanced
    AP_GROUPINFO("UA_TYPE", 3, AP_OpenDroneID, _ua_type_parm, MAV_ODID_UA_TYPE_NONE),

    AP_GROUPEND
};

// constructor
AP_OpenDroneID::AP_OpenDroneID()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("OpenDroneID must be singleton");
    }
#endif
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_OpenDroneID::init()
{
    _last_send_dynamic_messages_ms = 0;
    _last_send_static_messages_ms = 0;
    _mavlink_dynamic_period_ms = 1000;
    _mavlink_static_period_ms = 10000;

     _chan = MAVLINK_COMM_0; //set default to MAVLink channel 0
     _id_type = MAV_ODID_ID_TYPE_NONE;
     _uav_status = MAV_ODID_STATUS_UNDECLARED;
    _height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;

    _operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    _classification_type = MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED;
    _category_eu = MAV_ODID_CATEGORY_EU_UNDECLARED;
    _class_eu = MAV_ODID_CLASS_EU_UNDECLARED;
    _operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA;
    _description_type = MAV_ODID_DESC_TYPE_TEXT;
    _operator_timestamp = 0;
    _operator_position.lat = 0;
    _operator_position.lng = 0;
    _operator_position.alt = (int32_t) -1000*100; //default -1000m

    _area_count = 1; //default 1
    _area_radius = 0; //default 0
    _area_ceiling = -1000; //default -1000
    _area_floor = -1000; //default -1000

    memset(_id_or_mac,0,ODID_ID_SIZE);
    memset(_uas_id,0,ODID_ID_SIZE + 1);
    memset(_operator_id,0,ODID_ID_SIZE + 1);
    memset(_description,0,ODID_STR_SIZE + 1);

    transmit_basic_ID_message = false; //default we don't send any packets
    transmit_location_message = false;
    transmit_self_ID_message = false;
    transmit_system_message = false;
    transmit_operator_id_message = false;

    // UA_Type Parameter
    // If it has been set then use it else pre-arm error will be set by the arming checks
    _ua_type = MAV_ODID_UA_TYPE_NONE;
    if (_ua_type_parm.configured()) {
        _ua_type = MAV_ODID_UA_TYPE(_ua_type_parm.get());
    }

//code to be replaced: set dummy values in open drone ID messages
    set_mavlink_channel(MAVLINK_COMM_1); //set to MAVLink channel 1
    set_mavlink_dynamic_messages_period_ms(1000); //send dynamic messages at 1 Hz frequency i.e. every 1 second
    set_mavlink_static_messages_period_ms(10000); //send dynamic messages at 0.1 Hz frequency i.e. every 10 seconds
    set_id_type(MAV_ODID_ID_TYPE_SERIAL_NUMBER);

    set_operator_location_type(MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS);
    set_classification_type(MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED);
    set_category_eu(MAV_ODID_CATEGORY_EU_UNDECLARED);
    set_class_eu(MAV_ODID_CLASS_EU_UNDECLARED);
    set_operator_id_type(MAV_ODID_OPERATOR_ID_TYPE_CAA);
    set_description_type(MAV_ODID_DESC_TYPE_TEXT);

    set_operator_latitude(10);
    set_operator_longitude(30);
    set_operator_timestamp((time_t)(AP_HAL::micros())/1000); //set to current time
    set_area_count(1); //default 1
    set_area_radius(0); //default 0
    set_area_ceiling(-1000); //default -1000
    set_area_floor(-1000); //default -1000
    set_operator_altitude_geo(-1000); //default -1000

    uint8_t id_or_mac[ODID_ID_SIZE] = "000000000000";
    set_id_or_mac(id_or_mac);

    char uas_id[ODID_ID_SIZE + 1] = "SN000000123456789";
    set_uas_id(uas_id);

    char operator_id[ODID_ID_SIZE + 1] = "NLD000000123456789";
    set_operator_id(operator_id);

    char description[ODID_STR_SIZE + 1] = "MAVLink test flight";
    set_description(description);

    transmit_basic_ID_message = true;  //broadcast all message types
    transmit_location_message = true;
    transmit_self_ID_message = true;
    transmit_system_message = true;
    transmit_operator_id_message = true;
//end dummy values

}

// Perform the pre-arm checks and prevent arming if they are not satisifed
// Except in the case of an in-flight reboot
bool AP_OpenDroneID::pre_arm_check(char* failmsg, uint8_t failmsg_len) const
{
    // UA_Type Parameter
    // If it has been set then use it else pre-arm error will be set by the arming checks
    if (_ua_type_parm.get() == MAV_ODID_UA_TYPE_NONE) {
        // Set Pre-arm error
        strncpy(failmsg, "UA_TYPE Must Not Be NONE", failmsg_len);
        return false;
    }

    return true;
}

void AP_OpenDroneID::update()
{
    const uint32_t now = AP_HAL::millis();

    if (now - _last_send_dynamic_messages_ms >= _mavlink_dynamic_period_ms) {
        _last_send_dynamic_messages_ms = now;
        send_dynamic_out();
    }

    if (now - _last_send_static_messages_ms >= _mavlink_static_period_ms) {
        _last_send_static_messages_ms = now;
        send_static_out();
    }

    // Get the Takeoff Location
    // ahrs.get_takeoff_expected(void)

    // bool got_location = false;
    // if (FLIGHT_MODE == TAKEOFF) {
    //     _height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
        
    //     // got_takeoff_location = AP::ahrs().get_location(_takeoff_position);
        
    //     GPS_Status gps_status = AP::gps().status();
    //     bool got_bad_gps_fix = (gps_status ==  )

    //     // If we fail get the takeoff location we can't takeoff due to 
    //     if (!got_takeoff_location || got_bad_gps_fix) {
    //         // disarm the aircraft this cannot be overriden due to regaualtory requirements

    //             // an auth ID to disallow arming when we don't have the beacon
    //             local auth_id = arming:get_aux_auth_id()
    //             arming:set_aux_auth_failed(auth_id, "Ship: no beacon")

    //     }
    // }

    //     set_height_reference(MAV_ODID_HEIGHT_REF_OVER_TAKEOFF); //or MAV_ODID_HEIGHT_REF_OVER_GROUND if we have WGS-84 position(?)
    //     float height = create_altitude((current_location.alt - _operator_position.alt)*0.01f);

    // We need 

}

void AP_OpenDroneID::send_dynamic_out()
{
    if (transmit_location_message){
        send_location_message();
    }
}

void AP_OpenDroneID::send_static_out()
{
    if (transmit_basic_ID_message) {
        send_basic_id_message();
    }

    if (transmit_system_message) {
        send_system_message();
    }

    if (transmit_self_ID_message) {
        send_self_id_message();
    }

    if (transmit_operator_id_message) {
        send_operator_id_message();
    }
}

// The send_location_message
// all open_drone_id send functions use data stored in the open drone id class.
//This location send function is an exception. It uses live location data from the ArduPilot system.
void AP_OpenDroneID::send_location_message()
{
    AP_AHRS &ahrs = AP::ahrs();
    AP_Baro &barometer = AP::baro();
    const AP_GPS &gps = AP::gps();

    AP_GPS::GPS_Status gps_status = gps.status();
    bool got_bad_gps_fix = (gps_status == AP_GPS::GPS_Status::NO_GPS) || (gps_status == AP_GPS::GPS_Status::NO_FIX) ||
                            (gps_status == AP_GPS::GPS_Status::GPS_OK_FIX_2D);

    Location current_location;
    if (ahrs.get_location(current_location))
    {
        set_uav_status(MAV_ODID_STATUS_UNDECLARED);

        // What are conditions where we have invalid groundspeed vector?
        // For GPS it isn't very accurate if we aren't moving...

        float direction = ODID_INV_DIR;
        if (!got_bad_gps_fix) {
            direction = wrap_360(degrees(ahrs.groundspeed_vector().angle())); // heading (degrees)
        }


        const float speed_horizontal = create_speed_horizontal(ahrs.groundspeed());

        Vector3f velNED;
        UNUSED_RESULT(ahrs.get_velocity_NED(velNED));
        const float climb_rate = create_speed_vertical(-velNED.z); //make sure climb_rate is within Remote ID limit

        // calculate the climb rate.
        // Secondary method with fallbacks??

        // float climb_rate;
        // Vector3f vel;
        // if (ahrs.get_velocity_NED(vel)) {
        //     climb_rate = vel.z;
        // } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        //     climb_rate = gps.velocity().z;
        // } else {
        //     climb_rate = -barometer.get_climb_rate();
        // }

        int32_t latitude = 0;
        int32_t longitude = 0;
        if (current_location.check_latlng()) { //set location if they are valid
            latitude = current_location.lat;
            longitude = current_location.lng;
        }

        // FIXMME
        // Q? The mavlink docs reference being calculated against  29.92 in HG. Presumably it is actually against the US Standard Atmosphere Standard Day
        // need to be sure AP is outputting the correct barometric altitude frame here pressure and temperature assumptions.
        float altitude_barometric = 0.0;
        altitude_barometric = create_altitude(barometer.get_altitude());


        // FIX OTHER GPS DRIVERS to output this too!!!

        // SHould we check if the GPS driver has ellipsoid height available???
        float altitude_geodetic_cm = create_altitude(gps.height_above_ellipsoid() * 10.0);

        // FIX
        // HOW IS TAKEOFF DEFINED IS WE TAKEOFF MANY TIMES?  From the last takeoff or first takeoff?

        // Compute the current height above the takeoff location
        float height_above_takeoff;     // height above takeoff (meters)
        int32_t curr_alt_asml_cm;
        int32_t takeoff_alt_asml_cm;
        if (!current_location.get_alt_cm(Location::AltFrame::ABSOLUTE, curr_alt_asml_cm) ||
            !_takeoff_location.get_alt_cm(Location::AltFrame::ABSOLUTE, takeoff_alt_asml_cm)) {
            height_above_takeoff = ODID_INV_ALT;
        } else {
            height_above_takeoff = (curr_alt_asml_cm - takeoff_alt_asml_cm) * 0.01;
        }
        float height = create_altitude(height_above_takeoff);

        // Accuracy

        // If we have GPS 3D lock we presume that the accuracies of the system will track the GPS's reported accuracy
        MAV_ODID_HOR_ACC horizontal_accuracy_mav = MAV_ODID_HOR_ACC_UNKNOWN;
        MAV_ODID_VER_ACC vertical_accuracy_mav = MAV_ODID_VER_ACC_UNKNOWN;
        MAV_ODID_SPEED_ACC speed_accuracy_mav = MAV_ODID_SPEED_ACC_UNKNOWN;
        MAV_ODID_TIME_ACC timestamp_accuracy_mav = MAV_ODID_TIME_ACC_UNKNOWN;

        float horizontal_accuracy;
        if (gps.horizontal_accuracy(horizontal_accuracy)) {
            horizontal_accuracy_mav = create_enum_horizontal_accuracy(horizontal_accuracy);
        }

        float vertical_accuracy;
        if (gps.vertical_accuracy(vertical_accuracy)) {
            vertical_accuracy_mav = create_enum_vertical_accuracy(vertical_accuracy);
        }

        float speed_accuracy;
        if (gps.speed_accuracy(speed_accuracy)) {
            speed_accuracy_mav = create_enum_speed_accuracy(speed_accuracy);
        }

        // GPS timestamp accuracy will be different between RTK vs non-RTK GPSs
        // FIXME
        // float time_accuracy;
        // if (gps.time_accuracy(time_accuracy)) {
        //     time_accuracy_mav = create_enum_time_accuracy(time_accuracy);
        // }

        timestamp_accuracy_mav =  create_enum_timestamp_accuracy(1.0); //set to 1 sec accuracy

        // Barometer altitude accuraacy will be highly dependent on the airframe and installation of the barometer in use
        // thus ArduPilot cannot reasonably fill this in.
        // Instead allow a manufacturer to use a parameter to fill this in
        uint8_t barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
        if (!is_equal(_baro_accuracy.get(), -1.0f)) {
            barometer_accuracy = create_enum_vertical_accuracy(_baro_accuracy);
        }

        // Timestamp here is the number of seconds after into the current hour referenced to UTC time (up to one hour)

        // FIX we need to only set this if w have a GPS lock is 2D good enough for that? 
        float timestamp = ODID_INV_TIMESTAMP;
        if (!got_bad_gps_fix) {
            uint32_t time_week_ms = gps.time_week_ms();
            timestamp = float(time_week_ms % (3600 * 1000)) * 0.001;

            // FIX This function seems redundant as we the above shouldn't go out of bounds?
            timestamp = create_location_timestamp(timestamp);   //make sure timestamp is within Remote ID limit
        }


        MAV_ODID_STATUS status = get_uav_status();
        const uint8_t* id_or_mac = _id_or_mac;
        mavlink_channel_t mavlink_channel = get_mavlink_channel();

        mavlink_msg_open_drone_id_location_send(
            mavlink_channel,
            0,                                          // System ID (0 for broadcast)
            0,                                          // Component ID (0 for broadcast)
            id_or_mac,                                  // id_or_mac: unused. Only used for drone ID data received from other UAs
            status,
            uint16_t(direction * 100.0),                // Heading (centi-degrees)
            uint16_t(speed_horizontal * 100.0),         // Ground speed (cm/s)
            int16_t(climb_rate * 100.0),                // Climb rate (cm/s)
            latitude,
            longitude,
            altitude_barometric,
            altitude_geodetic_cm,
            MAV_ODID_HEIGHT_REF_OVER_TAKEOFF,           // height reference enum: Above takeoff location or above ground
            height,
            horizontal_accuracy_mav,
            vertical_accuracy_mav,
            barometer_accuracy,
            speed_accuracy_mav,
            timestamp,
            timestamp_accuracy_mav
        );
    }
}

void AP_OpenDroneID::send_basic_id_message() const
{
    mavlink_msg_open_drone_id_basic_id_send(
        _chan,
        0,                      // System ID (0 for broadcast)
        0,                      // Component ID (0 for broadcast)
        _id_or_mac,             // id_or_mac: unused. Only used for drone ID data received from other UAs
        _id_type,
        _ua_type,
        (uint8_t *) _uas_id     //_uas_id is char als in Open Drone ID core library, but MAVLink defines it as uint8_t.
    );
}

void AP_OpenDroneID::send_system_message() const
{
    mavlink_msg_open_drone_id_system_send(
        _chan,
        0,                          // System ID (0 for broadcast)
        0,                          // Component ID (0 for broadcast)
        _id_or_mac,                 // id_or_mac: unused. Only used for drone ID data received from other UAs
        _operator_location_type,
        _classification_type,
        _operator_position.lat,
        _operator_position.lng,
        _area_count,
        _area_radius,
        _area_ceiling,
        _area_floor,
        _category_eu,
        _class_eu,
        _operator_position.alt * 0.01f,     // Geodetic altitude relative to WGS84
        _operator_timestamp
    );
}

void AP_OpenDroneID::send_self_id_message() const
{
    mavlink_msg_open_drone_id_self_id_send(
        _chan,
        0,                  // System ID (0 for broadcast)
        0,                  // Component ID (0 for broadcast)
        _id_or_mac,         // id_or_mac: unused. Only used for drone ID data received from other UAs
        _description_type,
        _description
    );
}

void AP_OpenDroneID::send_operator_id_message() const
{
    mavlink_msg_open_drone_id_operator_id_send(
        _chan,
        0,                  // System ID (0 for broadcast)
        0,                  // Component ID (0 for broadcast)
        _id_or_mac,         // id_or_mac: unused. Only used for drone ID data received from other UAs
        _operator_id_type,
        _operator_id
    );
}

/*
* This converts a horizontal accuracy float value to the corresponding enum
*
* @param Accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_HOR_ACC AP_OpenDroneID::create_enum_horizontal_accuracy(float accuracy) const
{
    // Out of bounds return UKNOWN flag
    if (accuracy < 0.0 || accuracy >= 18520.0) {
        return MAV_ODID_HOR_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_HOR_ACC mavoutput;     // mavlink enum output
    } horiz_accuracy_table[] = {
        { 1.0,    MAV_ODID_HOR_ACC_1_METER},
        { 3.0,    MAV_ODID_HOR_ACC_3_METER},
        {10.0,    MAV_ODID_HOR_ACC_10_METER},
        {30.0,    MAV_ODID_HOR_ACC_30_METER},
        {92.6,    MAV_ODID_HOR_ACC_0_05NM},
        {185.2,   MAV_ODID_HOR_ACC_0_1NM},
        {555.6,   MAV_ODID_HOR_ACC_0_3NM},
        {926.0,   MAV_ODID_HOR_ACC_0_5NM},
        {1852.0,  MAV_ODID_HOR_ACC_1NM},
        {3704.0,  MAV_ODID_HOR_ACC_2NM},
        {7408.0,  MAV_ODID_HOR_ACC_4NM},
        {18520.0, MAV_ODID_HOR_ACC_10NM},
    };

    for (auto elem : horiz_accuracy_table) {
        if (accuracy <= elem.accuracy)
        {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_HOR_ACC_UNKNOWN;
}

/**
* This converts a vertical accuracy float value to the corresponding enum
*
* @param Accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_VER_ACC AP_OpenDroneID::create_enum_vertical_accuracy(float accuracy) const
{
    // Out of bounds return UKNOWN flag
    if (accuracy < 0.0 || accuracy >= 150.0) {
        return MAV_ODID_VER_ACC_UNKNOWN;
    }

    static const struct {
        float accuracy;                 // Accuracy bound in meters
        MAV_ODID_VER_ACC mavoutput;     // mavlink enum output
    } vertical_accuracy_table[] = {
        { 1.0,  MAV_ODID_VER_ACC_1_METER},
        { 3.0,  MAV_ODID_VER_ACC_3_METER},
        {10.0,  MAV_ODID_VER_ACC_10_METER},
        {25.0,  MAV_ODID_VER_ACC_25_METER},
        {45.0,  MAV_ODID_VER_ACC_45_METER},
        {150.0, MAV_ODID_VER_ACC_150_METER},
    };

    for (auto elem : vertical_accuracy_table) {
        if (accuracy <= elem.accuracy)
        {
            return elem.mavoutput;
        }
    }

    // Should not reach this
    return MAV_ODID_VER_ACC_UNKNOWN;
}

/**
* This converts a speed accuracy float value to the corresponding enum
*
* @param Accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
MAV_ODID_SPEED_ACC AP_OpenDroneID::create_enum_speed_accuracy(float accuracy) const
{
    // Out of bounds return UKNOWN flag
    if (accuracy < 0.0 || accuracy >= 10.0) {
        return MAV_ODID_SPEED_ACC_UNKNOWN;
    }

    if (accuracy < 0.3)
        return MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND;
    else if (accuracy < 1.0)
        return MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;
    else if (accuracy < 3.0)
        return MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND;
    else if (accuracy < 10.0)
        return MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND;

    // Should not reach this
    return MAV_ODID_SPEED_ACC_UNKNOWN;
}

/**
* This converts a timestamp accuracy float value to the corresponding enum
*
* @param Accuracy The timestamp accuracy in seconds
* @return Enum value representing the accuracy
*/
MAV_ODID_TIME_ACC AP_OpenDroneID::create_enum_timestamp_accuracy(float accuracy) const
{
    // Out of bounds return UKNOWN flag
    if (accuracy < 0.0 || accuracy >= 1.5) {
        return MAV_ODID_TIME_ACC_UNKNOWN;
    }

    MAV_ODID_TIME_ACC mavoutput [15] = {
        MAV_ODID_TIME_ACC_0_1_SECOND,
        MAV_ODID_TIME_ACC_0_2_SECOND,
        MAV_ODID_TIME_ACC_0_3_SECOND,
        MAV_ODID_TIME_ACC_0_4_SECOND,
        MAV_ODID_TIME_ACC_0_5_SECOND,
        MAV_ODID_TIME_ACC_0_6_SECOND,
        MAV_ODID_TIME_ACC_0_7_SECOND,
        MAV_ODID_TIME_ACC_0_8_SECOND,
        MAV_ODID_TIME_ACC_0_9_SECOND,
        MAV_ODID_TIME_ACC_1_0_SECOND,
        MAV_ODID_TIME_ACC_1_1_SECOND,
        MAV_ODID_TIME_ACC_1_2_SECOND,
        MAV_ODID_TIME_ACC_1_3_SECOND,
        MAV_ODID_TIME_ACC_1_4_SECOND,
        MAV_ODID_TIME_ACC_1_5_SECOND,
    };

    for (int8_t i = 1; i <= 15; i++) {
        if (accuracy <= 0.1 * i)
        {
            return mavoutput[i-1];
        }
    }

    // Should not reach this
    return MAV_ODID_TIME_ACC_UNKNOWN;
}

// Only reason we would need this is for cases where we cannot provide an accurate heading
// otherwise we can ensure it stays wrapped from 0 to 360deg

// make sure value is within limits of remote ID standard
// uint16_t AP_OpenDroneID::create_direction(uint16_t direction) const
// {
//     if (direction > ODID_MAX_DIR) { // constraint function can't be used, because out of range value is invalid
//         direction = ODID_INV_DIR;
//     }

//     return direction;
// }

 // make sure value is within limits of remote ID standard
uint16_t AP_OpenDroneID::create_speed_horizontal(uint16_t speed) const
{
    if (speed > ODID_MAX_SPEED_H) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_H;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
int16_t AP_OpenDroneID::create_speed_vertical(int16_t speed) const
{
    if (speed > ODID_MAX_SPEED_V) { // constraint function can't be used, because out of range value is invalid
        speed = ODID_INV_SPEED_V;
    }
    else if (speed < ODID_MIN_SPEED_V) {
        speed = ODID_INV_SPEED_V;
    }

    return speed;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_altitude(float altitude) const
{
    if (altitude > ODID_MAX_ALT) { // constraint function can't be used, because out of range value is invalid
        altitude = ODID_INV_ALT;
    }
    else if (altitude < ODID_MIN_ALT) {
        altitude = ODID_INV_ALT;
    }

    return altitude;
}

// make sure value is within limits of remote ID standard
float AP_OpenDroneID::create_location_timestamp(float timestamp) const
{
    if (timestamp > ODID_MAX_TIMESTAMP) { // constraint function can't be used, because out of range value is invalid
        timestamp = ODID_INV_TIMESTAMP;
    }
    else if (timestamp < 0) {
        timestamp = ODID_INV_TIMESTAMP;
    }

    return timestamp;
}

// singleton instance
AP_OpenDroneID *AP_OpenDroneID::_singleton;

namespace AP {

AP_OpenDroneID &opendroneid()
{
    return *AP_OpenDroneID::get_singleton();
}

}
#endif //AP_OPENDRONEID_ENABLED
