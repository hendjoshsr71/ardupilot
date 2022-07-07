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
     _ua_type = MAV_ODID_UA_TYPE_NONE;
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


//code to be replaced: set dummy values in open drone ID messages
    set_mavlink_channel(MAVLINK_COMM_1); //set to MAVLink channel 1
    set_mavlink_dynamic_messages_period_ms(1000); //send dynamic messages at 1 Hz frequency i.e. every 1 second
    set_mavlink_static_messages_period_ms(10000); //send dynamic messages at 0.1 Hz frequency i.e. every 10 seconds
    set_id_type(MAV_ODID_ID_TYPE_SERIAL_NUMBER);
    set_ua_type(MAV_ODID_UA_TYPE_AEROPLANE);

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

void AP_OpenDroneID::send_location_message()
{
    //all open_drone_id send functions use data stored in the open drone id class.
    //This location send function is an exception. It uses live location data from the ArduPilot system.

    AP_AHRS &ahrs = AP::ahrs();

    Location current_location;
    if (ahrs.get_location(current_location))
    {
        set_uav_status(MAV_ODID_STATUS_UNDECLARED);
        uint16_t direction = (uint16_t) degrees(AP::ahrs().get_gyro().x); //x = roll = heading
        direction = create_direction(direction); //make sure direction is within Remote ID limits

        uint16_t speed_horizontal = 0;
        const Vector2f speed = AP::ahrs().groundspeed_vector();
        if (!speed.is_nan() && !speed.is_zero()) {
           speed_horizontal = (uint16_t) sqrt(speed.x*speed.x + speed.y*speed.y);
           speed_horizontal = create_speed_horizontal(speed_horizontal); //make sure speed is within Remote ID limits
           set_uav_status(MAV_ODID_STATUS_AIRBORNE);
        }
        else {
            speed_horizontal = 0;
            speed_horizontal = create_speed_horizontal(speed_horizontal); //make sure speed is within Remote ID limits
            set_uav_status(MAV_ODID_STATUS_GROUND);
        }

        int32_t latitude = 0;
        int32_t longitude = 0;
        if (current_location.check_latlng()) { //set location if they are valid
            latitude = current_location.lat;
            longitude = current_location.lng;
        }

        float altitude_barometric = 0;
        const AP_Baro &barometer = AP::baro();

        altitude_barometric = create_altitude(barometer.get_altitude());
        float altitude_geodetic = create_altitude(current_location.alt * 0.01f);

        //needs improvement in future PRs
        set_height_reference(MAV_ODID_HEIGHT_REF_OVER_TAKEOFF); //or MAV_ODID_HEIGHT_REF_OVER_GROUND if we have WGS-84 position(?)
        float height = create_altitude((current_location.alt - _operator_position.alt)*0.01f);
                                                               //height needs to be set depending on the height reference.
                                                               //If set to MAV_ODID_HEIGHT_REF_OVER_TAKEOFF, height is the difference,
                                                               //between current altitude and take off height

//set to dummy values as AHRS class does not provide these metrics
        int16_t speed_vertical = 0;
        speed_vertical = create_speed_vertical(speed_vertical); //make sure speed is within Remote ID limit
        uint8_t horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
        horizontal_accuracy = create_enum_horizontal_accuracy(5.0); //set to 5.0 meter accuracy and use helper function to determine accuracy type.
        uint8_t vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
        vertical_accuracy = create_enum_vertical_accuracy(5.0); //set to 5.0 meter accuracy and use helper function to determine accuracy type.
        uint8_t barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
        barometer_accuracy = create_enum_vertical_accuracy(1.0); //set to 1.0 meter accuracy and use helper function to determine accuracy type.
        uint8_t speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN; //ahrs class does not provide accuracy readings
        speed_accuracy = create_enum_speed_accuracy(3.0); //set to 3.0 meter accuracy and use helper function to determine accuracy type.
        uint8_t timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;
        timestamp_accuracy =  create_enum_timestamp_accuracy(1.0); //set to 1 sec accuracy
//end dummy values

        //timestamp is in format seconds (up to one hour).microseconds e.g. 164.47, Maximum value = 3600
        uint32_t seconds = (AP_HAL::millis()/1000) % 3600;
        uint32_t milli_seconds = ((AP_HAL::millis()) % 1000);
        float timestamp = (float) seconds + (float) milli_seconds/1000.0;
        timestamp = create_location_timestamp(timestamp); //make sure timestamp is within Remote ID limit

        MAV_ODID_STATUS status = get_uav_status();
        MAV_ODID_HEIGHT_REF height_reference = get_height_reference();
        const uint8_t* id_or_mac = _id_or_mac;
        mavlink_channel_t mavlink_channel = get_mavlink_channel();

        mavlink_msg_open_drone_id_location_send(
            mavlink_channel,
            0,                  // System ID (0 for broadcast)
            0,                  // Component ID (0 for broadcast)
            id_or_mac,                  // id_or_mac: unused. Only used for drone ID data received from other UAs
            status,
            direction,
            speed_horizontal,
            speed_vertical,
            latitude,
            longitude,
            altitude_barometric,
            altitude_geodetic,
            height_reference,
            height,
            horizontal_accuracy,
            vertical_accuracy,
            barometer_accuracy,
            speed_accuracy,
            timestamp,
            timestamp_accuracy
        );
    }
}

void AP_OpenDroneID::send_basic_id_message() const
{
    mavlink_msg_open_drone_id_basic_id_send(
        _chan,
        0,                  // System ID (0 for broadcast)
        0,                  // Component ID (0 for broadcast)
        _id_or_mac,          // id_or_mac: unused. Only used for drone ID data received from other UAs
        _id_type,
        _ua_type,
        (uint8_t *) _uas_id   //_uas_id is char als in Open Drone ID core library, but MAVLink defines it as uint8_t.
    );
}

void AP_OpenDroneID::send_system_message() const
{
    mavlink_msg_open_drone_id_system_send(
        _chan,
        0,                  // System ID (0 for broadcast)
        0,                  // Component ID (0 for broadcast)
        _id_or_mac,          // id_or_mac: unused. Only used for drone ID data received from other UAs
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
        _operator_position.alt*0.01f,
        _operator_timestamp
    );
}

void AP_OpenDroneID::send_self_id_message() const
{
    mavlink_msg_open_drone_id_self_id_send(
        _chan,
        0,                  // System ID (0 for broadcast)
        0,                  // Component ID (0 for broadcast)
        _id_or_mac,          // id_or_mac: unused. Only used for drone ID data received from other UAs
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
        _id_or_mac,          // id_or_mac: unused. Only used for drone ID data received from other UAs
        _operator_id_type,
        _operator_id
    );
}

/**
* This converts a horizontal accuracy float value to the corresponding enum
*
* @param Accuracy The horizontal accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_HOR_ACC AP_OpenDroneID::create_enum_horizontal_accuracy(float Accuracy) const
{
    if (Accuracy >= 18520.0f)
        return MAV_ODID_HOR_ACC_UNKNOWN;
    else if (Accuracy >= 7408.0f)
        return MAV_ODID_HOR_ACC_10NM;
    else if (Accuracy >= 3704.0f)
        return MAV_ODID_HOR_ACC_4NM;
    else if (Accuracy >= 1852.0f)
        return MAV_ODID_HOR_ACC_2NM;
    else if (Accuracy >= 926.0f)
        return MAV_ODID_HOR_ACC_1NM;
    else if (Accuracy >= 555.6f)
        return MAV_ODID_HOR_ACC_0_5NM;
    else if (Accuracy >= 185.2f)
        return MAV_ODID_HOR_ACC_0_3NM;
    else if (Accuracy >= 92.6f)
        return MAV_ODID_HOR_ACC_0_1NM;
    else if (Accuracy >= 30.0f)
        return MAV_ODID_HOR_ACC_0_05NM;
    else if (Accuracy >= 10.0f)
        return MAV_ODID_HOR_ACC_30_METER;
    else if (Accuracy >= 3.0f)
        return MAV_ODID_HOR_ACC_10_METER;
    else if (Accuracy >= 1.0f)
        return MAV_ODID_HOR_ACC_3_METER;
    else if (Accuracy > 0.0f)
        return MAV_ODID_HOR_ACC_1_METER;
    else
        return MAV_ODID_HOR_ACC_UNKNOWN;
}

/**
* This converts a vertical accuracy float value to the corresponding enum
*
* @param Accuracy The vertical accuracy in meters
* @return Enum value representing the accuracy
*/
MAV_ODID_VER_ACC AP_OpenDroneID::create_enum_vertical_accuracy(float Accuracy) const
{
    if (Accuracy >= 150.0f)
        return MAV_ODID_VER_ACC_UNKNOWN;
    else if (Accuracy >= 45.0f)
        return MAV_ODID_VER_ACC_150_METER;
    else if (Accuracy >= 25.0f)
        return MAV_ODID_VER_ACC_45_METER;
    else if (Accuracy >= 10.0f)
        return MAV_ODID_VER_ACC_25_METER;
    else if (Accuracy >= 3.0f)
        return MAV_ODID_VER_ACC_10_METER;
    else if (Accuracy >= 1.0f)
        return MAV_ODID_VER_ACC_3_METER;
    else if (Accuracy > 0.0f)
        return MAV_ODID_VER_ACC_1_METER;
    else
        return MAV_ODID_VER_ACC_UNKNOWN;
}

/**
* This converts a speed accuracy float value to the corresponding enum
*
* @param Accuracy The speed accuracy in m/s
* @return Enum value representing the accuracy
*/
MAV_ODID_SPEED_ACC AP_OpenDroneID::create_enum_speed_accuracy(float Accuracy) const
{
    if (Accuracy >= 10.0f)
        return MAV_ODID_SPEED_ACC_UNKNOWN;
    else if (Accuracy >= 3.0f)
        return MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND;
    else if (Accuracy >= 1.0f)
        return MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND;
    else if (Accuracy >= 0.3f)
        return MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND;
    else if (Accuracy > 0.0f)
        return MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND;
    else
        return MAV_ODID_SPEED_ACC_UNKNOWN;
}

/**
* This converts a timestamp accuracy float value to the corresponding enum
*
* @param Accuracy The timestamp accuracy in seconds
* @return Enum value representing the accuracy
*/
MAV_ODID_TIME_ACC AP_OpenDroneID::create_enum_timestamp_accuracy(float Accuracy) const
{
    if (Accuracy > 1.5f)
        return MAV_ODID_TIME_ACC_UNKNOWN;
    else if (Accuracy > 1.4f)
        return MAV_ODID_TIME_ACC_1_5_SECOND;
    else if (Accuracy > 1.3f)
        return MAV_ODID_TIME_ACC_1_4_SECOND;
    else if (Accuracy > 1.2f)
        return MAV_ODID_TIME_ACC_1_3_SECOND;
    else if (Accuracy > 1.1f)
        return MAV_ODID_TIME_ACC_1_2_SECOND;
    else if (Accuracy > 1.0f)
        return MAV_ODID_TIME_ACC_1_1_SECOND;
    else if (Accuracy > 0.9f)
        return MAV_ODID_TIME_ACC_1_0_SECOND;
    else if (Accuracy > 0.8f)
        return MAV_ODID_TIME_ACC_0_9_SECOND;
    else if (Accuracy > 0.7f)
        return MAV_ODID_TIME_ACC_0_8_SECOND;
    else if (Accuracy > 0.6f)
        return MAV_ODID_TIME_ACC_0_7_SECOND;
    else if (Accuracy > 0.5f)
        return MAV_ODID_TIME_ACC_0_6_SECOND;
    else if (Accuracy > 0.4f)
        return MAV_ODID_TIME_ACC_0_5_SECOND;
    else if (Accuracy > 0.3f)
        return MAV_ODID_TIME_ACC_0_4_SECOND;
    else if (Accuracy > 0.2f)
        return MAV_ODID_TIME_ACC_0_3_SECOND;
    else if (Accuracy > 0.1f)
        return MAV_ODID_TIME_ACC_0_2_SECOND;
    else if (Accuracy > 0.0f)
        return MAV_ODID_TIME_ACC_0_1_SECOND;
    else
        return MAV_ODID_TIME_ACC_UNKNOWN;
}

// make sure value is within limits of remote ID standard
uint16_t AP_OpenDroneID::create_direction(uint16_t direction) const
{
    if (direction > ODID_MAX_DIR) { // constraint function can't be used, because out of range value is invalid
        direction = ODID_INV_DIR;
    }

    return direction;
}

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
