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
 * sim_vehicle.py --wipe-eeprom --console --map -A --serial1=uart:/dev/ttyUSB1:9600
 * (and a DroneBeacon MAVLink transponder connected to ttyUSB1)
 *
 * The Remote ID implementation expects a transponder that caches the received MAVLink messages from ArduPilot
 * and transmits them at the required intervals. So static messages are only send once to the transponder.
 */

#pragma once

#ifndef AP_OPENDRONEID_ENABLED
#define AP_OPENDRONEID_ENABLED 1
#endif

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Common/Location.h>

#define ODID_ID_SIZE 20
#define ODID_STR_SIZE 23

#define ODID_MIN_DIR         0       // Minimum direction
#define ODID_MAX_DIR         360     // Maximum direction
#define ODID_INV_DIR         361     // Invalid direction
#define ODID_MIN_SPEED_H     0       // Minimum speed horizontal
#define ODID_MAX_SPEED_H     254.25f // Maximum speed horizontal
#define ODID_INV_SPEED_H     255     // Invalid speed horizontal
#define ODID_MIN_SPEED_V     (-62)   // Minimum speed vertical
#define ODID_MAX_SPEED_V     62      // Maximum speed vertical
#define ODID_INV_SPEED_V     63      // Invalid speed vertical
#define ODID_MIN_ALT         (-1000) // Minimum altitude
#define ODID_MAX_ALT         31767.5f// Maximum altitude
#define ODID_INV_ALT         ODID_MIN_ALT // Invalid altitude
#define ODID_MAX_TIMESTAMP   (60 * 60)
#define ODID_INV_TIMESTAMP   0xFFFF  // Invalid, No Value or Unknown Timestamp
#define ODID_MAX_AREA_RADIUS 2550
#define ODID_AREA_COUNT_MIN  1
#define ODID_AREA_COUNT_MAX  65000

class AP_OpenDroneID {
public:
    AP_OpenDroneID();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_OpenDroneID);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    bool transmit_basic_ID_message;  //there are small differences in Remote ID standards https://github.com/opendroneid/opendroneid-core-c#comparison
    bool transmit_location_message;  //this means that for instance in the EU the operator ID message is compulsory, other areas it is optional
    bool transmit_self_ID_message;   //also the basic ID message (serial number) could be handled by the transponder itself
    bool transmit_system_message;    //so we need to have bool variables to control which MAVLink packages are sent
    bool transmit_operator_id_message;

    void init();
    void update();
    void send_dynamic_out();
    void send_static_out();

    // get methods (not sure if we need those)
    MAV_ODID_ID_TYPE get_id_type() const { return _id_type; }

    MAV_ODID_UA_TYPE get_ua_type() const { return _ua_type; }

    const uint8_t* get_id_or_mac() const { return _id_or_mac; }

    const char* get_uas_id() const { return _uas_id; }

    MAV_ODID_STATUS get_uav_status() const { return _uav_status; }

    MAV_ODID_HEIGHT_REF get_height_reference() const { return _height_reference; }

    MAV_ODID_OPERATOR_LOCATION_TYPE get_operator_location_type() const { return _operator_location_type; }

    MAV_ODID_CLASSIFICATION_TYPE get_classification_type() const { return _classification_type; }

    MAV_ODID_CATEGORY_EU get_category_eu() const { return _category_eu; }

    MAV_ODID_CLASS_EU get_class_eu() const { return _class_eu; }

    MAV_ODID_OPERATOR_ID_TYPE get_operator_id_type() const { return _operator_id_type; }

    const char * get_operator_id() const { return _operator_id; }

    MAV_ODID_DESC_TYPE get_description_type() const { return _description_type; }

    const char * get_description() const { return _description; }

    int32_t get_operator_latitude() const { return _operator_position.lat; }

    int32_t get_operator_longitude() const { return _operator_position.lng; }

    uint32_t get_operator_timestamp() const { return _operator_timestamp; }

    uint16_t get_area_count() const { return _area_count; }

    uint16_t get_area_radius() const { return _area_radius; }

    float get_area_ceiling() const { return _area_ceiling; }

    float get_area_floor() const { return _area_floor; }

    float get_operator_altitude_geo() const { return _operator_position.alt; }

    uint8_t get_mavlink_dynamic_messages_period() const { return _mavlink_dynamic_period_ms; }

    uint8_t get_mavlink_static_messages_period() const { return _mavlink_static_period_ms; }

    mavlink_channel_t get_mavlink_channel() const { return _chan; }

    //set methods
    void set_id_or_mac(uint8_t* id_or_mac) {
        //basically id_or_mac is basically a string, but is defined as uint8_t, so use strncpy
        memset(_id_or_mac, 0, ODID_ID_SIZE + 1); //just in case, according to strncpy definition, the source is padded with zeroes if source length is shorter
        strncpy((char*) _id_or_mac,(char*) id_or_mac, ODID_ID_SIZE + 1);
    }

    void set_uas_id(char* uas_id) {
        memset(_uas_id, 0, ODID_ID_SIZE + 1); //just in case, according to strncpy definition, the source is padded with zeroes if source length is shorter
        strncpy(_uas_id,uas_id,ODID_ID_SIZE + 1);
    }

    void set_id_type(MAV_ODID_ID_TYPE id_type) {
        //check if id_type is valid
        _id_type = (MAV_ODID_ID_TYPE) constrain_uint16(id_type, MAV_ODID_ID_TYPE_NONE, MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID);
    }

    void set_ua_type(MAV_ODID_UA_TYPE ua_type) {
        //check if ua_type is valid
         _ua_type = (MAV_ODID_UA_TYPE) constrain_uint16(ua_type, MAV_ODID_UA_TYPE_NONE, MAV_ODID_UA_TYPE_OTHER);
    }

    void set_height_reference(MAV_ODID_HEIGHT_REF reference) {
        //check if reference is valid
        _height_reference = (MAV_ODID_HEIGHT_REF) constrain_uint16(reference, MAV_ODID_HEIGHT_REF_OVER_TAKEOFF, MAV_ODID_HEIGHT_REF_OVER_GROUND);
    }

    void set_uav_status(MAV_ODID_STATUS status) {
        //check if status is valid
        _uav_status = (MAV_ODID_STATUS) constrain_uint16(status, MAV_ODID_STATUS_UNDECLARED, MAV_ODID_STATUS_EMERGENCY);
    }

    void set_operator_location_type(MAV_ODID_OPERATOR_LOCATION_TYPE operator_location_type) {
        //check if operator_location_type is valid
        _operator_location_type = (MAV_ODID_OPERATOR_LOCATION_TYPE) constrain_uint16(operator_location_type, MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF, MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED);
    }

    void set_classification_type(MAV_ODID_CLASSIFICATION_TYPE classification_type) {
        //check if classification_type is valid
        _classification_type = (MAV_ODID_CLASSIFICATION_TYPE) constrain_uint16(classification_type, MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED, MAV_ODID_CLASSIFICATION_TYPE_EU);
    }

    void set_category_eu(MAV_ODID_CATEGORY_EU category_eu) {
        //check if category_eu is valid
        _category_eu = (MAV_ODID_CATEGORY_EU) constrain_uint16(category_eu, MAV_ODID_CATEGORY_EU_UNDECLARED, MAV_ODID_CATEGORY_EU_CERTIFIED);
    }

    void set_class_eu(MAV_ODID_CLASS_EU class_eu) {
        //check if class_eu is valid
        _class_eu = (MAV_ODID_CLASS_EU) constrain_uint16(class_eu, MAV_ODID_CLASS_EU_UNDECLARED, MAV_ODID_CLASS_EU_CLASS_6);
    }

    void set_operator_id_type(MAV_ODID_OPERATOR_ID_TYPE type) {
    //for now _operator_id_type can only be MAV_ODID_OPERATOR_ID_TYPE_CAA
        _operator_id_type = (MAV_ODID_OPERATOR_ID_TYPE) constrain_uint16(type, MAV_ODID_OPERATOR_ID_TYPE_CAA, MAV_ODID_OPERATOR_ID_TYPE_CAA);
    }

    void set_operator_id(char* id) {
        memset(_operator_id, 0, ODID_ID_SIZE + 1); //just in case, according to strncpy definition, the source is padded with zeroes if source length is shorter
        strncpy(_operator_id, id, ODID_ID_SIZE + 1);
    }

    void set_description_type(MAV_ODID_DESC_TYPE type) {
        //for now _description_type can only be MAV_ODID_DESC_TYPE_TEXT
        _description_type = (MAV_ODID_DESC_TYPE) constrain_uint16(type, MAV_ODID_DESC_TYPE_TEXT, MAV_ODID_DESC_TYPE_TEXT);
    }
    void set_description(char* description) {
        memset(_description, 0, ODID_STR_SIZE + 1); //just in case, according to strncpy definition, the source is padded with zeroes if source length is shorter
        strncpy(_description, description, ODID_STR_SIZE + 1);
    }

    void set_operator_latitude(int32_t latitude) {
        _operator_position.lat = latitude;
    }

    void set_operator_longitude(int32_t longitude) {
        _operator_position.lng = longitude;
    }

    void set_operator_timestamp(time_t epoch_timestamp) {
        uint32_t timestamp_2019 = (uint32_t) epoch_timestamp - (uint32_t) 1546300800; //1546300800 = 1/1/2019 00:00:00 UTC
        _operator_timestamp = constrain_uint32(timestamp_2019, 0, UINT32_MAX);
    }

    void set_area_count(uint16_t count) {
        _area_count = constrain_uint16(count, ODID_AREA_COUNT_MIN, ODID_AREA_COUNT_MAX);
    }

    void set_area_radius(uint16_t radius) {
        _area_radius = constrain_uint16(radius, 0, ODID_MAX_AREA_RADIUS);
    }

    void set_area_ceiling(float ceiling) {
        _area_ceiling = constrain_float(ceiling, ODID_MIN_ALT, ODID_MAX_ALT);
    }

    void set_area_floor(float floor) {
         _area_floor = constrain_float(floor, ODID_MIN_ALT, ODID_MAX_ALT);
    }

    void set_operator_altitude_geo(float altitude){
        altitude = constrain_float(altitude, ODID_MIN_ALT, ODID_MAX_ALT);
        _operator_position.alt = (int32_t) altitude*100.0f;
    }

    void set_mavlink_dynamic_messages_period_ms(uint32_t period_ms){
       _mavlink_dynamic_period_ms = constrain_int32(period_ms, 100, 1000); //between 100ms and 1 seconds, Remote ID standard specifies location data to be broadcasted every 1 second or faster.
    }

    void set_mavlink_static_messages_period_ms(uint32_t period_ms){
       _mavlink_static_period_ms = constrain_int32(period_ms, 1000, 360000); //between 1 and 1 hour, Remote ID transponder needs to broadcast the static packages every 3 seconds or faster.
    }

    void set_mavlink_channel(mavlink_channel_t chan){
        _chan = chan;
    }

    //transmit functions to manually send a static MAVLink message
    void send_basic_id_message() const;
    void send_system_message() const;
    void send_self_id_message() const;
    void send_operator_id_message() const;
    void send_location_message();

    //helper functions
    MAV_ODID_HOR_ACC create_enum_horizontal_accuracy(float Accuracy) const;
    MAV_ODID_VER_ACC create_enum_vertical_accuracy(float Accuracy) const;
    MAV_ODID_SPEED_ACC create_enum_speed_accuracy(float Accuracy) const;
    MAV_ODID_TIME_ACC create_enum_timestamp_accuracy(float Accuracy) const;
    uint16_t create_direction(uint16_t direction) const;
    uint16_t create_speed_horizontal(uint16_t speed) const;
    int16_t create_speed_vertical(int16_t speed) const;
    float create_altitude(float altitude) const;
    float create_location_timestamp(float timestamp) const;

    // get singleton instance
    static AP_OpenDroneID *get_singleton() { return _singleton; }

private:
    static AP_OpenDroneID *_singleton;

    // parameters
    AP_Int32 _serial_number; // do we need this parameter?
    mavlink_channel_t _chan; // MAVLink channel that communicates with the Remote ID Transceiver
    uint32_t _last_send_dynamic_messages_ms;
    uint32_t _last_send_static_messages_ms;
    uint32_t _mavlink_dynamic_period_ms; //how often are mavlink dynamic messages sent in ms. E.g. 1000 = 1 Hz
    uint32_t _mavlink_static_period_ms; //how often are mavlink static messages sent in ms

    MAV_ODID_ID_TYPE _id_type;
    MAV_ODID_UA_TYPE _ua_type;
    MAV_ODID_STATUS _uav_status;
    MAV_ODID_HEIGHT_REF _height_reference;

    struct Location _operator_position;
    uint32_t _operator_timestamp;

    uint16_t _area_count;
    uint16_t _area_radius;
    float _area_ceiling;
    float _area_floor;

    MAV_ODID_OPERATOR_LOCATION_TYPE _operator_location_type;
    MAV_ODID_CLASSIFICATION_TYPE _classification_type;
    MAV_ODID_CATEGORY_EU _category_eu;
    MAV_ODID_CLASS_EU _class_eu;
    MAV_ODID_OPERATOR_ID_TYPE _operator_id_type;
    MAV_ODID_DESC_TYPE _description_type;

    uint8_t _id_or_mac[ODID_ID_SIZE + 1];
    char _uas_id[ODID_ID_SIZE + 1];
    char _operator_id[ODID_ID_SIZE + 1];
    char _description[ODID_STR_SIZE + 1];
};

namespace AP {
    AP_OpenDroneID &opendroneid();
};
