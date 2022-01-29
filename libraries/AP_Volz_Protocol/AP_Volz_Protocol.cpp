/*
 * AP_VOLZ_PROTOCOL.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: guy
 * 
 * January 2022: Added Volz DA26/30 RS485 Protocol & UAVOS & Volz ICD R485 Protocol
 */
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

#include "AP_Volz_Protocol.h"
#if NUM_SERVO_CHANNELS

#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Volz_Protocol::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("MASK",  1, AP_Volz_Protocol, bitmask, 0),

    // @Param: TYPE
    // @DisplayName: Volz Protocol Type
    // @Description: Sets the Volz Servo Protocol Type
    // @Bitmask: 0:Disabled, 1:DA26_RS485, 2:UAVOS_VOLZ_RS485_ICD, 3:Volz_Extended_Position
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE",  2, AP_Volz_Protocol, _protocol, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RATE
    // @DisplayName: Volz Maximum Update Rate (Hz)
    // @Description: Sets the maximum update rate to send new position targets to the servos (Note: this applies to all of the servos)
    // @Units: Hz
    // @Increment: 1
    // @Range: 50 400
    // @User: Standard
    AP_GROUPINFO("RATE",  3, AP_Volz_Protocol, _update_rate, 100),

    // Leave room for additional general parameters here
    // SKIP index 4 -10

    // Due to the Param name 16-char length limit there are only 5 chars for servo number plus descriptor
    // SERVO_VOLZ_1MIN <---> SERVO_VOLZ_16MIN

    // @Param: 1MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("1MIN", 11, AP_Volz_Protocol, _servo_angle_min[0], -180),

    // @Param: 1MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("1MAX", 12, AP_Volz_Protocol, _servo_angle_max[0], 180),

    // @Param: 2MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2MIN", 13, AP_Volz_Protocol, _servo_angle_min[1], -180),

    // @Param: 2MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2MAX", 14, AP_Volz_Protocol, _servo_angle_max[1], 180),

    // @Param: 3MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("3MIN", 15, AP_Volz_Protocol, _servo_angle_min[2], -180),

    // @Param: 3MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("3MAX", 16, AP_Volz_Protocol, _servo_angle_max[2], 180),

    // @Param: 4MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("4MIN", 17, AP_Volz_Protocol, _servo_angle_min[3], -180),

    // @Param: 4MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("4MAX", 18, AP_Volz_Protocol, _servo_angle_max[3], 180),

    // @Param: 5MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("5MIN", 19, AP_Volz_Protocol, _servo_angle_min[4], -180),

    // @Param: 5MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("5MAX", 20, AP_Volz_Protocol, _servo_angle_max[4], 180),

    // @Param: 6MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("6MIN", 21, AP_Volz_Protocol, _servo_angle_min[5], -180),

    // @Param: 6MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("6MAX", 22, AP_Volz_Protocol, _servo_angle_max[5], 180),

    // @Param: 7MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("7MIN", 23, AP_Volz_Protocol, _servo_angle_min[6], -180),

    // @Param: 7MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("7MAX", 24, AP_Volz_Protocol, _servo_angle_max[6], 180),

    // @Param: 8MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("8MIN", 25, AP_Volz_Protocol, _servo_angle_min[7], -180),

    // @Param: 8MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("8MAX", 26, AP_Volz_Protocol, _servo_angle_max[7], 180),

    // @Param: 9MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("9MIN", 27, AP_Volz_Protocol, _servo_angle_min[8], -180),

    // @Param: 9MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("9MAX", 28, AP_Volz_Protocol, _servo_angle_max[8], 180),

    // @Param: 10MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("10MIN", 29, AP_Volz_Protocol, _servo_angle_min[9], -180),

    // @Param: 10MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("10MAX", 30, AP_Volz_Protocol, _servo_angle_max[9], 180),

    // @Param: 11MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("11MIN", 31, AP_Volz_Protocol, _servo_angle_min[10], -180),

    // @Param: 11MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("11MAX", 32, AP_Volz_Protocol, _servo_angle_max[10], 180),

    // @Param: 12MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("12MIN", 33, AP_Volz_Protocol, _servo_angle_min[11], -180),

    // @Param: 12MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("12MAX", 34, AP_Volz_Protocol, _servo_angle_max[11], 180),

    // @Param: 13MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("13MIN", 35, AP_Volz_Protocol, _servo_angle_min[12], -180),

    // @Param: 13MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("13MAX", 36, AP_Volz_Protocol, _servo_angle_max[12], 180),

    // @Param: 14MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("14MIN", 37, AP_Volz_Protocol, _servo_angle_min[13], -180),

    // @Param: 14MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("14MAX", 38, AP_Volz_Protocol, _servo_angle_max[13], 180),

    // @Param: 15MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("15MIN", 39, AP_Volz_Protocol, _servo_angle_min[14], -180),

    // @Param: 15MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("15MAX", 40, AP_Volz_Protocol, _servo_angle_max[14], 180),

    // @Param: 16MIN
    // @DisplayName: Minimum Angle
    // @Description: Minimum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("16MIN", 41, AP_Volz_Protocol, _servo_angle_min[15], -180),

    // @Param: 16MAX
    // @DisplayName: Maximum Angle
    // @Description: Maximum servo angle (degrees): Used to linearize the mapping from PWM to desired output angle.
    // @Units: deg
    // @Range: -180 180
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("16MAX", 42, AP_Volz_Protocol, _servo_angle_max[15], 180),

    AP_GROUPEND
};

// constructor
AP_Volz_Protocol::AP_Volz_Protocol(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Volz_Protocol::init(void)
{
    // Return if Volz Protocol is Disabled
    if (Protocol(_protocol.get()) == Protocol::DISABLED) {
        return;
    }
    
    AP_SerialManager &serial_manager = AP::serialmanager();
    port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Volz,0);

    if (port) {
        const uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Robotis, 0);
        us_per_byte = 10.5 * 1e6 / baudrate;  // 91.1 usec with 115.2K baud 
        us_gap = 4.0 * 1e6 / baudrate;        // 34.722 usec with 115.2K baud
    }

    update_volz_bitmask(bitmask);

    // Protocol Changes Require Rebooting
    update_protocol_registers(_protocol);
}

void AP_Volz_Protocol::update()
{
    if (!initialised) {
        initialised = true;
        init();
        last_volz_update_time = AP_HAL::micros();
    }

    if (port == nullptr) {
        return;
    }

    if (last_used_bitmask != uint32_t(bitmask.get())) {
        update_volz_bitmask(bitmask);
    }

    // The volz_time_frame is updated inside of the update_volz_bitmask()
    // this limits the maximum update rate based upon: _update_rate, # of channels, safety factor, & average transmission time
    const uint32_t now = AP_HAL::micros();
    if (now - last_volz_update_time < delay_time_us) {
        return;
    }

    // REMOVE OR RETURN TO ABOVE
    // Prepare to Remove this txspace check we already have timing checks...
    if (port->txspace() < VOLZ_DATA_FRAME_SIZE) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "VOLZ Port%i: out of space \n", AP::serialmanager().find_portnum(AP_SerialManager::SerialProtocol_Volz, 0));
        return;
    }

    last_volz_update_time = now;
    delay_time_us = 0;

    // loop for all servo channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        // check if current channel is needed for Volz protocol
        if ((last_used_bitmask & (1U<<i)) == 0) {
            continue;
        }

        const SRV_Channel *channel = SRV_Channels::srv_channel(i);
        if (channel == nullptr) {
            continue;
        }

        // constrain current channel PWM within range
        const uint16_t output_pwm = constrain_int16(channel->get_output_pwm(), VOLZ_PWM_POSITION_MIN, VOLZ_PWM_POSITION_MAX);

        uint16_t cmd_transmit = 0x0800;     // Set to center position for the old protcol to not get compiler warning
        const Protocol protocol = Protocol(_protocol.get());
        switch (protocol) {
        case Protocol::DISABLED:
            break;
        case Protocol::DA26_RS485:
            {
                const float angle =  compute_angle_from_pwm(i, output_pwm, DA26_POSITION_ANGLE_MIN, DA26_POSITION_ANGLE_MAX);
                cmd_transmit = da26_compute_cmd_to_tx(angle);
                break;
            }
        case Protocol::UAVOS_VOLZ_RS485_ICD:
            {
                const float angle =  compute_angle_from_pwm(i, output_pwm, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN);
                cmd_transmit = icd_rs485_compute_cmd_to_tx(angle);
                break;
            }
        case Protocol::VOLZ_EXTENDED_POSITION:
            // This old method is suspect for many reasons..... bad scale factor, wrong linearization, doesn't form the transmitted command according to the spec.
            cmd_transmit = extended_position_compute_cmd_to_tx(output_pwm);
            break;
        }

        // prepare Volz protocol data.
        uint8_t data[VOLZ_DATA_FRAME_SIZE];

        data[0] = _reg_set_position;
        data[1] = i + 1;                // send actuator id as 1 based index so ch1 will have id 1, ch2 will have id 2 ....
        data[2] = HIGHBYTE(cmd_transmit);
        data[3] = LOWBYTE(cmd_transmit);

        // add CRC result to the message
        const uint16_t crc = crc_volz(data);
        data[4] = HIGHBYTE(crc);
        data[5] = LOWBYTE(crc);

        port->write(data, VOLZ_DATA_FRAME_SIZE);

        delay_time_us += VOLZ_DATA_FRAME_SIZE * us_per_byte + us_gap;
    }

    // Limit the maximum update rate according to the user's set parameter
    // Constrain the maximum update rate to be 400 Hz (2,500 us) & minimum update rate to 50 Hz (20,000 us)
    // const uint32_t maximum_rate_us = constrain_int32(float(1.0 /_update_rate * 1000000.0), 2500, 20000);
    const uint32_t maximum_rate_us = constrain_int32(float(1.0 /_update_rate * 1000000), 2500, 20000);
    if (delay_time_us < maximum_rate_us) {
        delay_time_us = maximum_rate_us;
    }
}

// calculate output servo angle given the input PWM
float AP_Volz_Protocol::compute_angle_from_pwm(uint8_t channel, uint16_t output_pwm, int16_t protocol_angle_min, int16_t protocol_angle_max)
{
    // Check the set min and max servo angle limits are within the protocol bounds
    const uint16_t angle_min = constrain_int16(_servo_angle_min[channel], protocol_angle_min, protocol_angle_max);
    const uint16_t angle_max = constrain_int16(_servo_angle_max[channel], protocol_angle_min, protocol_angle_max);

    // This linearization to convert from PWM to a servo angle assumes end points given by the variables and mid-pont at 0 degrees deflection
    // scale the PWM value to Volz value
    const float angle_to_pwm_scale = float(angle_max - angle_min) / float(VOLZ_PWM_POSITION_MAX - VOLZ_PWM_POSITION_MIN);
    const float angle_to_pwm_intercept = - angle_to_pwm_scale * (VOLZ_PWM_POSITION_MAX + VOLZ_PWM_POSITION_MIN) * 0.5;
    float angle = angle_to_pwm_scale * output_pwm + angle_to_pwm_intercept;

    // Constrain the angle to the servo min. and max. as defined by the protocol
    angle = constrain_float(angle, protocol_angle_min, protocol_angle_max);

    return angle;
}

// calculate CRC for volz serial protocol and send the data.
uint16_t AP_Volz_Protocol::crc_volz(uint8_t data[VOLZ_DATA_FRAME_SIZE])
{
    uint16_t crc = 0xFFFF;

    // calculate Volz CRC value according to protocol definition
    for(uint8_t i=0; i<4; i++) {
        // take input data into message that will be transmitted.
        crc = ((data[i] << 8) ^ crc);

        for(uint8_t j=0; j<8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

void AP_Volz_Protocol::update_volz_bitmask(uint32_t new_bitmask)
{
    last_used_bitmask = new_bitmask;

    uint8_t count = 0;
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (new_bitmask & (1U<<i)) {
            count++;
        }
    }
}

// Compute the originally coded protocol EXTENDED_POSITION
uint16_t AP_Volz_Protocol::extended_position_compute_cmd_to_tx(uint16_t pwm)
{
    pwm = constrain_int16(pwm, VOLZ_PWM_POSITION_MIN, VOLZ_PWM_POSITION_MAX);

    // scale the PWM value to Volz value
    const float pwm_to_hex_intercept = VOLZ_EXTENDED_POSITION_MIN - VOLZ_SCALE_VALUE * VOLZ_PWM_POSITION_MIN;
 
    return pwm * VOLZ_SCALE_VALUE + pwm_to_hex_intercept;
}


// Compute DA26 Protocol Transmitted Value Bytes
// see "DA26 RS485 Communication Protocol Specification", Section 3.4 Position Data pg 26/26
uint16_t AP_Volz_Protocol::da26_compute_cmd_to_tx(float angle)
{
    // Constrain the angle to the servo min. and max. as defined by the specification
    angle = constrain_float(angle, DA26_POSITION_ANGLE_MIN, DA26_POSITION_ANGLE_MAX);

    // Convert the desired angle to the Commanded Hexadecimal
    const float scale = DA26_POSTION_SCALE;
    const uint16_t cmd = static_cast<uint16_t>(angle * scale) + DA26_POSITION_CENTER;

    // Get the TX HIGH_BYTE: ARG1   [0 | 0 | 0 | Bit 11 | Bit 10 | Bit 9 | Bit 8 | Bit 7]
    const uint8_t high_byte = static_cast<uint8_t>(cmd >> 7);

    // Get the TX LOW_BYTE: ARG2    [0 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0]
    const uint8_t low_byte = static_cast<uint8_t>(cmd & 0x7F);

    return UINT16_VALUE(high_byte, low_byte);
}

// Compute UAVOS & VOLZ ICD Protocol Transmitted Value Bytes
// see "SD-01/02 RS485 ICD specification", Section 3.1 Position Data pg 2
// see "Volz ICD DA 26/30 DUPLEX", Section 6.1.1 pg 26/30
uint16_t AP_Volz_Protocol::icd_rs485_compute_cmd_to_tx(float angle)
{
    // Constrain the angle to the servo min. and max. as defined by the specification
    angle = constrain_float(angle, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MAX);

    // Convert the desired angle to the Commanded Hexadecimal
    uint16_t cmd_tx;
    if (is_negative(angle)) {
        // Compute the Twos Complement for transmission, note here we can guarantee to stay within uint16_t bounds
        cmd_tx = static_cast<uint16_t>(int16_t(4095) + int16_t(angle * UAVOS_VOLZ_RS485_ICD_POSTION_SCALE) + 1);
    } else {
        cmd_tx = static_cast<uint16_t>(angle * UAVOS_VOLZ_RS485_ICD_POSTION_SCALE);
    }

    return cmd_tx;
}

// NOTE this wont work for multi-protocol support with UAVOS and DA26 on the bus
// FIX ME 
// POssible solution use a a per channel bit mask???
void AP_Volz_Protocol::update_protocol_registers(uint8_t protocol_in)
{
    Protocol protocol = Protocol(protocol_in);
    switch (protocol) {
    case Protocol::DISABLED:
        break;
    case Protocol::DA26_RS485:
        _reg_set_position = DA26_NEW_POSITION_CMD;
        _reg_read_position = DA26_REPORT_ACTUAL_POSITION_CMD;
        _reg_read_servo_id = DA26_REPORT_ACTUATOR_ID_CMD;

        break;
    case Protocol::UAVOS_VOLZ_RS485_ICD:
        _reg_set_position = UAVOS_VOLZ_RS485_ICD_SET_POINT_CMD;
        _reg_read_position = UAVOS_VOLZ_RS485_ICD_REPORT_ACTUAL_POSITION_CMD;
        _reg_read_servo_id = UAVOS_VOLZ_RS485_ICD_READ_SERVO_ID_CMD;

        break;
    case Protocol::VOLZ_EXTENDED_POSITION:
        _reg_set_position = VOLZ_SET_EXTENDED_POSITION_CMD;
        _reg_read_position = VOLZ_REGISTER_UNKNOWN;
        _reg_read_servo_id = VOLZ_REGISTER_UNKNOWN;

        break;
    }
}
#endif //NUM_SERVO_CHANNELS
