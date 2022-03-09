/*
 * AP_VOLZ_PROTOCOL.h
 *
 *  Created on: Oct 31, 2017
 *      Author: guy tzoler
 *
 * Baud-Rate: 115.200 bits per second
 * Number of Data bits: 8
 * Number of Stop bits: 1
 * Parity: None
 * Half/Full Duplex: Half Duplex
 *
 * Volz Command and Response are all 6 bytes
 *
 * Command
 * byte	|	Communication Type
 * 1		Command Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 * byte	|	Communication Type
 * 1		Response Code
 * 2		Actuator ID
 * 3		Argument 1
 * 4		Argument 2
 * 5		CRC High-byte
 * 6		CRC	Low-Byte
 *
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#define VOLZ_SCALE_VALUE                    float(VOLZ_EXTENDED_POSITION_MAX - VOLZ_EXTENDED_POSITION_MIN) / float(VOLZ_PWM_POSITION_MAX - VOLZ_PWM_POSITION_MIN)	// Extended Position Data Format defines 100 as 0x0F80, which results in 1920 steps for +100 deg and 1920 steps for -100 degs meaning if you take movement a scaled between -1 ... 1 and multiply by 1920 you get the travel from center
#define VOLZ_DATA_FRAME_SIZE                6

#define VOLZ_PWM_POSITION_MIN               1000
#define VOLZ_PWM_POSITION_MAX               2000

// Note the Volz DA26 and UAVOS_VOLZ_RS485_ICD serial protocols are nearly identical except for the set_position and report_position commands.
#define VOLZ_PROTOCOL_DEFAULT                0       // Default to the original VOLZ servo Serial Protocol
#define VOLZ_REGISTER_UNKNOWN                0

// Original ArduPilot VOLZ Protocol, (Protocol Version TBD)
#define VOLZ_SET_EXTENDED_POSITION_CMD      0xDC
#define VOLZ_SET_EXTENDED_POSITION_RSP      0x2C

#define VOLZ_EXTENDED_POSITION_MIN          0x0080  // Extended Position Data Format defines -100 as 0x0080 decimal 128
#define VOLZ_EXTENDED_POSITION_CENTER       0x0800  // Extended Position Data Format defines 0 as 0x0800 - decimal 2048
#define VOLZ_EXTENDED_POSITION_MAX          0x0F80  // Extended Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840

// Legacy DA26/DA30 RS485 Communication Protocol Specification V116, File: DA26-RS485-Protocol-V116.docx
#define DA26_NEW_POSITION_CMD               0xDD
#define DA26_NEW_POSITION_RSP               0x44

#define DA26_REPORT_ACTUAL_POSITION_CMD     0x92
#define DA26_REPORT_ACTUAL_POSITION_RSP     0x62

#define DA26_REPORT_ACTUATOR_ID_CMD         0xDA
#define DA26_REPORT_ACTUATOR_ID_RSP         0x6D

// Position Data Format
#define DA26_POSITION_MIN          0x0000  // Maximum clockwise rotation: -106.66d degrees | 
#define DA26_POSITION_CENTER       0x0800  // 0 degrees | CMD_HEX 0x0800, CMD_DEC 2048 | TX_HEX 
#define DA26_POSITION_MAX          0x0FFF  // Counter-clockwise rotation Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840
#define DA26_POSTION_SCALE         float(float(0xB60 - DA26_POSITION_CENTER) / (45.0 - 0.0))    // Scale: This leads to a resolution of 0.0521 degrees per digit
#define DA26_POSITION_ANGLE_MIN    -106.666  // Maximum clockwise rotation, degrees
#define DA26_POSITION_ANGLE_MAX     106.614  // Maximum counter-clockwise rotation, degrees

// UAV SD-01/02 and VOLZ RS485 ICD Specification
#define UAVOS_VOLZ_RS485_ICD_SET_POINT_CMD                  0x76
#define UAVOS_VOLZ_RS485_ICD_SET_POINT_RSP                  0x56

#define UAVOS_VOLZ_RS485_ICD_SILENT_SET_POINT_CMD           0x77    // This commands set the servo position but a corresponding response is not sent from the servo

// Section 6.1.1 pg 26
// Position Data is represented using a 12-bit 2's complement format negative rotations (clockwise). Zero being the center position.
// Each digit is equal to 0.088 degrees of servo movement: 360 deg / 4096 value = 087890625 degrees per value
// Positive set points are counter clockwise rotations
// Negative set points are clockwise rotations
#define UAVOS_VOLZ_RS485_ICD_POSITION_MIN          0x0800  // Minimum defines -180 deg as 0x0800 decimal 2048
#define UAVOS_VOLZ_RS485_ICD_POSITION_CENTER       0x0000  // Center defines     0 deg as 0x0000 decimal 0
#define UAVOS_VOLZ_RS485_ICD_POSITION_MAX          0x0800  // Maximum defines +180 deg as 0x0800 decimal 2048
#define UAVOS_VOLZ_RS485_ICD_POSTION_SCALE         11.37777778    // 2^12 digits / 360 deg, Resolution: 0.087890625 deg / digit
#define UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN    -180.0  // Maximum clockwise rotation, degrees
#define UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MAX    180.0   // Maximum counter-clockwise rotation, degrees

#define UAVOS_VOLZ_RS485_ICD_REPORT_ACTUAL_POSITION_CMD     0x69
#define UAVOS_VOLZ_RS485_ICD_REPORT_ACTUAL_POSITION_RSP     0x49

#define UAVOS_VOLZ_RS485_ICD_READ_SERVO_ID_CMD              0xDA
#define UAVOS_VOLZ_RS485_ICD_READ_SERVO_ID_RSP              0xB0

class AP_Volz_Protocol {
public:
    AP_Volz_Protocol();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Volz_Protocol);

    void update();

    // Serial Protocol Type
    enum class Protocol : uint8_t{
        DISABLED = 0,
        DA26_RS485,
        UAVOS_VOLZ_RS485_ICD,
        VOLZ_EXTENDED_POSITION,
    };

    static const struct AP_Param::GroupInfo var_info[];

protected:
    uint16_t extended_position_compute_cmd_to_tx(uint16_t pwm);     // Extended_Position Format (originally coded)
    uint16_t da26_compute_cmd_to_tx(float angle);                   // DA26/30 Legacy Protocol: Computes transmitted command from a given angle
    uint16_t icd_rs485_compute_cmd_to_tx(float angle);              // UAVOS & VOLZ ISC RS485 Protocol: Computes transmitted command from a given angle
private:
    AP_HAL::UARTDriver *port;
    
    void init(void);
    uint16_t crc_volz(uint8_t data[VOLZ_DATA_FRAME_SIZE]);
    void update_volz_bitmask(uint32_t new_bitmask);
    void update_protocol_registers(uint8_t protocol);       // Update the Volz registers according to the Protocol Parameter

    float compute_angle_from_pwm(uint8_t channel, uint16_t output_pwm, int16_t protocol_angle_min, int16_t protocol_angle_max);

    uint32_t last_volz_update_time;
    uint32_t delay_time_us;
    uint32_t us_per_byte;
    uint32_t us_gap;

    uint32_t last_used_bitmask;

    // Parameters
    AP_Int32 bitmask;           // Servo channel bitmask
    AP_Int8 _protocol;          // Protocol type: VOLZ_EXTENDED_POSITION, VOLZ DA26/30, UAVOS/VOLZ RS485 ICD
    AP_Int16 _update_rate;      // Update rate in Hz
    AP_Int16 _servo_angle_min[16];
    AP_Int16 _servo_angle_max[16];

    bool initialised;

    // Protocol Dependent Registers
    uint8_t _reg_set_position;
    uint8_t _reg_read_position;
    uint8_t _reg_read_servo_id;
};
