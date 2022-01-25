/*
 * Build Command (Assuming starting directory is ardupilot)
 *    gcc -o ap_volz_code_test libraries/AP_Volz_Protocol/test_volz_int_main.cpp
 *    gcc -g -o ap_volz_code_test libraries/AP_Volz_Protocol/test_volz_int_main.cpp   // With Debugging symbols
 *
 * How to Use?
 *    ./eedump_ap_param eeprom.bin
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define UINT16_VALUE(hbyte, lbyte) (static_cast<uint16_t>(((hbyte)<<8)|(lbyte)))
#define FLT_EPSILON	__FLT_EPSILON__

/* 
 * @brief: Check whether a float is less than zero
 */
template <typename T>
inline bool is_negative(const T fVal1) {
    // static_assert(std::is_floating_point<T>::value || std::is_base_of<T,AP_Float>::value,
    //               "Template parameter not of type float");
    return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}

// #include "AP_Volz_Protocol.h"

// Position Data Format
#define DA26_POSITION_MIN          0x0000  // Maximum clockwise rotation: -106.66d degrees | 
#define DA26_POSITION_CENTER       0x0800  // 0 degrees | CMD_HEX 0x0800, CMD_DEC 2048 | TX_HEX 
#define DA26_POSITION_MAX          0x0FFF  // Counter-clockwise rotation Position Data Format defines +100 as 0x0F80 decimal 3968 -> full range decimal 3840
#define DA26_POSTION_SCALE         float(float(0xB60 - DA26_POSITION_CENTER) / (45.0 - 0.0))    // Scale: This leads to a resolution of 0.0521 degrees per bit
#define DA26_POSITION_ANGLE_MIN    -106.666  // Maximum clockwise rotation, degress
#define DA26_POSITION_ANGLE_MAX     106.614  // Maximum counter-clockwise rotation, degrees


// Section 6.1.1 pg 26
// Position Data is represented using a 12-bit 2's complement format. Zero being the center position.
// Each digit is equal to 0.088 degrees of servo movement: 360 deg / 4096 value = 0.088 degrees per value
// Positive set points are counter clockwise rotations
// Negative set points are clockwise rotations
#define UAVOS_VOLZ_RS485_ICD_POSITION_MIN          0x0000  // Minimum defines -180 deg as 0x0080 decimal 128
#define UAVOS_VOLZ_RS485_ICD_POSITION_CENTER       0x00    // Center defines     0 deg as   0x00 decimal 0
#define UAVOS_VOLZ_RS485_ICD_POSITION_MAX          0x0800  // Maximum defines +180 deg as 0x0800 decimal 2048
#define UAVOS_VOLZ_RS485_ICD_POSTION_SCALE         11.37777778    // Resolution: degrees per digit = 360 deg / 2^12 digits = 0.087890625 deg / digit
#define UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN    -180.0  // Maximum clockwise rotation, degrees
#define UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MAX    180.0   // Maximum counter-clockwise rotation, degrees

// Compute DA26 Protocol Transmitted Value Bytes
// see "DA26 RS485 Communication Protocol Specification", Section 3.4 Position Data pg 26/26
static uint16_t da26_compute_cmd_to_tx(float angle)
{
    // Constrain the angle to the servo min. and max. as defined by the specification
    // angle = constrain_float(angle, DA26_POSITION_ANGLE_MIN, DA26_POSITION_ANGLE_MAX);

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
static uint16_t icd_rs485_compute_cmd_to_tx(float angle)
{
    // Constrain the angle to the servo min. and max. as defined by the specification
    // angle = constrain_float(angle, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MAX);

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

const struct DA26_LEGACY_VALUES {
        float angle;            // angle (degrees)
        uint32_t cmd_hex;       // CMD hex value
        uint32_t cmd_dec;       // CMD decimal value
        uint32_t tx_hex;        // TX (transmitted) value in hex
        uint32_t tx_dec;        // TX (transmitted) value in decimal
    } conversion_table [] = {
        {106.614, 0xFFF, 4095, 0x1F7F, 8063},     // Defined as maximum counter-clockwise rotation
        { 55.0,  0xC20, 3104, 0x1820, 6176},
        { 45.0,  0xB60, 2912, 0x1660, 5728},
        {  0.0,  0x800, 2048, 0x1000, 4096},      // Defined as zero rotation
        {-22.0,  0x65A, 1626, 0x0C5A, 3162},
        {-17.0,  0x6BA, 1722, 0x0D3A, 3162},
        {-45.0,  0x4A1, 1185, 0x0921, 2337},
        {-106.666,  0x000,   0, 0x0000,    0},    // Defined as maximum clockwise rotation
    };

const struct ICD_RS485_LEGACY_VALUES {
        float angle;            // angle (degrees)
        uint32_t tx_hex;        // TX (transmitted) value in hex
        uint32_t tx_dec;        // TX (transmitted) value in decimal
    } icd_rs485_table [] = {
        {180.0, 0x800, 2048},     // Defined as maximum counter-clockwise rotation
        {170.0, 0x78E, 1934},
        { 45.0, 0x200,  512},
        {  0.0, 0x000,    0},      // Defined as zero rotation
        {-45.0, 0xE00, 3584},      // Note clock-wise rotations are the twos complement from 0
        {-170.0, 0x872, 2162},
        {-180.0, 0x800, 2048},    // Defined as maximum clockwise rotation
    };

int main(int argc, char *argv[])
{

    for(auto elem: conversion_table) {
        uint16_t output = da26_compute_cmd_to_tx(elem.angle);

        if (output != elem.tx_hex) {
            printf("Wrong value, angle =%f, output = %i, expected = %i \n", elem.angle, output, elem.tx_hex);
        }
    }

    printf("TEST icd_rs485_compute_cmd_to_tx \n");
    for(auto elem: icd_rs485_table) {
        uint16_t output = icd_rs485_compute_cmd_to_tx(elem.angle);
        if (output != elem.tx_hex) {
            printf("Wrong value, angle =%f, output = %i, expected = %i \n", elem.angle, output, elem.tx_hex);
        }
    }

return 0;
}