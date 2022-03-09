#include <AP_gtest.h>

#include <AP_Volz_Protocol/AP_Volz_Protocol.h>
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// Dummy class to access protected functions through a public interface for unit tests
class dummy : public AP_Volz_Protocol
{
public:
    int16_t da26_compute_cmd_to_tx_pub(float angle);
    int16_t icd_rs485_compute_cmd_to_tx_pub(float angle);
};

int16_t dummy::da26_compute_cmd_to_tx_pub(float angle)
{
    return dummy::da26_compute_cmd_to_tx(angle);
}

int16_t dummy::icd_rs485_compute_cmd_to_tx_pub(float angle)
{
    return dummy::icd_rs485_compute_cmd_to_tx(angle);
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

// Tests the Volz_servo DA26/DA30 Position Command Protocol
TEST(VOLZ_Servo_Test, Volz_Servo_da26_compute_cmd_to_tx) {
    dummy volz_servo;

    for(auto elem: conversion_table) {
        int accuracy = 2;
        EXPECT_NEAR(volz_servo.da26_compute_cmd_to_tx_pub(elem.angle), elem.tx_hex, accuracy);
    }
}

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

// Tests the Volz_servo DA26/DA30 Position Command Protocol
TEST(VOLZ_Servo_Test, Volz_Servo_icd_rs485_compute_cmd_to_tx) {
    dummy volz_servo;

    for(auto elem: icd_rs485_table) {
        int accuracy = 1;
        EXPECT_NEAR(volz_servo.icd_rs485_compute_cmd_to_tx_pub(elem.angle), elem.tx_hex, accuracy);
    }
}

AP_GTEST_MAIN()
