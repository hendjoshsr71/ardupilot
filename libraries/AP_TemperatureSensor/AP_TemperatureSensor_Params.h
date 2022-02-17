#pragma once

#include <AP_Param/AP_Param.h>

class AP_TemperatureSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_TemperatureSensor_Params(void);

    CLASS_NO_COPY(AP_TemperatureSensor_Params);

    AP_Int8  _type;             // 0=disabled, others see frontend enum TYPE
    AP_Int8  _i2c_bus;          // I2C bus number
    AP_Int8  _i2c_address;      // I2C address
    AP_Int32 _dev_id;           // Device ID
    AP_Int8  _analog_pin;            // Analog pin temperature value found on
    AP_Float _analog_pin_range_low;  // Temperature value for lowest pin voltage
    AP_Float _analog_pin_range_high; // Temperature value for highest pin voltage
};
