#include "AP_TemperatureSensor_Params.h"
#include "AP_TemperatureSensor.h"

#if AP_TEMPERATURE_SENSOR_ENABLED

#ifndef HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT 0
#endif

#ifndef HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT 0
#endif

#ifndef HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT
#define HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT 0
#endif

const AP_Param::GroupInfo AP_TemperatureSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Temperature Sensor Type
    // @Description: Enables temperature sensors
    // @Values: 0:Disabled, 1:TSYS01, 2:Analog Pin
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_TemperatureSensor_Params, _type, HAL_TEMPERATURE_SENSOR_I2C_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: DEVID
    // @DisplayName: Temperature Sensor ID
    // @Description: Temperature Sensor ID, taking into account its type, bus and instance
    // @ReadOnly: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("DEVID", 2, AP_TemperatureSensor_Params, _dev_id, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // @Param: I2C_BUS
    // @DisplayName: Temperature sensor I2C bus
    // @Description: Temperature sensor I2C bus number
    // @Range: 0 3
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_BUS", 3, AP_TemperatureSensor_Params, _i2c_bus, HAL_TEMPERATURE_SENSOR_I2C_BUS_DEFAULT),

    // @Param: I2C_ADDR
    // @DisplayName: Temperature sensor I2C address
    // @Description: Temperature sensor I2C address
    // @Range: 0 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("I2C_ADDR", 4, AP_TemperatureSensor_Params, _i2c_address, HAL_TEMPERATURE_SENSOR_I2C_ADDR_DEFAULT),

    // @Param: ANA_PIN
    // @DisplayName: Temperature sensing pin
    // @Description: Pin used to read the analog temperature voltage
    // @Values: 8:V5 Nano,11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 5, AP_TemperatureSensor_Params, _analog_pin, -1),

    // @Param: ANA_LOW
    // @DisplayName: Temperature pin low
    // @Description: Temperature pin's perceived temperature on the TEMPx_ANA_PIN when the voltage is at 0. Sometimes this can be be inverted so this value may be higher than TEMPx_ANA_PIN_HIGH.
    // @Units: C
    // @Increment: 0.01
    // @Range: -1000.0 1000.0
    // @User: Standard
    AP_GROUPINFO("ANA_LOW", 6, AP_TemperatureSensor_Params, _analog_pin_range_low, 10.0f),

    // @Param: ANA_HIGH
    // @DisplayName: Temperature pin high
    // @Description: Temperature pin's perceived temperature on the TEMPx_ANA_PIN when the voltage on the pin is 3.3V. Sometimes this can be be inverted so this value may be lower than TEMPx_ANA_PIN_LOW. For pins that have a built in voltage divider (i.e. Analog Airspeed), this value will be when 6.6V is input to the header.
    // @Units: C
    // @Increment: 0.01
    // @Range: -1000.0 1000.0
    // @User: Standard
    AP_GROUPINFO("ANA_HIGH", 7, AP_TemperatureSensor_Params, _analog_pin_range_high, 40.0f),

    AP_GROUPEND
};

AP_TemperatureSensor_Params::AP_TemperatureSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
