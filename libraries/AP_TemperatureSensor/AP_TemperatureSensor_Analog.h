/*
 * Driver for analog temperature sensor
 */

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_ANALOG_ENABLE
#define AP_TEMPERATURE_SENSOR_ANALOG_ENABLE AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_ANALOG_ENABLE
#include <AP_HAL/AP_HAL.h>
// #include <AP_HAL/Semaphores.h>
// #include <AP_HAL/Device.h>

class AP_TemperatureSensor_Analog : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;

public:
    void init(void) override;

    void update() override;

private:
    // Analog Inputs
    // a pin for reading the receiver RSSI voltage. 
    AP_HAL::AnalogSource *temperature_analog_source;

    float _get_analog_temperature();
};
#endif // AP_TEMPERATURE_SENSOR_Analog_ENABLE
