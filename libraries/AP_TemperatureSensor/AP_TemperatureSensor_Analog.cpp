#include "AP_TemperatureSensor_Analog.h"

#if AP_TEMPERATURE_SENSOR_ANALOG_ENABLE
// #include <utility>
// #include <stdio.h>
// #include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

void AP_TemperatureSensor_Analog::init()
{
    // a pin for reading the temperature voltage.
    temperature_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);
}

void AP_TemperatureSensor_Analog::update()
{
    _state.temperature = _get_analog_temperature();
}

float AP_TemperatureSensor_Analog::_get_analog_temperature()
{
    if (!temperature_analog_source || !temperature_analog_source->set_pin(_params._analog_pin)) {
        _state.healthy = false;
        return 0;
    }
    _state.healthy = true;

    float current_analog_voltage = temperature_analog_source->voltage_average();

    // change the max to the scaled pin max (but that doesn't seem to be available)
    // maybe just use y = mx + b instead of the interpolation?
    return linear_interpolate(_params._analog_pin_range_low, _params._analog_pin_range_high, current_analog_voltage, 0.0f, 3.3f);
}

#endif // AP_TEMPERATURE_SENSOR_Analog_ENABLE
