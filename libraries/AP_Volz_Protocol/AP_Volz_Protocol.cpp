/*
 * AP_VOLZ_PROTOCOL.cpp
 *
 *  Created on: Oct 31, 2017
 *      Author: guy
 * 
 * January 2022: Added Volz DA26/30 RS485 Protocol & UAVOS & Volz ICD R485 Protocol
 */
#include "AP_Volz_Protocol.h"

#if AP_VOLZ_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Scheduler/AP_Scheduler.h>

// #DEBUG  only
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Volz_Protocol::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of volz servo protocol to specific channels
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("MASK", 1, AP_Volz_Protocol, bitmask, 0),

// FIXME I THINK UAVOS_VOLZ_RS485_ICD should be default moving forward as it is supported from two manufacturers 

    // @Param: TYPE
    // @DisplayName: Volz Protocol Type
    // @Description: Sets the Volz Servo Protocol Type
    // @Bitmask: 0:Disabled, 1:DA26_RS485, 2:UAVOS_VOLZ_RS485_ICD, 3:Volz_Extended_Position
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 2, AP_Volz_Protocol, _protocol, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: RATE
    // @DisplayName: Volz Maximum Update Rate (Hz)
    // @Description: Sets the maximum update rate to send new position targets to the servos (Note: this applies to all of the servos)
    // @Units: Hz
    // @Increment: 1
    // @Range: 50 400
    // @User: Standard
    AP_GROUPINFO("RATE", 3, AP_Volz_Protocol, _update_rate, 100),

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

#if NUM_SERVO_CHANNELS > 1
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
#endif

#if NUM_SERVO_CHANNELS > 2
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
#endif

#if NUM_SERVO_CHANNELS > 3
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
#endif

#if NUM_SERVO_CHANNELS > 4
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
#endif

#if NUM_SERVO_CHANNELS > 5
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
#endif

#if NUM_SERVO_CHANNELS > 6
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
#endif

#if NUM_SERVO_CHANNELS > 7
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
#endif

#if NUM_SERVO_CHANNELS > 8
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
#endif

#if NUM_SERVO_CHANNELS > 9
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
#endif

#if NUM_SERVO_CHANNELS > 10
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
#endif

#if NUM_SERVO_CHANNELS > 11
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
#endif

#if NUM_SERVO_CHANNELS > 12
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
#endif

#if NUM_SERVO_CHANNELS > 13
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
#endif

#if NUM_SERVO_CHANNELS > 14
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
#endif

#if NUM_SERVO_CHANNELS > 15
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
#endif

////////////// MOVE THESE ABOVE ONCE DONE TESTING FOR PR  ////

    // @Param: SER0
    // @DisplayName: Serial 0, Channel Mask
    // @Description: Serial 0, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER0", 43, AP_Volz_Protocol, _bitmask[0], 0),

#if SERIALMANAGER_NUM_PORTS > 1
    // @Param: SER1
    // @DisplayName: Serial 1, Channel Mask
    // @Description: Serial 1, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER1", 44, AP_Volz_Protocol, _bitmask[1], 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 2
    // @Param: SER2
    // @DisplayName: Serial 2, Channel Mask
    // @Description: Serial 2, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER2", 45, AP_Volz_Protocol, _bitmask[2], 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 3
    // @Param: SER3
    // @DisplayName: Serial 3, Channel Mask
    // @Description: Serial 3, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER3", 46, AP_Volz_Protocol, _bitmask[3], 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 4
    // @Param: SER4
    // @DisplayName: Serial 4, Channel Mask
    // @Description: Serial 4, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER4", 47, AP_Volz_Protocol, _bitmask[4], 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 5
    // @Param: SER5
    // @DisplayName: Serial 5, Channel Mask
    // @Description: Serial 5, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER5", 48, AP_Volz_Protocol, _bitmask[5], 0),
#endif

#if SERIALMANAGER_NUM_PORTS > 6
    // @Param: SER6
    // @DisplayName: Serial 6, Channel Mask
    // @Description: Serial 6, Channel Mask: Servos which should be sent across this serial port
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Standard
    AP_GROUPINFO("SER6", 49, AP_Volz_Protocol, _bitmask[6], 0),
#endif

/// FIX ME MOVE ABOVE MIN?MAX PARAMS ONCE DONE TESTING

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

    AP_SerialManager& sm = AP::serialmanager();

    for (uint8_t i = 0; i < ARRAY_SIZE(_ports); i++) {
        _ports[i] = sm.find_serial(AP_SerialManager::SerialProtocol_Volz, i);
        if (_ports[i] == nullptr) {
            continue;
        }
        _num_ports++;
    }

    // Note there is only one baudrate for volzservos 115.2K baud
    // if (_port) {
    //     const uint32_t baudrate = serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Volz, 0);
    //     us_per_byte = 10.5 * 1e6 / baudrate;  // 91.1 usec with 115.2K baud 
    //     us_gap = 4.0 * 1e6 / baudrate;        // 34.722 usec with 115.2K baud
    // }

    // Protocol Changes Require Rebooting
    update_protocol_registers(_protocol);
}

void AP_Volz_Protocol::update()
{
    if (!initialised) {
        initialised = true;
        init();
        _last_volz_update_time = AP_HAL::micros();
        return;
    }

    // FIX ME: Update the delay_time based upon the main calling loop rate eg Copter's 400Hz
    // this limits the maximum update rate based upon: _update_rate, # of channels, safety factor, & average transmission time
    const uint32_t now = AP_HAL::micros();
    if (_last_volz_update_time != 0  && now - _last_volz_update_time < (0.5 * _delay_time_us)) {
        return;
    }
    _last_volz_update_time = now;
    _delay_time_us = 0;

    // Update protocol Registers if needed
    if (_protocol.get() != _last_protocol) {
        update_protocol_registers(_protocol);
    }

    // Loop over all of the Volz enabled serial ports
    for (uint8_t port_id = 0; port_id < _num_ports; port_id++) {
        if (_ports[port_id] == nullptr) {
            continue;
        }

        // Should just do this on initialization and store it
        const int8_t ser_n_num = AP::serialmanager().find_portnum(AP_SerialManager::SerialProtocol_Volz, port_id);
        if (ser_n_num == -1) {
            continue;
        }

        if (_ports[port_id]->txspace() < VOLZ_DATA_FRAME_SIZE) {
            // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "VOLZ Port %u: out of space \n", port_id);
            continue;
        }

        // Loop over all servo channels
        for (uint8_t ch_id = 0; ch_id < NUM_SERVO_CHANNELS; ch_id++) {

            // check if current channel is needed for Volz protocol
            if ((_bitmask[ser_n_num].get() & (1U<<ch_id)) == 0) {
                continue;
            }

            uint16_t cmd_transmit = 0;
            if (!compute_position_command_tx(ch_id, cmd_transmit)) {
                continue;
            }

            // prepare Volz protocol data.
            uint8_t data[VOLZ_DATA_FRAME_SIZE];

            data[0] = _reg_set_position;
            data[1] = ch_id + 1;                // send actuator id as 1 based index so CH1 will have ID 1, CH2 will have ID 2 ....
            data[2] = HIGHBYTE(cmd_transmit);
            data[3] = LOWBYTE(cmd_transmit);

            // add CRC result to the message
            const uint16_t crc = crc_volz(data);
            data[4] = HIGHBYTE(crc);
            data[5] = LOWBYTE(crc);

            _ports[port_id]->write(data, VOLZ_DATA_FRAME_SIZE);

            _delay_time_us += VOLZ_DATA_FRAME_SIZE * _us_per_byte + _us_gap;
        }
    }

    // Limit the maximum update rate according to the user's set parameter
    // Constrain the maximum update rate to be 400 Hz (2,500 us) & minimum update rate to 50 Hz (20,000 us)
    
#ifndef HAL_BUILD_AP_PERIPH
    const uint32_t loop_period_us = AP::scheduler().get_loop_period_us();
#else
    const uint32_t loop_period_us = 2500;   //// FIX ME HOW TO GET LOOP period here
#endif

    const int16_t update_rate_us = MAX(float(1.0 /_update_rate * 1000000.0), loop_period_us);
    const uint32_t maximum_rate_us = constrain_int32(update_rate_us, 2500, 20000);

    _delay_time_us = (AP_HAL::micros() - _last_volz_update_time);     // Compute the total time to complete updates across all serial and servo channels
    if(_delay_time_us < maximum_rate_us) {
        _delay_time_us = maximum_rate_us - _delay_time_us - 0.25 * loop_period_us;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"VOLZ: Not enough time between loops for update rate: %i", _update_rate.get());
        _delay_time_us = 500;
    }
}

bool AP_Volz_Protocol::compute_position_command_tx(uint8_t ch_id, uint16_t &command_tx)
{
    const SRV_Channel *channel = SRV_Channels::srv_channel(ch_id);
    if (channel == nullptr) {
        return false;
    }

    // constrain current channel PWM within range
    const uint16_t output_pwm = constrain_int16(channel->get_output_pwm(), VOLZ_PWM_POSITION_MIN, VOLZ_PWM_POSITION_MAX);

    const Protocol protocol = Protocol(_protocol.get());
    switch (protocol) {
    case Protocol::DISABLED:
        return false;
    case Protocol::DA26_RS485:
        {
            const float angle =  compute_angle_from_pwm(ch_id, output_pwm, DA26_POSITION_ANGLE_MIN, DA26_POSITION_ANGLE_MAX);
            command_tx = da26_compute_cmd_to_tx(angle);
            break;
        }
    case Protocol::UAVOS_VOLZ_RS485_ICD:
        {
            const float angle =  compute_angle_from_pwm(ch_id, output_pwm, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MIN, UAVOS_VOLZ_RS485_ICD_POSITION_ANGLE_MAX);
            command_tx = icd_rs485_compute_cmd_to_tx(angle);
            break;
        }
    case Protocol::VOLZ_EXTENDED_POSITION:
        command_tx = extended_position_compute_cmd_to_tx(output_pwm);
        break;
    }

    return true;
}

// calculate output servo angle given the input PWM
float AP_Volz_Protocol::compute_angle_from_pwm(uint8_t channel, uint16_t pwm, float protocol_angle_min, float protocol_angle_max)
{
    // Check the set min and max servo angle limits are within the protocol bounds
    const float angle_min = constrain_float(_servo_angle_min[channel], protocol_angle_min, protocol_angle_max);
    const float angle_max = constrain_float(_servo_angle_max[channel], protocol_angle_min, protocol_angle_max);

    // This linearization to convert from PWM to a servo angle assumes end points given by the variables
    // and a mid-pont at 0 degrees deflection to scale the PWM value to an output angle
    const float angle_to_pwm_scale = (angle_max - angle_min) / (VOLZ_PWM_POSITION_MAX - VOLZ_PWM_POSITION_MIN);
    const float angle_to_pwm_intercept = - angle_to_pwm_scale * (VOLZ_PWM_POSITION_MAX + VOLZ_PWM_POSITION_MIN) * 0.5;
    float angle = angle_to_pwm_scale * pwm + angle_to_pwm_intercept;

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
    const uint16_t cmd = static_cast<uint16_t>((angle * DA26_POSTION_SCALE) + DA26_POSITION_CENTER);

    // Get the TX HIGH_BYTE: ARG1 [0 | 0 | 0 | Bit 11 | Bit 10 | Bit 9 | Bit 8 | Bit 7]
    const uint8_t high_byte = static_cast<uint8_t>(cmd >> 7);

    // Get the TX LOW_BYTE:  ARG2 [0 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0]
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

// NOTE: This wont work for multi-protocol support with UAVOS_VOLZ_RS485_ICD and DA26 on the same bus
// Possible solution use a per channel bit mask for each protocol
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

#endif  // AP_VOLZ_ENABLED
