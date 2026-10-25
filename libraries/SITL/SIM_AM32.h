/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Simulator for AM32 ESCs on the serial ESC telemetry wire

  Answers what the flight controller asks of each ESC: a KISS telemetry
  frame when its telemetry bit is set, the info frame and identity record
  for DShot command 6, and the settings record for command 11 (see
  AP_ESC_Identity.h). SITL's RCOutput hands it the telemetry requests and
  DShot commands it would otherwise drop.

./Tools/autotest/sim_vehicle.py -v ArduCopter --serial5=sim:am32 --console

param set SERIAL5_PROTOCOL 16  // ESC telemetry
param set SERVO_BLH_AUTO 1
param set SERVO_BLH_TRATE 10
param set MOT_PWM_TYPE 6       // DShot600
reboot

  Every ESC answers with the same firmware, "AM32_SIM_F051" 2.20-dev at
  1a2b3c4d with bootloader 19 at 5e6f7a8b, and settings that differ by ESC.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AM32_ENABLED

#include "SIM_SerialDevice.h"

namespace SITL {

class AM32 : public SerialDevice {
public:

    static constexpr uint32_t FW_HASH = 0x1a2b3c4d;
    static constexpr uint32_t BL_HASH = 0x5e6f7a8b;

    // update state
    void update(const class Aircraft &aircraft);

    // from SITL's RCOutput: one channel asked for KISS telemetry, or sent a
    // DShot command
    void telem_request(uint32_t mask);
    void dshot_command(uint8_t command, uint8_t chan);

private:
    static constexpr uint8_t NUM_ESCS = 8;

    void make_settings(uint8_t esc, uint8_t *block) const;
    uint8_t make_identity(uint8_t esc, uint8_t *rec) const;
    void send(const uint8_t *buf, uint16_t len) const;

    volatile uint32_t telem_mask;
    volatile uint8_t command;
    volatile uint8_t command_chan;
};

}

#endif  // AP_SIM_AM32_ENABLED
