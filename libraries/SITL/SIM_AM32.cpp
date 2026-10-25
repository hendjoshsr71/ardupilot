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
  Simulator for AM32 ESCs on the serial ESC telemetry wire, see SIM_AM32.h
*/

#include "SIM_config.h"

#if AP_SIM_AM32_ENABLED

#include "SIM_AM32.h"

#include <AP_Math/crc.h>
#include <string.h>

using namespace SITL;

void AM32::telem_request(uint32_t mask)
{
    telem_mask = mask;
}

void AM32::dshot_command(uint8_t _command, uint8_t chan)
{
    // only these are answered on the wire; the rest must not overwrite one
    if (_command != 6 && _command != 11) {
        return;
    }
    command_chan = chan;
    command = _command;
}

// the settings block: programmed, layout 3, bootloader 19, firmware 2.20, then
// bytes that tell the ESCs apart
void AM32::make_settings(uint8_t esc, uint8_t *block) const
{
    block[0] = 1;
    block[1] = 3;
    block[2] = 19;
    block[3] = 2;
    block[4] = 20;
    for (uint8_t i = 5; i < 192; i++) {
        block[i] = uint8_t(i * 7 + esc);
    }
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}

// the identity record, laid out as AM32's Inc/esc_identity.h
uint8_t AM32::make_identity(uint8_t esc, uint8_t *rec) const
{
    static const char name[] = "AM32_SIM_F051";
    const uint8_t n = strlen(name);
    uint8_t settings[192];
    make_settings(esc, settings);
    const uint16_t settings_crc = crc16_ccitt(settings, sizeof(settings), 0xFFFF);

    rec[0] = 0xA3;
    rec[1] = 0x31;
    rec[2] = 1;
    rec[3] = 26 + n;
    rec[4] = 2;             // firmware 2.20
    rec[5] = 20;
    rec[6] = 0;             // dev
    rec[7] = 0;             // clean tree, not a CAN build
    rec[8] = settings[1];
    put32(&rec[9], FW_HASH);
    rec[13] = settings[2];  // bootloader 19
    rec[14] = 3;            // protocol
    rec[15] = 32;           // its devinfo holds a hash and a type
    put32(&rec[16], BL_HASH);
    rec[20] = 255;          // official
    rec[21] = settings_crc;
    rec[22] = settings_crc >> 8;
    rec[23] = n;
    memcpy(&rec[24], name, n);
    const uint16_t crc = crc16_ccitt(rec, 24 + n, 0xFFFF);
    rec[24 + n] = crc;
    rec[25 + n] = crc >> 8;
    return 26 + n;
}

void AM32::send(const uint8_t *buf, uint16_t len) const
{
    write_to_autopilot((const char *)buf, len);
}

void AM32::update(const class Aircraft &aircraft)
{
    if (!init_sitl_pointer()) {
        return;
    }

    const uint8_t cmd = command;
    if (cmd != 0) {
        command = 0;
        const uint8_t esc = command_chan;
        if (esc < NUM_ESCS) {
            uint8_t buf[256];
            uint8_t settings[192];
            make_settings(esc, settings);
            if (cmd == 6) {
                // the info frame, then the identity record
                memcpy(buf, settings, 48);
                buf[48] = crc8_generic(buf, 48, 0x07);
                const uint8_t n = make_identity(esc, &buf[49]);
                send(buf, 49 + n);
            } else if (cmd == 11) {
                buf[0] = 0xA3;
                buf[1] = 0x32;
                buf[2] = 1;
                buf[3] = 198;
                memcpy(&buf[4], settings, 192);
                const uint16_t crc = crc16_ccitt(buf, 196, 0xFFFF);
                buf[196] = crc;
                buf[197] = crc >> 8;
                send(buf, 198);
            }
        }
        return;
    }

    const uint32_t mask = telem_mask;
    if (mask != 0) {
        telem_mask = 0;
        for (uint8_t esc = 0; esc < NUM_ESCS; esc++) {
            if (mask & (1U << esc)) {
                // 30C, 16V, no current, nothing consumed, stopped
                uint8_t kiss[10] {30, 1600 >> 8, 1600 & 0xFF};
                kiss[9] = crc8_generic(kiss, 9, 0x07);
                send(kiss, sizeof(kiss));
                break;
            }
        }
    }
}

#endif  // AP_SIM_AM32_ENABLED
