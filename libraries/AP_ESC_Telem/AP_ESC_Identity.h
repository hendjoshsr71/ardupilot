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
  What each AM32 ESC says about itself: its firmware's version, type and
  git hash, its bootloader's version and git hash, the layout version of
  its settings, and the 192 byte settings block itself.

  The records are AM32's (Inc/esc_identity.h there). Each starts 0xa3, a
  kind (0x31 identity, 0x32 settings), a version and its length, and ends
  with a CRC-16/CCITT-FALSE of the rest. They arrive three ways:

    serial ESC telemetry (AP_BLHeli): DShot command 6 is answered with the
      frame of the first 48 settings bytes and their CRC8 that every AM32
      sends, then the identity record; command 11 with the settings record
    extended DShot telemetry (RCOutput): streamed a byte at a time in the
      debug1 and debug2 frames while the motor is stopped, the settings for
      one pass after command 11
    DroneCAN (AP_DroneCAN): FlexDebug 101 follows each GetNodeInfo answer,
      FlexDebug 102 answers a FlexDebug 103 request

  The identity goes into every log (ESCI), into the log as text and into
  the banner. The settings block is fetched once per boot and written into
  one log (ESCE); every ESCI after it names that log (SLog) and carries the
  block's CRC (SCRC), so a log without the block still says which settings
  were in force.
 */
#pragma once

#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_AM32_IDENTITY_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>

#ifndef ESC_TELEM_MAX_ESCS
    #define ESC_TELEM_MAX_ESCS NUM_SERVO_CHANNELS
#endif

/*
  put the records back together from the EDT stream of one ESC

  Each chunk is COBS encoded behind a 0x00 delimiter:
    kind, tag, index << 4 | (count - 1), 16 data bytes, CRC8 of those 19
  For the identity the data is the identity record; for the settings it is
  the bare 192 byte block, whose CRC the identity carries. The tag is the
  low byte of that CRC, so chunks of different contents never mix.
 */
class AP_ESC_Identity_EDT {
public:
    static constexpr uint8_t CHUNK_DATA = 16;
    static constexpr uint8_t CHUNK_RAW = 3 + CHUNK_DATA + 1;
    static constexpr uint8_t IDENTITY_CHUNKS = 4;
    static constexpr uint8_t SETTINGS_CHUNKS = 12;

    // the next symbol's byte: the caller has dropped repeats
    void symbol(uint8_t byte);

    // a complete identity record, once; its length or 0
    uint8_t take_identity(uint8_t *rec, uint8_t buflen);
    // a complete settings block (192 bytes) and its CRC, once
    bool take_settings(uint8_t *block, uint16_t &crc);

private:
    void chunk(void);

    uint8_t encoded[CHUNK_RAW + 2];
    uint8_t nencoded = 0;
    bool synced = false;   // a delimiter has been seen

    struct Assembly {
        uint8_t tag;
        uint8_t count;
        uint16_t have;      // chunks held, by index
        bool complete;
    };
    Assembly identity {};
    Assembly settings {};
    uint8_t identity_data[IDENTITY_CHUNKS * CHUNK_DATA];
    uint8_t settings_data[SETTINGS_CHUNKS * CHUNK_DATA];
};

class AP_ESC_Identity {
public:
    static constexpr uint8_t RECORD_MAGIC = 0xA3;
    static constexpr uint8_t KIND_IDENTITY = 0x31;
    static constexpr uint8_t KIND_SETTINGS = 0x32;
    static constexpr uint8_t RECORD_VERSION = 1;
    static constexpr uint8_t IDENTITY_MIN_LEN = 26;
    static constexpr uint8_t SETTINGS_LEN = 192;
    static constexpr uint8_t SETTINGS_RECORD_LEN = 4 + SETTINGS_LEN + 2;
    // what every AM32 answers DShot command 6 with: 48 settings bytes and a CRC8
    static constexpr uint8_t INFO_FRAME_LEN = 49;

    // DroneCAN FlexDebug ids, from AM32's reserved range
    static constexpr uint16_t FLEXDEBUG_IDENTITY = 101;
    static constexpr uint16_t FLEXDEBUG_SETTINGS = 102;
    static constexpr uint16_t FLEXDEBUG_REQUEST = 103;
    static constexpr uint8_t REQUEST_IDENTITY = 1;
    static constexpr uint8_t REQUEST_SETTINGS = 2;

    enum class Source : uint8_t {
        NONE = 0,
        SERIAL_INFO = 1,    // the info frame only: an AM32 without identity records
        SERIAL = 2,
        EDT = 3,
        DRONECAN = 4,
    };

    struct Identity {
        Source source;
        uint8_t fw_major;
        uint8_t fw_minor;
        uint8_t fw_type;            // MAVLink FIRMWARE_VERSION_TYPE
        uint8_t fw_flags;           // 1: built from a dirty tree, 2: a CAN build
        uint32_t fw_hash;           // first 8 hex digits of the commit, 0 unknown
        uint8_t eeprom_version;     // the settings layout
        uint8_t bl_version;
        uint8_t bl_protocol;
        uint8_t bl_type;            // as the bootloader says, 1 a dev build from a dirty tree
        bool bl_known;              // bl_hash and bl_type are the bootloader's
        uint32_t bl_hash;
        bool settings_crc_known;
        uint16_t settings_crc;
        char name[30];              // FILE_NAME, the make target
    };

    // the length of a whole, valid record of this kind at the start of buf, else 0
    static uint8_t record_length(const uint8_t *buf, uint16_t len, uint8_t kind);
    static bool parse_identity(const uint8_t *rec, uint16_t len, Source source, Identity &id);
    static bool parse_info_frame(const uint8_t *frame, uint16_t len, Identity &id);
    static uint16_t crc16(const uint8_t *buf, uint16_t len, uint16_t crc=0xFFFF);
    static uint8_t crc8(const uint8_t *buf, uint16_t len);

    // a link's records for the ESC at esc_index; true if valid. Any thread
    bool handle_identity_record(uint8_t esc_index, Source source, const uint8_t *rec, uint16_t len,
                                uint8_t can_node=0, uint8_t can_driver=0);
    bool handle_info_frame(uint8_t esc_index, const uint8_t *frame, uint16_t len);
    bool handle_settings_record(uint8_t esc_index, Source source, const uint8_t *rec, uint16_t len);

    // rcout thread: the byte of an EDT debug1 (type 4) or debug2 (type 5)
    // frame. Lock free, and drops a repeat of the last before queueing
    void handle_edt_debug(uint8_t esc_index, uint8_t type, uint8_t byte);

    // what a link should ask this ESC for
    bool wants_identity(uint8_t esc_index) const;
    bool wants_settings(uint8_t esc_index) const;
    // the ESC does not answer for its settings (an AM32 without the records)
    void settings_unavailable(uint8_t esc_index);

    // the DroneCAN node and driver an ESC's records came from: false for other links
    bool can_node(uint8_t esc_index, uint8_t &node, uint8_t &driver) const;

    // 100Hz from AP_ESC_Telem::update(): EDT records, requests over EDT, logging
    void update();

    // one line per distinct identity, e.g. "ESC1-4 AM32 2.20-dev 7eaa34e4 BL19 3a2b1c0d",
    // for the log and the banner. next starts at 0; false when there are no more
    bool get_firmware_line(uint8_t &next, char *buf, uint8_t buflen) const;

private:
    bool store_identity(uint8_t esc_index, const Identity &id, uint8_t can_node, uint8_t can_driver);
    bool store_settings(uint8_t esc_index, const uint8_t *block);
    void update_edt(void);
    void request_settings_over_edt(void);
    void write_logs(void);
    void announce(void);
    bool same_firmware(const Identity &a, const Identity &b) const;
    void format_firmware(const Identity &id, uint8_t first, uint8_t last, char *buf, uint8_t buflen) const;

    struct State {
        Identity id;
        uint8_t can_node;
        uint8_t can_driver;
        bool logged;            // ESCI written in the current log
        bool announced;
        bool settings_unavailable;
        bool settings_have;     // a block came this boot, with this CRC
        uint16_t settings_crc;
        uint16_t settings_log;  // the log holding it, 0 not yet written
        uint8_t *settings;      // the block, until it is written into a log
        uint8_t edt_requests;
        uint32_t edt_request_ms;
        uint32_t learned_ms;    // when the firmware last changed, for announcing
    };
    State *state[ESC_TELEM_MAX_ESCS] {};
    mutable HAL_Semaphore sem;

    // EDT: the rcout thread queues, update() takes
    struct EdtByte {
        uint8_t esc_index;
        uint8_t byte;
    };
    ObjectBuffer<EdtByte> *edt_queue = nullptr;
    uint16_t edt_last[ESC_TELEM_MAX_ESCS] {};  // rcout thread: last (type << 8 | byte) + 1
    AP_ESC_Identity_EDT *edt[ESC_TELEM_MAX_ESCS] {};

    uint8_t log_start_count = 0;
    uint32_t last_announce_ms = 0;
};

#endif // AP_ESC_TELEM_AM32_IDENTITY_ENABLED
