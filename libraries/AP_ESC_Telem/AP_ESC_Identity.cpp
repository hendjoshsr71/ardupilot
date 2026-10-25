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

#include "AP_ESC_Identity.h"

#if AP_ESC_TELEM_AM32_IDENTITY_ENABLED

#include <AP_Math/crc.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// asking an EDT ESC for its settings again: a settings pass takes up to 14s at 1kHz
#define EDT_SETTINGS_RETRY_MS 20000U
#define EDT_SETTINGS_MAX_REQUESTS 5
// wait this long after an identity changes before saying it, so ESCs learned
// together are said together
#define ANNOUNCE_SETTLE_MS 2000U

/*
  AP_ESC_Identity_EDT
 */
void AP_ESC_Identity_EDT::symbol(uint8_t byte)
{
    if (byte == 0) {
        if (synced && nencoded > 0) {
            chunk();
        }
        synced = true;
        nencoded = 0;
        return;
    }
    if (!synced) {
        return;
    }
    if (nencoded >= sizeof(encoded)) {
        // a delimiter was lost: wait for the next one
        synced = false;
        return;
    }
    encoded[nencoded++] = byte;
}

void AP_ESC_Identity_EDT::chunk(void)
{
    // COBS decode, which must give exactly one chunk
    uint8_t raw[CHUNK_RAW];
    uint8_t n = 0;
    uint8_t i = 0;
    while (i < nencoded) {
        const uint8_t code = encoded[i++];
        if (uint16_t(i) + code - 1 > nencoded) {
            return;
        }
        for (uint8_t j = 1; j < code; j++) {
            if (n >= CHUNK_RAW) {
                return;
            }
            raw[n++] = encoded[i++];
        }
        if (code < 0xFF && i < nencoded) {
            if (n >= CHUNK_RAW) {
                return;
            }
            raw[n++] = 0;
        }
    }
    if (n != CHUNK_RAW || AP_ESC_Identity::crc8(raw, CHUNK_RAW - 1) != raw[CHUNK_RAW - 1]) {
        return;
    }

    const uint8_t kind = raw[0];
    const uint8_t tag = raw[1];
    const uint8_t index = raw[2] >> 4;
    const uint8_t count = (raw[2] & 0x0F) + 1;
    Assembly *a;
    uint8_t *data;
    uint8_t max_chunks;
    if (kind == AP_ESC_Identity::KIND_IDENTITY) {
        a = &identity;
        data = identity_data;
        max_chunks = IDENTITY_CHUNKS;
    } else if (kind == AP_ESC_Identity::KIND_SETTINGS) {
        a = &settings;
        data = settings_data;
        max_chunks = SETTINGS_CHUNKS;
    } else {
        return;
    }
    if (count > max_chunks || index >= count) {
        return;
    }
    if (a->tag != tag || a->count != count) {
        // different contents: start again
        a->tag = tag;
        a->count = count;
        a->have = 0;
        a->complete = false;
    }
    memcpy(&data[index * CHUNK_DATA], &raw[3], CHUNK_DATA);
    a->have |= 1U << index;
    if (a->have == (1U << count) - 1) {
        a->complete = true;
    }
}

uint8_t AP_ESC_Identity_EDT::take_identity(uint8_t *rec, uint8_t buflen)
{
    if (!identity.complete) {
        return 0;
    }
    identity.complete = false;
    identity.have = 0;
    const uint8_t len = MIN(buflen, identity.count * CHUNK_DATA);
    memcpy(rec, identity_data, len);
    return len;
}

bool AP_ESC_Identity_EDT::take_settings(uint8_t *block, uint16_t &crc)
{
    if (!settings.complete) {
        return false;
    }
    settings.complete = false;
    settings.have = 0;
    if (settings.count != SETTINGS_CHUNKS) {
        return false;
    }
    memcpy(block, settings_data, AP_ESC_Identity::SETTINGS_LEN);
    crc = AP_ESC_Identity::crc16(block, AP_ESC_Identity::SETTINGS_LEN);
    return (crc & 0xFF) == settings.tag;
}

/*
  AP_ESC_Identity: the records
 */
uint16_t AP_ESC_Identity::crc16(const uint8_t *buf, uint16_t len, uint16_t crc)
{
    return crc16_ccitt(buf, len, crc);
}

uint8_t AP_ESC_Identity::crc8(const uint8_t *buf, uint16_t len)
{
    return crc8_generic(buf, len, 0x07);
}

uint8_t AP_ESC_Identity::record_length(const uint8_t *buf, uint16_t len, uint8_t kind)
{
    if (len < 6 || buf[0] != RECORD_MAGIC || buf[1] != kind || buf[2] != RECORD_VERSION) {
        return 0;
    }
    const uint8_t n = buf[3];
    if (n < 6 || n > len) {
        return 0;
    }
    if (kind == KIND_IDENTITY && (n < IDENTITY_MIN_LEN || n != IDENTITY_MIN_LEN + buf[23])) {
        return 0;
    }
    if (kind == KIND_SETTINGS && n != SETTINGS_RECORD_LEN) {
        return 0;
    }
    const uint16_t crc = buf[n - 2] | (buf[n - 1] << 8);
    return crc16(buf, n - 2) == crc ? n : 0;
}

static uint32_t le32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (uint32_t(p[3]) << 24);
}

bool AP_ESC_Identity::parse_identity(const uint8_t *rec, uint16_t len, Source source, Identity &id)
{
    const uint8_t n = record_length(rec, len, KIND_IDENTITY);
    if (n == 0) {
        return false;
    }
    memset(&id, 0, sizeof(id));
    id.source = source;
    id.fw_major = rec[4];
    id.fw_minor = rec[5];
    id.fw_type = rec[6];
    id.fw_flags = rec[7];
    id.eeprom_version = rec[8];
    id.fw_hash = le32(&rec[9]);
    id.bl_version = rec[13];
    id.bl_protocol = rec[14];
    id.bl_known = rec[15] >= 32;
    if (id.bl_known) {
        id.bl_hash = le32(&rec[16]);
        id.bl_type = rec[20];
    }
    id.settings_crc_known = true;
    id.settings_crc = rec[21] | (rec[22] << 8);
    const uint8_t name_len = MIN(rec[23], sizeof(id.name) - 1);
    memcpy(id.name, &rec[24], name_len);
    return true;
}

bool AP_ESC_Identity::parse_info_frame(const uint8_t *frame, uint16_t len, Identity &id)
{
    // byte 0 is 1 in a programmed settings block
    if (len < INFO_FRAME_LEN || frame[0] != 1 || crc8(frame, INFO_FRAME_LEN - 1) != frame[INFO_FRAME_LEN - 1]) {
        return false;
    }
    memset(&id, 0, sizeof(id));
    id.source = Source::SERIAL_INFO;
    id.eeprom_version = frame[1];
    id.bl_version = frame[2];
    id.fw_major = frame[3];
    id.fw_minor = frame[4];
    return true;
}

bool AP_ESC_Identity::handle_identity_record(uint8_t esc_index, Source source, const uint8_t *rec, uint16_t len,
                                             uint8_t node, uint8_t driver)
{
    Identity id;
    if (esc_index >= ESC_TELEM_MAX_ESCS || !parse_identity(rec, len, source, id)) {
        return false;
    }
    return store_identity(esc_index, id, node, driver);
}

bool AP_ESC_Identity::handle_info_frame(uint8_t esc_index, const uint8_t *frame, uint16_t len)
{
    Identity id;
    if (esc_index >= ESC_TELEM_MAX_ESCS || !parse_info_frame(frame, len, id)) {
        return false;
    }
    {
        WITH_SEMAPHORE(sem);
        if (state[esc_index] != nullptr && state[esc_index]->id.source > Source::SERIAL_INFO) {
            // never replace a whole identity with the little the info frame has
            return true;
        }
    }
    return store_identity(esc_index, id, 0, 0);
}

bool AP_ESC_Identity::handle_settings_record(uint8_t esc_index, Source source, const uint8_t *rec, uint16_t len)
{
    if (esc_index >= ESC_TELEM_MAX_ESCS || record_length(rec, len, KIND_SETTINGS) == 0) {
        return false;
    }
    return store_settings(esc_index, &rec[4]);
}

bool AP_ESC_Identity::same_firmware(const Identity &a, const Identity &b) const
{
    return a.fw_major == b.fw_major && a.fw_minor == b.fw_minor &&
        a.fw_type == b.fw_type && a.fw_flags == b.fw_flags && a.fw_hash == b.fw_hash &&
        a.eeprom_version == b.eeprom_version &&
        a.bl_version == b.bl_version && a.bl_protocol == b.bl_protocol &&
        a.bl_known == b.bl_known && a.bl_hash == b.bl_hash && a.bl_type == b.bl_type &&
        (a.source == Source::SERIAL_INFO) == (b.source == Source::SERIAL_INFO) &&
        strncmp(a.name, b.name, sizeof(a.name)) == 0;
}

bool AP_ESC_Identity::store_identity(uint8_t esc_index, const Identity &id, uint8_t node, uint8_t driver)
{
    WITH_SEMAPHORE(sem);
    State *s = state[esc_index];
    if (s == nullptr) {
        s = NEW_NOTHROW State();
        if (s == nullptr) {
            return false;
        }
        state[esc_index] = s;
    }
    s->can_node = node;
    s->can_driver = driver;
    if (s->id.source != Source::NONE && same_firmware(s->id, id) &&
        s->id.settings_crc_known == id.settings_crc_known && s->id.settings_crc == id.settings_crc) {
        // nothing new, whichever link it came over
        return true;
    }
    if (s->id.source == Source::NONE || !same_firmware(s->id, id)) {
        s->announced = false;
        s->learned_ms = AP_HAL::millis();
    }
    s->id = id;
    s->logged = false;
    if (s->settings_have && id.settings_crc_known && id.settings_crc != s->settings_crc) {
        // the settings changed (a configurator, a DShot command): fetch them again
        s->settings_have = false;
        s->settings_unavailable = false;
        s->edt_requests = 0;
    }
    return true;
}

bool AP_ESC_Identity::store_settings(uint8_t esc_index, const uint8_t *block)
{
    const uint16_t crc = crc16(block, SETTINGS_LEN);
    WITH_SEMAPHORE(sem);
    State *s = state[esc_index];
    if (s == nullptr) {
        // settings belong to an ESC we know: ask for its identity first
        return false;
    }
    if (s->settings_have && s->settings_crc == crc) {
        return true;
    }
    if (s->settings == nullptr) {
        s->settings = NEW_NOTHROW uint8_t[SETTINGS_LEN];
        if (s->settings == nullptr) {
            return false;
        }
    }
    memcpy(s->settings, block, SETTINGS_LEN);
    s->settings_have = true;
    s->settings_crc = crc;
    s->settings_log = 0;
    return true;
}

bool AP_ESC_Identity::wants_identity(uint8_t esc_index) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    const State *s = state[esc_index];
    return s == nullptr || s->id.source == Source::SERIAL_INFO;
}

bool AP_ESC_Identity::wants_settings(uint8_t esc_index) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    const State *s = state[esc_index];
    if (s == nullptr || s->settings_unavailable) {
        return false;
    }
    if (!s->settings_have) {
        return true;
    }
    return s->id.settings_crc_known && s->id.settings_crc != s->settings_crc;
}

void AP_ESC_Identity::settings_unavailable(uint8_t esc_index)
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return;
    }
    WITH_SEMAPHORE(sem);
    if (state[esc_index] != nullptr) {
        state[esc_index]->settings_unavailable = true;
    }
}

bool AP_ESC_Identity::can_node(uint8_t esc_index, uint8_t &node, uint8_t &driver) const
{
    if (esc_index >= ESC_TELEM_MAX_ESCS) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    const State *s = state[esc_index];
    if (s == nullptr || s->id.source != Source::DRONECAN || s->can_node == 0) {
        return false;
    }
    node = s->can_node;
    driver = s->can_driver;
    return true;
}

/*
  EDT
 */
void AP_ESC_Identity::handle_edt_debug(uint8_t esc_index, uint8_t type, uint8_t byte)
{
    ObjectBuffer<EdtByte> *q = edt_queue;
    if (esc_index >= ESC_TELEM_MAX_ESCS || q == nullptr) {
        return;
    }
    // the same reply is decoded every output cycle until the next one comes,
    // and the ESC holds each symbol for several: only a change is a new one
    const uint16_t key = ((type << 8) | byte) + 1;
    if (edt_last[esc_index] == key) {
        return;
    }
    edt_last[esc_index] = key;
    const EdtByte b { esc_index, byte };
    q->push(b);
}

void AP_ESC_Identity::update_edt(void)
{
    if (edt_queue == nullptr) {
        edt_queue = NEW_NOTHROW ObjectBuffer<EdtByte>(256);
        return;
    }
    EdtByte b;
    while (edt_queue->pop(b)) {
        AP_ESC_Identity_EDT *r = edt[b.esc_index];
        if (r == nullptr) {
            r = edt[b.esc_index] = NEW_NOTHROW AP_ESC_Identity_EDT();
            if (r == nullptr) {
                continue;
            }
        }
        r->symbol(b.byte);

        uint8_t rec[AP_ESC_Identity_EDT::IDENTITY_CHUNKS * AP_ESC_Identity_EDT::CHUNK_DATA];
        const uint8_t len = r->take_identity(rec, sizeof(rec));
        if (len > 0) {
            handle_identity_record(b.esc_index, Source::EDT, rec, len);
        }
        uint8_t block[SETTINGS_LEN];
        uint16_t crc;
        if (r->take_settings(block, crc)) {
            bool matches;
            {
                WITH_SEMAPHORE(sem);
                const State *s = state[b.esc_index];
                // a chunk's tag is only 8 bits: the identity's CRC checks the whole
                matches = s != nullptr && s->id.settings_crc_known && s->id.settings_crc == crc;
            }
            if (matches) {
                store_settings(b.esc_index, block);
            }
        }
    }
}

void AP_ESC_Identity::request_settings_over_edt(void)
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        bool ask = false;
        {
            WITH_SEMAPHORE(sem);
            State *s = state[i];
            if (s != nullptr && s->id.source == Source::EDT && s->edt_requests < EDT_SETTINGS_MAX_REQUESTS &&
                (s->edt_requests == 0 || now - s->edt_request_ms >= EDT_SETTINGS_RETRY_MS) &&
                (!s->settings_have || s->id.settings_crc != s->settings_crc)) {
                s->edt_requests++;
                s->edt_request_ms = now;
                ask = true;
            }
        }
        if (ask) {
            hal.rcout->send_dshot_command(AP_HAL::RCOutput::DSHOT_SETTINGS_REQUEST, i, 0, 10, false);
        }
    }
}

/*
  logging and announcing. Hex is lower case, whatever %x gives
 */
static const char hex_digits[] = "0123456789abcdef";

void AP_ESC_Identity::write_logs(void)
{
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr || !logger->logging_started()) {
        return;
    }
    WITH_SEMAPHORE(sem);
    if (log_start_count != logger->get_log_start_count()) {
        // a new log: every identity goes into it
        log_start_count = logger->get_log_start_count();
        for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
            if (state[i] != nullptr) {
                state[i]->logged = false;
            }
        }
    }
    const uint64_t now_us = AP_HAL::micros64();
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        State *s = state[i];
        if (s == nullptr) {
            continue;
        }
        if (s->settings != nullptr && s->settings_log == 0) {
            // once a boot, into whichever log is open, in rows of 32 bytes
            for (uint8_t ofs = 0; ofs < SETTINGS_LEN; ofs += 32) {
                char hex[65];
                for (uint8_t j = 0; j < 32; j++) {
                    hex[j * 2] = hex_digits[s->settings[ofs + j] >> 4];
                    hex[j * 2 + 1] = hex_digits[s->settings[ofs + j] & 0xF];
                }
                hex[64] = '\0';
                // @LoggerMessage: ESCE
                // @Description: An AM32 ESC's 192 byte settings block, once a boot
                // @Field: TimeUS: Time since system startup
                // @Field: I: ESC instance, starts from 0
                // @Field: Ofs: offset of these 32 bytes in the block
                // @Field: Hex: the 32 bytes, in hex
                logger->WriteCritical("ESCE", "TimeUS,I,Ofs,Hex", "s#--", "F---", "QBBZ",
                                      now_us, i, ofs, hex);
            }
            s->settings_log = logger->find_last_log();
            delete[] s->settings;
            s->settings = nullptr;
            // and an ESCI in this log that says so
            s->logged = false;
        }
        if (!s->logged) {
            const Identity &id = s->id;
            // @LoggerMessage: ESCI
            // @Description: An AM32 ESC's identity, in every log
            // @Field: TimeUS: Time since system startup
            // @Field: I: ESC instance, starts from 0
            // @Field: Src: where it came from: 1 serial telemetry info frame only, 2 serial telemetry, 3 extended DShot telemetry, 4 DroneCAN
            // @Field: Ver: firmware version, major * 100 + minor
            // @Field: Typ: firmware type, as MAVLink FIRMWARE_VERSION_TYPE
            // @Field: Fl: firmware flags, 1 built from a tree with uncommitted changes, 2 a CAN build
            // @Field: Hash: firmware git hash, the first 8 hex digits, 0 unknown
            // @Field: EE: settings layout (EEPROM) version
            // @Field: BL: bootloader version
            // @Field: BLP: bootloader protocol version, 0 for a bootloader without devinfo
            // @Field: BLT: bootloader type, as Typ, 1 for a dev build from a tree with uncommitted changes
            // @Field: BLH: bootloader git hash, 0 unknown
            // @Field: SCRC: CRC-16/CCITT of the settings block
            // @Field: SLog: the log holding this boot's settings block (ESCE), 0 not yet written
            // @Field: Name: FILE_NAME, the target the firmware was built for
            logger->WriteCritical("ESCI", "TimeUS,I,Src,Ver,Typ,Fl,Hash,EE,BL,BLP,BLT,BLH,SCRC,SLog,Name",
                                  "s#-------------", "F--------------", "QBBHBBIBBBBIHHZ",
                                  now_us, i, uint8_t(id.source),
                                  uint16_t(id.fw_major * 100U + id.fw_minor), id.fw_type, id.fw_flags, id.fw_hash,
                                  id.eeprom_version, id.bl_version, id.bl_protocol, id.bl_type, id.bl_hash,
                                  id.settings_crc, s->settings_log, id.name);
            s->logged = true;
        }
    }
#endif // HAL_LOGGING_ENABLED
}

static const char *fw_type_suffix(uint8_t fw_type)
{
    switch (fw_type) {
    case 0:
    case 1:
        return "-dev";
    case 64:
        return "-alpha";
    case 128:
        return "-beta";
    case 192:
        return "-rc";
    default:
        return "";
    }
}

// " 7eaa34e4", and "+" for a build from a tree with uncommitted changes
static void format_hash(char *buf, uint32_t hash, bool dirty)
{
    buf[0] = ' ';
    for (uint8_t i = 0; i < 8; i++) {
        buf[1 + i] = hex_digits[(hash >> (28 - 4 * i)) & 0xF];
    }
    buf[9] = dirty ? '+' : '\0';
    buf[10] = '\0';
}

void AP_ESC_Identity::format_firmware(const Identity &id, uint8_t first, uint8_t last, char *buf, uint8_t buflen) const
{
    char escs[12];
    if (first == last) {
        hal.util->snprintf(escs, sizeof(escs), "ESC%u", unsigned(first + 1));
    } else {
        hal.util->snprintf(escs, sizeof(escs), "ESC%u-%u", unsigned(first + 1), unsigned(last + 1));
    }
    // the info frame has no type or hashes
    const bool full = id.source != Source::SERIAL_INFO;
    char fw_hash[11] = "";
    if (full && id.fw_hash != 0) {
        format_hash(fw_hash, id.fw_hash, id.fw_flags & 1);
    }
    char bl_hash[11] = "";
    if (full && id.bl_known && id.bl_hash != 0) {
        format_hash(bl_hash, id.bl_hash, id.bl_type == 1);
    }
    hal.util->snprintf(buf, buflen, "%s AM32 %u.%02u%s%s BL%u%s",
                       escs, unsigned(id.fw_major), unsigned(id.fw_minor),
                       full ? fw_type_suffix(id.fw_type) : "", fw_hash,
                       unsigned(id.bl_version), bl_hash);
}

bool AP_ESC_Identity::get_firmware_line(uint8_t &next, char *buf, uint8_t buflen) const
{
    WITH_SEMAPHORE(sem);
    for (uint8_t i = next; i < ESC_TELEM_MAX_ESCS; i++) {
        const State *s = state[i];
        if (s == nullptr || s->id.source == Source::NONE) {
            continue;
        }
        // a run of ESCs with the same firmware is one line
        uint8_t last = i;
        while (last + 1 < ESC_TELEM_MAX_ESCS && state[last + 1] != nullptr &&
               state[last + 1]->id.source != Source::NONE && same_firmware(s->id, state[last + 1]->id)) {
            last++;
        }
        format_firmware(s->id, i, last, buf, buflen);
        next = last + 1;
        return true;
    }
    return false;
}

void AP_ESC_Identity::announce(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_announce_ms < 1000) {
        return;
    }
    last_announce_ms = now;
    WITH_SEMAPHORE(sem);
    for (uint8_t i = 0; i < ESC_TELEM_MAX_ESCS; i++) {
        State *s = state[i];
        if (s == nullptr || s->announced || now - s->learned_ms < ANNOUNCE_SETTLE_MS) {
            continue;
        }
        uint8_t last = i;
        while (last + 1 < ESC_TELEM_MAX_ESCS && state[last + 1] != nullptr &&
               !state[last + 1]->announced && same_firmware(s->id, state[last + 1]->id)) {
            last++;
        }
        char line[50];
        format_firmware(s->id, i, last, line, sizeof(line));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s", line);
        for (uint8_t j = i; j <= last; j++) {
            state[j]->announced = true;
        }
        i = last;
    }
}

void AP_ESC_Identity::update()
{
    update_edt();
    request_settings_over_edt();
    write_logs();
    announce();
}

#endif // AP_ESC_TELEM_AM32_IDENTITY_ENABLED
