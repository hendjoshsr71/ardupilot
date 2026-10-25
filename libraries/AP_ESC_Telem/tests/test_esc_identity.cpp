/*
  AM32 identity and settings records, as the three links deliver them

  The vectors are what AM32's SITL build sends: its answer to DShot
  command 6 (the info frame and the identity record) and to command 11
  (the settings record) on the serial telemetry wire, and one pass of each
  of its EDT streams, as the bytes of the symbols a flight controller keeps
  after dropping repeats.
 */
#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_ESC_Telem/AP_ESC_Identity.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_ESC_TELEM_AM32_IDENTITY_ENABLED

#include "test_esc_identity_vectors.h"

using Identity = AP_ESC_Identity::Identity;
using Source = AP_ESC_Identity::Source;

static const uint8_t *identity_record = &info_answer[AP_ESC_Identity::INFO_FRAME_LEN];
static const uint16_t identity_record_len = sizeof(info_answer) - AP_ESC_Identity::INFO_FRAME_LEN;

TEST(ESCIdentity, the_serial_answers_parse)
{
    Identity info;
    ASSERT_TRUE(AP_ESC_Identity::parse_info_frame(info_answer, sizeof(info_answer), info));
    EXPECT_EQ(info.source, Source::SERIAL_INFO);
    EXPECT_EQ(info.fw_major, 2);
    EXPECT_EQ(info.fw_minor, 20);

    ASSERT_EQ(AP_ESC_Identity::record_length(identity_record, identity_record_len, AP_ESC_Identity::KIND_IDENTITY),
              identity_record_len);
    Identity id;
    ASSERT_TRUE(AP_ESC_Identity::parse_identity(identity_record, identity_record_len, Source::SERIAL, id));
    EXPECT_EQ(id.fw_major, 2);
    EXPECT_EQ(id.fw_minor, 20);
    EXPECT_EQ(id.fw_hash, EXPECTED_FW_HASH);
    EXPECT_TRUE(id.fw_flags & 2);   // the SITL target is a CAN build
    EXPECT_EQ(id.eeprom_version, info.eeprom_version);
    EXPECT_EQ(id.bl_version, info.bl_version);
    EXPECT_EQ(id.bl_protocol, 0);   // SITL has no bootloader devinfo
    EXPECT_FALSE(id.bl_known);
    EXPECT_STREQ(id.name, "AM32_SITL_CAN");

    ASSERT_EQ(AP_ESC_Identity::record_length(settings_answer, sizeof(settings_answer), AP_ESC_Identity::KIND_SETTINGS),
              uint8_t(AP_ESC_Identity::SETTINGS_RECORD_LEN));
    const uint8_t *block = &settings_answer[4];
    EXPECT_EQ(AP_ESC_Identity::crc16(block, AP_ESC_Identity::SETTINGS_LEN), id.settings_crc);
    // the info frame is the block's first 48 bytes
    EXPECT_EQ(memcmp(block, info_answer, 48), 0);
}

TEST(ESCIdentity, a_damaged_record_is_refused)
{
    uint8_t rec[sizeof(info_answer)];
    memcpy(rec, identity_record, identity_record_len);
    rec[10] ^= 0x01;
    EXPECT_EQ(AP_ESC_Identity::record_length(rec, identity_record_len, AP_ESC_Identity::KIND_IDENTITY), 0);
    // short, or asked for as the other kind
    EXPECT_EQ(AP_ESC_Identity::record_length(identity_record, identity_record_len - 1, AP_ESC_Identity::KIND_IDENTITY), 0);
    EXPECT_EQ(AP_ESC_Identity::record_length(identity_record, identity_record_len, AP_ESC_Identity::KIND_SETTINGS), 0);

    uint8_t frame[AP_ESC_Identity::INFO_FRAME_LEN];
    memcpy(frame, info_answer, sizeof(frame));
    frame[7] ^= 0x80;
    Identity id;
    EXPECT_FALSE(AP_ESC_Identity::parse_info_frame(frame, sizeof(frame), id));
}

static void feed(AP_ESC_Identity_EDT &r, const uint8_t *stream, uint16_t len, int16_t skip=-1)
{
    for (uint16_t i = 0; i < len; i++) {
        if (i != skip) {
            r.symbol(stream[i]);
        }
    }
}

TEST(ESCIdentity, edt_gives_the_same_identity_and_settings)
{
    AP_ESC_Identity_EDT r;
    feed(r, edt_identity_pass, sizeof(edt_identity_pass));
    // a chunk is complete at the next delimiter
    r.symbol(0);
    uint8_t rec[64];
    const uint8_t len = r.take_identity(rec, sizeof(rec));
    ASSERT_GE(len, identity_record_len);
    EXPECT_EQ(memcmp(rec, identity_record, identity_record_len), 0);
    EXPECT_EQ(r.take_identity(rec, sizeof(rec)), 0) << "taken once";

    feed(r, edt_settings_pass, sizeof(edt_settings_pass));
    r.symbol(0);
    uint8_t block[AP_ESC_Identity::SETTINGS_LEN];
    uint16_t crc;
    ASSERT_TRUE(r.take_settings(block, crc));
    EXPECT_EQ(memcmp(block, &settings_answer[4], sizeof(block)), 0);
    EXPECT_EQ(crc, AP_ESC_Identity::crc16(&settings_answer[4], sizeof(block)));
}

TEST(ESCIdentity, a_lost_symbol_costs_one_pass)
{
    AP_ESC_Identity_EDT r;
    uint8_t rec[64];
    // drop a symbol inside the second chunk: that chunk fails its CRC8
    feed(r, edt_identity_pass, sizeof(edt_identity_pass), 22 + 5);
    r.symbol(0);
    EXPECT_EQ(r.take_identity(rec, sizeof(rec)), 0);
    // the next pass fills it in
    feed(r, edt_identity_pass, sizeof(edt_identity_pass));
    r.symbol(0);
    EXPECT_GE(r.take_identity(rec, sizeof(rec)), identity_record_len);
}

TEST(ESCIdentity, the_store_keeps_the_best_and_groups_the_same)
{
    AP_ESC_Identity store;
    EXPECT_TRUE(store.wants_identity(0));
    EXPECT_FALSE(store.wants_settings(0));

    // an AM32 without the records: version only, still wanting the identity
    ASSERT_TRUE(store.handle_info_frame(0, info_answer, AP_ESC_Identity::INFO_FRAME_LEN));
    EXPECT_TRUE(store.wants_identity(0));
    char line[50];
    uint8_t next = 0;
    ASSERT_TRUE(store.get_firmware_line(next, line, sizeof(line)));
    EXPECT_STREQ(line, EXPECTED_INFO_LINE);

    ASSERT_TRUE(store.handle_identity_record(0, Source::SERIAL, identity_record, identity_record_len));
    EXPECT_FALSE(store.wants_identity(0));
    EXPECT_TRUE(store.wants_settings(0));
    // the info frame never replaces it
    store.handle_info_frame(0, info_answer, AP_ESC_Identity::INFO_FRAME_LEN);
    EXPECT_FALSE(store.wants_identity(0));

    ASSERT_TRUE(store.handle_settings_record(0, Source::SERIAL, settings_answer, sizeof(settings_answer)));
    EXPECT_FALSE(store.wants_settings(0));

    // two ESCs with the same firmware are one line
    ASSERT_TRUE(store.handle_identity_record(1, Source::EDT, identity_record, identity_record_len));
    next = 0;
    ASSERT_TRUE(store.get_firmware_line(next, line, sizeof(line)));
    EXPECT_STREQ(line, EXPECTED_LINE_1_2);
    EXPECT_FALSE(store.get_firmware_line(next, line, sizeof(line)));
}

TEST(ESCIdentity, a_build_from_uncommitted_changes_is_marked)
{
    // the same record with its dirty flag set, and its CRC to match
    uint8_t rec[sizeof(info_answer)];
    memcpy(rec, identity_record, identity_record_len);
    rec[7] |= 1;
    const uint16_t crc = AP_ESC_Identity::crc16(rec, identity_record_len - 2);
    rec[identity_record_len - 2] = crc & 0xFF;
    rec[identity_record_len - 1] = crc >> 8;

    AP_ESC_Identity store;
    ASSERT_TRUE(store.handle_identity_record(0, Source::SERIAL, identity_record, identity_record_len));
    ASSERT_TRUE(store.handle_identity_record(1, Source::SERIAL, rec, identity_record_len));
    // and it is not the same firmware as the clean build of that commit
    char line[50];
    uint8_t next = 0;
    ASSERT_TRUE(store.get_firmware_line(next, line, sizeof(line)));
    EXPECT_STREQ(line, EXPECTED_LINE_1);
    ASSERT_TRUE(store.get_firmware_line(next, line, sizeof(line)));
    EXPECT_STREQ(line, EXPECTED_LINE_2_DIRTY);
    EXPECT_FALSE(store.get_firmware_line(next, line, sizeof(line)));
}

#endif // AP_ESC_TELEM_AM32_IDENTITY_ENABLED

AP_GTEST_MAIN()
