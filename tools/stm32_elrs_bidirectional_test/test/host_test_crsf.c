/* Host-side unit test for the CRSF protocol layer (crsf.c/crsf.h). Compiled
   and run with the native compiler, no hardware/toolchain involved. Verifies
   pack/unpack round-trip, tick<->us conversion, frame building, and the
   streaming parser before anything is cross-compiled for the STM32. */
#include <stdio.h>
#include <string.h>
#include "../src/crsf.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

static void test_tick_round_trip(void) {
  int samples[] = {988, 1000, 1500, 1750, 2012, 1};
  for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++) {
    uint16_t tick = crsf_us_to_tick(samples[i]);
    int back = crsf_tick_to_us(tick);
    int clamped = samples[i] < 988 ? 988 : (samples[i] > 2012 ? 2012 : samples[i]);
    int diff = back - clamped;
    if (diff < 0) diff = -diff;
    CHECK(diff <= 1, "tick round-trip within 1us");
  }
  CHECK(crsf_us_to_tick(1500) == 992, "1500us maps to CRSF center tick 992");
}

static void test_channel_pack_round_trip(void) {
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = (uint16_t)(172 + i * 100);
  uint8_t payload[22];
  crsf_pack_channels(ch, payload);
  uint16_t out[16];
  crsf_unpack_channels(payload, out);
  for (int i = 0; i < 16; i++) {
    CHECK(out[i] == ch[i], "packed/unpacked channel value matches");
  }
}

static void test_rc_frame_parses_back(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1000 + i * 50;
  uint8_t frame[CRSF_RC_FRAME_SIZE];
  crsf_build_rc_frame(frame, ch_us);

  CHECK(frame[0] == CRSF_TX_ADDR, "rc frame address is CRSF_TX_ADDR");
  CHECK(frame[1] == 24, "rc frame length field is 24");
  CHECK(frame[2] == CRSF_FRAME_RC_CHANNELS, "rc frame type is RC_CHANNELS");

  CrsfParser parser;
  crsf_parser_init(&parser);
  CrsfFrame out;
  bool got = false;
  for (uint8_t i = 0; i < CRSF_RC_FRAME_SIZE; i++) {
    got = crsf_parser_push(&parser, frame[i], &out);
  }
  CHECK(got, "parser accepts a freshly built rc frame");
  CHECK(out.type == CRSF_FRAME_RC_CHANNELS, "parsed type is RC_CHANNELS");
  CHECK(out.payload_len == CRSF_RC_PAYLOAD_SIZE, "parsed payload length is 22");

  uint16_t tick[16];
  crsf_unpack_channels(out.payload, tick);
  for (int i = 0; i < 16; i++) {
    int back = crsf_tick_to_us(tick[i]);
    int diff = back - ch_us[i];
    if (diff < 0) diff = -diff;
    CHECK(diff <= 1, "parsed rc channel value matches source within 1us");
  }
}

static void test_battery_frame_parses_back(void) {
  uint8_t frame[16];
  uint8_t len = crsf_build_battery_frame(frame, 126, 5, 1234, 77);
  CHECK(len == 12, "battery frame length is 12 bytes");
  CHECK(frame[0] == CRSF_ADDR_FC, "battery frame address is CRSF_ADDR_FC");
  CHECK(frame[2] == CRSF_FRAME_BATTERY, "battery frame type is BATTERY");

  CrsfParser parser;
  crsf_parser_init(&parser);
  CrsfFrame out;
  bool got = false;
  for (uint8_t i = 0; i < len; i++) {
    got = crsf_parser_push(&parser, frame[i], &out);
  }
  CHECK(got, "parser accepts a freshly built battery frame");
  CHECK(out.type == CRSF_FRAME_BATTERY, "parsed type is BATTERY");
  CHECK(out.payload_len == 8, "parsed battery payload length is 8");
  uint16_t voltage = (uint16_t)((out.payload[0] << 8) | out.payload[1]);
  CHECK(voltage == 126, "parsed battery voltage round-trips");
}

static void test_parser_rejects_bad_crc(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1500;
  uint8_t frame[CRSF_RC_FRAME_SIZE];
  crsf_build_rc_frame(frame, ch_us);
  frame[25] ^= 0xFF; /* corrupt CRC */

  CrsfParser parser;
  crsf_parser_init(&parser);
  CrsfFrame out;
  bool got = false;
  for (uint8_t i = 0; i < CRSF_RC_FRAME_SIZE; i++) {
    got = crsf_parser_push(&parser, frame[i], &out) || got;
  }
  CHECK(!got, "parser rejects a frame with a corrupted CRC");
}

static void test_parser_resyncs_on_garbage_prefix(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1600;
  uint8_t frame[CRSF_RC_FRAME_SIZE];
  crsf_build_rc_frame(frame, ch_us);

  CrsfParser parser;
  crsf_parser_init(&parser);
  CrsfFrame out;
  bool got = false;
  uint8_t garbage[] = {0x11, 0x22, 0x33};
  for (size_t i = 0; i < sizeof(garbage); i++) {
    got = crsf_parser_push(&parser, garbage[i], &out) || got;
  }
  CHECK(!got, "garbage prefix alone produces no frame");
  for (uint8_t i = 0; i < CRSF_RC_FRAME_SIZE; i++) {
    got = crsf_parser_push(&parser, frame[i], &out);
  }
  CHECK(got, "parser resyncs and parses a valid frame after garbage bytes");
}

int main(void) {
  test_tick_round_trip();
  test_channel_pack_round_trip();
  test_rc_frame_parses_back();
  test_battery_frame_parses_back();
  test_parser_rejects_bad_crc();
  test_parser_resyncs_on_garbage_prefix();

  if (failures == 0) {
    printf("All CRSF host tests passed.\n");
    return 0;
  }
  printf("%d CRSF host test failure(s).\n", failures);
  return 1;
}
