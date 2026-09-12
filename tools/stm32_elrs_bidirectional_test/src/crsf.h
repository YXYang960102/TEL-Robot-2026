#ifndef CRSF_H
#define CRSF_H

/* CRSF protocol layer, ported from the reference elrs_f401ccu6_platformio
   project's crsf.c/crsf.h (crsf.c pack/unpack/CRC logic, unmodified in
   substance). Adapted to avoid <string.h>: this firmware links with
   -nostdlib, so memset()/memcpy() have no implementation available;
   pack/unpack/parser-init use explicit byte loops instead. */

#include <stdint.h>
#include <stdbool.h>

#ifndef CRSF_TX_ADDR
#define CRSF_TX_ADDR 0xEEu
#endif

#define CRSF_ADDR_FC             0xC8u
#define CRSF_FRAME_RC_CHANNELS   0x16u
#define CRSF_FRAME_BATTERY       0x08u
#define CRSF_MAX_FRAME_SIZE      64u
#define CRSF_RC_PAYLOAD_SIZE     22u
#define CRSF_RC_FRAME_SIZE       26u

uint8_t crsf_crc8(const uint8_t *data, uint8_t len);
uint16_t crsf_us_to_tick(int us);
int crsf_tick_to_us(uint16_t tick);
void crsf_pack_channels(const uint16_t ch[16], uint8_t payload[22]);
void crsf_unpack_channels(const uint8_t payload[22], uint16_t ch[16]);
void crsf_build_rc_frame(uint8_t frame[26], const int ch_us[16]);
uint8_t crsf_build_battery_frame(uint8_t *frame, uint16_t voltage_01v,
                                  uint16_t current_01a, uint32_t capacity_mah,
                                  uint8_t remaining_percent);

typedef struct {
  uint8_t buf[CRSF_MAX_FRAME_SIZE];
  uint8_t index;
  uint8_t expected_len;
} CrsfParser;

typedef struct {
  uint8_t address;
  uint8_t length;
  uint8_t type;
  uint8_t payload_len;
  uint8_t payload[CRSF_MAX_FRAME_SIZE];
} CrsfFrame;

void crsf_parser_init(CrsfParser *p);
bool crsf_parser_push(CrsfParser *p, uint8_t b, CrsfFrame *out);

#endif
