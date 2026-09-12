#include "crsf.h"

uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint16_t crsf_us_to_tick(int us) {
  if (us < 988) us = 988;
  if (us > 2012) us = 2012;
  int tick = ((us - 1500) * 8 / 5) + 992;
  if (tick < 0) tick = 0;
  if (tick > 2047) tick = 2047;
  return (uint16_t)tick;
}

int crsf_tick_to_us(uint16_t tick) {
  return ((int)tick - 992) * 5 / 8 + 1500;
}

void crsf_pack_channels(const uint16_t ch[16], uint8_t payload[22]) {
  for (uint8_t i = 0; i < 22; i++) payload[i] = 0;

  uint32_t bitbuf = 0;
  uint8_t bits = 0;
  uint8_t out = 0;

  for (uint8_t i = 0; i < 16; i++) {
    bitbuf |= ((uint32_t)(ch[i] & 0x07FFu)) << bits;
    bits += 11;
    while (bits >= 8) {
      payload[out++] = (uint8_t)(bitbuf & 0xFFu);
      bitbuf >>= 8;
      bits -= 8;
    }
  }
}

void crsf_unpack_channels(const uint8_t payload[22], uint16_t ch[16]) {
  uint32_t bitbuf = 0;
  uint8_t bits = 0;
  uint8_t in = 0;

  for (uint8_t i = 0; i < 16; i++) {
    while (bits < 11 && in < 22) {
      bitbuf |= ((uint32_t)payload[in++]) << bits;
      bits += 8;
    }
    ch[i] = (uint16_t)(bitbuf & 0x07FFu);
    bitbuf >>= 11;
    bits -= 11;
  }
}

void crsf_build_rc_frame(uint8_t frame[26], const int ch_us[16]) {
  uint16_t ch_tick[16];
  for (uint8_t i = 0; i < 16; i++) {
    ch_tick[i] = crsf_us_to_tick(ch_us[i]);
  }

  frame[0] = (uint8_t)CRSF_TX_ADDR;
  frame[1] = 24; /* type(1) + payload(22) + crc(1) */
  frame[2] = CRSF_FRAME_RC_CHANNELS;
  crsf_pack_channels(ch_tick, &frame[3]);
  frame[25] = crsf_crc8(&frame[2], 23);
}

uint8_t crsf_build_battery_frame(uint8_t *frame, uint16_t voltage_01v,
                                  uint16_t current_01a, uint32_t capacity_mah,
                                  uint8_t remaining_percent) {
  frame[0] = CRSF_ADDR_FC;
  frame[1] = 10; /* type(1) + payload(8) + crc(1) */
  frame[2] = CRSF_FRAME_BATTERY;

  frame[3] = (uint8_t)(voltage_01v >> 8);
  frame[4] = (uint8_t)(voltage_01v & 0xFF);
  frame[5] = (uint8_t)(current_01a >> 8);
  frame[6] = (uint8_t)(current_01a & 0xFF);
  frame[7] = (uint8_t)((capacity_mah >> 16) & 0xFF);
  frame[8] = (uint8_t)((capacity_mah >> 8) & 0xFF);
  frame[9] = (uint8_t)(capacity_mah & 0xFF);
  frame[10] = remaining_percent;

  frame[11] = crsf_crc8(&frame[2], 9);
  return 12;
}

static bool crsf_is_possible_address(uint8_t b) {
  switch (b) {
    case 0x00:
    case 0xC8:
    case 0xEA:
    case 0xEC:
    case 0xEE:
      return true;
    default:
      return false;
  }
}

void crsf_parser_init(CrsfParser *p) {
  p->index = 0;
  p->expected_len = 0;
  for (uint8_t i = 0; i < CRSF_MAX_FRAME_SIZE; i++) p->buf[i] = 0;
}

bool crsf_parser_push(CrsfParser *p, uint8_t b, CrsfFrame *out) {
  if (p->index == 0) {
    if (!crsf_is_possible_address(b)) {
      return false;
    }
  }

  p->buf[p->index++] = b;

  if (p->index == 2) {
    p->expected_len = p->buf[1];
    if (p->expected_len < 2 || p->expected_len > 62) {
      p->index = 0;
      p->expected_len = 0;
      return false;
    }
  }

  if (p->index >= 2 && p->expected_len > 0) {
    uint8_t total = (uint8_t)(p->expected_len + 2);
    if (p->index >= total) {
      uint8_t crc_calc = crsf_crc8(&p->buf[2], (uint8_t)(p->expected_len - 1));
      uint8_t crc_rx = p->buf[total - 1];

      if (crc_calc == crc_rx) {
        out->address = p->buf[0];
        out->length = p->buf[1];
        out->type = p->buf[2];
        out->payload_len = (uint8_t)(p->expected_len - 2);
        for (uint8_t i = 0; i < out->payload_len; i++) {
          out->payload[i] = p->buf[3 + i];
        }
        p->index = 0;
        p->expected_len = 0;
        return true;
      }

      p->index = 0;
      p->expected_len = 0;
      return false;
    }
  }

  if (p->index >= CRSF_MAX_FRAME_SIZE) {
    p->index = 0;
    p->expected_len = 0;
  }

  return false;
}
