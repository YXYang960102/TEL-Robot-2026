#include "ch_line_format.h"

#include "tel_channel_map.h"

static uint16_t append_int(char *buf, uint16_t pos, uint16_t limit, int value) {
  if (value < 0) {
    if (pos < limit) buf[pos++] = '-';
    value = -value;
  }
  char digits[6];
  uint8_t n = 0;
  if (value == 0) {
    digits[n++] = '0';
  } else {
    while (value > 0 && n < (uint8_t)sizeof(digits)) {
      digits[n++] = (char)('0' + (value % 10));
      value /= 10;
    }
  }
  while (n > 0 && pos < limit) {
    buf[pos++] = digits[--n];
  }
  return pos;
}

uint16_t format_ch_line(char *buf, uint16_t bufSize, const int ch_us[16]) {
  if (bufSize < CH_LINE_MAX_CHARS) {
    if (bufSize > 0) buf[0] = '\0';
    return 0;
  }

  const uint16_t limit = (uint16_t)(bufSize - 2u); /* reserve '\n' + '\0' */
  uint16_t pos = 0;
  const char *tag = "CH,";
  for (uint8_t i = 0; tag[i] != '\0'; i++) buf[pos++] = tag[i];

  const uint8_t indices[6] = {
    TEL_CH_DRIVE_FORWARD, TEL_CH_DRIVE_TURN, TEL_CH_MECHANISM,
    TEL_CH_AUXILIARY, TEL_CH_FIRE, TEL_CH_MODE
  };
  for (uint8_t i = 0; i < 6; i++) {
    if (i > 0 && pos < limit) buf[pos++] = ',';
    pos = append_int(buf, pos, limit, ch_us[indices[i]]);
  }
  buf[pos++] = '\n';
  buf[pos] = '\0';
  return pos;
}
