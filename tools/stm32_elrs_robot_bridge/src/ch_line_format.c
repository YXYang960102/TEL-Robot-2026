#include "ch_line_format.h"

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

uint16_t format_rc_line(char *buf, uint16_t bufSize, const int ch_us[16]) {
  if (bufSize < CH_LINE_MAX_CHARS) {
    if (bufSize > 0) buf[0] = '\0';
    return 0;
  }

  const uint16_t limit = (uint16_t)(bufSize - 2u); /* reserve '\n' + '\0' */
  uint16_t pos = 0;
  buf[pos++] = 'R';
  buf[pos++] = 'C';
  buf[pos++] = ',';
  for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
    if (i > 0 && pos < limit) buf[pos++] = ',';
    pos = append_int(buf, pos, limit, ch_us[i]);
  }
  buf[pos++] = '\n';
  buf[pos] = '\0';
  return pos;
}

bool ch_line_should_forward(bool ever_received, uint32_t now_ms,
                            uint32_t last_rc_rx_ms, uint32_t stale_ms) {
  return ever_received && (uint32_t)(now_ms - last_rc_rx_ms) < stale_ms;
}

/* Parses an unsigned decimal field ending at ',' or at end. */
static bool parse_field(const char *line, uint16_t len, uint16_t *pos,
                        uint32_t max, bool last, uint32_t *out) {
  uint32_t v = 0;
  uint16_t digits = 0;
  while (*pos < len && line[*pos] >= '0' && line[*pos] <= '9') {
    v = v * 10u + (uint32_t)(line[*pos] - '0');
    if (v > max || ++digits > 9u) return false;
    (*pos)++;
  }
  if (digits == 0u) return false;
  if (last) {
    if (*pos != len) return false;
  } else {
    if (*pos >= len || line[*pos] != ',') return false;
    (*pos)++;
  }
  *out = v;
  return true;
}

bool parse_status_line(const char *line, uint16_t len, rcmap_status *out) {
  if (len < 3u || line[0] != 'S' || line[1] != 'T' || line[2] != ',') return false;
  uint16_t pos = 3;
  uint32_t flags, seq, code;
  if (!parse_field(line, len, &pos, 255u, false, &flags) ||
      !parse_field(line, len, &pos, 0xFFFFFFu, false, &seq) ||
      !parse_field(line, len, &pos, 65535u, true, &code)) {
    return false;
  }
  out->flags = (uint8_t)flags;
  out->event_seq = seq;
  out->event_code = (uint16_t)code;
  out->rolling = 0;
  return true;
}
