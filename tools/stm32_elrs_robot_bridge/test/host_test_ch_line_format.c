/* Host-side unit test for ch_line_format.c (RC line out, ST line in,
   forward gate). Native compiler, no hardware or UART involved. */
#include <stdio.h>
#include <string.h>
#include "ch_line_format.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

/* text + "*HH" (+ '\n' when nl) into out, the way the other end writes it. */
static const char *sealed(const char *text, char *out, int nl) {
  uint16_t n = (uint16_t)strlen(text);
  memcpy(out, text, n);
  n = rcmap_line_seal(out, n, 120u);
  if (nl) out[n++] = '\n';
  out[n] = '\0';
  return out;
}

static void test_rc_line(void) {
  char want[128];
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1500;
  ch_us[0] = 1800; ch_us[1] = 1200; ch_us[2] = 1000; ch_us[3] = 2000;
  ch_us[4] = 988; ch_us[5] = 2012; ch_us[6] = 1500; ch_us[7] = 1193; ch_us[8] = 1602;
  ch_us[9] = 2000; ch_us[10] = 1111; /* beyond RCMAP_CHANNEL_COUNT: must not appear */

  char buf[CH_LINE_MAX_CHARS];
  uint16_t len = format_rc_line(buf, sizeof(buf), ch_us);
  CHECK(strcmp(buf, sealed("RC,1800,1200,1000,2000,988,2012,1500,1193,1602,2000", want, 1)) == 0,
        "RC line carries exactly channels 1-10 in order, then its check");
  CHECK(buf[len - 4] == '*', "the check sits right before the newline");
  CHECK(rcmap_line_open(buf, (uint16_t)(len - 1)) == (int)(len - 4), "and the Mega's check accepts it");
  /* One digit flipped on the wire ('0' -> '8' is one bit: 1000 -> 1800,
     enough to read as "pressed" on the fire / arm channels) must fail the
     check. */
  char flipped[CH_LINE_MAX_CHARS];
  memcpy(flipped, buf, len);
  flipped[14] = '8';
  CHECK(strncmp(flipped, "RC,1800,1200,1800,", 18) == 0, "test flips CH3's hundreds digit");
  CHECK(rcmap_line_open(flipped, (uint16_t)(len - 1)) < 0, "a flipped digit fails the check");
  CHECK(len == (uint16_t)strlen(buf), "returned length matches the string");

  for (int i = 0; i < 16; i++) ch_us[i] = -500;
  format_rc_line(buf, sizeof(buf), ch_us);
  CHECK(strcmp(buf, sealed("RC,-500,-500,-500,-500,-500,-500,-500,-500,-500,-500", want, 1)) == 0,
        "negative values keep their sign");

  char small[8] = "xxxxxxx";
  CHECK(format_rc_line(small, sizeof(small), ch_us) == 0 && small[0] == '\0',
        "an undersized buffer reports zero length instead of overflowing");
}

static void test_forward_gate(void) {
  CHECK(!ch_line_should_forward(false, 1000u, 0u, 500u), "nothing before the first RC frame");
  CHECK(ch_line_should_forward(true, 1000u, 1000u, 500u), "a fresh frame is forwarded");
  CHECK(ch_line_should_forward(true, 1499u, 1000u, 500u), "still forwarded at 499 ms");
  CHECK(!ch_line_should_forward(true, 1500u, 1000u, 500u), "silent at exactly stale_ms (RF lost)");
  CHECK(!ch_line_should_forward(true, 90000u, 1000u, 500u), "silent for a long outage");
  CHECK(ch_line_should_forward(true, 5u, 0xFFFFFFF0u, 500u), "millis wrap-around isn't stale");
}

static void test_status_line(void) {
  rcmap_status s;
  char line[128];
  const char *ok = sealed("ST,67,123456,10", line, 0);
  CHECK(parse_status_line(ok, (uint16_t)strlen(ok), &s), "well-formed status parses");
  CHECK(s.flags == 67 && s.event_seq == 123456u && s.event_code == 10, "fields land in place");

  /* Well-formed checks around bad fields, and good fields with a bad or
     missing check (an old Mega, a flipped digit, lower-case hex). */
  const char *bad[] = {
    "ST,256,1,1", "ST,1,16777216,1", "ST,1,1,65536", "ST,1,1", "ST,1,1,1,1",
    "ST,,1,1", "ST,a,1,1", "XX,1,1,1", "ST,1,1,1x", "ST,-1,1,1", "ST",
  };
  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    rcmap_status untouched = {9, 9, 9, 9};
    const char *b = sealed(bad[i], line, 0);
    CHECK(!parse_status_line(b, (uint16_t)strlen(b), &untouched), bad[i]);
    CHECK(untouched.flags == 9 && untouched.event_seq == 9, "rejected line leaves output untouched");
  }
  const char *no_check = "ST,75,123456,10";
  CHECK(!parse_status_line(no_check, (uint16_t)strlen(no_check), &s), "a line without the check is dropped");
  sealed("ST,75,123456,10", line, 0);
  line[4] = '4';                                     /* flags 75 -> 74: armed bit lost */
  CHECK(!parse_status_line(line, (uint16_t)strlen(line), &s), "a flipped flags digit is dropped");
  sealed("ST,75,123456,10", line, 0);
  for (char *c = line; *c; c++) if (*c >= 'A' && *c <= 'F') *c = (char)(*c - 'A' + 'a');
  if (strpbrk(line + strlen(line) - 2, "abcdef")) {
    CHECK(!parse_status_line(line, (uint16_t)strlen(line), &s), "lower-case hex is not accepted");
  }
  /* An aim frame (2026-10-17) is just another ST line within the same
     field ranges: the bridge passes it through unchanged. This is the
     line the Mega prints for 3.52 m, 1620 us, key 1 (vision 10) seen,
     aim offset -15 px. */
  const char *aim = sealed("ST,241,1746516,33120", line, 0);
  CHECK(parse_status_line(aim, (uint16_t)strlen(aim), &s), "aim frame line parses");
  rcmap_aim a;
  CHECK(rcmap_aim_from_status(&s, &a), "and decodes as an aim frame");
  CHECK(a.distance_cm == 352 && a.elevation_pulse_us == 1620 && a.selected_id == 10 &&
        a.target_id == 10 && a.valid && a.aim_offset_px == -15, "aim fields land in place");
}

int main(void) {
  test_rc_line();
  test_forward_gate();
  test_status_line();
  if (failures == 0) {
    printf("All ch_line_format host tests passed.\n");
    return 0;
  }
  printf("%d ch_line_format host test failure(s).\n", failures);
  return 1;
}
