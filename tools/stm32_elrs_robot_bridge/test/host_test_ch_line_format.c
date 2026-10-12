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

static void test_rc_line(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1500;
  ch_us[0] = 1800; ch_us[1] = 1200; ch_us[2] = 1000; ch_us[3] = 2000;
  ch_us[4] = 988; ch_us[5] = 2012; ch_us[6] = 1500; ch_us[7] = 1193; ch_us[8] = 1602;
  ch_us[9] = 2000; ch_us[10] = 1111; /* beyond RCMAP_CHANNEL_COUNT: must not appear */

  char buf[CH_LINE_MAX_CHARS];
  uint16_t len = format_rc_line(buf, sizeof(buf), ch_us);
  CHECK(strcmp(buf, "RC,1800,1200,1000,2000,988,2012,1500,1193,1602,2000\n") == 0,
        "RC line carries exactly channels 1-10 in order");
  CHECK(len == (uint16_t)strlen(buf), "returned length matches the string");

  for (int i = 0; i < 16; i++) ch_us[i] = -500;
  format_rc_line(buf, sizeof(buf), ch_us);
  CHECK(strcmp(buf, "RC,-500,-500,-500,-500,-500,-500,-500,-500,-500,-500\n") == 0,
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
  const char *ok = "ST,67,123456,10";
  CHECK(parse_status_line(ok, (uint16_t)strlen(ok), &s), "well-formed status parses");
  CHECK(s.flags == 67 && s.event_seq == 123456u && s.event_code == 10, "fields land in place");

  const char *bad[] = {
    "ST,256,1,1", "ST,1,16777216,1", "ST,1,1,65536", "ST,1,1", "ST,1,1,1,1",
    "ST,,1,1", "ST,a,1,1", "XX,1,1,1", "ST,1,1,1x", "ST,-1,1,1", "ST",
  };
  for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
    rcmap_status untouched = {9, 9, 9, 9};
    CHECK(!parse_status_line(bad[i], (uint16_t)strlen(bad[i]), &untouched), bad[i]);
    CHECK(untouched.flags == 9 && untouched.event_seq == 9, "rejected line leaves output untouched");
  }
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
