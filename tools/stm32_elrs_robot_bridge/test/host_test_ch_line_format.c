/* Host-side unit test for ch_line_format.c/h. Native compiler, no hardware
   or CRSF framing/UART involved. */
#include <stdio.h>
#include <string.h>
#include "ch_line_format.h"
#include "tel_channel_map.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

static void test_formats_expected_fields_in_order(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = 1500;
  ch_us[TEL_CH_DRIVE_FORWARD] = 1800;
  ch_us[TEL_CH_DRIVE_TURN] = 1200;
  ch_us[TEL_CH_MECHANISM] = 1000;
  ch_us[TEL_CH_AUXILIARY] = 2000;
  ch_us[TEL_CH_FIRE] = 999;
  ch_us[TEL_CH_MODE] = 1750;

  char buf[CH_LINE_MAX_CHARS];
  uint16_t len = format_ch_line(buf, sizeof(buf), ch_us);
  CHECK(len > 0, "format_ch_line reports a non-zero length for a large-enough buffer");
  CHECK(strcmp(buf, "CH,1800,1200,1000,2000,999,1750\n") == 0,
        "formatted line matches the expected field order and values");
  CHECK(len == (uint16_t)strlen(buf), "returned length matches the actual string length");
}

static void test_handles_negative_values(void) {
  int ch_us[16];
  for (int i = 0; i < 16; i++) ch_us[i] = -500;
  char buf[CH_LINE_MAX_CHARS];
  format_ch_line(buf, sizeof(buf), ch_us);
  CHECK(strcmp(buf, "CH,-500,-500,-500,-500,-500,-500\n") == 0,
        "negative channel values format with a leading minus sign");
}

static void test_rejects_undersized_buffer(void) {
  int ch_us[16] = {0};
  char buf[4] = {'x', 'x', 'x', 'x'};
  uint16_t len = format_ch_line(buf, sizeof(buf), ch_us);
  CHECK(len == 0, "an undersized buffer reports zero length rather than overflowing");
  CHECK(buf[0] == '\0', "an undersized buffer is left as an empty string");
}

int main(void) {
  test_formats_expected_fields_in_order();
  test_handles_negative_values();
  test_rejects_undersized_buffer();

  if (failures == 0) {
    printf("All ch_line_format host tests passed.\n");
    return 0;
  }
  printf("%d ch_line_format host test failure(s).\n", failures);
  return 1;
}
