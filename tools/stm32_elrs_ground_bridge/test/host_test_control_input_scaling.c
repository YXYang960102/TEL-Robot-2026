/* Host-side unit test for control_input_scaling.c/h. */
#include <stdio.h>
#include "control_input_scaling.h"

static int failures = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); \
    failures++; \
  } \
} while (0)

static void test_adc_axis_endpoints_and_clamp(void) {
  CHECK(adc_axis_to_pulse_us(0) == 1000, "ADC 0 maps to 1000us");
  CHECK(adc_axis_to_pulse_us(4095) == 2000, "ADC 4095 maps to 2000us");
  CHECK(adc_axis_to_pulse_us(60000) == 2000, "out-of-range ADC input clamps to 2000us, doesn't wrap");
  uint16_t mid = adc_axis_to_pulse_us(2048);
  CHECK(mid > 1490 && mid < 1510, "mid-scale ADC input maps near the 1500us center");
}

static void test_digital_switch(void) {
  CHECK(digital_switch_to_pulse_us(true) == 2000, "active switch maps to 2000us");
  CHECK(digital_switch_to_pulse_us(false) == 1000, "inactive switch maps to 1000us");
}

static void test_mode_switch_thresholds_match_mega_side(void) {
  /* Mega-side thresholds (OperatorModeConstants): SEMI_AUTO_THRESHOLD=1667,
     FULL_MANUAL_THRESHOLD=1833. These outputs must land cleanly on each
     side of both thresholds. */
  uint16_t fullAuto = mode_switch_to_pulse_us(false, false);
  uint16_t semiAuto = mode_switch_to_pulse_us(true, false);
  uint16_t fullManual = mode_switch_to_pulse_us(false, true);
  uint16_t fullManualAlt = mode_switch_to_pulse_us(true, true);

  CHECK(fullAuto < 1667, "full-auto bit pattern lands below SEMI_AUTO_THRESHOLD");
  CHECK(semiAuto >= 1667 && semiAuto < 1833, "semi-auto bit pattern lands between the two thresholds");
  CHECK(fullManual >= 1833, "full-manual bit pattern lands at/above FULL_MANUAL_THRESHOLD");
  CHECK(fullManualAlt >= 1833, "the remaining (both-bits) combination also fails toward full-manual");
}

int main(void) {
  test_adc_axis_endpoints_and_clamp();
  test_digital_switch();
  test_mode_switch_thresholds_match_mega_side();

  if (failures == 0) {
    printf("All control_input_scaling host tests passed.\n");
    return 0;
  }
  printf("%d control_input_scaling host test failure(s).\n", failures);
  return 1;
}
