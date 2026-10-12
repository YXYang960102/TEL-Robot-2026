#include "control_input_scaling.h"

uint16_t adc_axis_to_pulse_us(uint16_t adcValue) {
  if (adcValue > 4095u) adcValue = 4095u;
  /* 1000 + adcValue * 1000 / 4095, kept as a single integer expression so
     the division only happens once and stays exact at both ends
     (0 -> 1000, 4095 -> 2000). */
  return (uint16_t)(1000u + ((uint32_t)adcValue * 1000u) / 4095u);
}

uint16_t digital_switch_to_pulse_us(bool active) {
  return active ? 2000u : 1000u;
}

uint16_t mode_switch_to_pulse_us(bool bit0, bool bit1) {
  if (!bit0 && !bit1) return 1500u; /* full-auto */
  if (bit0 && !bit1) return 1700u;  /* semi-auto */
  return 1900u;                     /* full-manual: any other combination */
}
