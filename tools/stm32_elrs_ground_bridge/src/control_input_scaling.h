#ifndef CONTROL_INPUT_SCALING_H
#define CONTROL_INPUT_SCALING_H

/* Converts this board's raw control inputs (GPIO switches, ADC axes) into
   the project's existing pulse-microsecond channel convention (the same
   1000-2000us range SBUS used to produce), so the Mega-side thresholds
   (OperatorModeConstants, TuningConstants::StartingSide, etc.) keep
   working unchanged. Pure arithmetic, no registers -- host-testable. */

#include <stdint.h>
#include <stdbool.h>

/* adcValue is a 12-bit ADC1 reading (0..4095, clamped if out of range).
   Maps linearly to 1000..2000us. */
uint16_t adc_axis_to_pulse_us(uint16_t adcValue);

/* A momentary button/switch: active (pressed, or switched to the "high"
   position) maps to 2000us, inactive to 1000us. Used for the fire button
   and the starting-side LEFT/RIGHT switch (RIGHT = active, matching
   TuningConstants::StartingSide::THRESHOLD_US = 1500 on the Chassis side:
   active(2000) >= 1500 -> RIGHT, inactive(1000) < 1500 -> LEFT). */
uint16_t digital_switch_to_pulse_us(bool active);

/* The 3-way operator mode switch, read as 2 digital bits: 00 -> full-auto,
   01 -> semi-auto, any other combination -> full-manual (failing toward
   the safer manual state on an unexpected bit pattern, e.g. a switch mid-
   travel). Matches OperatorModeConstants::SEMI_AUTO_THRESHOLD=1667 /
   FULL_MANUAL_THRESHOLD=1833 on the Shooter side (1500-2000 split into
   thirds): 1500 < 1667 -> full-auto, 1700 is between the two thresholds
   -> semi-auto, 1900 >= 1833 -> full-manual. */
uint16_t mode_switch_to_pulse_us(bool bit0, bool bit1);

#endif
