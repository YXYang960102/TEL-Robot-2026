# Ground-station ELRS/CRSF bridge

Desk/build verification only (`python3 build.py`). Never connects to or
flashes hardware. `dist/verification.json` keeps `flashed`/`hardware_tested`
hardcoded `false`, matching `tools/stm32_elrs_bidirectional_test`'s
convention -- this firmware has not been loaded onto a board.

## Role

The other half of the two-board ELRS deployment (the robot-side half is
`tools/stm32_elrs_robot_bridge`). This board sits with the operator,
between real control inputs and the ES900TX radio module, and feeds the
laptop running `tools/dashboard/`:

```
control inputs (ADC/GPIO)  -->  this board  <--USART1 PA9-->  ES900TX (CRSF, RF)
                                     |
                                USART2 PA2/PA3 (polled)
                                     v
                          USB-UART adapter --> dashboard laptop
```

- **Downlink**: real control inputs (see "Control inputs" below) are
  scaled to the project's existing pulse-microsecond convention
  (`src/control_input_scaling.c`) and sent as `CRSF_FRAME_RC_CHANNELS`
  frames on USART1, at the same 100 Hz/10 ms cadence as
  `tools/stm32_elrs_bidirectional_test`.
- **Uplink**: `CRSF_FRAME_TEL_CHUNK` frames arriving back on USART1 are fed
  to `tools/stm32_elrs_common/src/tel_reassembler.c`; once a line is fully
  reassembled it is written out USART2 (polled, not IRQ-driven -- there is
  no other USART2 traffic to interleave with) to the USB-UART adapter,
  with a trailing `\n` appended. The line itself is the Mega's original
  `TEL,...` string, unchanged, so `tools/dashboard/` needs no code changes
  -- it just needs to read this adapter's serial port at 115200 baud
  instead of (or in addition to) the Mega's direct USB port.

USART1/PA9's half-duplex wiring to ES900TX, its baud, and its bring-up
code are unchanged from `tools/stm32_elrs_bidirectional_test`, which is
hardware-confirmed working over real ELRS RF (see
`docs/codex-handoff.md`). USART2 is new here: the bench rig used PA2/PA3
for ES900RX, which this board does not have.

## Control inputs (provisional, see `src/config.h`)

The real multi-function keypad has not been built yet. This firmware reads
placeholder pins so it has something concrete to build and test against,
**not a claim about final wiring**:

- `PA0`/`PA1` (ADC1_IN0/IN1): drive-forward / drive-turn axes, scaled
  linearly 0-4095 -> 1000-2000us.
- `PA4`/`PA5`: 2-bit operator-mode switch (00=full-auto, 01=semi-auto,
  anything else=full-manual -- deliberately fails toward the safer manual
  state on an unexpected bit pattern).
- `PA6`: momentary fire-confirm button.
- `PA7`: starting-side LEFT/RIGHT switch.

All digital inputs are active-low (pull-up idle). Channel order and the
scaling thresholds that must line up with the Mega-side
`OperatorModeConstants`/`TuningConstants::StartingSide` thresholds are in
`tools/stm32_elrs_common/src/tel_channel_map.h` and
`src/control_input_scaling.h`. Re-measure and re-wire all of this once the
real keypad exists; nothing else needs to change if the channel meanings
stay the same.

## Verification done so far

- `test/host_test_control_input_scaling.c`: native host test for the ADC/
  digital/mode-switch scaling functions, including that the mode-switch
  outputs land on the correct side of the Mega's existing
  `OperatorModeConstants` thresholds.
- `tools/stm32_elrs_common`'s own host tests (CRSF framing, reassembly)
  cover the rest of what this firmware calls.
- `build.py` cross-compiles, checks the vector table (SysTick/USART1
  wired), checks image bounds, and checks for undefined symbols.

## Not done

No bench power-up, no ES900TX pairing, no real control-input wiring (the
pins above are placeholders), no ADC calibration. The register-level
bring-up for USART1/PA9 is adapted line-for-line from the hardware-
confirmed bench rig; USART2, the ADC1 single-conversion polling code, and
all of this firmware's own logic are unverified beyond the checks above.
