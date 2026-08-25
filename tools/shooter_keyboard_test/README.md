# Shooter keyboard bench test

This page is separate from the robot Dashboard. It connects directly to the
Mega through Chrome Web Serial and only works with the
`mega_shooter_keyboard_test` firmware environment.

## Controls

- `E`: enable outputs.
- Arrow up/down: manual angle forward/reverse.
- Arrow left/right: manual rotate reverse/forward.
- `1`: zero the angle encoder at the current known mechanical reference.
- `2` / `3` / `4`: disabled until the elevation limits are measured and the
  angle calibration flag is enabled.
- `P`: select horizontal profiled-position mode (default). It uses
  potentiometer feedback without PID gains.
- `C`: select horizontal PID mode. The rotate PIDF constants currently default
  to zero, so this mode will not move until those constants are tuned.
- `5` through `9`: disabled until the horizontal potentiometer points are
  measured and the rotate calibration flag is enabled.
- Space: neutral and disable all outputs.
- `Q`: neutral, disable, and disconnect.

## Run

1. Build and upload the `mega_shooter_keyboard_test` environment from
   PlatformIO.
2. Serve this directory from localhost.
3. Open it in Chrome, click `Connect Arduino`, and choose the Mega serial port.
4. Keep actuator power off for the first connection and verify all PWM values
   report `1500 us` while disabled.

The firmware disables all outputs if commands stop for more than `250 ms`.
USB may power the Mega logic for this test. Do not power the servos or Talon FX
from the Mac or Mega 5 V pin; use the rated external supply and a shared ground.

After calibration, horizontal position tests are capped at `10%` command in
`ShooterBenchConstants.h`. The normal mechanism constant remains `100%`, with
the final `20%` of each move linearly reduced toward the configured minimum
approach ratio. At the `10%` test cap, the minimum envelope is `1%`. Manual arrow
control remains true open-loop and does not have a target-based 80/20 profile.

## Horizontal potentiometer

- Signal: Mega A3.
- Power: use the sensor's rated supply and share ground with the Mega.
- Left, center, and right calibration values are currently all `0` and position
  control is explicitly disabled.
- Measure and replace all three raw/degree values, then deliberately enable the
  calibration flag before position testing.
- Position modes stop if the analog reading is outside the broad electrical
  validity range. Hard switches remain the final direction-specific protection.

## Hard limit inputs

- Angle up: Mega D52.
- Angle down: Mega D53.
- Rotate left: Mega D43.
- Rotate right: Mega D42.
- The legacy electrical assumption is an externally driven active-high signal.
  Do not leave these inputs floating; verify the sensor voltage and polarity
  before enabling actuator power.

Each input is debounced for `8 ms`. A triggered switch blocks only motion farther
into that limit. Motion in the opposite direction remains available so the
mechanism can leave the switch. If both switches on one axis are triggered,
commands in either direction are blocked.
