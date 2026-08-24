# Shooter keyboard bench test

This page is separate from the robot Dashboard. It connects directly to the
Mega through Chrome Web Serial and only works with the
`mega_shooter_keyboard_test` firmware environment.

## Controls

- `E`: enable outputs.
- Arrow up/down: manual angle forward/reverse.
- Arrow left/right: manual rotate reverse/forward.
- `1`: zero the angle encoder at the current known mechanical reference.
- `2`: command the provisional angle setpoint 1.
- `3`: command the provisional maximum angle.
- `5` through `9`: intentionally rejected until rotate feedback exists.
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
