# Shooter keyboard bench test

## A5 horizontal-only test (2026-09-19)

Enable-handshake fix: the page now waits for firmware-enabled telemetry with
the requested limit mode before accepting arrows. Old disabled telemetry during
this wait does not cancel the enable write. While waiting it sends only neutral
manual state; confirmation must arrive within 1000 ms. Telemetry loss still
stops at 350 ms, and firmware retains its 250 ms watchdog and 400 ms jog cap.
Stop, blur, or mode changes cancel pending enable. Press arrows anew after the
page says firmware confirmed enabled. Reload the page after updating.
New firmware appends a stop-reason field to v2 telemetry (BOOT/NONE/OPERATOR/
TIMEOUT/JOG_CAP/LIMIT/BAD_COMMAND). The page still accepts earlier v2 firmware,
but reports an unknown firmware stop reason for that version; rebuild/upload
mega_rotate_only_test manually to get precise firmware reasons.

For Jeremy's continuous-rotation 360-degree servo and camera turntable, use
`mega_rotate_only_test` and `rotate.html`, NOT the full Shooter environment
below. The isolated build attaches only A5 and does not initialize AS5600,
A3, elevation, flywheel, Vision, or chassis. Production Rotate remains D29.

Build: `pio run -e mega_rotate_only_test` (no upload).
After review, upload that environment manually through PlatformIO.
Serve this directory using `python3 -m http.server 8000 --bind 127.0.0.1`
and open `http://localhost:8000/rotate.html` in Chrome.

Jeremy confirmed only the camera and servo are installed, without limit switches.
For that setup, check the explicit no-limits acknowledgement, then click Enable.
The firmware reports `noLimits=1` and the page warns that D43/D42 are ignored.
No pin jumper is needed. Stop/timeout clears this authorization; check again
before re-enabling. Without the checkbox, normal limit protection remains active.
Enable using the visible button only; hold Left/Right to jog, release to stop.
Enter stops/disables but keeps serial; Space stops/disconnects. Focus loss,
hidden page, serial failure or stale telemetry disables the browser commands.
Firmware independently disables after >250 ms without a valid manual command
or after 400 ms of continuous same-direction jogging. Re-enable explicitly.
Unknown/malformed/oversized lines fail disabled; oversized lines are discarded
through newline. There is no generic heartbeat that can sustain an old jog.

Provisional PWM: neutral 1500 us, jog +/-50 us (1450/1550). This is 10% of
the historical 500 us half-span, NOT measured speed or angle. Photo labels
35Kg HV / 360 degrees / XT do not establish the neutral calibration or voltage.
Tune only `RotateBenchConstants.h` after a signal-only and unloaded stop test.

Preserved active-high directional limit inputs: D43 left, D42 right. This
isolated test enables pull-ups so disconnected limits block rather than float.
A limit in protected mode immediately stops AND disables; no debounce delay. Verify externally
driven low=clear/high=blocked before power when using protected mode. For the
explicit no-limits test, use the checkbox rather than bridging inputs.
No position encoder means no absolute-angle or
software travel guarantee. Keep a physical power cut available and prevent
camera USB cable entanglement. Software neutral is not an electrical E-stop.

Protocol v2: `E` enables protected mode; `E_NO_LIMITS` explicitly enables the
approved no-limits bench mode. `X` stops and clears bypass. Telemetry is
`$ROTATE,2,enabled,pulse_us,left_raw_high,right_raw_high,noLimits`.
The new page rejects v1 telemetry, so update both page and firmware together.

Hardware acceptance: first motor power off, verify only A5 pulses, mode reporting,
boot disabled, key-release, Enter, Space, browser loss and USB loss. Then
unloaded verify neutral really stops; short jog direction; 400 ms cutoff;
limits and reverse escape after re-enable. No firmware was uploaded by Codex.

## Existing multi-axis test (unchanged)

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
