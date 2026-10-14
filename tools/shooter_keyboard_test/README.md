# A5 vision-follow bench

`vision_rotate.html` + `mega_vision_rotate_test` is the standalone turret
bench: the A5 continuous-rotation servo driven by Orin YOLO vision data over
Serial1 (tx -> PIDF -> servo), not by the keyboard. The shooter head has no
sensors and no limit switches, so nothing stops the servo except the
operator: keep a hand on Enter (stop) or a physical power cut.

- Firmware: `src/Bench/VisionRotateTest.cpp`, state machine
  `src/Control/VisionRotateBenchState.h`, gains
  `src/Constants/VisionRotateBenchConstants.h` (kP, kI, IZone, output and I
  caps). The boot line `$VROTATE_GAINS,...` shows what was actually flashed.
- Telemetry: `$VROTATE,6,...` every 100 ms (field list in the firmware).
  The page only accepts v6; flash the firmware and open the page from the
  same commit.
- Offline analysis of a captured log (v4/v5/v6):
  `python3 analyze_vision_rotate_log.py <log> [--settle-threshold-px N]`.
- The production turret uses the same control law with the gains converted
  to its normalized command (`TuningConstants::Rotate::VISION_*`,
  `src/Control/VisionFollowController.h`); `test/vision_follow_controller_test.cpp`
  keeps the two in step.

Build and upload: `pio run -e mega_vision_rotate_test -t upload`. Serve this
directory (`python3 -m http.server 8000 --bind 127.0.0.1`) and open
`http://localhost:8000/vision_rotate.html` in Chrome or Edge.

The older multi-axis keyboard bench (`index.html` /
`mega_shooter_keyboard_test`) was removed 2026-10-14 together with the
encoder, potentiometer and limit-switch code it existed to test;
`mega_manual_axes_test` with the main Dashboard replaces it.
