# Codex <-> Claude Handoff Log

Shared, append-only log for TEL_Robot_2026. Read the complete file before
project work and append new entries chronologically.

---

## 2026-08-20 - Codex: Chassis and YOLO transport foundation

**User Request:** Jeremy assigned chassis and Vision work to the `Chassis`
branch and asked Codex to combine the current two-wheel tank control with the
actual `YOLO_Detect_single` serial transport before moving on to Dribbler and
Shooter.

**Discussion Result:** Keep Jetson as a fact producer and Mega as the motor
controller. Vision accepts only the exact five-field packet
`tx,ty,distance_mm,target_id,valid`; chassis manual input remains SBUS, but now
uses a command API that a future autonomous controller can also call. Invalid,
malformed, stale Vision data is neutralized. SBUS failsafe, lost-frame, or
100 ms silence commands a chassis stop.

**Why:** YOLO must not directly own motors, stale input must not leave the last
motor command active, and future autonomous control needs one bounded chassis
command interface instead of reaching into Servo PWM values.

**Changed:** `Vision` now validates exactly five numeric fields, range checks
them, tracks packet freshness separately from target validity, and zeroes target
facts after invalid/malformed/stale input. `SBUS` now exposes health, frame age,
and normalized forward/turn commands and neutralizes channels on failure.
`Chassis` now consumes bounded `[-1,1]` commands and provides `stop()`.
`main.cpp` routes healthy SBUS commands into Chassis and stops otherwise.

**Added:** `ChassisConstants.h` stores ESC pulses and command deadband;
`SBUSConstants.h` stores receiver ranges and the 100 ms timeout.

**Calculation:** `forward=(ch0-1500)/500`, `turn=(ch3-1500)/500`.
Tank mixing is `right=forward-turn`, `left=forward+turn`; if either magnitude
exceeds 1, both are divided by the largest magnitude. Output is
`1500 + command*500 us`, clamped to `1000..2000 us`. Example: forward `0.6`,
turn `0.2` gives right `0.4 -> 1700 us` and left `0.8 -> 1900 us`.

**Impact:** Dashboard telemetry schema is unchanged. Shooter, Dribbler,
ShooterMG996, YOLO Python code, and autonomous movement behavior are unchanged.

**Evidence:** `platformio run -e megaatmega2560` succeeded: RAM 1076/8192 bytes
(13.1%), flash 16788/253952 bytes (6.6%). No firmware upload, serial loopback,
receiver bench test, or energized chassis test was performed.

**Next Test:** With wheels raised and an immediate power-disable path, verify
neutral SBUS produces 1500 us on both drive outputs, forward commands equal
wheel direction, turn commands opposite wheel direction, and disconnecting the
receiver stops both outputs within 100 ms. Separately send valid, invalid,
malformed, and paused YOLO packets with actuators disabled and inspect telemetry.
