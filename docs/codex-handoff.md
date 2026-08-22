# Codex <-> Claude Handoff Log

Shared, append-only log for TEL_Robot_2026. Read the complete file before
project work and append new entries chronologically.

---

## 2026-08-20 - Codex: fixed-speed feed and beam-break counting

**User Request:** Jeremy assigned feeding and storage-tower behavior to the
`Dribbler` branch and asked for fixed-speed ball feeding with beam-break
detection like the legacy firmware.

**Discussion Result:** A requested shot runs the existing feed outputs at one
configured speed. A ball is counted only after the exit beam is stably blocked
and then stably clears. YOLO does not decrement or invent the physical count;
the Arduino event is the authoritative confirmation that one ball passed.

**Why:** The previous modular code read output pin 51 as if it were a sensor and
decremented on every loop while LOW. Separating output pin 51 from legacy exit
sensor pin 45 and using a complete blocked-to-clear event prevents repeated
counts from one ball or a held sensor.

**Changed:** `Dribbler` now stores `shootRemaining`, reports feeding/sensor/event
state, drives pin 2 with a fixed PWM while a request remains, controls the gate
on pin 51, debounces the exit sensor on pin 45, and stops automatically when the
requested count reaches zero.

**Added:** `DribblerConstants.h` contains feed PWM 180, pin polarity, 8 ms
debounce, and request bounds `0..12`. `PIN_DRIBBLE_EXIT_SENSOR=45` restores the
legacy top beam-break input without reusing an actuator output.

**Calculation:** One confirmed event is `stableBlocked: true -> false` after at
least 8 ms at each raw transition. Example: request 3, beam blocks, then clears:
remaining changes `3 -> 2` and completed count `0 -> 1`. A beam held LOW for
100 updates remains one incomplete event and does not decrement repeatedly.

**Impact:** Storage-tower multi-motor sequencing, ultrasonic behavior, YOLO
return transport, and automatic shot requests remain future work. PWM 180 and
sensor polarity are architecture defaults, not measured hardware calibration.

**Evidence:** `platformio run -e megaatmega2560` succeeded: RAM 1038/8192 bytes
(12.7%), flash 13050/253952 bytes (5.1%). No firmware upload, sensor fixture,
motor power, or real-ball test was performed.

**Next Test:** With motor power disabled, monitor pin 45 while manually blocking
and clearing the sensor and confirm exactly one count per passage. Then power
the feed mechanism with an immediate stop path and tune `FEED_PWM` from a low
safe value while confirming pin 51 polarity before moving a real ball.

---

## 2026-08-22 — Claude Code

**User Request:** Jeremy approved Codex's Orin/Mega power-gated startup
proposal (shared inbox) and said implement it; Claude owns the Mega side.
While grounding that work, found and fixed (with Jeremy's authorization) two
issues Codex's later mutual review also independently confirmed.

**Discussion Result / Changed:**
1. `7895f50`: this branch's `Chassis`/`SBUS`/`Vision` were the pre-fix
   versions (unclamped 4-wheel PWM math, no SBUS failsafe) and `main.cpp`
   unconditionally called `Dribbler::setShootRequest(3)` on every boot before
   any RC link existed. Ported `Chassis` branch's already-fixed versions
   verbatim; changed boot to `setShootRequest(0)` and gated chassis commands
   on `SBUS::isHealthy()`.
2. `9cfa016`: ported `Chassis`'s completed Mega<->Orin handshake
   (`MEGA_READY,1` once then `MEGA_HEARTBEAT,1` every 100ms;
   `VISION_STANDBY/STARTING/READY/ERROR,<version>` parsing with strict
   version-field validation, not just prefix match) and a Shooter safety
   fix: `Shooter::update()`'s vertical PID ran unconditionally every loop
   with `setV` hardcoded to `2000` (unreachable by the 0-1023 ADC `pot`
   input), `Kp2=0.45`, and no `SetOutputLimits()` anywhere in the repo -
   proportional term alone (`0.45 * (2000-1023) = 439.65`) exceeds PID_v1's
   default 255 output ceiling, so `escV` saturated to `1755us` from the
   first `loop()` iteration regardless of SBUS/vision state. This branch's
   own `Shooter.cpp` was actually an older variant that didn't even gate
   horizontal aim on `Vision::isValid()` - both axes are now held at
   neutral (1500us) and `falcon` stopped until `Vision::isVisionReady()`
   is true.

**Why:** Codex's mutual-review pass (`進行審核`, 2026-08-22) read the actual
branch objects independently and caught: (a) this branch had no handshake
code at all - if flashed standalone with Orin's `--wait-for-mega`, Orin would
sit in `WAIT_MEGA` forever; (b) the Shooter saturation hazard, with exact
math; (c) a bug in Claude's own `parseOrinControl()` that checked only the
lifecycle prefix and never validated the version field (fixed on `Chassis` at
`fcc6ca5` before this port, so it's included here too).

**Calculation:** See above - `2000 - 1023 = 977` worst-case error,
`0.45 * 977 = 439.65 > 255` (PID_v1 default output ceiling), so `outV`
clamps to `255`, `escV.writeMicroseconds(1500 + 255) = 1755`.

**Impact:** `Shooter`, `Vision`, `Chassis`/`SBUS` (from `7895f50`) all
changed. `Dribbler.cpp`'s own ball-feed/counting logic (Codex's `a4f1bbb`
work, entry above) was not touched by either port.

**Evidence:** `avr-g++ -fsyntax-only` against every `.cpp` under `src/`
(excluding `Bench/Sensors/Actuators`, matching `build_src_filter`), using the
exact flags/includes from this machine's cached
`.pio/build/megaatmega2560/idedata.json` - all compile clean on both passes.
No `pio` link, no firmware upload, no hardware test.

**Next Test:** Same as `Chassis` branch's equivalent entries: bench-verify
with actuators unpowered/wheels off ground, simulate the Orin heartbeat/
lifecycle lines over a USB-serial adapter into `Serial1` before trusting this
on real hardware. Additionally for this fix specifically: confirm on the
bench that `escH`/`escV`/`falcon` all read exactly `1500us` (measure with a
servo tester or oscilloscope, not just code review) while Orin is not yet
sending `VISION_READY`, before ever connecting the real shooter mechanism.
