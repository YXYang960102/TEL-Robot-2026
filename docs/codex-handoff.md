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

---

## 2026-08-22 — Claude Code

**User Request:** Jeremy approved Codex's Orin/Mega power-gated startup
proposal (shared inbox) and said implement it; Claude owns the Mega side.
Then, after Codex's read-only mutual review (`進行審核`), Jeremy authorized
fixing every finding.

**Discussion Result / Changed, in commit order:**
1. `4920882`: added the Mega->Orin heartbeat and Orin lifecycle-line parsing
   to `Vision`, against a self-proposed wire format that turned out to be
   wrong.
2. `b440f24`: corrected that format to match Codex's already-implemented and
   unit-tested Orin side exactly - `MEGA_READY,1` once then
   `MEGA_HEARTBEAT,1` every 100ms; `VISION_STANDBY/STARTING/READY/ERROR,1`
   parsed via a `VISION_` prefix check; added `Vision::isVisionReady()`.
3. `fcc6ca5`: Codex's review found `parseOrinControl()` checked only the
   prefix and never validated the version field, so `VISION_READY,999` or
   even `VISION_READY,` (empty) would both be accepted as a fresh READY.
   Fixed to parse and require the version field equal `PROTOCOL_VERSION`,
   still tolerating extra trailing fields after a valid version (matching
   Orin's own tolerance).
4. `b8c39d9`: Codex's review found `Shooter::update()`'s vertical PID ran
   unconditionally every loop regardless of any readiness state - `setV` is
   hardcoded to `2000`, unreachable by the `analogRead(PIN_POT)` 0-1023
   range, `Kp2=0.45` (`src/Constants/PIDConfig.h`), and no
   `SetOutputLimits()` call exists anywhere in this repo (grepped to
   confirm), so PID_v1's default 0-255 output ceiling saturates
   immediately: `0.45 * (2000-1023) = 439.65 > 255`, so `escV` gets
   `1500 + 255 = 1755us` from the very first `loop()` iteration after boot,
   independent of SBUS health or vision validity. Gated the whole function
   on `Vision::isVisionReady()`: neutral (1500us) on `escH`/`escV`/`falcon`
   and `readyH`/`readyV=false` until a fresh `VISION_READY` is seen.

**Why:** The heartbeat/lifecycle work is the Claude-owned half of the
approved Orin power-gate proposal (Codex: Orin/YOLO_Detect_single side;
Claude: TEL/Mega side). The two fixes in items 3-4 came directly out of
Codex's independent read-only review of the actual branch objects, not
self-review - re-verified both findings against source (`PIDConfig.h`,
`grep -rn SetOutputLimits`) before fixing rather than taking the review at
face value.

**Calculation:** See item 4 above for the exact saturation math. Heartbeat
timing: Orin's incoming-Mega timeout is 1000ms and its own status/heartbeat
cadence is 100ms; Mega's `HEARTBEAT_INTERVAL_MS=100` matches that cadence,
giving roughly 10 heartbeat slots inside Orin's timeout window as margin.
Vision-state staleness reuses the existing `PACKET_TIMEOUT_MS=300` (already
used for numeric-target staleness) for lifecycle-line staleness too.

**Impact:** `Vision` and `Shooter` changed. Numeric target parsing (5-field,
range-checked, `PACKET_TIMEOUT_MS`), `Chassis`/`SBUS` failsafe behavior, and
`Dribbler` are unaffected by this entry's changes (Dribbler's own port of
this same work is a separate commit sequence on the `Dribbler` branch, see
that branch's own handoff entry).

**Evidence:** `avr-g++ -fsyntax-only` against every `.cpp` under `src/`
(excluding `Bench/Sensors/Actuators`), using the exact flags/include paths
from this machine's cached `.pio/build/megaatmega2560/idedata.json` - all
compile clean after every commit in this sequence, re-run after each fix, not
just once at the end. No `pio` link, no firmware upload, no hardware test of
any kind - the Shooter fix in particular has never been bench-verified with
a multimeter or servo tester, only reasoned about from source.

**Next Test:** Before connecting the real shooter mechanism, bench-verify
with the ESC/servo signal wires disconnected from any load: power the Mega
alone and confirm (with a servo tester or oscilloscope on `escH`/`escV`/
`falcon`) that all three sit at exactly 1500us while no `VISION_READY` line
is being sent, then feed a simulated `VISION_READY,1` over `Serial1` (e.g.
from a USB-serial adapter) and confirm the vertical PID only then starts
moving `escV` away from neutral. Only after that holds, proceed to the
existing Chassis/SBUS bench test sequence above and eventually a real
Orin<->Mega link test.

---

## 2026-08-24 - Codex: Chassis, Vision, and IO readability refactor

**User Request:** After synchronizing formal branches, refactor one owning
branch at a time using mechanism-owned Constants and visible subsystem APIs.
This entry covers only the formal `Chassis` branch and preserves the existing
two powered front wheels plus passive rear caster architecture.

**Constants:** `ChassisConstants.h` now groups right/left signal pins and
inversion, output PWM, and manual command bounds/deadband. The old
`SBUSConstants.h` became `IOConstants.h`, which owns receiver channel indices,
raw ranges, mapped pulse ranges, auxiliary range, and frame timeout.
`VisionConstants.h` now groups `Transport`, `Validation`, and visibly isolated
`Legacy` prediction values. Duplicate Chassis pin macros were removed from
`Pins.h` after migration.

**Chassis API:** `Chassis.h` visibly separates lifecycle, manual open-loop, and
telemetry. `setDriveCommand()` was renamed `setOpenLoop(forward, turn)` and
`main.cpp` was updated. Tank mixing remains `right=forward-turn` and
`left=forward+turn`, followed by common normalization. New getters expose
bounded commands and actual PWM values without exposing Servo objects.

**IO API:** SBUS channel storage is private. Callers now use named command and
telemetry getters instead of public `ch0/ch1/ch2/ch3/ch8` globals. The current
firmware mapping remains exact: receiver channel 0 drives forward, channel 1
is inverted for turn, channel 2 is the mechanism input, channel 3 is the
auxiliary input, and channel 8 is mode. The `TEL` telemetry packet retains the
same five values in the same order, so the Dashboard wire schema is unchanged.
Failsafe, lost-frame, and 100 ms stale-frame neutralization are unchanged.

**Vision API:** Public methods are grouped as lifecycle, YOLO target facts,
and transport/Orin status. `getXPred()` remains as a documented compatibility
alias for `tx`. The exact five-field packet, numeric/range validation,
`MEGA_READY,1`, 100 ms `MEGA_HEARTBEAT,1`, strict lifecycle version checking,
300 ms staleness, and invalid-target clearing are unchanged.

**Evidence:** `git diff --check` passes. PlatformIO `megaatmega2560` release
build succeeds: RAM `1205/8192` bytes (14.7%), Flash `17960/253952` bytes
(7.1%). No firmware upload, UART loopback, receiver test, PWM measurement, or
powered chassis test was performed.

**Next:** Commit and push this refactor separately on `Chassis`, request Claude
review, then switch the normal working tree to `Dribbler`. Do not merge into
`dev` before subsystem verification.
