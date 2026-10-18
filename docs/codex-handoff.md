# Codex <-> Claude Handoff Log

Shared, append-only log for TEL_Robot_2026. Read the complete file before
project work and append new entries chronologically.

## 2026-08-20 - Codex - Shooter architecture baseline

### Scope and ownership

- Branch: `Shooter` only. `ShooterMG996` was intentionally left unchanged as
  the existing UNO/continuous-servo bench-test branch.
- Jeremy's latest direction supersedes the earlier two-encoder assumption:
  the first official elevation architecture uses one AS5600 as shared feedback
  for two mechanically linked elevation servos.
- Added a third continuous servo output for horizontal turret motion and an
  open-loop flywheel output placeholder for the Falcon 500 path.

### Control sequence

1. All four outputs attach at boot but remain at their neutral/stop pulse.
2. `Shooter::setOutputsEnabled(true)` is required before any command moves.
3. Elevation starts with `setElevationManual()` open-loop testing.
4. `zeroElevationAtCurrentPosition()` must succeed with a valid AS5600 sample
   before `setElevationTargetCounts()` can enable closed-loop PID.
5. Vision does not directly command Shooter in this baseline. Automatic aiming
   is deferred until each actuator and feedback path passes bench testing.

### Encoder and control math

- AS5600 raw range is 0..4095. Signed wrap delta is corrected by +/-4096 when
  it crosses the 2048-count half-turn boundary, then accumulated into relative
  multi-turn counts.
- Counts convert to degrees with `counts * 360 / 4096`; 1024 counts is 90 deg.
- Samples jumping more than 180 counts per update are rejected as provisional
  noise protection. This threshold must be checked against real mechanism speed.
- One PID error (`targetCounts - relativeCounts`) produces a pulse offset clamped
  to +/-80 us. Left receives `1500 + offset`; right receives `1500 - offset`.
- Elevation target range is provisionally 0..1024 counts and ready tolerance is
  +/-4 counts. These are not hardware-validated limits.
- Turret open-loop output is `1500 + command * 120 us` for command -1..1.
- Flywheel open-loop output is `1500 + command * 500 us` for command 0..1.

### Safety and unresolved hardware facts

- Servo stop pulses, output signs, PID gains, target limits and pin 26 for the
  right elevation servo are provisional and must be confirmed on the bench.
- `Shooter::isReady()` intentionally remains false. Horizontal absolute angle
  and flywheel speed feedback do not exist yet, so overall shooter readiness
  cannot be claimed safely.
- The existing pin-49 Servo-style flywheel signal is only an interface
  placeholder. Confirm the actual Falcon 500 motor-controller connection before
  enabling it; no RPM control is implemented.
- No limit switch or physical hard-stop calibration is represented yet.

### Verification

- PlatformIO `megaatmega2560` build: success.
- RAM: 1135 / 8192 bytes (13.9%).
- Flash: 13440 / 253952 bytes (5.3%).
- Hardware upload and movement test: not performed.

---

## 2026-08-23 - Codex: reusable Shooter PIDF and named loop constants

**User Request:** Jeremy approved adding `kIZone` and `kFF` to the usual PID
gains for mechanisms using encoder feedback. Every gain must be a `double`,
default to `0.0`, live in Constants, and identify its owning Shooter loop:
horizontal rotation (`kRotate...`), elevation angle (`kAngle...`), or Falcon
500 flywheel speed (`kFlywheel...`). Codex owns implementation; Claude owns
independent review and process verification.

**Discussion Result:** The formal `Shooter` branch now uses a project-owned,
hardware-independent `PidfController` instead of `PID_v1` for the existing
one-AS5600/two-elevation-servo closed loop. Rotate and Flywheel receive named
PIDF configurations but remain open-loop because neither currently has the
required angle/RPM feedback. `ShooterMG996`, Chassis, Dribbler, dev, and YOLO
were intentionally left unchanged.

**Why:** IZone prevents a large far-away error from accumulating integral
state, but IZone alone cannot prevent windup while the final output is
saturated. The controller therefore also limits integral contribution and
conditionally rejects integration that would push a saturated output farther
into saturation. A reusable controller keeps the same behavior available to
future encoder mechanisms without duplicating hidden state inside Shooter.

**Changed:** `ShooterConstants.h` now contains the fifteen explicitly named
Rotate/Angle/Flywheel gains and three `PidfConfig` objects. All gains are
currently `0.0`, so closed-loop output is deliberately neutral until bench
tuning. Shooter elevation now computes normalized PIDF output, resets state on
disable/manual/homing/invalid sensor/large target change, requires position
and velocity stability for readiness, and exposes the P/I/D/FF terms,
normalized command, velocity, and saturation state for telemetry. The unused
PlatformIO `PID_v1` dependency was removed.

**Added:** `src/Control/PidfController.h/.cpp` and
`test/pidf_controller_test.cpp`. The controller accepts physical-unit target
and measurement values plus measured seconds per update. Derivative is taken
on measurement to avoid setpoint derivative kick. Feedforward uses an explicit
caller-provided reference; elevation currently passes `1.0`, making `kAnglekFF`
a constant normalized holding bias until a gravity/angle model is mechanically
defined.

**Flow:** AS5600 raw angle -> validated multi-turn relative counts -> elevation
target/error in counts -> PIDF normalized output `[-1,1]` -> provisional
`+/-80 us` offset -> left `1500 + offset` and right `1500 - offset`. The two
elevation servos share one PIDF loop because they share one mechanical axis and
one AS5600; the second output is mirrored rather than independently controlled.

**Calculation:** `error = setpoint - measurement`. Integral is cleared unless
`kI != 0`, `kIZone > 0`, and `abs(error) <= kIZone`. While enabled,
`Iacc += error * dt`, then `I = clamp(kI * Iacc, -0.25, 0.25)` and integration
is rejected when it would worsen output saturation. `P = kP * error`,
`D = -kD * (measurement - previousMeasurement) / dt`, and
`FF = kFF * feedforwardReference`. Final normalized output is
`clamp(P + I + D + FF, -1, 1)`. Elevation pulse offset is
`round(output * 80 us)`.

Example with temporary nonzero values only for illustrating the math:
setpoint `1024 counts`, previous measurement `998`, current measurement `1000`,
`dt=0.02 s`, `kP=0.02`, `kI=0.01`, `kD=0.001`, `kIZone=10`, `kFF=0.05`, and
feedforward reference `1.0`. Error is `24`, outside IZone, so `I=0`.
Measurement rate is `(1000-998)/0.02=100 counts/s`; therefore `P=0.48`,
`D=-0.10`, `FF=0.05`, output `0.43`, and offset `round(0.43*80)=34 us`.
Left receives `1534 us`; mirrored right receives `1466 us`. With the actual
current all-zero gains, the same input produces output `0` and both receive
`1500 us`.

Readiness requires `abs(error) <= 1 count`, measurement speed no greater than
`50 counts/s`, and both conditions continuously true for `100 ms`. The control
period is `20 ms`; a gap over `100 ms` resets PIDF and commands neutral.

**Impact:** Existing manual open-loop elevation, turret, and flywheel APIs are
retained. Overall `Shooter::isReady()` intentionally remains false because
horizontal angle and flywheel speed feedback still do not exist. Current
`+/-80 us`, signs, readiness thresholds, and feedforward reference remain
provisional hardware values. The Chassis branch's Vision-ready Shooter safety
gate is not yet integrated into this formal Shooter branch.

**Evidence:** Native C++ tests pass for all-zero defaults, P/D/FF term math,
IZone entry/exit/reset, zero-IZone integral disable, output saturation, and
anti-windup recovery. `git diff --check` passes. PlatformIO
`megaatmega2560` release build succeeds: RAM `1169/8192` bytes (14.3%), flash
`13202/253952` bytes (5.2%). The only compiler warnings are pre-existing
signed/unsigned and unused-variable warnings inside the vendored AS5600
library. No upload, energized actuator test, encoder motion test, or PID tuning
was performed.

**Next Test:** Follow the agreed sequence: (1) outputs-disabled pulse and stop
verification, (2) constrained manual open-loop direction/deadband test for each
servo, (3) AS5600 raw/sign/wrap/noise validation with outputs neutral, then
(4) closed-loop tuning with `kI=kD=kFF=0`, small `kAnglekP`, constrained
`+/-80 us`, and a physical immediate-disable path. Add IZone/FF only after the
P-only response and mechanism coupling are measured. Do not merge to dev until
Claude review and hardware evidence are resolved.

---

## 2026-08-23 - Codex: Shooter manual motor configuration and soft limits

**User Request:** Add FRC-style motor inversion and soft-limit configuration,
give every Shooter mechanism a manual open-loop function, and place all motor
parameters in Constants. Fixed manual actions must use `2000 us` forward,
`1500 us` stop, and `1000 us` reverse.

**Changed Constants:** `ShooterConstants.h` now owns `ServoMotorConfig` and
`PositionLimitConfig`. Elevation, Turret, and Flywheel each declare explicit
forward/stop/reverse pulse values and an inversion flag. The two elevation
servos share one command; left is currently non-inverted and right is inverted.
Elevation closed-loop output remains separately constrained to `+/-80 us`.
Elevation soft limits are enabled from relative AS5600 count `0` through
`1024`. Compile-time assertions reject reversed PWM or position ranges.

**New API:** `OpenLoopAction` provides `REVERSE`, `STOP`, and `FORWARD`.
`Shooter::setAngleSpeed(action)`, `setRotateSpeed(action)`, and
`setFlywheelSpeed(action)` provide the requested discrete manual functions.
The existing proportional APIs remain available for later joystick input:
`setElevationManual(double)`, `setTurretManual(double)`, and
`setFlywheelOpenLoop(double)`, all using normalized range `[-1, 1]`.

**Output Mapping:** A normalized command is linearly mapped by each motor's
Constants config. Command `+1` maps to the configured forward pulse, `0` maps
to neutral, and `-1` maps to reverse. An inverted motor negates the command
before mapping. Therefore elevation forward currently outputs left `2000 us`
and right `1000 us`; reverse outputs left `1000 us` and right `2000 us`; stop
outputs `1500 us` to both. Rotate and Flywheel currently map to the same
`1000/1500/2000 us` convention without inversion.

**Soft-Limit Behavior:** `areElevationSoftLimitsActive()` is true only after
`zeroElevationAtCurrentPosition()` succeeds and the AS5600 remains valid.
While active, positive motion is stopped at or above `1024` relative counts,
and negative motion is stopped at or below `0`. The guard applies to both
manual and PIDF commands. If a closed-loop command reaches a limit, PIDF state
is reset before neutral is written to prevent integral buildup. Before homing,
manual motion remains available for controlled setup/homing and software soft
limits are inactive; the operator must use low-risk bench conditions and a
physical disable path.

**Hardware Boundary:** These outputs use Arduino's PWM `Servo` interface.
Unlike a CAN smart motor controller, it cannot configure motor-controller
brake/coast mode, smart current limits, or hardware-persisted soft limits.
`1500 us` requests neutral/stop from the connected device but is not evidence
of an electrical brake. Falcon 500 brake and current-limit support must wait
until the actual CAN controller and protocol are identified; no fake API was
added.

**Evidence:** `git diff --check` passes. Standalone native PIDF tests pass.
PlatformIO `megaatmega2560` release build succeeds: RAM `1183/8192` bytes
(14.4%), Flash `13940/253952` bytes (5.5%). No upload, powered motor test,
direction verification, homing test, or soft-limit motion test was performed.

**Next Test:** With mechanisms unloaded and a physical immediate-disable path,
first verify disabled/stop pulses, then test each action at the signal level.
Confirm actual motor direction before changing any `INVERTED` constant. Validate
AS5600 count direction and homing separately, then approach each software limit
slowly. PID tuning remains after open-loop and sensor validation.

---

## 2026-08-23 - Codex: Falcon 500/Talon FX PWM flywheel correction

**Supersedes:** The preceding manual-control entry incorrectly described the
Flywheel as accepting reverse speed and the shared `OpenLoopAction::REVERSE`.
Jeremy clarified that the Falcon 500's integrated Talon FX receives PWM from
the Arduino and the mechanism only needs stop or forward launch demand. Motor
installation direction is a Constants configuration, not an operator command.

**Changed:** Added flywheel-specific `FlywheelAction { STOP, FORWARD }` and
changed `Shooter::setFlywheelSpeed()` to accept only that type.
`setFlywheelOpenLoop(double)` is again constrained to `[0.0, 1.0]`; negative
requests clamp to stop. Elevation and Turret continue using the bidirectional
`OpenLoopAction`.

**Constants Semantics:** Flywheel PWM constants are now named
`MIN_PULSE_US=1000`, `NEUTRAL_US=1500`, and `MAX_PULSE_US=2000`; no value is
named reverse speed. With `Flywheel::INVERTED=false`, logical forward demand
maps from neutral toward maximum PWM. With `INVERTED=true`, the same positive
logical demand maps from neutral toward minimum PWM. Callers never request a
negative Flywheel command.

**Hardware Boundary:** The current path writes PWM on
`PIN_SHOOTER_FLYWHEEL` (Mega pin 49) to the Talon FX. This one-way command path
does not provide Falcon velocity feedback to Arduino, so the current Flywheel
remains open-loop even though PIDF constants are reserved for future feedback.

**Evidence:** `git diff --check` passes. Standalone PIDF tests pass. PlatformIO
`megaatmega2560` release build succeeds: RAM `1183/8192` bytes (14.4%), Flash
`13940/253952` bytes (5.5%). No upload, PWM measurement, Talon FX calibration,
motor direction test, or powered flywheel test was performed.

---

## 2026-08-23 - Codex: Shooter Constants and subsystem API refactor

**User Request:** Synchronize the formal branches and then refactor one owning
branch at a time. Start with the formal `Shooter` branch and make its Constants
and subsystem API as readable as the referenced Team8169 FRC structure. Keep
the normal single-worktree workflow.

**Branch State:** Before this refactor, the verified Shooter baseline was
committed as `d829bd0` and pushed to `origin/Shooter`. `Chassis`, `Dribbler`,
`ShooterMG996`, `dev`, and `main` were fetched and already matched their remote
tracking refs. No subsystem branch was merged into another branch.

**Architecture:** `ShooterConstants.h` is now grouped by mechanism as
`Angle`, `Rotate`, `Flywheel`, and visibly isolated `Legacy` calibration data.
Each mechanism owns its signal pin, PWM range, inversion, typed manual action,
and reserved PIDF configuration. Angle additionally owns AS5600 conversion,
sample validation, position limits, controller timing, readiness, and
closed-loop output limits. Shared `ServoMotorConfig` and
`PositionLimitConfig` value types moved to `src/Control/ActuatorConfig.h`.
Duplicate Shooter pin macros were removed from `Pins.h` after all Shooter
references moved to `ShooterConstants.h`.

**Public API:** `Shooter.h` now presents capabilities in six visible groups:
lifecycle/safety, manual open-loop control, angle closed-loop/homing, status,
and telemetry, with implementation details private. Names consistently use
`Angle`, `Rotate`, and `Flywheel`. Manual methods are
`setAngleAction/setAngleOpenLoop`, `setRotateAction/setRotateOpenLoop`, and
`setFlywheelAction/setFlywheelOpenLoop`. The flywheel type still permits only
STOP/FORWARD. Closed-loop methods are limited to the sensor-backed angle axis.

**Sensor Boundary:** `AS5600Encoder` no longer imports Shooter Constants.
It receives an immutable `AS5600EncoderConfig` at construction and only owns
I2C acquisition, magnet validity, wrap-aware signed delta, multi-turn relative
counts, rejected-sample counting, and stale-sample validity. Shooter owns the
actual 4096-count, 180-count maximum-delta, and 100 ms timeout values.

**Preserved Behavior:** Outputs remain disabled by default. Stop is 1500 us.
The paired angle servos remain mirrored by inversion, angle software limits
remain inactive until homed with a valid encoder, PIDF state still resets when
a limit blocks motion, and all PIDF gains remain `0.0`. Rotate and Flywheel
remain open-loop. `Shooter::isReady()` intentionally remains false until
horizontal angle and flywheel-speed feedback exist.

**Evidence:** `git diff --check` passes. Native PIDF tests pass. PlatformIO
`megaatmega2560` release build succeeds: RAM `1203/8192` bytes (14.7%), Flash
`14006/253952` bytes (5.5%). The only warnings are in the vendored Seeed AS5600
library. No firmware upload, PWM measurement, sensor movement, homing, or
powered mechanism test was performed.

**Next:** Commit and push this refactor separately on `Shooter`, request Claude
review, then switch the normal working tree to the next owning branch. Do not
merge to `dev` until each subsystem's behavior has independent evidence.

---

## 2026-08-24 - Codex: isolated Mac keyboard Shooter bench control

**User Request:** Use a MacBook keyboard as the temporary real-time operator
interface while testing the formal `Shooter` branch through Arduino USB. Arrow
up/down manually jog the paired elevation servos, arrow left/right manually jog
the horizontal continuous servo, `1/2/3` select elevation zero/setpoint 1/upper
target, and `5/6/7/8/9` are reserved for horizontal position targets.

**Discussion Result:** Added a dedicated Mega bench firmware environment and a
standalone localhost Web Serial page. The existing Dashboard layout and normal
robot `main.cpp` are unchanged. The page sends complete manual axis state on
key press/release and a heartbeat while armed. Firmware starts disabled and
disables all outputs if commands stop for more than 250 ms. Horizontal numeric
targets are deliberately rejected because the Rotate axis has no position
sensor; pretending timed motion is an angle was rejected as unsafe.

**Why:** VS Code's line-oriented Serial Monitor cannot reliably represent key
release. A focused browser page provides both keydown and keyup events, while
the firmware watchdog remains the final safety boundary if the browser, USB,
or Mac stops sending. The bench build includes only Bench, Shooter, Sensors,
and Control sources, preventing Chassis, Dribbler, Vision, SBUS, and the stale
normal main from starting during this test.

**Changed:** `platformio.ini` now excludes `src/Bench/` from the normal Mega
environment and adds `mega_shooter_keyboard_test`, whose source filter includes
only the bench entrypoint and Shooter dependencies. `Shooter.h/.cpp` expose
read-only control-mode, AS5600 raw-count, and rejected-sample telemetry needed
by the bench display. No existing actuator command behavior was changed.

**Added:** `ShooterBenchConstants.h` owns serial rate, watchdog and telemetry
periods, limited manual authority, and provisional elevation targets.
`ShooterKeyboardTest.cpp` owns the USB command parser, enable/disable gate,
watchdog, numeric actions, and `$TEL`/`$EVENT` output. The separate
`tools/shooter_keyboard_test/index.html` page owns keyboard state, Web Serial,
telemetry display, focus-loss disable, and disconnect cleanup; its README owns
the operator procedure.

**Flow:** Chrome key state -> USB Serial at 115200 baud -> newline command ->
bench parser -> public `Shooter` API -> Servo PWM. `M,angle,rotate` carries each
manual direction as `-1`, `0`, or `1`. `E` neutralizes then enables outputs;
`X` neutralizes and disables; `K` refreshes the armed heartbeat. Space, page
blur, hidden page, disconnect, write failure, or a 250 ms firmware timeout all
lead to disabled outputs. `1` zeros the valid AS5600 at the current physical
reference. `2` requests 512 relative counts and `3` requests 1024 counts, but
only after homing and while outputs are enabled. `5..9` stop Rotate and emit
`ROTATE_POSITION_UNAVAILABLE_NO_SENSOR`.

**Calculation:** Initial manual commands are limited to `+/-0.10`. With the
current 1000/1500/2000 us motor endpoints, a non-inverted command `+0.10`
produces `1500 + 0.10 * (2000 - 1500) = 1550 us`; `-0.10` produces
`1500 + 0.10 * (1000 - 1500) = 1450 us`. The right elevation motor is inverted,
so it receives the opposite pulse for the same axis command. AS5600 conversion
is `degrees = relativeCounts * 360 / 4096`; setpoint 1 at 512 counts is 45 deg
and the provisional maximum at 1024 counts is 90 deg. These positions and
motor signs remain unverified mechanical values. PIDF gains remain all zero,
so numeric targets intentionally produce neutral closed-loop output until a
small P-only hardware test is explicitly configured.

**Impact:** The existing Dashboard, Chassis, Dribbler, Vision, Auto, SBUS,
ShooterMG996, and normal runtime orchestration are intentionally unchanged.
The normal Mega build remains 1203/8192 RAM and 14006/253952 flash. The isolated
bench build uses 1132/8192 RAM and 15696/253952 flash. Horizontal absolute
zero, limits, and setpoints still require a rotate encoder or homing switch.

**Evidence:** Native `PidfController` tests pass. JavaScript syntax check and
`git diff --check` pass. Both `megaatmega2560` and
`mega_shooter_keyboard_test` PlatformIO builds pass. The local page was rendered
and inspected with no browser console errors. Only pre-existing warnings in the
vendored Seeed AS5600 library remain. No firmware upload, serial permission,
electrical PWM measurement, encoder movement, or powered actuator test was
performed.

**Next Test:** Upload only `mega_shooter_keyboard_test`. Initially leave motor
power disconnected, connect Chrome, verify disabled telemetry and 1500 us on
all three displayed servo paths, then verify AS5600 raw/relative direction by
hand. With the mechanism unloaded, external rated servo power, shared ground,
and an immediate physical power cut, enable and tap each arrow briefly to
confirm direction at the limited 1450/1550 us commands. Physically place the
elevation at its known lower reference before pressing `1`. Tune P-only only
after this open-loop and sensor evidence; do not use `2/3` for motion while all
PIDF gains are zero, and do not enable `5..9` until Rotate feedback exists.

---

## 2026-08-24 - Codex: Shooter bench keyboard hint bar

**User Request:** Add an on-page keyboard guide to the standalone Shooter test
page without changing its existing telemetry layout.

**Changed:** Added a responsive keycap-style hint bar above the status strip in
`tools/shooter_keyboard_test/index.html`. It lists the manual elevation arrows,
manual rotation arrows, elevation keys `1..3`, output enable, emergency stop,
and disconnect controls. Horizontal position keys `5..9` are visibly marked as
waiting for a position sensor so the page does not imply that unimplemented
closed-loop behavior is available.

**Impact:** This is presentation-only. Serial commands, firmware, test logic,
telemetry cards, Dashboard layout, and all robot subsystems are unchanged.

**Evidence:** Extracted JavaScript passes `node --check`; `git diff --check`
passes. The page was served locally and visually inspected with all hints
visible, normal wrapping, no overlap, and no browser warning/error logs. The
Arduino connection button was not pressed and no hardware action was taken.

---

## 2026-08-24 - Codex: Direction-aware Shooter hard limits

**User Request:** Preserve and implement the proposed second safety layer for
the Shooter. Elevation and horizontal rotation each have two infrared
break-beam limit inputs. A triggered limit must block motion farther into that
end while still allowing the mechanism to move away from it.

**Legacy Source:** The supplied 1201-line
`/Users/jeremy/Downloads/all_robot_017/src/main.cpp` used Mega D52/D53 for
vertical up/down and D43/D42 for horizontal left/right. It treated a HIGH input
as triggered, but its hard-switch checks stopped both directions whenever
either switch on an axis was active. The new implementation retains those pins
and electrical assumption while replacing the full-axis lockout with
direction-aware escape behavior.

**Architecture:** `ShooterConstants.h` owns all four pin assignments, active
polarity, internal-pullup selection, and the 8 ms debounce period.
`DigitalLimitSwitchConfig` is the reusable hardware configuration value.
`Shooter` owns pin initialization, debounced states, telemetry accessors, and
the application of the safety state to actuator commands. The pure
`DirectionalLimit` control helper owns the rule: a reverse limit blocks only a
negative command and a forward limit blocks only a positive command.

**Direction Mapping:** Elevation up is the positive mechanism command and is
blocked by D52; elevation down is negative and is blocked by D53. Horizontal
right is positive and is blocked by D42; horizontal left is negative and is
blocked by D43. The paired elevation servo inversion remains inside each motor
configuration, after the mechanism-level safety decision. If both switches on
one axis are active, both nonzero directions are blocked; zero remains zero.

**Control Flow:** Each `Shooter::update()` refreshes the AS5600 and all four
debounced switches before checking the output-enable gate. Manual elevation and
closed-loop elevation both pass through the same hard-limit and AS5600
soft-limit path. If a closed-loop output points into a triggered limit, the
controller is reset and both elevation PWM outputs return to neutral, avoiding
continued integral accumulation. Horizontal manual output is limited before
PWM conversion. Reverse escape remains available on either axis.

**Bench Telemetry:** The isolated Shooter firmware `$TEL` packet now appends
four flags in this order: angle up, angle down, rotate left, rotate right. The
standalone keyboard page expects 26 fields and displays two hard-limit cards,
showing `clear` or `BLOCKED`. Its README documents the pin mapping, polarity,
debounce, floating-input warning, and directional behavior. The normal robot
Dashboard layout was not changed.

**Evidence:** The standalone native `DirectionalLimit` test passes all clear,
single-limit, both-limit, reverse-escape, and zero-command cases.
`git diff --check` passes. PlatformIO builds pass for both `megaatmega2560`
(1255/8192 bytes RAM, 14714/253952 bytes flash) and
`mega_shooter_keyboard_test` (1184/8192 bytes RAM, 16512/253952 bytes flash).
Only pre-existing warnings inside the vendored Seeed AS5600 library remain.
The bench page was rendered at the default desktop viewport and at 390 px
mobile width with no horizontal overflow, clipped metric text, or browser
warning/error logs. No firmware upload or powered hardware test was performed.

**Required Bench Verification:** Before actuator power, confirm every
untriggered input reads `clear` and every blocked beam reads `BLOCKED`. These
inputs use `INPUT`, not `INPUT_PULLUP`, so they must not float. Confirm sensor
output voltage is Mega-safe and change only the Constants polarity/pullup
settings if the actual module differs. With low manual authority and an
immediate physical power cut available, trigger one limit at a time: the key
toward that end must produce neutral PWM while the opposite key must still
move away. Repeat for all four switches, then test both switches on an axis.

---

## 2026-08-25 - Codex: Horizontal potentiometer and 80/20 position profile

**User Request:** Replace the Shooter horizontal full-turn encoder assumption
with a potentiometer because the mechanism is expected to move only about
`-90..+90 deg`. Preserve true manual open-loop control, add target-based motion
that runs at cruise speed for the first 80% and gradually slows over the final
20%, and make the same deceleration envelope available to PID control.

**Architecture:** `AnalogPositionSensor` owns periodic A3 sampling, broad
electrical-range validity, timeout, and exponential filtering. `ShooterConstants`
owns the signal pin, provisional three-point calibration, soft limits, filter,
PIDF values, ready criteria, and 80/20 profile values. The pure
`PositionDecelerationProfile` helper owns the testable envelope calculation.
`Shooter` owns mode selection, target state, controller state, hard/soft safety,
angle conversion, PWM output, and telemetry. The standalone bench firmware and
page own only operator commands and presentation.

**Calibration:** Legacy `all_robot_017` evidence used A3 and observed raw values
near `55..535`. This implementation intentionally starts with provisional raw
points `90 / 295 / 500` mapped to `-90 / 0 / +90 deg`. These are not accepted
mechanical calibration values. Measure left, center, and right with actuator
power off and replace the Constants before any target-position motion. A3 is
also named `PIN_POT` in the legacy pin header; no other active source claims A3.

**Modes:** `MANUAL_OPEN_LOOP` keeps the arrow-key command direct and therefore
has no target or percentage-of-travel profile. `PROFILED_POSITION` uses
potentiometer feedback but no PID gains: it directly commands the sign of the
position error under the 80/20 envelope. `CLOSED_LOOP` calculates PIDF and then
clamps its magnitude to the same envelope. PIDF defaults remain `0.0`, so PID
mode intentionally stays neutral until hardware tuning. Profiled position is
feedback-based even though it does not use PID; it must not be described as
true open-loop control.

**80/20 Calculation:** At target selection, the subsystem captures the initial
absolute error. While `abs(currentError) / initialError >= 0.20`, the envelope
is the configured cruise command. Inside the final 20%, it is
`cruise * remainingRatio / 0.20`, with a configurable minimum approach ratio.
The normal cruise constant is `1.0`; the isolated bench passes a `0.10` maximum,
so its envelope ranges from 10% down to 1%. Within two raw counts the output is
neutral; after 100 ms continuously in tolerance, Rotate reports ready.

**Safety:** Every horizontal command still passes through D43/D42 direction-
specific hard limits. When A3 is valid, raw 90/500 are additional soft limits
that block only motion farther outward. Position modes fail neutral when the
sensor is invalid. Manual mode remains available under hard limits, but A3
soft limits are applied when its reading is valid. The broad `5..1018`
electrical validity range cannot guarantee detection of every floating or
miswired analog input, so hard switches and pre-power telemetry checks remain
required.

**Bench Controls and Telemetry:** `P` selects profiled position (default), `C`
selects PID, and `5/6/7/8/9` request center, left limit, left intermediate,
right intermediate, and right limit. Numeric target commands are capped at
10%. The `$TEL` packet grows from 26 to 43 fields and appends Rotate mode,
sensor validity, raw and filtered readings, degrees, target raw/degrees, error,
progress, profile envelope, output, PIDF terms, ready, and soft-limit status.
The standalone page displays these values without changing the robot Dashboard.

**Evidence:** Native PIDF, DirectionalLimit, and PositionDecelerationProfile
tests pass. JavaScript syntax and `git diff --check` pass. PlatformIO builds
pass for `megaatmega2560` (1412/8192 RAM, 17698/253952 flash) and
`mega_shooter_keyboard_test` (1564/8192 RAM, 20746/253952 flash). The bench page
was inspected at desktop and 390 px mobile widths with no horizontal overflow
or browser warning/error logs. Only pre-existing warnings in the vendored
Seeed AS5600 library remain. No firmware upload or powered hardware test was
performed.

**Required Bench Verification:** First upload the isolated test environment and
leave actuator power disconnected. Confirm A3 raw/filtered values move smoothly
and monotonically by hand, identify the actual left/center/right values, update
Constants, then verify all four hard-limit flags. With the mechanism unloaded,
external rated servo power, shared ground, and a physical power cut ready, use
arrows first to validate signs. Select `P`, test only the center and nearby
intermediate target at the 10% cap, and confirm progress/envelope/output fall in
the final 20%. Tune P-only later with `C`; keep I, D, IZone, and FF at zero until
P-only direction, response, soft limits, and stop tolerance are proven.

---

## 2026-08-25 - Codex: Elevation numeric key remap

**User Request:** Change the isolated Shooter keyboard mapping to `1` elevation
zero, `2` elevation minimum, `3` elevation setpoint 1, and `4` elevation
maximum.

**Implementation:** The bench command parser keeps `1` as the current-position
AS5600 zero operation. Key `2` now requests `ANGLE_INITIAL_COUNTS`, key `3`
requests `ANGLE_SETPOINT_1_COUNTS`, and the previous maximum command moves to
key `4`. All three target keys retain the existing output-enabled and
angle-homed/encoder-valid gates in `Shooter::setAngleTargetCounts()`. The
standalone page key hints, accepted key list, and README were updated to match.

**Scope:** No Shooter mechanism calculations, Constants values, horizontal
controls, serial telemetry format, normal robot Dashboard, or other subsystem
were changed. The `mega_shooter_keyboard_test` build passes at 1578/8192 RAM
and 20794/253952 flash; page JavaScript syntax and `git diff --check` pass. No
upload or powered mechanism test was performed.
## 2026-08-25 - Codex: Uncalibrated Shooter position limits reset to zero

Jeremy confirmed that all provisional Shooter position-limit/calibration
values should be zero until hardware measurement; existing motor and limit
switch pins must remain unchanged.

- Angle `MIN_POSITION_COUNTS`, `MAX_POSITION_COUNTS`, and bench targets are now
  zero. `POSITION_LIMITS_CALIBRATED=false` disables both software limits and
  rejects closed-loop angle targets. AS5600 zeroing and low-authority manual
  open-loop control remain available.
- Rotate left/center/right raw and degree calibration values are now zero.
  `POSITION_CALIBRATED=false` disables software limits, rejects profiled/PID
  position targets, and makes degree telemetry return zero without dividing by
  an uncalibrated zero-width range.
- Physical direction-specific limit switches remain active on D52/D53 and
  D43/D42. Motor/sensor signal pins, PWM values, inversion, and PIDF gains were
  not changed.
- Keyboard commands 2-9 now fail safely through the subsystem target guards;
  manual arrow control and command watchdog behavior are unchanged.

Verification: `git diff --check` and all three native tests passed. PlatformIO
was not rerun by Codex because `pio` is unavailable in its shell. Jeremy had
already reported a successful compile before this latest constants change, so
this revision still needs a fresh PlatformIO compile and has not been uploaded
or hardware-tested.
## 2026-08-25 - Codex: Independent Shooter soft-limit enable flags

Jeremy approved FRC-style explicit soft-limit enable settings. Angle and Rotate
now each expose `FORWARD_SOFT_LIMIT_ENABLED` and
`REVERSE_SOFT_LIMIT_ENABLED` in `ShooterConstants.h`; all four default to
`false`. The separate calibration flags remain false, and all position values
remain zero. A software limit reports active only when calibration is valid,
at least one direction is enabled, and the required feedback/homing state is
valid. Compile-time assertions reject enabling a soft limit before calibration
or with a reversed/equal range. Physical limit inputs and every pin remain
unchanged. Native control tests and `git diff --check` pass; PlatformIO was not
available in Codex's shell, so Jeremy must recompile before upload.

## 2026-09-19 — Codex: isolated A2 horizontal manual bench

**User Request / Discussion Result:** Jeremy wants manual keyboard testing of
the camera-carrying horizontal turntable first, then camera-only following
without AS5600. He confirmed a continuous-rotation servo and explicitly
approved implementation. Photo IMG_1026 shows 35Kg HV / 360 degrees / XT;
it does not verify neutral pulse, voltage, speed, or exact vendor specification.

**Why:** Existing Shooter init attaches multiple actuators and initializes
feedback. Added an isolated environment rather than changing production D29
or energizing unrelated outputs. This follows the manual-first subsystem skill.

**Changed / Added:** platformio.ini adds mega_rotate_only_test; only
Bench/RotateOnlyTest.cpp is built. RotateBenchConstants.h owns A2 (Mega D56),
limits D43/D42, neutral1500us, +/-50us jog, timeout250ms and jog cap400ms.
RotateBenchState.h is a pure tested command/watchdog state machine.
tools/shooter_keyboard_test/rotate.html is a horizontal-only Web Serial page;
README provides the procedure. Existing full-Shooter UI and runtime unchanged.

**Flow / Contract:** Chrome -> USB115200 newline E (neutral+enable), X
(stop+disable), M,0,-1/0/1 (left/stop/right) -> state machine -> only A2 Servo.
No K heartbeat: browser repeats complete manual state every50ms. Firmware emits
$ROTATE,1,enabled,pulse_us,left_blocked,right_blocked at100ms. Page refuses to
enable without this signature. Numeric angle commands and malformed commands
fail disabled; overflow discards the entire line. Bounded32-byte parser work
prevents watchdog starvation. Enter stops/disables; Space stops/disconnects;
enable requires the visible UI button. Blur/hidden/telemetry loss stop browser
commands. Write-generation invalidation cancels queued pre-stop commands.

**Calculation:** pulse=1500+direction*50us, direction=-1/0/+1; hence
1450/1500/1550us (inversion false). 50/500=10% of historical half-span,
not measured RPM or degrees. Unsigned now-lastCommand>250ms disables;
same-direction jog>=400ms disables despite heartbeat, requiring re-enable.
No position feedback means no absolute angle or software travel limit.

**Safety / Impact:** D43/D42 retain active-high direction-specific protection,
with pull-ups so disconnected inputs block; immediate stop+disable on a limit.
After re-enable the opposite direction can escape. Actual switches/wiring not
confirmed; if absent, test is blocked pending explicit constrained alternative,
not bypassed. Neutral is provisional and is not electrical power removal.
No AS5600/A3 initialization, no elevation/flywheel/vision/chassis activation.
YOLO target validity/UART contract and eventual camera-follow control untouched.

**Evidence:** Native C++ state tests passed (neutral, +/- pulses, release,
timeout, no rearm by manual state, jog cap despite heartbeat, bad command,
disable and millis rollover). Node mock UI tests passed (button-only enable,
arrow/release, Enter, blur, wrong firmware rejection, stale queued-write
cancellation). All three PlatformIO environments compile/link: normal1412 RAM/
17542 flash; old bench1522/19242; new rotate-only395/5350 bytes. Existing normal
and old bench warn about zero calibration division in Shooter.cpp and vendored
AS5600; no warning in new isolated build. git diff --check passed. No real
browser render/USB/servo measurement, upload, actuation, commit or push.
User's .vscode/extensions.json change preserved.

**Next Test:** Confirm limit wiring before powering motor; upload ONLY new
environment manually, signal-only inspect A2/disabled state and stop paths.
Then unloaded calibrate actual neutral, short jog signs, 400ms cutoff, limits,
USB loss, physical power cut and camera cable clearance. Claude please review
the isolated test and protocol before camera-follow integration. Pure image
tracking must not weaken the existing RGB-D valid flag; separate future contract.

## 2026-09-19 — Codex: approved no-limits A2 bench option

Jeremy confirmed only camera and continuous servo are installed, no D43/D42
switches, and approved a clearly acknowledged no-limits manual option. Added
checkbox + E_NO_LIMITS command, not a permanent bypass. E retains protected
mode. stop(), bad commands, timeout and jog cap clear noLimits. Firmware limit
application uses this state; UI displays actual bypass from protocol-v2 telemetry
($ROTATE,2,enabled,pulse,leftHigh,rightHigh,noLimits), rejects v1 and clears the
checkbox on stop/firmware disable. Changing checkbox while armed stops outputs.
Why: permit the explicitly approved bench setup without fake pin jumpers or
silently disabling production limits. Only dedicated A2 firmware/page/tests/docs
changed; production Shooter, old multi-axis UI and YOLO remain unchanged.
Pulse formula remains1500+direction*50us: -1=>1450,0=>1500,+1=>1550.
250ms command timeout and400ms same-direction cap remain; neither guarantees
angle or prevents accumulated rotation/cable entanglement. No encoder feedback.
Native C++ and Node mock tests pass, including bypass with both limits high,
normal-mode blocking, bypass revocation by X/timeout/cap, checkbox command,
uncheck stop and old-protocol rejection. No upload/physical motion performed.
Next: update dedicated firmware and page together; motor power off verify A2
and mode/stop reporting, then unloaded neutral/direction and physical cutoff test.

## 2026-09-19 — Codex: enable race correction, explicitly approved

Jeremy reported immediate disable and no movement despite no-limits checkbox.
Read-only mock reproduced old disabled telemetry cancelling the queued enable
write (zero serial writes). Jeremy approved fixing this software defect; it is
not proof that this alone caused the hardware symptom. Added pendingEnable:
old disabled telemetry cannot clear the pending write; only matching enabled
mode telemetry after write completion arms arrows. Waiting sends neutral only,
never repeats E, expires at1000ms; 350ms telemetry timeout still applies.
Stop/blur/mode-change cancels pending; no async completion can rearm after stop.
UI now labels focus loss, hidden page, telemetry/enable timeout and manual stops.
Dedicated firmware adds an optional final v2 stop reason for250ms TIMEOUT,
400ms JOG_CAP, LIMIT, BAD_COMMAND, OPERATOR and BOOT. Old v2 is accepted but
its firmware reason is explicitly unknown. Output mapping remains1500+dir*50us
(+1=>1550,-1=>1450), no timeout/output increase or production/YOLO change.
Node mock regression including exact race, no motion before confirmation,
matching-mode arm, jog-cap reason and enable timeout passes; native state tests
and isolated PlatformIO build pass. No upload, motor action or live-browser
hardware verification. Next: reload page with motor power off, verify pending
then confirmed state, short key pulse and stop; upload rebuilt dedicated firmware
only to add exact stop reasons. Full physical neutral/direction tests remain.

## 2026-09-19 — Claude Code: JOG_OFFSET_US raised 50us -> 250us

Jeremy uploaded the mega_rotate_only_test firmware from the entry above and
reported holding an arrow key produced no visible motion, then an apparent
disable right around release. Before changing anything, asked which exact
stopReason the page showed at disable, since the firmware already
distinguishes TIMEOUT/JOG_CAP/LIMIT/BAD_COMMAND — a real bug (e.g. a stale
disabled telemetry line, or a mode mismatch) would look different from the
already-documented MAX_JOG_MS=400 safety cap simply firing before a 10%-
authority (50us) jog produced any perceptible rotation against real mechanism
friction. Jeremy did not report back which reason string appeared and instead
asked directly to raise the authority to half of the historical 500us
half-span. Changed `RotateBenchConstants::JOG_OFFSET_US` from 50 to 250 (still
below `NEUTRAL_US=1500`'s midpoint distance to either 1000/2000 rail), updated
its `static_assert` ceiling from 50 to 250 to keep it a real gate rather than
silently widening scope, and updated the adjacent comment with the date/who/why.

This does not by itself prove which of the two hypotheses (JOG_CAP firing as
designed vs. a real disable-path defect) explains what Jeremy saw — increasing
authority only helps if the true cause was insufficient torque against
friction. If the same hold-then-disable pattern recurs at 250us, the next
diagnostic step is still to read the exact stopReason on screen before
changing any more constants.

**Changed:** `src/Constants/RotateBenchConstants.h` only (`JOG_OFFSET_US`
50->250, its `static_assert` ceiling 50->250, comment updated).

**Impact:** Output pulse mapping is now `1500 + direction*250us`
(+1=>1750us, -1=>1250us; previously 1550/1450). No other constant
(NEUTRAL_US, COMMAND_TIMEOUT_MS=250, MAX_JOG_MS=400, TELEMETRY_MS=100,
INVERTED, pins) changed. Production Shooter branch, dev, main, and the
YOLO/Orin side are untouched.

**Evidence:** Updated the two hardcoded pulse assertions in
`test/rotate_bench_state_test.cpp` (1550->1750, 1450->1250) that were tied to
the old offset — re-ran it natively, passes. Re-ran `test/rotate_page_test.cjs`
(unaffected, doesn't assert pulse magnitude), passes. Re-compiled
`src/Bench/RotateOnlyTest.cpp` with `avr-g++ -fsyntax-only` against this
machine's AVR toolchain/Mega2560 core, clean (only a pre-existing, unrelated
`<util/delay.h>` optimization-flag warning from the manual syntax-only
invocation, not from `pio`'s real build flags). No `pio` build/upload, no
motor power, no live hardware test performed by Claude.

**Next Test:** Jeremy re-uploads `mega_rotate_only_test` and re-tests a short
key press with motor power confirmed safe (belt/coupling checked per the
page's own warning banner). If it still doesn't move or still disables
immediately, read the exact `stopReason` text shown on the page at that
moment and report it verbatim before any further constant change — that
string is the actual discriminator between "safety cap firing as designed"
and "a real defect," and guessing a bigger number again without it risks
masking a real bug instead of fixing it.

## 2026-09-19 — Claude Code: removed MAX_JOG_MS (400ms hold cap) at Jeremy's request

Jeremy did not report the `stopReason` string from the entry above and
instead said the release-then-disable pattern was this feature and asked to
remove it, stating the current setup is motor + camera only, operator
present, doing manual testing. This still doesn't establish whether the
original symptom was JOG_CAP firing as designed or something else — Jeremy
chose to remove the cap rather than diagnose further, which is his call to
make for a supervised bench test.

**Changed:**
- `src/Control/RotateBenchState.h`: removed the `jogStarted` member and the
  `else if (... direction!=0 && now-jogStarted>=MAX_JOG_MS) stop("JOG_CAP")`
  branch from `tick()`; removed the now-dead `jogStarted = now` write in
  `command()`. `COMMAND_TIMEOUT_MS` (250ms loss-of-link watchdog) is
  untouched and still the only auto-stop condition in `tick()` — deliberately
  kept, since it protects against a stale/missing-command condition (USB
  drop, page crash) that is unrelated to "held a key too long while
  watching," which is what Jeremy asked to remove.
- `src/Constants/RotateBenchConstants.h`: removed the now-unused
  `MAX_JOG_MS` constant (grepped the tree first; nothing else referenced it).
- `tools/shooter_keyboard_test/rotate.html`: updated the warning banner and
  the keyboard-help paragraph, which both explicitly described the 400ms cap
  to the operator — left stale UI copy would have misdescribed current
  firmware behavior. Left the `JOG_CAP` entry in the client-side `reasons`
  lookup table as-is (harmless defensive mapping if an unflashed older
  firmware image ever sends that reason; does not assume it will).

**Why:** Requested explicitly. `COMMAND_TIMEOUT_MS` stays because it guards a
different, still-relevant failure mode (control link lost while a nonzero
command was last in effect) that has nothing to do with intentional
sustained holding during supervised testing.

**Impact:** With outputs enabled, holding a direction key now drives the
motor continuously until released (or until 250ms passes with no command,
or a limit/disconnect/blur/hidden-page stop fires) instead of forcibly
stopping and requiring re-enable every 400ms.

**Evidence:** Updated `test/rotate_bench_state_test.cpp`'s two blocks that
asserted JOG_CAP firing after a sustained multi-heartbeat hold (both the
normal and no-limits-mode cases) to instead assert the mechanism stays
enabled through that same window — re-ran natively, passes. Re-ran
`test/rotate_page_test.cjs` (unaffected), passes. Re-compiled
`src/Bench/RotateOnlyTest.cpp` with `avr-g++ -fsyntax-only`, clean. No `pio`
build/upload, no motor power, no live hardware test performed by Claude.

**Next Test:** Jeremy re-uploads and re-tests. Since the original symptom's
root cause was never confirmed via `stopReason`, if holding-then-releasing
still shows unexpected disable behavior after this change, that is now good
evidence it was never JOG_CAP alone — check for TIMEOUT (missed heartbeats,
e.g. a slow/blocked serial write) or a genuine firmware/hardware issue next,
rather than assuming another constant needs tuning.

## 2026-09-19 — Claude Code: JOG_OFFSET_US raised to 500 (full historical authority)

Jeremy confirmed the previous 250us (50%) change worked ("好，可以了"),
asked to speed up further with no target number, and separately confirmed
the Mega<->Orin Nano UART wiring (pins 6/8/10) is handled. He then
explicitly authorized full speed: "速度可以調整為全速，線我已確認."

**Changed:**
- `src/Constants/RotateBenchConstants.h`: `JOG_OFFSET_US` raised from `250`
  to `500` — 100% of the historical 500us half-span, i.e. the servo's full
  attached range (1500 +- 500 = 1000-2000us, matching the `.attach(SIGNAL_PIN,
  1000, 2000)` call in `RotateOnlyTest.cpp`). Updated the `static_assert`
  ceiling from `<= 250` to `<= 500` in lockstep so it remains a real bench-
  authority gate rather than a stale/no-op check.
- `test/rotate_bench_state_test.cpp`: updated the three pulse-value
  assertions tied to the offset (`1750`->`2000` twice, `1250`->`1000`) to
  match.
- No change to `tools/shooter_keyboard_test/rotate.html` — it doesn't
  hardcode the offset, it only displays whatever pulse value telemetry
  reports.

**Why:** Explicitly requested. Jeremy stated the wiring/voltage question
(Mega TX 5V -> Orin RX 3.3V direction) is handled on his side before asking
for full speed, so this change assumes that is resolved outside this repo.

**Impact:** With outputs enabled, a held direction key now commands
1000us/2000us (previously 1250us/1750us) — the servo's full attached rail
in each direction. No other behavior changed (COMMAND_TIMEOUT_MS, limit
handling, JOG_CAP removal from the prior entry all unaffected).

**Evidence:** Re-ran `test/rotate_bench_state_test.cpp` natively (g++
-std=c++17), passes. Re-ran `test/rotate_page_test.cjs`, passes.
Re-compiled `src/Bench/RotateOnlyTest.cpp` with `avr-g++ -fsyntax-only`
against the cached AVR toolchain/core/variant, clean. No `pio` build/upload,
no motor power, no live hardware test performed by Claude.

**Next Test:** Jeremy re-uploads `mega_rotate_only_test` and re-verifies at
full authority before trusting it unattended: confirm direction (left/right
keys drive the correct physical direction), confirm both physical stops are
reached without the servo audibly straining past them, and confirm the
limit switches (or no-limits mode, if still unwired) still behave correctly
at the higher speed — a mechanism that looked fine at 50% authority can
still overshoot or hit a limit harder at 100%.

## 2026-09-21 — Claude Code: new isolated bench, `mega_vision_rotate_test` (A2 driven by Orin vision instead of the keyboard)

Jeremy is about to wire the Mega to the Orin Nano for the first time (Orin
40-pin pins 6/8/10 = GND/TX/RX -> Mega Serial1; Orin on its stock 19V
supply, Mega on Mac USB) and wants to test "pure horizontal camera-follow":
one 360 continuous servo (the same A2 as `mega_rotate_only_test`), no AS5600
yet (backup only, not installed). Checked the actual Orin-side protocol in
`YOLO_Detect_single/serial_tx.py` + `config.py` rather than trusting the
`orin_mega_power_gated_handshake` memory note, which turned out stale for
this branch: the `MEGA_READY`/`MEGA_HEARTBEAT` handshake exists in the
`Chassis`/`Dribbler`/`STM32-ELRS` branches' `Vision.cpp` but not in
`Shooter`/`dev`/`main`, and none of them parse the real 5-field
`tx,ty,distance,target_id,valid` CSV (current `Shooter` `Vision.cpp` only
reads the first field). This bench does not touch any of that production
code — it is a third, fully isolated environment, parallel to
`mega_rotate_only_test`, built from scratch against the protocol actually
observed in `serial_tx.py`.

**Added (all new files, nothing existing modified except `platformio.ini`):**
- `src/Constants/VisionRotateBenchConstants.h` — reuses the already-tuned
  mechanism constants from `RotateBenchConstants.h` (pin, neutral, limits,
  `COMMAND_TIMEOUT_MS`, USB baud) via `using` declarations instead of
  duplicating them, and adds: `PROTOCOL_VERSION=1` (must match Orin's
  `CONTROLLER_PROTOCOL_VERSION`), `VISION_BAUD=115200` (Serial1, matches
  Orin `SERIAL_BAUD`), `DEADBAND_PX=25` (provisional), `FOLLOW_JOG_OFFSET_US
  =150` (deliberately far below the 500us manual-jog ceiling — first time
  this servo is ever driven by vision data instead of a human),
  `VISION_TIMEOUT_MS=500`, `MEGA_HEARTBEAT_INTERVAL_MS=200` (well under
  Orin's 1.0s `MEGA_HEARTBEAT_TIMEOUT_SECONDS`).
- `src/Control/VisionRotateBenchState.h` — new pure state machine (host
  testable, mirrors `RotateBenchState.h`'s style). Direction comes only from
  `onVision(tx, valid, now)`: `valid=0` forces direction to 0 immediately
  (never coasts on a lost target), otherwise a deadbanded sign of `tx`. The
  browser only arms/disarms (`E`/`E_NO_LIMITS`/`X`) and proves its tab is
  alive via a new `P` ping (no direction keys are sent — direction is 100%
  firmware-computed from vision). Two independent watchdogs in `tick()`:
  `COMMAND_TIMEOUT_MS` on the USB `P` pings (browser tab/USB link) and
  `VISION_TIMEOUT_MS` on Serial1 vision lines (Orin link) — either lapsing
  stops the motor with a distinct `stopReason` (`TIMEOUT` vs
  `VISION_TIMEOUT`).
- `src/Bench/VisionRotateTest.cpp` — firmware entry. Sends `MEGA_READY` on
  Serial1 immediately in `setup()`, independent of the motion-armed gate,
  so Orin can start streaming before the operator even opens the browser
  page; sends `MEGA_HEARTBEAT` every `MEGA_HEARTBEAT_INTERVAL_MS`. Parses
  the Orin's 5-field CSV off Serial1 with a small bounded hand-rolled
  `strtol`-based parser (malformed lines are dropped silently — only
  silence trips `VISION_TIMEOUT`, a bad line isn't treated as an operator
  error). USB telemetry line (`$VROTATE,3,...`) always includes the decoded
  vision fields (`tx,ty,distance,target_id,valid,msSinceVision`) regardless
  of armed state, so Jeremy can watch real vision data on the page before
  ever enabling the motor.
- `platformio.ini`: new `[env:mega_vision_rotate_test]`, extends
  `env:mega_shooter_keyboard_test` the same way `mega_rotate_only_test`
  does (inherits its `build_flags`/`lib_deps`, only swaps
  `build_src_filter`).
- `tools/shooter_keyboard_test/vision_rotate.html` — new Web Serial page,
  same safety scaffold as `rotate.html` (`pendingEnable`/`enableWritten`/
  `ENABLE_TIMEOUT_MS` race guard, blur/hidden/pagehide/disconnect all stop,
  `noLimits` requires an explicit checkbox). No arrow-key handling — while
  armed it sends `P` every 50ms instead. Adds a live "鏡頭資料" panel
  (tx/ty/distance/target_id/valid/msSinceVision) so Jeremy can validate the
  Orin link before pressing enable. Warning banner states plainly that the
  tx-sign-to-rotation-direction mapping has never been verified.
- `test/vision_rotate_bench_state_test.cpp`, `test/vision_rotate_page_test.cjs`
  — new native/host tests mirroring the existing rotate-bench test style:
  deadband edges (25px boundary, both signs), `valid=0` override,
  `VISION_TIMEOUT` firing while USB pings stay healthy, `TIMEOUT` firing
  while vision stays healthy (proves the two watchdogs are independent),
  limit/no-limits behavior, explicit `X`, `BAD_COMMAND`, `P` while disabled
  is a no-op not an error, 32-bit `millis()` rollover, plus page-side
  protocol/vision-panel/race-guard checks.

**Why:** First-ever Orin<->Mega wiring and first-ever vision-driven motor
motion on this project. Kept a manual browser-page enable gate (Jeremy's
explicit choice) rather than "moves automatically the instant a target is
valid," and kept the follow speed far below the already-authorized manual
jog ceiling, because none of direction sign, link stability, or mechanical
behavior under vision control have ever been observed on real hardware.

**Impact:** Fully additive — `megaatmega2560` (production), `src/Vision/*`,
`main.cpp`, and `mega_rotate_only_test` are untouched. Jeremy separately
said (2026-09-21) that if horizontal camera-follow checks out, this should
eventually be merged into `dev` and become the real Shooter-rotate path,
noting the motor signal pin will likely change from the bench's A2 later —
noted for a future task, no merge or production wiring done yet. He then
specified the target packet format for that merge: match the `Chassis`
branch's `Vision.cpp`/`Vision.h`/`VisionConstants.h` (checked via `git show
Chassis:...`, not from memory), which is materially more complete than this
bench's parser or the current `Shooter` `Vision.cpp`:
- Class-based `Vision` tracking `tx/ty/distance/targetId/valid` plus link
  health (`hasPacket`, `lastPacketMs`, `PACKET_TIMEOUT_MS=300`) **and**
  Orin's own lifecycle (`OrinState::STANDBY/STARTING/READY/ERROR`, parsed
  from inbound `VISION_STANDBY,<ver>` / `VISION_STARTING,<ver>` /
  `VISION_READY,<ver>` / `VISION_ERROR,<ver>` lines that Orin's
  `send_status()` sends) — this bench's `VisionRotateTest.cpp` only sends
  `MEGA_READY`/`MEGA_HEARTBEAT` outbound and never parses those inbound
  Orin-state lines, which is a real gap versus what Jeremy wants merged.
- Strict field parsing (`strtod`/`strtol` with `*end=='\0'` checks, exact
  comma-count check) and range validation
  (`MAX_ABS_TX=400`/`MAX_ABS_TY=1000`/`MAX_DISTANCE_MM=20000`/target-id
  range) that rejects and invalidates on any out-of-range or malformed
  packet, versus this bench's looser hand-rolled parser.
- `PACKET_MAX_CHARS=80` (vs. this bench's 32-char line buffer),
  `HEARTBEAT_INTERVAL_MS=100` (vs. this bench's 200ms).
Not applied to the bench now — Jeremy said this is for "合併時"
(merge time), and no hardware test has happened yet. Recorded here so the
eventual merge/rewrite has the exact reference instead of a vague pointer.

**Evidence:** `g++ -std=c++17` on `vision_rotate_bench_state_test.cpp`,
passes. `node test/vision_rotate_page_test.cjs`, passes. `avr-g++
-fsyntax-only` on `VisionRotateTest.cpp` against the cached AVR
toolchain/core/variant, clean (only the pre-existing harmless
`util/delay.h` optimization warning also seen on every other bench file).
No `pio` build/upload, no motor power, no Orin connected, no live hardware
test performed by Claude.

**Next Test:** Jeremy wires Orin<->Mega, uploads `mega_vision_rotate_test`,
opens `vision_rotate.html`, and — **without pressing enable** — first
confirms the "鏡頭資料" panel updates sanely (tx/ty/distance/valid change as
a target moves in frame, `msSinceVision` stays small and doesn't grow,
Orin's own console shows `MEGA_READY received` so the handshake is
confirmed both directions). Only after that, press enable and watch the
very first movement closely: confirm the physical rotation direction
matches the sign convention (`tx>0` = target right of center should
rotate the camera to bring it back toward center), confirm it actually
approaches and settles near center instead of diverging, and keep a hand on
power the whole time since neither the direction sign nor the mechanical
behavior under vision control has been observed before.


## 2026-09-23 — Codex: approved bench signal A2 → A5; UART echo unresolved

Jeremy moved the physical servo signal to A5 and approved updating both manual
and vision bench environments, keeping production Shooter unchanged. Changed
shared RotateBenchConstants SIGNAL_PIN from56 to59 (Mega core PIN_A5=59), both
bench static_asserts, page titles/telemetry labels, README and platformio comments.
Vision bench inherits the shared pin. No UART pins, baud, protocol, debug output,
speed or safety changes. Manual remains1500±500us; vision remains1500±150us
(1350/1500/1650), not measured RPM. Production Rotate staysD29.
Evidence: both native state tests and both Node UI mock tests pass; both PlatformIO
builds pass (manual450 RAM/5408 flash; vision654/7332). No upload or powered test.
Existing platformio.ini line29 whitespace warning and unrelated dirty files retained.
Jeremy's new screenshot showed RX1 VISION_STANDBY,1 plus MEGA_HEARTBEAT,1 and
0xFF. After stopping Orin Python he reports only MEGA_HEARTBEAT,1 remains.
This supports a return path for Mega TX1 to RX1, not proof of a particular short
or of successful Mega→Orin reception. He has no meter. Next diagnostic: power
everything off before isolating the external D19 RX1 connection, then observe
USB debug with motor power off; never enable follow until valid UART verified.
Claude: pin migration supersedes previous A2 bench descriptions; follow runtime
and Orin WAIT_MEGA remain unchanged. Reload pages and upload intended bench
only after review; Codex has not flashed either device.


## 2026-09-23 — Codex: approved isolated bare-Mega UART diagnostic

Jeremy reports bare Mega, no expansion board and NO D18/D19 jumper. Screenshot shows intermittent RX1 zero/corrupt bytes and fragments M/E/G, not a clean repeated heartbeat. Earlier intentional D18/D19 loopback successfully received MEGA_HEARTBEAT,1; that result is NOT evidence of an unintended short. Floating RX/coupling remains a hypothesis, not a diagnosis. Jeremy has no multimeter.

Jeremy approved an isolated diagnostic. Added TEL src/Bench/UartBareTest.cpp, src/Constants/UartBareBenchConstants.h, env mega_uart_bare_test in platformio.ini, and tools/shooter_keyboard_test/UART_BARE_TEST.md. No production or YOLO runtime changes. No Servo or actuator initialization. Serial1 RX1 D19 INPUT_PULLUP; 10 seconds QUIET (no TX) then 10 seconds TX (MEGA_HEARTBEAT,1 every 200 ms), then stop TX. USB reports aggregate counts, first 16 bytes in hex, and instantaneous RX pin level. Send R after DONE to repeat. Acquisition loops are bounded. No RX buffer flush at phase transition; driver-delivered bytes only, not framing-error/electrical measurements.

IMPORTANT: test USB-only bare Mega, no shield, no jumper, no Orin, no motor. Disconnect power before changing connections. Internal RX pullup is toward Mega 5V and this test must not be used connected to Orin. Restore the appropriate normal bench firmware before reconnecting the system.

Verification: pio run -e mega_uart_bare_test passed; RAM 416/8192 bytes, flash 4206/253952 bytes. Source inspected for isolated build and absence of actuator initialization. NOT flashed, NOT hardware-tested. Existing A5 benches and temporary RX1 per-byte logging untouched.

Next evidence requested: RESULT,QUIET and RESULT,TX lines at 115200 baud after 20 seconds. Both rx_bytes=0 supports floating-input hypothesis but does not prove all wiring/hardware healthy. RX only during TX suggests coupling/leakage/return path; cannot alone establish a hard short. RX during QUIET warrants further investigation. Claude: do not repeat the already-completed intentional loopback or treat it as unjumpered evidence; keep motor follow disabled until UART path is understood.

## 2026-09-25 — Claude Code: Orin<->Mega UART bring-up, full diagnostic log (link still not proven; no root cause confirmed yet)

Checkpoint of the entire Orin<->Mega UART debugging session so far (2026-09-21
through today), since it has spanned many small steps across both Claude and
Codex and needs a single place to catch up from. No code changes in this
entry — this is a record of what has actually been tested and what each
result does/doesn't prove. Motor follow has never been enabled; no upload of
`mega_vision_rotate_test` has been attempted since the `A2`->`A5` pin
migration Codex made (see Codex's own entries above for that change).

**Orin-side software (fixed, not the current blocker):**
- `/dev/ttyTHS1` initially failed with `Permission denied` — user `jeremy` was
  not in the `dialout` group. Fixed with `sudo usermod -aG dialout jeremy` +
  re-login. Port now opens successfully every time.
- Confirmed `vision_main.py`'s `--wait-for-mega` design is already correct
  (stays in lightweight standby, sending `VISION_STANDBY,1`, and does not
  import Ultralytics/open the camera until a versioned `MEGA_READY`/
  `MEGA_HEARTBEAT` line is seen) — no code change needed there.
- Confirmed via `deploy/README.md` that the boot-time systemd service
  (`yolo-vision.service.example`) is documented but was never installed on
  this Orin — manual `source venv/bin/activate && python3 test_coordinate.py
  --source realsense --serial --serial-port /dev/ttyTHS1 --wait-for-mega` is
  what Jeremy has been running by hand for every test so far.

**Root symptom (still unresolved):** Orin stays at `WAIT_MEGA: waiting for
MEGA_READY / MEGA_HEARTBEAT` indefinitely. Mega's bench firmware reports
`VISION_TIMEOUT` (no Serial1 data ever arrives). Raw-byte tests
(`stty -F /dev/ttyTHS1 115200 raw -echo && cat /dev/ttyTHS1`, and the same on
`/dev/ttyTHS2`, the only two `ttyTHS*` nodes that exist on this Orin) show
**zero bytes received, repeatedly, across many retests** — including after
reseating the physical header connector. This is the one fact everything
else has to be consistent with.

**What has been eliminated as the cause, with direct evidence (not just code
review):**
1. *Orin-side Python bugs* — `cat`/`stty` bypass Python and `serial_tx.py`
   entirely and read the OS UART driver's raw bytes directly. Zero bytes at
   that level cannot be explained by any Python logic error; it is
   necessarily upstream of the app. Full review of `serial_tx.py`/
   `vision_main.py`/TEL's bench receive path found nothing that would cause
   total silence (see the 2026-09-24 review entry in the shared inbox for
   the itemized findings — a few real-but-unrelated hardening items were
   found in `VisionRotateTest.cpp`'s `parseVisionLine`, none of which can
   cause zero bytes).
2. *Wrong `/dev/ttyTHS*` device node* — `sudo /opt/nvidia/jetson-io/jetson-io.py`
   confirms the 40-pin header pins 8/10 are configured as `uarta` (not
   `unused`). `sudo dmesg | grep -iE "tty|uart"` shows `ttyTHS1` probed at
   MMIO `0x3100000`, which is UARTA's address on this SoC — so `/dev/ttyTHS1`
   is confirmed to be the physically correct node for pins 8/10.
   `/dev/ttyTHS2` (MMIO `0x3140000`) is a different, unrelated controller.
3. *40-pin header UART not enabled in the pinmux* — ruled out by the same
   `jetson-io.py` output (already `uarta`, not `unused`).
4. *Mega's own Serial1 hardware* — an informal D18(TX1)<->D19(RX1) jumper
   loopback cleanly received Mega's own `MEGA_HEARTBEAT,1` character-by-
   character. Codex correctly flagged that this alone doesn't rule out an
   unintended permanent short (the test wasn't done in isolation). Codex
   then built `src/Bench/UartBareTest.cpp` (env `mega_uart_bare_test`) — a
   fully bare Mega (no shield, no jumper, no Orin, no Servo/actuator init)
   that runs 10s QUIET + 10s TX and reports aggregate `rx_bytes`/`tx_frames`/
   instantaneous RX pin level over USB. Jeremy ran it: `RESULT,QUIET,
   rx_bytes=0,...` and `RESULT,TX,rx_bytes=0,tx_frames=49,...` — clean in
   both phases. This is evidence *against* a hard TX1/RX1 short on the Mega
   board (a real short would show up here too), so Claude's earlier "Mega
   board is shorted" conclusion is retracted.
5. *Level-shifter HV/LV supply pins* — initially looked like the strongest
   lead (Jeremy confirmed to Codex that the module's dedicated `HV`/`LV`
   supply pins, distinct from the HV1-4/LV1-4 signal channels, are not
   wired, only GND is shared). This was retracted after Jeremy confirmed the
   exact same physical module, in the exact same unconnected-HV/LV
   configuration, was used successfully last year — so this specific board
   evidently does not require those pins to function, and it is not new to
   this setup.

**What is confirmed different from "last year, worked":** the Mega board is
the same physical unit as last year; the level-shifter module is the same
physical unit, moved over as-is; pins used (6/8/10) are unchanged. **The
Jetson Orin Nano itself is a new/different physical board** (not the unit
used last year) — this is the one confirmed-changed variable, though the
mechanism by which a new-but-same-model Orin would fail here is not yet
identified (pinmux and device node both check out correct on this board).

**In progress, no multimeter available (Jeremy doesn't have one):**
Physical reseating of the Orin-side header connector at pins 6/8/10 was done
and visually inspected (no bent/damaged pins) — retested `cat
/dev/ttyTHS1` afterward, still 0 bytes. Since electrical measurement isn't
possible, next steps are substitution-based (no special tools needed):
swap the three jumper wires between the Orin header and the level shifter
for unused spares (rule out an internally-broken wire, which is a common
dupont-wire failure mode invisible to inspection), then if still silent, try
different unused channels on the level shifter itself (rule out one dead
MOSFET channel on that specific board). Neither has been done yet as of this
entry.

**Evidence discipline maintained throughout:** no `pio` build/upload beyond
what Codex already verified (native/Node tests + `pio run` build, not
flashed by Claude), no motor power, no follow enabled, only Jeremy has
touched real hardware. All diagnostic firmware (`mega_uart_bare_test`) is
explicitly documented as USB-only / not for use connected to Orin (its RX
pull-up is to Mega's 5V).

## 2026-09-25 — Claude Code: Orin<->Mega UART link CONFIRMED WORKING — root cause was a JetPack 6.2.2 DMA driver bug, not hardware

**Resolution of the entire debugging thread above.** Root cause found and fixed;
no hardware was ever at fault. Physical wiring, the level-shifter module, the
Mega board, GND, pinmux, and the `/dev/ttyTHS1` device node were all correct
the whole time — the only actual variable was the new Orin Nano's JetPack
6.2.2 (L4T R36.x) UART DMA driver.

**Root cause:** confirmed via web search against multiple independent NVIDIA
Developer Forum threads (see Sources below) plus a maintained community fix
repo (`jetsonhacks/jetson-orin-uart`): on JetPack 6.2.2, UEFI injects `dmas`/
`dma-names` properties into the `serial@3100000` (`ttyTHS1`, 40-pin header
UART1, pins 8/10) device tree node at boot, enabling DMA mode. The
`serial-tegra` driver has a bug in DMA mode that zeroes most of each received
transfer's bytes to `0x00`. This explains every earlier "zero bytes" result:
`cat`/`stty` were in fact receiving data the whole time, but `0x00` bytes are
invisible in a terminal, so a stream of near-total-NUL garbage looked
identical to true silence. The one clue that should have caught this earlier:
switching from `cat` to `xxd`/hexdump revealed a clean, regular `0d 0a`
(`\r\n`) pattern every ~17-18 bytes — exactly matching Mega's
`MEGA_HEARTBEAT,1\r\n` cadence — proving frames were arriving on schedule,
just with their content zeroed.

**Fix applied:** `jetsonhacks/jetson-orin-uart` — a device tree overlay that
removes the `dmas`/`dma-names` properties from the `serial@3100000` node,
forcing the driver into PIO (interrupt-driven) mode, which is unaffected by
this bug. Installed with:
```
sudo apt install device-tree-compiler python3
git clone https://github.com/jetsonhacks/jetson-orin-uart.git
cd jetson-orin-uart && sudo bash install.sh && sudo reboot
```
The installer compiled the overlay, auto-detected this board's FDT
(`kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb`, confirming it is an
Orin Nano Super Dev Kit), backed up `extlinux.conf` before changing it, and
added a new `UARTFix` boot entry as default (previous entry kept as a
fallback in the boot menu — reversible without needing recovery mode).

**Verified on real hardware (by Jeremy, not Claude):**
- `sudo dmesg | grep -i '3100000\|pio\|dma'` after reboot shows
  `serial-tegra 3100000.serial: RX in PIO mode` and `... TX in PIO mode`.
- `stty -F /dev/ttyTHS1 115200 raw -echo && timeout 5 xxd /dev/ttyTHS1` now
  shows a perfectly clean, repeating `MEGA_HEARTBEAT,1\r\n` — confirmed on
  real hardware, this is the actual first successful byte-for-byte UART
  reception between this Mega and this Orin.

**Caveat for later:** the fix's own README warns that re-running
`jetson-io.py` will overwrite the `OVERLAYS` boot line and silently undo this
fix (falling back to broken DMA mode) — re-run `sudo bash install.sh` if that
ever happens again.

**Everything from the earlier entries in this debugging thread (missing
HV/LV supply, possible TX1/RX1 short, wrong device node, wiring reseat, etc.)
turned out to be dead ends** — worth keeping in the log as a record of what
was checked and ruled out, but none of it should be treated as a real
finding about this hardware going forward.

**Next Test:** with raw UART now proven, re-run the actual application layer:
`python3 test_coordinate.py --source realsense --serial --serial-port
/dev/ttyTHS1 --wait-for-mega` on the Orin (Mega still needs to be running
`mega_vision_rotate_test` — note Codex's pin migration from A2 to A5 means
this must be re-uploaded/re-verified since the UART bring-up). Expect Orin to
print `MEGA_READY received` immediately instead of hanging at `WAIT_MEGA`, and
`vision_rotate.html`'s "鏡頭資料" panel to start showing live tx/ty/distance/
valid data. Only after confirming that — and before ever pressing enable —
sanity-check the tx sign convention as previously planned; nothing about
direction/follow behavior has been tested yet.

Sources:
- [GitHub - jetsonhacks/jetson-orin-uart](https://github.com/jetsonhacks/jetson-orin-uart)
- [Solved: UART/Serial Port not working after upgradint to Jetpack 6.2.2 (Orin Nano/NX) - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/solved-uart-serial-port-not-working-after-upgradint-to-jetpack-6-2-2-orin-nano-nx/363837)
- [DMA on /dev/ttyTHS1 corrupts receiving data - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/dma-on-dev-ttyths1-corrupts-receiving-data/369191)

## 2026-09-25 — Claude Code: vision-follow direction was inverted, flipped independently of the manual bench

**First real vision-follow test result** (with UART now working): Orin
correctly locks a target and streams `tx/ty/distance/target_id/valid` to
Mega, `mega_vision_rotate_test` correctly reports enabled + those values live
in `vision_rotate.html`. Jeremy pressed enable and observed on real hardware:
target on the right (`tx>0`) rotates the servo *away* from center instead of
toward it — the sign convention assumed when this bench was built was wrong.

**Fix:** `VisionRotateBenchConstants.h` no longer inherits
`RotateBenchConstants::INVERTED` (the manual-jog bench's own, separately-
verified-correct setting) — it now declares its own `INVERTED = true`,
independent of the manual bench. This only affects `VisionRotateBenchState::
pulse()`'s sign; `onVision()`'s deadband/direction-sign logic is unchanged.
`FOLLOW_JOG_OFFSET_US` (150us) and `DEADBAND_PX` (25px) are unchanged.

**Evidence:** native `g++` state test updated (three hardcoded pulse
assertions flipped: 1650<->1350 at the two deadband-exceeded cases and the
no-limits case) and passes; `node vision_rotate_page_test.cjs` passes
unaffected (it doesn't depend on `INVERTED`); `avr-g++ -fsyntax-only` on
`VisionRotateTest.cpp` clean. **Not yet re-uploaded or re-tested on real
hardware** — Jeremy needs to re-flash `mega_vision_rotate_test` and confirm
the servo now turns toward the target instead of away from it before trusting
this.

**Next Test:** re-upload, re-enable with the same live-target setup, confirm
the servo now visibly moves toward center as the target moves right/left
(not away from it), and that it settles near center (doesn't oscillate or
overshoot) before considering follow behavior proven.

## 2026-09-25 — Claude Code: FOLLOW_JOG_OFFSET_US raised 150us -> 300us

Jeremy asked for full speed on the vision-follow bench right after the first
successful direction-corrected test. Flagged the difference from the manual
bench's speed history: every manual-jog speed increase (50->250->500us) was
made with Jeremy physically present, pressing keys, watching each stage in
real time — the vision-follow speed has no such staged history yet (it's
only ever run once, and that run also had an unresolved spurious-disable
issue still being diagnosed). Recommended doubling instead of jumping
straight to the 500us historical ceiling; Jeremy agreed ("那先加速" — do the
smaller step first).

**Changed:** `VisionRotateBenchConstants.h`, `FOLLOW_JOG_OFFSET_US`
150 -> 300 (still `<=500`, `static_assert` ceiling unchanged and still holds).

**Evidence:** native `g++` test updated (pulse assertions 1350/1650 ->
1200/1800, matching `NEUTRAL_US(1500) +- 300` with `INVERTED=true`) and
passes; `node vision_rotate_page_test.cjs` passes unaffected; `avr-g++
-fsyntax-only` on `VisionRotateTest.cpp` clean. Not yet re-uploaded/tested on
real hardware.

**Still open, not addressed in this change:** Jeremy separately reported the
bench fully disables (not just stops moving) when the target reaches screen
center, requiring re-pressing enable. No code path in `VisionRotateBenchState`
sets direction-reaching-zero as a disable trigger, so this needs the actual
`停用原因`/`stopReason` value from `vision_rotate.html` at the moment it
happens (`TIMEOUT`/`VISION_TIMEOUT`/`LIMIT`/`BAD_COMMAND`) before it can be
diagnosed — not yet provided. Do not assume this is fixed or explained by
the speed change.

**Next Test:** re-upload `mega_vision_rotate_test`, re-run the live-target
test. Watch for: (a) whether 300us is still stable/controllable or starts
overshooting/oscillating around center, (b) capture the actual stop reason
the next time the unexplained disable happens.

## 2026-09-30 — Claude Code: added a creep zone to fix center oscillation

Jeremy reported: after vision-follow reaches screen center, it occasionally
oscillates back and forth, and this gets worse the higher `FOLLOW_JOG_OFFSET_US`
is set. Root cause: the controller was pure on/off (bang-bang) — full speed
right up to the deadband edge, then stop — so any real control-loop latency
(waiting for the next Serial1 frame) plus servo momentum causes it to
overshoot past center and correct back the other way. Overshoot distance
scales directly with speed, which matches exactly what was observed.

**Changed:**
- `VisionRotateBenchConstants.h`: added `SLOW_ZONE_PX = 80` and
  `FOLLOW_CREEP_OFFSET_US = 150`. `DEADBAND_PX` (25) and `FOLLOW_JOG_OFFSET_US`
  (300) unchanged.
- `VisionRotateBenchState.h`: added a `speedOffsetUs` member. `onVision()` now
  picks one of three zones from `|tx|`: `<=DEADBAND_PX` -> stop (0),
  `<=SLOW_ZONE_PX` -> creep (`FOLLOW_CREEP_OFFSET_US`), else -> full speed
  (`FOLLOW_JOG_OFFSET_US`). `pulse()` and `stop()` updated to use
  `speedOffsetUs` instead of the flat `FOLLOW_JOG_OFFSET_US` constant.

**Why not a full PID/continuous-proportional controller:** a three-zone
creep step is simpler to reason about and verify (exact expected pulse per
zone, same style as the existing tests) than tuning a continuous gain
constant with no real-hardware data yet on how much creep speed actually
helps. If creep alone isn't enough once Jeremy re-tests, a continuous
proportional term (speed scaling linearly with `|tx|` above the deadband) is
the natural next step, reusing the existing `PidfController` pattern already
used for Shooter.

**Evidence:** native `g++` test updated — two existing deadband-boundary
assertions changed from full-speed to creep-speed pulse values, and a new
block added asserting the creep/full-speed boundary at 80/81px both signs;
passes. `node vision_rotate_page_test.cjs` passes unaffected. `avr-g++
-fsyntax-only` on `VisionRotateTest.cpp` clean. Not yet re-uploaded or
re-tested on real hardware.

**Next Test:** re-upload `mega_vision_rotate_test`, re-run the live-target
follow test. Confirm the servo now visibly slows down as it approaches
center instead of slamming in at full speed, and settles without the
back-and-forth oscillation Jeremy saw before. If it still oscillates, the
creep speed (150) may still be too fast, or the slow zone (80px) too narrow
for this camera's field of view/frame rate — both are provisional, tunable
values pending this real-hardware result.

## 2026-09-30 — Claude Code: reused PositionDecelerationProfile instead of a hand-rolled speed step

Jeremy pointed out the hand-rolled two-speed step added just above duplicates
a design he'd already specified and that already exists in this codebase:
`PositionDecelerationProfile` (used for Shooter elevation's "80/20 position
profile"). Replaced the step function with it.

**Changed:** `VisionRotateBenchState.h` now includes
`PositionDecelerationProfile.h` (same directory, header-only, no build_src_filter
change needed). `onVision()`'s speed selection is now:
```
PositionDecelerationProfile::calculateMaximumCommand(
    SLOW_ZONE_PX, absTx, 1.0, FOLLOW_JOG_OFFSET_US, FOLLOW_CREEP_OFFSET_US)
```
— using `SLOW_ZONE_PX` (80) as the profile's reference distance and
`decelerationFraction=1.0` (ramp spans the whole zone) makes this simplify to
a linear ramp `FOLLOW_JOG_OFFSET_US * |tx| / SLOW_ZONE_PX`, floored at
`FOLLOW_CREEP_OFFSET_US` and capped at `FOLLOW_JOG_OFFSET_US` — full speed
exactly at/beyond 80px, a genuine gradual ramp down through the slow zone
(not a step), floored at the creep speed near the deadband so it doesn't
stall out approaching center. `DEADBAND_PX`/`SLOW_ZONE_PX`/
`FOLLOW_CREEP_OFFSET_US`/`FOLLOW_JOG_OFFSET_US` constants unchanged from the
step-function version.

**Evidence:** native `g++` test rewritten to assert the ramp shape (40px ->
floored at creep, 60px -> mid-ramp 225us, 80px -> full speed, -60px mirrored)
instead of the old two-step boundary; passes. `node
vision_rotate_page_test.cjs` unaffected, passes. `avr-g++ -fsyntax-only`
clean. Not yet re-uploaded or re-tested on real hardware — still the same
provisional 25/80/150/300 constants, only the shape of the curve between them
changed from a step to a ramp.

**Next Test:** same as above — re-upload, re-run the live-target follow
test, confirm smoother deceleration into center and no oscillation.

## 2026-10-05 — Claude Code: tightened deadband and slow zone (follow felt too slow on real hardware)

Jeremy tested the new deceleration ramp and found overall follow speed too
slow. Offered two deadband options (20 or 15); picked the more conservative
(20, smaller step down from the previous 25) consistent with this project's
staged-tuning history, rather than jumping straight to 15.

**Changed:** `VisionRotateBenchConstants.h` — `DEADBAND_PX` 25 -> 20,
`SLOW_ZONE_PX` 80 -> 70. `FOLLOW_CREEP_OFFSET_US` (150) and
`FOLLOW_JOG_OFFSET_US` (300) unchanged — this reaches both the creep floor
and full speed sooner (at a smaller `|tx|`) without changing the speeds
themselves.

**Evidence:** native `g++` test's deadband-boundary and speed-profile
assertions recomputed for the new thresholds (20/21px and 35/49/70px
boundaries) and pass. `node vision_rotate_page_test.cjs` unaffected, passes.
`avr-g++ -fsyntax-only` clean. Not yet re-uploaded or re-tested on real
hardware.

**Next Test:** re-upload, re-run the live-target follow test. If still not
fast enough, the next lever is 15px deadband (the other value Jeremy
offered) or raising `FOLLOW_JOG_OFFSET_US` itself (still below the 500us
manual-jog ceiling) rather than narrowing the zones further — narrowing the
slow zone much more risks bringing back the overshoot/oscillation this ramp
was added to fix in the first place.

## 2026-10-06 — Claude Code: production rotate-axis full-auto/semi-auto/full-manual mode, vision-follow wired into Shooter (NOT hardware-tested, NOT committed)

**Context:** Jeremy's competition sequence — Orin powers up and idles in
standby for a long period before the match; Mega/mechanism main power only
comes on when they take the field. From that moment, the turret must default
to autonomous vision-follow unless the operator explicitly switches modes
with a custom keypad (ELRS -> this robot's existing SBUS receiver). This is
categorically different from every bench safety model built so far this
session, all of which assumed a human watching a browser tab over USB —
there is no laptop on the field. Confirmed with Jeremy before implementing:
(1) the real turret hardware is the same open-loop continuous-rotation servo
as the A5 bench, on the existing production pin (`Rotate::SIGNAL_PIN=29`),
but **no limit switches will be installed**; (2) semi-auto and full-auto are
identical for the rotate axis (both auto-aim) — semi-auto only changes
whether *firing* needs a button press, and that trigger/flywheel logic
doesn't exist yet, so it's out of scope here.

**Explored before writing any code** (two parallel research passes): SBUS's
actual channel usage (`ch0/ch1/ch3` used by Chassis mecanum mixing, `ch2`/
`ch8` computed but never read by anything — safe to repurpose), confirmed
the underlying `bfs::SbusData` struct already carries `lost_frame`/
`failsafe` fields that the wrapper never exposed (no RC-signal-loss
protection existed anywhere in this codebase until now), and Shooter's
existing `RotateControlMode` architecture (`DISABLED`/`MANUAL_OPEN_LOOP`/
`PROFILED_POSITION`/`CLOSED_LOOP`, the latter two require a potentiometer
that `Rotate::POSITION_CALIBRATED=false` already keeps permanently disabled
and that the real mechanism won't have installed anyway).

**Changed/Added:**
- `src/Vision/Vision.h`/`.cpp`, new `src/Constants/VisionConstants.h`
  (replacing the old `VisionConst` namespace, confirmed unused anywhere
  else) — the production Vision receiver was previously a stub (first CSV
  field only, no handshake at all). Replaced wholesale with the Chassis
  branch's already-built, already-reviewed version (`git show
  Chassis:src/Vision/...`), unmodified: `MEGA_READY`/`MEGA_HEARTBEAT`
  handshake, full 5-field `tx,ty,distance,target_id,valid` parse with strict
  field-end/range validation, Orin lifecycle tracking
  (`VISION_STANDBY`/`STARTING`/`READY`/`ERROR`), `isConnected()`/
  `isVisionReady()`/`getPacketAgeMs()` for link-health checks. This was the
  long-deferred "merge should match Chassis's packet format" decision from
  2026-09-21/2026-09-25 — finally has a real caller now.
- `src/IO/SBUS.h`/`.cpp` — added `SBUS::signalLost` (now-exposed
  `lost_frame`/`failsafe` from the library, plus a 200ms no-new-frame
  timeout as a second independent check) and `SBUS::modeChannel`
  (provisionally aliases `ch8`; re-point once the real keypad exists and its
  actual channel is known).
- New `src/IO/OperatorMode.h`/`.cpp` — `enum class OperatorMode { FULL_AUTO,
  SEMI_AUTO, FULL_MANUAL }`, `OperatorModeSelector::current()` thresholds
  `SBUS::modeChannel` into thirds (provisional, needs re-measuring against
  the real keypad) and **always returns `FULL_MANUAL` when `SBUS::
  signalLost`** — losing the RC link must never leave an axis armed for
  autonomous motion.
- `src/Constants/ShooterConstants.h`, `Rotate` namespace — added
  `LIMIT_SWITCHES_INSTALLED = false` (see below) and the vision-follow
  tuning constants `VISION_DEADBAND_PX=20`/`VISION_SLOW_ZONE_PX=70`/
  `VISION_CREEP_COMMAND=0.3`/`VISION_CRUISE_COMMAND=0.6` — directly carried
  over from the A5 bench's real-hardware-tuned pixel thresholds (20/70px,
  2026-10-05), with cruise/creep normalized from the bench's 300us/150us
  using the same 500us-authority ratio. Explicitly commented that these are
  unverified on the actual turret mechanism and must be re-checked.
- `src/Shooter/Shooter.h`/`.cpp` — `RotateControlMode` gained
  `VISION_FOLLOW`; new `Shooter::setRotateVisionFollow()`.
  `updateRotate()`'s new branch does not touch the potentiometer at all
  (none installed) — same deadband -> `PositionDecelerationProfile`-ramped
  creep -> cruise shape already proven on the bench, reading `Vision::
  getTx()`/`isValid()` directly, output through the existing
  `writeRotateCommand()` (so `DirectionalLimit`/limit-switch handling stays
  identical to every other mode). **Sign is unverified** — tx>0 maps to a
  positive command with no inversion baked into the vision-follow code
  itself; if it's backwards on first power-up (as it was on the bench
  before `INVERTED` was flipped there), the fix is flipping the *existing*
  `ShooterConstants::Rotate::INVERTED` flag, not adding a second inversion
  here.
- **Safety fix, not just a feature addition:** `isRotateLeftLimitTriggered()`/
  `isRotateRightLimitTriggered()` now return `false` unconditionally when
  `Rotate::LIMIT_SWITCHES_INSTALLED` is false. Found while implementing
  this: `ShooterConstants::Rotate`'s limit-switch config already has
  `LIMIT_USE_INTERNAL_PULLUP = false` (comment: "Legacy all_robot_017
  wiring"), so with no switches installed those pins would float and read
  an unpredictable value — worse than the bench's deliberate
  pulled-high-when-disconnected fail-safe design. This affects *all* Rotate
  modes, not just vision-follow, since the mechanism will never have
  switches installed regardless of mode.
- `src/main.cpp` — after `SBUS::update()`/`Vision::update()`, arbitrates:
  `FULL_MANUAL` or an unhealthy vision link (`!isConnected() ||
  !isVisionReady()`) -> `Shooter::disableRotate()`; otherwise ->
  `Shooter::setRotateVisionFollow()`. **Deviation from the original
  plan worth flagging**: the plan said "reuse the existing manual joystick
  path" for full-manual — there isn't one. No SBUS channel is wired to
  Rotate's `MANUAL_OPEN_LOOP` anywhere in this codebase (the keypad that
  would drive it doesn't exist yet). Defaulted to `disableRotate()` (safe,
  does nothing) rather than inventing an unreviewed channel mapping for
  manual rotate control — that's a separate decision for whenever the
  keypad's full channel layout is actually known.
- `platformio.ini` — `mega_shooter_keyboard_test` now also builds
  `+<Vision/>` / `-I src/Vision`. Without this it would have failed to link
  once `Shooter.cpp` started including `Vision.h` — caught and fixed via
  compile verification before considering this done, see Evidence.

**Explicitly deferred (per Jeremy's own scoping):** flywheel/firing
auto-trigger for semi-auto ("confirm before firing") — flywheel has zero
trigger mechanism today, independent future feature. Angle (elevation) axis
— untouched. Potentiometer-based Rotate closed loop — not applicable, no
pot on the real mechanism. Real keypad channel/threshold calibration —
`SBUS::modeChannel`/`OperatorModeConstants` thresholds are placeholders
pending the keypad actually being built.

**Evidence:** `avr-g++ -fsyntax-only` against the full `megaatmega2560`
production environment (`main.cpp` pulling in every subsystem, plus each
changed `.cpp` compiled individually) — clean, including the previously-weird
case-insensitive-filesystem false error (`<sbus.h>` angle-include resolving
to `src/IO/SBUS.h` instead of the real library when `-I src/IO` was listed
before `-I lib/sbus/src`; fixed by reordering the manual verification
command only, not a real code issue). Re-ran the full existing native test
suite (`directional_limit_test`, `pidf_controller_test`,
`position_deceleration_profile_test`, `rotate_bench_state_test`,
`vision_rotate_bench_state_test`) plus both `.cjs` page tests — all still
pass, nothing regressed. Confirmed `mega_shooter_keyboard_test`,
`mega_rotate_only_test`, and `mega_vision_rotate_test` bench environments
are unaffected (the latter two fully replace their `build_src_filter`, never
touch `Shooter.cpp`; the first needed the `platformio.ini` fix above, now
verified). **No `pio` build/upload, no motor power, no real SBUS/ELRS
signal, no real Orin link tested against this code at all — this is static
verification only.** Not committed.

**Next Test — do not skip straight to "trust auto-arm on power-up":**
matches the plan's own staged-verification section. First, motor power
disconnected (logic/SBUS/Vision wiring only): confirm telemetry
(`Shooter::getRotateControlMode()`, and whatever gets exposed for
`OperatorModeSelector::current()`/`SBUS::signalLost`) behaves correctly —
normal SBUS -> `FULL_AUTO`, RC unplugged -> flips to `FULL_MANUAL`
automatically, Orin/vision link down -> rotate axis disables automatically.
Only after that logic is confirmed correct on real hardware should motor
power be reconnected and actual vision-follow be tested on the pin-29
mechanism, with someone able to cut power immediately — direction sign and
the carried-over bench speed constants have never been run on this
mechanism before.

## 2026-10-06 — Claude Code: cross-branch integration (merge/refactor complete, Shooter -> dev -> main, Vision synced to Chassis/Dribbler/ShooterMG996)

Jeremy asked to integrate today's vision-follow/mode-switching work across
the whole project and clean up the now-redundant bench test files, rather
than leaving it isolated on `Shooter`. Explicitly confirmed scope before
doing anything irreversible: `dev`/`main` get the full `Shooter` branch
merged in as the new baseline; `Chassis`/`Dribbler`/`ShooterMG996` only get
their `Vision` module synced to the one canonical version (their own
subsystem code stays untouched). Also confirmed: do this now, even though
today's vision-follow/mode work is still only statically verified, not
hardware-tested — real-hardware verification happens after, on whichever
branch.

**Branches touched, in order:**

1. **`Shooter`** (committed `9a38c71`, pushed): bundled the production
   auto/semi-auto/manual + vision-follow work from earlier today with a
   cleanup pass — removed `rotate.html`/`mega_rotate_only_test` (manual-jog
   bench, superseded by production `VISION_FOLLOW`) and
   `UART_BARE_TEST.md`/`mega_uart_bare_test` (one-time diagnostic, issue
   already fixed), stripped the temporary `RX1:` debug echo out of
   `VisionRotateTest.cpp`, inlined `RotateBenchConstants.h`'s few needed
   values directly into `VisionRotateBenchConstants.h` so the surviving
   vision-follow bench no longer depends on a file that no longer exists,
   rewrote `tools/shooter_keyboard_test/README.md` (was describing the now
   deleted bench in detail). Kept `ShooterKeyboardTest.cpp`/`index.html`
   (Angle/Flywheel bench — unrelated to vision, still the only way to bench
   those axes independently) and `vision_rotate.html`/
   `mega_vision_rotate_test` (the one surviving standalone bench). Verified
   full production env + both surviving bench envs compile, full native/page
   test suite passes.

2. **`Chassis`**: no change needed — its `Vision.{h,cpp}`/`VisionConstants.h`
   were already byte-identical to the canonical version (this is in fact
   where the canonical version originally came from, back on 2026-09-24).

3. **`Dribbler`** (committed `d64b1b5`, pushed): its own
   `Vision.{h,cpp}`/`VisionConstants.h` turned out to already be a
   functionally-complete, independently-built equivalent (full handshake,
   same public API) — just under its own flat `VisionConst` namespace
   instead of the canonical nested `VisionConstants::Transport/Validation`.
   Swapped for the canonical version; its own `Shooter.cpp` (which already
   called `isVisionReady()`/`isValid()`/`getXPred()`) needed no changes.
   Nothing else on this branch touched — its own more-developed `SBUS`
   wrapper (`isHealthy()`/`getDriveForward()`/`getDriveTurn()`/dedicated
   `SBUSConstants.h`) and PID_v1-based `Shooter.cpp` (which already has a
   *working* fire-trigger: flywheel gated on
   `Vision::isValid() && readyH && readyV && Dribbler::getShootRemaining() > 0`)
   are untouched and still there.

4. **`ShooterMG996`** (committed `cf8b88f`, pushed): its `Vision.{h,cpp}`
   was the older first-field-style stub (loose `.toFloat()`/`.toInt()`
   parsing, no handshake). Swapped for the canonical version — purely
   additive from this branch's own call sites (`Shooter.cpp`/`Telemetry.cpp`
   only ever called the subset of methods the stub already had).

5. **`dev`** (merge commit `5ad3a9d`, pushed): `git merge Shooter`, real
   conflicts in `platformio.ini`, `Constants/Pins.h`, `ShooterConstants.h`,
   `Shooter.{h,cpp}`, `Vision.{h,cpp}`/`VisionConstants.h`, `main.cpp` —
   resolved file-by-file on their merits, not a blanket "ours" or "theirs":
   - Vision/Shooter/ShooterConstants: took `Shooter`'s side (the new
     architecture supersedes dev's own simpler PID_v1-style Shooter, which
     turned out to be the same family as Dribbler/ShooterMG996's old
     versions).
   - `Pins.h`: kept **dev's** side instead (its own `#pragma once`
     modernization; `Shooter`'s copy still used the old `#ifndef` guard,
     content otherwise identical).
   - `main.cpp`: hand-merged — kept dev's `Telemetry::init()/update()` calls
     and its safer `Dribbler::setShootRequest(0)` ("disarmed until
     confirmation flow enables it" — dev's own explicit comment/intent, kept
     over `Shooter`'s unexplained `setShootRequest(3)`), added `Shooter`'s
     new `OperatorMode`-gated `setRotateVisionFollow()`/`disableRotate()`
     call.
   - `platformio.ini`: combined both sides' `-I` paths and bench env
     sections; added the `-<Bench/>` source-filter exclusion to the main env
     (dev never needed it before since it had no `Bench/` folder until this
     merge).
   - **Real capability lost in this merge, flagged for follow-up**: dev's
     old `Shooter.cpp` had a *working* fire-trigger (same condition as
     Dribbler's: `Vision::isValid() && readyH && readyV &&
     Dribbler::getShootRemaining() > 0` → fires the flywheel). The new
     `PidfController`-based architecture has no equivalent yet — flywheel
     firing/trigger logic is still completely unbuilt on it. This needs to
     be redesigned on top of the new architecture when the semi-auto
     fire-confirmation flow actually gets built (see the 2026-10-06 entry
     above — explicitly out of scope for that work).
   - Verified: full compile of dev's own `main.cpp` (pulls in `Telemetry`,
     `Chassis`, `Dribbler`, `Sensors`, `Control`), both bench envs, and the
     complete native/page test suite — all pass.

6. **`main`** (merge commit `3c21b98`, pushed): confirmed `main` was a pure
   ancestor of `dev` (zero unique commits) before touching it — so this was
   `git merge dev` with **zero conflicts**, not a repeat of the `dev`
   conflict resolution. Brought along dev's own unique work too
   (`tools/dashboard/`, `docs/autonomous-zone-strategy.md`,
   `src/Constants/Mode.h`) that `main` didn't have yet. Verified the same
   full compile check passes.

**Not touched**: `STM32-ELRS` branch — outside the scope Jeremy specified
(`Chassis`/`Dribbler`/`ShooterMG996` only for the Vision sync).

**Still true, unchanged by any of this**: none of today's
auto/semi-auto/manual + vision-follow logic has been hardware-tested on any
branch. The staged verification plan from the earlier 2026-10-06 entry
still applies, now on whichever branch(es) Jeremy actually flashes.

## 2026-10-06 — Claude Code: auto-fire sequence (lock → spin-up → feed → fire), Shooter branch only

Jeremy described the full desired flow: camera locks/follows target → elevation
angle and flywheel speed both ready → Dribbler feeds a ball → fire. Full-auto
does all of this automatically; semi-auto does everything except the final
fire step automatically, which needs a button press.

**Investigated before writing code** and found three real hardware/feedback
gaps, not just missing software — confirmed with Jeremy how to handle each:
1. **Elevation (Angle) closed-loop can never arm right now** —
   `ShooterConstants::Angle::POSITION_LIMITS_CALIBRATED` is hardcoded `false`,
   so `setAngleTargetCounts()` always fails into `disableAngle()`. Jeremy:
   mechanical limits haven't been measured yet, will be done later. **This
   fire sequence deliberately does not gate on angle readiness** — wire
   `isAngleReady()` into `Auto::update()`'s `canAutoFire`/state-advance
   checks once that calibration exists.
2. **Flywheel (Falcon 500) has zero speed feedback** — confirmed via earlier
   handoff entries and a fresh grep (no RPM/tachometer anywhere in `src/`):
   the PWM command path is one-way. Jeremy: use a time-based proxy for now
   (commanded at shoot-speed for long enough == probably spun up).
3. **Dribbler's ball sensor is non-functional as currently wired** —
   `PIN_DRIBBLE_DOWN` (pin 51) is set `OUTPUT` in `init()` but also
   `digitalRead()` in `update()`, so it reads back whatever it just wrote,
   not a real ball-presence signal. Not fixed here (separate wiring issue,
   predates this work) — "ball fed" is also approximated by time, same
   reasoning as the flywheel.

**Added/changed:**
- `src/Shooter/Shooter.h`/`.cpp`:
  - `updateRotate()`'s `VISION_FOLLOW` branch: `rotateReady` used to be
    hardcoded `false`. Now set properly — reuses the exact same settle-time
    pattern `PROFILED_POSITION`/`CLOSED_LOOP` already use
    (`Rotate::READY_SETTLE_TIME_MS`): must stay inside
    `Rotate::VISION_DEADBAND_PX` for the settle time, not just touch it for
    one tick, before `isRotateReady()` reports locked. No new getter needed
    — `isRotateReady()` already existed and is reused as-is.
  - New `isFlywheelReady()` + private `flywheelSpinStartMs`/`flywheelReady`:
    true once the commanded flywheel output has stayed at/above
    `Flywheel::SHOOT_COMMAND` for `Flywheel::SPIN_UP_MS`. Reset in
    `stopAll()` too (the early-return path in `update()` when
    `!outputsEnabled` would otherwise skip the normal reset).
  - `Shooter::isReady()` deliberately left untouched (still hardcoded
    `false` with its existing comment) — its intended meaning is "fully
    ready including angle," which still isn't achievable. `Auto` checks
    `isRotateReady() && isFlywheelReady()` directly instead, explicitly
    excluding angle, with a comment explaining why.
- `src/Constants/ShooterConstants.h`, `Flywheel` namespace: new
  `SHOOT_COMMAND` (0.8, provisional open-loop spin command) and
  `SPIN_UP_MS` (1500, provisional time-based ready threshold) — both
  commented as unverified placeholders pending real feedback/real shots.
- `src/Dribbler/Dribbler.h`/`.cpp`: new `setFeedAllowed(bool)` + private
  `feedAllowed` flag. `update()`'s existing `if (shoot_pice > 0) run(); else
  stop();` became `if (feedAllowed && shoot_pice > 0) ...` — the existing
  (imperfect) ball-count logic is completely untouched; this only adds an
  external gate, because without it Dribbler would start feeding the moment
  anyone called `setShootRequest()`, regardless of whether Shooter was even
  aimed yet.
- `src/IO/SBUS.h`/`.cpp`: new `fireChannel` (provisionally aliases `ch2`,
  same pattern as `modeChannel` aliasing `ch8` — both placeholders pending
  the real keypad).
- New `src/Constants/AutoConstants.h`: `FEED_DURATION_MS` (500),
  `COOLDOWN_MS` (300), `FIRE_BUTTON_THRESHOLD` (1500) — all provisional.
- New `src/Auto/Auto.h`/`.cpp` (was a completely empty placeholder file —
  first real use of it): state machine `IDLE -> SPINNING_UP ->
  READY_TO_FIRE -> FEEDING -> COOLDOWN -> IDLE`. Also absorbed the
  `OperatorMode`-driven Rotate arbitration that used to live directly in
  `main.cpp`'s `loop()` (moved here so all cross-subsystem mode decisions
  live in one place) — full-auto/semi-auto both call
  `Shooter::setRotateVisionFollow()`; full-manual or unhealthy vision calls
  `Shooter::disableRotate()`, same logic as before, just relocated.
  `READY_TO_FIRE` auto-advances in `FULL_AUTO`; in `SEMI_AUTO` it waits for
  an edge-triggered `SBUS::fireChannel` press. Losing rotate-lock or
  flywheel-ready while at `READY_TO_FIRE` drops back to `SPINNING_UP`.
- `src/main.cpp`: calls `Auto::init()` in `setup()`, `Auto::update()` in
  `loop()` (replacing the inline mode-arbitration block that moved into
  `Auto.cpp`). Also changed `Dribbler::setShootRequest(3)` →
  `setShootRequest(0)` at boot — now that `Auto::update()` will auto-fire
  whenever a shot is queued and the axis is locked+spun-up, booting with 3
  pre-queued would auto-fire three times with zero operator action.

**Evidence:** `avr-g++ -fsyntax-only` against the full production
`megaatmega2560` build (`main.cpp` pulling in every subsystem) and each
changed/new `.cpp` individually — clean. `mega_shooter_keyboard_test` bench
env re-verified (touches `Shooter.cpp`). Full existing native test suite
(`directional_limit`/`pidf_controller`/`position_deceleration_profile`/
`vision_rotate_bench_state`) and both `.cjs` page tests re-run, all pass.
No native test added for `Auto` itself/the state machine — consistent with
this codebase's existing pattern, `Shooter`/`Dribbler`/`SBUS` are tightly
coupled to Arduino statics and have never had host-side unit tests either,
only compile verification. **No `pio` build/upload, no motor power, no real
SBUS/fire-button/Orin link tested. This has not been merged to
`dev`/`main`/`Chassis`/`Dribbler`/`ShooterMG996` yet** — stayed on `Shooter`
only this time; ask explicitly before repeating the earlier cross-branch
integration for this feature.

**Next Test (staged, same discipline as every prior step):** power off
motors first. Confirm via telemetry that `Auto`'s fire-state machine
transitions correctly with `Dribbler::setShootRequest()` set to some count:
does it sit in IDLE with 0 queued, enter SPINNING_UP once a shot is queued
and vision/mode allow it, reach READY_TO_FIRE only after real
`isRotateReady()`/`isFlywheelReady()` (watch these over serial — do they
flip at sane times, not instantly/never), and in semi-auto does it correctly
wait at READY_TO_FIRE until `fireChannel` is pressed. Only once that's all
confirmed sane should motor power be reconnected, with someone able to cut
power immediately — the flywheel command magnitude, spin-up time, and feed
duration are all unverified guesses right now.

## 2026-10-11 — Claude Code: wired dev integration (Chassis/Dribbler merged into dev, new WiredControl USB Dashboard protocol, wireless/final pins deferred)

**User Request:** Codex's 2026-10-11 handoff (`inbox/claude-codex-message.md`)
asked for `dev` to become the official competition architecture by merging
`Shooter`/`Chassis`/`Dribbler`'s bench work in, with a USB-direct PC
Dashboard keyboard layer (W/S/A/D chassis, R/F feed, arrow-key
Rotate/Angle jog, L manual/vision-follow toggle, Enter/Space semantics),
listing deletion candidates before deleting, and explicitly deferring
wireless (dual-STM32/ELRS) and final pin assignments. Jeremy scoped this
round to wired-only ("先做有線的部分，無線跟 pin 配置等你明天給資料"),
confirmed the real ball sensor is a break-beam matching `Dribbler`
branch's `ExitSensor` design, and after reviewing the feed-mechanism test
video (`2u4uYxGWIXyhUO-BA1XwYuZ36QUG2A_GpggLlCAceVY.MP4`, 6 motors — 2
horizontal collector + 4 vertical pinch-feed, all one logical group)
confirmed F forcibly interrupts a running R (switches to single-shot, does
not resume R afterward) with a 3-second jam timeout as a starting value.

**Discussion Result:** Full plan written and approved via plan mode
(`/Users/jeremy/.claude/plans/optimized-foraging-bumblebee.md`) before any
file was touched. Not a wholesale branch-file copy — `dev`'s own
`Chassis.cpp`/`Dribbler.cpp` were still the pre-merge versions (four-wheel
mecanum mixing; the pin-51-double-duty sensor/output bug), and two
additional pre-existing problems were found during this integration that
had nothing to do with any branch merge: `Telemetry.cpp` referenced a
deleted `SBUS::ch0..ch8` (the file would not compile at all), and
`src/IO/MechLink.h` used `String` without including `<Arduino.h>` itself
(silently relied on whichever `.cpp` happened to include it first in some
other order — `OperatorMode.cpp`/`MechLink.cpp` both failed to compile
standalone). Both are fixed as part of this integration; `pio run -e
megaatmega2560` had apparently never actually been run to completion
before this, only bench environments.

**Why:** `dev` needs to be a real, buildable competition baseline, not a
loose collection of divergent branch snapshots, and the Dashboard needed
an actual command-sending capability (it was read-only telemetry before —
`port.readable.pipeTo(...)` with no `getWriter()` anywhere) before any
wired keyboard layer could exist at all.

**Changed:**
- `src/Chassis/Chassis.{h,cpp}`: replaced with the `Chassis` branch's
  two-wheel differential drive (`setOpenLoop`/`setTankOpenLoop`/
  `setOutputsEnabled`/`stop()`), dropping the old four-servo
  `vx/vy/w` mecanum mixing that read `MechLink` channels directly. New
  `src/Chassis/DifferentialDriveMixer.h` ported alongside it (used by
  `setOpenLoop`, not by the new wired tank command, which calls
  `setTankOpenLoop` directly to keep A/D's sign unambiguous — see below).
- `src/Dribbler/Dribbler.{h,cpp}`: replaced the pin-51-double-duty version
  with the `Dribbler` branch's `ExitSensor`(pin 45, active-low, 8ms
  debounce)/`Feeder`/`DribblerControlMode` design, **plus** two additions
  the branch didn't have: (1) `setFeedAllowed(bool)` restored/kept so
  `Auto.cpp`'s existing auto-fire permission-gate call sites don't change,
  gating only `COUNTED_FEED` (not `MANUAL_OPEN_LOOP`, which wired R uses
  and isn't gated by Auto at all); (2) a 3-second jam timeout
  (`DribblerConstants::Jam::TIMEOUT_MS`) — `COUNTED_FEED` mode had no
  timeout at all before, a stuck/missing sensor would have run the feed
  motors forever. `Feeder` now fans a single command out to
  `Feeder::CHANNEL_COUNT` PWM/enable pin pairs instead of assuming exactly
  one motor (see "Not yet done" below — still only 1 channel wired today).
- `src/Telemetry/Telemetry.cpp`: fixed the dead `SBUS::ch0..ch8`
  references to `MechLink::ch0/ch1/modeChannel/ch3/fireChannel` (repurposing
  the old ch2/ch8 slots since MechLink doesn't expose a literal ch2/ch8),
  appended new trailing fields (wired-armed, chassis L/R pulse-us, dribbler
  mode/feeding/exit-sensor, Shooter rotate/angle mode, Falcon pulse-us +
  **command ratio** — deliberately not RPM, this Falcon has no speed
  feedback on this PWM path). `tools/dashboard/index.html`'s
  `parseTelemetry()` treats all new fields as optional (`p.length > N`),
  same pattern the existing robotX/robotY/heading fields already used, so
  older/partial telemetry doesn't break the page.
- `src/IO/MechLink.h`: added the missing `#include <Arduino.h>` (pre-existing
  bug, see above).
- `src/Auto/Auto.cpp`: two small, explicit, well-contained changes — (1)
  `WiredControl::isRotateAuthorityActive()` skips Auto's own
  disableRotate()/setRotateVisionFollow() arbitration for one tick when
  the wired Dashboard is armed, since `WiredControl` now calls those exact
  same `Shooter` methods itself based on its own L toggle + vision health;
  (2) `WiredControl::isDribblerAuthorityActive()` skips Auto's
  `Dribbler::setFeedAllowed(false)` in the `!canAutoFire` branch while a
  wired F request is in flight — otherwise a wired F could never actually
  run the feed motors, since `canAutoFire` is always false with no RC
  signal present (the normal case during wired-only testing), and Auto was
  forcing that gate false every single tick. **The auto-fire state machine
  itself (`FireState`/`canAutoFire`) is completely untouched** — `mode`
  stays `FULL_MANUAL` with no RC link regardless of anything wired control
  does, so auto-fire structurally cannot trigger from the wired Dashboard,
  satisfying Codex's "L switching != starting auto-fire."
- `src/Shooter/Shooter.{h,cpp}`: added `getFlywheelCommand()` (returns the
  raw 0.0-1.0 open-loop command) purely for the Dashboard's honest
  us+ratio Falcon display.
- `src/Constants/Pins.h`: removed the stale Chassis/Dribbler `#define`s
  (pins now live in `ChassisConstants.h`/`DribblerConstants.h`, following
  those branches' own convention); `src/Constants/TuningConstants.h`
  gained the `Chassis` branch's `RightDrive`/`LeftDrive`/`Manual`
  namespaces (the file's own header comment already anticipated this
  exact merge-by-combining-namespace-blocks).
- `src/main.cpp`: added `WiredControl::init()`/`WiredControl::update()`,
  the latter called right after `MechLink::update()`/`Vision::update()`
  and before `Auto::update()`, so WiredControl's authority flags and its
  own direct `Chassis`/`Dribbler`/`Shooter` calls are current before Auto
  arbitrates and before each subsystem's own `update()` applies them.

**Added:**
- `src/IO/WiredControl.{h,cpp}` + `src/Constants/WiredControlConstants.h`:
  new production module, USB `Serial` (separate hardware port from
  `Serial2`/`MechLink`'s wireless link — nothing shared, no risk of the two
  fighting over the same UART). Line-based protocol, same buffered-byte-
  read/`$EVENT,`-reply shape every existing bench (`ShooterKeyboardTest`/
  `ChassisKeyboardTest`/`VisionRotateTest`) already uses:
  `E`/`X`/`K` (enable/disable+clear-pending/keepalive), `T,<left>,<right>`
  (tank drive, -1..1 each), `ROT,<dir>`/`ANG,<dir>` (Rotate/Angle manual
  jog, -1/0/1), `R` (toggle continuous feed — `MANUAL_OPEN_LOOP`, not
  gated by Auto), `F` (single-ball request — `COUNTED_FEED`, forcibly
  clears R first), `L` (toggle Rotate manual-jog vs vision-follow, clears
  any pending manual jog, defaults to **manual** on boot/enable — not
  vision-follow, since vision-follow would otherwise start driving the
  turret the instant outputs are enabled with no explicit per-axis
  operator action). 250ms command watchdog (`WiredControlConstants::
  COMMAND_TIMEOUT_MS`, matching the existing bench convention) auto-
  disables everything on silence.
- `tools/dashboard/index.html`: was pure telemetry-receive-only before
  (`port.readable.pipeTo(...)`, no `getWriter()` anywhere, no keyboard
  handling at all). Added a `writer`, an Enable button (disabled until
  connected, like every bench page), full W/S/A/D/arrows/R/F/L/Enter/Space
  handling with `event.repeat` suppression on single-press keys, an input-
  focus guard (`target.tagName === 'INPUT'/'TEXTAREA'`/`isContentEditable`
  → ignore — this specific guard didn't exist on any existing bench page,
  borrowed from `docs/elrs-architecture/system.html`'s own convention for
  it since the Dashboard, unlike the benches, has real text inputs on the
  same page), `window blur`/`visibilitychange` → stop, `navigator.serial`
  `disconnect` event → clean teardown, a 75ms heartbeat that resends
  whichever axes are non-neutral (not just a bare keepalive, so a dropped
  packet can't leave a motor stuck), and a new Control panel showing
  chassis pulse-us, dribbler mode/feeding/exit-sensor, Rotate/Angle mode,
  and Falcon us+ratio.
- `test/dashboard_control_test.cjs`: host test for the three pure command
  functions (`tankCommand`/`rotateCommand`/`angleCommand`), extracted by
  source text and run in a `vm` context rather than loading the whole
  1600-line page (most of it is canvas/localStorage/drag-resize layout
  code that runs at top-level load and isn't worth mocking for what this
  round needs verified) — covers neutral, W/S/A/D individually (**A = left
  wheel forward + right wheel backward**, Jeremy's explicit non-standard
  polarity, asserted exactly so it can't silently drift), combined W+A,
  opposing-key cancellation, and both Rotate/Angle axes.

**Pending pin table (explicitly NOT final, per Jeremy — real pins come
separately):**
| Subsystem | Pin(s) used this round | Source |
|---|---|---|
| Chassis right/left drive | 39 / 38 | `Chassis` branch (reused dev's old FR/FL pins) |
| Chassis pins 40/41 | unused | old BR/BL, free since this is 2-wheel not 4-wheel |
| Dribbler ExitSensor | 45 | `Dribbler` branch |
| Dribbler Feeder (6 real motors, 1 channel wired today) | PWM 2 / enable 51 | single-motor bench pins, `Feeder::CHANNEL_COUNT` ready to grow |

**Cleanup candidates presented, none deleted yet** (per Codex's explicit
"list before deleting" requirement) — old 4-wheel `Chassis.cpp`/pin-51-
double-duty `Dribbler.cpp` logic is gone (replaced in place, not left as
dead code); three divergent `tools/dashboard/index.html` copies across
`dev`/`Shooter`/`Chassis` collapse to keeping only `dev`'s (now the
official one); `tools/chassis_keyboard_test/`/`src/Bench/
ChassisKeyboardTest.cpp` deliberately kept this round as a protocol
reference/regression check, not deleted.

**Evidence:**
- `platformio run -e megaatmega2560` (real build+link, not syntax-only —
  a working `pio` binary was found at
  `~/.platformio/penv/bin/platformio`): **SUCCESS**, RAM 24.2%
  (1986/8192B), Flash 10.5% (26718/253952B). This is the first time this
  exact environment is known to have been built to completion; it was
  broken (the `SBUS`/`MechLink.h` issues above) before this session even
  started touching it.
- `mega_shooter_keyboard_test` and `mega_vision_rotate_test` bench
  environments re-built clean (both pull in `Shooter.cpp`, which changed).
- Full existing native suite re-run and passing: `pidf_controller_test`,
  `directional_limit_test`, `position_deceleration_profile_test`,
  `shot_table_lookup_test`, `vision_rotate_bench_state_test`,
  `vision_rotate_page_test.cjs`.
- New `dashboard_control_test.cjs`: passing.
- `node --check` on the extracted dashboard `<script>` block: clean.

**Not yet done / hardware-unverified (explicit, not glossed over):**
- **No `pio upload`, no power, no real hardware run of any of this.**
  Jeremy is testing today (10/11) with a real USB-connected Mega.
- No host test exists for `Dribbler.cpp`'s jam-timeout/sensor logic or
  `WiredControl.cpp`'s command parsing/arbitration themselves — both are
  tightly coupled to Arduino statics (`millis()`/`digitalRead`/
  `digitalWrite`/`Serial`) the same way `Shooter.cpp`/the pre-merge
  `Dribbler.cpp` always were, and neither ever had host tests on their
  branch of origin either. Verified by compile+link success and manual
  trace only — flagging this honestly rather than claiming coverage that
  doesn't exist.
- A/D chassis polarity (Jeremy's explicit literal spec: A = left wheel
  forward + right wheel backward) is implemented exactly as specified but
  **not confirmed against real wiring/chassis orientation** — first thing
  to check on hardware, per Jeremy's own instruction not to silently
  invert it if it looks wrong.
- `ExitSensor` active-low assumption carried over from the `Dribbler`
  branch unchanged — not re-verified against the real break-beam's actual
  wiring/polarity.
- Jam timeout (3000ms) is Jeremy's own confirmed starting value, open to
  live tuning.
- Wireless (dual-STM32/ELRS, CRSF channel/event encoding, loss-timeout
  replay handling) and final pin assignments: **explicitly deferred**, not
  attempted this round, per Jeremy's own scoping.
- `Vision::isConnected()`/`isVisionReady()` gating for `L`'s vision-follow
  path reuses the same healthy-check `Auto.cpp` already had — today's
  PIDF vision-follow tuning is separately unfinished (see the 2026-10-10
  entry above), so `L` toggling to vision-follow is only expected to prove
  "the mode switch works safely," not "tracking is good."

## 2026-10-11 — Claude Code: fixes for Codex review of e4a70a5 (wired path not yet accepted)

**User Request:** Codex reviewed `e4a70a5` and did not pass it (four P1,
one P2, several secondary items — see `inbox/claude-codex-message.md`
"Codex review e4a70a5"). Jeremy approved the fix plan, including the
pin-table Enable gate (and accepted that today's bench test needs the pin
table confirmed before Enable will work).

**Discussion Result:** Every Codex finding was checked against the code
before changing anything and all were real. One extra finding while
checking: `Shooter::setOutputsEnabled(true)` was never called anywhere in
production, not just in the wired path — Auto's own vision-follow/flywheel
writes had also never actually reached the outputs.

**Changed:**
- `src/IO/WiredControl.{h,cpp}` (rewritten around these rules):
  - Firmware-side refusal: `R/F/L/T/ROT/ANG/FLY` while disarmed →
    `$EVENT,REJECTED_DISARMED`, no state change. Disarmed `update()` also
    stops any stray `MANUAL_OPEN_LOOP` feed (defense in depth). Fixes
    Codex's repro (X then R → feed ran for 4s+ while disarmed).
  - Enable/disarm now drive every subsystem: `Chassis` and
    `Shooter::setOutputsEnabled(true/false)` + `Shooter::stopAll()`,
    `Dribbler::stop()`, feed gate false, all commands cleared, L back to
    manual. One `disarm(reason)` path is shared by X, watchdog, and RC
    takeover.
  - Enable gate: `E` is refused (`ENABLE_REJECTED_PINS_UNCONFIRMED`) while
    `PIN_TABLE_CONFIRMED` (new, `src/Constants/Pins.h`) is false.
  - Control source: `E` is refused while `MechLink` reports a live link
    (`ENABLE_REJECTED_RC_LINK_ACTIVE`); an armed session disarms itself the
    moment one appears (`DISABLED_RC_LINK_ACTIVE`). Wired and wireless never
    drive outputs at the same time.
  - Watchdog is checked **before** reading new bytes, so a heartbeat that
    only arrives after the 250ms deadline can't rescue an expired session.
  - F while a previous F is in flight → `FEED_BUSY`, no count/jam-clock
    reset, no queueing. R still cancels a pending F (last explicit feed
    command wins); F still interrupts R and does not resume it.
  - New `FLY,<0..1>` flywheel command (open-loop ratio, not RPM), applied
    every armed tick, zeroed by every disarm path.
  - Keyboard axes are scaled for first bring-up
    (`WiredControlConstants::CHASSIS/ROTATE/ANGLE_MANUAL_SCALE` = 0.30 /
    0.10 / 0.10; rotate/angle match `ShooterBenchConstants`' existing
    "limited authority" 0.10, chassis has no prior bench value).
  - `isRotateAuthorityActive()`/`isDribblerAuthorityActive()` replaced by a
    single `isActive()`.
- `src/Auto/Auto.cpp`: the two per-feature skips from `e4a70a5` are gone;
  `Auto::update()` now returns at the top (fire state → IDLE) whenever
  `WiredControl::isActive()`. Since wired can only be armed with no RC
  link, auto-fire couldn't run then anyway; this just stops Auto from
  writing Rotate/flywheel/feed-gate over wired commands.
- `src/Dribbler/Dribbler.cpp`: `setShootRequest()` seeds
  `ballSeenDuringRequest` from the debounced exit-sensor state instead of
  always `false`, so a ball already in the beam when F is pressed counts
  when it leaves (previously the feed ran on to a second ball or the 3s jam
  timeout).
- `tools/dashboard/index.html`: Falcon range slider → `FLY,` (disabled until
  armed, reset to 0 on every disarm/event, resent by the heartbeat while
  non-zero); keyup always releases a held key regardless of focus, and
  `focusin` on a text box releases everything held; only real text inputs
  count as typing targets (the range slider doesn't, so Enter/Space still
  work with it focused); Enable/Connect buttons blur after click so Space
  can't re-activate them; `emergencyStop()` drops local armed state before
  sending `X`; handles the new firmware events; dimmed `:disabled` style.

**Added:**
- `test/wired_control_test.cpp` + `test/host_stubs/` (fake Arduino
  time/pins/Serial, recording Chassis/Shooter/Vision/MechLink): compiles the
  **real** `WiredControl.cpp` and `Dribbler.cpp`. Built twice — with
  `-DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=true` (15 armed-path scenarios) and
  without (gate refuses Enable). The override is a hard `#error` in any
  `ARDUINO` build (verified). Scenarios: everything refused while disarmed;
  X-then-R repro; stray manual feed stopped; enable/disable reach every
  subsystem; watchdog timeout; late heartbeat can't rescue; heartbeat keeps
  alive; RC refuses/takes over; axis scaling; FLY range/parsing + bad/
  unknown commands; L toggle + re-enable returns to manual; single shot
  with ball already blocking; single shot from clear beam; jam timeout with
  busy F not restarting the clock; F interrupts R and R does not resume.
  **Mutation-checked:** reverting each fix individually (old
  `ballSeenDuringRequest=false`, no disarmed rejection, watchdog after read,
  no Shooter enable, no FEED_BUSY, no RC takeover) makes the matching test
  fail.
- `test/dashboard_page_test.cjs`: loads the **whole** dashboard script in a
  vm with a permissive DOM stub; covers Enable flow, W/A press+repeat+
  release, R/F/L repeat suppression, arrows, text-box typing ignored, keyup
  after focus moved, focusin release, slider → FLY + heartbeat resend,
  Enter with slider focused, disarmed slider/heartbeat ignored, firmware
  timeout/RC-takeover resets, telemetry resync, blur, Space disconnect.
- `test/README`: exact build/run commands for every host test.

**Evidence:** `platformio run` for `megaatmega2560` (RAM 25.4% 2083/8192,
Flash 11.3% 28768/253952), `mega_shooter_keyboard_test`,
`mega_vision_rotate_test` — all SUCCESS. All host tests above plus the
existing native/page suite pass. Dashboard opened in a browser: control
panel, slider, and new telemetry fields render (fed one synthetic `TEL`
line through the page's own `handleLine`). No upload, no power, no
hardware.

**Not yet done / hardware-unverified:**
- `PIN_TABLE_CONFIRMED` is false, so **wired Enable is refused until the
  real pin table is checked and that constant flipped**.
- The robot-side STM32 bridge (`tools/stm32_elrs_robot_bridge`) sends a
  `CH,...` line to the Mega every period whether or not RF is up, so with
  the bridge plugged into Serial2, `MechLink::signalLost` stays false and
  wired Enable is refused (`ENABLE_REJECTED_RC_LINK_ACTIVE`). For wired
  testing, unplug the bridge from Serial2. Making the bridge/MechLink
  report real RF loss belongs to the wireless phase.
- Pre-existing, not changed here: with an RC link, Auto's own Rotate/
  flywheel/feed writes still never reach hardware because nothing enables
  Shooter outputs in the RC path either. Needs a decision in the wireless
  phase.
- Manual scale values (0.30 chassis, 0.10 rotate/angle) are starting values.
- Everything from the previous entry's hardware-unverified list still
  applies (A/D polarity on real wiring, ExitSensor polarity, 3s jam value,
  vision-follow quality).
- Still deferred: wireless, final pins, file cleanup.

## 2026-10-11 — Claude Code: fixes for Codex re-review of afed238 (three remaining items)

**User Request:** Codex confirmed the previous round's fixes and listed
three remaining items. Jeremy approved the plan and chose "stop keys win
while armed" for the first one.

**Changed:**
- `tools/dashboard/index.html` [P1]: Enter/Space no longer get swallowed by
  the text-box guard while armed. Previously, with R or the Falcon running
  and focus in the stream URL box, Enter didn't disable and Space didn't
  disconnect, and the heartbeat kept the session alive. Now while armed
  they always stop/disconnect; while disarmed, text boxes keep normal
  Enter/Space. Movement/R/F/L keys are still ignored in text boxes. The old
  page-test assertion that "Enter in a text box sends nothing" (which
  encoded the bug) was replaced.
- `src/IO/WiredControl.{h,cpp}` [P2] oversized line: after
  `COMMAND_TOO_LONG` the rest of the line is now discarded up to `\n`
  (`discardingLine`), including across separate serial reads, so a tail
  like `...E`/`...R`/`...FLY,1` is never parsed as its own command.
  Overflow while armed also disarms (`DISABLED_CORRUPT_COMMAND`), treated
  like a lost link.
- `src/IO/WiredControl.cpp` [P2] non-finite numbers: `parseValue()` rejects
  anything `!isfinite()` after `strtod` (nan/inf in any case, signed,
  "infinity", overflow like `1e999`). Rejected values → `BAD_COMMAND`, the
  previous command is kept, and the watchdog is not refreshed. Covers every
  numeric entry point (`T` both fields, `ROT`, `ANG`, `FLY`).

**Added tests:** `testOversizedLineTailIsDiscarded` (24 x `z` + `E`
doesn't arm; next line parses normally; armed oversized lines ending in
R/FLY/T don't run and disarm; overflow split across reads) and
`testNonFiniteNumbersRejected` (11 spellings x 5 entry points, values
unchanged, garbage-only traffic still times out) in
`test/wired_control_test.cpp`. Page test: R + Falcon running → focus into
the stream URL box → Enter stops (no further heartbeat); same with Space →
stop then disconnect; disarmed Enter/Space in the box send nothing.

**Evidence:** Mutation-checked: removing `isfinite`, removing the
discard-to-newline, or removing the corrupt-stream disarm each fails its
test; the new page test fails against the committed `afed238` HTML (Enter
in text box sends no `X`). Codex's `/tmp/wired-review-extra.cpp` now
prints `armed=0` / `isNaN=0` and passes. Full native + page suites pass
(both `wired_control_test` builds). `platformio run`: `megaatmega2560`
(RAM 2108/8192, Flash 28854/253952), `mega_shooter_keyboard_test`,
`mega_vision_rotate_test` all SUCCESS. No upload/power/hardware.

**Still open (unchanged):** `PIN_TABLE_CONFIRMED=false` refuses Enable; the
robot STM32 bridge on Serial2 keeps `signalLost` false and also refuses
Enable; RC-path Shooter enable; A/D and ExitSensor polarity, jam timeout,
manual scale values on real hardware; wireless, final pins, cleanup.

## 2026-10-11 — Claude Code: standalone Mega <-> STM32 UART link test (before ELRS)

**User Request:** Jeremy (via Codex's handoff, and confirmed directly):
prove the STM32 and Mega can talk both ways before wiring the ELRS RX.
Mega Serial2 D16 TX → STM32 PA10, STM32 PA9 → Mega D17, 115200 8N1;
Orin stays on Serial1 D18/D19. The STM32 is a **new, never-flashed**
WeAct F401CCU6 (not the earlier board). Jeremy: circuit and level shifter
are already wired, don't re-investigate them.

**Added:**
- `src/Bench/UartLinkTest/ulink.{h,c}`: shared protocol, plain C, no libc
  (the same file builds into the Mega sketch, the STM32 firmware and the
  host test). ASCII lines `<body>*<CRC-8>\n`; `D` data every 100 ms from
  each side, `A` ACK echoing run/seq/payload (an ACK never triggers an
  ACK), `C` Mega→STM32 run start/stop, `S` STM32→Mega status every 250 ms.
  One outstanding packet per side, 50 ms ACK timeout; only an ACK matching
  the outstanding run+seq+payload counts; late/duplicate/wrong-payload ACKs
  have their own counters. Receivers verify payload content and track
  sequence gaps/duplicates. Bounded 200-byte line buffer, oversized lines
  dropped through the next `\n`. LINK_DOWN after 5 consecutive timeouts,
  LINK_UP after 3 consecutive successes. Explicit run start (data begins
  only after the STM32 confirms) and drain (report only after both sides
  stopped sending and have nothing outstanding, else NOT CONFIRMED after
  2 s). The 100 ms cadence is this test's own; it is not the ELRS rate
  design.
- `src/Bench/UartLinkTest/MegaUartLinkTest.cpp` + new isolated env
  `mega_uart_link_test` (builds only `Bench/UartLinkTest/`, so it can't
  contend with the robot build for Serial2; serial buffers raised to 256
  so a 1 s USB progress print can't drop Serial2 bytes — verified in the
  ELF: `Serial`/`Serial2` objects are 0x21d bytes). USB commands
  `RUN,<s>`, `START`, `STOP`, `STATUS`; prints progress, LINK/STM32-alive
  events, a human-readable final report with both directions, RTT
  min/avg/max for each side, error counters, `RESULT: PASS/FAIL`, and a
  `$ULINK_RESULT,...` machine line.
- `tools/stm32_mega_uart_test/`: register-level STM32 firmware (USART1
  PA9/PA10, IRQ → 512-byte ring, hardware overrun/framing/noise and
  ring-full counted into `bad`), clock/GPIO/USART bring-up reused from
  `stm32_elrs_robot_bridge`, linked at **0x08000000** for the ROM USB DFU
  bootloader (no WeAct HID bootloader assumed on a fresh board — the old
  0x08004000 images don't apply). `build.py` runs the host test first,
  then cross-compiles and checks vectors/stack/size/no undefined symbols
  (adds `-fno-tree-loop-distribute-patterns` so gcc can't emit hidden
  memset/memcpy calls under `-nostdlib`). README has the dfu-util flash
  steps, the Mega upload/monitor commands, and the acceptance procedure.
- `tools/stm32_mega_uart_test/test/host_test_ulink.c`: unit checks (CRC-8
  check value 0xF4, payload) and a 1 ms-stepped simulation of both real
  endpoints through two byte channels with per-line faults: clean 60 s
  (M sent 600 / S received 600, S sent 601 / M received 601, zero errors,
  drain confirmed, A-lines == D-lines), dropped data, dropped ACK,
  corruption that breaks the format, corruption that stays well-formed
  (only CRC catches it), late ACK, duplicate ACK, cut + reconnect
  (LINK_DOWN, ~20 timeouts, LINK_UP, no further timeouts), stop while cut
  (NOT CONFIRMED), lost control lines, oversized line whose tail looks
  like `C,1,0`, wrong run / wrong role.

**Evidence:** host test passes; mutation-checked (removing CRC check,
discard-to-newline, outstanding-match for ACKs, D payload verification,
drain condition, or making stale ACKs trigger an ACK each fails a test —
the CRC mutation initially slipped through, which is why the well-formed
corruption test exists). `build.py`: 3640-byte image, vectors at
0x08000000, SysTick/USART1 wired, no undefined symbols, sha256
f80173da…7a43. `platformio run`: `mega_uart_link_test` SUCCESS (RAM 1583/8192);
`megaatmega2560`, `mega_shooter_keyboard_test`, `mega_vision_rotate_test`
still SUCCESS. **Not flashed, not run on hardware.**

**Next Test (Jeremy, on the bench):** flash STM32 via ROM DFU and the Mega
env (README), confirm `STM32_STATUS_RECEIVED`, `RUN,60` → expect ~600 each
way and `RESULT: PASS`; then `START`, pull one signal wire → `LINK_DOWN`,
replug → `LINK_UP`, `STOP`. Paste the report back. ELRS RX only after this
passes on real hardware.

**Side note (pre-existing, not changed):** a full rebuild shows
`division by zero` warnings at `src/Shooter/Shooter.cpp:946/953`
(`rotateRawToDegrees`, from `a720ac0`): the rotate calibration raw values
are all still 0, so the denominators are constant zero. Needs a guard or
real calibration values; out of scope here.

## 2026-10-11 — Claude Code: UART link test hardware results (Jeremy ran it, 9d6aa12)

**Flash:** STM32 (fresh WeAct F401CCU6) entered ROM DFU (`0483:df11`, alt 0
`@Internal Flash /0x08000000`) after Jeremy fixed the USB data connection;
`dfu-util -a 0 -s 0x08000000:leave -D stm32f401cc_mega_uart_test.bin` →
`Download done`, 3640 bytes (sha256 f80173da… matches `verification.json`).
The initial `dfuERROR ... firmware is corrupt` was the blank chip before
download; `Invalid DFU suffix` is because a raw .bin was used. Mega flashed
with `pio run -e mega_uart_link_test -t upload`.

**Monitor note:** `pio device monitor` sends keystrokes one by one without
echo; the first `RUN,60` attempt arrived as `UNKNOWN_COMMAND` (likely a typo
the operator couldn't see). Use
`pio device monitor -e mega_uart_link_test --echo --filter send_on_enter`.

**60 s run (run 1636) — PASS:**
```
duration_ms=60000  drain=confirmed (STM32 final counters)
Mega->STM32: sent=600 acked=600 timeout=0 lateAck=0 dupAck=0 badAck=0 | received=600 lost(gap)=0 dup=0
STM32->Mega: sent=604 acked=604 timeout=0 lateAck=0 dupAck=0 badAck=0 | received=604 lost(gap)=0 dup=0
RTT seen by Mega:  min=4 avg=5.19 max=9 ms
RTT seen by STM32: min=3 avg=5.27 max=9 ms
line errors: Mega bad=0 overflow=0 | STM32 bad=0 overflow=0
RESULT: PASS
$ULINK_RESULT,1636,60000,1,1,600,600,0,0,0,0,600,0,0,604,604,0,0,0,0,604,0,0,0,0,0,0,4,9,3111,3,9,3183
```
Observation: the STM32 sent 604 in the Mega's 60 s, and the lead grew
during the run (+2 by 28 s, +4 by 55 s) — its 16 MHz HSI is running ~0.6 %
fast relative to the Mega (within HSI's ±1 % factory spec). Harmless at
115200 here (zero errors), but it adds to the nominal +0.25 % BRR error of
the 420000-baud ES900RX link in the ELRS firmware, which also runs on HSI.
Worth considering the board's HSE crystal before/while doing ELRS.

**Disconnect/recover (run 38824):** signal wires are soldered and can't be
unplugged, so Jeremy chose to hold the STM32's NRST for ~3 s mid-run (real
"peer disappears" event; no code change). Result: LINK_DOWN at t=12753 ms
(5 consecutive timeouts), STM32_SILENT ~1 s later, 30 timeouts total during
the hold, none counted as success; after release STM32_STATUS_RECEIVED and
LINK_UP at t=15508 ms, Mega timeouts stayed at 30 for the remaining ~19 s,
every Mega packet acked again (RTT avg 5.15 ms). STM32→Mega had received 124
with 0 gaps before the reset. `Mbad=1`: one partial/garbled line from the
reset moment, rejected by CRC. Report shows `drain=NOT CONFIRMED`,
`RESULT: FAIL`, `S>M=0/0` — expected: the rebooted STM32 doesn't rejoin the
run, so its counters restart and it stops sending data.

**Not verified:** STM32→Mega recovery after a real outage (the reset
approach can't show it; would need a rejoin feature or a software CUT
command), a physical wire break (soldered), behavior over longer runs.

**Next:** per Codex's gate, the wired Mega↔STM32 link has passed on real
hardware for the 60 s and outage-detection checks above; ELRS RX is the next
stage once Codex reviews 9d6aa12 and these results.

## 2026-10-11 — Claude Code: ELRS step 1 (downlink only) — robot bridge on the fresh board + Mega bench env

**User Request:** after the Mega↔STM32 UART test passed on hardware, Jeremy
wired the ES900RX to the robot STM32's PA2/PA3 (Dupont) and approved
"step 1: get the wireless link working" (downlink only). Topology confirmed:
PC / homemade keypad → ground STM32 (the **old** board, WeAct HID
bootloader) → ES900TX ))) ES900RX → robot STM32 (the **new** board) → Mega
Serial2. ES900TX not wired yet; the PC will connect to the ground STM32
through its own USB-C (step 2, not started).

**Changed (`tools/stm32_elrs_robot_bridge`):**
- Linked at `0x08000000` (was `0x08004000`) for the fresh board's ROM DFU
  flash; `build.py`/`linker.ld`/`verification.json` updated. 2360 bytes,
  sha256 4d4fae92…2b38.
- `CH,...` forwarding to the Mega is gated by
  `ch_line_should_forward()` (new, host-tested incl. wrap-around): only
  while a validated `RC_CHANNELS` frame arrived within 500 ms. Previously
  the bridge sent a line every 20 ms regardless, so `MechLink` could never
  see RF loss (and the wired Dashboard's RC-link gate would refuse Enable
  forever). Assumes ES900RX stops emitting RC frames on link loss — the
  bench test is what verifies it.

**Added:** `src/Bench/ElrsLinkTest/MegaElrsLinkTest.cpp` + env
`mega_elrs_link_test` (production `MechLink.cpp` unchanged + a printer:
channels 4x/s, LINK_OK/LINK_LOST events with previous-state duration,
count of `fwd` changes; serial buffers 256). README section "2026-10-11
changes" has flash steps and pass criteria.

**Ground side for step 1:** no new firmware. The old board should still
carry `tools/stm32_elrs_bidirectional_test` (RC frames every 10 ms, CH1
cycling 1000/1500/2000 every 2 s) — enough as a signal source. Not
confirmed what's actually on it; its LED behavior will tell.

**Evidence:** robot bridge `build.py` (host test + image checks) passes;
`platformio run`: `mega_elrs_link_test`, `megaatmega2560`,
`mega_uart_link_test` SUCCESS. Not flashed, not run.

**Step 2 (not started, needs Codex's view):** PC → ground STM32 over its
own USB-C needs a USB device (CDC) stack. Every STM32 tool here is
register-level/no-HAL; hand-writing an OTG FS CDC stack is large and hard to
debug without SWD. Alternative: build the ground firmware on PlatformIO's
STM32 Arduino core (`board = blackpill_f401cc`, built-in USB CDC Serial,
HSE 25 MHz + PLL clocking — which also removes the measured ~0.6 % HSI
error), reusing `tools/stm32_elrs_common` C code. This is an architecture
call; not taken yet.

## 2026-10-11 — Claude Code: ELRS step 1 hardware result — downlink PASS end to end (b624da7)

**Setup that worked:** ground = old STM32 (WeAct HID bootloader) still
running `tools/stm32_elrs_bidirectional_test` (RC frames every 10 ms, CH1
cycling 1000/1500/2000 every 2 s) → **2.4 GHz** ELRS TX module (PA9
half-duplex, 400000) ))) **2.4 GHz** ELRS RX (firmware 3.5.1, ISM2G4) →
robot STM32 (fresh board, `stm32_elrs_robot_bridge` b624da7 at 0x08000000,
PA3 ← RX's TX, PA2 → RX's RX) → Mega Serial2 running `mega_elrs_link_test`.

**How we got there (module configuration, done by Jeremy in the ELRS web
UIs):**
- The first pair tried was 900 MHz ES900TX/ES900RX (both 3.2.0) and could
  never link: TX was **FCC915** with UID 180,148,31,165,224,154, RX was
  **AU915** with UID 212,50,59,163,20,74 — different regulatory domain and
  different binding. (Claude also caused a side problem there: advised a
  Save on the RX options page while "WiFi auto on interval" was blank,
  which ELRS stores as -1 = auto-WiFi disabled.)
- Jeremy switched to 2.4 GHz ELRS. New RX was set to: Serial Protocol
  **CRSF** (it shipped as Inverted SBUS), UART baud **420000** (was
  100000), WiFi auto-on 60, Model Match off, telemetry not forced off,
  Lock on first connection on. New TX: same binding phrase as the RX
  (UIDs matched on both pages before saving), **UART inverted unchecked**
  (the STM32 drives a plain non-inverted UART; the earlier hardware-
  confirmed 900 MHz TX also had it unchecked). The binding phrase itself
  is intentionally not written into the repo.

**Result (`pio device monitor -e mega_elrs_link_test`):**
- `$ELRS_EVENT,LINK_OK` within ~15 ms of the monitor opening; `fwd`
  stepping 1000 → 1500 → 2000 every 2 s, and every other channel exactly
  the ground firmware's fixed values (turn 1500, aux 1000, fire 1000,
  mode 1000) — channel order and µs scaling are correct end to end.
- Ground STM32 powered off: `$ELRS_EVENT,LINK_LOST` at t=9238 ms (last
  `link=OK` print at 9000 ms; exact power-off instant not captured, so
  latency is only bounded, ≤ the designed ~0.7 s), all channels back to
  neutral 1500. This **confirms on hardware that the ELRS RX stops sending
  RC_CHANNELS frames on link loss** and that the bridge's new silence →
  MechLink `signalLost` path works — the reason for b624da7's change.
- Ground STM32 powered back on: `$ELRS_EVENT,LINK_OK` at t=8391 ms of a
  fresh monitor session, `fwd` cycling resumed.

**Safety note for the next step:** the bench ground firmware sends
mode = 1000, which the production `OperatorModeSelector` reads as
**FULL_AUTO** (< 1667). Do not run the production robot build against this
ground firmware; the real ground firmware (step 2) must define safe
defaults (manual mode, fire released) before any production test over RF.

**Not covered yet:** uplink (Mega `TEL` → TEL_CHUNK → ground), long-run
packet statistics, range, the HSI baud margin at 420000 under temperature,
and everything in step 2 (PC/keypad input to the ground STM32 over its
USB-C, CRSF channel/event table, F/R/L anti-repeat, wired/wireless
authority) — step 2's USB-stack architecture question is with Codex.

## 2026-10-11 — Claude Code: wireless PC control (ELRS step 2) + feed-tower trigger redesign

**User Request:** "現在要準備接上電腦，用電腦控制整台機器人" (connect the PC and
control the whole robot from it). Jeremy chose to go **straight to wireless**
(skip a wired test), approved the plan, and mid-round redefined the feed
tower: "送球塔的部分是平時低電位，收到發射的訊號已100ms做發送高電位…才做自動送球、補球"
(the line idles low; on a fire signal send a 100 ms HIGH pulse; the tower
does its own feeding/refilling). His answers: the Mega only sends the
trigger pulse; R (continuous feed) is no longer needed; whether the tower
has feedback is unknown. The pin table is still to come from Jeremy.

**Discussion Result / architecture (step 2's open question):**
- **Ground STM32 firmware** (`tools/stm32_ground_control`) is built on the
  PlatformIO STM32 Arduino core (`blackpill_f401cc`) for its USB CDC stack
  and HSE 25 MHz clocking. The bare-metal tools stay as they are. This is
  the alternative proposed in the step-1 entry; Codex has not reviewed it yet.
- **One shared C codec**, `src/IO/RcControlMap.{h,c}`, used by the ground,
  the robot bridge, the Mega and the host tests:
  - The 9-channel table is in its header. CH1-4 are full resolution: drive
    L/R, flywheel, mode (fixed 2000 = FULL_MANUAL). CH5 is the arm 2-position
    switch. CH6/7 are rotate/angle jog (3 levels). CH8/9 are the F/L
    **event counters** (6 levels).
  - It fits ELRS Hybrid switch mode. The real 6-position outputs
    (988/1193/1398/1602/1807/2012 µs) are covered by tests.
- **Event counters, not held buttons:**
  - Exactly one action per +1 step.
  - After a reconnect, the first level seen is only a baseline, so nothing
    is replayed.
  - A jump of more than one step is ignored and reported as `EVENT_SKIPPED`.
- **Enable needs an edge:**
  - Wireless enables only on a CH5 low→high edge.
  - The edge detector is reset on link loss, so a recovered link never
    re-enables by itself.
  - Arm going low disarms a wireless session.
- **Control authority:** there is exactly one owner, WIRED or WIRELESS.
  - The other source's E is refused with `ENABLE_REJECTED_OTHER_SOURCE`.
  - The other source's commands are refused with `REJECTED_NOT_OWNER`.
  - **Wired X always disarms, whoever owns the robot.**
  - Wired K only refreshes a WIRED session.
- **Status uplink:** the Mega sends `ST,flags,seq,code` every 100 ms on
  Serial2. The robot bridge packs it into a standard CRSF **Battery** frame:
  - voltage = 0xA5 marker + flags
  - current = event code
  - capacity = 24-bit event sequence

  ELRS relays this as FC telemetry. The ground prints `$EVENT,<name>` when
  the sequence advances, plus `$WIRELESS,...` every 250 ms and `$RFLINK,...`
  (LQ/RSSI from LINK_STATISTICS).
- **Feed tower:** the Mega only drives a trigger line.
  - It idles inactive and goes active for `PULSE_MS` 100.
  - A `COOLDOWN_MS` 1000 lockout follows; the value is provisional and
    waits for the real tower cycle time.
  - A trigger inside the lockout gets `FEED_BUSY` and is **dropped, not
    queued**.
  - Disarm cuts an active pulse but does not shorten the lockout.
  - There is no sensor feedback, so the Mega reports only
    `FEED_TRIGGER_SENT`.
  - R is removed and is now `UNKNOWN_COMMAND`.

**Why:**
- Held-level buttons over a lossy, round-robin link either repeat or get
  missed. Counters make F/L idempotent.
- The edge requirement and the reset on link loss prevent a robot that
  comes back into range from re-arming by itself.
- The single owner keeps a USB page and a radio from both driving at once,
  while the wired stop still always works.
- Using the Battery frame avoids depending on a custom frame type surviving
  the radio.

**Changed:**
- `src/IO/WiredControl.*` → `ManualControl.*` and
  `WiredControlConstants.h` → `ManualControlConstants.h` (git mv, then
  rewritten).
  - `update()` order: watchdogs, wired lines, wireless frame, outputs,
    status.
  - Watchdogs: wired 250 ms; wireless RC loss gives `DISABLED_RC_LINK_LOST`;
    no valid frame for 250 ms gives `COMMAND_TIMEOUT`.
  - Kept from afed238/5d02fca: the wired hardening (NaN/inf, oversize
    discarded through `\n`, `DISABLED_CORRUPT_COMMAND`).
- `src/IO/MechLink.*`
  - Parses `RC,c1..c9`, with values limited to 0..3000 and an 80-byte line
    cap; oversized lines are discarded through `\n`.
  - `channels[9]`, `frameCount`, `signalLost` (200 ms).
  - `sendStatus()`.
  - Safe values on loss: arm 1000, flywheel 1000, mode 2000, counters 1000.
- `src/Dribbler/*`, `src/Constants/DribblerConstants.h`: rewritten as the
  trigger tower.
  - `Tower::TRIGGER_PIN` 2 is **PROVISIONAL**.
  - The line is written inactive before `pinMode(OUTPUT)`.
- `src/Telemetry/Telemetry.cpp`
  - Slots 7-11: left, right, mode, arm and F-counter channels.
  - Slots 21-23: owner (0/1/2), tower busy, trigger count.
- `src/main.cpp`, `src/Auto/Auto.cpp`, `src/Constants/Pins.h`: renames and
  comments only. Auto still steps aside while `ManualControl::isActive()`.
  `PIN_TABLE_CONFIRMED` is **still false**.
- `tools/stm32_elrs_robot_bridge`
  - Downlink is now `RC,...` with 9 channels.
  - The USART1 RX interrupt sorts `ST,` lines into a status buffer.
  - A Battery status frame goes out every 100 ms while the status line is
    ≤ 500 ms old.
  - `build.py` also compiles `RcControlMap.c`.
  - The `dist/` image is rebuilt: 2932 bytes, sha256 `ddd9ef21…12e3e`.
  - The unused `tel_channel_map.h` include is removed; that header is now
    marked as old-ground-bridge only.
- `tools/dashboard/index.html`
  - R is removed and the labels are updated.
  - New metrics: control owner, feed tower busy/ready plus trigger count,
    wireless status and RF LQ.
  - **Enable is now a pending handshake:**
    - Clicking sends E and starts sending K heartbeats.
    - The page arms only on `OUTPUTS_ENABLED`, and only if it asked
      (the Mega prints the wireless operator's enable on USB too), or on
      telemetry/`$WIRELESS` confirming its own source.
    - After 3 s with no confirmation it gives up.
    - On a rejection or timeout it sends X **only on the ground link**,
      because a wired X would disarm a wireless owner.
  - Telemetry resync counts as "armed" only when the owner is WIRED.
  - Disarm event list: `OUTPUTS_DISABLED`, `COMMAND_TIMEOUT`,
    `DISABLED_RC_LINK_LOST`, `DISABLED_CORRUPT_COMMAND`.
  - Ground `PC_TIMEOUT`/`ENABLE_RELEASED`/… also disarm.
  - A stale `$WIRELESS` warns but does **not** auto-stop; the robot's own
    RC-loss failsafe does that.
- `src/Bench/ElrsLinkTest/MegaElrsLinkTest.cpp` and the
  `mega_elrs_link_test` env (+`RcControlMap.c`)
  - Decodes each frame with the production codec.
  - Sends status back, with "armed" mirroring the arm channel.
  - Reports `OUTPUTS_ENABLED`/`DISABLED`/`FEED_TRIGGER_SENT`/
    `EVENT_SKIPPED` as events.
  - This makes it a motor-free round-trip test.

**Added:**
- `src/IO/RcControlMap.{h,c}`
- `test/rc_control_map_test.c`
- `test/manual_control_test.cpp` (renamed from `wired_control_test.cpp` and
  rewritten)
- `tools/stm32_ground_control/` (platformio.ini, `src/GroundCommandState.*`,
  `src/main.cpp`, `src/shared_sources.c`,
  `test/host_test_ground_state.cpp`, README)
- Ground PC protocol: the same lines as wired. The ground's own watchdog is
  250 ms: `PC_TIMEOUT` releases the arm channel.

**Calculation:**
- Drive channel: µs = 1500 + 500·cmd. For example, T,1,-1 → CH1 2000 /
  CH2 1000.
- Flywheel: µs = 1000 + 1000·ratio, with a 0.02 deadband. For example,
  FLY,0.40 → 1400.
- Counters: 1000 + 200·k. They are accepted within ±90 µs of a level, so
  ELRS's 1193 is read as k=1.
- Arm counts as requested only at ≥ 1750 µs.
- Jog: < 1250 → −1; > 1750 → +1; anything else (including ELRS's
  1398/1602) → 0.
- Mega outputs keep the wired scales: chassis 0.30, rotate/angle 0.10.

**Impact:**
- Wired USB control behaves as before, except that R is gone and the stop
  and authority rules above apply.
- **The old ground firmware (`stm32_elrs_bidirectional_test`) sends
  CH4 = 1000 = FULL_AUTO.** Do not run the production build against it.
- The new ground sends 2000.
- `PIN_TABLE_CONFIRMED` false still refuses every Enable
  (`ENABLE_REJECTED_PINS_UNCONFIRMED`).

**Evidence (host/build only, nothing flashed, no hardware run):**
- PIO SUCCESS for all 5 Mega envs; production RAM 27.6 %, Flash 11.5 %.
- Ground `pio run` SUCCESS (RAM 9.0 %, Flash 17.2 %, vector table at
  0x08000000).
- Robot bridge `build.py` passes all checks.
- Host tests pass: rc_control_map, manual_control (both pin-gate variants),
  ground state, ch_line_format, ulink, and the 5 older C++ tests.
- Node tests pass: `dashboard_page_test` (new handshake, ground lines,
  authority resync), `dashboard_control_test`, `vision_rotate_page_test`.

**Not verified / risks:**
1. **Whether Battery frames from the RX actually reach the ground over the
   air.** The earlier "bidirectional" bench may only have shown
   LINK_STATISTICS. Without that uplink, wireless Enable cannot complete.
   The page gives up after 3 s and releases the arm channel, which is the
   safe failure.
2. Status coalescing: one status line is sent per 100 ms and carries only
   the latest event. Two events within 100 ms show only the second;
   `$WIRELESS` flags still resync the page.
3. CH6+ round-robin latency depends on the ELRS packet rate.
4. Whether the 3.3 V-referenced trigger pulse is compatible with the
   tower's input.
5. Tower cycle time (`COOLDOWN_MS`).
6. The real pin table.

**Next Test (Jeremy runs; wheels off the ground):**
1. Flash the new robot bridge `dist/` image and the ground board
   (`tools/stm32_ground_control/README.md`; this overwrites its HID
   bootloader). Upload `mega_elrs_link_test`.
2. Connect the Dashboard to the ground board's USB. Check: Enable completes,
   F → `FEED_TRIGGER_SENT`, `$RFLINK` LQ, ground power-off → Mega
   `LINK_LOST`.
3. Only after the pin table is confirmed: run the production build at low
   speed. Check direction, Enter/Space, F, L, ground power-off while moving,
   and closing the page while moving.

## 2026-10-11 — Codex：Jeremy 授權補回 R 水平輸送帶，F 獨立觸發塔

User Request / Discussion Result: Jeremy 明確確認整台實控馬達共六顆：底盤兩顆、水平輸送帶、飛輪、仰角、水平各一；塔只需短暫 HIGH 自行完成一輪。直接授權實作 R 開關水平輸送帶、F 單次塔觸發，不再詢問。

Changed / Why: Dashboard R 單次按下送命令（repeat 忽略）；ManualControl 有線 R 僅 owner/armed 下切換，F 不影響輸送帶。地面端 R 切換持續狀態，新增 CH10 1000=off /2000=on，機器人使用狀態而非對每包 toggle。CH1-9 語意不變；wire format RC 現為十欄，bridge/地面/Mega 必須同步更新。停用、watchdog、RF loss 清除輸送帶；地面 release/E 清除狀態。追加 CONVEYOR_ON/OFF 事件代碼。F 保留 100ms HIGH、從觸發起 1000ms lockout，忙碌不排隊，不聲稱出球成功。

Added / Calculation: Dribbler 新增獨立 conveyor API，沿用先前 PWM+enable 驅動介面，RUN_PWM=180/255 約70.6% 為既有起始值而非已確認速度。正式 PWM/enable 腳位未知，均設255為未配置，不寫任何硬體腳；PIN_TABLE_CONFIRMED 仍false，並新增編譯檢查防止未配置輸送帶卻設true、或與塔/彼此腳位碰撞。isConveyorOn 表示要求的開關狀態，未配置時不代表馬達實際轉動。待 Jeremy 提供正式腳位/驅動介面再落實。

Evidence: manual_control host confirmed/unconfirmed皆過，新增 R/F 獨立、R反向切換、X、逾時、無線重複幀及失聯清除；ground host 新增 disarmed R拒絕、R切換/編解碼/F不影響/watchdog/re-enable；rcmap、dashboard page（R/F/L repeat）、dashboard control tests皆過。bridge build.py及映像檢查通過，重建dist；Mega production PIO SUCCESS RAM2291 Flash29286，ground PIO SUCCESS RAM5888 Flash45196。git diff --check通過。未燒錄、未上機；未完成整個16f26be廣泛review。

Next Test / Impact: 等正式pin後再次編譯，三端同步燒錄，實測R獨立開關、F單脈衝/忙碌、Enter/Space/失聯停機。不能沿用舊九欄bridge搭新十欄Mega。目前只有控制路徑完成，輸送帶實際硬體輸出待pin配置。

## 2026-10-11 — Codex：優先有線全手動，更新操作輸出比例

Jeremy 要求先 PC USB-C→Mega 有線全手動，不先測無線；底盤0.5、水平旋轉滿輸出1.0、輸送帶0.5、仰角MG996R且無encoder希望0.3、飛輪保留速度拉條。
新增 mega_wired_manual_test env (TEL_WIRED_MANUAL_ONLY)，拒絕無線frame控制、不執行Auto、L維持manual。共用ManualControl比例改底盤0.5/水平1.0/仰角0.3，輸送帶 PWM128/255=50.2%。飛輪既有FLY比例路徑不變，仍為PWM開迴路而非RPM閉迴路。沒有拆除水平硬限位。
已跑新profile host（拒無線、L不切追蹤、0.5/1.0/-0.3/FLY0.6數值、X停用）通過；完整manual_control confirmed host也過。新env PIO SUCCESS RAM2120 Flash26600，未燒錄。
重要未決：使用者說單顆MG996R，現有Shooter仍attach兩顆仰角pin27/26，PWM以1500中立速度方式控制，型號可能180定位或360連續。已async詢問版本；尚未改為單顆或position slew，ANGLE_MANUAL_SCALE=0.3只是參數準備，不聲稱仰角可用。PIN_TABLE_CONFIRMED仍false，正式pin待Jeremy提供，輸送帶仍未配置。MG996R版本確定後才完成對應仰角輸出/停止語意。全手動profile目前不是硬體驗收完成。

## 2026-10-11 — Codex：按 Jeremy 指示先使用單顆 MG996R 定位版

User Request / Discussion Result: Jeremy 明確採用定位版先寫，若硬體不符再切連續旋轉版。取代上一則「型號待確認」。
Changed: ShooterConstants::Angle 新增 POSITION_SERVO=true、EXTERNAL_ENCODER_ENABLED=false。移除第二顆Servo物件/attach；保留舊right telemetry欄位但回0，單顆仍使用LEFT_SIGNAL_PIN暫定27。無外接encoder不初始化/輪詢AS5600，拒絕encoder counts closed-loop目標。POSITION_SERVO=false可切回單顆連續旋轉速度模式。
Calculation: ServoPositionJog累積目標 target += command *1000 us/s *dt；手動縮放0.3，因此10ms變3us、1秒變300us，例如1500→1800us。不是實測速度或仰角。暫定起始1500us、上下界1000/2000us，保留硬限位方向判斷；迴圈間隔>100ms不補跳、NaN/越界command不更新。
Semantics: 放開鍵command0保持最後目標。啟動disabled不attach定位舵機；首次Enable會送初始1500us（實際位置未知可能移動）。stopAll/disableAngle/Enter/watchdog在定位模式detach，不送1500作停止，保留軟體target；停止脈衝不等於断電、不保證保持扭力；再啟用使用保留target，無實際位置回饋。連續旋轉模式則0命令仍送中立。
Evidence: 新test/servo_position_jog_test.cpp驗證0.3累積、放鍵、停頓、NaN、兩端clamp與反向離開邊界通過；mega_wired_manual_test PIO SUCCESS RAM1992 Flash25050。git diff --check通過。未燒錄/未實機，PIN_TABLE_CONFIRMED仍false；正式pin、實際servo行程仍待提供/實測。


## 2026-10-11 — Codex：Jeremy 授權清理未使用檔案與建置快取

Jeremy 明確要求「刪掉會影響嗎？不會的話就刪」，承接先前盤點。已刪根目錄及 stm32_ground_control 的 .pio 快取、未引用 lib/sbus、Constants/Mode.h、PIDConfig.h、src/list.md、include/README、lib/README、舊 .vscode/arduino.json，以及 .DS_Store/空 .claude/.dist。Dashboard data-panel="sbus" 僅 UI 名稱，不引用被移除函式庫。保留現有未提交功能修改、全部 dist 燒錄產物、實際引用的 AS5600、測試程式、舊架構資料與編輯器除錯設定。功能控制流程未改；下次編譯需要重建快取/可能重新下載依賴。以引用搜尋及刪除後 git diff --check 檢查；未重新編譯或上機。


## 2026-10-11 — Codex：正式底盤腳位及 D45 單線 ESC 輸送帶

Jeremy 提供輸送帶 D45、右輪 D47、左輪 D46，確認輸送帶 VCC/GND/單訊號 ESC 伺服脈寬介面，並明確批准「開始」。ChassisConstants 腳位39/38改47/46；Dribbler 移除 analogWrite+enable，改單 Servo D45，R保持切換、stop/disable/watchdog回停止脈寬；F塔100ms邏輯不變。輸送帶暫採雙向ESC校準1500us停止、1750us運轉，即1500+0.5*500；使用者確認介面，不代表確認ESC中立/端點。實測若單向ESC停點1000us，需先改校準，不能把1750us當實測50% RPM。PIN_TABLE_CONFIRMED仍false，整台正式pin表未齊，Enable仍拒絕。
新增 conveyor_esc_test 檢查D45、起始1500、R狀態運轉1750及stop回1500；Servo host stub記錄attached/pin/pulse。主機測試 conveyor_esc、manual_control confirmed及unconfirmed gate均過。PIO megaatmega2560及mega_wired_manual_test均SUCCESS，後者RAM1995 Flash25148；清理後依賴已重新下載/快取重建，證實這兩個Mega環境可編譯。git diff --check通過。沒有上傳/硬體測試；未驗證其他Mega bench或STM32環境的清理後重建。新增底盤左右/輸送帶腳位相異assert。後續低速台架確認ESC停點與方向，再確認其他pin後開啟Enable。


## 2026-10-11 — Codex：右底盤改 D48

Jeremy 回報 D47 改 D48。檢查共享工作目錄 ChassisConstants.h 已是右48/左46，未覆寫其修改；輸送帶45不變。mega_wired_manual_test重新編譯SUCCESS，RAM1995 Flash25148，未上傳；Enable門檻仍false。


## 2026-10-11 — Codex：只測底盤與輸送帶的局部確認版本

Jeremy 授權先測右D48/左D46/輸送帶D45，其他pin先不管。新增 mega_drive_conveyor_test，繼承有線全手動，TEL_DRIVE_CONVEYOR_ONLY局部腳位確認允許Enable；編譯assert鎖定45/46/48且禁止脫離wired模式。完整production與mega_wired_manual_test的全機pin gate仍false。此profile Shooter不初始化sensor/attach、不執行update、不寫任何軸或飛輪，setOutputsEnabled仍false；塔不pinMode/不digitalWrite/requestFeed回false。UI其他命令仍可發送但沒有硬體輸出，F會拒絕（既有FEED_BUSY回報，不表示塔真的忙）。底盤/輸送帶R、Enter/Space/250ms watchdog保留。ESC校準仍暫定1500停止、1750輸送帶運轉，非RPM50%驗收。
主機conveyor_esc partial test確認1750run/1500stop與塔GPIO sentinel完全未寫，通過。新env PIO SUCCESS RAM1888 Flash18840；git diff --check過。未上傳未實機。上傳請改用pio run -e mega_drive_conveyor_test -t upload，不要用原全手動env。先架空輪子、無球測W/S/A/D/R/Enter/Space，確認ESC中立與方向。


## 2026-10-11 — Codex To Claude：加入D24仰角、D23水平手動測試

Jeremy提供仰角D24、180度定位馬達，水平D23，明確同意加入有線局部測試。更新ShooterConstants/Pins別名；新env mega_manual_axes_test繼承mega_drive_conveyor_test，TEL_MANUAL_AXES_TEST允許兩軸，底盤右48/左46/輸送帶45保留。上傳改用 pio run -e mega_manual_axes_test -t upload。Dashboard Enable後W/S/A/D底盤0.5、R輸送帶0.5、上下仰角0.3、左右水平1.0；Enter停用、Space停用斷線。飛輪仍不attach/不write、塔仍拒絕不write，自動/無線不開放。完整production pin gate仍false。
此profile跳過未確認sensor/limit初始化與讀取，因此沒有實際硬限位保護；手動機構行程需Jeremy觀察。仰角1000..2000us暫定、1500起始，0.3*1000us/s=300us/s，10ms變3us，鬆鍵保持，停用detach，首次Enable可能移到1500。180度型號不代表已校準機構行程。水平暫定1500中立，滿輸出1000/2000，方向依既有反相設定。ESC停止點仍需實機。
新env編譯SUCCESS RAM1933 Flash22990；ServoPositionJog主機測試過；本次修改檔diff --check過。全working-tree檢查另發現同時更新的robot_bridge dist .map第143行尾空白，沒有改動他人生成產物。未上傳未實機。共享工作目錄有其他協作者更新，勿覆寫。


## 2026-10-11 — Codex To Claude：D25 Falcon拉條開放

Jeremy提供Falcon訊號D25並同意開放目前有線測試。ShooterConstants::Flywheel/PIN_FALCON改25；mega_manual_axes_test增加TEL_MANUAL_FLYWHEEL_TEST，在局部Shooter init預載1500us後attach D25，update輸出FLY命令，stopAll仍回0指令1500us。舊mega_drive_conveyor_test仍不開飛輪；塔仍拒絕/無輸出，未確認sensor/limits跳過。各六顆馬達訊號：右48左46、輸送帶45、仰角24、水平23、飛輪25。
拉條0..100→ratio0..1→1500+ratio*500us，例20%=1600us、50%=1750us；是開迴路指令不是RPM/PIDF，ESC中立仍沿既有1500假設待實測。開機即1500us，未Enable也有中立脈衝；首次Enable拉條0，Enter/Space/watchdog清零。先無球低比例確認停點，不能把型號或編譯當硬體校準。
PIO mega_manual_axes_test SUCCESS RAM1935 Flash23228；manual_control confirmed host與dashboard_control_test皆過，修改檔diff --check過。未上傳/未實機；保留vendor AS5600既有warnings。上傳命令 pio run -e mega_manual_axes_test -t upload，Dashboard Enable後拉條可用；F仍停用。此則取代上一則此env「飛輪停用」。


## 2026-10-11 — Codex To Claude：底盤與仰角手動比例改1.0

Jeremy直接要求底盤/仰角改1.0。ManualControlConstants CHASSIS_MANUAL_SCALE與ANGLE_MANUAL_SCALE均1.00，共用有線/無線手動路徑也採此比例；輸送帶0.5、水平1.0及飛輪拉條不變。仰角定位slew=1.0*1000us/s，10ms變10us、1500→2000需0.5s（軟體目標，不是實測舵機速度），1000..2000clamp保留。底盤±1對應1000/2000us依反相方向。更新有線host數值斷言。wired profile host通過（需-Wno-unused-function，其他profile測試函式在此編譯分支未呼叫）；PIO mega_manual_axes_test SUCCESS RAM1935 Flash23188。修改檔diff --check過。未上傳/實機，沿用上傳env；未確認限位仍不啟用。


## 2026-10-12 — Codex：Dashboard HTTPS部署

Jeremy明確要求部署HTTPS網站。使用Sites建立owner-private TEL Robot Dashboard，project_id appgprj_6acc72ec26408191bc872f13043b71a5；URL https://tel-robot-dashboard.yuxiuy15.chatgpt.site。僅tools/dashboard/index.html原樣靜態發布，不上傳韌體或整個repo，不改控制协议；USB Serial仍瀏覽器本機與Mega直接通訊。Dashboard control/page node測試皆過；來源commit 0c857fc1f03e490322d0308393ba7902b3498c3b（獨立Sites source repo，非robot git commit），部署工具確認succeeded。網站私人需同帳號登入。未做遠端頁面USB實機驗證；若網站host或登入框架限制serial權限，需另驗證。當前部署是快照，後續本機改Dashboard不會自動同步。源checkout在/private/tmp/tel-dashboard-site，雲端source已保存，可再取回。未改Mega韌體。


## 2026-10-12 — Codex：Jeremy授權推送整個dev並改GitHub Pages

Jeremy明確要求現在dev推上去、Dashboard部署Pages。本機dev領先origin/dev八個提交，沒有落後；本次包含工作目錄已授權功能/腳位/清理及協作者現有修改。新增.github/workflows/dashboard-pages.yml，dev Dashboard相關push觸發，Node兩個control/page測試通過才發布tools/dashboard；Pages artifact不含robot source/firmware。公開repo且現有GitHub登入admin。維持原Sites網站不刪除。正式全機gate仍false，局部手動env可Enable，不聲稱實機通過。


## 2026-10-12 — Codex To Claude：dev已推送、GitHub Pages部署完成

Jeremy授權推送現在全部dev並改Dashboard GitHub Pages。整合commit d7b6cbe已push origin/dev（包含先前8個本機提交）；Pages https://yxyang960102.github.io/TEL-Robot-2026/，公開HTTPS、不需ChatGPT登入。workflow .github/workflows/dashboard-pages.yml 由dev的Dashboard相關push自動跑2個Node測試，再deploy工具資料夾。Pages設build_type workflow、HTTPS enforced，github-pages環境新增dev精確分支allow policy，其他保護保留。run38209907595 attempt3 success；初次未啟用Pages、第二次dev被預設環境規則拒絕，均已解決。curl回200，live HTML SHA256與本機index完全相同1c8f9fc45eb67805f3a21f1d933dcbd02cc93943006842bf22f6744604e13adc。
兩個Dashboard node測試及Mega production/manual_axes編譯SUCCESS，未做Pages實機USB驗證。production compile顯示既存rotateRawToDegrees除零warning（未校準raw端點），full gate仍false，手動局部profile不使用位置換算；不要把compile成功視為整台自動功能驗收。生成bridge map仍有一處尾空白，本次沒有重寫生成產物。原Sites私人網站沒有刪除，今後以GitHub Pages網址為主要分享入口。Chrome/Edge直接USB接Mega，需使用正確手動profile，無需拷貝網頁或重燒僅為换筆電。

## 2026-10-13 — Claude Code: production turret vision-follow switched to the bench-tuned PIDF

**User Request:** "這個數值可以，可以寫進dev嗎？正式版的參數". Jeremy accepted the A5
bench tuning (kP 5 µs/px, kI 5 µs/(px·s) in a 20 px IZone, I cap ±60 µs,
output ±500 µs, kD 0) and asked for it to become the production
parameters on dev.

Jeremy's answers to the open questions:
- Fire-ready tolerance: **5 px**, held for 100 ms.
- The bench servo **is the D23 turret servo, same mounting**, so the
  production direction follows the bench result.

**Discussion Result / Why:**
- dev's production `VISION_FOLLOW` was still the pre-PIDF scheme:
  - ±20 px deadband (output 0),
  - 0.3 creep up to 70 px,
  - 0.6 cruise beyond 70 px.
- Its sign was the opposite of the bench-validated mapping: tx > 0 gave
  `+magnitude`, which with `Rotate::INVERTED = false` is a pulse above
  1500. That sign had always been marked UNVERIFIED.
- dev also still had the older deadband bench.
- Merging the whole Shooter branch into dev conflicts in six files: its
  MechLink, Telemetry and Dashboard predate the wired/wireless work. So
  only the bench files were brought over.

**Changed / Added:**
1. Commit `473d154` brings the Shooter branch's vision-rotate bench files
   into dev, unchanged:
   - `VisionRotateBenchConstants.h`, `VisionRotateBenchState.h`,
     `VisionRotateTest.cpp`;
   - the page and `analyze_vision_rotate_log.py`;
   - both tests and `.gitignore` (`__pycache__`).

   `git log Shooter..origin/dev` shows dev never modified these files after
   the fork, so nothing on dev is overwritten.
2. New `src/Control/VisionFollowController.h`. It is pure, header-only,
   built on `PidfController`, and implements the same control law and lock
   rules as `VisionRotateBenchState`:
   - Invalid target, NaN or inf → command 0 and the lock resets.
   - The first frame of a new or switched target only primes (no output).
   - A control gap > `MAX_CONTROL_GAP_MS` (100) re-primes too, so a stalled
     loop can't dump one large dt into I.
   - Ready = |tx| ≤ tolerance continuously for the settle time.
3. `TuningConstants::Rotate`:
   - `VISION_DEADBAND_PX`, `VISION_SLOW_ZONE_PX`, `VISION_CREEP_COMMAND`
     and `VISION_CRUISE_COMMAND` are removed, along with their asserts.
   - They are replaced by `VISION_kP = 0.01`, `VISION_kI = 0.01`,
     `VISION_kD = 0`, `VISION_kIZONE_PX = 20`, `VISION_kFF = 0`,
     `VISION_MAX_COMMAND = 1.0`, `VISION_MAX_INTEGRAL_COMMAND = 0.12` and
     `VISION_READY_TOLERANCE_PX = 5`, with new range asserts.
4. `ShooterConstants::Rotate::VISION_FOLLOW` composes the config, reusing
   the existing `READY_SETTLE_TIME_MS` (100) and `MAX_CONTROL_GAP_MS` (100).
   The comment now records the verified sign.
5. `Shooter.cpp`:
   - The `VISION_FOLLOW` branch is now `rotateVisionFollow.update(tx,
     isValid, targetId, millis())`, then `writeRotateCommand(command)`.
   - `rotateReady` comes from the controller.
   - `rotateControllerOutput` mirrors the command for telemetry.
   - `resetRotateController()` also resets the follow controller, so every
     mode change, stop or disable drops the lock and its integral.
   - The other rotate modes and the position PIDF are untouched.
6. `test/vision_follow_controller_test.cpp`, also added to test/README.

**Calculation:** normalized command 1.0 = the full 500 µs. `Rotate::INVERTED`
is false, so pulse = 1500 + 500·command. Each bench gain is divided by 500:

| | bench | production |
|---|---|---|
| kP | 5 µs/px | 0.01 /px |
| kI | 5 µs/(px·s) | 0.01 /(px·s) |
| I cap | ±60 µs | ±0.12 |
| output cap | ±500 µs | ±1.0 |

Examples:
- tx = 40 → −0.40 → 1300 µs.
- tx = 400 → −1.0 → 1000 µs; tx = −400 → 2000 µs.
- tx = 5 held → −0.05 − 0.12 = −0.17 → 1415 µs.
- tx = 25 → outside the zone, so I = 0 and −0.25 → 1375 µs.

**Evidence:**
- The new test pins the production gains to the bench constants divided by
  500, so they can't drift silently.
- It also checks the examples above, the ready window (90 ms not ready,
  100 ms ready, 6 px breaks it), and the reset rules.
- It runs the **production controller and the real bench state machine
  side by side** over a 28-frame approach/overshoot/hold sequence at
  10 fps: the pulses match within ±1 µs (rounding).
- All 8 PIO envs build: production, shooter_keyboard, vision_rotate,
  uart_link, elrs_link, wired_manual, drive_conveyor and manual_axes.
- All host and page tests pass: vision_follow, vision bench, pidf,
  directional_limit, position_profile, shot_table, servo_jog, rcmap,
  manual_control (three variants), conveyor, ground, and the 3 node tests.
  `git diff --check` is clean.

**Not verified / Impact:**
- Production can't run vision-follow on hardware yet. The full pin gate is
  still false, and the wired manual profiles forbid L→vision.
- The behavior is proven only through the bench (same servo) plus the
  equivalence test.
- Auto now fires only when |tx| ≤ 5 px for 100 ms. That is stricter than
  the old 20 px deadband, so lock-on takes longer but should be more
  accurate.
- Production's loop runs about every 10 ms and integrates I in wall time,
  while the bench integrates per vision frame. The I growth per second is
  the same; the equivalence test runs at 10 fps.

**Next Test:** once the production pin gate is open, check direction on
the first enable before trusting Auto: target to the right → turret turns
right, pulse below 1500. Then check that it settles at center like the
bench.

## 2026-10-14 — Claude Code: cleanup batch 1 — pins in one place, single elevation servo, elevation limits removed

**User Request:** Jeremy had three asks:
- Why is there a `RIGHT_SIGNAL_PIN` when the elevation now needs one motor?
- What are `UP_LIMIT_PIN = 52` / `DOWN_LIMIT_PIN = 53`? They aren't used.
- "我改pin就想改一個地方就好，爲什麼還要去改Pins.h？這個安全協定把它拿掉"
  (I want to change a pin in one place; drop the extra Pins.h check).

Mid-change he added: rename the elevation pin to `Angle_PIN`. The code uses
`ANGLE_PIN`, following the project's all-caps constant convention.

His decisions:
- Keep an automatic "no two outputs on one pin" check, since it never needs
  a second edit.
- Leave the `PIN_TABLE_CONFIRMED` Enable gate untouched.
- Push dev and main.

**Discussion Result / Why:**
- **`RIGHT_SIGNAL_PIN` (26)** and friends were leftovers from the old
  two-servo elevation. The second output had already been removed; the
  constants, `RIGHT_MOTOR`, `RIGHT_INVERTED`, `rightAnglePulseUs` and a
  getter that always returned 0 were not.
- **D52/D53 were a real production bug**:
  - They are legacy all_robot_017 elevation up/down limit inputs.
  - Production configured them as `INPUT` with no pull-up, but no switches
    are installed, so the pins float.
  - `limitAngleCommand()` used them through `DirectionalLimit`, so a
    floating HIGH could randomly block one elevation direction.
  - The rotate axis already had `LIMIT_SWITCHES_INSTALLED = false`; the
    angle axis did not.
  - The wired manual profiles skip sensor init, which is why this was never
    seen on the bench.
- **Pins.h held two more copies of pins:**
  - the bench "must equal 48/46/45, 24/23, 25" static_asserts, which forced
    a second edit;
  - unused legacy `#define PIN_ESC_HORI 23 / PIN_ESC_VER 24 / PIN_FALCON 25 /
    PIN_SAFE 28 / PIN_POT A3`, referenced nowhere.

**Changed:**
- `TuningConstants::Angle`: `LEFT_INVERTED` / `RIGHT_INVERTED` become
  `INVERTED` (= false, unchanged).
- `ShooterConstants::Angle`:
  - `LEFT_SIGNAL_PIN` / `RIGHT_SIGNAL_PIN` become `ANGLE_PIN = 24`.
  - `LEFT_MOTOR` / `RIGHT_MOTOR` become `MOTOR`.
  - The up/down limit pins, the `LIMIT_*` constants and the
    `UP/DOWN_LIMIT_SWITCH` configs are removed.
- `Shooter.h/.cpp`:
  - `angleLeftServo` becomes `angleServo`.
  - `leftAnglePulseUs` becomes `anglePulseUs`; `rightAnglePulseUs` is
    removed.
  - `getLeftAnglePulseUs()` becomes `getAnglePulseUs()`;
    `getRightAnglePulseUs()` is removed.
  - The angle limit-switch states, getters, init and update calls are
    removed.
  - `limitAngleCommand()` no longer applies switch limits. Encoder soft
    limits are unchanged and still off until calibrated.
  - Rotate limit handling is untouched.
- `ShooterKeyboardTest.cpp` bench: prints a fixed 0 in the three removed
  fields, so the shooter_keyboard_test page's columns don't shift.
- `Pins.h`:
  - Removed the bench pin-equality asserts, the unused `PIN_*` defines, and
    the chassis/conveyor collision asserts (superseded, see below).
  - Kept the Enable gate, `MECH_LINK_SERIAL_PORT`, and the "flywheel
    profile requires the axes profile" `#error`.
  - The header comment now says where pins live.
- `main.cpp`: one constexpr pin check, written in C++11 recursive style for
  the AVR toolchain. It covers chassis L/R, conveyor, tower trigger,
  elevation, turret and flywheel:
  - all of them must be distinct;
  - none may sit on a hardware serial pin (0/1, 14–19).

  Nothing in it repeats a pin value.

**Evidence:**
- All 8 PIO envs build.
- All host and page tests pass: vision_follow, vision bench, pidf,
  directional_limit, position_profile, shot_table, servo_jog, rcmap,
  manual_control (three variants), conveyor (two), ground, and 3 node
  tests. `git diff --check` is clean.
- Behavior checks, each reverted afterwards:
  - (A) Elevation 24 → 30, edited only in ShooterConstants:
    `mega_manual_axes_test` builds.
  - (B) Elevation = 25 (the flywheel pin): production fails with "Two
    outputs share a pin".
  - (C) Elevation = 17 (Serial2): fails with "An output is on a hardware
    serial pin".
- Not run on hardware. The only production behavior change is that the
  elevation is no longer blockable by floating D52/D53.

**Not in this batch (listed for Jeremy, needs his direction):**
- Semi-auto fire confirm reads `MechLink::fireChannel`, which is always
  released, so semi-auto never fires.
- Dead AS5600 elevation encoder / counts closed loop.
- Uncalibrated rotate pot (A3) position control and its divide-by-zero
  warning.
- `Vision::getXPred()` is computed but unused.
- About 20 bench-profile `#ifdef`s in Shooter/Dribbler.
- Superseded `tools/stm32_elrs_ground_bridge` and
  `stm32_elrs_bidirectional_test`.

## 2026-10-14 — Claude Code: shooter head has no sensors — encoder, potentiometer and limit-switch code removed

**User Request:** "POSITION_SENSOR_PIN = A3 / LEFT_LIMIT_PIN = 43 / RIGHT_LIMIT_PIN = 42 這些也不用了，發射頭上面都沒有任何sensor了，總共只有三個馬達，水平、仰角、飛輪，水平角度PIDF都是靠鏡頭來做"
(these aren't used either; the shooter head has no sensors at all, only
three motors, and turret aiming is done by the camera).

Jeremy's answers:
- Also remove the AS5600 elevation encoder and the elevation closed loop.
- Delete the old `mega_shooter_keyboard_test` bench entirely.
- **Keep** the `POSITION_SERVO` switch for a continuous-rotation elevation
  servo.

**Discussion Result / Why:** each of those three pins led into a whole
subsystem that can never run on this robot:
- **A3** fed rotate potentiometer position control: two target modes, a
  position PIDF, soft limits, and raw→degree conversion. That conversion
  was the source of the long-standing divide-by-zero warning, since it was
  never calibrated.
- **42/43** fed the limit-switch debounce/read/direction-block machinery.
- The **AS5600** fed elevation homing, the closed loop and soft limits.

The only consumer of most of it was the old keyboard bench, which existed
to test exactly these sensors.

**Changed:**
- `Shooter.h/.cpp`, rewritten small. There are three outputs and no inputs:
  - `RotateControlMode {DISABLED, MANUAL_OPEN_LOOP, VISION_FOLLOW}`.
  - `AngleControlMode {DISABLED, MANUAL_OPEN_LOOP}`.
  - The public API is what Auto, ManualControl, Telemetry and the host
    stubs use, so callers are unchanged. Removed the unused
    `set*Action`/`Action` enums, the angle target/homing functions, the
    rotate target functions, and the sensor/limit getters.
  - Behavior kept:
    - Positional elevation jog: hold the target, detach on disable.
    - Continuous elevation (`POSITION_SERVO = false`) is attached at init
      and sent neutral on disable.
    - Vision-follow is the same `VisionFollowController`.
    - Flywheel open loop with the time-based spin-up proxy.
  - **`isReady()` used to be hard-coded `false`**, which is why the
    Dashboard's "shooter ready" always showed NO. It now returns
    `rotateReady && flywheelReady`, the same two conditions Auto checks.
  - The roughly 11 bench-profile `#ifdef`s collapsed into two constexprs:
    - `DRIVES_AXES` is false only in drive+conveyor.
    - `DRIVES_FLYWHEEL` is false in drive+conveyor; the axes profile adds
      the flywheel flag.
  - All three outputs preload neutral before `attach`.
- `ShooterConstants.h`: pins, PWM endpoints and `VISION_FOLLOW` only.
  Removed: encoder, pot, limit switches, soft limits, angle/rotate/flywheel
  position PIDF, the closed-loop constants, and the unused
  `Legacy::HALF_AUTO_VERTICAL`.
- `TuningConstants.h`: Angle/Rotate/Flywheel keep only `INVERTED`, plus
  `VISION_*` and `SHOOT_COMMAND`/`SPIN_UP_MS`. Calibration placeholders,
  zero PIDF gains, `PROFILE_*` and their asserts are removed.
- `ActuatorConfig.h`: only `ServoMotorConfig` remains.
- Deleted:
  - `src/Sensors/` (AS5600Encoder, AnalogPositionSensor)
  - `lib/Seeed_Arduino_AS5600/`
  - `src/Control/DirectionalLimit.h` + test
  - `src/Control/PositionDecelerationProfile.h` + test
  - `src/Bench/ShooterKeyboardTest.cpp`, `ShooterBenchConstants.h`
  - `tools/shooter_keyboard_test/index.html`
  - the `mega_shooter_keyboard_test` env
- `platformio.ini`:
  - `mega_vision_rotate_test` is now standalone (it used to extend the
    keyboard env).
  - Dropped the `-I src/Sensors` include.
  - Dropped the unused `Ultrasonic` and `br3ttb/PID` lib_deps, which
    nothing in src includes.
- **Vision-follow bench** (`mega_vision_rotate_test`): the D43/D42 limit
  inputs, `applyLimits`, `noLimits` and `E_NO_LIMITS` are removed.
  - Telemetry goes from **`$VROTATE,6`** = v5 without the left/right-limit
    and no-limits fields.
  - The page drops the "no limits" checkbox, sends plain `E`, and its
    warning now says there are no limit switches.
  - `analyze_vision_rotate_log.py` now parses by per-version field layout,
    accepting v4/v5/v6 and rejecting count mismatches.
  - The tools README is rewritten for the vision bench only.
- Dashboard: `ROTATE_MODE_NAMES` / `ANGLE_MODE_NAMES` now match the new
  enums.
- Auto: a comment naming the removed `isAngleReady()` is updated.

**Evidence:**
- All 7 PIO envs build. A clean rebuild of production and manual_axes
  shows **no compiler warnings**; the rotate divide-by-zero and the vendor
  AS5600 warnings are gone.
- Production uses RAM 1864 B and Flash 23934 B.
- All host and page tests pass: vision_follow, vision bench (new case:
  `E_NO_LIMITS` is now `BAD_COMMAND`), pidf, shot_table, servo_jog, rcmap,
  manual_control (three variants), conveyor (two), ground, dashboard page
  and control tests, and the vision page (v6; v4/v5 lines rejected).
- The analyzer was checked on a mixed v4/v5/v6 log: 3 well-formed lines
  matched, 3 malformed lines rejected.
- `git diff --check` is clean. Not run on hardware.

**Impact:**
- The vision page and the bench firmware must come from the same commit
  (v6).
- Production behavior is unchanged except:
  - nothing reads the floating A3/42/43 inputs any more;
  - "shooter ready" can now become true.

**Next Test:** re-flash `mega_manual_axes_test` and confirm W/S/A/D, R, the
arrows, the Falcon slider, Enter and Space behave as before. Re-flash
`mega_vision_rotate_test` and confirm the page connects (v6) and follows
without the checkbox.

## 2026-10-14 — Claude Code: cleanup batch 2 (items 2–4) — vision alias, bench flags in one header, retired STM32 firmwares

**User Request:** "先做 2、3、4" (do items 2, 3 and 4 of the second cleanup list
first). Item 1 is still open and needs Jeremy's design decision: operator
mode switching plus the semi-auto fire confirm, which are unreachable from
either PC path today.

**Changed:**
- **Item 2, vision.**
  - `Vision::getXPred()` was only an alias for `tx`, and nothing called it.
    It is removed.
  - Correction to what Claude told Jeremy earlier: `updatePrediction()` is
    *not* dead prediction code. It is the function that stores each parsed
    frame. It was misnamed, so it is renamed `storeTarget()`, with a
    comment saying no prediction happens.
- **Item 3, bench flags.** New `src/Constants/BuildProfile.h` is the only
  place that tests the `TEL_*` bench macros:
  - It turns them into constexpr `WIRED_MANUAL_ONLY`,
    `PARTIAL_HARDWARE_BENCH`, `DRIVES_SHOOTER_AXES`, `DRIVES_FLYWHEEL` and
    `DRIVES_FEED_TOWER`.
  - It `#error`s on invalid combinations: a partial bench without
    wired-only, axes without drive+conveyor, flywheel without axes.
  - Users:
    - `Pins.h`: `PIN_TABLE_CONFIRMED` is the host-test override, otherwise
      `BuildProfile::PARTIAL_HARDWARE_BENCH`. The conveyor-configured
      assert now applies to every build.
    - `ManualControlConstants::WIRED_MANUAL_ONLY` aliases the profile.
    - `Dribbler.cpp` uses three plain `if`s instead of `#ifdef`s.
    - `Shooter.cpp` reads the profile.

  The `src/` tree no longer tests a bench macro anywhere else. The test
  file still selects its variants with `#if`, by design.
- **Item 4, retired firmwares.**
  - Removed `tools/stm32_elrs_ground_bridge` (replaced by
    `tools/stm32_ground_control`) and `tools/stm32_elrs_bidirectional_test`.
    The latter is a bench whose ground side sends CH4 = 1000, which
    production reads as FULL_AUTO, a hazard if it is ever paired with the
    robot firmware.
  - Also removed their only-used shared pieces: `tel_channel_map.h` and
    `tel_reassembler.{c,h}` with its host test.
  - Comments and READMEs that cited them as provenance now say they were
    retired 2026-10-14 and remain in git history.
  - The ground_control README now says to reflash any ground board that
    still carries the old firmware.
- **Comment fix** in `src/IO/RcControlMap.h`: it claimed the Battery-frame
  status path was "already proven" over this ELRS link. It is **not
  verified on hardware**; the comment now says so.

**Evidence:**
- `BuildProfile.h` combinations:
  - The 4 valid combinations compile.
  - All 3 invalid ones fail with their `#error` message.
- All 7 Mega envs build. The ground STM32 `pio run` succeeds.
- Robot bridge `build.py` checks pass. The image sha256 is unchanged
  (`99b017a4…`), because only comments changed. The path-dependent dist
  artifacts were restored rather than committed.
- Host and page tests pass:
  - crsf and tel_chunk_split (the remaining common tests);
  - vision_follow, vision bench, pidf, shot_table, servo_jog, rcmap;
  - manual_control (three variants), conveyor (two), ground;
  - three node tests.

  `git diff --check` is clean. Not run on hardware. There is no behavior
  change in any env.

**Still open, item 1:** `OperatorModeSelector` reads only wireless CH4,
which the ground station always sends as 2000 = FULL_MANUAL. The wired path
has no mode command. Semi-auto fire confirm reads `MechLink::fireChannel`,
which is always released. So Auto, the shot table and the vision-ready gate
cannot be started from the PC. This needs Jeremy's choice of mode key and
fire-confirm key.

## 2026-10-14 — Claude Code: operator modes — M switches full manual / semi auto / full auto, F fires

**User Request:** item 1 of the second cleanup list. Jeremy: "用M鍵 / 發射都是F" (M switches
modes, F always fires). He then chose:
- **Full auto fires by itself** whenever it is ready; F is needed only in
  semi-auto.
- **M can be pressed any time while armed, and disarm returns to full
  manual.** Enter, Space, link loss and timeout all go back to manual, and
  a new Enable never starts in auto.
- **The flywheel follows the shot table** by camera distance in the auto
  modes.

**Discussion Result / Why:** the auto path was dead in three places, not
only "missing a key":
1. `Auto::update()` returned whenever `ManualControl::isActive()`, but
   outputs exist only while armed. So Auto could only run when every
   output was disabled.
2. It waited for `Dribbler::getShootRemaining() > 0`, which nothing ever
   set. It was zeroed at boot.
3. The mode came only from wireless CH4, which the ground always sent as
   2000 (manual). The wired path had no mode command, and the semi-auto
   confirm read `MechLink::fireChannel`, which was hard-wired to released.

**New ownership while armed:**

| | FULL_MANUAL | SEMI_AUTO / FULL_AUTO |
|---|---|---|
| chassis, elevation, conveyor | ManualControl | ManualControl |
| turret | ManualControl (arrows / L) | Auto: vision-follow PIDF |
| flywheel | ManualControl (slider) | Auto: shot table (`SHOOT_COMMAND` 0.8 with no target) |
| F | fires the tower directly | a fire request taken by Auto |

ManualControl still owns Enable, the watchdogs and link-loss handling in
every mode.

**Changed:**
- `RcControlMap.{h,c}` (shared):
  - CH4 levels: 2000 / 1750 / 1500 = full manual / semi auto / full auto.
  - `rcmap_mode_from_us` is **fail-safe**: ≥1875 manual, ≥1625 semi,
    ≥1250 auto, and anything below 1250 (including 0) is **manual**.
  - `rcmap_encode` now sends the command's mode; previously it was always
    2000.
  - Status: the 16-bit event-code field carries **mode+1 in its top 4
    bits**, so 0 means not reported and older firmware never reads as
    "full auto". The robot bridge forwards those 16 bits unchanged, and
    **its image sha256 is unchanged (`99b017a4…`), so no reflash is
    needed**.
  - New events, appended to the end of the list: `MODE_FULL_MANUAL`,
    `MODE_SEMI_AUTO`, `MODE_FULL_AUTO`, `FIRE_NOT_READY`,
    `ENABLE_REJECTED_NOT_MANUAL`.
- `OperatorMode.h`: the enum values are now the `RCMAP_MODE_*` ids. The
  `OperatorModeSelector` and `OperatorMode.cpp` are removed.
- `ManualControl`:
  - Owns `operatorMode`. Wired `M` (armed owner only) cycles the mode;
    while disarmed it gets `REJECTED_DISARMED`. Wireless takes the level
    from CH4 while the wireless side is the owner.
  - On the wired-manual benches, M stays manual (like L).
  - Every mode change zeroes the manual turret and flywheel commands and
    any pending F.
  - `clearCommands()` (used by Enable and disarm) resets the mode to manual.
  - A **wireless Enable whose CH4 isn't manual is refused**
    (`ENABLE_REJECTED_NOT_MANUAL`).
  - `applyOutputs()` skips the turret and flywheel outside full manual.
  - New public API: `getOperatorMode()` (manual when disarmed),
    `takeFireRequest()` and `reportEvent()`.
  - The status line uses `rcmap_status_code(event, mode)`.
- `Auto`, rewritten:
  - Runs only when `getOperatorMode() != FULL_MANUAL`, which implies armed.
  - With a camera: turret vision-follow, flywheel from the shot table.
  - States are SPINNING_UP → READY (`Shooter::isReady()`: turret locked ≤5
    px for 100 ms and flywheel spun up) → fire one `Dribbler::requestFeed()`
    → COOLDOWN (`Auto::COOLDOWN_MS` 300; the tower's 1000 ms lockout also
    applies).
  - Full auto fires whenever READY. Semi-auto fires only on F while READY;
    otherwise it reports `FIRE_NOT_READY` and the press is not queued.
  - With no camera link, the turret is disabled, the flywheel is 0, and F
    gets `FIRE_NOT_READY`.
- `Dribbler`: removed the dead shot queue and feed gate (`setShootRequest`,
  `getShootRemaining`, `setFeedAllowed`, `DribblerConstants::Request`).
  `main.cpp` no longer calls it.
- `MechLink`: removed `fireChannel`.
- `TuningConstants`: removed `Auto::FEED_DURATION_MS`,
  `FIRE_BUTTON_THRESHOLD` and the `OperatorMode` thresholds, which moved to
  RcControlMap. `Auto::COOLDOWN_MS` is re-documented.
- Telemetry slot 13 (formerly the always-0 shot-queue count) is now
  **mode+1**: 1 full auto, 2 semi, 3 manual, 0 not reported.
- Ground station:
  - `M` cycles CH4 and zeroes the flywheel channel; `$GROUND_EVENT,MODE_*`.
  - `release()` (X, PC timeout, oversize) and `E` reset CH4 to manual.
  - Disarmed M gets `REJECTED_NOT_ENABLED`.
  - `RobotStatusTracker` masks the event from the code, and `$WIRELESS`
    gains a 10th field, `mode` (-1 = not reported).
  - README protocol table updated.
- Dashboard:
  - M key: one press, no auto-repeat.
  - A mode metric replaces the old "balls left", and the control badge
    shows the mode.
  - The Falcon slider is disabled and ignored outside full manual and
    resets to 0 on a mode change.
  - The mode comes from TEL slot 13 (minus 1), `$WIRELESS` field 10, and
    the `MODE_*` events.
  - Disarm shows full manual. The hint text documents M.

**Evidence:**
- All 7 Mega envs build. The production clean build shows **no warnings**
  (RAM 1977 B, Flash 24210 B).
- The ground STM32 `pio run` succeeds.
- Bridge `build.py` passes and the image is unchanged (path-dependent dist
  artifacts restored).
- Host tests, all passing. The manual_control build now includes the real
  `Auto.cpp`, with stubs for `Shooter::isReady`, `Vision::isValid` and
  `getDistance`.
  - `manual_control`, three variants. New cases:
    - wired M cycle, disarmed refusal, reset on X, Enable and watchdog;
    - the slider ignored in auto;
    - semi-auto F refused until ready, then exactly one shot, then refused
      during cooldown, and no shot without F;
    - full auto: ~2–3 shots in 2.5 s paced by the tower lockout, none
      without lock, none after M back to manual;
    - no camera: no turret, no flywheel, no shots;
    - shot-table flywheel at 5 m;
    - wireless CH4 mode follow, F via Auto, arm low resets to manual,
      Enable with CH4 = auto refused;
    - status mode bits;
    - on the wired-manual bench, M stays manual.
  - `rc_control_map`: mode levels and boundaries, the fail-safe low values,
    encode/decode, status packing including old-firmware → unknown, a
    Battery round trip, and every event having a name.
  - `ground`: the M cycle and resets, mode in `$WIRELESS`, event masking,
    and old-firmware → -1.
  - The rest: conveyor (two variants), vision_follow, vision bench, pidf,
    shot_table, servo_jog, crsf, tel_chunk_split, and the three node
    tests. The page test adds M, the mode display, the slider lock,
    old-firmware slot 13 = 0 not shown as "FULL AUTO", and the
    `$WIRELESS` mode.
- **Mutation checks** (each reverted):
  - Letting semi-auto fire without F fails `testSemiAutoFiresOnlyWhenReady`.
  - Removing the mode reset on disarm/Enable fails `testModeKeyWired`.
- `git diff --check` is clean. **Not run on hardware.**

**Impact / not verified:**
- Behavior in full manual is unchanged.
- The auto modes cannot run on hardware yet: the production pin gate is
  still false and the wired benches lock M to manual.
- Production firmware, the ground firmware and the Dashboard should be
  updated together. Mismatched versions show the mode as unknown; they
  never show it as "full auto".
- The Battery-frame uplink is still unverified on hardware.
- Shot-table values and `SHOOT_COMMAND` are placeholders.
- The elevation stays manual in auto, since it has no sensor; the shot
  table's angle is not applied.

**Next Test** (once the pin gate is open, wheels and turret clear, no balls
first):
1. M cycles and the Dashboard shows each mode.
2. Semi auto: the turret follows, the flywheel spins to the table value,
   F before lock gives `FIRE_NOT_READY`, F after lock fires once.
3. Full auto fires about once per second while locked.
4. Enter, Space and pulling the USB cable each stop it and return to full
   manual.

## 2026-10-15 — Claude Code: elevation servo pulse range 1000–2000 → 500–2500 µs (provisional)

**User Request:** the elevation servo turns only ~90° under the program, ~180° under an
older Arduino sketch, and ~270° on a standalone servo tester. Jeremy chose
"先開到 500–2500 µs，慢慢測完再縮": open it to 500–2500 µs, then narrow it after
measuring.

**Why:**
- `Angle::MIN/MAX_PULSE_US` were 1000/2000. Both the jog target and
  `Servo::attach()` clamp to them.
- The servo spans ~270° over 500–2500 µs, the tester's range. The old
  sketch was most likely `servo.write(0..180)`, which is 544–2400 µs.

**Changed:**
- `ShooterConstants::Angle`: `MIN_PULSE_US` 500, `MAX_PULSE_US` 2500.
  Neutral and the initial 1500 are unchanged, and so is the jog rate
  (1000 µs/s, so the full range takes 2 s).
- Checked in the AVR Servo source: `attach(pin, 500, 2500)` stores 11/−25
  in its int8 fields → `SERVO_MIN()` 500, `SERVO_MAX()` 2500, so there is
  no wrap.
- TEL gains **slot 28 = elevation target pulse**, appended so older parsers
  are unaffected. The Dashboard shows it as "elevation pulse / 仰角脈寬"
  for finding the mechanism's ends.

**Risk:** there is no elevation limit switch. If the mechanism travels less
than the servo, the servo can drive into a hard stop and stall.
These are the servo's limits, **not** the mechanism's; narrow MIN/MAX to
the measured ends.

**Evidence:**
- production, manual_axes, wired_manual and drive_conveyor build.
- Dashboard page test passes (new slot-28 case) and so does the control
  test; servo_jog passes.
- Not run on hardware.

**Next Test:** run `mega_manual_axes_test` and jog up/down slowly. At each
real mechanical end, before it binds, note the Dashboard pulse. Then set
MIN/MAX to those values.

## 2026-10-15 — Claude Code: elevation commanded as an absolute angle (branch claude/elevation-angle, not on dev yet)

**User Request:** a teammate (何大工程師) pointed out the elevation is a
positional servo, so it "should directly control a target angle" instead of
the hold-to-move jog. Jeremy chose:
- a Dashboard slider plus a number box, with the arrow keys as fine
  adjustment;
- auto modes take the elevation from the shot table;
- servo degrees first, mechanism degrees once measured.

Mid-way he asked to push only the 500–2500 µs version first (`eb78970`,
pushed to dev). This work therefore sits on its own branch,
`claude/elevation-angle`, until he approves it.

**Changed:**
- New pure `src/Control/ServoAngleMap.h`: a two-point linear degrees↔pulse
  map with a pulse clamp, and an ascending min/max angle whichever way the
  servo is mounted. Added `ServoPositionJog::set()` for an absolute,
  clamped target; NaN keeps the old target.
- `ShooterConstants::Angle`:
  - `CAL_A`/`CAL_B` (provisional servo spec: 500 µs = 0°, 2500 µs = 270°)
    form `ANGLE_MAP`.
  - `STEP_DEGREES` = 1.
  - **`MECHANISM_CALIBRATED = false`.** Auto applies the shot table's angle
    only once this is true, because the table's angles are elevation
    angles and would mean nothing as servo angles. Until then the operator
    keeps the elevation in every mode.
- `Shooter`: `setAngleTargetDegrees`, `stepAngleTargetDegrees` and
  getters for the target and the min/max degrees. On a continuous-rotation
  elevation they return false.
- `ManualControl`:
  - Wired `ANGT,<deg>` (go to) and `ANGS,<n>` (n steps). The usual
    armed/owner checks apply, bad numbers get `BAD_COMMAND`, and both
    refresh the watchdog.
  - When Auto owns the elevation (auto mode and calibrated), they get
    `REJECTED_AUTO_MODE` (new event, appended), and `applyOutputs` skips
    the elevation.
  - The old `ANG` jog command is still supported and is what wireless CH7
    uses.
- `Auto`: sets the elevation from `shot.angleDegrees` when calibrated and a
  target is valid; otherwise it holds.
- TEL appends slots 29–31: target °, min °, max °.
- Dashboard:
  - Elevation slider (its limits come from the TEL range), a number box
    with a "設定仰角 / Set" button (Enter stays the stop key), and the
    target ° shown alongside the pulse.
  - **↑/↓ now send `ANGS,±1` per press and per key-repeat** instead of
    holding `ANG`. The heartbeat no longer resends an angle axis.

**Evidence:**
- All 7 Mega envs and the ground board build. The bridge image is
  unchanged.
- New `test/servo_angle_map_test.cpp`: the provisional and an inverted
  "real" calibration, the clamp, a round trip, `set()` and hold.
- manual_control has a new elevation test: disarmed refusal, ANGT/ANGS
  values, bad input refused, the watchdog kept alive, and the operator
  keeping the elevation in auto while uncalibrated. All three variants
  pass.
- Dashboard page test (arrow steps, slider/box `ANGT`, an empty box sends
  nothing, slots 29–31) and control test pass.
- All other host tests pass. Not run on hardware.

**Next:**
1. Jeremy measures the elevation ends with `eb78970`.
2. Then two known elevation points → `CAL_A`/`CAL_B` and `MIN`/`MAX`
   pulses → `MECHANISM_CALIBRATED = true`.
3. Then merge this branch.

## 2026-10-15 — Claude Code: elevation angle merged into dev, with the Dashboard falling back to the jog for older firmware / wireless

**User Request:** "合併到 main，然後dev改角度" (merge into main, then switch dev to the
angle version).
- main is at `49be790`, a merge of `eb78970` (500–2500 µs plus the pulse
  display).
- dev was fast-forwarded to `d19465f` (absolute elevation angle).

**Found before pushing dev:** GitHub Pages deploys the Dashboard from dev,
so the angle page would also be used against main's firmware.
- That firmware doesn't know `ANGS`, so the arrows would have done nothing.
- The ground station doesn't know `ANGT`/`ANGS` either, and its CH7
  carries only up/stop/down, so wireless elevation would also have broken.

**Changed (Dashboard only):** `absoluteElevation` is true only while the
USB-connected Mega reports TEL slots 29–31.
- When true: ↑/↓ send `ANGS,±1`, and the slider, number box and Set
  button are enabled.
- Otherwise (older firmware, or any `$WIRELESS` line from the ground
  station): the previous hold-to-move `ANG,±1/0` jog is used, including the
  heartbeat resend, and the angle widgets are disabled.
- Disconnecting resets to the jog.
- The `angleCommand`/`sendAngleIfChanged` jog path is restored for that
  fallback.

**Evidence:**
- Page test covers: the jog without angle telemetry, ANGS after TEL slots
  29–31, the slider and box, and the `$WIRELESS` switch back to the jog.
- Control test covers `angleCommand` again.
- production and manual_axes builds pass (firmware is unchanged in this
  commit). Not run on hardware.

## 2026-10-15 — Claude Code: shot table stores the elevation pulse; arrows back to hold-to-move

**User Request:**
- The camera is fixed on the turret, so it doesn't tilt with the elevation.
  ty therefore gives no elevation feedback, and a ty-PIDF elevation is
  impossible.
- Jeremy approved: "射擊表的仰角欄從「角度」改成「脈寬（µs）」，自動模式直接用查到的脈寬"
  (the shot table's elevation column becomes a pulse in µs, and auto uses
  the looked-up pulse directly) and "dev 的上下鍵改回「按住移動」，拉條和數字框保留"
  (on dev the arrows go back to hold-to-move; keep the slider and number
  box).

**Why:** the positional servo's pulse *is* its position. Test shots record
the Dashboard's pulse directly, which needs no angle calibration and adds
no conversion error.

**Changed:**
- `TuningConstants::ShotTable::Entry` is now
  `{distanceMeters, flywheelCommand, elevationPulseUs}`.
  - The table's elevation column holds placeholders (all 1500).
  - **`ELEVATION_MEASURED = false`**: Auto applies the table pulse
    (`Shooter::setAnglePulseUs`, new and clamped) only once it is true, so
    placeholder pulses can never move the real elevation.
  - `ManualControl::autoOwnsElevation()` uses the same flag.
- `ShotTableLookup` now interpolates `elevationPulseUs`.
- Removed `Angle::MECHANISM_CALIBRATED`, `STEP_DEGREES`,
  `Shooter::stepAngleTargetDegrees`, and the wired `ANGS` command, which
  is now `UNKNOWN_COMMAND`.
- `ANGLE_MAP` stays only for the Dashboard's degree readout and the
  slider/number box (`ANGT`).
- Dashboard:
  - ↑/↓ are back to the hold-to-move `ANG` jog on every firmware and over
    the radio.
  - The slider, number box and Set button (`ANGT`) are enabled only with
    angle-capable USB firmware (TEL 29–31).
  - The label shows the angle and the pulse.

**Evidence:**
- All 7 Mega envs and the ground board build.
- `shot_table_lookup_test` was updated to pulses (interpolation 2.5 m →
  1700 µs, 3.25 m → 1550 µs).
- manual_control elevation test passes, covering:
  - ANGT, bad input, and ANGS now unknown;
  - the ANG jog unchanged;
  - in auto with a target in view and the table unmeasured, **no
    elevation pulse set**.
- **Mutation:** forcing Auto to apply the table regardless fails that
  test.
- Dashboard page test covers: arrows are `ANG` with and without angle
  telemetry, no `ANGS`, and the widgets' enable state.
- All other host and page tests pass. Not run on hardware.

**Next:**
1. Test shots per distance: record the Dashboard pulse and the Falcon %,
   and fill in `TABLE`.
2. Set `ELEVATION_MEASURED = true`.

## 2026-10-16 — Claude Code: gamepad support in the Dashboard + single-motor gamepad bench (A5)

**User Request:** test whether an Xbox-protocol pad (a 冰原狼 Xbox-layout pad
now; a flight stick at competition) together with the PC keyboard can
drive motors.

Jeremy's spec:
- One stick: Y = forward/back, X = turn.
- Fire is the flight stick's top button; on the Xbox pad it is Y for now.
- Everything else stays on the keyboard. RT for the flywheel was dropped.
- First test: a single continuous servo/ESC on **A5**, X axis = left/right
  turn, at full ±500 µs.

**Added:**
- `src/Bench/GamepadMotorTest/GamepadMotorTest.cpp` and env
  `mega_gamepad_motor_test`. It is standalone and drives only A5.
  - It speaks the Dashboard's wired protocol: E/X/K/T. FLY/ROT/ANG/F/L/R/M
    are accepted to keep the link alive but drive nothing.
  - Events: OUTPUTS_ENABLED / DISABLED / COMMAND_TIMEOUT /
    REJECTED_DISARMED / BAD_COMMAND / UNKNOWN_COMMAND /
    DISABLED_CORRUPT_COMMAND.
  - A TEL line in the production 28-field layout: armed in 18, the A5
    pulse in 19, owner = wired in 21, mode+1 = 3.
  - Motor pulse = 1500 + 500·(right − left)/2. That gives D or stick right
    → 2000, A or stick left → 1000, and forward alone → 1500.
  - 250 ms watchdog. The output is preloaded at 1500 before attach and
    forced to 1500 when disarmed.
- `test/gamepad_motor_test.cpp` compiles the bench .cpp as-is against
  `test/host_stubs`. It covers: disarmed refusal; D, A, half, forward-only,
  mixed and clamped turns; the 28-field TEL; keepalive via K/FLY; the
  watchdog; bad and unknown input; an oversized line; and X.
- Host stubs:
  - `HostSerial` gains `begin` and int/long/char print overloads.
  - `A5` is defined.
  - The Servo stub's `attach(pin, min, max)` now records the pin.
  - Leftovers from the removed sensors are dropped: the `A3` define and
    `AS5600.h`.
  - The stale WiredControl comment is updated.
- Dashboard: Gamepad API polling every 20 ms. A "gamepad / 手把" metric
  shows the profile, the pad id, the raw X/Y values and the indices of
  pressed buttons, which is for mapping the flight stick later.
  - Profiles:
    - `standard` (Xbox layout): left stick axes 0/1, fire = button 3 (Y).
    - `other` (flight stick): axes 0/1, fire = button 0, **provisional**.
  - Deadzone 0.15 with rescale.
  - Tank mix uses the keyboard's polarity: left = fwd − turn,
    right = fwd + turn. Stick right equals D (`T,-1,1`).
  - The stick wins while it is pushed; otherwise the keyboard drives.
  - **After Enable the stick must return to center once before it drives.**
    Unplugging the pad zeroes the drive, and disarm clears it.
  - Fire sends one F per press.

**Evidence:**
- All 8 PIO envs build. The new bench uses RAM 536 B.
- gamepad_motor host test passes. The Dashboard page test adds gamepad
  cases: disarmed, center-first, analog and deadzone values, keyboard
  fallback, fire edge, unplug, the flight-stick profile, and disarm.
- All other host and page tests pass. Not run on hardware.

**Future requirements from the teammate chat (2026-10-15/16), not
started:**
1. Flight-stick testing.
2. CH5 is no longer needed (the conveyor tower runs itself). CH8 becomes a
   2-value fire signal: one value at rest, another for 100 ms on fire. The
   Mega outputs HIGH to the tower for 100 ms, and ball counting moves to
   the tower side.
3. Semi-auto target-box selection needs 9 values (keyboard numpad 3×3).
4. Box-switch logic: fine-adjust offsets are stored on the Mega, each press
   accumulates, and switching the box clears them.
5. Key remap. On the stick: fire, forward/back/left/right, "head home",
   and the two index-finger buttons.
6. Manual uses preset values and semi-auto uses the AI's values. The
   conveyor belt is controlled by the tower side.

Note that a turret "home" is impossible without a turret position sensor;
only the elevation can home, to a fixed pulse.

## 2026-10-16 — Claude Code: gamepad bench — arrows left/right also drive A5

**Hardware result:** Jeremy reports **the gamepad works**, driving the A5
motor through `mega_gamepad_motor_test`. This is the first hardware
confirmation of the Dashboard's gamepad path.

**User Request:** let the keyboard's ←/→ control the motor too. Jeremy
approved the plan.

**Changed (bench firmware only):**
- `GamepadMotorTest.cpp` now parses `ROT,<d>`, which the Dashboard's ←/→
  send (they are the turret keys on the real robot).
- While ROT is non-zero it wins over the T-turn from the stick or A/D;
  `ROT,0` hands control back. E and any disarm clear it.
- The Dashboard and production firmware are unchanged.

**Evidence:**
- The `gamepad_motor_test` host test adds these cases: → 2000, ← 1000,
  arrow overrides the stick, release goes back to the stick (1750), a bad
  ROT value is refused, and X / E clear a held arrow.
- The env builds and the page test passes. The ←/→ path itself has not
  been run on hardware.

## 2026-10-16 — Claude Code: control table v2, phase 1 (channels + keys + fire)

**User Request:** implement the team's competition control table in
phases. Jeremy approved "分階段：先通道＋按鍵＋發射".

The spec:
- One stick: Y = forward/back, X = turn.
- Fire: the flight stick's top button (Xbox Y for now).
- Keyboard:
  - Enter = disable; Space = stop and disconnect.
  - 1–9 = the 3×3 holes: 1 left-bottom, 2 left-mid, 3 left-top,
    4 mid-bottom, 5 center, 6 mid-top, 7 right-bottom, 8 right-mid,
    9 right-top.
  - Arrows = elevation / turret fine adjust.
  - M = mode.
  - Q W E R = auto paths left 1, left 2, right 1, right 2.

**Discussion Result (Jeremy's answers):**
- Arm stays, moved to CH6.
- Hole selection is recorded only. AI aiming at the chosen hole is phase 2,
  and a turret AS5600 is coming later.
- QWER: only the trigger flow for now.
- Flywheel: no wireless flywheel channel. Phase 2 drives it from the 3×3
  hole, using a shot table of hole + distance → elevation and flywheel.
- The Mega conveyor output (D45) and the R key are removed; the tower side
  drives the conveyor.
- Auto paths run "once": both once per Enable **and** a record since boot
  shown on the Dashboard.
- M order stays full manual → semi auto → full auto.

**Why the channels are laid out this way:** ELRS Hybrid mode sends CH1–4 at
full resolution every packet and CH5 as a 2-position switch every packet.
CH6 and above are 6-position values, sent round robin. So:
- the 10-level hole goes in CH3;
- fire goes in CH5, so a 100 ms press can't be missed;
- the 5-level nudge and auto-path channels go in CH7 and CH8.

`RCMAP_CHANNEL_COUNT` stays 10, so the robot bridge's `RC,…` line is
unchanged. Its image is unchanged too: sha 99b017a4…, rebuilt and
compared.

**Changed:**
- `src/IO/RcControlMap.{h,c}`: channel table v2.

  | CH | Function | Encoding |
  |----|----------|----------|
  | CH1 | forward | continuous |
  | CH2 | turn | continuous |
  | CH3 | hole | 1000+100k (k 0–9), ±40 µs |
  | CH4 | mode | 2000 / 1750 / 1500 |
  | CH5 | fire | ≥1750 µs = on |
  | CH6 | arm | ≥1750 µs = on |
  | CH7 | nudge | 1000+200k (center/up/down/left/right), ±90 µs |
  | CH8 | auto | 1000+200k (none/1–4), ±90 µs |

  - The tolerances cover ELRS's 988/1193/1398/1602/1807 steps.
  - A frame with a value between levels is rejected, never guessed.
  - The counter API is removed. New events: `TARGET_SELECTED`,
    `AUTO_PATH_STARTED`, `AUTO_PATH_REJECTED`. `EVENT_SKIPPED` and
    `CONVEYOR_*` are kept as retired slots, so the codes don't shift.
- `src/IO/ManualControl.{h,cpp}`:
  - Wired commands added: `TGT,<0..9>` and `AUTO,<1..4>`. `R` and `L` are
    removed and are now `UNKNOWN_COMMAND`. In full manual the turret is
    always manual.
  - Wireless decode:
    - Arm, fire and auto act only on their edge (auto from none to a
      path).
    - Arcade becomes tank: left = fwd − turn, right = fwd + turn (the
      Dashboard's polarity).
    - Nudge drives an angle or rotate jog.
    - No wireless flywheel.
    - An RF outage re-arms all three edge detectors, so nothing held
      through it is replayed.
  - `AUTO` is refused (`AUTO_PATH_REJECTED`) for an invalid path, on the
    wired-only benches, and when a path has already run in this Enable.
  - The used-path mask is kept until reboot.
- Conveyor removed:
  - Gone from `DribblerConstants`, `Dribbler`, the Pins assert and the
    pincheck.
  - `test/conveyor_esc_test.cpp` deleted.
  - Env `mega_drive_conveyor_test` renamed to `mega_drive_test`, and flag
    `TEL_DRIVE_CONVEYOR_ONLY` to `TEL_DRIVE_ONLY`.
- `MechLink` failsafe values are v2: hole, fire, arm, nudge and auto =
  1000; mode = 2000 (manual).
- Telemetry: TEL slots 7–11 are now forward, turn, mode, arm and fire.
  Appended slots: 32 = hole, 33 = auto path this Enable, 34 = paths-used
  mask (bit n = path n).
- Ground STM32 (`GroundCommandState`):
  - `T` becomes forward = (l+r)/2, turn = (r−l)/2.
  - ROT / ANG become nudge; elevation wins if both are held.
  - `F` holds fire for 100 ms.
  - `TGT` sets the hole, which is held after the key is released.
  - `AUTO` sends the path for 200 ms; a press during that pulse is
    ignored.
  - FLY / ANGT are accepted but have no channel.
- Dashboard:
  - Keys 1–9 send `TGT,n`; Q W E R send `AUTO,1..4`; neither
    auto-repeats.
  - **The keyboard no longer drives or fires.** WASD, F, R and L are
    removed (W/E/R/Q are now auto-path keys). The stick drives and its
    button fires. Arrows, M, Enter and Space are unchanged.
  - New cards: "target hole", with a 3×3 grid drawn left column = 1–3
    from the bottom, and "auto path" (this Enable + used).
  - On USB the cards come from TEL 32–34. Through the ground station there
    is no such telemetry, so the page shows what it sent once the robot's
    event comes back. Its record is labelled "本頁送出紀錄" (page-sent
    record), not "since boot".
  - The MechLink table relabelled.
- Benches: the ELRS link bench prints the v2 fields. The gamepad bench
  accepts TGT / AUTO and no longer lists L / R.
- Docs: the ground README protocol table and test/README.

**Calculation (examples):**
- Hole 7: CH3 = 1000 + 100·7 = 1700 µs → CRSF (1700−1500)·8/5 + 992 =
  1312.
- Nudge LEFT (k=3): 1600 µs, which ELRS 6-position sends as 1602
  (error 2 ≤ 90).
- Stick full right with no forward: turn = +1 → left = −1, right = +1,
  the same as `T,-1,1`.

**Impact:**
- Ground and robot must be flashed together: v1 and v2 channel tables are
  incompatible, so a v1 ground's frames are rejected as corrupt and the
  robot watchdog disarms it.
- The Dashboard must be the new version for keys 1–9 / QWER.
- Unchanged: the wired protocol's T / ROT / ANG / ANGT / FLY / F / M, the
  M order, Auto's behavior, and the PIN_TABLE_CONFIRMED gate.

**Evidence:**
- All 8 PIO envs build. The ground pio build passes. The bridge
  `build.py` sha is unchanged and its dist artifacts were restored.
- `manual_control_test` was rewritten for v2 and passes in three
  variants: confirmed, unconfirmed gate, and wired-manual (the
  wired-manual one needs `TEL_HOST_TEST_PIN_TABLE_CONFIRMED`; this is
  older behavior and test/README now says so). Coverage includes:
  - TGT / AUTO refused while disarmed, bad values, same hole twice gives no
    second event;
  - one auto path per Enable, the mask kept after X / E and cleared only by
    init;
  - the wired-only bench refuses AUTO;
  - forward/turn mix and clamping, all four nudge directions, no wireless
    flywheel;
  - fire held = one shot, fire held through an outage is not replayed,
    fire while disarmed does not latch;
  - wireless target and auto edges, auto held through an outage does not
    start;
  - a corrupt CH3 times out; authority for fire / auto.
- The rc_control_map, ground host test, gamepad, Dashboard page/control
  tests and every other host test pass. A browser check of the cards
  looked right.
- **Nothing was run on hardware.**

**Next Test:**
1. Flash the ground and the robot Mega together (`mega_elrs_link_test`
   first).
2. With the motors off, press the stick button, 1–9 and QWER. Expect
   `FEED_TRIGGER_SENT`, `TARGET_SELECTED` and `AUTO_PATH_STARTED`, and the
   second QWER in an Enable should give `REJECTED`.
3. Then, wheels off the ground, check the stick direction and the
   arrows.

**Phase 2 (not started):**
- Semi-auto AI lock on the chosen hole (Orin protocol, with Codex/YOLO).
- Per-hole shot table (hole + distance → elevation and flywheel).
- Fine-adjust offsets stored on the Mega, cleared on a hole change.
- The auto paths themselves (need localization).
- The flight stick's real fire-button index.
- The turret AS5600.


## 2026-10-16 — Codex: approved operator selection, Orin protocol v2

**User Request / Discussion Result:** Jeremy said 「那你可以開始改了」 after
reviewing Claude's scope lock. Implemented only selected-hole transport/override;
recognition, tracking/memory, depth algorithms and config values unchanged.

**Changed / Added / Why:** YOLO_Detect_single serial_tx.py stores selected_target_id
from strict MEGA_TARGET,2,0|4..12 (invalid commands preserve prior choice; not a
heartbeat). target_manager.select gains optional selected_target_id=0; nonzero
requires exactly one matching reliable hole, otherwise None, without auto fallback.
Override leaves automatic state untouched, bypasses auto ranking/red priority/
shot budget/stable-frame delay. Zero executes original code. vision_main polls
serial in standalone serial mode too and passes selection to manager. Its main()
explicitly constructs protocol_version=2; config and constructor legacy default
remain 1 to preserve existing parameter values. Deploy matching Mega v2.
Added docs/operator-selection-v2.md and tests/test_operator_selection.py; recovery
fake link now models enabled and selected_target_id. No new runtime dependencies.

**Flow / Calculation:** Mega converts key→vision only once, e.g. key1→10;
Orin receives MEGA_TARGET,2,10→matches id10→existing depth resolver→five-field
packet. tx=cx-width//2, ty=cy-height//2, pixels right/down positive; e.g.
(250,350),1000x800 gives -250,-50. Distance mm. Missing selected target outputs
0,0,0,0,0. Depth-only still invalid. Webcam's existing valid=1,distance=0 remains
bench-only behavior; Mega competition must gate usable distance. Selection held
until valid replacement/0 (including reconnect); production --wait-for-mega and
Mega returned-id checks remain required. 100ms selection resend is Mega-owned;
300ms numeric timeout Mega-owned, existing Orin heartbeat timeout 1s unchanged.

**Evidence:** PYTHONDONTWRITEBYTECODE=1 venv/bin/python -m unittest discover -s tests:
54 tests PASS, including original 46. git diff --check PASS. New cases cover nine
choices, fragments, malformed/version mismatch, heartbeats, missing/ambiguous/
unreliable targets, auto-state preservation, depth-only gate, full synthetic
positional grid and mocked real session output. No hardware/recording validation,
no flash, commit, push or deployment. Pre-existing handoff edits and Thumbs.db
preserved. TEL runtime not modified.

**Limits / Next Test:** IDs derive from geometric layout/anchors, not YOLO class.
Full-grid synthetic numbering passes; partial cold-start, tilt, occlusion and
flow drift are not certified. Existing automatic target-loss reset deferral remains.
Claude may implement Mega phase2 against docs/operator-selection-v2.md: statuses
v2, MEGA_TARGET vision ids, returned-id/valid/range/timeout gates. With outputs
disabled, test each physical hole, covering chosen hole, rapid switches and peer
restarts before enabling mechanisms. Do not claim competition readiness from host
suite. See YOLO_Detect_single/docs/operator-selection-v2.md for full contract.

## 2026-10-16 — Claude Code: Mega side of Orin protocol v2 (selected hole, per-hole shot tables, fine-adjust)

**User Request:** implement the Mega side against Codex's
`YOLO_Detect_single/docs/operator-selection-v2.md`. Jeremy decided:
- **B**: both semi-auto and full auto shoot the selected hole; with no hole
  selected, the AI picks.
- Fine-adjust steps for now: 5 px horizontal, 10 µs elevation.
- The 9 per-hole tables start as copies of the current table.

**Changed:**
- **Hole numbering.** `src/Vision/OperatorTarget.h` (new) is the only
  place that converts a key to a vision id:
  `4 + (2 − rowFromBottom)·3 + col`. For example, key 1 → 10 and key 9 → 6.
  `static_assert` pins all 9 values.
- **`Vision`** (protocol v2):
  - `PROTOCOL_VERSION` is now 2, so v1 lifecycle lines are refused.
  - `MEGA_TARGET,2,<id>` is sent right after `MEGA_READY`, immediately on
    every change, and with every heartbeat (100 ms).
  - `setSelectedTarget()`: changing the selection drops the frame in hand
    at once.
  - `storeTarget()`: while a hole is selected, a frame for any other id is
    `valid=false`. The link itself still counts as connected.
- **`main.cpp`**: after `ManualControl::update()` it calls
  `Vision::setSelectedTarget(visionIdForHole(getTarget()))`, so a switch
  takes effect before `Auto` runs in the same loop.
- **`Auto`**:
  - `hasTarget` = valid **and distance > 0**. Without a distance it can
    aim but `ready` is false, so it never fires. This covers the webcam
    case of valid=1 with distance 0.
  - The flywheel comes from `ShotTable::tableForHole(target)`.
  - Elevation = table pulse + elevation offset, but only once
    `ELEVATION_MEASURED` is true. It is still false.
  - `Shooter::setRotateAimOffsetPx(aim offset)`.
- **`Shooter`**: vision-follow takes `tx + aimOffsetPx` as its input. A
  positive offset makes the turret aim to the right of the hole. Example:
  R = +10, and the loop locks when the target is at tx = −10. `stopAll`
  clears it.
- **`ManualControl`**, fine-adjust:
  - In semi-auto and full auto, one arrow press adds one step. A press is
    the direction changing to non-zero, from 0 or straight to the opposite
    side. Wired ROT,1 resent every heartbeat and wireless CH7 repeated
    every frame do not add more.
  - Left/right → aim px, clamped to ±100.
  - Up/down → elevation µs, clamped to ±200, with the same direction as
    `ANG,1`. Up/down only change Auto's elevation **once Auto owns it**;
    until then they stay the manual elevation jog.
  - A new hole, Enable and disarm all clear the offsets. A key still held
    after a clear must be released before it counts again.
  - Full manual: no change, the arrows are still hold-to-move.
- **`TuningConstants`**:
  - `ShotTable::HOLE_1..HOLE_9` plus `BY_HOLE` / `tableForHole()`. All are
    placeholder copies of `TABLE`.
  - `Auto::AIM_OFFSET_*` and `ELEVATION_OFFSET_*`.
- **Telemetry**: appended slots 35 = vision id sent to the Orin,
  36 = aim offset px, 37 = elevation offset µs.
- **Dashboard**: the target-hole card has a new line: "視覺 #id 已看到 ✓ /
  看不到 · 微調 水平/仰角", or "AI 自選" (the AI picks) when no hole is
  selected.
- **`mega_vision_rotate_test` bench**: `PROTOCOL_VERSION` is now 2. It sends
  no `MEGA_TARGET`, so the Orin picks on its own. Without this change, the
  bench would also stop working once the Orin is updated.

**Added tests:**
- `test/operator_target_test.cpp`: key → id for all 9 holes, and the
  frame-match rule.
- `test/vision_link_test.cpp`: runs the **real Vision.cpp** with
  `test/vision_stubs/Arduino.h`. Covers:
  - READY / HEARTBEAT / TARGET packet order;
  - v1 refused;
  - a switch drops the current frame;
  - another id or a bonus hole is invalid;
  - `0,0,0,0,0`;
  - stale frames;
  - resending to a restarted Orin.
- `manual_control_test`, new cases: no distance means no fire, each hole's
  own table, wired and wireless fine-adjust (step, hold, change of
  direction, clamp, clear on hole switch, a held key needs a release,
  disarm). The fire tests now place a target with a distance in view.

**Evidence:**
- All 8 Mega envs build. megaatmega2560 RAM went from 2073 to 3547 B (43%);
  the 1.4 KB added is the per-hole tables in SRAM, and they can move to
  PROGMEM if that ever gets tight.
- Ground pio builds. The bridge is unaffected because RcControlMap did not
  change.
- All host and page tests pass.
- **Not run on hardware.**

**Impact:** **The Orin (YOLO_Detect_single with Codex's v2 changes) and this
Mega firmware must be deployed together**, because a v1/v2 mix refuses the
other side and vision stops working. Unchanged: the turret PIDF gains,
`ELEVATION_MEASURED = false`, and full-manual behavior.

**Next Test** (outputs off or motors disconnected, Orin and Mega flashed at
the same time):
1. Press keys 1–9 in turn. The Dashboard should show the matching vision
   id (#10 / #7 / #4 …). The Orin should frame that physical hole and show
   "已看到 ✓".
2. Cover the selected hole. It should show "看不到", with no fallback to
   another hole.
3. Switch holes quickly, and restart the Orin and the Mega each once.
4. Then, with outputs on and in semi-auto, check the direction of the
   left/right fine-adjust: pressing → should make the turret aim to the
   right of the hole.

## 2026-10-16 — Claude Code: shot tables by height (3 rows + bonus), extrapolation beyond 8.5 m

**User Request / Discussion Result:** Jeremy asked how to lay out the table
for "every hole has a different elevation and speed, and those change with
distance". Agreed with Jeremy, including after he checked the diagram:
- The trajectory depends on the hole's height and the distance to it, so
  the three holes of a row share one table.
- An angled shot needs no extra table. The Orin's `DEPTH_DISTANCE_MODE =
  "range"` measures the straight-line distance from the camera to the
  selected hole, which is already longer at an angle.
- Each hole gets one elevation trim.
- Tables run from 2.5 m, because the rules put the barrier 250 cm from the
  net.
- The bonus holes are at a different height, so they get their own table
  (Jeremy: **A**).
- Beyond 8.5 m, predict along the 8.0 → 8.5 trend (Jeremy).

**Changed (supersedes the 9 per-hole tables from 95632be):**
- `TuningConstants::ShotTable`: `BOTTOM_ROW` / `MIDDLE_ROW` / `TOP_ROW` /
  `BONUS_ROW`.
  - Each table is 2.5–8.5 m in 0.5 m steps (13 entries) of {flywheel,
    elevation pulse}. All four are the same placeholder for now.
  - `ELEVATION_TRIM_US[13]` is indexed by Orin vision id, and the comments
    give the matching keys.
  - `tableForVisionId()`:

    | Vision id | Table | Keys |
    |---|---|---|
    | 10–12 | bottom | 1 4 7 |
    | 7–9 | middle | 2 5 8 |
    | 4–6 | top | 3 6 9 |
    | 1–3 | bonus | — |
    | 0 / unknown | none | — |
  - Approximate heights read from the rules drawing (to be measured on the
    field): 120 / 180 / 240 / 290 cm.
- `ShotTableLookup::lookup`:
  - Beyond the last entry it extrapolates linearly along the last segment.
    The flywheel stays within 0–1; the elevation is limited by
    `Shooter::setAnglePulseUs` to MIN/MAX.
  - Below the first entry it clamps to that entry.
  - A null table is a no-op.
- `ShotTableLookup::lookupTarget(id, m, out)` = the id's height table +
  that hole's trim. It returns false when the id has no table.
- `Auto`: the table is chosen from the **frame's id**. With a hole
  selected, Vision only passes that hole's frames; with none selected, it
  is the Orin's own pick, so the bonus holes also get the right table.
  `hasTarget` = valid && distance > 0 && the id has a table, so an id with
  no table is never fired at.

**Calculation:** placeholder table, 8.0 m = 0.96 and 8.5 m = 0.98, so
9.0 m → 0.98 + (0.98 − 0.96) = 1.00, and 9.5 m is also 1.00 because the
flywheel is capped there. The elevation uses the same trend and is capped
by the servo limits.

**Evidence:**
- All 8 Mega envs build. megaatmega2560 RAM went from 3547 to 2587 B
  (31.6%).
- `shot_table_lookup_test` was updated and now prints its result:
  extrapolation 4.5 m → 0.95 / 1300 µs, flywheel cap, null table, all four
  real tables, `lookupTarget` + trim.
- New `manual_control_test` case `testShotTableByHeight`:
  - id → table for all 12 ids;
  - every table is 2.5 + 0.5·i;
  - Auto uses the frame id's table;
  - id 0 never fires;
  - a bonus hole has a table and fires.
- All other host and page tests pass. Nothing has run on hardware.

**Next Test:**
1. Measure the actual hole heights on the field.
2. With the turret locked (Dashboard shows "已看到 ✓"), read the **camera
   distance** shown on the Dashboard. Fill each row's flywheel and elevation
   at 2.5–8.5 m.
3. Use the trims only for a single hole that is consistently off.
4. Once the elevation values are real, set `ELEVATION_MEASURED = true`.

## 2026-10-16 — Claude Code: pins confirmed (tower trigger D21), production Enable allowed

**User Request:** Jeremy gave the feed tower trigger pin as **D21**. The
other pins are the competition ones he confirmed on 2026-10-14 ("正式比賽還是一樣").

**Changed:**
- `DribblerConstants::Tower::TRIGGER_PIN` changed from 2 (a provisional
  placeholder) to 21.
- `Pins.h`: `PIN_TABLE_CONFIRMED = true`. The comment lists every confirmed
  pin:

  | Pin | Use |
  |-----|-----|
  | D48 / D46 | chassis right / left |
  | D23 | turret |
  | D24 | elevation |
  | D25 | Falcon |
  | D21 | tower trigger |
  | Serial1 D18 / D19 | Orin |
  | Serial2 D16 / D17 | ELRS bridge |
- The Enable gate code is unchanged. Setting the flag back to false
  disables Enable again.
- `TEL_HOST_TEST_PIN_TABLE_CONFIRMED` is still a hard error on Arduino. In
  the host tests, `=true` runs the armed suite, `=false` runs the refusal
  suite, and leaving it unset now behaves like the firmware (armed).
  test/README has the matching commands.

**Risk to check on hardware:** D21 is also the Mega 2560's I2C SCL line,
and the board has a 10 kΩ pull-up to 5 V on it. While the Mega is in reset
or the bootloader (power-up, upload, reset button), before
`Dribbler::init()` drives the pin LOW, the trigger line can sit HIGH. If
the tower acts on a HIGH **level**, it may feed a ball every time the Mega
resets.

Check it:
- **Multimeter:** hold the Mega's RESET button and measure D21 to GND. If
  it reads about 5 V, the risk is real.
- **Live test:** with the tower connected and no ball loaded, press reset
  on the Mega and watch whether the tower moves.

Fixes, if needed:
- On the tower side, act only on a rising edge, and ignore the first
  stretch after power-up.
- Or add a 1 kΩ pull-down from D21 to GND. That divides 5 V × 1k/11k ≈
  0.45 V, which reads LOW, and it doesn't fight the Mega's push-pull output.
- Or move the trigger to a pin without a pull-up (D20/D21 both have one).

**Evidence:**
- All 8 Mega envs build, and `Dribbler.cpp.o` / `main.cpp.o` were rebuilt
  after the header change.
- `manual_control_test` passes in four variants: true, false, unset, and
  wired-manual.
- The gamepad and Dashboard page tests pass.
- Not run on hardware.

## 2026-10-16 — Claude Code: Dashboard mode / auto-path menus, MODE command, target reset per Enable

**User Request:**
- Add two mouse-driven menus to the Dashboard:
  - **Mode**: picked before Enable, and the robot runs in that mode once
    enabled. M can still switch modes at any time.
  - **Auto path**: None / left 1 / left 2 / right 1 / right 2. This menu
    replaces the QWER keys. None means no path runs.
- Mode behavior:
  - Full auto shoots on its own.
  - Semi auto gives the chassis to the operator. The AI locks a target on
    its own at first; a digit key switches the target, and the robot waits
    for the fire signal.
  - Full manual hands everything to the operator.
- Jeremy chose "**every Enable resets the target to 0, and key 0 returns
  to the AI's pick**".
- Each step of an auto path has its own speed. The paths themselves will
  come later.

**Changed:**
- Firmware `ManualControl`:
  - New wired command `MODE,<0..2>` (0 full auto, 1 semi, 2 manual) sets
    the mode directly. The wired benches stay in full manual, the same as
    with M.
  - `requestEnable` resets `target` to 0, so every Enable starts on the
    Orin's own pick.
  - **Enable itself still always starts in full manual.** That safety rule
    is unchanged; the Dashboard sends `MODE` only after the robot confirms
    Enable.
- Ground `GroundCommandState`:
  - `MODE,<n>` sets CH4 to that level and emits
    `$GROUND_EVENT,MODE_*` (shared `emitModeEvent()`).
  - `E` resets `cmd.target` to 0. CH3 is held on the ground, so without
    this the robot would take the old hole straight back after re-enabling.
- Dashboard:
  - New "模式" and "自動路徑" menus under the Enable button.
  - Once the robot confirms Enable (`setControlArmed`: `justEnabled`),
    `applyStartMenus()` sends `MODE,n` (skipped for full manual) and then
    `AUTO,n` (skipped for None). It also resets the local target and auto
    display.
  - Changing the mode menu while enabled sends `MODE,n` immediately. While
    enabled, the mode menu follows the mode the robot reports; after a
    disarm it keeps that value as the next start mode.
  - Changing the path menu takes effect at the next Enable.
  - Both menus drop focus after a change, so arrows, Enter and Space go
    back to robot control.
  - **QWER removed.** The 0 key sends `TGT,0`. Auto-path names are now
    左一 / 左二 / 右一 / 右二.
- Auto-path content (timed steps, a speed per step) is not done yet.
  `AUTO` currently only triggers and records.

**Evidence:**
- All 8 Mega envs and the ground pio build pass.
- `manual_control_test`, new case `testModeCommandAndTargetReset`:
  - `MODE` 0/1/2;
  - bad values;
  - refused while disarmed;
  - M still cycles from the current mode;
  - the target is reset by Enable;
  - 0 returns to the AI's pick;
  - the wired-manual bench stays manual on `MODE`.
- Ground host test:
  - `MODE` levels;
  - bad values;
  - `MODE` while not enabled is refused;
  - `E` resets the target.
- Dashboard page test:
  - the menus send `E` → `MODE,1` → `AUTO,3` after the confirmation;
  - nothing is sent before the confirmation;
  - the mode menu follows the reported mode;
  - changing the mode menu while enabled sends `MODE`;
  - changing the path menu sends nothing;
  - QWER send nothing;
  - 0 sends `TGT,0`.
- A browser check of the layout looked right.
- Not run on hardware.

**Compatibility:** this Dashboard sends `MODE`, which older firmware or an
older ground station rejects as `UNKNOWN_COMMAND`. In that case the robot
just stays in full manual, which is safe. The Dashboard, the Mega and the
ground station should all be updated together.

## 2026-10-16 — Claude Code: Dashboard realtime trace and vision stream panels removed

**User Request:** Jeremy asked to remove "即時曲線" (the realtime trace) and
"串流" (the vision stream).

**Changed (`tools/dashboard/index.html` only):**
- Removed the `data-panel="stream"` panel: MJPEG URL box, image, status,
  the `tel-dashboard-stream-url-v1` localStorage key, and the `.stream-frame`
  / `.stream-placeholder` CSS.
- Removed the `data-panel="trace"` panel: `#chart` canvas, `drawChart()`,
  `drawLine()`, and the 180-frame `history`.
- `.stream-controls` stays, because the control panel's slider / button
  rows use it.
- `applyLayout()` now keeps only the panels that still exist. Without this,
  a previously saved layout would keep the removed panels' space at the
  bottom of the page.
- The tx / ty / distance readouts, the field view, the log and the other
  panels are unchanged.

**Evidence:**
- The Dashboard page and control tests pass. The test's "text box" is now
  the elevation angle input, and a new assertion checks that the two panels
  are gone.
- In a real browser:
  - demo mode updates the readouts with no console errors;
  - editing the layout with an old saved layout containing stream / trace
    prunes them correctly.

## 2026-10-16 — Claude Code: Dashboard shows the vision distance in meters

**User Request:** show the distance that vision measures on the Dashboard.

**Changed (`tools/dashboard/index.html`):**
- The Vision panel's distance card already existed. Before this change it
  showed raw mm with the note "目前單位依 YOLO 輸出", and it showed 0 when
  there was no target.
- It now reads "相機距離" and shows **meters with 2 decimals**, which is
  the shot table's unit. The line below it gives the original mm and the
  note "相機到洞口的直線距離，填射擊表用這個".
- With no valid target, or a distance of 0, it shows "--" / "看不到目標",
  never "0 m".
- The control panel's target-hole card also shows the distance:
  - "視覺 #10 已看到 ✓ 3.50 m"
  - "AI 自選 #11 3.50 m"
- New helper `distanceMeters(data)`: valid && distance > 0 ? mm / 1000 :
  null.

**Limit:** this distance comes from TEL telemetry, so it is only available
when the Dashboard is connected to the **Mega's USB**. The ground station's
wireless status uplink has no distance field, so a wireless connection
shows nothing here. Shot-table calibration therefore has to be done over
USB.

**Evidence:**
- The Dashboard page and control tests pass, with new assertions for 3.50 m
  / 3500 mm, the no-target case, and the AI-pick distance.
- In a real browser, demo mode shows "1.62 m · 1616 mm".

## 2026-10-16 — Claude Code: Dashboard mode / auto-path menus moved to their own panel

**User Request:** Jeremy sent a screenshot: "把這兩個選單單獨拉出來" (pull
these two menus out on their own).

**Changed (`tools/dashboard/index.html`):**
- The "模式" and "自動路徑" menus moved out of the Manual Control panel
  into a new `data-panel="setup"` panel, "Match Setup / 模式與自動路徑".
  It sits directly above Manual Control and can be dragged and resized
  like the other panels. The element ids and behavior are unchanged.
- This panel is naturally short (about 130 px), and `updatePanelDensity()`
  treats any panel under 210 px as tiny and shrinks its text to 10 px. The
  panel is marked `data-fixed-density` so the density pass skips it.
- With an older saved layout, the new panel is placed using the existing
  fallback.

**Evidence:**
- The page test passes, with a new assertion that both menus are in the
  setup panel and not in the control panel.
- In a real browser: panel order is vision → setup → control, and the text
  is back to normal size.

## 2026-10-16 — Claude Code: Dashboard safety panel removed; Enable / Falcon / elevation in their own panel

**User Request:** delete the safety panel. Move Enable, the speed slider and
the elevation set out of the Manual Control panel into a panel of their own.

**Changed (`tools/dashboard/index.html`):**
- Deleted the `data-panel="safety"` panel. Removed with it: `#risk`,
  `updateRisk()`, the `.risk` / `.risk-card` / `.hint-list` CSS, and the
  matching compact / tiny rules.
- New `data-panel="operate"` panel, "Enable & Outputs / 啟用與輸出". It
  sits right below Match Setup and holds:
  - the Enable button, the key hint text, and the armed badge
    `#controlArmedBadge`;
  - the Falcon slider;
  - the elevation slider, number box and Set button.

  The panel uses `data-fixed-density` so it never shrinks to the small-text
  layout. Element ids are unchanged, so no JS changed.
- The Manual Control panel keeps only the readout cards; its title is now
  "手動操控狀態".
- Panel order: vision → setup → operate → control → field / telemetry,
  with sbus / log on the side.

**Evidence:**
- The page and control tests pass, with new assertions:
  - Enable, the slider and Set are inside the operate panel, not the
    control panel;
  - the safety panel is gone.
- In a real browser: demo mode works with no console errors, and the panel
  order and layout look right.

## 2026-10-17 — Claude Code: Xbox gamepad layout, match start (Y) + 3:00 countdown, keyboard trimmed

**User Request (Jeremy):**
- The gamepad uses the Xbox layout by default:

  | Control | Action |
  |---|---|
  | Left stick Y | forward / back |
  | Right stick X | turn |
  | D-pad | elevation / turret fine adjust |
  | X / A / B | full auto / semi auto / full manual |
  | RB | fire |
  | LB | turret home (AS5600, later) |
  | Y | start the auto modes and show a 3:00 countdown |
- Flow: connect → pick the path (and mode) → Enable, **and the robot does
  not move (sticks dead)** → press Y to run everything.
- The countdown starts on the first Y. At 0 it only shows 0:00 and does
  nothing else.
- Remove M and the arrows from the keyboard.

**Changed (`tools/dashboard/index.html` only; firmware, ground station and
Orin are unchanged):**
- `GAMEPAD_PROFILES.standard` (browser Gamepad API "standard" mapping):

  | Field | Value |
  |---|---|
  | forwardAxis | 1 |
  | turnAxis | 2 |
  | fireButton | 5 (RB) |
  | startButton | 3 (Y) |
  | homeButton | 4 (LB) |
  | modeButtons | {X(2) → 0, A(0) → 1, B(1) → 2} |
  | dpad | 12–15 |

  `other` (flight stick) stays axes 0/1 with fire on button 0, and has no
  start or mode buttons; the new Start button and the menu cover those.
- Every button acts once per press, through `gamepadButtonsPrev`. A press
  made while disabled is never replayed later.
- `pollGamepad()`:
  - Mode buttons work at any time through `selectMode()`. Before the start
    they only set the menu; during the match they send `MODE,n`.
  - Once enabled, Y → `startMatch()`.
  - LB only logs "AS5600 尚未安裝".
  - **Before `matchStarted` it returns without sending any motion.**
  - After the start: the stick (center first again) → T, the D-pad →
    `dpadAxes()` → ROT/ANG through the existing hold-to-move path. In the
    auto modes the firmware still turns each press into one fine-adjust
    step. RB → F.
- `startMatch()` (Y or the new "開始比賽 / Start（Y）" button):
  - once per Enable;
  - sends `MODE,n` (skipped for full manual), then `AUTO,n` (skipped for
    None);
  - `matchEndsAt = now + 3:00`.

  The menus are no longer applied at Enable, only at Y. Enable still
  resets the target display to the AI's pick.
- Before the start the Falcon slider, the elevation slider/box/Set and
  Start-while-pending are all disabled or ignored. The heartbeat only
  sends K.
- A disarm clears `matchStarted`, so a new Enable waits for Y again, but
  the running countdown keeps going. `renderMatchTimer()` runs every
  200 ms and on every UI update:
  - shows m:ss;
  - at 0 shows "0:00" in red ("時間到") and changes nothing else;
  - a new Y restarts it from 3:00.
- Keyboard: only 1–9 / 0, Enter and Space are left. M, the arrows, the
  keyup handler and the focusin hold-release handler are removed, because
  nothing on the keyboard is held any more.

**AS5600 turret home: design recorded, not implemented (Jeremy:
"先記著之後做"):**
- **Hardware:**
  - The AS5600 sits on the turret motor shaft, which drives the turret's
    big pulley through a belt at 180 : 36.5. One turret turn is about
    4.93 magnet turns.
  - The turret only travels about 180°, which is about 2.47 magnet turns,
    or about 56 counts per turret degree.
  - It will use I2C on D20 / D21. **The feed tower trigger (D21 today) has
    to move first** (Jeremy: "送球塔的腳位之後會改").
- **Multi-turn counting:**
  - `delta = raw − last`; if delta > 2048, subtract 4096; if delta <
    −2048, add 4096; `total += delta`.
  - The magnet must move less than 2048 counts between reads. At 2 rev/s
    that is about 80 counts per 10 ms, so it is safe.
- **Home (Jeremy):**
  - On the robot, pick a setpoint inside the first magnet turn (0–4095)
    as the origin, and count left/right from there.
  - At boot, if the turret is inside that first turn, a dedicated **turret
    PIDF (separate from the vision-follow PIDF)** brings it to the
    setpoint.
  - Otherwise, LB brings it back to the setpoint.
  - LB: one press drives to center and stops. In the auto modes it pauses
    vision-follow, then hands the turret back.
- **Caveat to raise with Jeremy when this is built:** the same raw count
  repeats about every 73° of turret travel (one magnet turn). At boot the
  Mega cannot tell which turn it is in, so it assumes the nearest one. The
  automatic home is only right if the turret is powered up within about
  ±36° of center (half a magnet turn). If it starts at, say, 90°, it
  locks onto the wrong turn and "home" lands about 73° off. Options when
  implementing:
  - mark the turret for power-up;
  - home against a mechanical end (the AS5600 count stops changing) once;
  - add a switch.
  - Also add software limits of about ±85°.

**Evidence:**
- `dashboard_control_test` now pins the Xbox map (axes, RB / LB / Y,
  X / A / B → modes, the D-pad) and `dpadAxes()`.
- `dashboard_page_test` was rewritten for the new flow and covers:
  - nothing moves before Y (stick, D-pad, RB, slider), only K;
  - digits still go out, and X / A / B only set the menu;
  - Y sends `MODE` / `AUTO` once, starts 3:00, then 2:00 and 0:01; a second
    Y does nothing;
  - disarm stops the match but not the clock; at 0 the robot stays
    enabled;
  - the ground station sends `MODE,1` / `AUTO,3` only at Y;
  - X / A / B and the menu during the match;
  - LB does nothing except the log line;
  - right stick X turn, left stick Y forward, the other axes are ignored;
  - RB fires and Y doesn't;
  - unplugging zeroes T / ROT;
  - the flight-stick profile;
  - the keyboard M / arrows are gone.
- In a real browser the layout is right and the console has no errors.
- Not run on hardware.

## 2026-10-17 — Claude Code: aim frames on the wireless uplink (distance / elevation pulse over the radio)

**User Request:** the Dashboard shows the camera distance and the elevation
pulse only on the Mega's USB, because they live in the TEL line. Jeremy:
"好，現在做" — make them show through the ground station too.

**Why this design:**
- The uplink stays on the CRSF **Battery** frame, chosen earlier because a
  custom frame type may not survive ELRS.
- The robot bridge forwards any `ST,<flags>,<seq>,<code>` line within its
  field ranges (flags ≤ 255, seq ≤ 0xFFFFFF, code ≤ 65535), so the aim
  data rides in those three fields and **the bridge needs no change**. The
  rebuilt image sha is still 99b017a4….
- One line per 100 ms keeps it safe with the bridge's "latest line every
  100 ms" forwarding.

**Changed:**
- `RcControlMap.{h,c}`: aim frame codec.
  - code: bit 15 = aim frame, bits 0–14 = distance in cm (0 = none).
  - seq: bits 0–11 elevation pulse µs, 12–15 vision id sent to the Orin,
    16–19 the frame's vision id, bit 20 valid.
  - flags: aim offset px as int8.
  - New functions `rcmap_aim_to_status()`, `rcmap_aim_from_status()`,
    `rcmap_status_is_aim()`.
  - A status code is at most 0x3FFF (12-bit event plus mode + 1 in bits
    12–15), so the two kinds can never be confused. The test checks every
    event × mode.
- New `src/IO/AimUplink.{h,cpp}` builds the frame from:
  - `Vision::isValid` / `getDistance` (mm → cm; 0 when invalid);
  - `getSelectedTarget`, `getTargetId`;
  - `Shooter::getAnglePulseUs`;
  - `ManualControl::getAimOffsetPx`.
- `ManualControl::sendStatusIfDue()` alternates the 100 ms slot: status,
  aim, status, aim, starting with status. **Status and aim each now go out
  every 200 ms**, well inside the ground's 1 s staleness window. Events
  therefore reach the ground up to about 100 ms later.
- Ground `RobotStatusTracker`:
  - An aim frame is stored apart and never emits `$EVENT` or touches the
    status / flags.
  - `tick()` emits `$AIM,<fresh>,<distanceCm>,<elevationPulseUs>,
    <selectedId>,<targetId>,<valid>,<aimOffsetPx>` after `$WIRELESS`.
  - `fresh` is 0, with zeros, after 1 s without an aim frame.
- Dashboard: `handleAimStatus()` fills the camera-distance card, the
  elevation pulse readout and the target-hole line from `$AIM`.
  `updateDistanceReadout()` is now shared with the TEL path. The radio
  carries no elevation offset, so the target line omits it there.
- `tools/stm32_elrs_robot_bridge`: test-only changes.
  - The host test now proves the bridge parses an aim `ST` line, and that
    it decodes back.
  - `build.py` links `RcControlMap.c` into that host test.
  - The firmware image is unchanged.

**Compatibility:** an old ground with new Mega firmware would read aim frames
as status, with garbage flags. **Flash the ground and the Mega together**;
they are already being flashed together on 2026-10-17. An old Mega with the
new ground just shows `$AIM,0,...`.

**Evidence:**
- All 8 Mega envs build (RAM 2594 B). The ground pio build passes. The
  bridge `build.py` passes its host test and the sha is unchanged.
- `rc_control_map_test`:
  - aim round trip through the Battery frame;
  - clamps;
  - no status code ever looks like an aim frame.
- `manual_control_test` new case `testAimUplink`:
  - status-first alternation, 5 / 5 per second;
  - content with and without a target;
  - status frames unaffected.

  `testStatusUplink` waits updated from 120 to 220 ms.
- Ground `testAimFrames`:
  - no `$EVENT`, flags untouched, the `$AIM` line, the next event still
    reported;
  - stale, and never heard.
- Dashboard page test: `$AIM` seen / not seen / AI pick / stale /
  malformed.
- All other host and page tests pass.
- **Not run on hardware.** The Battery-frame uplink itself is still
  unproven over the air until the 2026-10-17 wireless test.

## 2026-10-17 — Claude Code: wireless uplink works on hardware; slower-telemetry tuning

**Hardware result (Jeremy, first wireless bench of the day):**
- The Battery-frame status uplink **works over the air**, for the first
  time. The Dashboard showed "disarmed / none · 489 ms" with RF LQ 96–99%
  up and down.
- The readout kept flipping to "no robot status", because status frames
  arrived up to about 1 s apart. ELRS sends telemetry only in sparse slots
  (default ratio about 1:64), keeps only the latest Battery frame, and
  status and aim frames now share those slots.
- RSSI showed "-232 dBm": a display bug.

**Changed (Jeremy chose the code-side fix first; ELRS Telem Ratio left at
its default):**
- Ground `RobotStatusTracker::STALE_MS` goes from 1 s to **3 s** for both
  `$WIRELESS` and `$AIM`. This is display only; the robot still disarms
  itself on RC loss.
- Mega `sendStatusIfDue()`: slots are now **status, status, aim**
  (`statusSlot` 0 / 1 / 2) instead of 1:1, so status gets two thirds of the
  sparse telemetry slots.
- Dashboard:
  - `ENABLE_PENDING_TIMEOUT_MS` goes from 3 s to **5 s**.
  - New `rssiDbm()`: above 127 the byte is a signed value (232 means
    −24 dBm); otherwise it follows the spec ("dBm × −1", 45 means −45 dBm).

**Evidence:**
- Ground host test updated (still fresh at 1 s, stale at 3.05 s) and
  passes.
- `manual_control_test` passes, with the slot pattern now S S A (6 status
  and 3 aim frames in 900 ms).
- Dashboard tests pass (5 s pending, RSSI −45 / −24).
- The four ManualControl envs and the ground pio build pass. The bridge is
  untouched.

**Next:**
- Reflash the Mega and the ground station, then watch the age number in
  "wireless link".
- If it still flips, raise the ELRS Telem Ratio to 1:8 or 1:4 (ELRS Lua on
  an EdgeTX radio; it is stored on the TX).

## 2026-10-17 — Claude Code: wireless bench results and link robustness (robot bridge reflashed, 11c1 pad, PC timeout, buffers)

**Hardware results (2026-10-17 morning, Jeremy):**
- **Root cause of "Enable always fails"**: the robot's ELRS bridge STM32
  still ran the 2026-10-11 `16f26be` image. It sends **9** RC channels;
  `RCMAP_CHANNEL_COUNT` became 10 in `d7b6cbe` (10-12), so `MechLink`
  rejected every line.
  - TEL slots 7–11 were stuck at the `setSafe` values
    1500,1500,2000,1000,1000.
  - Meanwhile the bridge LED stayed solid and the Battery-frame uplink
    worked.
  - Claude had wrongly said the bridge needed no reflash, because the repo
    image sha was unchanged.
  - After reflashing `dist/` (99b017a4, 10 channels), **wireless Enable
    works end to end** (OUTPUTS_ENABLED and match start over the radio).
- The team's pad is Vendor 11c1 in D-input mode with no "standard" mapping.
  A profile was added in `0384adc`: hat D-pad on axis 9, RB fire, Y start.
- Two remaining issues: repeated `ground: PC_TIMEOUT` (the ground station
  heard no Dashboard line for 250 ms) and occasional
  `UNKNOWN_EVENT_CODE`.

**Changed:**
- Ground `PC_TIMEOUT_MS` 250 → **500 ms**. The Dashboard sends at least
  every 75 ms.
- Ground `RobotStatusTracker::onStatus`: a status whose event code is ≥
  `RCMAP_EVENT_COUNT` is corrupt and is dropped whole: no `$EVENT`, flags
  untouched, freshness not renewed.
- Mega production envs: `SERIAL_RX_BUFFER_SIZE` / `SERIAL_TX_BUFFER_SIZE`
  64 → 256, the same as the ELRS bench that passed on 10-11. This is so the
  TEL print can't block the loop long enough to overflow Serial2 RX. RAM
  goes from 2594 to 3746 B (45.7%).

**Evidence:**
- Ground host test passes: still armed at 251 ms, released at 501 ms;
  corrupt status dropped.
- Ground pio and the four production-family Mega envs build.
- Hardware retest pending (reflash ground + Mega).

**Open:**
- Where corrupt status frames come from is not yet known. Candidates: the
  Mega → bridge line, or the bridge's parse.
- If `PC_TIMEOUT` persists, check whether the Dashboard window was occluded
  or in the background (Chrome timer throttling).

## 2026-10-17 — Claude Code: direction fixes from the robot test (turning, turret jog)

**Robot result (Jeremy):** stick left turned the robot right and stick right
turned it left. The horizontal fine-adjust (turret jog) was also reversed.

**Changed:**
- Dashboard `gamepadTank()`: stick right is now `T,+turn,−turn`, left
  wheel forward, where it used to be `T,−1,1`. Fixed in the page so both
  the wired and the wireless paths follow, since the ground station turns
  T into forward/turn and the Mega mixes it back. No reflash.
  Forward/back unchanged.
- New `TuningConstants::Rotate::MANUAL_JOG_INVERTED = true`.
  `ManualControl::applyOutputs()` flips only the operator's full-manual
  turret jog (D-pad / wireless nudge). `Rotate::INVERTED` stays false,
  because the vision-follow direction was validated on the bench. The
  auto-mode aim offset sign (+ = right) is unaffected.
- Tests: page test turn expectations; `manual_control_test` uses a `JOG`
  sign plus a `static_assert` pinning the robot-verified value.

**Evidence:** page, control and `manual_control_test` (3 variants) pass, and
the 4 Mega envs build. **Next:** reflash the Mega, check that stick
left/right and D-pad left/right now match, and check that forward/back
is still right.

## 2026-10-17 — Claude Code: feed tower trigger moved D21 → D22

**User Request:** Jeremy asked whether the trigger idles LOW, worried that
the tower "fires by itself now and then". Decision: move the trigger to
**D22**.

**Why:**
- In production the trigger idles LOW (driven) and pulses HIGH for 100 ms
  per shot.
- But D21 is the Mega 2560's I2C SCL, which has an on-board pull-up.
  Whenever the pin is not driven, it sits **HIGH**:
  - during reset, boot and upload;
  - in every bench build that doesn't drive the tower
    (`mega_elrs_link_test`, `mega_serial1_test`, the wired / drive / axes
    benches, and so on).
- D22 has no pull-up, and D20/D21 are left free for the turret AS5600
  (I2C).

**Changed:**
- `DribblerConstants::Tower::TRIGGER_PIN` 21 → 22.
- Comments updated in `DribblerConstants.h` and the `Pins.h` pin list.
- The pincheck passes; the outputs are D22–D25, D45 and D46.

**Remaining:**
- D22 still floats until `Dribbler::init()`, and in benches that don't
  drive it. A pull-down (about 1–10 kΩ) on the tower's input, or
  rising-edge triggering on the tower side, keeps it safe.
- **Rewire the trigger to D22 before flashing.**

**Evidence:** `manual_control_test` passes in 3 variants and the 4 Mega
production-family envs build. Not run on hardware.

## 2026-10-17 — Claude Code: Falcon slider over the wireless link (CH4)

**User Request:** Jeremy asked for the Dashboard's Falcon slider to work
over the wireless link too. Until now the radio had no flywheel channel
(table v2), so the slider did nothing while wireless. He chose option B:
the flywheel shares CH4 with the mode, at 1% steps. Option A was 6 levels
on CH9.

**Discussion Result / Why:**
- Only CH1–4 are full resolution in ELRS Hybrid; CH6 and up have 6
  positions.
- The flywheel only matters in full manual, because in the autos the
  shot table drives it. So it can share CH4 with the mode.

**Changed:** new CH4 layout (`RcControlMap.h`):

| CH4 value | Meaning |
|---|---|
| 2000 µs | full manual, flywheel 0 (unchanged) |
| 1750 µs | semi auto (unchanged) |
| 1500 µs | full auto (unchanged) |
| 1000 + 300·f µs, 0 < f ≤ 1 | full manual with the flywheel at f (3 µs = 1%) |
| 1300–1399 µs | gap; `rcmap_decode` rejects the frame |

- `RCMAP_MODE_FULL_AUTO_MIN_US` moved 1250 → 1400.
- `rcmap_command.flywheel` added.
  - Encode: sent only in full manual, rounded to whole percent; NaN or
    ≤ 0 sends 2000.
  - Decode: rounds to the nearest 1%, 0 outside full manual, and returns
    the exact mode level in `mode_us`.
- Ground STM32:
  - `FLY,<0..1>` is now parsed. It is BAD_COMMAND outside 0..1 and is
    kept only in full manual.
  - A mode change (`MODE`/`M`), `E`, `X` and a PC timeout set it to 0.
  - `ANGT` stays no-channel.
- Mega `processWirelessFrame`:
  - `flywheelCommand = c.flywheel`. `applyOutputs` already uses it only
    in full manual.
  - A wireless Enable whose CH4 carries a flywheel speed is refused with
    `ENABLE_REJECTED_NOT_MANUAL`.

**Safety and compatibility:**
- The three old levels are unchanged. A ground STM32 still on the old
  image therefore can never make a new Mega spin the flywheel.
- A new ground with an old Mega:
  - Values ≥ 1250 µs read as full auto. That is a flywheel above about
    83%, so the old Mega would switch modes.
  - Lower values are ignored, since the old Mega forces the flywheel to 0.
  - Reflash both together.
- The Dashboard was not changed: it already gates the slider on armed +
  match started + full manual and resends FLY with the heartbeat.

**Impact:** reflash the Mega and the ground STM32. Robot bridge: no; a
rebuild gives the same `.bin`, sha256 99b017a463ba3163…, because it links
only the status helpers. Orin: no.

**Evidence:**
- `rc_control_map_test`: every 1% round-trips, also with ±1 µs jitter;
  the gap is rejected; old levels decode with the flywheel at 0.
- `manual_control_test` (3 variants) adds `testWirelessFlywheel`: speed
  in manual, 0 in semi auto, 0 on link loss, Enable refused when the
  speed isn't 0.
- Ground host test adds `testFlywheel`.
- Vision, operator-target, shot-table and the two Dashboard tests pass.
- Builds pass:
  - `megaatmega2560` (RAM 45.7%);
  - `mega_wired_manual_test`, `mega_drive_test`, `mega_manual_axes_test`
    and `mega_elrs_link_test`;
  - the ground STM32.
- Not run on hardware.

**Next Test:** after reflashing both boards: Enable over the radio, Y,
full manual, slider 10% → 30%. The Falcon should follow and the Mega TEL
mode channel should read about 1030 → 1090. Switching to semi auto must
drop it, as must releasing Enable.

**Separate open hardware item:** the Falcon turns slowly at the 1500 µs
neutral even though D25 and GND are confirmed. The code always outputs
1500 when stopped. Suspects: the Falcon's own neutral/deadband
configuration (it is borrowed from another team) or its PWM calibration.
Check the LED colour at neutral and unplug the signal wire to confirm.

## 2026-10-17 — Claude Code: two-stage PC watchdog on the ground STM32 + stall logging

**User Request:** Jeremy reported "still drops now and then" after
flashing 90092ec. The log showed `ground: PC_TIMEOUT`: the ground STM32 had
heard nothing from the Dashboard for more than 500 ms, and released the
enable. That meant a new Enable and a new Y mid-match. He approved:
diagnostics plus a two-stage timeout.

**Why:**
- The ground's loop reads USB before it checks the timeout, with the same
  `now`, so its own stalls can't fake a timeout.
- That leaves a page stall or a USB stall. The log could not tell which.
- The old one-stage design made every hiccup cost the whole match state.

**Changed:**
- `GroundCommandState`:
  - `PC_HOLD_MS` = 300. Above it, `holdMotion()` sets forward, turn,
    nudge and flywheel to neutral, keeps arm/mode/target, and prints
    `$GROUND_EVENT,PC_HOLD` once. The next accepted line prints
    `$GROUND_EVENT,PC_RESUMED,<silent ms>` and clears the hold.
  - `PC_TIMEOUT_MS` 500 → 1500. Above it the enable is released and
    `$GROUND_EVENT,PC_TIMEOUT` is printed, unchanged. Old Dashboards
    still recognise it.
  - Every accepted line goes through `notePcLine()`. Rejected or unknown
    lines still don't count as the PC being alive.
- Dashboard:
  - `PC_GAP <ms> (page busy)` is logged when a heartbeat tick runs more
    than 250 ms late. This is measured only while enabled or enabling, and
    resets on disarm.
  - `WRITE_SLOW <ms> (USB)` is logged when `writer.write` takes more
    than 250 ms.
  - The hold and resume events are log-only and do not disarm.

**Impact:**
- A frozen PC now stops motion in 300 ms (was 500) and disarms at 1.5 s
  (was 0.5 s).
- Reflash only the ground STM32. The Dashboard updates through Pages on
  push. The Mega, bridge and Orin are unchanged.

**Evidence:**
- Ground host test `testPcWatchdog` rewritten to cover: hold at 301 ms
  with arm/mode/target kept, printed once; resume with the gap (640);
  outputs return on the resends; a rejected line is not alive; timeout at
  1501 ms; no stale resume after a new E; quiet while disarmed.
- Page test covers: hold/resume don't disarm; PC_GAP is logged at
  625 ms and not at 75 ms or while disarmed; no false gap on the first
  tick after Enable; WRITE_SLOW.
- Control test passes. The ground firmware builds. Not run on hardware.

**Next Test:** use the robot normally. When it hiccups, the log
should show `ground: PC_HOLD` and then `PC_RESUMED,<ms>`, and the robot
should keep its enable. A `PC_GAP` next to it means the page was busy.
`WRITE_SLOW`, or a large `PC_RESUMED` with no `PC_GAP`, means USB or the
hub.

## 2026-10-17 — Claude Code: Falcon stop trim + Flywheel INVERTED committed

**User Request:** The flywheel creeps backwards when it should be still,
even though D25 and GND are confirmed. Jeremy asked to send a small
forward value at rest instead. He also asked to commit his local
`Flywheel::INVERTED = true`, which he had already flashed after the robot
test.

**Changed:**
- `TuningConstants::Flywheel::INVERTED = true`. This is Jeremy's value,
  now committed.
- New `TuningConstants::Flywheel::STOP_TRIM_US = 10`, with a
  `static_assert` in ShooterConstants.h limiting it to 0..30 µs.
- `Shooter::writeFlywheelCommand()`: a command ≤ 0 becomes
  `STOP_TRIM_US / 500` and goes through the same `commandToPulseUs()`
  INVERTED handling.
  - "Stopped" covers disabled (`stopAll`), before Y, and the slider at 0.
  - Commands above 0 are unchanged.
  - `init()` still sends a plain 1500 for the very first pulse;
    `stopAll()` right after it applies the trim.

**Calculation:** 10 µs → command 0.02. INVERTED gives −0.02, so the pulse
is 1500 + 0.02·(1000 − 1500) = **1490 µs**. Not inverted it would be
1510 µs. Checked values:

| Trim (µs) | Pulse, INVERTED | Pulse, not inverted |
|---|---|---|
| 5 | 1495 | 1505 |
| 15 | 1485 | 1515 |
| 20 | 1480 | 1520 |
| 30 | 1470 | 1530 |

The Falcon's neutral deadband is about ±4%, roughly ±20 µs.

**Why:**
- This borrowed Falcon reads 1500 as slightly reverse, probably because
  of its own neutral or deadband configuration.
- The trim is a robot-specific workaround. Set it back to 0 for a Falcon
  that is configured normally.
- The 30 µs cap stops a typo from spinning a disabled flywheel.

**Impact:** reflash the Mega only. The Dashboard's Falcon PWM readout will
show 1490 at rest.

**Evidence:**
- Mega envs build: `megaatmega2560`, manual axes, wired manual, drive,
  gamepad motor and vision rotate.
- `manual_control_test` passes.
- The pulse arithmetic was checked on the host (table above).
- Not run on hardware.

**Next Test:**
1. Flash, then leave the robot disabled and watch the flywheel.
2. Still creeping backwards: raise the trim to 15, then 20.
3. Creeping forwards: lower it to 5.
4. Then Enable, Y, and check that the slider at 0 is also still.

## 2026-10-17 — Claude Code: wireless enable/disconnect fixes (bridge, ground, Dashboard)

**User Request:** after 34fd71f the link kept dropping:
`PC_HOLD` → `PC_TIMEOUT`, "no confirmation" on the first Enable, and
`OUTPUTS_ENABLED` arriving 6 s after E. Jeremy approved fixing all of it.
The ELRS TX OLED shows 150Hz 25mW 1:32 TLM (not 1:64 as assumed before).
Jeremy is changing it to 1:4 on the module.

**Causes found:**
1. **Robot bridge.**
   - Any line from the Mega not starting with `ST,` was stored and
     re-sent as a `TEL_CHUNK` (0x7F) frame every 20 ms, forever.
   - The production Mega only sends `ST,`. But one garbled line (a Mega
     reset or upload mid-line) is enough to start about 40 custom frames
     a second, which compete with the Battery status frames for ELRS's
     telemetry slots.
   - This is a likely cause of multi-second status lag. It is unproven:
     the ground doesn't count frame types.
2. **Dashboard.** A *fresh* `$WIRELESS` (received within 3 s, but its
   content possibly seconds old) saying "not armed by wireless" disarmed
   the page silently. The heartbeat stopped, and the ground then fired
   `PC_HOLD` and `PC_TIMEOUT`. This matches 6:57:47 and 7:02:06.
3. **Ground.** E while CH6 was already high did `release()` + arm in the
   same instant, so CH6 never went low on air. The robot (which needs a
   low→high edge, and after an RC outage first needs a low) never
   confirmed: "no confirmation", and only a second press worked.

**Changed:**
- Bridge `main.c`: non-`ST,` lines are dropped, and the TEL_CHUNK sender
  and `TEL_CHUNK_SEND_PERIOD_MS` are removed. Only status goes up.
  - New image: 2584 bytes, sha256 `ee0344818ee2b1c2…`.
  - **Reflash the robot bridge.**
- Ground `GroundCommandState`:
  - `enabled` (E accepted) is separate from CH6 (`cmd.armed`).
  - `ARM_LOW_MS` = 200: CH6 rises only after it has been low for 200 ms,
    at once if it already was.
  - `enableEpoch()` is bumped on E and on a real release; `main.cpp`
    then calls `RobotStatusTracker::invalidate()`, so `fresh` = 0 until
    the next status. Events still come through.
  - LED follows `isEnabled()`.
- Dashboard:
  - `$WIRELESS` no longer disarms the page. It can still confirm a
    pending Enable. A mismatch is logged once per enable.
  - Disarm comes from robot events (`DISARM_EVENTS`) and ground
    releases.
  - Safe either way: the robot needs a new CH6 edge, and the ground
    releases 1.5 s after the page goes quiet.
  - `$RFLINK` changes are logged: lost / back, and LQ below 50% (once
    until it recovers above 70%).

**Evidence:**
- Ground host tests add `testArmEdge` (immediate raise; 200 ms low
  when re-enabling from high; X then quick E; release during the wait;
  epoch) and `testStatusInvalidate`. `testInputHardening` was adjusted.
- Page test: a status mismatch is logged once and does not disarm; the
  robot event does; RF logging.
- Control, rc map and manual_control tests pass. The bridge build.py
  runs its host test, then the cross-build and checks. The ground
  firmware builds.
- Not run on hardware.

**Next Test:**
1. Reflash the bridge and the ground.
2. Set TLM 1:4.
3. Enable once: it should confirm in about 1 s.
4. Drive for several minutes; there should be no `PC_TIMEOUT`.
5. If it still drops, the log should now say why: `robot status says…`,
   `RF link lost`, `RF LQ low`, `PC_GAP` or `WRITE_SLOW`.

## 2026-10-17 — Claude Code: tx / ty over the wireless link; vision panel trimmed

**User Request:** With the camera connected, the Dashboard showed the
distance over the wireless link but no tx/ty. Jeremy wanted tx/ty shown,
and the vision panel cut down to distance, tx and ty.

**Why:** The aim frame was full (46 bits), so tx/ty needed a frame of
their own. With TLM now at 1:4 the uplink has room for it.

**Changed:**
- `RcControlMap`: new vision frame.

  | Field | Contents |
  |---|---|
  | code | bits 15-14 = `01`; bits 0-13 = distance in cm, max 16383 |
  | seq | tx·2 and ty·2, each 12-bit two's complement (0.5 px steps, ±1023.5 px) |
  | flags | bit 0 valid; bits 1-4 = vision id |

  - Status codes are at most 0x3FFF and aim frames have bit 15 set, so
    the three never collide. The bridge's ST line field ranges still
    hold, so the bridge is unchanged (same .bin, ee034481).
- Mega:
  - `AimUplink::sendVision()`. tx/ty/distance are 0 when not valid.
  - Status slots are now status, aim, status, vision (were status,
    status, aim). Status goes out every 200 ms; aim and vision every
    400 ms each.
- Ground: `RobotStatusTracker` keeps vision frames apart and emits
  `$VISION,<fresh>,<cm>,<tx*2>,<ty*2>,<valid>,<id>` every 250 ms, in
  half pixels because there is no float printf.
- Dashboard:
  - `handleVisionStatus` shows tx/ty, or `--` when the data is stale or
    not valid.
  - The vision panel keeps only tx, ty and distance. The targetId,
    valid, shooterReady, operatorMode and warnBits cards and their code
    were removed. The mode stays visible in the enable badge and the
    mode menu.

**Impact:**
- Flash the Mega and the ground STM32 **together**. An older ground would
  read a vision frame as a status with a bogus event.
- The bridge and the Orin are unchanged.

**Evidence:**
- rc map `test_vision_frames`: round trip, clamps, no overlap with
  status/aim.
- `manual_control_test` (3 variants): slot order, and the new
  `testVisionUplink`.
- Ground `testVisionFrames`.
- Page test covers `$VISION` and the removed card ids.
- Vision link, operator target, shot table and control tests pass.
- Builds pass: `megaatmega2560` (RAM 45.9%), axes, wired, drive,
  elrs-link and the ground. Bridge sha unchanged.
- Not run on hardware.

**Next Test:**
1. Flash the Mega and the ground STM32.
2. Point the camera at a hole while on the wireless link.
3. tx/ty should update about every 0.4 s and match the Orin window's
   overlay.
4. Moving the target out of view should show `--`.

## 2026-10-17 — Claude Code: flywheel at 0 in the autos for turret-follow testing

**User Request:** Jeremy wants to test the turret vision-follow in semi
auto without the flywheel running. He asked for the shot-table flywheel
values to be 0.0, not a switch, so that the flywheel behaves as it does
when manually disabled.

**Changed:**
- Every flywheel entry in all four `ShotTable` rows is 0.00. They were
  0.72 at 2.5 m, rising 0.03 per 0.5 m to 0.98 at 8.5 m; the old values
  are noted in a comment.
- `Flywheel::SHOOT_COMMAND` 0.8 → 0.0. This is the no-target command in
  the autos, so the flywheel stays stopped there too. With command 0,
  `writeFlywheelCommand` sends the same stop-trim pulse as when disabled:
  1490 µs (`STOP_TRIM_US` 10 with INVERTED).
- `Shooter::updateFlywheelReady()` is now ready only if the command is
  > 0 *and* ≥ SHOOT_COMMAND. Otherwise SHOOT_COMMAND 0 would have made a
  stopped flywheel "ready" after 1.5 s, and the autos could have fed a
  ball into it.
- `shot_table_lookup_test` asserts the interpolation as a midpoint within
  range rather than strictly increasing. A test comment was updated.

**Impact:**
- Semi and full auto: the turret follows; the flywheel stays stopped;
  the autos never fire.
- Full manual: unchanged; the Dashboard slider still works.
- Reflash the Mega only.
- **Restore the table and SHOOT_COMMAND before any shooting in the
  autos.**

**Evidence:**
- `manual_control_test` (3 variants), `shot_table_lookup_test`.
- Builds pass: `megaatmega2560`, axes and wired.
- Not run on hardware.

**Note:** Jeremy's working tree had uncommitted changes that are not
Claude's: a chassis command filter (`COMMAND_FILTER_TIME_CONSTANT_MS`),
plus edits to MechLink, Telemetry and the Dashboard. They were left
untouched. Flashing from that folder includes them.

## 2026-10-18 — Claude Code: committed Jeremy's working-tree drive changes (597c614)

**User Request:** "push everything on dev". Uncommitted edits in Jeremy's
tree (author not Claude) were reviewed, tested and committed.

**What they do:**
- **Chassis output filter.** An exponential filter on the left/right
  command, tau `COMMAND_FILTER_TIME_CONSTANT_MS` = 80 ms, with
  α = dt/(dt+τ). It reaches 63% of a step in 80 ms and 95% in 240 ms.
  `stop()` and disable snap straight to neutral (safety stops are never
  eased). The first tick after an enable snaps to the target.
- **Rescaled deadband.** `applyDeadband` now returns
  (|c| − 0.04)/(1 − 0.04) with the sign kept, instead of a step at 0.04.
- **TEL slot 38: `MechLink::lastLineAgeMs()`.** The time since the last
  valid RC line. The wired Dashboard shows it in the RF link cell, with
  a ⚠ at ≥ 200 ms (LINE_TIMEOUT_MS).
- **Dashboard gamepad mix.**
  - `TURN_INNER_RATIO` = −0.5 and `DRIVE_TURN_GAIN` = (1−r)/(1+r) = 3.
    The turn gain blends from 1 at no forward to 3 at full forward.
  - Both wheels are scaled by the larger one rather than clipped.
  - Examples: full forward + full turn gives (1, −0.5), where it used to
    clip to (1, 0). Turn alone still gives (1, −1).
  - The wireless path inverts exactly (ground: forward = (l+r)/2,
    turn = (r−l)/2; Mega mixes back), so the ratio survives.
  - Note: at full forward a small turn is 3× stronger than before.

**Evidence:**
- Page, control, manual_control (2 variants) and gamepad_motor tests
  pass.
- Builds pass: `megaatmega2560`, axes, wired and drive.
- No dedicated host test for the Chassis filter (Chassis is stubbed in
  the host tests).
- Not run on hardware by Claude.

## 2026-10-18 — Claude Code: first field shot data in the tables; flywheel "ready" time-based

**User Request:** Jeremy shot the center column (Dashboard keys 4/5/6) at
2.5 m and 5 m, and asked to load those values to check semi-auto lock and
fire at a standstill.

**Changed:**
- `ShotTable`: one table per row, two entries each:

  | Row | 2.5 m | 5 m |
  |---|---|---|
  | bottom | 0.42 / 2407 µs | 0.43 / 2183 µs |
  | middle | 1.00 / 2500 µs | 0.46 / 2063 µs |
  | top | 1.00 / 1010 µs | 0.80 / 2442 µs |

  - The top row's two points are different shots (a flat shot at 2.5 m,
    a lob at 5 m), so values interpolated between them are suspect.
  - The bonus holes have **no table**, so the autos never fire at them.
  - `ELEVATION_MEASURED` = true: the autos now set the elevation pulse
    from the table plus the up/down offset.
  - New `EXTRAPOLATE_PAST_LAST_ENTRY` = false. Past the last entry the
    values are held (Jeremy's choice while the tables stop at 5 m).
    `ShotTableLookup::lookup()` takes an `extrapolate` argument that
    defaults to it.
- `Flywheel::SHOOT_COMMAND` 0.0 → **0.5**: the pre-spin with no target
  (Jeremy's choice). It is no longer the "ready" threshold.
- `Shooter::updateFlywheelReady()`: ready means the command is > 0 and
  has stayed within ±0.02 of where it settled for `SPIN_UP_MS`
  (1500 ms). The old "≥ SHOOT_COMMAND" meant 42–46% table values were
  never ready.
- The existing tests (`shot_table_lookup_test`, `manual_control_test`)
  were adapted to measured elevation, bonus without a table, and the
  hold. No new test files.

**Evidence:**
- Table test passes; `manual_control_test` passes in all 3 variants;
  page test passes.
- Builds pass: `megaatmega2560`, axes and wired.
- Not run on hardware.

**Next Test:** at a standstill at 2.5 m and 5 m in semi auto, select
keys 4, 5 and 6 in turn and check:
1. the elevation moves to the table pulse;
2. the flywheel goes to the table %;
3. 1.5 s after the turret locks, RB fires.

## 2026-10-18 — Claude Code: pad remap (LB fire / RB home, LT/RT ±1%), auto flywheel offset, aim step 20 px, pre-spin 20%

**User Request:**
- Pre-spin with no target at 20%.
- Semi auto's left/right fine-adjust "does nothing".
- Swap RB and LB.
- LT and RT become buttons: −1% / +1% Falcon per press, in semi auto
  and full manual.
- The 11c1 pad reads LT = button 8 and RT = button 9; their analog axes
  3 and 4 are not used.

**Why:** the aim fine-adjust worked but was invisible. 5 px × kP 0.01 is
a 0.05 command, about 25 µs on the turret servo, which is inside its
deadband; only the slow integral moved it.

**Changed:**
- `Flywheel::SHOOT_COMMAND` 0.5 → 0.2.
- `Auto::AIM_OFFSET_STEP_PX` 5 → 20 px (max stays 100).
- **Auto flywheel offset.** In semi and full auto, LT/RT adjust an
  offset of −20..+20% that is added to the shot table's value.
  - Dashboard sends `FLYO,<n>` (absolute). The robot clamps
    table + offset to 0..1. It is not added to the no-target pre-spin.
  - Mega: `ManualControl::flywheelOffsetPct`, set by wired `FLYO` or by
    CH4 (wireless).
  - It is cleared on a mode change, Enable and disarm. The Dashboard
    also sends `FLYO,0` after every `TGT` (new hole).
  - `Auto` adds it to `shot.flywheelCommand`.
- **CH4 encoding (`RcControlMap`).** In the autos the level carries the
  offset: level + 4 µs per 1%. That gives semi 1670–1830 and full
  1420–1580, both inside their bands.
  - Exact 1750 / 1500 means offset 0, so an older ground STM32 stays
    compatible.
  - An older Mega reads the same mode and simply ignores the offset.
  - Decoding rounds to the nearest 1%, so ±1 µs link noise is absorbed.
- **Ground STM32.** Parses `FLYO,<-20..20>` (BAD_COMMAND outside that
  range or for a non-integer). It keeps the value only in the autos and
  resets it on a mode change, E, X and a PC timeout.
- **Dashboard.** Profiles: Xbox fire 4 (LB), home 5 (RB), LT 6, RT 7;
  11c1 fire 6, home 7, LT 8, RT 9.
  - `nudgeFlywheel()`: in full manual, the slider ±1% (`FLY`); in the
    autos, the offset (`FLYO`).
  - The heartbeat resends `FLYO` while it is non-zero.
  - The label shows the offset in the autos.

**Evidence:**
- The existing tests were extended: rc map (offset round trip with
  ±1 µs, clamp, manual ignored), ground (`FLYO`), `manual_control_test`
  (FLYO adds / refuses / clears; 3 variants), and the page and control
  tests (new map, LT/RT in manual and auto, reset on hole and mode).
- Builds pass: Mega (4 envs) and the ground. Bridge sha unchanged
  (ee034481).
- Not run on hardware.

**Impact:** flash the Mega and the ground STM32 together. The Dashboard
updates through Pages after a push. The bridge and the Orin are
unchanged.

## 2026-10-18 — Claude Code: Auto target hold 0.5 s; Falcon pulse on the wireless Dashboard

**User Request:**
- While locked in semi auto, the flywheel "jumps up and down".
- Jeremy wants to see the Falcon's ESC value while adjusting over the
  radio.
- Push when done.

**Why it jumped:** this was not the radio, because Auto runs on the Mega
from the Orin's UART frames (~11 fps).
- Any single frame that is invalid, has distance 0, or is for another
  hole made `hasTarget` false at once.
- The flywheel then dropped to the pre-spin (`SHOOT_COMMAND`, 20%)
  until the next good frame about 90 ms later.
- Each drop also restarted the 1.5 s "ready" wait (a change of more
  than 2%), so frequent drops could block firing altogether.

**Changed:**
- **Target hold.** New `TuningConstants::Auto::TARGET_HOLD_MS` = 500.
  - `Auto` keeps the last target's table flywheel and elevation pulse for
    that long after the target drops. The LT/RT offset and the elevation
    offset are still applied on top.
  - The hold does not cross a new hole selection
    (`Vision::getSelectedTarget`), and it is cleared in full manual and
    without vision.
  - Firing still needs a target in the current frame (`ready` uses
    `hasTarget`).
- **Vision frame field.** Code bits 0–13 are now the Falcon's actual
  pulse (`Shooter::getFlywheelPulseUs`) instead of the distance, which
  the aim frame already carries.
  - `rcmap_vision.flywheel_pulse_us`; ground `$VISION,<fresh>,<flywheelUs>,...`.
  - The Dashboard shows it in "Falcon PWM" over the radio as `1290 us
    (42.0%)`, where % = |pulse − 1500| / 5. At rest this reads 1490 us,
    the stop trim.
- Tests: the existing ones were adjusted (vision frame field; a hold
  check in the semi-auto test); the host stubs gained
  `getFlywheelPulseUs`. No new test files.

**Evidence:**
- All host tests pass.
- Builds pass: Mega (4 envs; RAM 40.2%, down from 45.9% because the
  shot tables are smaller) and the ground. Bridge sha unchanged.
- Not run on hardware.

**Impact:** flash the Mega and the ground STM32 together. This bundles
b1d6e1b and 562b817, which were not flashed yet.

## 2026-10-18 — Claude Code: flywheel ramp-up limit and Auto elevation slew (current limiting)

**User Request:**
- At 5 m in semi auto, keys 4 and 5 switch fine, but key 6 (top row)
  stops the flywheel, and only disable + Enable brings it back.
- Jeremy agrees it is likely a current surge, and asked for the
  elevation and flywheel to move slowly.

**Why:**
- Switching middle → top jumped the flywheel 46% → 80% and the elevation
  2063 → 2442 µs in the same instant.
- A Mega reset matches the symptom: it boots disarmed and needs a fresh
  CH6 low → high, hence disable + Enable.
- The same values had worked in full manual, where they were reached
  gradually with the slider and D-pad.

**Changed:**
- **Flywheel ramp.** New `Flywheel::RAMP_UP_PER_SECOND` = 0.5.
  - `Shooter::rampFlywheel()` moves the output toward the command at up
    to 50% per second when going up (0 → 80% takes 1.6 s). Going down,
    and `stopAll` / disable, take effect at once.
  - It applies in every mode, so the manual slider is ramped too.
  - "Ready" and `getFlywheelCommand()` now use the ramped output, so the
    1.5 s `SPIN_UP_MS` wait starts only once the ramp arrives.
- **Elevation slew.** New `Auto::ELEVATION_SLEW_US_PER_SECOND` = 400.
  - Auto moves the elevation toward table + offset at that rate instead
    of `setAnglePulseUs(target)` in one jump. For example,
    2063 → 2442 µs takes ~0.95 s.
  - Each step is limited to 100 ms worth after a stall. The full-manual
    D-pad jog is unchanged (1000 µs/s).
- **Tests.** The host stub's `setAnglePulseUs` now moves `anglePulseUs`,
  which is reset per test. The elevation and aim-uplink tests allow for
  the slew. No new test files.

**Evidence:**
- `manual_control_test` (3 variants), the table test and the page test
  pass.
- Builds pass: `megaatmega2560`, axes, wired, drive and gamepad motor.
- Not run on hardware.

**Next Test:**
1. Flash the Mega.
2. Switch 5 → 6 at 5 m: the elevation should glide, the flywheel should
   ramp, and there should be no reset.
3. If it still resets, the power wiring needs checking (the elevation
   servo should have its own supply).

## 2026-10-18 — Claude Code: flywheel ramps down too; spin-down after disable (option B)

**User Request:** ramp the flywheel down as well, and on disable let it
spin down rather than stop instantly. The hard stop is bad for the motor.

**Why:**
- At neutral the Falcon applies its own Neutral Mode. This borrowed unit
  is evidently set to Brake, and PWM can't change that.
- Claude recommended setting the Falcon to Coast in Phoenix Tuner (A).
  Jeremy chose B: software spin-down.

**Changed:**
- New `Flywheel::RAMP_DOWN_PER_SECOND` = 0.5.
- `Shooter::rampFlywheel()` now limits both directions.
- **Disabled.** `Shooter::update()` still stops the turret and elevation
  at once (`stopAll`), but then ramps the flywheel output down to 0 and
  writes it. The Falcon is **still driven** during this spin-down
  (≤ 2 s from 100%), then it gets the 1490 µs stop trim.
- `stopAll()` no longer zeroes the output or the ramp clock. It writes
  the current output (no jump) and zeroes the command.
- `init()` sets the output to 0.
- Re-enabling during the spin-down continues from the current output.

**Safety note:** a disable (including link loss and watchdogs) no longer
cuts the flywheel drive instantly. It ramps down over up to 2 s. Setting
the Falcon to Coast would allow returning to an instant neutral.

**Evidence:**
- Mega envs build: production, axes, wired, drive, gamepad motor and
  vision rotate.
- `manual_control_test` passes; Shooter is stubbed there, so the ramp is
  not host-tested.
- Not run on hardware.
