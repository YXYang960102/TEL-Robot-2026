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
