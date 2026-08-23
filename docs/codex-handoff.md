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
