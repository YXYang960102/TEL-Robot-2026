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
