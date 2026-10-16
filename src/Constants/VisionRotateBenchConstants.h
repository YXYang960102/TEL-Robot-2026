#pragma once
#include <stdint.h>

// Standalone Orin-vision-follow bench for the A5 horizontal servo. Does not
// touch production Shooter/Vision. Self-contained (no longer shares
// RotateBenchConstants.h with the manual-jog bench, which was removed
// 2026-10-06 once this vision-follow path became the maintained one).
//
// Briefly moved to A4 then back to A5 (2026-10-09, same day): Jeremy
// rewired the servo signal to A4 on the bench, this constant was updated
// to match, then the wire was moved back to A5 and this was reverted to
// match again. SIGNAL_PIN and physical wiring must always match -- a
// mismatch produces no error, just a servo that looks unresponsive while
// the firmware computes correct, changing pulse values on a pin nothing is
// listening to. Re-check this comment against the actual wire before
// assuming either one is right.
namespace VisionRotateBenchConstants {
constexpr uint8_t SIGNAL_PIN = 59; // Mega A5 is digital pin 59.
// No limit switches (Jeremy, 2026-10-14: the shooter head has no sensors);
// the D43/D42 inputs and the E_NO_LIMITS mode were removed.
constexpr int NEUTRAL_US = 1500;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250; // USB (browser tab) watchdog
constexpr uint32_t TELEMETRY_MS = 100;
constexpr unsigned long BAUD = 115200; // USB Serial, for the browser page

// tx>0 (target right of center) was observed on real hardware 2026-09-25 to
// rotate the servo away from center instead of toward it.
constexpr bool INVERTED = true;

// Must match the Orin's protocol (YOLO_Detect_single vision_main.main()
// uses 2 since 2026-10-16). This bench sends no MEGA_TARGET, so the Orin
// keeps its own automatic target choice (selection 0).
constexpr int PROTOCOL_VERSION = 2;
constexpr unsigned long VISION_BAUD = 115200; // Serial1, must match Orin SERIAL_BAUD

// Deadband/slow-zone/creep/jog-speed bang-bang-with-ramp control retired
// 2026-10-09 in favor of pure PIDF (see below) -- Jeremy's call, direct
// tx error -> PidfController -> servo command instead of a hand-shaped
// speed profile.
//
// DEADBAND_PX itself was retired a few hours later the same day: measured
// oscillation during live tuning (+-30~40px swings) never got anywhere
// near the old 1px boundary, so the deadband wasn't the problem, and
// Jeremy's explicit call (overriding Codex's "don't loosen the deadband"
// handoff note -- see docs/codex-handoff.md) was to drop it entirely and
// let output=kP*error(+filtered D) settle to zero on its own, FRC-bench
// style. Whether that's actually silent at tx=0 depends on the servo's
// own minimum-movable-offset floor -- see kD_FILTER_TAU_S below and the
// neutral-point measurement procedure in docs/codex-handoff.md.

// Pure PIDF: setpoint=0, measurement=tx, straight into PidfController
// (src/Control/PidfController.h -- the same utility Shooter's Angle/Rotate/
// Flywheel loops use) for P(+I+FF, both 0 here). D is NOT the value
// PidfController computes internally -- VisionRotateBenchState always
// constructs it with kD=0 and instead applies this kD externally to a
// time-constant-filtered rate (see kD_FILTER_TAU_S), so PidfController's
// shared, unfiltered instantaneous-rate behavior stays untouched for
// Shooter's Angle/Rotate/Flywheel loops. All gains start at 0.0, same
// convention every other PIDF loop in this project uses -- tune up from
// zero on the bench, don't guess a starting point here.
constexpr double kP = 5.0;
// I added 2026-10-13 (Jeremy): only inside |tx| <= kIZone (7 px at first,
// widened to 20 px the same day at Jeremy's request), to push
// through the continuous servo's dead zone around 1500 us where small P
// outputs (kP*5px = 25 us) don't move it, leaving it a few px off center.
// Units: us per (px*s) -- at a steady 20 px error I grows 5*20 = 100 us/s.
// Outside the zone PidfController zeroes the integral (no stale windup
// carried through an overshoot); stop/target change/target lost reset it.
// The servo command is a SPEED, so this loop already integrates once
// through the mechanism -- extra I can make it hunt around center; start
// small. Limited separately by PIDF_INTEGRAL_LIMIT_US below.
constexpr double kI = 5.0;
constexpr double kD = 0.0; // applied externally to filtered rate, not via PidfController
constexpr double kIZone = 20.0; // px; integral only accumulates while |error|<=kIZone
                                // (and kIZone>0) -- see PidfController::calculate().
constexpr double kFF = 0.0;

// Low-pass time constant (seconds) for the D term's rate-of-change input.
// PidfController's own measurementRate is instantaneous/unfiltered -- at
// this bench's dt (~50-100ms, FPS-dependent) a single noisy or bursty
// sample could otherwise dominate D. alpha = dt/(tau+dt) keeps the
// filter's effective response time anchored to this constant regardless
// of frame-rate jitter. 0.1s is a starting point to tune like any other
// gain here, not a measured/validated value yet.
constexpr double kD_FILTER_TAU_S = 0.1;

// Servo authority ceiling, enforced via PidfController::setOutputLimits().
// Raised 300 -> 500 (2026-10-13, Jeremy's explicit call, overriding the
// earlier "keep 300" handoff note): Jeremy wants the hardware's top speed
// when the target is far off-center. 500 = the full 1000..2000 us range the
// servo is attached with, i.e. the servo's own maximum. kP is unchanged, so
// behavior inside |tx| < 300/kP = 60 px (where it already settles at center
// as Jeremy wants) is identical; only |tx| > 60 px gets faster, reaching
// full speed at |tx| >= 500/kP = 100 px. Higher arrival speed may overshoot
// center -- if so, add kD or step back toward 400, don't change kP first.
constexpr int PIDF_OUTPUT_LIMIT_US = 500;

static_assert(PIDF_OUTPUT_LIMIT_US > 0 && PIDF_OUTPUT_LIMIT_US <= 500,
              "Bench authority exceeded");

// Cap on the I term alone (us). PidfController's default integral limit is
// +-1.0 -- left at that, any kI here would contribute at most 1 us. 60 us is
// enough to get past a typical continuous-servo dead zone without letting I
// alone drive the turret fast.
constexpr int PIDF_INTEGRAL_LIMIT_US = 60;
static_assert(PIDF_INTEGRAL_LIMIT_US > 0 && PIDF_INTEGRAL_LIMIT_US <= PIDF_OUTPUT_LIMIT_US,
              "Integral limit must fit inside the output limit");

constexpr uint32_t VISION_TIMEOUT_MS = 500; // Serial1 (Orin) link watchdog,
                                             // independent of COMMAND_TIMEOUT_MS.
constexpr uint32_t MEGA_HEARTBEAT_INTERVAL_MS = 200; // well under Orin's 1.0s
                                                       // MEGA_HEARTBEAT_TIMEOUT_SECONDS
}
