#pragma once

#include <stdint.h>

// Centralizes the parameters that actually get changed at competition --
// motor direction, flywheel speed, vision-follow tuning -- in one place, organized by mechanism namespace
// (mirrors ShooterConstants.h's own Angle/Rotate/Flywheel grouping, plus
// Auto/ShotTable). ShooterConstants.h and AutoConstants.h (now retired
// into Auto below) pull their tunable values
// from here via `using namespace` so there is exactly one place to edit
// each one.
//
// What stays OUT of here, in each module's own Constants file: pin
// numbers, servo PWM endpoints, communication/protocol constants, timing
// -- these are wiring/hardware-spec facts, not "turn the knob" values, and mixing
// them in here would make them easier to change by accident.
//
// Other branches (e.g. Chassis) have their own TuningConstants.h following
// this same convention/namespace-per-mechanism pattern, scoped to whatever
// mechanisms actually exist on that branch. When branches merge, the two
// files combine by keeping both branches' namespace blocks side by side,
// the same way ShooterConstants.h/Pins.h conflicts were resolved earlier.
namespace TuningConstants {

namespace LeftDrive {
constexpr bool INVERTED = false;
}

namespace RightDrive {
constexpr bool INVERTED = true;
}

namespace Manual {
constexpr double COMMAND_DEADBAND = 0.04;
constexpr double MIN_COMMAND = -1.0;
constexpr double MAX_COMMAND = 1.0;
}

namespace Angle {

constexpr bool INVERTED = false;

}

namespace Rotate {

constexpr bool INVERTED = false;

// Vision-follow (RotateControlMode::VISION_FOLLOW), PIDF on image-space tx
// (src/Control/VisionFollowController.h). These are the gains Jeremy tuned
// and accepted on the real turret servo on the A5 bench (2026-10-13:
// "settles at dead center", full speed when far off), converted from the
// bench's microseconds to this axis's normalized command, where 1.0 = the
// servo's full 500 us offset (1000..2000 us):
//   bench kP 5 us/px          -> 5/500  = 0.01 per px
//   bench kI 5 us/(px*s)      -> 5/500  = 0.01 per (px*s)
//   bench I cap +-60 us       -> 60/500 = 0.12
//   bench output cap +-500 us -> 1.0 (full speed)
//   kIZone 20 px, kD 0 (unchanged units).
// Example: tx = 40 px -> -0.40 command -> 1300 us (tx > 0 drives below
// 1500 us: the direction validated on the bench with this same servo and
// mounting). test/vision_follow_controller_test.cpp pins these to the
// bench constants, so the two can't drift apart silently.
constexpr double VISION_kP = 0.01;
constexpr double VISION_kI = 0.01;
constexpr double VISION_kD = 0.0;
constexpr double VISION_kIZONE_PX = 20.0;
constexpr double VISION_kFF = 0.0;
constexpr double VISION_MAX_COMMAND = 1.0;
constexpr double VISION_MAX_INTEGRAL_COMMAND = 0.12;
// "Locked, OK to fire" for Auto: |tx| within this for READY_SETTLE_TIME_MS
// (Jeremy, 2026-10-13: 5 px).
constexpr double VISION_READY_TOLERANCE_PX = 5.0;

}

namespace Flywheel {

constexpr bool INVERTED = false;

// Auto-fire sequence (Auto::update()). No RPM/tachometer feedback exists for
// this Falcon 500 (PWM command path is one-way) -- SHOOT_COMMAND/SPIN_UP_MS
// are both provisional placeholders standing in for real speed feedback.
// Re-tune against real shots once hardware is available; do not trust these
// numbers unattended.
constexpr double SHOOT_COMMAND = 0.8;
constexpr uint32_t SPIN_UP_MS = 1500;

}

namespace Auto {

// Minimum time from one auto shot to the next (src/Auto/Auto.cpp). The
// feed tower's own lockout (DribblerConstants::Tower::COOLDOWN_MS, 1000 ms)
// also applies, so the real full-auto rate is whichever is longer.
constexpr uint32_t COOLDOWN_MS = 300;

}

namespace ShotTable {

// Distance (meters, measured from Vision) -> flywheel command and
// elevation servo pulse. Both are the actual actuator values to send:
//   flywheelCommand   open-loop Falcon command 0..1 (no speed feedback,
//                     so not an RPM)
//   elevationPulseUs  the elevation servo's target pulse, as shown on the
//                     Dashboard "elevation pulse" readout. The elevation is
//                     a positional servo, so a pulse is a position; storing
//                     the pulse (not an angle) needs no angle calibration.
// Fill it from test shots: at each distance, adjust until it scores and
// copy the Dashboard's pulse and the Falcon % here.
//
// PLACEHOLDERS: no real shots have been measured. The flywheel column is a
// guess; the elevation column is all neutral (1500 us) and is NOT applied
// until ELEVATION_MEASURED is set true -- placeholder pulses must never
// move the real elevation. Until then the operator keeps the elevation in
// every mode.
struct Entry {
    double distanceMeters;
    double flywheelCommand;
    double elevationPulseUs;
};
constexpr bool ELEVATION_MEASURED = false;
constexpr Entry TABLE[] = {
    {2.0, 0.70, 1500.0},
    {2.5, 0.72, 1500.0},
    {3.0, 0.75, 1500.0},
    {3.5, 0.78, 1500.0},
    {4.0, 0.80, 1500.0},
    {4.5, 0.82, 1500.0},
    {5.0, 0.84, 1500.0},
    {5.5, 0.86, 1500.0},
    {6.0, 0.88, 1500.0},
    {6.5, 0.90, 1500.0},
    {7.0, 0.92, 1500.0},
    {7.5, 0.94, 1500.0},
    {8.0, 0.96, 1500.0},
};
constexpr int TABLE_COUNT = sizeof(TABLE) / sizeof(TABLE[0]);

}

static_assert(Rotate::VISION_MAX_COMMAND > 0.0 && Rotate::VISION_MAX_COMMAND <= 1.0,
              "Vision-follow command limit out of range");
static_assert(Rotate::VISION_MAX_INTEGRAL_COMMAND >= 0.0 &&
              Rotate::VISION_MAX_INTEGRAL_COMMAND <= Rotate::VISION_MAX_COMMAND,
              "Vision-follow integral limit must fit inside the command limit");
static_assert(Rotate::VISION_kIZONE_PX >= 0.0 && Rotate::VISION_READY_TOLERANCE_PX > 0.0,
              "Vision-follow pixel windows must be non-negative");
static_assert(ShotTable::TABLE_COUNT >= 2, "Shot table needs at least two entries to interpolate");

}
