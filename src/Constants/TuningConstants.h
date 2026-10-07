#pragma once

#include <stdint.h>

// Centralizes the parameters that actually get changed at competition --
// angle limits, motor direction, flywheel speed, each mechanism's PIDF,
// vision-follow tuning -- in one place, organized by mechanism namespace
// (mirrors ShooterConstants.h's own Angle/Rotate/Flywheel grouping, plus
// Auto/OperatorMode/ShotTable). ShooterConstants.h, AutoConstants.h (now
// retired into Auto below), and OperatorMode.h pull their tunable values
// from here via `using namespace` so there is exactly one place to edit
// each one.
//
// What stays OUT of here, in each module's own Constants file: pin
// numbers, servo PWM endpoints, communication/protocol constants, sensor
// sampling periods/timeouts/valid ranges, limit-switch debounce -- these
// are wiring/hardware-spec facts, not "turn the knob" values, and mixing
// them in here would make them easier to change by accident.
//
// Other branches (e.g. Chassis) have their own TuningConstants.h following
// this same convention/namespace-per-mechanism pattern, scoped to whatever
// mechanisms actually exist on that branch. When branches merge, the two
// files combine by keeping both branches' namespace blocks side by side,
// the same way ShooterConstants.h/Pins.h conflicts were resolved earlier.
namespace TuningConstants {

namespace Angle {

constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = true;

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;

// Keep position control disabled until both mechanical limits are measured.
constexpr bool POSITION_LIMITS_CALIBRATED = false;
constexpr long MIN_POSITION_COUNTS = 0;
constexpr long MAX_POSITION_COUNTS = 0;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = false;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = false;

}

namespace Rotate {

constexpr bool INVERTED = false;

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;

// Position control remains disabled until all three potentiometer points are measured.
constexpr bool POSITION_CALIBRATED = false;
constexpr int LEFT_POSITION_RAW = 0;
constexpr int CENTER_POSITION_RAW = 0;
constexpr int RIGHT_POSITION_RAW = 0;
constexpr double LEFT_POSITION_DEGREES = 0.0;
constexpr double CENTER_POSITION_DEGREES = 0.0;
constexpr double RIGHT_POSITION_DEGREES = 0.0;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = false;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = false;

// Vision-follow (RotateControlMode::VISION_FOLLOW). Deadband/slow-zone are
// image-space pixels, same values tuned on the real A5 bench through
// 2026-10-05 (see docs/codex-handoff.md) -- re-verify once this runs on the
// actual turret mechanism/pin, they are not guaranteed to transfer as-is.
// Cruise/creep are normalized to this axis's full +-1.0 command range using
// the same 300us/150us : 500us ratio the bench used.
constexpr int VISION_DEADBAND_PX = 20;
constexpr int VISION_SLOW_ZONE_PX = 70;
constexpr double VISION_CREEP_COMMAND = 0.3;
constexpr double VISION_CRUISE_COMMAND = 0.6;

constexpr double PROFILE_CRUISE_COMMAND = 1.0;
constexpr double PROFILE_MINIMUM_APPROACH_RATIO = 0.10;
constexpr double PROFILE_DECELERATION_FRACTION = 0.20;

}

namespace Flywheel {

constexpr bool INVERTED = false;

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;

// Auto-fire sequence (Auto::update()). No RPM/tachometer feedback exists for
// this Falcon 500 (PWM command path is one-way) -- SHOOT_COMMAND/SPIN_UP_MS
// are both provisional placeholders standing in for real speed feedback.
// Re-tune against real shots once hardware is available; do not trust these
// numbers unattended.
constexpr double SHOOT_COMMAND = 0.8;
constexpr uint32_t SPIN_UP_MS = 1500;

}

namespace Auto {

// How long Dribbler::setFeedAllowed(true) stays on to push one ball into the
// spinning flywheel. No working ball sensor exists to confirm this directly
// (see Dribbler.cpp) -- this is a time-based approximation, same spirit as
// Flywheel::SPIN_UP_MS.
constexpr uint32_t FEED_DURATION_MS = 500;

// Pause after a completed feed before the sequence can start again, so one
// queued shot can't immediately re-trigger itself.
constexpr uint32_t COOLDOWN_MS = 300;

// MechLink::fireChannel threshold for the semi-auto fire-confirm button
// (momentary, edge-triggered in Auto::update()).
constexpr int FIRE_BUTTON_THRESHOLD = 1500;

}

namespace OperatorMode {

// MechLink::modeChannel thresholds (1500-2000 split into thirds, see
// tools/stm32_elrs_ground_bridge's mode_switch_to_pulse_us): the real
// keypad doesn't exist yet -- re-measure its actual output against these
// once it's built and update here; nothing else needs to change.
constexpr int SEMI_AUTO_THRESHOLD = 1667;
constexpr int FULL_MANUAL_THRESHOLD = 1833;

}

namespace ShotTable {

// Distance (meters, measured from Vision) -> flywheel command -> target
// elevation angle. flywheelCommand is an OPEN-LOOP COMMAND STRENGTH on the
// same -1.0~1.0 scale Shooter::setFlywheelOpenLoop() already uses, NOT a
// real RPM -- the flywheel (Falcon 500) has no speed feedback, so this
// stores the actual actuator value to send, the same convention
// all_robot_017's legacy half_automode_vertical_falconESC/angle tables
// used (store real actuator values, not a fake RPM<->command conversion).
// angleDegrees is computed but not yet driven to Angle anywhere -- Angle's
// closed-loop calibration isn't done (see Angle::POSITION_LIMITS_CALIBRATED
// above), so there's nowhere safe to send it yet.
//
// All entries below are placeholder values (no real shots have been
// measured). Re-measure and fill in real numbers before relying on this at
// competition -- it exists so the lookup/interpolation code has something
// concrete to run against, not because these numbers are correct.
struct Entry {
    double distanceMeters;
    double flywheelCommand;
    double angleDegrees;
};

constexpr Entry TABLE[] = {
    {2.0, 0.70, 15.0},
    {2.5, 0.72, 17.0},
    {3.0, 0.75, 19.0},
    {3.5, 0.78, 21.0},
    {4.0, 0.80, 23.0},
    {4.5, 0.82, 25.0},
    {5.0, 0.84, 27.0},
    {5.5, 0.86, 29.0},
    {6.0, 0.88, 31.0},
    {6.5, 0.90, 33.0},
    {7.0, 0.92, 35.0},
    {7.5, 0.94, 37.0},
    {8.0, 0.96, 39.0},
};
constexpr int TABLE_COUNT = sizeof(TABLE) / sizeof(TABLE[0]);

}

static_assert(Rotate::VISION_SLOW_ZONE_PX > Rotate::VISION_DEADBAND_PX,
              "Vision slow zone must be outside the deadband");
static_assert(Rotate::VISION_CREEP_COMMAND > 0.0 &&
              Rotate::VISION_CREEP_COMMAND <= Rotate::VISION_CRUISE_COMMAND,
              "Vision creep command must be positive and no faster than cruise");
static_assert(Rotate::VISION_CRUISE_COMMAND > 0.0 &&
              Rotate::VISION_CRUISE_COMMAND <= 1.0,
              "Vision cruise command out of range");
static_assert(Rotate::PROFILE_DECELERATION_FRACTION > 0.0 &&
              Rotate::PROFILE_DECELERATION_FRACTION <= 1.0,
              "Invalid rotate deceleration fraction");
static_assert(Rotate::PROFILE_MINIMUM_APPROACH_RATIO >= 0.0 &&
              Rotate::PROFILE_MINIMUM_APPROACH_RATIO <= 1.0,
              "Invalid rotate minimum approach ratio");
static_assert(!Rotate::POSITION_CALIBRATED ||
                  Rotate::LEFT_POSITION_RAW < Rotate::CENTER_POSITION_RAW,
              "Rotate left calibration must be below center");
static_assert(!Rotate::POSITION_CALIBRATED ||
                  Rotate::CENTER_POSITION_RAW < Rotate::RIGHT_POSITION_RAW,
              "Rotate center calibration must be below right");
static_assert(OperatorMode::SEMI_AUTO_THRESHOLD < OperatorMode::FULL_MANUAL_THRESHOLD,
              "Operator mode thresholds must be ordered");
static_assert(ShotTable::TABLE_COUNT >= 2, "Shot table needs at least two entries to interpolate");

}
