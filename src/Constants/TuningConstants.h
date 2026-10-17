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
constexpr bool INVERTED = false;
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

// The operator's turret jog (D-pad / wireless nudge left-right, full
// manual): on the robot (2026-10-17) "right" turned the turret left. The
// vision-follow direction (INVERTED, validated on the bench) is right, so
// only the jog is flipped here -- flipping INVERTED would reverse vision
// follow too. The auto-mode aim offset (+ = right) is unaffected.
constexpr bool MANUAL_JOG_INVERTED = true;

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

// Operator fine-adjust in the auto modes (Jeremy, 2026-10-16): each arrow
// press (wireless: each nudge from center) adds one step; a new target hole,
// Enable and disarm clear both. PROVISIONAL step sizes -- tune on the field.
//   left/right: the turret aims this many pixels right (+) / left (-) of
//     the hole's center (vision-follow drives tx + offset to 0).
//   up/down: added to the shot table's elevation pulse; only once the
//     table is measured (ShotTable::ELEVATION_MEASURED), until then up/down
//     stay the manual elevation jog.
constexpr double AIM_OFFSET_STEP_PX = 5.0;
constexpr double AIM_OFFSET_MAX_PX = 100.0;
constexpr double ELEVATION_OFFSET_STEP_US = 10.0;
constexpr double ELEVATION_OFFSET_MAX_US = 200.0;

}

namespace ShotTable {

// Camera distance (meters, Vision::getDistance() / 1000) -> flywheel command
// and elevation servo pulse, the actual actuator values to send:
//   flywheelCommand   open-loop Falcon command 0..1 (no speed feedback,
//                     so not an RPM)
//   elevationPulseUs  the elevation servo's target pulse, as shown on the
//                     Dashboard "elevation pulse" readout (a positional
//                     servo: a pulse is a position, no angle calibration).
//
// ONE TABLE PER HEIGHT, not per hole (Jeremy, 2026-10-16): the trajectory
// depends on the hole's height and the distance to it, so the three holes
// of a row share a table. Shooting a side hole at an angle needs no extra
// table: the camera measures the distance to that hole itself (Orin
// DEPTH_DISTANCE_MODE "range", straight-line camera -> hole), which is
// longer at an angle, so the lookup lands on a farther row by itself.
// Heights from the rules drawing (hole centers above the floor, approx.,
// confirm on the field): bottom 120 cm, middle 180 cm, top 240 cm, bonus
// holes 290 cm.
//
// Rows every 0.5 m from 2.5 m (the barrier is 250 cm from the net, so the
// robot is never closer) to 8.5 m; the key is the CAMERA's distance, so
// read it off the Dashboard while locked on the hole, never a tape measure
// (the camera's own bias is then calibrated in). Between rows: linear
// interpolation. Below 2.5 m: the 2.5 m values. Beyond 8.5 m: extrapolated
// along the 8.0 -> 8.5 m trend (Jeremy) -- the flywheel still stops at
// 100% and the elevation at the servo's limits.
//
// Fill it from test shots: at each distance, adjust until it scores and
// copy the Dashboard's pulse and the Falcon % here. Then use
// ELEVATION_TRIM_US below for a single hole that is consistently high/low.
//
// PLACEHOLDERS: no real shots have been measured; all four tables are the
// same guess. The elevation column is all neutral (1500 us) and is NOT
// applied until ELEVATION_MEASURED is set true -- placeholder pulses must
// never move the real elevation. Until then the operator keeps the
// elevation in every mode.
struct Entry {
    double distanceMeters;
    double flywheelCommand;
    double elevationPulseUs;
};
constexpr bool ELEVATION_MEASURED = false;

// Bottom row, ~120 cm: keys 1, 4, 7 = Orin vision ids 10, 11, 12.
constexpr Entry BOTTOM_ROW[] = {
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
    {8.5, 0.98, 1500.0},
};
constexpr int BOTTOM_ROW_COUNT = sizeof(BOTTOM_ROW) / sizeof(BOTTOM_ROW[0]);

// Middle row, ~180 cm: keys 2, 5, 8 = Orin vision ids 7, 8, 9.
constexpr Entry MIDDLE_ROW[] = {
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
    {8.5, 0.98, 1500.0},
};
constexpr int MIDDLE_ROW_COUNT = sizeof(MIDDLE_ROW) / sizeof(MIDDLE_ROW[0]);

// Top row, ~240 cm: keys 3, 6, 9 = Orin vision ids 4, 5, 6.
constexpr Entry TOP_ROW[] = {
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
    {8.5, 0.98, 1500.0},
};
constexpr int TOP_ROW_COUNT = sizeof(TOP_ROW) / sizeof(TOP_ROW[0]);

// Bonus small holes, ~290 cm: Orin vision ids 1, 2, 3 (not selectable;
// only the Orin's own pick in full auto with no hole selected).
constexpr Entry BONUS_ROW[] = {
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
    {8.5, 0.98, 1500.0},
};
constexpr int BONUS_ROW_COUNT = sizeof(BONUS_ROW) / sizeof(BONUS_ROW[0]);

// Per-hole elevation trim (us, added to its row's pulse), indexed by Orin
// vision id. 0 unless one hole turns out consistently high or low.
constexpr double ELEVATION_TRIM_US[13] = {
    0.0,            // 0: no target
    0.0, 0.0, 0.0,  // 1-3: bonus holes
    0.0, 0.0, 0.0,  // 4-6: top row, keys 3 6 9
    0.0, 0.0, 0.0,  // 7-9: middle row, keys 2 5 8
    0.0, 0.0, 0.0,  // 10-12: bottom row, keys 1 4 7
};

struct Table {
    const Entry* entries;
    int count;
};

// Orin vision id -> its height's table; {nullptr, 0} for 0 / unknown ids
// (no table = Auto never fires at it).
inline Table tableForVisionId(int visionId) {
    if (visionId >= 1 && visionId <= 3) return {BONUS_ROW, BONUS_ROW_COUNT};
    if (visionId >= 4 && visionId <= 6) return {TOP_ROW, TOP_ROW_COUNT};
    if (visionId >= 7 && visionId <= 9) return {MIDDLE_ROW, MIDDLE_ROW_COUNT};
    if (visionId >= 10 && visionId <= 12) return {BOTTOM_ROW, BOTTOM_ROW_COUNT};
    return {nullptr, 0};
}

inline double elevationTrimUs(int visionId) {
    return visionId >= 0 && visionId <= 12 ? ELEVATION_TRIM_US[visionId] : 0.0;
}

}

static_assert(Rotate::VISION_MAX_COMMAND > 0.0 && Rotate::VISION_MAX_COMMAND <= 1.0,
              "Vision-follow command limit out of range");
static_assert(Rotate::VISION_MAX_INTEGRAL_COMMAND >= 0.0 &&
              Rotate::VISION_MAX_INTEGRAL_COMMAND <= Rotate::VISION_MAX_COMMAND,
              "Vision-follow integral limit must fit inside the command limit");
static_assert(Rotate::VISION_kIZONE_PX >= 0.0 && Rotate::VISION_READY_TOLERANCE_PX > 0.0,
              "Vision-follow pixel windows must be non-negative");
static_assert(ShotTable::BOTTOM_ROW_COUNT >= 2 && ShotTable::MIDDLE_ROW_COUNT >= 2 &&
              ShotTable::TOP_ROW_COUNT >= 2 && ShotTable::BONUS_ROW_COUNT >= 2,
              "Each shot table needs at least two entries to interpolate");
static_assert(Auto::AIM_OFFSET_STEP_PX > 0.0 && Auto::AIM_OFFSET_MAX_PX >= Auto::AIM_OFFSET_STEP_PX &&
              Auto::ELEVATION_OFFSET_STEP_US > 0.0 &&
              Auto::ELEVATION_OFFSET_MAX_US >= Auto::ELEVATION_OFFSET_STEP_US,
              "Fine-adjust steps must be positive and fit inside their limits");

}
