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

// Chassis output smoothing (Chassis::update()). An exponential filter on
// the final left/right command before it becomes a pulse, time constant in
// ms: output tracks a step input to ~63% in this many ms, ~95% in 3x this.
// Smooths frame-to-frame stick/RF noise so it doesn't land directly on the
// PWM pins; does NOT apply to stop()/disable(), which still snap straight
// to neutral. 80 ms is a first guess -- fast enough that an intentional
// stick move still feels immediate, slow enough to average out single-frame
// jitter at ~10-15 ms/loop. TUNE ON THE ROBOT.
constexpr double COMMAND_FILTER_TIME_CONSTANT_MS = 80.0;
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

// true since the 2026-10-17 robot test (Jeremy): the slider's direction is
// the shooting direction with the pulse going below 1500 us.
constexpr bool INVERTED = true;

// Stop trim (Jeremy, 2026-10-17): this Falcon creeps backwards at the
// 1500 us neutral (D25 and GND checked), so "stopped" (command 0: disabled,
// before Y, slider at 0) sends this many us toward the slider's spin
// direction instead -- 1490 us with INVERTED = true. TUNE ON THE ROBOT:
// still creeping backwards -> raise (15, 20); creeping forwards -> lower
// (5). 0 = plain 1500 (a normally configured Falcon). Capped at 30 us in
// ShooterConstants.h so a typo can't spin a disabled flywheel.
constexpr int STOP_TRIM_US = 10;

// Auto-fire sequence (Auto::update()). No RPM/tachometer feedback exists for
// this Falcon 500 (PWM command path is one-way), so "ready" is time-based:
// the flywheel counts as spun up SPIN_UP_MS after its command last changed
// (Shooter::updateFlywheelReady). PROVISIONAL -- tune against real shots.
// SHOOT_COMMAND: the autos' pre-spin with no target in view (Jeremy,
// 2026-10-18: 50%, then 20%); with a target the shot table's flywheel
// value (+ the LT / RT offset) is used.
// It no longer sets the "ready" threshold (that made every table value
// below it -- 42%, 46% -- never ready).
constexpr double SHOOT_COMMAND = 0.2;
constexpr uint32_t SPIN_UP_MS = 1500;

// Ramp-up limit on the Falcon command, in every mode (2026-10-18: an
// instant 46% -> 80% jump in semi auto, together with the elevation move,
// drew enough current to reset the Mega). 0.5 = 50% per second, so 0 -> 80%
// takes 1.6 s. The "ready" wait (SPIN_UP_MS) starts once the ramp has
// reached the command.
constexpr double RAMP_UP_PER_SECOND = 0.5;
// Ramp-down limit, also used after a disable / stop (Jeremy, 2026-10-18:
// this Falcon brakes hard at neutral, so the flywheel spins down instead,
// still driven, then neutral). 0.5 = from 80% to stop in 1.6 s.
constexpr double RAMP_DOWN_PER_SECOND = 0.5;

}

namespace Auto {

// Auto moves the elevation toward the shot table's pulse at most this fast
// instead of jumping (2026-10-18, current limiting with the flywheel ramp):
// 400 us/s, e.g. 2063 -> 2442 us in ~0.95 s. The D-pad jog in full manual
// keeps its own Angle::POSITION_RATE_US_PER_SECOND.
constexpr double ELEVATION_SLEW_US_PER_SECOND = 400.0;

// After the target drops out of a frame, Auto keeps its last shot (flywheel
// and elevation) this long before falling back to the pre-spin (2026-10-18:
// single-frame vision dropouts pulsed the flywheel). Firing still needs a
// target in the current frame.
constexpr uint32_t TARGET_HOLD_MS = 500;

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
// 20 px (2026-10-18, was 5): 5 px x kP 0.01 = a ~25 us turret command,
// inside the servo's deadband, so a press barely moved the turret.
constexpr double AIM_OFFSET_STEP_PX = 20.0;
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
// The key is the CAMERA's distance: read it off the Dashboard while locked
// on the hole, never a tape measure (the camera's own bias is then
// calibrated in). Between entries: linear interpolation. Below the first
// entry: the first entry's values. Beyond the last entry: see
// EXTRAPOLATE_PAST_LAST_ENTRY.
//
// Fill it from test shots: at each distance, adjust until it scores and
// copy the Dashboard's pulse and the Falcon % here. Then use
// ELEVATION_TRIM_US below for a single hole that is consistently high/low.
//
// FIRST FIELD DATA (Jeremy, 2026-10-18): the center column (Dashboard keys
// 4 / 5 / 6), each applied to its whole row, at 2.5 m and 5 m only.
// PROVISIONAL -- add 3.0 / 3.5 / 4.0 / 4.5 m and beyond. The top row's two
// points are two different shots (2.5 m: low 1010 us at 100%; 5 m: lob
// 2442 us at 80%), so the interpolation between them mixes the two and
// probably misses; measure the top row's middle distances first.
struct Entry {
    double distanceMeters;
    double flywheelCommand;
    double elevationPulseUs;
};
// The elevation column is measured now: the autos set the elevation from
// it (plus the operator's up/down fine-adjust).
constexpr bool ELEVATION_MEASURED = true;

// Beyond the last entry: false = hold the last entry's values (Jeremy,
// 2026-10-18, while the tables stop at 5 m -- two points extrapolate
// wildly, e.g. the middle row would fall to 24% at 6 m); true = extend the
// last two entries' trend (his rule for past 8.5 m, once the tables reach
// it).
constexpr bool EXTRAPOLATE_PAST_LAST_ENTRY = false;

// Bottom row, ~120 cm: keys 1, 4, 7 = Orin vision ids 10, 11, 12.
constexpr Entry BOTTOM_ROW[] = {
    {2.5, 0.42, 2407.0},
    {5.0, 0.43, 2183.0},
};
constexpr int BOTTOM_ROW_COUNT = sizeof(BOTTOM_ROW) / sizeof(BOTTOM_ROW[0]);

// Middle row, ~180 cm: keys 2, 5, 8 = Orin vision ids 7, 8, 9.
constexpr Entry MIDDLE_ROW[] = {
    {2.5, 1.00, 2500.0},
    {5.0, 0.46, 2063.0},
};
constexpr int MIDDLE_ROW_COUNT = sizeof(MIDDLE_ROW) / sizeof(MIDDLE_ROW[0]);

// Top row, ~240 cm: keys 3, 6, 9 = Orin vision ids 4, 5, 6.
constexpr Entry TOP_ROW[] = {
    {2.5, 1.00, 1010.0},
    {5.0, 0.80, 2442.0},
};
constexpr int TOP_ROW_COUNT = sizeof(TOP_ROW) / sizeof(TOP_ROW[0]);

// Bonus small holes (~290 cm, Orin vision ids 1-3): no data yet, so no
// table -- the autos never fire at them (and don't move the elevation for
// them). Only reachable as the Orin's own pick with no hole selected.

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
// and the bonus holes 1-3 (no table = Auto never fires at it).
inline Table tableForVisionId(int visionId) {
    if (visionId >= 4 && visionId <= 6) return {TOP_ROW, TOP_ROW_COUNT};
    if (visionId >= 7 && visionId <= 9) return {MIDDLE_ROW, MIDDLE_ROW_COUNT};
    if (visionId >= 10 && visionId <= 12) return {BOTTOM_ROW, BOTTOM_ROW_COUNT};
    return {nullptr, 0};
}

inline double elevationTrimUs(int visionId) {
    return visionId >= 0 && visionId <= 12 ? ELEVATION_TRIM_US[visionId] : 0.0;
}

}

static_assert(Flywheel::RAMP_UP_PER_SECOND > 0.0 && Flywheel::RAMP_DOWN_PER_SECOND > 0.0 &&
              Auto::ELEVATION_SLEW_US_PER_SECOND > 0.0,
              "Flywheel ramp and elevation slew must be positive");
static_assert(Manual::COMMAND_FILTER_TIME_CONSTANT_MS >= 0.0,
              "Chassis command filter time constant must not be negative");
static_assert(Rotate::VISION_MAX_COMMAND > 0.0 && Rotate::VISION_MAX_COMMAND <= 1.0,
              "Vision-follow command limit out of range");
static_assert(Rotate::VISION_MAX_INTEGRAL_COMMAND >= 0.0 &&
              Rotate::VISION_MAX_INTEGRAL_COMMAND <= Rotate::VISION_MAX_COMMAND,
              "Vision-follow integral limit must fit inside the command limit");
static_assert(Rotate::VISION_kIZONE_PX >= 0.0 && Rotate::VISION_READY_TOLERANCE_PX > 0.0,
              "Vision-follow pixel windows must be non-negative");
static_assert(ShotTable::BOTTOM_ROW_COUNT >= 2 && ShotTable::MIDDLE_ROW_COUNT >= 2 &&
              ShotTable::TOP_ROW_COUNT >= 2,
              "Each shot table needs at least two entries to interpolate");
static_assert(Auto::AIM_OFFSET_STEP_PX > 0.0 && Auto::AIM_OFFSET_MAX_PX >= Auto::AIM_OFFSET_STEP_PX &&
              Auto::ELEVATION_OFFSET_STEP_US > 0.0 &&
              Auto::ELEVATION_OFFSET_MAX_US >= Auto::ELEVATION_OFFSET_STEP_US,
              "Fine-adjust steps must be positive and fit inside their limits");

}
