#pragma once

#include <stdint.h>

#include "../Vision/OperatorTarget.h"
#include "ShotTableData.h"

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

// true since 2026-10-25 (Jeremy, on the robot): flips the D-pad / ANG jog
// direction (and with it the autos' up/down fine-adjust, which follows the
// jog). The shot table's elevation pulses are absolute and do not depend
// on it.
constexpr bool INVERTED = true;

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

// Which side of 1500 us is the shooting direction (slider, LT / RT, shot
// table): false = above (1500-2000), true = below (1000-1500). false since
// 2026-10-25 (Jeremy, on the robot); it was true from 2026-10-17.
constexpr bool INVERTED = false;

// Stop pulse offset (Jeremy, 2026-10-17): this Falcon creeps at the plain
// 1500 us neutral (D25 and GND checked), so "stopped" (command 0: disabled,
// before Y, slider at 0, end of a spin-down) sends 1500 + this, in absolute
// microseconds -- NOT relative to INVERTED. It is a property of this
// Falcon, so flipping INVERTED must not move it (2026-10-25: the old
// "toward the spin direction" trim would have jumped from 1490 to 1510).
// -10 = 1490 us, the value used since 2026-10-17. TUNE ON THE ROBOT: if it
// creeps one way, move 5 us the other way; 0 = plain 1500 (a normally
// configured Falcon). Limited to +-30 us in ShooterConstants.h so a typo
// can't spin a disabled flywheel.
constexpr int STOP_PULSE_OFFSET_US = -10;

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
// ONE TABLE PER DASHBOARD HOLE (keys 1-9, bonus Q W E = holes 10-12 since
// 2026-10-27), generated from the Excel tools/shot_table/shot_table.xlsx
// into ShotTableData.h (2026-10-18): edit the Excel and run
// `python3 tools/shot_table/apply.py --upload`, never the generated header.
// A grid hole with no points uses its row's center hole (1/7 -> 4,
// 2/8 -> 5, 3/9 -> 6); with none there either, the autos never fire at it.
// A bonus hole with no points of its own is never fired at (no fallback:
// it is smaller and higher than any grid hole).
//
// The key is the CAMERA's distance: read it off the Dashboard while locked
// on the hole, never a tape measure (the camera's own bias is then
// calibrated in). Between entries: linear interpolation. Below the first
// entry: the first entry's values. Beyond the last entry: see
// EXTRAPOLATE_PAST_LAST_ENTRY. ELEVATION_TRIM_US below adds a per-hole
// offset on top, if ever needed.
using Entry = ShotTableData::Point;
// The elevation column is measured now: the autos set the elevation from
// it (plus the operator's up/down fine-adjust).
constexpr bool ELEVATION_MEASURED = true;

// Beyond the last entry: false = hold the last entry's values (Jeremy,
// 2026-10-18, while the tables stop at 5 m -- two points extrapolate
// wildly, e.g. the middle row would fall to 24% at 6 m); true = extend the
// last two entries' trend (his rule for past 8.5 m, once the tables reach
// it).
constexpr bool EXTRAPOLATE_PAST_LAST_ENTRY = false;

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

// Orin vision id -> its hole's table (Dashboard hole via OperatorTarget),
// or, for a grid hole with no points, that row's center hole; {nullptr, 0}
// for 0, unknown ids, a bonus hole without points and an empty row (Auto
// never fires).
inline Table tableForVisionId(int visionId) {
    for (uint8_t hole = 1; hole <= OperatorTarget::LAST_HOLE; hole++) {
        if (OperatorTarget::visionIdForHole(hole) != visionId) continue;
        if (ShotTableData::HOLE_COUNTS[hole] > 0) {
            return {ShotTableData::HOLE_POINTS[hole], ShotTableData::HOLE_COUNTS[hole]};
        }
        if (hole >= OperatorTarget::FIRST_BONUS_HOLE) {
            return {nullptr, 0};
        }
        const uint8_t center = static_cast<uint8_t>(4 + (hole - 1) % 3); // same row, middle column
        if (ShotTableData::HOLE_COUNTS[center] > 0) {
            return {ShotTableData::HOLE_POINTS[center], ShotTableData::HOLE_COUNTS[center]};
        }
        return {nullptr, 0};
    }
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
static_assert(Auto::AIM_OFFSET_STEP_PX > 0.0 && Auto::AIM_OFFSET_MAX_PX >= Auto::AIM_OFFSET_STEP_PX &&
              Auto::ELEVATION_OFFSET_STEP_US > 0.0 &&
              Auto::ELEVATION_OFFSET_MAX_US >= Auto::ELEVATION_OFFSET_STEP_US,
              "Fine-adjust steps must be positive and fit inside their limits");

}
