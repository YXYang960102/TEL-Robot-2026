#pragma once

#include <stdint.h>

#include "../Vision/OperatorTarget.h"
#include "ShotTableData.h"

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

constexpr double COMMAND_FILTER_TIME_CONSTANT_MS = 80.0; // chassis output filter, ~63% in this time
}

namespace Angle {

constexpr bool INVERTED = true; // flips the D-pad / ANG jog

}

namespace Rotate {

constexpr bool INVERTED = false;

constexpr bool MANUAL_JOG_INVERTED = true; // flips only the operator turret jog

constexpr double VISION_kP = 0.01; // command per px of tx
constexpr double VISION_kI = 0.02; // inside VISION_kIZONE_PX only
constexpr double VISION_kD = 0.0;
constexpr double VISION_kIZONE_PX = 20.0;
constexpr double VISION_kFF = 0.0;
constexpr double VISION_MAX_COMMAND = 1.0;
constexpr double VISION_MAX_INTEGRAL_COMMAND = 0.12;
constexpr double VISION_READY_TOLERANCE_PX = 6.0; // |tx| within this = locked
constexpr double VISION_FRAME_kD = 0.0008; // command per px/s
constexpr double VISION_STALL_COMMAND = 0.17; // push when stuck just outside the lock
constexpr double VISION_STALL_RATE_PX_PER_S = 30.0;
constexpr uint32_t VISION_STALL_DELAY_MS = 200;
constexpr uint32_t VISION_STALL_PULSE_MS = 80; // push length
constexpr bool VISION_HOLD_I_IN_TOLERANCE = true;
constexpr double SEARCH_COMMAND = 0.3; // turn speed toward an unseen hole
constexpr unsigned long SEARCH_MAX_MS = 3000;
constexpr unsigned long LOST_TURN_MS = 1000; // keep turning toward the last-seen side
constexpr double LOST_EDGE_PX = 256.0;
constexpr bool SWEEP_ENABLED = true; // nothing seen: sweep left first, then right
constexpr unsigned long SWEEP_START_MS = 500; // nothing seen this long = sweep
constexpr double SWEEP_LEFT_DEG = -90.0;
constexpr double SWEEP_RIGHT_DEG = 35.0;

}

namespace Flywheel {

constexpr bool INVERTED = false; // false = 1500-2000 us shoots

constexpr int STOP_PULSE_OFFSET_US = -10; // stopped = 1490 us (the Falcon creeps at 1500)

constexpr double SHOOT_COMMAND = 0.2; // pre-spin only if the default hole has no table
constexpr uint32_t SPIN_UP_MS = 1500; // spun up this long after the command changes

constexpr double RAMP_UP_PER_SECOND = 0.5; // 0 -> 80% in 1.6 s
constexpr double RAMP_DOWN_PER_SECOND = 0.5; // 80% -> stop in 1.6 s

}

namespace AutoPath {
constexpr double DRIVE_COMMAND = 1.0; // both wheels, straight
constexpr double TURN_COMMAND = 0.35;
constexpr unsigned long DIAGONAL_TURN_MS = 0; // left 1 / right 1: first turn
constexpr unsigned long DIAGONAL_DRIVE_MS = 3000; // left 1 / right 1: straight
constexpr int8_t LEFT1_LAST_TURN_DIRECTION = 1; // +1 right, -1 left
constexpr unsigned long LEFT1_LAST_TURN_MS = 500;
constexpr int8_t RIGHT1_LAST_TURN_DIRECTION = 1; // +1 right, -1 left
constexpr unsigned long RIGHT1_LAST_TURN_MS = 1000;
constexpr unsigned long FORWARD_DRIVE_MS = 4000; // left 2 / right 2: straight
constexpr unsigned long SIDE_TURN_MS = 500; // left 2 / right 2: turn
constexpr unsigned long SIDE_DRIVE_MS = 1000; // left 2 / right 2: second straight
constexpr unsigned long SIDE_TURN_BACK_MS = 1000; // left 2 / right 2: turn back
constexpr unsigned long PAUSE_MS = 300; // standing still between moves
constexpr unsigned long PENDING_MS = 1500; // AUTO,n waits this long for full auto
constexpr double STICK_TAKEOVER = 0.20; // stick past this stops the path
constexpr bool RIGHT_PATHS_HOLD_TURRET_UNTIL_SEEN = true; // right paths: turret at 0 deg until the hole is seen
constexpr bool RIGHT_TURN_LEFT_WHEEL_BACK = true; // turn right = left wheel back
}

namespace Auto {

constexpr double ELEVATION_SLEW_US_PER_SECOND = 400.0;
constexpr double ELEVATION_READY_TOLERANCE_US = 10.0; // autos fire within this of the table
constexpr unsigned long ELEVATION_SETTLE_MS = 500; // full auto: arrived this long before firing
constexpr unsigned long SEMI_FIRE_HOLD_MS = 1000; // semi F waits this long for ready
constexpr uint8_t AUTO_PATH_ELEVATION_HOLE = 4; // elevation preset while a path drives
constexpr double AUTO_PATH_ELEVATION_DISTANCE_M = 4.5;

constexpr bool ROW_CYCLE_ENABLED = true; // full auto hole cycle
constexpr uint8_t SHOTS_PER_HOLE = 3; // first half
constexpr uint8_t SECOND_HALF_SHOTS_PER_HOLE = 2; // second half
constexpr uint8_t BONUS_SHOTS_PER_HOLE = 1; // lit Q W E, both halves
constexpr bool CYCLE_BOTTOM_UP = true; // columns bottom to top, from the nearest bottom hole
constexpr unsigned long HOLE_SWITCH_DELAY_MS = 2000; // wait after a hole's last ball
constexpr unsigned long HOLE_NOT_FOUND_SKIP_MS = 7000; // no shot and no lock this long = next hole; 0 = never
constexpr uint16_t FULL_AUTO_EXCLUDED_HOLES = 0; // bit n = hole n
constexpr bool CYCLE_SKIPS_BONUS_ROW = true; // Q W E only when lit
constexpr uint16_t BONUS_ROW_HOLES = (1u << 10) | (1u << 11) | (1u << 12); // holes 10-12
constexpr bool LIT_BONUS_PRIORITY = true; // a lit box first
constexpr unsigned long AIM_SWITCH_SETTLE_MS = 500; // no fire this long after an aim change
constexpr unsigned long LIT_DARK_MS = 1000; // dark this long before the box may fire again

constexpr uint32_t TARGET_HOLD_MS = 500; // keep the last shot after the target drops

constexpr uint8_t DISTANCE_MEDIAN_FRAMES = 5; // median of depth frames
constexpr double DISTANCE_LATCH_BAND_M = 0.05; // latched distance follows moves past this
constexpr unsigned long DISTANCE_LATCH_SETTLE_MS = 300;

constexpr uint32_t COOLDOWN_MS = 300; // between auto shots

constexpr unsigned long AIM_JOG_SETTLE_MS = 300;
constexpr double AIM_JOG_COMMAND = 0.2; // hand turn speed in the autos / test
constexpr bool AIM_OFFSET_ENABLED = false; // false = horizontal trim off
constexpr double AIM_OFFSET_MAX_PX = 300.0;
constexpr double ELEVATION_JOG_US_PER_SECOND = 200.0;
constexpr double ELEVATION_OFFSET_MAX_US = 200.0;

}

namespace ShotTable {

using Entry = ShotTableData::Point;
constexpr bool ELEVATION_MEASURED = true; // autos use the table's elevation

constexpr bool EXTRAPOLATE_PAST_LAST_ENTRY = false; // false = hold the last entry

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

inline Table tableForVisionId(int visionId, uint8_t color = 0) {
    if (color >= ShotTableData::COLOR_COUNT) color = 0;
    const auto& counts = ShotTableData::HOLE_COUNTS[color];
    const auto& points = ShotTableData::HOLE_POINTS[color];
    for (uint8_t hole = 1; hole <= OperatorTarget::LAST_HOLE; hole++) {
        if (OperatorTarget::visionIdForHole(hole) != visionId) continue;
        if (counts[hole] > 0) {
            return {points[hole], counts[hole]};
        }
        const uint8_t center = hole >= OperatorTarget::FIRST_BONUS_HOLE
            ? static_cast<uint8_t>(OperatorTarget::FIRST_BONUS_HOLE + 1)
            : static_cast<uint8_t>(4 + (hole - 1) % 3);
        if (counts[center] > 0) {
            return {points[center], counts[center]};
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
static_assert(Rotate::VISION_STALL_COMMAND >= 0.0 && Rotate::VISION_STALL_COMMAND <= 0.3 &&
                  Rotate::VISION_STALL_RATE_PX_PER_S > 0.0,
              "Turret stall push: a small creep, not a drive");
static_assert(Rotate::VISION_kIZONE_PX >= 0.0 && Rotate::VISION_READY_TOLERANCE_PX > 0.0,
              "Vision-follow pixel windows must be non-negative");
static_assert(AutoPath::DRIVE_COMMAND > 0.0 && AutoPath::DRIVE_COMMAND <= 1.0 &&
                  AutoPath::TURN_COMMAND > 0.0 && AutoPath::TURN_COMMAND <= 1.0,
              "Auto path commands are 0..1 ratios");
static_assert(AutoPath::STICK_TAKEOVER > 0.0 && AutoPath::STICK_TAKEOVER < 1.0,
              "Stick takeover threshold is a 0..1 command");
static_assert(Auto::AIM_JOG_COMMAND > 0.0 && Auto::AIM_JOG_COMMAND <= 1.0, "Hand-turn speed is a 0..1 command");
static_assert(Auto::AIM_OFFSET_MAX_PX > 0.0 && Auto::AIM_OFFSET_MAX_PX < 400.0 &&
              Auto::ELEVATION_JOG_US_PER_SECOND > 0.0 &&
              Auto::ELEVATION_OFFSET_MAX_US > 0.0,
              "Fine-adjust steps must be positive and fit inside their limits");

}
