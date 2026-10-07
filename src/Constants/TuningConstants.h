#pragma once

// Centralizes the parameters that actually get changed at competition on
// this branch -- motor direction and manual-drive shaping -- organized by
// mechanism namespace. ChassisConstants.h pulls these via `using
// namespace` so each value has exactly one definition; pins and servo PWM
// endpoints stay in ChassisConstants.h (wiring facts, not "turn the knob"
// values).
//
// The Shooter branch has its own TuningConstants.h following this same
// convention/namespace-per-mechanism pattern, scoped to the mechanisms
// that exist there (Angle/Rotate/Flywheel/Auto/OperatorMode/ShotTable).
// When branches merge, the two files combine by keeping both branches'
// namespace blocks side by side, the same way ShooterConstants.h/Pins.h
// conflicts were resolved during the earlier Shooter -> dev merge.
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

namespace StartingSide {
// MechLink's auxiliary channel (tools/stm32_elrs_common/src/
// tel_channel_map.h's TEL_CH_AUXILIARY; the retired SBUS class's
// AUXILIARY_CHANNEL before this branch moved off SBUS) threshold used to
// latch LEFT vs RIGHT once at boot. Provisional -- the real starting-side
// switch doesn't exist yet (see tools/stm32_elrs_ground_bridge/README.md);
// re-measure once it's built.
constexpr int THRESHOLD_US = 1500; // >= threshold -> RIGHT, < threshold -> LEFT
}

}
