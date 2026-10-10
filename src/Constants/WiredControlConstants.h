#pragma once

#include <stdint.h>

// Production wired Dashboard command protocol (USB Serial, separate from
// Serial2/MechLink's wireless ELRS link). Added 2026-10-11 dev integration,
// see docs/codex-handoff.md. Values match the existing bench convention
// (ShooterBenchConstants::COMMAND_TIMEOUT_MS/COMMAND_BUFFER_SIZE) rather
// than inventing new ones.
namespace WiredControlConstants {

constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 24;

// Keyboard keys are on/off (-1/0/1 on the wire); these scale that into the
// actual open-loop command for first-power-on bring-up. Rotate/Angle match
// ShooterBenchConstants' 0.10 "limited authority for the first unloaded
// mechanism tests"; chassis has no prior bench value, 0.30 is a
// conservative starting point. Raise only after real-hardware checks.
constexpr double CHASSIS_MANUAL_SCALE = 0.30;
constexpr double ROTATE_MANUAL_SCALE = 0.10;
constexpr double ANGLE_MANUAL_SCALE = 0.10;

static_assert(CHASSIS_MANUAL_SCALE > 0.0 && CHASSIS_MANUAL_SCALE <= 1.0, "Invalid chassis scale");
static_assert(ROTATE_MANUAL_SCALE > 0.0 && ROTATE_MANUAL_SCALE <= 1.0, "Invalid rotate scale");
static_assert(ANGLE_MANUAL_SCALE > 0.0 && ANGLE_MANUAL_SCALE <= 1.0, "Invalid angle scale");

}
