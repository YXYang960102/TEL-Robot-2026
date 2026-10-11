#pragma once

#include <stddef.h>
#include <stdint.h>

// Operator control (src/IO/ManualControl.cpp): wired USB Dashboard and the
// wireless ELRS path share these. Timeout/buffer values match the existing
// bench convention (ShooterBenchConstants).
namespace ManualControlConstants {

// Wired: no command line for this long while the wired side owns the robot
// = disarm. Wireless: no *valid* RC frame for this long while the wireless
// side owns it = disarm (on top of MechLink's own RF-loss timeout).
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 24;

// Status line to the robot bridge for the wireless uplink.
constexpr unsigned long STATUS_PERIOD_MS = 100;

// Keyboard keys are on/off (-1/0/1); these scale that into the actual
// open-loop command for first-power-on bring-up. Rotate/Angle match
// ShooterBenchConstants' 0.10 "limited authority for the first unloaded
// mechanism tests"; chassis has no prior bench value, 0.30 is a
// conservative starting point. Raise only after real-hardware checks.
// Applied identically to wired and wireless commands.
constexpr double CHASSIS_MANUAL_SCALE = 0.30;
constexpr double ROTATE_MANUAL_SCALE = 0.10;
constexpr double ANGLE_MANUAL_SCALE = 0.10;

static_assert(CHASSIS_MANUAL_SCALE > 0.0 && CHASSIS_MANUAL_SCALE <= 1.0, "Invalid chassis scale");
static_assert(ROTATE_MANUAL_SCALE > 0.0 && ROTATE_MANUAL_SCALE <= 1.0, "Invalid rotate scale");
static_assert(ANGLE_MANUAL_SCALE > 0.0 && ANGLE_MANUAL_SCALE <= 1.0, "Invalid angle scale");

}
