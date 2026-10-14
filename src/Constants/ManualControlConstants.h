#pragma once

#include <stddef.h>
#include <stdint.h>

#include "BuildProfile.h"

// Operator control (src/IO/ManualControl.cpp): wired USB Dashboard and the
// wireless ELRS path share these.
namespace ManualControlConstants {

// Wired: no command line for this long while the wired side owns the robot
// = disarm. Wireless: no *valid* RC frame for this long while the wireless
// side owns it = disarm (on top of MechLink's own RF-loss timeout).
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 24;

// Status line to the robot bridge for the wireless uplink.
constexpr unsigned long STATUS_PERIOD_MS = 100;

// Jeremy's manual test commands (normalized output, not measured RPM).
constexpr double CHASSIS_MANUAL_SCALE = 1.00;
constexpr double ROTATE_MANUAL_SCALE = 1.00;
constexpr double ANGLE_MANUAL_SCALE = 1.00;
constexpr bool WIRED_MANUAL_ONLY = BuildProfile::WIRED_MANUAL_ONLY;

static_assert(CHASSIS_MANUAL_SCALE > 0.0 && CHASSIS_MANUAL_SCALE <= 1.0, "Invalid chassis scale");
static_assert(ROTATE_MANUAL_SCALE > 0.0 && ROTATE_MANUAL_SCALE <= 1.0, "Invalid rotate scale");
static_assert(ANGLE_MANUAL_SCALE > 0.0 && ANGLE_MANUAL_SCALE <= 1.0, "Invalid angle scale");

}
