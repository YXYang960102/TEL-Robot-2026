#pragma once

#include <stddef.h>
#include <stdint.h>

#include "BuildProfile.h"

namespace ManualControlConstants {

constexpr unsigned long COMMAND_TIMEOUT_MS = 250; // no command this long = disarm
constexpr size_t COMMAND_BUFFER_SIZE = 24;

constexpr unsigned long STATUS_PERIOD_MS = 100; // status uplink period

constexpr double CHASSIS_MANUAL_SCALE = 1.00; // normalized output
constexpr double ROTATE_MANUAL_SCALE = 1.00;
constexpr double ANGLE_MANUAL_SCALE = 1.00;
constexpr bool WIRED_MANUAL_ONLY = BuildProfile::WIRED_MANUAL_ONLY;

static_assert(CHASSIS_MANUAL_SCALE > 0.0 && CHASSIS_MANUAL_SCALE <= 1.0, "Invalid chassis scale");
static_assert(ROTATE_MANUAL_SCALE > 0.0 && ROTATE_MANUAL_SCALE <= 1.0, "Invalid rotate scale");
static_assert(ANGLE_MANUAL_SCALE > 0.0 && ANGLE_MANUAL_SCALE <= 1.0, "Invalid angle scale");

}
