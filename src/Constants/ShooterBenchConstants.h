#pragma once

#include <Arduino.h>

#include "ShooterConstants.h"

namespace ShooterBenchConstants {

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr unsigned long TELEMETRY_PERIOD_MS = 100;
constexpr size_t COMMAND_BUFFER_SIZE = 24;

// Start with limited authority for the first unloaded mechanism tests.
constexpr double ANGLE_MANUAL_COMMAND = 0.10;
constexpr double ROTATE_MANUAL_COMMAND = 0.10;

constexpr long ANGLE_INITIAL_COUNTS =
    ShooterConstants::Angle::MIN_POSITION_COUNTS;
constexpr long ANGLE_SETPOINT_1_COUNTS = 512;
constexpr long ANGLE_MAXIMUM_COUNTS =
    ShooterConstants::Angle::MAX_POSITION_COUNTS;

static_assert(ANGLE_MANUAL_COMMAND > 0.0 && ANGLE_MANUAL_COMMAND <= 1.0,
              "Invalid angle bench command");
static_assert(ROTATE_MANUAL_COMMAND > 0.0 && ROTATE_MANUAL_COMMAND <= 1.0,
              "Invalid rotate bench command");
static_assert(ANGLE_SETPOINT_1_COUNTS >= ANGLE_INITIAL_COUNTS,
              "Angle setpoint is below the initial position");
static_assert(ANGLE_SETPOINT_1_COUNTS <= ANGLE_MAXIMUM_COUNTS,
              "Angle setpoint is above the maximum position");

}
