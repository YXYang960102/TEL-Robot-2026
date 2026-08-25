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
constexpr double ROTATE_POSITION_TEST_MAX_COMMAND = 0.10;

constexpr long ANGLE_INITIAL_COUNTS =
    ShooterConstants::Angle::MIN_POSITION_COUNTS;
constexpr long ANGLE_SETPOINT_1_COUNTS = 0;
constexpr long ANGLE_MAXIMUM_COUNTS =
    ShooterConstants::Angle::MAX_POSITION_COUNTS;

constexpr int ROTATE_LEFT_LIMIT_RAW =
    ShooterConstants::Rotate::LEFT_POSITION_RAW;
constexpr int ROTATE_LEFT_SETPOINT_RAW =
    (ShooterConstants::Rotate::LEFT_POSITION_RAW +
     ShooterConstants::Rotate::CENTER_POSITION_RAW) / 2;
constexpr int ROTATE_CENTER_RAW =
    ShooterConstants::Rotate::CENTER_POSITION_RAW;
constexpr int ROTATE_RIGHT_SETPOINT_RAW =
    (ShooterConstants::Rotate::CENTER_POSITION_RAW +
     ShooterConstants::Rotate::RIGHT_POSITION_RAW) / 2;
constexpr int ROTATE_RIGHT_LIMIT_RAW =
    ShooterConstants::Rotate::RIGHT_POSITION_RAW;

static_assert(ANGLE_MANUAL_COMMAND > 0.0 && ANGLE_MANUAL_COMMAND <= 1.0,
              "Invalid angle bench command");
static_assert(ROTATE_MANUAL_COMMAND > 0.0 && ROTATE_MANUAL_COMMAND <= 1.0,
              "Invalid rotate bench command");
static_assert(ROTATE_POSITION_TEST_MAX_COMMAND > 0.0 &&
                  ROTATE_POSITION_TEST_MAX_COMMAND <= 1.0,
              "Invalid rotate position test command");
static_assert(!ShooterConstants::Angle::POSITION_LIMITS_CALIBRATED ||
                  ANGLE_SETPOINT_1_COUNTS >= ANGLE_INITIAL_COUNTS,
              "Angle setpoint is below the initial position");
static_assert(!ShooterConstants::Angle::POSITION_LIMITS_CALIBRATED ||
                  ANGLE_SETPOINT_1_COUNTS <= ANGLE_MAXIMUM_COUNTS,
              "Angle setpoint is above the maximum position");
static_assert(!ShooterConstants::Rotate::POSITION_CALIBRATED ||
                  (ROTATE_LEFT_LIMIT_RAW < ROTATE_LEFT_SETPOINT_RAW &&
                   ROTATE_LEFT_SETPOINT_RAW < ROTATE_CENTER_RAW),
              "Invalid left rotate test positions");
static_assert(!ShooterConstants::Rotate::POSITION_CALIBRATED ||
                  (ROTATE_CENTER_RAW < ROTATE_RIGHT_SETPOINT_RAW &&
                   ROTATE_RIGHT_SETPOINT_RAW < ROTATE_RIGHT_LIMIT_RAW),
              "Invalid right rotate test positions");

}
