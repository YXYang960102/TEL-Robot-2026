#pragma once

#include <stddef.h>

namespace ChassisBenchConstants {

constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr unsigned long TELEMETRY_PERIOD_MS = 100;
constexpr size_t COMMAND_BUFFER_SIZE = 24;
constexpr double MANUAL_COMMAND = 0.50;
constexpr int DEFAULT_DRIVE_MODE = 2;

static_assert(MANUAL_COMMAND > 0.0 && MANUAL_COMMAND <= 1.0,
              "Invalid chassis bench command");
static_assert(DEFAULT_DRIVE_MODE == 1 || DEFAULT_DRIVE_MODE == 2,
              "Invalid default drive mode");

}
