#pragma once
#include <stdint.h>

namespace VisionRotateBenchConstants {
constexpr uint8_t SIGNAL_PIN = 59;
constexpr int NEUTRAL_US = 1500;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250; // USB (browser tab) watchdog
constexpr uint32_t TELEMETRY_MS = 100;
constexpr unsigned long BAUD = 115200; // USB Serial, for the browser page

constexpr bool INVERTED = true; // tx > 0 turned the servo away

constexpr int PROTOCOL_VERSION = 2; // must match the Orin
constexpr unsigned long VISION_BAUD = 115200; // Serial1, must match Orin SERIAL_BAUD

constexpr double kP = 5.0; // us per px
constexpr double kI = 5.0; // inside kIZone only
constexpr double kD = 0.0; // applied externally to filtered rate, not via PidfController
constexpr double kIZone = 20.0;
constexpr double kFF = 0.0;

constexpr double kD_FILTER_TAU_S = 0.1; // D-term rate low-pass

constexpr int PIDF_OUTPUT_LIMIT_US = 500; // servo authority

static_assert(PIDF_OUTPUT_LIMIT_US > 0 && PIDF_OUTPUT_LIMIT_US <= 500,
              "Bench authority exceeded");

constexpr int PIDF_INTEGRAL_LIMIT_US = 60; // I term cap
static_assert(PIDF_INTEGRAL_LIMIT_US > 0 && PIDF_INTEGRAL_LIMIT_US <= PIDF_OUTPUT_LIMIT_US,
              "Integral limit must fit inside the output limit");

constexpr uint32_t VISION_TIMEOUT_MS = 500; // Serial1 (Orin) link watchdog
constexpr uint32_t MEGA_HEARTBEAT_INTERVAL_MS = 200;
}
