#pragma once

#include <stdint.h>

namespace IOConstants {

namespace Sbus {
constexpr uint8_t DRIVE_FORWARD_CHANNEL = 0;
constexpr uint8_t DRIVE_TURN_CHANNEL = 1;
constexpr uint8_t MECHANISM_CHANNEL = 2;
constexpr uint8_t AUXILIARY_CHANNEL = 3;
constexpr uint8_t MODE_CHANNEL = 8;

constexpr int RAW_MIN = 170;
constexpr int RAW_MAX = 1820;
constexpr int PULSE_MIN_US = 1000;
constexpr int PULSE_NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US = 2000;
constexpr int AUXILIARY_MIN_US = 1200;
constexpr int AUXILIARY_MAX_US = 1800;
constexpr unsigned long FRAME_TIMEOUT_MS = 100;
}

}
