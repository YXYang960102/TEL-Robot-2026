#pragma once

#include <stdint.h>

namespace ChassisConstants {

namespace RightDrive {
constexpr uint8_t SIGNAL_PIN = 39;
constexpr bool INVERTED = false;
}

namespace LeftDrive {
constexpr uint8_t SIGNAL_PIN = 38;
constexpr bool INVERTED = false;
}

namespace Output {
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr int RANGE_US = 500;
}

namespace Manual {
constexpr double COMMAND_DEADBAND = 0.04;
constexpr double MIN_COMMAND = -1.0;
constexpr double MAX_COMMAND = 1.0;
}

static_assert(Output::MIN_PULSE_US < Output::NEUTRAL_US, "Invalid chassis minimum PWM");
static_assert(Output::NEUTRAL_US < Output::MAX_PULSE_US, "Invalid chassis maximum PWM");

}
