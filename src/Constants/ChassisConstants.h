#pragma once

#include <stdint.h>

#include "TuningConstants.h"

namespace ChassisConstants {

namespace RightDrive {
using namespace TuningConstants::RightDrive;
constexpr uint8_t SIGNAL_PIN = 39;
}

namespace LeftDrive {
using namespace TuningConstants::LeftDrive;
constexpr uint8_t SIGNAL_PIN = 38;
}

namespace Output {
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr int RANGE_US = 500;
}

namespace Manual {
using namespace TuningConstants::Manual;
}

static_assert(Output::MIN_PULSE_US < Output::NEUTRAL_US, "Invalid chassis minimum PWM");
static_assert(Output::NEUTRAL_US < Output::MAX_PULSE_US, "Invalid chassis maximum PWM");

}
