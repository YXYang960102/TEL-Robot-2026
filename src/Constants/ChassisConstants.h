#pragma once

#include <stdint.h>

#include "TuningConstants.h"

// Ported from the Chassis branch during the 2026-10-11 dev integration
// (docs/codex-handoff.md). Two active drive wheels + passive casters, not
// four mecanum wheels -- replaces the old Pins.h PIN_FR/FL/BR/BL scheme.
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
