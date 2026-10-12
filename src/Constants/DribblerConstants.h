#pragma once

#include <stdint.h>

// Feed tower interface (2026-10-11, Jeremy): the tower has its own
// controller that drives its motors and refills by itself. The Mega only
// owns one trigger line: idle LOW, a 100 ms HIGH pulse asks the tower to
// feed one ball. The tower gives no feedback (as far as known), so the
// Mega can only ever say "trigger sent", never "one ball went out".
//
// Robot total: six controlled motors (two chassis, one horizontal conveyor,
// flywheel, elevation, turret). The tower adds only a trigger output.
namespace DribblerConstants {

// Single RC-pulse ESC signal, confirmed by Jeremy (2026-10-11).
// Provisional bidirectional calibration: 1500 us stop, 1750 us run (0.5).
namespace Conveyor {
constexpr uint8_t SIGNAL_PIN = 45;
constexpr int STOP_PULSE_US = 1500;
constexpr int RUN_PULSE_US = 1750;
constexpr bool CONFIGURED = SIGNAL_PIN != 255;
}

namespace Tower {
// PROVISIONAL pin -- the real pin table hasn't been confirmed yet.
// PIN_TABLE_CONFIRMED in Pins.h refuses Enable until it is.
constexpr uint8_t TRIGGER_PIN = 2;
constexpr bool TRIGGER_ACTIVE_HIGH = true;

constexpr unsigned long PULSE_MS = 100;

// Lockout after the start of a pulse, covering the tower's own feed +
// refill cycle. A trigger inside this window is refused (FEED_BUSY), never
// queued. PROVISIONAL starting value: the tower's real cycle time hasn't
// been measured -- tune on the robot.
constexpr unsigned long COOLDOWN_MS = 1000;
}

namespace Request {
// Only used by Auto's auto-fire sequence (src/Auto/Auto.cpp).
constexpr int MIN_COUNT = 0;
constexpr int MAX_COUNT = 12;
}

static_assert(Tower::COOLDOWN_MS >= Tower::PULSE_MS,
              "The lockout has to cover at least the pulse itself");

}

static_assert(DribblerConstants::Conveyor::SIGNAL_PIN != DribblerConstants::Tower::TRIGGER_PIN,
              "Conveyor and tower outputs must use distinct pins");
