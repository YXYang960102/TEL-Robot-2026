#pragma once

#include <stdint.h>

// Feed tower interface (2026-10-11, Jeremy): the tower has its own
// controller that drives its motors and refills by itself. The Mega only
// owns one trigger line: idle LOW, a 100 ms HIGH pulse asks the tower to
// feed one ball. The tower gives no feedback (as far as known), so the
// Mega can only ever say "trigger sent", never "one ball went out".
//
// The horizontal conveyor is driven by the tower side too (2026-10-16), so
// the Mega no longer has a conveyor output (the old D45 ESC was removed).
namespace DribblerConstants {

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

static_assert(Tower::COOLDOWN_MS >= Tower::PULSE_MS,
              "The lockout has to cover at least the pulse itself");

}
