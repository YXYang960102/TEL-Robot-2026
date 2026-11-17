#pragma once

#include <stdint.h>

namespace DribblerConstants {

namespace Tower {
constexpr uint8_t TRIGGER_PIN = 22; // D22
constexpr bool TRIGGER_ACTIVE_HIGH = true;

constexpr uint8_t DISABLE_PIN = 69; // A15

constexpr unsigned long PULSE_MS = 100; // fire trigger pulse

constexpr unsigned long COOLDOWN_MS = 1000; // tower feed + refill lockout

constexpr uint8_t READY_PIN = 68; // A14
constexpr unsigned long READY_MIN_HIGH_MS = 20; // A14 HIGH this long = a ball
constexpr bool READY_REQUIRED = true; // autos wait for the A14 ball
}

static_assert(Tower::COOLDOWN_MS >= Tower::PULSE_MS,
              "The lockout has to cover at least the pulse itself");
static_assert(Tower::READY_MIN_HIGH_MS < 100,
              "The Nano's ready pulse is 100 ms; the debounce must be shorter");

}
