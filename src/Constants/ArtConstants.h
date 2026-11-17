#pragma once

#include <stdint.h>

namespace ArtConstants {

namespace Link {
constexpr uint8_t SIGNAL_PIN = 47;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr double COMMAND = 0.4; // 0..1 of full speed: 1300 us (inverted)
constexpr bool INVERTED = true;
constexpr unsigned long RUN_MS = 2000;
}

namespace Sound {
constexpr bool ACTIVE_HIGH = true;
constexpr uint8_t WAVE_PIN = 41;
constexpr uint8_t LAUGH_PIN = 42;
constexpr unsigned long WAVE_CLIP_MS = 3000; // 動感光波 clip length; a laugh waits this long
constexpr uint8_t READY_PIN = 40;
constexpr uint8_t START_PIN = 43; // 春日部防衛隊出動, on Y
constexpr unsigned long PULSE_MS = 1000;
constexpr unsigned long READY_PERIOD_MS = 10000; // 準備好了 repeat period while disabled
constexpr uint8_t READY_NORMAL_PLAYS = 3; // normal plays before the stutter
constexpr uint8_t READY_STUTTER_CUTS = 4; // retriggers per stutter
constexpr unsigned long READY_STUTTER_HIT_MS = 150;
constexpr unsigned long REARM_MS = 50; // idle before a retrigger
}

static_assert(Link::COMMAND >= 0.0 && Link::COMMAND <= 1.0, "Link command is a 0..1 ratio");
static_assert(Link::MIN_PULSE_US < Link::NEUTRAL_US && Link::NEUTRAL_US < Link::MAX_PULSE_US,
              "Invalid link motor PWM");

}
