#pragma once

#include <stdint.h>

// Creative design (Jeremy, 2026-11-07), same show as last year: every shot
// the "動感光波" link motor turns and the 動感光波 sound plays; the first
// shot at a hole different from the previous shot's adds "哇哈哈哈哈".
namespace ArtConstants {

// Link motor: Fire Phoenix (火鳳凰) brushed ESC on D47, driven like the
// chassis ESCs (1000-1500-2000 us, 1500 = stop). One direction for
// LINK_RUN_MS per shot, then stop. PROVISIONAL: LINK_COMMAND and
// LINK_INVERTED are guesses -- check the direction and speed on the robot.
namespace Link {
constexpr uint8_t SIGNAL_PIN = 47;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr double COMMAND = 0.5;       // 0..1 of full speed -> 1750 us
constexpr bool INVERTED = false;      // true -> 1250 us
constexpr unsigned long RUN_MS = 1500;
}

// Sound module triggers: idle HIGH, pulled LOW for PULSE_MS to play one
// clip (Jeremy: "下拉電位").
namespace Sound {
constexpr uint8_t WAVE_PIN = 40;   // 動感光波, every shot
constexpr uint8_t LAUGH_PIN = 41;  // 哇哈哈哈哈, first shot at a new hole
constexpr uint8_t SPARE_PIN = 42;  // third clip, reserved: held HIGH, never pulled
constexpr unsigned long PULSE_MS = 200;
// The laugh waits for the wave clip to finish, so the two never overlap.
// PROVISIONAL: set it to the 動感光波 clip's length.
constexpr unsigned long LAUGH_DELAY_MS = 1500;
}

static_assert(Link::COMMAND >= 0.0 && Link::COMMAND <= 1.0, "Link command is a 0..1 ratio");
static_assert(Link::MIN_PULSE_US < Link::NEUTRAL_US && Link::NEUTRAL_US < Link::MAX_PULSE_US,
              "Invalid link motor PWM");

}
