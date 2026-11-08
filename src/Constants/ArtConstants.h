#pragma once

#include <stdint.h>

// Creative design (Jeremy, 2026-11-07), same show as last year: every shot
// (any mode: each fire pulse the Mega sends) the "動感光波" link motor turns
// and the 動感光波 sound plays at the same time; while enabled, the turret
// moving plays "哇哈哈哈哈"; while disabled, "準備好了" repeats.
namespace ArtConstants {

// Link motor: Fire Phoenix (火鳳凰) brushed ESC on D47, driven like the
// chassis ESCs (1000-1500-2000 us, 1500 = stop). One direction for
// RUN_MS per shot (2 s, Jeremy; was 1.5 s), then stop. PROVISIONAL: LINK_COMMAND and
// LINK_INVERTED are guesses -- check the direction and speed on the robot.
namespace Link {
constexpr uint8_t SIGNAL_PIN = 47;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
// 0..1 of full speed: 1500 - 0.4 * 500 = 1300 us inverted (Jeremy,
// 2026-11-08; 0.1 = 1450 us since 2026-11-07, before that 0.5 = 1250 us).
constexpr double COMMAND = 0.4;
// true since 2026-11-07 (Jeremy, on the robot: 1750 us turned it the wrong
// way).
constexpr bool INVERTED = true;
constexpr unsigned long RUN_MS = 2000;
}

// Sound module triggers: idle LOW, HIGH for PULSE_MS to play one clip
// (Jeremy, 2026-11-07; until then idle HIGH, pulled LOW for 200 ms).
namespace Sound {
constexpr bool ACTIVE_HIGH = true;
// Jeremy, 2026-11-08, from the robot: D40's clip is "準備好了" (now played
// while disabled), D42's is "哇哈哈哈哈", so 動感光波 is on D41.
constexpr uint8_t WAVE_PIN = 41;   // 動感光波, every shot
// Laugh: once each time the aimed hole changes, while enabled (Jeremy,
// 2026-11-08). Briefly: on any turret movement >= 0.3 s, which in semi auto
// played it over and over while locked and cut 動感光波 off.
constexpr uint8_t LAUGH_PIN = 42;  // 哇哈哈哈哈, aimed hole changed
// The module plays one clip at a time and a new trigger cuts the one
// playing, so a laugh waits this long after a 動感光波 starts. PROVISIONAL:
// set it to the 動感光波 clip's length.
constexpr unsigned long WAVE_CLIP_MS = 3000;
constexpr uint8_t READY_PIN = 40;  // 準備好了, repeats while disabled
constexpr unsigned long PULSE_MS = 1000;
// While disabled: 準備好了 at once, then every READY_PERIOD_MS (start to
// start; Jeremy, 2026-11-08, was 1 s LOW between plays). Stops at once on
// Enable.
constexpr unsigned long READY_PERIOD_MS = 10000;
// Jeremy, 2026-11-08: the first READY_NORMAL_PLAYS plays of each disabled
// period are normal; every later one stutters, 準準準準準備好了: a new
// trigger makes the module drop the clip and start it again, so
// READY_STUTTER_CUTS short triggers (HIGH READY_STUTTER_HIT_MS, then LOW
// REARM_MS) and one full one. PROVISIONAL: set READY_STUTTER_HIT_MS to how
// long "準" lasts on the module.
constexpr uint8_t READY_NORMAL_PLAYS = 3;
constexpr uint8_t READY_STUTTER_CUTS = 4;
constexpr unsigned long READY_STUTTER_HIT_MS = 150;
// A trigger while the line is still active drops it to idle this long
// first, so the module sees a fresh edge (shots can come 1 s apart, the
// tower lockout).
constexpr unsigned long REARM_MS = 50;
}

static_assert(Link::COMMAND >= 0.0 && Link::COMMAND <= 1.0, "Link command is a 0..1 ratio");
static_assert(Link::MIN_PULSE_US < Link::NEUTRAL_US && Link::NEUTRAL_US < Link::MAX_PULSE_US,
              "Invalid link motor PWM");

}
