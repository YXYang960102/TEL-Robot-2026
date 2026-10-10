#pragma once

#include <stdint.h>

// Ported from the Dribbler branch during the 2026-10-11 dev integration
// (docs/codex-handoff.md), with two additions: a multi-channel Feeder
// output (see below) and a jam timeout (Jam::TIMEOUT_MS).
namespace DribblerConstants {

namespace Feeder {
enum class Action : uint8_t {
    STOP = 0,
    FEED = 1
};

// The real feed mechanism is 6 motors (2 horizontal collector rollers +
// 4 vertical pinch-feed rollers that squeeze the ball up into the
// flywheel), confirmed from the bench test video + Jeremy 2026-10-11 --
// not the single motor the original Dribbler-branch bench assumed. R and
// F both command this entire group together (Jeremy confirmed there is no
// need to drive the collector and the vertical feed separately), so one
// logical command still fans out to every channel here.
//
// Final motor-driver pin assignments are NOT YET PROVIDED (Jeremy is
// sending the real pin table separately) -- this currently only wires the
// one channel the single-motor bench rig already uses, so wired testing
// has something real to drive today. Add entries to PWM_PINS/ENABLE_PINS
// once the real pin table arrives; Dribbler::writeFeedOutput() already
// loops over every channel, so no other code needs to change.
constexpr uint8_t CHANNEL_COUNT = 1;
constexpr uint8_t PWM_PINS[CHANNEL_COUNT] = {2};
constexpr uint8_t ENABLE_PINS[CHANNEL_COUNT] = {51};
constexpr bool ENABLE_ACTIVE_HIGH = true;
constexpr int MIN_PWM = 0;
constexpr int MAX_PWM = 255;
constexpr int DEFAULT_FEED_PWM = 180;
constexpr double DEFAULT_FEED_COMMAND =
    DEFAULT_FEED_PWM / static_cast<double>(MAX_PWM);
}

namespace ExitSensor {
constexpr uint8_t SIGNAL_PIN = 45;
constexpr bool BLOCKED_LOW = true;
constexpr unsigned long DEBOUNCE_MS = 8;
}

namespace Request {
constexpr int MIN_COUNT = 0;
constexpr int MAX_COUNT = 12;
}

namespace Jam {
// If a COUNTED_FEED request (F) has been actively running the feed motors
// for this long without the ExitSensor reporting a full blocked->cleared
// edge (one ball fully passing), declare a jam: stop, clear the request,
// and report $EVENT,FEED_JAM. Starting value, Jeremy confirmed 2026-10-11
// ("3秒(建議起始值)") -- tune on the bench with real balls.
constexpr unsigned long TIMEOUT_MS = 3000;
}

static_assert(
    Feeder::DEFAULT_FEED_PWM >= Feeder::MIN_PWM &&
        Feeder::DEFAULT_FEED_PWM <= Feeder::MAX_PWM,
    "Invalid dribbler feed PWM");

}
