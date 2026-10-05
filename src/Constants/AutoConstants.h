#pragma once
#include <stdint.h>

// Auto-fire sequence (src/Auto/Auto.h/.cpp). Every value here is a
// provisional placeholder pending real hardware/keypad: none of it has been
// measured or tested on real hardware yet.
namespace AutoConstants {

// How long Dribbler::setFeedAllowed(true) stays on to push one ball into the
// spinning flywheel. No working ball sensor exists to confirm this directly
// (see Dribbler.cpp) -- this is a time-based approximation, same spirit as
// Flywheel::SPIN_UP_MS.
constexpr uint32_t FEED_DURATION_MS = 500;

// Pause after a completed feed before the sequence can start again, so one
// queued shot can't immediately re-trigger itself.
constexpr uint32_t COOLDOWN_MS = 300;

// SBUS::fireChannel threshold for the semi-auto fire-confirm button
// (momentary, edge-triggered in Auto::update()). fireChannel provisionally
// aliases SBUS ch2 (see SBUS.h) -- re-measure once the real keypad exists.
constexpr int FIRE_BUTTON_THRESHOLD = 1500;

}
