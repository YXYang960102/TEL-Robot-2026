#pragma once

#include <stdint.h>

// Feed tower interface: the tower has its own controller (a Nano) that
// drives its motors and refills by itself. Three lines (Jeremy, 2026-11-07;
// until then D22 alone carried both disable and fire), all sharing the
// Nano's ground:
//   A15 disable  Mega -> Nano  HIGH while DISABLED (boot before the first
//                              Enable, Disable / Enter / Space / X, RC
//                              loss, any watchdog), LOW while enabled.
//   D22 fire     Mega -> Nano  LOW, except a 100 ms HIGH pulse per
//                              requested ball while enabled.
//   A14 ready    Nano -> Mega  a 100 ms HIGH pulse each time the tower has
//                              a ball loaded and ready to shoot.
// The Mega latches a A14 pulse as "one ball ready" and clears it with the
// next shot, so it can also count balls loaded and shots sent. That is
// still only what the Nano reports, not a sensor on the ball leaving.
//
// The horizontal conveyor is driven by the tower side too (2026-10-16), so
// the Mega no longer has a conveyor output (the old D45 ESC was removed).
namespace DribblerConstants {

namespace Tower {
// D22 (Jeremy, 2026-10-17; was D21). D21 is the Mega's I2C SCL with an
// on-board pull-up, so whenever the pin wasn't driven -- reset, upload,
// and every bench build that doesn't drive the tower -- it sat HIGH and
// the tower could fire on its own. D22 has no pull-up; it floats until
// Dribbler::init() drives it LOW (a 10k pull-down to GND keeps it LOW then).
constexpr uint8_t TRIGGER_PIN = 22;
constexpr bool TRIGGER_ACTIVE_HIGH = true;

// A15 (= digital 69; Jeremy 2026-11-07, the wire is on A15 -- the code
// said D15 for a few hours). Wire a 10k pull-up to 5V so the Nano reads
// "disabled" while the Mega resets or is being flashed.
constexpr uint8_t DISABLE_PIN = 69; // A15

constexpr unsigned long PULSE_MS = 100;

// Lockout after the start of a pulse, covering the tower's own feed +
// refill cycle. A trigger inside this window is refused (FEED_BUSY), never
// queued. PROVISIONAL starting value: the tower's real cycle time hasn't
// been measured -- tune on the robot.
constexpr unsigned long COOLDOWN_MS = 1000;

// A14 (= digital 68; Jeremy 2026-11-07, the wire is on A14 -- the code
// said A1, then D14, earlier that day). Wire a 10k pull-down to GND, so a
// loose wire never reads as a pulse. A pulse counts once the line has been HIGH for READY_MIN_HIGH_MS
// (the Nano's pulse is 100 ms; shorter blips are noise). After boot there
// is no ball until the first pulse, so the Nano has to send one once it is
// enabled (A15 LOW) with a ball loaded.
constexpr uint8_t READY_PIN = 68; // A14
constexpr unsigned long READY_MIN_HIGH_MS = 20;
// true = every mode waits for a A14 pulse before each shot: the autos treat
// "no ball" as not ready (semi-auto F answers FIRE_NOT_READY), full manual
// and test F answer FEED_BUSY. false = the pulses are only counted and
// shown on the Dashboard; firing is as before. Set true once the Dashboard
// shows one pulse per ball.
constexpr bool READY_REQUIRED = false;
}

static_assert(Tower::COOLDOWN_MS >= Tower::PULSE_MS,
              "The lockout has to cover at least the pulse itself");
static_assert(Tower::READY_MIN_HIGH_MS < 100,
              "The Nano's ready pulse is 100 ms; the debounce must be shorter");

}
