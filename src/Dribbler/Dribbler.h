#pragma once

#include <Arduino.h>

#include "../Constants/DribblerConstants.h"

// Feed tower lines (DribblerConstants.h): A15 HIGH while disabled; D22 LOW
// except a 100 ms HIGH pulse per requested ball while enabled, followed by
// a lockout for the tower's cycle; A14 pulses from the tower mark a loaded
// ball.
class Dribbler {
public:
    static void init();   // starts disabled: A15 HIGH, D22 LOW
    static void update();

    // Robot enabled (A15 LOW, pulses allowed) or disabled (A15 HIGH, no
    // pulses). Either way any pulse in progress is cut short; the lockout
    // is not shortened (the tower may still be mid-cycle). The ball latch
    // is kept: the ball is still in the tower.
    static void setEnabled(bool enabled);
    static bool isEnabled();

    // Cuts any pulse short (D22 LOW). Does not shorten the lockout.
    static void stop();

    // One ball. Returns false (and sends nothing) while disabled or while a
    // pulse or the lockout is still running. No queueing. Called directly
    // for F in full manual / test (no ball check), and by Auto when it
    // fires -- Auto checks towerAllowsShot() itself first.
    static bool requestFeed();

    static bool isPulseActive();
    static bool isBusy();
    static unsigned long getTriggerCount(); // shots sent since boot

    // A14: a pulse since the last shot ("one ball ready"), and balls that
    // reached the shooting position since boot (a pulse while one is
    // already latched is the same ball, not counted). towerAllowsShot() is what the autos (only) wait for: always true while
    // Tower::READY_REQUIRED is false.
    static bool hasBall();
    static unsigned long getLoadedCount();
    static bool towerAllowsShot();

private:
    static void writeTrigger(bool active);
    static void writeDisable(bool disabled);
    static void pollReady();

    static bool enabled;
    static bool pulseActive;
    static bool everTriggered;
    static unsigned long pulseStartMs;
    static unsigned long triggerCount;
    static bool ballReady;
    static unsigned long loadedCount;
    static bool readyHigh;        // A14 seen HIGH on the last poll
    static bool readyCounted;     // this HIGH already counted
    static unsigned long readyHighSinceMs;
};
