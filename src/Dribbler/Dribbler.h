#pragma once

#include <Arduino.h>

#include "../Constants/DribblerConstants.h"

// Feed tower trigger. See DribblerConstants.h: the tower feeds and refills
// itself; while the robot is enabled the Mega sends a 100 ms HIGH pulse per
// requested ball (line LOW otherwise) and then refuses further triggers for
// the tower's cycle time; while it is disabled the line is held HIGH.
class Dribbler {
public:
    static void init();   // starts disabled: line HIGH
    static void update();

    // Robot enabled (line idles LOW, pulses allowed) or disabled (line held
    // HIGH, no pulses). Either way any pulse in progress is cut short; the
    // lockout is not shortened (the tower may still be mid-cycle).
    static void setEnabled(bool enabled);
    static bool isEnabled();

    // Cuts any pulse short: back to the idle level (LOW enabled, HIGH
    // disabled). Does not shorten the lockout.
    static void stop();

    // One ball. Returns false (and sends nothing) while disabled, or while
    // a pulse or the lockout is still running -- no queueing. Called directly for F in
    // full-manual, and by Auto when it fires.
    static bool requestFeed();

    static bool isPulseActive();
    static bool isBusy();
    static unsigned long getTriggerCount();

private:
    static void writeLine(bool active);

    static bool enabled;
    static bool pulseActive;
    static bool everTriggered;
    static unsigned long pulseStartMs;
    static unsigned long triggerCount;
};
