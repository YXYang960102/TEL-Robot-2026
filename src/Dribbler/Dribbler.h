#pragma once

#include <Arduino.h>

#include "../Constants/DribblerConstants.h"

// Feed tower trigger. See DribblerConstants.h: the tower feeds and refills
// itself; the Mega only sends a 100 ms HIGH pulse per requested ball and
// then refuses further triggers for the tower's cycle time.
class Dribbler {
public:
    static void init();
    static void update();

    // Drops the line LOW immediately (cutting any pulse short) and clears
    // Auto's queue. Does not shorten the lockout: the tower may still be
    // mid-cycle.
    static void stop();

    // One ball. Returns false (and sends nothing) while a pulse or the
    // lockout is still running -- no queueing.
    static bool requestFeed();

    static bool isPulseActive();
    static bool isBusy();
    static unsigned long getTriggerCount();

    // Auto's auto-fire sequence (src/Auto/Auto.cpp): queue N shots, and
    // allow them to go out one trigger at a time while feedAllowed is set.
    static void setShootRequest(int count);
    static int getShootRemaining();
    static void setFeedAllowed(bool allowed);

private:
    static void writeLine(bool active);

    static bool pulseActive;
    static bool everTriggered;
    static unsigned long pulseStartMs;
    static unsigned long triggerCount;
    static int shootRemaining;
    static bool feedAllowed;
};
