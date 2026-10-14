#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Constants/DribblerConstants.h"

// Feed tower trigger. See DribblerConstants.h: the tower feeds and refills
// itself; the Mega only sends a 100 ms HIGH pulse per requested ball and
// then refuses further triggers for the tower's cycle time.
class Dribbler {
public:
    static void setConveyor(bool on);
    static bool isConveyorOn();
    static void init();
    static void update();

    // Drops the line LOW immediately (cutting any pulse short) and stops
    // the conveyor. Does not shorten the lockout: the tower may still be
    // mid-cycle.
    static void stop();

    // One ball. Returns false (and sends nothing) while a pulse or the
    // lockout is still running -- no queueing. Called directly for F in
    // full-manual, and by Auto when it fires.
    static bool requestFeed();

    static bool isPulseActive();
    static bool isBusy();
    static unsigned long getTriggerCount();

private:
    static void writeLine(bool active);

    static Servo conveyorEsc;
    static bool conveyorOn;
    static bool pulseActive;
    static bool everTriggered;
    static unsigned long pulseStartMs;
    static unsigned long triggerCount;
};
