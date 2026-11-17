#pragma once

#include <Arduino.h>

#include "../Constants/DribblerConstants.h"

class Dribbler {
public:
    static void init();   // starts disabled: A15 HIGH, D22 LOW
    static void update();

    static void setEnabled(bool enabled);
    static bool isEnabled();

    static void stop();

    static bool requestFeed();

    static bool isPulseActive();
    static bool isBusy();
    static unsigned long getTriggerCount();

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
    static bool readyCounted;
    static unsigned long readyHighSinceMs;
};
