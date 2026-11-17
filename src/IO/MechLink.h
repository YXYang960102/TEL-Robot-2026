#pragma once

#include <Arduino.h>

#include "RcControlMap.h"

class MechLink {
public:
    static void init();
    static void update();

    static int channels[RCMAP_CHANNEL_COUNT];

    static unsigned long frameCount;

    static unsigned long rejectedLines;

    static bool signalLost;

    static int modeChannel;

    static unsigned long lastLineAgeMs();

    static void sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode);

private:
    static void parseLine();
    static void setSafe();
};
