#pragma once

#include <Arduino.h>

#include "RcControlMap.h"

// Wireless link to the robot-side STM32 bridge (tools/stm32_elrs_robot_bridge)
// on Pins.h's MECH_LINK_SERIAL_PORT.
//
// Downlink: the bridge forwards the decoded CRSF channels as
//   "RC,c1,c2,...,c9\n"   (pulse microseconds, table in RcControlMap.h)
// and goes silent when the ELRS link drops, so "no valid line for
// LINE_TIMEOUT_MS" is the RF-loss signal (signalLost).
//
// Uplink: sendStatus() writes "ST,flags,seq,code\n" back to the bridge,
// which packs it into a CRSF Battery frame for the ground station.
class MechLink {
public:
    static void init();
    static void update();

    // Latest valid channel values (us), neutral/safe while signalLost.
    static int channels[RCMAP_CHANNEL_COUNT];

    // Increments once per valid RC line, so a consumer can process each
    // received frame exactly once.
    static unsigned long frameCount;

    // True until the first valid line ever arrives, and again whenever one
    // hasn't arrived for LINE_TIMEOUT_MS.
    static bool signalLost;

    // OperatorMode / Auto inputs. modeChannel is CH4 (the ground sends
    // FULL_MANUAL for now); there is no wireless fire-confirm channel, so
    // fireChannel stays released.
    static int modeChannel;
    static int fireChannel;

    static void sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode);

private:
    static void parseLine();
    static void setSafe();
};
