#pragma once

#include <Arduino.h>

#include "RcControlMap.h"

// Wireless link to the robot-side STM32 bridge (tools/stm32_elrs_robot_bridge)
// on Pins.h's MECH_LINK_SERIAL_PORT.
//
// Downlink: the bridge forwards the decoded CRSF channels as
//   "RC,c1,c2,...,c10*HH\n"   (pulse microseconds, table in RcControlMap.h)
// and goes silent when the ELRS link drops, so "no valid line for
// LINE_TIMEOUT_MS" is the RF-loss signal (signalLost).
//
// Uplink: sendStatus() writes "ST,flags,seq,code*HH\n" back to the bridge,
// which packs it into a CRSF Battery frame for the ground station.
//
// "*HH" (2026-10-27) is RcControlMap's line check: a line whose check
// doesn't match is dropped and counted (rejectedLines), never acted on.
// The link runs at LINK_BAUD = 250000, which both the Mega (U2X, UBRR 7)
// and the bridge (BRR 0x40) hit exactly; at 115200 the Mega was +2.1% off.
class MechLink {
public:
    static void init();
    static void update();

    // Latest valid channel values (us), neutral/safe while signalLost.
    static int channels[RCMAP_CHANNEL_COUNT];

    // Increments once per valid RC line, so a consumer can process each
    // received frame exactly once.
    static unsigned long frameCount;

    // Lines dropped since boot: bad check, malformed or oversize (TEL slot
    // 39). A healthy link stays at 0 or grows very slowly.
    static unsigned long rejectedLines;

    // True until the first valid line ever arrives, and again whenever one
    // hasn't arrived for LINE_TIMEOUT_MS.
    static bool signalLost;

    // Raw CH4 (operator mode, see RcControlMap.h), for telemetry. FULL_MANUAL
    // while signalLost. ManualControl decodes the mode from the frame itself.
    static int modeChannel;

    // Milliseconds since the last valid "RC,..." line, for telemetry (e.g.
    // checking the real ELRS/bridge update rate against LINE_TIMEOUT_MS
    // with the chassis output-disabled or the drive motors unpowered,
    // rather than guessing). Keeps growing (never received = a large,
    // still-growing number) rather than wrapping to something small.
    static unsigned long lastLineAgeMs();

    static void sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode);

private:
    static void parseLine();
    static void setSafe();
};
