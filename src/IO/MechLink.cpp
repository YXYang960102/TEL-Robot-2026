#include "MechLink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../Constants/Pins.h"

namespace {
// No valid line for longer than this counts as signal lost. The bridge
// forwards every 20 ms while RF is up and goes silent when it drops.
constexpr unsigned long LINE_TIMEOUT_MS = 200;
// Must match the bridge's USART1 (tools/stm32_elrs_robot_bridge/src/config.h).
constexpr unsigned long LINK_BAUD = 250000;
constexpr size_t LINE_MAX_CHARS = 80;
char rx[LINE_MAX_CHARS];
size_t rxLength = 0;
bool rxDiscarding = false;
unsigned long lastValidLineMs = 0;
bool everReceivedLine = false;
}  // namespace

int MechLink::channels[RCMAP_CHANNEL_COUNT];
unsigned long MechLink::frameCount = 0;
unsigned long MechLink::rejectedLines = 0;
bool MechLink::signalLost = true;
int MechLink::modeChannel = RCMAP_MODE_FULL_MANUAL_US;

void MechLink::init() {
    MECH_LINK_SERIAL_PORT.begin(LINK_BAUD);
    setSafe();
}

void MechLink::update() {
    while (MECH_LINK_SERIAL_PORT.available() > 0) {
        const char c = static_cast<char>(MECH_LINK_SERIAL_PORT.read());
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            if (!rxDiscarding) {
                rx[rxLength] = '\0';
                parseLine();
            } else {
                rejectedLines++;
            }
            rxLength = 0;
            rxDiscarding = false;
            continue;
        }
        if (rxDiscarding) {
            continue;
        }
        if (rxLength + 1 < LINE_MAX_CHARS) {
            rx[rxLength++] = c;
        } else {
            rxLength = 0;
            rxDiscarding = true; // oversize line: drop it through the next '\n'
        }
    }

    signalLost = !everReceivedLine || (millis() - lastValidLineMs) > LINE_TIMEOUT_MS;
    if (signalLost) {
        setSafe();
    }
}

void MechLink::parseLine() {
    if (rxLength == 0) {
        return;
    }
    // The check first: a line with a flipped digit must never be acted on.
    const int textLength = rcmap_line_open(rx, static_cast<uint16_t>(rxLength));
    if (textLength < 0 || strncmp(rx, "RC,", 3) != 0) {
        rejectedLines++;
        return;
    }
    rx[textLength] = '\0';

    int values[RCMAP_CHANNEL_COUNT];
    char* cursor = rx + 3;
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        char* end = nullptr;
        const long v = strtol(cursor, &end, 10);
        const bool last = i + 1 == RCMAP_CHANNEL_COUNT;
        if (end == cursor || v < 0 || v > 3000 || *end != (last ? '\0' : ',')) {
            rejectedLines++;
            return; // malformed: drop the whole line, keep last-good values
        }
        values[i] = static_cast<int>(v);
        cursor = end + 1;
    }

    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        channels[i] = values[i];
    }
    modeChannel = values[RCMAP_CH_MODE];
    frameCount++;
    lastValidLineMs = millis();
    everReceivedLine = true;
}

unsigned long MechLink::lastLineAgeMs() {
    // Never received anything: millis() itself is already a large, still-
    // growing number, same "clearly stale" meaning as a real old timestamp.
    return everReceivedLine ? millis() - lastValidLineMs : millis();
}

void MechLink::setSafe() {
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        channels[i] = 1500;
    }
    // Released / none / manual: what the ground sends when idle.
    channels[RCMAP_CH_TARGET] = 1000;
    channels[RCMAP_CH_MODE] = RCMAP_MODE_FULL_MANUAL_US;
    channels[RCMAP_CH_FIRE] = 1000;
    channels[RCMAP_CH_ARM] = 1000;
    channels[RCMAP_CH_NUDGE] = 1000;
    channels[RCMAP_CH_AUTO] = 1000;
    modeChannel = RCMAP_MODE_FULL_MANUAL_US;
}

void MechLink::sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode) {
    // "ST,255,16777215,65535" is 21 chars; + "*HH" + "\r\n" + '\0'.
    char line[32];
    const int n = snprintf(line, sizeof(line), "ST,%u,%lu,%u", static_cast<unsigned>(flags),
                           eventSeq & 0xFFFFFFUL, static_cast<unsigned>(eventCode));
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(line)) {
        return;
    }
    const uint16_t length = rcmap_line_seal(line, static_cast<uint16_t>(n), sizeof(line) - 1);
    if (length == 0) {
        return;
    }
    line[length] = '\0';
    MECH_LINK_SERIAL_PORT.println(line);
}
