#include "MechLink.h"

#include <Arduino.h>
#include <stdlib.h>

#include "../Constants/Pins.h"

namespace {
// No line for longer than this counts as signal lost, same role the old
// SBUS class's frame timeout played.
constexpr unsigned long LINE_TIMEOUT_MS = 200;
constexpr unsigned int LINE_MAX_CHARS = 64;

String rx = "";
unsigned long lastValidLineMs = 0;
bool everReceivedLine = false;
}  // namespace

int MechLink::ch0 = 1500;
int MechLink::ch1 = 1500;
int MechLink::ch3 = 1500;
bool MechLink::signalLost = true; // fail safe until the first line ever arrives
int MechLink::modeChannel = 1500;
int MechLink::fireChannel = 1500;

void MechLink::init() {
    MECH_LINK_SERIAL_PORT.begin(115200);
    rx.reserve(LINE_MAX_CHARS);
}

void MechLink::update() {
    while (MECH_LINK_SERIAL_PORT.available()) {
        char c = MECH_LINK_SERIAL_PORT.read();

        if (c == '\n') {
            parseLine(rx);
            rx = "";
        } else if (c != '\r') {
            if (rx.length() < LINE_MAX_CHARS) {
                rx += c;
            } else {
                rx = ""; // oversize line: drop it and resync on the next '\n'
            }
        }
    }

    signalLost = !everReceivedLine || (millis() - lastValidLineMs) > LINE_TIMEOUT_MS;
    if (signalLost) {
        setNeutral();
    }
}

void MechLink::parseLine(const String& line) {
    if (!line.startsWith("CH,")) {
        return;
    }

    String fields[6];
    int fieldCount = 0;
    int start = 3; // skip "CH,"

    while (fieldCount < 6 && start <= static_cast<int>(line.length())) {
        int idx = line.indexOf(',', start);
        if (idx < 0) {
            fields[fieldCount++] = line.substring(start);
            break;
        }
        fields[fieldCount++] = line.substring(start, idx);
        start = idx + 1;
    }
    if (fieldCount != 6) {
        return;
    }

    long values[6];
    for (int i = 0; i < 6; i++) {
        char* end = nullptr;
        values[i] = strtol(fields[i].c_str(), &end, 10);
        if (end == fields[i].c_str() || *end != '\0') {
            return; // malformed field: drop the whole line, keep last-good values
        }
    }

    ch0 = static_cast<int>(values[0]);
    ch1 = static_cast<int>(values[1]);
    // values[2] (mechanism) has no consumer on this branch yet.
    ch3 = static_cast<int>(values[3]);
    fireChannel = static_cast<int>(values[4]);
    modeChannel = static_cast<int>(values[5]);

    lastValidLineMs = millis();
    everReceivedLine = true;
}

void MechLink::setNeutral() {
    ch0 = 1500;
    ch1 = 1500;
    ch3 = 1500;
    modeChannel = 1500;
    fireChannel = 1500;
}

void MechLink::sendTelemetryLine(const String& line) {
    MECH_LINK_SERIAL_PORT.println(line);
}
