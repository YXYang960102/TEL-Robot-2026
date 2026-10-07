#include "MechLink.h"

#include <stdlib.h>

#include "../Constants/Pins.h"

namespace {
// No line for longer than this counts as signal lost, same role the old
// SBUS class's FRAME_TIMEOUT_MS played.
constexpr unsigned long LINE_TIMEOUT_MS = 200;
constexpr unsigned int LINE_MAX_CHARS = 64;
constexpr int PULSE_MIN_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US = 2000;

String rx = "";
}  // namespace

int MechLink::driveForwardPulseUs = NEUTRAL_US;
int MechLink::auxiliaryPulseUs = NEUTRAL_US;
int MechLink::mechanismPulseUs = NEUTRAL_US;
int MechLink::driveTurnPulseUs = NEUTRAL_US;
int MechLink::modePulseUs = NEUTRAL_US;
unsigned long MechLink::lastValidFrameMs = 0;
bool MechLink::hasValidFrame = false;

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

    if (!isHealthy()) {
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

    driveForwardPulseUs = static_cast<int>(values[0]);
    driveTurnPulseUs = static_cast<int>(values[1]);
    mechanismPulseUs = static_cast<int>(values[2]);
    auxiliaryPulseUs = static_cast<int>(values[3]);
    // values[4] (fire) has no consumer on this branch yet.
    modePulseUs = static_cast<int>(values[5]);

    lastValidFrameMs = millis();
    hasValidFrame = true;
}

bool MechLink::isHealthy() {
    return hasValidFrame && millis() - lastValidFrameMs <= LINE_TIMEOUT_MS;
}

unsigned long MechLink::getFrameAgeMs() {
    return hasValidFrame ? millis() - lastValidFrameMs : 0xFFFFFFFFUL;
}

double MechLink::getDriveForward() {
    return isHealthy() ? normalizedPulse(driveForwardPulseUs) : 0.0;
}

double MechLink::getDriveTurn() {
    return isHealthy() ? normalizedPulse(driveTurnPulseUs) : 0.0;
}

int MechLink::getDriveForwardPulseUs() {
    return driveForwardPulseUs;
}

int MechLink::getAuxiliaryPulseUs() {
    return auxiliaryPulseUs;
}

int MechLink::getMechanismPulseUs() {
    return mechanismPulseUs;
}

int MechLink::getDriveTurnPulseUs() {
    return driveTurnPulseUs;
}

int MechLink::getModePulseUs() {
    return modePulseUs;
}

void MechLink::sendTelemetryLine(const String& line) {
    MECH_LINK_SERIAL_PORT.println(line);
}

void MechLink::setNeutral() {
    driveForwardPulseUs = NEUTRAL_US;
    auxiliaryPulseUs = NEUTRAL_US;
    mechanismPulseUs = NEUTRAL_US;
    driveTurnPulseUs = NEUTRAL_US;
    modePulseUs = NEUTRAL_US;
}

double MechLink::normalizedPulse(int pulseUs) {
    const int constrainedPulse = constrain(pulseUs, PULSE_MIN_US, PULSE_MAX_US);
    return (constrainedPulse - NEUTRAL_US) / static_cast<double>(PULSE_MAX_US - NEUTRAL_US);
}
