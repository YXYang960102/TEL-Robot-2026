#include "MechLink.h"

#include <stdlib.h>

#include "../Constants/Pins.h"
#include "../Constants/MechLinkConstants.h"

namespace {
constexpr unsigned int LINE_MAX_CHARS = 64;
String rx = "";
}  // namespace

// Role-preserving field assignment: the old SBUS.cpp's four raw-channel
// variables each played one fixed role (ch0=drive forward, ch3=drive turn
// -- the two that getDriveForward()/getDriveTurn() actually read -- ch2/ch1/
// ch8 unconsumed extras). That mapping is kept here against the new
// transport's semantic fields (tools/stm32_elrs_common/src/
// tel_channel_map.h), not against the old raw SBUS channel *indices*
// (which this branch's old code didn't even map 1:1 -- ch1 came from raw
// index 3, ch3 from raw index 1 -- an artifact of this branch's specific
// old transmitter/receiver endpoint mapping that has no meaning now).
int MechLink::ch0 = 1500; // drive forward
int MechLink::ch1 = 1500; // auxiliary
int MechLink::ch2 = 1500; // mechanism
int MechLink::ch3 = 1500; // drive turn
int MechLink::ch8 = 1500; // mode
unsigned long MechLink::lastValidFrameMs = 0;
bool MechLink::hasValidFrame = false;

void MechLink::init() {
    MECH_LINK_SERIAL_PORT.begin(115200);
    rx.reserve(LINE_MAX_CHARS);
    setNeutral();
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

    ch0 = static_cast<int>(values[0]); // drive forward
    ch3 = static_cast<int>(values[1]); // drive turn
    ch2 = static_cast<int>(values[2]); // mechanism
    ch1 = static_cast<int>(values[3]); // auxiliary
    // values[4] (fire) has no consumer on this branch yet.
    ch8 = static_cast<int>(values[5]); // mode

    lastValidFrameMs = millis();
    hasValidFrame = true;
}

bool MechLink::isHealthy() {
    return hasValidFrame && millis() - lastValidFrameMs <= MechLinkConst::FRAME_TIMEOUT_MS;
}

unsigned long MechLink::getFrameAgeMs() {
    return hasValidFrame ? millis() - lastValidFrameMs : 0xFFFFFFFFUL;
}

double MechLink::getDriveForward() {
    return isHealthy() ? normalizedPulse(ch0) : 0.0;
}

double MechLink::getDriveTurn() {
    return isHealthy() ? normalizedPulse(ch3) : 0.0;
}

void MechLink::setNeutral() {
    ch0 = MechLinkConst::PULSE_NEUTRAL_US;
    ch1 = MechLinkConst::PULSE_NEUTRAL_US;
    ch2 = MechLinkConst::PULSE_NEUTRAL_US;
    ch3 = MechLinkConst::PULSE_NEUTRAL_US;
    ch8 = MechLinkConst::PULSE_NEUTRAL_US;
}

double MechLink::normalizedPulse(int pulseUs) {
    const int constrainedPulse = constrain(
        pulseUs, MechLinkConst::PULSE_MIN_US, MechLinkConst::PULSE_MAX_US);
    return (constrainedPulse - MechLinkConst::PULSE_NEUTRAL_US) /
           static_cast<double>(MechLinkConst::PULSE_MAX_US - MechLinkConst::PULSE_NEUTRAL_US);
}
