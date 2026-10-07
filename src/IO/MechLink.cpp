#include "MechLink.h"

#include <stdlib.h>

#include "../Constants/Pins.h"

namespace {
constexpr unsigned long LINE_TIMEOUT_MS = 200;
constexpr unsigned int LINE_MAX_CHARS = 64;
constexpr int NEUTRAL_US = 1500;

String rx = "";
}  // namespace

// Role-preserving field assignment, same approach used on every other
// branch's MechLink: ch0/ch3 feed Chassis.cpp's vx/w (drive forward/turn,
// the two axes the new ground-bridge firmware actually has joystick
// inputs for -- tools/stm32_elrs_ground_bridge/src/config.h's
// ADC_PIN_DRIVE_FORWARD/ADC_PIN_DRIVE_TURN). ch1 feeds Chassis.cpp's vy
// (mecanum strafe); there is no third analog axis on the ground-bridge
// yet, so this maps to the "mechanism" field, which that firmware always
// sends as neutral for now -- strafe stays at zero until a real third
// axis is wired, same as it effectively was before (the old SBUS.cpp fed
// it from a real channel, but nothing on the ground-bridge produces a
// real strafe command yet either way).
int MechLink::ch0 = 1500;
int MechLink::ch1 = 1500;
int MechLink::ch2 = 1500;
int MechLink::ch3 = 1500;
int MechLink::ch8 = 1500;
unsigned long MechLink::lastValidLineMs = 0;
bool MechLink::everReceivedLine = false;

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

    const bool signalLost = !everReceivedLine ||
        (millis() - lastValidLineMs) > LINE_TIMEOUT_MS;
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

    ch0 = static_cast<int>(values[0]); // drive forward (vx)
    ch3 = static_cast<int>(values[1]); // drive turn (w)
    ch1 = static_cast<int>(values[2]); // mechanism, stand-in for strafe (vy)
    ch2 = static_cast<int>(values[3]); // auxiliary
    // values[4] (fire) has no consumer on this branch yet.
    ch8 = static_cast<int>(values[5]); // mode

    lastValidLineMs = millis();
    everReceivedLine = true;
}

void MechLink::setNeutral() {
    ch0 = NEUTRAL_US;
    ch1 = NEUTRAL_US;
    ch2 = NEUTRAL_US;
    ch3 = NEUTRAL_US;
    ch8 = NEUTRAL_US;
}
