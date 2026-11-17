#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include "../Control/VisionRotateBenchState.h"

namespace {
Servo rotateServo;
VisionRotateBenchState state;
char usbLine[16];
uint8_t usbLength = 0;
bool usbDiscard = false;
char visionLine[32];
uint8_t visionLength = 0;
bool visionDiscard = false;
uint32_t lastTelemetry = 0;
uint32_t lastHeartbeat = 0;
int lastTx = 0, lastTy = 0, lastDistance = 0, lastTargetId = 0;
bool lastValid = false;

using namespace VisionRotateBenchConstants;
static_assert(SIGNAL_PIN == A5, "This bench requires Arduino Mega A5");

void applyOutput() {
    rotateServo.writeMicroseconds(state.pulse());
}

bool parseVisionLine(const char* s) {
    int values[5];
    const char* p = s;
    for (int field = 0; field < 5; ++field) {
        char* end;
        long v = strtol(p, &end, 10);
        if (end == p) return false;
        values[field] = (int)v;
        p = end;
        if (field < 4) {
            if (*p != ',') return false;
            ++p;
        }
    }
    lastTx = values[0]; lastTy = values[1]; lastDistance = values[2];
    lastTargetId = values[3]; lastValid = values[4] != 0;
    return true;
}
}

void setup() {
    rotateServo.writeMicroseconds(NEUTRAL_US);
    rotateServo.attach(SIGNAL_PIN, 1000, 2000);
    applyOutput();

    Serial.begin(BAUD);
    Serial1.begin(VISION_BAUD);
    Serial.println(F("$VISION_ROTATE_READY,3"));
    Serial.print(F("$VROTATE_GAINS,kP=")); Serial.print(kP, 4);
    Serial.print(F(",kI=")); Serial.print(kI, 4);
    Serial.print(F(",kIZone=")); Serial.print(kIZone, 1);
    Serial.print(F(",I_LIMIT_US=")); Serial.print(PIDF_INTEGRAL_LIMIT_US);
    Serial.print(F(",kD=")); Serial.print(kD, 4);
    Serial.print(F(",kD_TAU_S=")); Serial.print(kD_FILTER_TAU_S, 4);
    Serial.print(F(",OUT_LIMIT_US=")); Serial.println(PIDF_OUTPUT_LIMIT_US);
    Serial1.print(F("MEGA_READY,"));
    Serial1.println(PROTOCOL_VERSION);
    lastHeartbeat = millis();
}

void loop() {
    state.tick(millis());

    for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
        const char ch = Serial.read();
        if (ch == '\n') {
            if (!usbDiscard) { usbLine[usbLength] = '\0'; state.command(usbLine, millis()); }
            usbLength = 0; usbDiscard = false;
        } else if (ch != '\r' && !usbDiscard) {
            if (usbLength < sizeof(usbLine) - 1) usbLine[usbLength++] = ch;
            else { usbDiscard = true; state.stop("BAD_COMMAND"); }
        }
        applyOutput();
    }

    for (uint8_t budget = 0; budget < 64 && Serial1.available(); ++budget) {
        const char ch = Serial1.read();
        if (ch == '\n') {
            if (!visionDiscard && visionLength > 0) {
                visionLine[visionLength] = '\0';
                if (parseVisionLine(visionLine)) state.onVision(lastTx, lastValid, lastTargetId, millis());
            }
            visionLength = 0; visionDiscard = false;
        } else if (ch != '\r') {
            if (visionLength < sizeof(visionLine) - 1) visionLine[visionLength++] = ch;
            else visionDiscard = true;
        }
    }

    state.tick(millis());
    applyOutput();

    const uint32_t now = millis();
    if (uint32_t(now - lastHeartbeat) >= MEGA_HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = now;
        Serial1.print(F("MEGA_HEARTBEAT,"));
        Serial1.println(PROTOCOL_VERSION);
    }
    if (uint32_t(now - lastTelemetry) >= TELEMETRY_MS) {
        lastTelemetry = now;
        Serial.print(F("$VROTATE,6,"));
        Serial.print(state.enabled ? 1 : 0); Serial.print(',');
        Serial.print(state.pulse()); Serial.print(',');
        Serial.print(state.stopReason); Serial.print(',');
        Serial.print(lastTx); Serial.print(',');
        Serial.print(lastTy); Serial.print(',');
        Serial.print(lastDistance); Serial.print(',');
        Serial.print(lastTargetId); Serial.print(',');
        Serial.print(lastValid ? 1 : 0); Serial.print(',');
        Serial.print((unsigned long)(now - state.lastVisionRx)); Serial.print(',');
        Serial.print(state.lastDtMs); Serial.print(',');
        Serial.print(state.lastPTermUs); Serial.print(',');
        Serial.print(state.lastDTermUs); Serial.print(',');
        Serial.print(state.lastOutputRawUs); Serial.print(',');
        Serial.print(state.lastSaturated ? 1 : 0); Serial.print(',');
        Serial.println(state.lastITermUs);
    }
}
