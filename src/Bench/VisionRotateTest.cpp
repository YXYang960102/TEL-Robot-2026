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

// Existing active-high limit wiring: disconnected inputs are pulled HIGH and
// block movement. Never bridge a limit just to make the motor move.
using namespace VisionRotateBenchConstants;
static_assert(SIGNAL_PIN == A5, "This bench requires Arduino Mega A5");

void applyOutput() {
    state.applyLimits(digitalRead(LEFT_LIMIT_PIN) == HIGH,
                      digitalRead(RIGHT_LIMIT_PIN) == HIGH);
    rotateServo.writeMicroseconds(state.pulse());
}

// Expected Orin format: tx,ty,distance,target_id,valid (5 signed ints).
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
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    rotateServo.writeMicroseconds(NEUTRAL_US);
    rotateServo.attach(SIGNAL_PIN, 1000, 2000);
    applyOutput();

    Serial.begin(BAUD);
    Serial1.begin(VISION_BAUD);
    // Distinct signature so the old rotate.html (keyboard bench) can't
    // mistakenly drive this firmware.
    Serial.println(F("$VISION_ROTATE_READY,3"));
    // Handshake is independent of the motion-armed gate: tell Orin we're
    // here as early as possible so it can start streaming before the
    // operator even opens the browser page.
    Serial1.print(F("MEGA_READY,"));
    Serial1.println(PROTOCOL_VERSION);
    lastHeartbeat = millis();
}

void loop() {
    state.tick(millis());

    // Bounded USB input so continuous garbage cannot starve the watchdog.
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

    // Bounded Serial1 input (Orin vision CSV). A malformed line is dropped,
    // not treated as an operator error — only silence trips VISION_TIMEOUT.
    for (uint8_t budget = 0; budget < 64 && Serial1.available(); ++budget) {
        const char ch = Serial1.read();
        if (ch == '\n') {
            if (!visionDiscard && visionLength > 0) {
                visionLine[visionLength] = '\0';
                if (parseVisionLine(visionLine)) state.onVision(lastTx, lastValid, millis());
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
        Serial.print(F("$VROTATE,3,"));
        Serial.print(state.enabled ? 1 : 0); Serial.print(',');
        Serial.print(state.pulse()); Serial.print(',');
        Serial.print(digitalRead(LEFT_LIMIT_PIN) == HIGH ? 1 : 0); Serial.print(',');
        Serial.print(digitalRead(RIGHT_LIMIT_PIN) == HIGH ? 1 : 0); Serial.print(',');
        Serial.print(state.noLimits ? 1 : 0); Serial.print(',');
        Serial.print(state.stopReason); Serial.print(',');
        Serial.print(lastTx); Serial.print(',');
        Serial.print(lastTy); Serial.print(',');
        Serial.print(lastDistance); Serial.print(',');
        Serial.print(lastTargetId); Serial.print(',');
        Serial.print(lastValid ? 1 : 0); Serial.print(',');
        Serial.println((unsigned long)(now - state.lastVisionRx));
    }
}
