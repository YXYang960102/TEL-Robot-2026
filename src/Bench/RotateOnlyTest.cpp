#include <Arduino.h>
#include <Servo.h>
#include "../Control/RotateBenchState.h"

namespace {
Servo rotateServo;
RotateBenchState state;
char line[16];
uint8_t length = 0;
bool discardLine = false;
uint32_t lastTelemetry = 0;
// Existing active-high limit wiring: disconnected inputs are pulled HIGH and
// block movement. Never bridge a limit just to make the motor move.
using namespace RotateBenchConstants;
static_assert(SIGNAL_PIN == A5, "This bench requires Arduino Mega A5");

void applyOutput() {
    state.applyLimits(digitalRead(LEFT_LIMIT_PIN) == HIGH,
                      digitalRead(RIGHT_LIMIT_PIN) == HIGH);
    rotateServo.writeMicroseconds(state.pulse());
}
}

void setup() {
    Serial.begin(RotateBenchConstants::BAUD);
    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    rotateServo.writeMicroseconds(RotateBenchConstants::NEUTRAL_US);
    rotateServo.attach(SIGNAL_PIN, 1000, 2000);
    applyOutput();
    Serial.println(F("$ROTATE_READY,2"));
}

void loop() {
    state.tick(millis());
    // Bounded input work so continuous garbage cannot starve the watchdog.
    for (uint8_t budget = 0; budget < 32 && Serial.available(); ++budget) {
        const char ch = Serial.read();
        if (ch == '\n') {
            if (!discardLine) {
                line[length] = '\0';
                state.command(line, millis());
            }
            length = 0;
            discardLine = false;
        } else if (ch != '\r' && !discardLine) {
            if (length < sizeof(line) - 1) line[length++] = ch;
            else { discardLine = true; state.stop("BAD_COMMAND"); }
        }
        applyOutput();
    }
    state.tick(millis());
    applyOutput();
    const uint32_t now = millis();
    if (uint32_t(now - lastTelemetry) >= RotateBenchConstants::TELEMETRY_MS) {
        lastTelemetry = now;
        // Distinct signature prevents the old full-Shooter UI being accepted.
        Serial.print(F("$ROTATE,2,"));
        Serial.print(state.enabled ? 1 : 0); Serial.print(',');
        Serial.print(state.pulse()); Serial.print(',');
        Serial.print(digitalRead(LEFT_LIMIT_PIN) == HIGH ? 1 : 0); Serial.print(',');
        Serial.print(digitalRead(RIGHT_LIMIT_PIN) == HIGH ? 1 : 0);
        Serial.print(','); Serial.print(state.noLimits ? 1 : 0);
        Serial.print(','); Serial.println(state.stopReason);
    }
}
