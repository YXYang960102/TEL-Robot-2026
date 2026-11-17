#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

constexpr uint8_t MOTOR_PIN = A5;
constexpr int NEUTRAL_US = 1500;
constexpr int SPAN_US = 500;  // full speed: 1000..2000 us
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr unsigned long TELEMETRY_MS = 100;
constexpr size_t MAX_LINE_LENGTH = 32;

Servo motor;
bool armed = false;
double turn = 0.0;
double rotateKey = 0.0;
int pulseUs = NEUTRAL_US;
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
char line[MAX_LINE_LENGTH];
size_t lineLength = 0;
bool discarding = false;

void event(const char* name) {
    Serial.print(F("$EVENT,"));
    Serial.println(name);
}

void writeMotor() {
    const double command = rotateKey != 0.0 ? rotateKey : turn;
    pulseUs = armed ? NEUTRAL_US + static_cast<int>(lround(command * SPAN_US)) : NEUTRAL_US;
    motor.writeMicroseconds(pulseUs);
}

void disarm(const char* reason) {
    armed = false;
    turn = 0.0;
    rotateKey = 0.0;
    writeMotor();
    event(reason);
}

bool parseValue(const char* text, double& out) {
    char* end = nullptr;
    const double v = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(v)) return false;
    out = v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v);
    return true;
}

void handleLine(char* text) {
    if (text[0] == '\0') return;
    if (strcmp(text, "E") == 0) {
        armed = true;
        turn = 0.0;
        rotateKey = 0.0;
        lastCommandMs = millis();
        writeMotor();
        event("OUTPUTS_ENABLED");
        return;
    }
    if (strcmp(text, "X") == 0) {
        disarm("OUTPUTS_DISABLED");
        return;
    }
    const bool isTank = strncmp(text, "T,", 2) == 0;
    const bool isRotate = strncmp(text, "ROT,", 4) == 0;
    const bool known = isTank || isRotate || strcmp(text, "K") == 0 ||
        strncmp(text, "FLY,", 4) == 0 ||
        strncmp(text, "ANG", 3) == 0 || strncmp(text, "TGT,", 4) == 0 ||
        strncmp(text, "AUTO,", 5) == 0 ||
        (text[1] == '\0' && strchr("FM", text[0]) != nullptr);
    if (!known) {
        event("UNKNOWN_COMMAND");
        return;
    }
    if (!armed) {
        if (strcmp(text, "K") != 0) event("REJECTED_DISARMED");
        return;
    }
    if (isTank) {
        char* comma = strchr(text + 2, ',');
        double left = 0.0, right = 0.0;
        if (comma == nullptr) { event("BAD_COMMAND"); return; }
        *comma = '\0';
        if (!parseValue(text + 2, left) || !parseValue(comma + 1, right)) {
            event("BAD_COMMAND");
            return;
        }
        turn = (right - left) / 2.0;
    }
    if (isRotate) {
        double value = 0.0;
        if (!parseValue(text + 4, value)) {
            event("BAD_COMMAND");
            return;
        }
        rotateKey = value;
    }
    lastCommandMs = millis();
    writeMotor();
}

void readUsb() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            if (!discarding) {
                line[lineLength] = '\0';
                handleLine(line);
            }
            lineLength = 0;
            discarding = false;
            continue;
        }
        if (discarding) continue;
        if (lineLength + 1 < MAX_LINE_LENGTH) {
            line[lineLength++] = c;
        } else {
            lineLength = 0;
            discarding = true;
            if (armed) disarm("DISABLED_CORRUPT_COMMAND");
        }
    }
}

void printTelemetry(unsigned long now) {
    Serial.print(F("TEL,"));
    Serial.print(now);
    Serial.print(F(",0,0,0,0,0,"));
    Serial.print(F("1500,1500,2000,1000,1000,")); // RC slots 7-11
    Serial.print(F("0,3,0,"));
    Serial.print(F("NaN,NaN,NaN,"));
    Serial.print(armed ? 1 : 0);
    Serial.print(',');
    Serial.print(pulseUs);                        // 19 "left" pulse = the A5 motor
    Serial.print(F(",1500,"));
    Serial.print(armed ? 1 : 0);
    Serial.println(F(",0,0,0,0,1500,0.0"));
}

}

void setup() {
    Serial.begin(115200);
    motor.writeMicroseconds(NEUTRAL_US);
    motor.attach(MOTOR_PIN, NEUTRAL_US - SPAN_US, NEUTRAL_US + SPAN_US);
    writeMotor();
    Serial.println(F("$GAMEPAD_MOTOR_TEST_READY,A5,+-500us"));
}

void loop() {
    readUsb();
    const unsigned long now = millis();
    if (armed && now - lastCommandMs > COMMAND_TIMEOUT_MS) {
        disarm("COMMAND_TIMEOUT");
    }
    if (now - lastTelemetryMs >= TELEMETRY_MS) {
        lastTelemetryMs = now;
        printTelemetry(now);
    }
}
