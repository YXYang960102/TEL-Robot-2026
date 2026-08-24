#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "../Constants/ShooterBenchConstants.h"
#include "../Shooter/Shooter.h"

using namespace ShooterBenchConstants;

namespace {

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
bool commandTimedOut = false;

const char* angleModeName() {
    switch (Shooter::getAngleControlMode()) {
        case AngleControlMode::MANUAL_OPEN_LOOP:
            return "manual";
        case AngleControlMode::CLOSED_LOOP:
            return "closed";
        case AngleControlMode::DISABLED:
        default:
            return "disabled";
    }
}

void sendEvent(const char* event) {
    Serial.print(F("$EVENT,"));
    Serial.println(event);
}

void stopAndDisable(const char* event) {
    Shooter::setOutputsEnabled(false);
    sendEvent(event);
}

void refreshCommandWatchdog() {
    lastCommandMs = millis();
    commandTimedOut = false;
}

bool parseDirection(const char* text, int& value) {
    char* end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < -1 || parsed > 1) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

void applyManualState(int angleDirection, int rotateDirection) {
    Shooter::setAngleOpenLoop(
        angleDirection * ANGLE_MANUAL_COMMAND);
    Shooter::setRotateOpenLoop(
        rotateDirection * ROTATE_MANUAL_COMMAND);
}

void processManualCommand(char* command) {
    char* angleText = strtok(command + 2, ",");
    char* rotateText = strtok(nullptr, ",");
    char* trailingText = strtok(nullptr, ",");
    int angleDirection = 0;
    int rotateDirection = 0;

    if (angleText == nullptr || rotateText == nullptr ||
        trailingText != nullptr ||
        !parseDirection(angleText, angleDirection) ||
        !parseDirection(rotateText, rotateDirection)) {
        sendEvent("BAD_MANUAL_COMMAND");
        return;
    }

    refreshCommandWatchdog();
    if (!Shooter::areOutputsEnabled()) {
        applyManualState(0, 0);
        return;
    }
    applyManualState(angleDirection, rotateDirection);
}

void processCommand(char* command) {
    if (command[0] == '\0') {
        return;
    }

    if (strncmp(command, "M,", 2) == 0) {
        processManualCommand(command);
        return;
    }

    if (command[1] != '\0') {
        sendEvent("BAD_COMMAND");
        return;
    }

    switch (command[0]) {
        case 'E':
            Shooter::stopAll();
            Shooter::setOutputsEnabled(true);
            refreshCommandWatchdog();
            sendEvent("OUTPUTS_ENABLED");
            break;
        case 'X':
            commandTimedOut = false;
            stopAndDisable("OUTPUTS_DISABLED");
            break;
        case 'K':
            refreshCommandWatchdog();
            break;
        case '1':
            refreshCommandWatchdog();
            Shooter::setAngleOpenLoop(0.0);
            Shooter::setRotateOpenLoop(0.0);
            Shooter::zeroAngleAtCurrentPosition();
            sendEvent(Shooter::isAngleHomed()
                ? "ANGLE_ZEROED"
                : "ANGLE_ZERO_FAILED");
            break;
        case '2':
            refreshCommandWatchdog();
            if (!Shooter::areOutputsEnabled()) {
                sendEvent("ANGLE_TARGET_REJECTED_OUTPUTS_DISABLED");
                break;
            }
            sendEvent(Shooter::setAngleTargetCounts(ANGLE_SETPOINT_1_COUNTS)
                ? "ANGLE_SETPOINT_1"
                : "ANGLE_TARGET_REJECTED");
            break;
        case '3':
            refreshCommandWatchdog();
            if (!Shooter::areOutputsEnabled()) {
                sendEvent("ANGLE_TARGET_REJECTED_OUTPUTS_DISABLED");
                break;
            }
            sendEvent(Shooter::setAngleTargetCounts(ANGLE_MAXIMUM_COUNTS)
                ? "ANGLE_MAXIMUM"
                : "ANGLE_TARGET_REJECTED");
            break;
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            refreshCommandWatchdog();
            Shooter::setRotateOpenLoop(0.0);
            sendEvent("ROTATE_POSITION_UNAVAILABLE_NO_SENSOR");
            break;
        default:
            sendEvent("UNKNOWN_COMMAND");
            break;
    }
}

void readCommands() {
    while (Serial.available() > 0) {
        const char next = static_cast<char>(Serial.read());
        if (next == '\r') {
            continue;
        }
        if (next == '\n') {
            commandBuffer[commandLength] = '\0';
            processCommand(commandBuffer);
            commandLength = 0;
            continue;
        }
        if (commandLength + 1 < COMMAND_BUFFER_SIZE) {
            commandBuffer[commandLength++] = next;
        } else {
            commandLength = 0;
            sendEvent("COMMAND_TOO_LONG");
        }
    }
}

void enforceCommandWatchdog() {
    if (!Shooter::areOutputsEnabled()) {
        return;
    }
    if (millis() - lastCommandMs <= COMMAND_TIMEOUT_MS) {
        return;
    }

    commandTimedOut = true;
    stopAndDisable("COMMAND_TIMEOUT");
}

void sendTelemetry() {
    const unsigned long nowMs = millis();
    if (nowMs - lastTelemetryMs < TELEMETRY_PERIOD_MS) {
        return;
    }
    lastTelemetryMs = nowMs;

    Serial.print(F("$TEL,"));
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(Shooter::areOutputsEnabled() ? 1 : 0);
    Serial.print(',');
    Serial.print(commandTimedOut ? 1 : 0);
    Serial.print(',');
    Serial.print(angleModeName());
    Serial.print(',');
    Serial.print(Shooter::isAngleEncoderValid() ? 1 : 0);
    Serial.print(',');
    Serial.print(Shooter::isAngleHomed() ? 1 : 0);
    Serial.print(',');
    Serial.print(Shooter::getAngleRawCounts());
    Serial.print(',');
    Serial.print(Shooter::getAngleCounts());
    Serial.print(',');
    Serial.print(Shooter::getAngleDegrees(), 3);
    Serial.print(',');
    Serial.print(Shooter::getAngleTargetCounts());
    Serial.print(',');
    Serial.print(Shooter::getAngleErrorCounts());
    Serial.print(',');
    Serial.print(Shooter::getLeftAnglePulseUs());
    Serial.print(',');
    Serial.print(Shooter::getRightAnglePulseUs());
    Serial.print(',');
    Serial.print(Shooter::getRotatePulseUs());
    Serial.print(',');
    Serial.print(Shooter::getAngleControllerOutput(), 4);
    Serial.print(',');
    Serial.print(Shooter::getAngleProportionalTerm(), 4);
    Serial.print(',');
    Serial.print(Shooter::getAngleIntegralTerm(), 4);
    Serial.print(',');
    Serial.print(Shooter::getAngleDerivativeTerm(), 4);
    Serial.print(',');
    Serial.print(Shooter::getAngleFeedforwardTerm(), 4);
    Serial.print(',');
    Serial.print(Shooter::isAngleReady() ? 1 : 0);
    Serial.print(',');
    Serial.print(Shooter::areAngleSoftLimitsActive() ? 1 : 0);
    Serial.print(',');
    Serial.print(Shooter::getAngleRejectedSampleCount());
    Serial.println();
}

}

void setup() {
    Serial.begin(SERIAL_BAUD);
    Shooter::init();
    Shooter::setOutputsEnabled(false);
    sendEvent("BENCH_READY_OUTPUTS_DISABLED");
}

void loop() {
    readCommands();
    enforceCommandWatchdog();
    Shooter::update();
    sendTelemetry();
}
