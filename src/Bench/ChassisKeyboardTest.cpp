#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "../Chassis/Chassis.h"
#include "../Constants/ChassisBenchConstants.h"

using namespace ChassisBenchConstants;

namespace {

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
bool commandTimedOut = false;
int driveMode = DEFAULT_DRIVE_MODE;

void sendEvent(const char* event) {
    Serial.print(F("$EVENT,"));
    Serial.println(event);
}

void refreshCommandWatchdog() {
    lastCommandMs = millis();
    commandTimedOut = false;
}

void stopAndDisable(const char* event) {
    Chassis::setOutputsEnabled(false);
    sendEvent(event);
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

void applyManualState(int firstDirection, int secondDirection) {
    if (!Chassis::areOutputsEnabled()) {
        Chassis::stop();
        return;
    }

    if (driveMode == 1) {
        Chassis::setTankOpenLoop(
            firstDirection * MANUAL_COMMAND,
            secondDirection * MANUAL_COMMAND);
        return;
    }

    Chassis::setOpenLoop(
        firstDirection * MANUAL_COMMAND,
        secondDirection * MANUAL_COMMAND);
}

void processManualCommand(char* command) {
    char* firstText = strtok(command + 2, ",");
    char* secondText = strtok(nullptr, ",");
    char* trailingText = strtok(nullptr, ",");
    int firstDirection = 0;
    int secondDirection = 0;

    if (firstText == nullptr || secondText == nullptr ||
        trailingText != nullptr ||
        !parseDirection(firstText, firstDirection) ||
        !parseDirection(secondText, secondDirection)) {
        stopAndDisable("BAD_MANUAL_COMMAND");
        return;
    }

    refreshCommandWatchdog();
    applyManualState(firstDirection, secondDirection);
}

void selectDriveMode(int nextMode) {
    Chassis::stop();
    driveMode = nextMode;
    refreshCommandWatchdog();
    sendEvent(driveMode == 1 ? "TANK_MODE" : "ARCADE_MODE");
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
        stopAndDisable("BAD_COMMAND");
        return;
    }

    switch (command[0]) {
        case 'E':
            Chassis::setOutputsEnabled(true);
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
            selectDriveMode(1);
            break;
        case '2':
            selectDriveMode(2);
            break;
        default:
            stopAndDisable("UNKNOWN_COMMAND");
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
            stopAndDisable("COMMAND_TOO_LONG");
        }
    }
}

void enforceCommandWatchdog() {
    if (!Chassis::areOutputsEnabled() ||
        millis() - lastCommandMs <= COMMAND_TIMEOUT_MS) {
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
    Serial.print(Chassis::areOutputsEnabled() ? 1 : 0);
    Serial.print(',');
    Serial.print(commandTimedOut ? 1 : 0);
    Serial.print(',');
    Serial.print(driveMode);
    Serial.print(',');
    Serial.print(Chassis::getForwardCommand(), 3);
    Serial.print(',');
    Serial.print(Chassis::getTurnCommand(), 3);
    Serial.print(',');
    Serial.print(Chassis::getLeftCommand(), 3);
    Serial.print(',');
    Serial.print(Chassis::getRightCommand(), 3);
    Serial.print(',');
    Serial.print(Chassis::getLeftPulseUs());
    Serial.print(',');
    Serial.println(Chassis::getRightPulseUs());
}

}

void setup() {
    Serial.begin(SERIAL_BAUD);
    Chassis::init();
    Chassis::setOutputsEnabled(false);
    sendEvent("BENCH_READY_OUTPUTS_DISABLED");
}

void loop() {
    readCommands();
    enforceCommandWatchdog();
    Chassis::update();
    sendTelemetry();
}
