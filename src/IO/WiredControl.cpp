#include "WiredControl.h"

#include <stdlib.h>
#include <string.h>

#include "../Chassis/Chassis.h"
#include "../Constants/DribblerConstants.h"
#include "../Dribbler/Dribbler.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"

char WiredControl::commandBuffer[WiredControlConstants::COMMAND_BUFFER_SIZE];
size_t WiredControl::commandLength = 0;

bool WiredControl::armed = false;
bool WiredControl::everConnected = false;
bool WiredControl::timedOut = false;
unsigned long WiredControl::lastCommandMs = 0;

double WiredControl::tankLeftCommand = 0.0;
double WiredControl::tankRightCommand = 0.0;
double WiredControl::rotateManualCommand = 0.0;
double WiredControl::angleManualCommand = 0.0;
// Defaults to manual (not vision-follow): vision-follow would otherwise
// start driving the turret toward whatever Vision locks onto the instant
// outputs are enabled, with no explicit per-axis operator action -- manual
// requires an actual arrow-key press to move anything. Matches the
// project's "stays stopped until configured/enabled" convention.
bool WiredControl::rotateManualActive = true;
bool WiredControl::continuousFeedOn = false;
bool WiredControl::singleShotPending = false;

void WiredControl::init() {
    commandLength = 0;
    armed = false;
    everConnected = false;
    timedOut = false;
    lastCommandMs = millis();

    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    rotateManualActive = true;
    continuousFeedOn = false;
    singleShotPending = false;
}

void WiredControl::update() {
    readIncomingLines();
    enforceWatchdog();

    if (!armed) {
        // Defense in depth: even if something upstream left a COUNTED_FEED
        // request queued, don't let it run while disarmed.
        Dribbler::setFeedAllowed(false);
        return;
    }

    Chassis::setTankOpenLoop(tankLeftCommand, tankRightCommand);

    if (rotateManualActive) {
        Shooter::setRotateOpenLoop(rotateManualCommand);
    } else {
        const bool visionHealthy =
            Vision::isConnected() && Vision::isVisionReady();
        if (visionHealthy) {
            Shooter::setRotateVisionFollow();
        } else {
            Shooter::disableRotate();
        }
    }
    Shooter::setAngleOpenLoop(angleManualCommand);

    if (singleShotPending) {
        if (Dribbler::consumeJamEvent()) {
            singleShotPending = false;
            sendEvent("FEED_JAM");
        } else if (Dribbler::getShootRemaining() == 0) {
            singleShotPending = false;
            sendEvent("FEED_SINGLE_DONE");
        }
    }
    Dribbler::setFeedAllowed(singleShotPending);
}

bool WiredControl::isRotateAuthorityActive() {
    return armed;
}

bool WiredControl::isDribblerAuthorityActive() {
    return armed && singleShotPending;
}

bool WiredControl::isArmed() {
    return armed;
}

bool WiredControl::isConnected() {
    return everConnected;
}

bool WiredControl::didTimeOut() {
    return timedOut;
}

void WiredControl::readIncomingLines() {
    while (Serial.available() > 0) {
        const char next = static_cast<char>(Serial.read());
        if (next == '\r') {
            continue;
        }
        if (next == '\n') {
            commandBuffer[commandLength] = '\0';
            everConnected = true;
            processCommand(commandBuffer);
            commandLength = 0;
            continue;
        }
        if (commandLength + 1 < WiredControlConstants::COMMAND_BUFFER_SIZE) {
            commandBuffer[commandLength++] = next;
        } else {
            commandLength = 0;
            sendEvent("COMMAND_TOO_LONG");
        }
    }
}

void WiredControl::processCommand(char* command) {
    if (command[0] == '\0') {
        return;
    }

    if (strncmp(command, "T,", 2) == 0) {
        handleTankCommand(command + 2);
        return;
    }
    if (strncmp(command, "ROT,", 4) == 0) {
        handleAxisCommand(command + 4, rotateManualCommand);
        return;
    }
    if (strncmp(command, "ANG,", 4) == 0) {
        handleAxisCommand(command + 4, angleManualCommand);
        return;
    }

    if (command[1] != '\0') {
        sendEvent("BAD_COMMAND");
        return;
    }

    switch (command[0]) {
        case 'E':
            setEnabled(true);
            break;
        case 'X':
            setEnabled(false);
            break;
        case 'K':
            refreshWatchdog();
            break;
        case 'R':
            refreshWatchdog();
            continuousFeedOn = !continuousFeedOn;
            singleShotPending = false;
            Dribbler::setFeedOpenLoop(
                continuousFeedOn
                    ? DribblerConstants::Feeder::DEFAULT_FEED_COMMAND
                    : 0.0);
            sendEvent(continuousFeedOn
                ? "FEED_CONTINUOUS_ON"
                : "FEED_CONTINUOUS_OFF");
            break;
        case 'F':
            refreshWatchdog();
            // Forcibly interrupts a running R (Jeremy, 2026-10-11: "F 強制
            // 打斷 R，改成送一顆就停") -- does not resume R afterward.
            continuousFeedOn = false;
            singleShotPending = true;
            Dribbler::setShootRequest(1);
            sendEvent("FEED_SINGLE_START");
            break;
        case 'L':
            refreshWatchdog();
            rotateManualActive = !rotateManualActive;
            rotateManualCommand = 0.0; // clear any stale manual jog
            sendEvent(rotateManualActive
                ? "ROTATE_MANUAL"
                : "ROTATE_VISION_FOLLOW");
            break;
        default:
            sendEvent("UNKNOWN_COMMAND");
            break;
    }
}

void WiredControl::handleTankCommand(char* args) {
    char* leftText = strtok(args, ",");
    char* rightText = strtok(nullptr, ",");
    char* trailingText = strtok(nullptr, ",");

    char* leftEnd = nullptr;
    char* rightEnd = nullptr;
    const double left = leftText ? strtod(leftText, &leftEnd) : 0.0;
    const double right = rightText ? strtod(rightText, &rightEnd) : 0.0;

    if (leftText == nullptr || rightText == nullptr ||
        trailingText != nullptr ||
        leftEnd == leftText || *leftEnd != '\0' ||
        rightEnd == rightText || *rightEnd != '\0') {
        sendEvent("BAD_COMMAND");
        return;
    }

    refreshWatchdog();
    tankLeftCommand = constrain(left, -1.0, 1.0);
    tankRightCommand = constrain(right, -1.0, 1.0);
}

void WiredControl::handleAxisCommand(char* args, double& target) {
    char* end = nullptr;
    const double value = strtod(args, &end);
    if (end == args || *end != '\0') {
        sendEvent("BAD_COMMAND");
        return;
    }
    refreshWatchdog();
    target = constrain(value, -1.0, 1.0);
}

void WiredControl::enforceWatchdog() {
    if (!armed) {
        return;
    }
    if (millis() - lastCommandMs <= WiredControlConstants::COMMAND_TIMEOUT_MS) {
        return;
    }
    timedOut = true;
    setEnabled(false);
    sendEvent("COMMAND_TIMEOUT");
}

void WiredControl::setEnabled(bool enabled) {
    armed = enabled;
    refreshWatchdog();

    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    rotateManualActive = true; // always re-enable into the safer manual axis
    continuousFeedOn = false;
    singleShotPending = false;

    Chassis::setOutputsEnabled(enabled);
    Dribbler::stop();
    Dribbler::setFeedAllowed(false);

    if (!enabled) {
        Shooter::disableRotate();
        Shooter::disableAngle();
    }

    sendEvent(enabled ? "OUTPUTS_ENABLED" : "OUTPUTS_DISABLED");
}

void WiredControl::refreshWatchdog() {
    lastCommandMs = millis();
    timedOut = false;
}

void WiredControl::sendEvent(const char* name) {
    Serial.print(F("$EVENT,"));
    Serial.println(name);
}
