#include "WiredControl.h"

#include <stdlib.h>
#include <string.h>

#include "../Chassis/Chassis.h"
#include "../Constants/DribblerConstants.h"
#include "../Constants/Pins.h"
#include "../Dribbler/Dribbler.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"
#include "MechLink.h"

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
double WiredControl::flywheelCommand = 0.0;
// Defaults to manual (not vision-follow): vision-follow would otherwise
// start driving the turret toward whatever Vision locks onto the instant
// outputs are enabled, with no explicit per-axis operator action.
bool WiredControl::rotateManualActive = true;
bool WiredControl::continuousFeedOn = false;
bool WiredControl::singleShotPending = false;

void WiredControl::init() {
    commandLength = 0;
    armed = false;
    everConnected = false;
    timedOut = false;
    lastCommandMs = millis();
    clearCommands();
}

void WiredControl::update() {
    // Watchdog is checked before reading new bytes: a heartbeat that only
    // shows up after the deadline must not rescue an already-expired
    // session (it would otherwise refresh the timer moments before the
    // check and keep stale motions alive).
    enforceWatchdog();

    if (armed && !MechLink::signalLost) {
        disarm("DISABLED_RC_LINK_ACTIVE");
    }

    readIncomingLines();

    if (!armed) {
        // Every actuator command is refused while disarmed (see
        // processCommand), so nothing here should be running -- but make
        // sure, rather than trust that no path ever left it on.
        Dribbler::setFeedAllowed(false);
        if (Dribbler::getControlMode() == DribblerControlMode::MANUAL_OPEN_LOOP) {
            Dribbler::stop();
        }
        return;
    }

    using namespace WiredControlConstants;
    Chassis::setTankOpenLoop(
        tankLeftCommand * CHASSIS_MANUAL_SCALE,
        tankRightCommand * CHASSIS_MANUAL_SCALE);

    if (rotateManualActive) {
        Shooter::setRotateOpenLoop(rotateManualCommand * ROTATE_MANUAL_SCALE);
    } else if (Vision::isConnected() && Vision::isVisionReady()) {
        Shooter::setRotateVisionFollow();
    } else {
        Shooter::disableRotate();
    }
    Shooter::setAngleOpenLoop(angleManualCommand * ANGLE_MANUAL_SCALE);
    Shooter::setFlywheelOpenLoop(flywheelCommand);

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

bool WiredControl::isActive() {
    return armed;
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

double WiredControl::getFlywheelCommand() {
    return flywheelCommand;
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

    if (strcmp(command, "E") == 0) {
        requestEnable();
        return;
    }
    if (strcmp(command, "X") == 0) {
        disarm("OUTPUTS_DISABLED");
        return;
    }
    if (strcmp(command, "K") == 0) {
        if (armed) {
            refreshWatchdog();
        }
        return;
    }

    const bool isTank = strncmp(command, "T,", 2) == 0;
    const bool isRotate = strncmp(command, "ROT,", 4) == 0;
    const bool isAngle = strncmp(command, "ANG,", 4) == 0;
    const bool isFlywheel = strncmp(command, "FLY,", 4) == 0;
    const bool isSingle = command[1] == '\0' &&
        (command[0] == 'R' || command[0] == 'F' || command[0] == 'L');

    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isSingle) {
        sendEvent("UNKNOWN_COMMAND");
        return;
    }

    // Firmware-side refusal, independent of what the page thinks its own
    // armed state is: nothing moves unless this side is armed.
    if (!armed) {
        sendEvent("REJECTED_DISARMED");
        return;
    }

    if (isTank) {
        double left = 0.0;
        double right = 0.0;
        if (!parseTank(command + 2, left, right)) {
            sendEvent("BAD_COMMAND");
            return;
        }
        refreshWatchdog();
        tankLeftCommand = left;
        tankRightCommand = right;
        return;
    }
    if (isRotate || isAngle || isFlywheel) {
        double value = 0.0;
        const double minimum = isFlywheel ? 0.0 : -1.0;
        if (!parseValue(command + 4, minimum, 1.0, value)) {
            sendEvent("BAD_COMMAND");
            return;
        }
        refreshWatchdog();
        if (isRotate) {
            rotateManualCommand = value;
        } else if (isAngle) {
            angleManualCommand = value;
        } else {
            flywheelCommand = value;
        }
        return;
    }

    refreshWatchdog();
    switch (command[0]) {
        case 'R':
            // Last explicit feed command wins: R also cancels a pending F.
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
            // F while a previous F is still in flight is refused outright:
            // no restarting its ball count or jam clock, no queueing a
            // second ball behind it.
            if (singleShotPending) {
                sendEvent("FEED_BUSY");
                break;
            }
            // Forcibly interrupts a running R (Jeremy, 2026-10-11: "F 強制
            // 打斷 R，改成送一顆就停") -- does not resume R afterward.
            continuousFeedOn = false;
            singleShotPending = true;
            Dribbler::setShootRequest(1);
            sendEvent("FEED_SINGLE_START");
            break;
        case 'L':
            rotateManualActive = !rotateManualActive;
            rotateManualCommand = 0.0;
            sendEvent(rotateManualActive
                ? "ROTATE_MANUAL"
                : "ROTATE_VISION_FOLLOW");
            break;
    }
}

bool WiredControl::parseTank(char* args, double& left, double& right) {
    char* leftText = strtok(args, ",");
    char* rightText = strtok(nullptr, ",");
    char* trailingText = strtok(nullptr, ",");
    if (leftText == nullptr || rightText == nullptr || trailingText != nullptr) {
        return false;
    }
    return parseValue(leftText, -1.0, 1.0, left) &&
           parseValue(rightText, -1.0, 1.0, right);
}

bool WiredControl::parseValue(char* args, double minimum, double maximum, double& value) {
    char* end = nullptr;
    const double parsed = strtod(args, &end);
    if (end == args || *end != '\0') {
        return false;
    }
    value = constrain(parsed, minimum, maximum);
    return true;
}

void WiredControl::requestEnable() {
    if (!PIN_TABLE_CONFIRMED) {
        sendEvent("ENABLE_REJECTED_PINS_UNCONFIRMED");
        return;
    }
    if (!MechLink::signalLost) {
        sendEvent("ENABLE_REJECTED_RC_LINK_ACTIVE");
        return;
    }

    clearCommands();
    Shooter::stopAll();
    Dribbler::stop();
    Dribbler::setFeedAllowed(false);
    Chassis::setOutputsEnabled(true);
    Shooter::setOutputsEnabled(true);
    armed = true;
    timedOut = false;
    refreshWatchdog();
    sendEvent("OUTPUTS_ENABLED");
}

void WiredControl::enforceWatchdog() {
    if (!armed) {
        return;
    }
    if (millis() - lastCommandMs <= WiredControlConstants::COMMAND_TIMEOUT_MS) {
        return;
    }
    timedOut = true;
    disarm("COMMAND_TIMEOUT");
}

void WiredControl::disarm(const char* reasonEvent) {
    armed = false;
    clearCommands();
    Chassis::setOutputsEnabled(false);
    Shooter::setOutputsEnabled(false);
    Dribbler::stop();
    Dribbler::setFeedAllowed(false);
    sendEvent(reasonEvent);
}

void WiredControl::clearCommands() {
    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    flywheelCommand = 0.0;
    rotateManualActive = true;
    continuousFeedOn = false;
    singleShotPending = false;
}

void WiredControl::refreshWatchdog() {
    lastCommandMs = millis();
}

void WiredControl::sendEvent(const char* name) {
    Serial.print(F("$EVENT,"));
    Serial.println(name);
}
