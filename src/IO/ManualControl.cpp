#include "ManualControl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../Chassis/Chassis.h"
#include "../Constants/Pins.h"
#include "../Constants/TuningConstants.h"
#include "../Dribbler/Dribbler.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"
#include "MechLink.h"

using Owner = ManualControl::Owner;

char ManualControl::commandBuffer[ManualControlConstants::COMMAND_BUFFER_SIZE];
size_t ManualControl::commandLength = 0;
bool ManualControl::discardingLine = false;

Owner ManualControl::owner = Owner::NONE;
bool ManualControl::everConnected = false;
bool ManualControl::timedOut = false;
unsigned long ManualControl::lastWiredCommandMs = 0;

unsigned long ManualControl::lastFrameCount = 0;
unsigned long ManualControl::lastValidWirelessMs = 0;
bool ManualControl::wirelessArmPrev = true;
rcmap_counter ManualControl::feedCounter;
rcmap_counter ManualControl::lmodeCounter;

double ManualControl::tankLeftCommand = 0.0;
double ManualControl::tankRightCommand = 0.0;
double ManualControl::rotateManualCommand = 0.0;
double ManualControl::angleManualCommand = 0.0;
double ManualControl::flywheelCommand = 0.0;
// Defaults to manual (not vision-follow): vision-follow would otherwise
// start driving the turret toward whatever Vision locks onto the instant
// outputs are enabled, with no explicit per-axis operator action.
bool ManualControl::rotateManualActive = true;
OperatorMode ManualControl::operatorMode = OperatorMode::FULL_MANUAL;
bool ManualControl::fireRequested = false;

unsigned long ManualControl::eventSeq = 0;
uint16_t ManualControl::lastEventCode = RCMAP_EVENT_NONE;
unsigned long ManualControl::lastStatusMs = 0;

void ManualControl::init() {
    commandLength = 0;
    discardingLine = false;
    owner = Owner::NONE;
    everConnected = false;
    timedOut = false;
    lastWiredCommandMs = millis();
    lastFrameCount = MechLink::frameCount;
    lastValidWirelessMs = millis();
    resetWirelessTracking();
    clearCommands();
    eventSeq = 0;
    lastEventCode = RCMAP_EVENT_NONE;
    lastStatusMs = millis();
}

void ManualControl::update() {
    // Watchdogs run before new input is read: a heartbeat or frame that
    // only shows up after the deadline must not rescue an expired session.
    enforceWatchdogs();
    readIncomingLines();
    processWirelessFrame();

    if (owner != Owner::NONE) {
        applyOutputs();
    }
    sendStatusIfDue();
}

bool ManualControl::isActive() {
    return owner != Owner::NONE;
}

bool ManualControl::isArmed() {
    return owner != Owner::NONE;
}

Owner ManualControl::getOwner() {
    return owner;
}

bool ManualControl::isConnected() {
    return everConnected;
}

bool ManualControl::didTimeOut() {
    return timedOut;
}

double ManualControl::getFlywheelCommand() {
    return flywheelCommand;
}

OperatorMode ManualControl::getOperatorMode() {
    return owner == Owner::NONE ? OperatorMode::FULL_MANUAL : operatorMode;
}

bool ManualControl::takeFireRequest() {
    const bool requested = fireRequested;
    fireRequested = false;
    return requested;
}

void ManualControl::reportEvent(rcmap_event event) {
    sendEvent(event);
}

// ---------------------------------------------------------------- wired

void ManualControl::readIncomingLines() {
    while (Serial.available() > 0) {
        const char next = static_cast<char>(Serial.read());
        if (next == '\r') {
            continue;
        }
        if (next == '\n') {
            if (discardingLine) {
                // End of an oversized line: everything in it was dropped,
                // resume parsing from the next line.
                discardingLine = false;
                commandLength = 0;
                continue;
            }
            commandBuffer[commandLength] = '\0';
            everConnected = true;
            processCommand(commandBuffer);
            commandLength = 0;
            continue;
        }
        if (discardingLine) {
            continue;
        }
        if (commandLength + 1 < ManualControlConstants::COMMAND_BUFFER_SIZE) {
            commandBuffer[commandLength++] = next;
        } else {
            // Drop the rest of this line too, so its tail (e.g. "...E") is
            // never parsed as a fresh command. A corrupt wired stream while
            // the wired side owns the robot is treated like a lost link.
            commandLength = 0;
            discardingLine = true;
            sendEvent(RCMAP_EVENT_COMMAND_TOO_LONG);
            if (owner == Owner::WIRED) {
                disarm(RCMAP_EVENT_DISABLED_CORRUPT_COMMAND);
            }
        }
    }
}

void ManualControl::processCommand(char* command) {
    if (command[0] == '\0') {
        return;
    }

    if (strcmp(command, "E") == 0) {
        requestEnable(Owner::WIRED);
        return;
    }
    if (strcmp(command, "X") == 0) {
        // An explicit stop always stops, whichever side owns the robot.
        disarm(RCMAP_EVENT_OUTPUTS_DISABLED);
        return;
    }
    if (strcmp(command, "K") == 0) {
        if (owner == Owner::WIRED) {
            refreshWiredWatchdog();
        }
        return;
    }

    const bool isTank = strncmp(command, "T,", 2) == 0;
    const bool isRotate = strncmp(command, "ROT,", 4) == 0;
    const bool isAngle = strncmp(command, "ANG,", 4) == 0;
    const bool isFlywheel = strncmp(command, "FLY,", 4) == 0;
    // Elevation as an absolute angle (Dashboard slider / number box):
    // ANGT,<deg> = go to. The arrows still use the ANG hold-to-move jog.
    const bool isAngleTarget = strncmp(command, "ANGT,", 5) == 0;
    const bool isSingle = command[1] == '\0' &&
        (command[0] == 'F' || command[0] == 'L' || command[0] == 'R' || command[0] == 'M');

    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isSingle && !isAngleTarget) {
        sendEvent(RCMAP_EVENT_UNKNOWN_COMMAND);
        return;
    }

    // Firmware-side refusal, independent of what the page believes.
    if (owner == Owner::NONE) {
        sendEvent(RCMAP_EVENT_REJECTED_DISARMED);
        return;
    }
    if (owner != Owner::WIRED) {
        sendEvent(RCMAP_EVENT_REJECTED_NOT_OWNER);
        return;
    }

    if (isTank) {
        double left = 0.0;
        double right = 0.0;
        if (!parseTank(command + 2, left, right)) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
            return;
        }
        refreshWiredWatchdog();
        tankLeftCommand = left;
        tankRightCommand = right;
        return;
    }
    if (isAngleTarget) {
        double value = 0.0;
        if (!parseValue(command + 5, -1000.0, 1000.0, value)) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
            return;
        }
        refreshWiredWatchdog();
        if (autoOwnsElevation()) {
            sendEvent(RCMAP_EVENT_REJECTED_AUTO_MODE);
            return;
        }
        if (!Shooter::setAngleTargetDegrees(value)) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND); // continuous elevation: no angle
        }
        return;
    }
    if (isRotate || isAngle || isFlywheel) {
        double value = 0.0;
        const double minimum = isFlywheel ? 0.0 : -1.0;
        if (!parseValue(command + 4, minimum, 1.0, value)) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
            return;
        }
        refreshWiredWatchdog();
        if (isRotate) {
            rotateManualCommand = value;
        } else if (isAngle) {
            angleManualCommand = value;
        } else {
            flywheelCommand = value;
        }
        return;
    }

    refreshWiredWatchdog();
    if (command[0] == 'R') {
        Dribbler::setConveyor(!Dribbler::isConveyorOn());
        sendEvent(Dribbler::isConveyorOn() ? RCMAP_EVENT_CONVEYOR_ON : RCMAP_EVENT_CONVEYOR_OFF);
    } else if (command[0] == 'F') {
        handleFire();
    } else if (command[0] == 'M') {
        cycleOperatorMode();
    } else {
        toggleRotateMode();
    }
}

bool ManualControl::parseTank(char* args, double& left, double& right) {
    char* leftText = strtok(args, ",");
    char* rightText = strtok(nullptr, ",");
    char* trailingText = strtok(nullptr, ",");
    if (leftText == nullptr || rightText == nullptr || trailingText != nullptr) {
        return false;
    }
    return parseValue(leftText, -1.0, 1.0, left) &&
           parseValue(rightText, -1.0, 1.0, right);
}

bool ManualControl::parseValue(char* args, double minimum, double maximum, double& value) {
    char* end = nullptr;
    const double parsed = strtod(args, &end);
    // strtod also accepts "nan"/"inf" spellings and overflows to inf;
    // constrain() can't clamp NaN, so non-finite input is rejected outright.
    if (end == args || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    value = constrain(parsed, minimum, maximum);
    return true;
}

// ------------------------------------------------------------- wireless

void ManualControl::resetWirelessTracking() {
    // Require CH5 to be seen low before the next enable, and take the next
    // counter levels as a fresh baseline: nothing from before an outage is
    // ever replayed.
    wirelessArmPrev = true;
    rcmap_counter_reset(&feedCounter);
    rcmap_counter_reset(&lmodeCounter);
}

void ManualControl::processWirelessFrame() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) return;
    if (MechLink::signalLost || MechLink::frameCount == lastFrameCount) {
        return;
    }
    lastFrameCount = MechLink::frameCount;

    rcmap_command c;
    if (!rcmap_decode(MechLink::channels, RCMAP_CHANNEL_COUNT, &c)) {
        return; // invalid frame: ignored; if it persists the watchdog disarms
    }
    lastValidWirelessMs = millis();

    const bool rising = c.armed && !wirelessArmPrev;
    const bool falling = !c.armed && wirelessArmPrev;
    wirelessArmPrev = c.armed;
    if (falling && owner == Owner::WIRELESS) {
        disarm(RCMAP_EVENT_OUTPUTS_DISABLED);
    }
    const uint8_t frameMode = rcmap_mode_from_us(c.mode_us);
    if (rising) {
        // An Enable must never start in an auto mode: the ground station
        // resets CH4 to manual when it raises the arm channel, so anything
        // else here is stale or corrupt.
        if (frameMode != RCMAP_MODE_FULL_MANUAL) {
            sendEvent(RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL);
        } else {
            requestEnable(Owner::WIRELESS);
        }
    }

    // Counters always track, so presses made while not owning the robot
    // can't fire later.
    const int feed = rcmap_counter_update(&feedCounter, c.feed_counter);
    const int lmode = rcmap_counter_update(&lmodeCounter, c.lmode_counter);

    if (owner == Owner::WIRELESS) {
        if (Dribbler::isConveyorOn() != c.conveyor) {
            Dribbler::setConveyor(c.conveyor);
            sendEvent(c.conveyor ? RCMAP_EVENT_CONVEYOR_ON : RCMAP_EVENT_CONVEYOR_OFF);
        }
        tankLeftCommand = c.left;
        tankRightCommand = c.right;
        rotateManualCommand = c.rotate;
        angleManualCommand = c.angle;
        flywheelCommand = c.flywheel;
        setOperatorMode(static_cast<OperatorMode>(frameMode));
        if (feed == RCMAP_COUNTER_EVENT) handleFire();
        if (lmode == RCMAP_COUNTER_EVENT) toggleRotateMode();
        if (feed == RCMAP_COUNTER_SKIPPED || lmode == RCMAP_COUNTER_SKIPPED) {
            sendEvent(RCMAP_EVENT_EVENT_SKIPPED);
        }
    } else if (feed == RCMAP_COUNTER_EVENT || lmode == RCMAP_COUNTER_EVENT) {
        sendEvent(owner == Owner::NONE ? RCMAP_EVENT_REJECTED_DISARMED
                                       : RCMAP_EVENT_REJECTED_NOT_OWNER);
    }
}

// --------------------------------------------------------------- shared

void ManualControl::requestEnable(Owner source) {
    if (!PIN_TABLE_CONFIRMED) {
        sendEvent(RCMAP_EVENT_ENABLE_REJECTED_PINS_UNCONFIRMED);
        return;
    }
    if (owner != Owner::NONE && owner != source) {
        sendEvent(RCMAP_EVENT_ENABLE_REJECTED_OTHER_SOURCE);
        return;
    }

    clearCommands();
    Shooter::stopAll();
    Dribbler::stop();
    Chassis::setOutputsEnabled(true);
    Shooter::setOutputsEnabled(true);
    owner = source;
    timedOut = false;
    refreshWiredWatchdog();
    lastValidWirelessMs = millis();
    sendEvent(RCMAP_EVENT_OUTPUTS_ENABLED);
}

void ManualControl::enforceWatchdogs() {
    if (MechLink::signalLost) {
        resetWirelessTracking();
    }

    const unsigned long now = millis();
    if (owner == Owner::WIRED &&
        now - lastWiredCommandMs > ManualControlConstants::COMMAND_TIMEOUT_MS) {
        timedOut = true;
        disarm(RCMAP_EVENT_COMMAND_TIMEOUT);
    } else if (owner == Owner::WIRELESS) {
        if (MechLink::signalLost) {
            disarm(RCMAP_EVENT_DISABLED_RC_LINK_LOST);
        } else if (now - lastValidWirelessMs > ManualControlConstants::COMMAND_TIMEOUT_MS) {
            timedOut = true;
            disarm(RCMAP_EVENT_COMMAND_TIMEOUT);
        }
    }
}

void ManualControl::disarm(rcmap_event reason) {
    owner = Owner::NONE;
    clearCommands();
    Chassis::setOutputsEnabled(false);
    Shooter::setOutputsEnabled(false);
    Dribbler::stop();
    sendEvent(reason);
}

void ManualControl::clearCommands() {
    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    flywheelCommand = 0.0;
    rotateManualActive = true;
    // Disarm and every Enable start over in full manual, with no stale F.
    operatorMode = OperatorMode::FULL_MANUAL;
    fireRequested = false;
}

void ManualControl::triggerFeed() {
    // No sensor feedback from the tower: "trigger sent" is all the Mega can
    // honestly report. Busy = still inside the tower's lockout; refused,
    // never queued.
    sendEvent(Dribbler::requestFeed() ? RCMAP_EVENT_FEED_TRIGGER_SENT : RCMAP_EVENT_FEED_BUSY);
}

void ManualControl::handleFire() {
    if (operatorMode == OperatorMode::FULL_MANUAL) {
        triggerFeed();
    } else {
        // Auto decides whether the shot can go now (semi-auto: only when the
        // turret is locked and the flywheel is up; it reports FIRE_NOT_READY
        // otherwise). Full-auto fires by itself, so the request is harmless.
        fireRequested = true;
    }
}

bool ManualControl::autoOwnsElevation() {
    return operatorMode != OperatorMode::FULL_MANUAL &&
           TuningConstants::ShotTable::ELEVATION_MEASURED;
}

void ManualControl::cycleOperatorMode() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) {
        // Wired benches have no camera/auto path: stay manual, like L.
        sendEvent(RCMAP_EVENT_MODE_FULL_MANUAL);
        return;
    }
    setOperatorMode(operatorMode == OperatorMode::FULL_MANUAL ? OperatorMode::SEMI_AUTO
                    : operatorMode == OperatorMode::SEMI_AUTO  ? OperatorMode::FULL_AUTO
                                                               : OperatorMode::FULL_MANUAL);
}

void ManualControl::setOperatorMode(OperatorMode mode) {
    if (mode == operatorMode) {
        return;
    }
    operatorMode = mode;
    // Whoever had the turret/flywheel before the switch gives them up from
    // zero: leaving auto must not jump the flywheel back to an old slider
    // value, and entering it must not carry a manual jog.
    rotateManualCommand = 0.0;
    flywheelCommand = 0.0;
    fireRequested = false;
    sendEvent(mode == OperatorMode::FULL_MANUAL ? RCMAP_EVENT_MODE_FULL_MANUAL
              : mode == OperatorMode::SEMI_AUTO ? RCMAP_EVENT_MODE_SEMI_AUTO
                                                : RCMAP_EVENT_MODE_FULL_AUTO);
}

void ManualControl::toggleRotateMode() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) {
        rotateManualActive = true;
        sendEvent(RCMAP_EVENT_ROTATE_MANUAL);
        return;
    }
    rotateManualActive = !rotateManualActive;
    rotateManualCommand = 0.0; // clear any stale manual jog
    sendEvent(rotateManualActive ? RCMAP_EVENT_ROTATE_MANUAL : RCMAP_EVENT_ROTATE_VISION_FOLLOW);
}

void ManualControl::applyOutputs() {
    using namespace ManualControlConstants;
    Chassis::setTankOpenLoop(
        tankLeftCommand * CHASSIS_MANUAL_SCALE,
        tankRightCommand * CHASSIS_MANUAL_SCALE);

    // Elevation: the operator's, unless Auto sets it from the shot table
    // (auto mode and a calibrated elevation map). The open-loop call also
    // holds an absolute ANGT target (zero jog = stay put).
    if (!autoOwnsElevation()) {
        Shooter::setAngleOpenLoop(angleManualCommand * ANGLE_MANUAL_SCALE);
    }

    if (operatorMode != OperatorMode::FULL_MANUAL) {
        return; // turret and flywheel belong to Auto (runs right after this)
    }
    if (rotateManualActive) {
        Shooter::setRotateOpenLoop(rotateManualCommand * ROTATE_MANUAL_SCALE);
    } else if (Vision::isConnected() && Vision::isVisionReady()) {
        Shooter::setRotateVisionFollow();
    } else {
        Shooter::disableRotate();
    }
    Shooter::setFlywheelOpenLoop(flywheelCommand);
}

uint8_t ManualControl::statusFlags() {
    uint8_t flags = 0;
    if (owner != Owner::NONE) flags |= RCMAP_STATUS_ARMED;
    if (owner == Owner::WIRELESS) flags |= RCMAP_STATUS_OWNER_WIRELESS;
    if (owner == Owner::WIRED) flags |= RCMAP_STATUS_OWNER_WIRED;
    if (PIN_TABLE_CONFIRMED) flags |= RCMAP_STATUS_PINS_CONFIRMED;
    if (Dribbler::isBusy()) flags |= RCMAP_STATUS_FEED_BUSY;
    if (rotateManualActive) flags |= RCMAP_STATUS_ROTATE_MANUAL;
    if (!MechLink::signalLost) flags |= RCMAP_STATUS_RC_LINK_OK;
    return flags;
}

void ManualControl::sendStatusIfDue() {
    const unsigned long now = millis();
    if (now - lastStatusMs < ManualControlConstants::STATUS_PERIOD_MS) {
        return;
    }
    lastStatusMs = now;
    MechLink::sendStatus(statusFlags(), eventSeq,
                         rcmap_status_code(lastEventCode, static_cast<uint8_t>(getOperatorMode())));
}

void ManualControl::sendEvent(rcmap_event event) {
    Serial.print(F("$EVENT,"));
    Serial.println(rcmap_event_name(event));
    lastEventCode = static_cast<uint16_t>(event);
    eventSeq++;
}

void ManualControl::refreshWiredWatchdog() {
    lastWiredCommandMs = millis();
}
