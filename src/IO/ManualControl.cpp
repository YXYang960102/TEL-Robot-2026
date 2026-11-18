#include "ManualControl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "../Auto/Auto.h"
#include "../Auto/AutoPath.h"
#include "../Chassis/Chassis.h"
#include "../Constants/Pins.h"
#include "../Constants/TuningConstants.h"
#include "../Dribbler/Dribbler.h"
#include "../Sensors/TurretAngle.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"
#include "AimUplink.h"
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
bool ManualControl::wirelessFirePrev = true;
uint8_t ManualControl::wirelessAutoPrev = RCMAP_AUTO_NONE;
bool ManualControl::wirelessHomePrev = true;
bool ManualControl::wirelessTrimClearPrev = true;
bool ManualControl::wirelessCapturePrev = true;
bool ManualControl::wirelessStartPrev = true;
bool ManualControl::matchStartRequested = false;
bool ManualControl::captureOn = false;
uint8_t ManualControl::ballColor = RCMAP_BALL_YELLOW;
uint8_t ManualControl::half = RCMAP_HALF_FIRST;
bool ManualControl::pathStickSemi = false;
uint8_t ManualControl::lastPathState = 0;
unsigned long ManualControl::lastVersionEventMs = 0;

double ManualControl::tankLeftCommand = 0.0;
double ManualControl::tankRightCommand = 0.0;
double ManualControl::rotateManualCommand = 0.0;
double ManualControl::angleManualCommand = 0.0;
double ManualControl::flywheelCommand = 0.0;
OperatorMode ManualControl::operatorMode = OperatorMode::FULL_MANUAL;
bool ManualControl::fireRequested = false;
uint8_t ManualControl::target = RCMAP_TARGET_NONE;
uint8_t ManualControl::autoPathThisEnable = RCMAP_AUTO_NONE;
uint8_t ManualControl::autoPathsUsedMask = 0;
double ManualControl::aimOffsetPx = 0.0;
double ManualControl::elevationOffsetUs = 0.0;
int ManualControl::flywheelOffsetPct = 0;
int ManualControl::rotateNudgePrev = 0;
bool ManualControl::turretHeldHome = false;
uint16_t ManualControl::targetSelectSeq = 0;
bool ManualControl::aimJogging = false;
bool ManualControl::aimJogSettling = false;
bool ManualControl::aimHandHeld = false;
uint8_t ManualControl::handHeldAimTarget = 0;
unsigned long ManualControl::aimJogReleasedMs = 0;
int ManualControl::angleNudgePrev = 0;
unsigned long ManualControl::elevationJogLastMs = 0;
bool ManualControl::elevationJogPrev = false;
uint16_t ManualControl::trimSeq = 0;

unsigned long ManualControl::eventSeq = 0;
uint16_t ManualControl::lastEventCode = RCMAP_EVENT_NONE;
unsigned long ManualControl::lastStatusMs = 0;
uint8_t ManualControl::statusSlot = 0;

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
    target = RCMAP_TARGET_NONE;
    autoPathThisEnable = RCMAP_AUTO_NONE;
    autoPathsUsedMask = 0;
    eventSeq = 0;
    lastEventCode = RCMAP_EVENT_NONE;
    lastStatusMs = millis();
    statusSlot = 0;
}

void ManualControl::update() {
    enforceWatchdogs();
    readIncomingLines();
    processWirelessFrame();

    if (owner != Owner::NONE) {
        updateFineAdjust();
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

uint8_t ManualControl::getTarget() {
    return target;
}

uint8_t ManualControl::getAutoPathThisEnable() {
    return autoPathThisEnable;
}

uint8_t ManualControl::getAutoPathsUsedMask() {
    return autoPathsUsedMask;
}

void ManualControl::readIncomingLines() {
    while (Serial.available() > 0) {
        const char next = static_cast<char>(Serial.read());
        if (next == '\r') {
            continue;
        }
        if (next == '\n') {
            if (discardingLine) {
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
        disarm(RCMAP_EVENT_OUTPUTS_DISABLED);
        return;
    }
    if (strcmp(command, "K") == 0) {
        if (owner == Owner::WIRED) {
            refreshWiredWatchdog();
        }
        return;
    }

    if (strcmp(command, "TZERO") == 0) {
        const bool ok = owner == Owner::NONE && TurretAngle::setZeroHere();
        sendEvent(ok ? RCMAP_EVENT_TURRET_ZERO_SET : RCMAP_EVENT_TURRET_ZERO_REJECTED);
        return;
    }

    const bool isTank = strncmp(command, "T,", 2) == 0;
    const bool isRotate = strncmp(command, "ROT,", 4) == 0;
    const bool isAngle = strncmp(command, "ANG,", 4) == 0;
    const bool isFlywheel = strncmp(command, "FLY,", 4) == 0;
    const bool isAngleTarget = strncmp(command, "ANGT,", 5) == 0;
    const bool isTarget = strncmp(command, "TGT,", 4) == 0;
    const bool isAutoPath = strncmp(command, "AUTO,", 5) == 0;
    const bool isModeSet = strncmp(command, "MODE,", 5) == 0;
    const bool isFlywheelOffset = strncmp(command, "FLYO,", 5) == 0;
    const bool isSingle = command[1] == '\0' && (command[0] == 'F' || command[0] == 'M');
    const bool isTurretHome = strcmp(command, "THOME") == 0;
    const bool isTrimClear = strcmp(command, "TCLR") == 0;
    const bool isCapture = strcmp(command, "CAPT") == 0;
    const bool isBallColor = strncmp(command, "COL,", 4) == 0;
    const bool isHalf = strncmp(command, "HALF,", 5) == 0;
    const bool isMatchStart = strcmp(command, "START") == 0;

    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isSingle && !isAngleTarget &&
        !isTarget && !isAutoPath && !isModeSet && !isFlywheelOffset && !isTurretHome &&
        !isTrimClear && !isCapture && !isBallColor && !isHalf && !isMatchStart) {
        sendEvent(RCMAP_EVENT_UNKNOWN_COMMAND);
        return;
    }

    if (owner == Owner::NONE) {
        sendEvent(RCMAP_EVENT_REJECTED_DISARMED);
        return;
    }
    if (owner != Owner::WIRED) {
        sendEvent(RCMAP_EVENT_REJECTED_NOT_OWNER);
        return;
    }

    if (isTurretHome) {
        refreshWiredWatchdog();
        requestTurretHome();
        return;
    }
    if (isTrimClear) {
        refreshWiredWatchdog();
        clearTrim();
        return;
    }
    if (isCapture) {
        refreshWiredWatchdog();
        toggleCapture();
        return;
    }
    if (isMatchStart) {
        refreshWiredWatchdog();
        matchStart();
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
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
        }
        return;
    }
    if (isTarget || isAutoPath || isModeSet || isFlywheelOffset || isBallColor || isHalf) {
        const char* digits = command + (isTarget || isBallColor ? 4 : 5);
        char* end = nullptr;
        const long n = strtol(digits, &end, 10);
        const long maximum = isTarget ? RCMAP_TARGET_MAX
                           : isBallColor ? RCMAP_BALL_COLORS - 1
                           : isHalf ? RCMAP_HALF_SECOND
                           : isAutoPath ? RCMAP_AUTO_PATHS
                           : isFlywheelOffset ? RCMAP_AUTO_OFFSET_MAX_PCT
                                        : static_cast<long>(OperatorMode::TEST);
        const long minimum = isAutoPath || isHalf ? 1 : isFlywheelOffset ? -RCMAP_AUTO_OFFSET_MAX_PCT : 0;
        if (end == digits || *end != '\0' || n < minimum || n > maximum) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
            return;
        }
        refreshWiredWatchdog();
        if (isHalf) {
            setHalf(static_cast<uint8_t>(n), true);
        } else if (isBallColor) {
            setBallColor(static_cast<uint8_t>(n), true);
        } else if (isFlywheelOffset) {
            flywheelOffsetPct = isAutoMode(operatorMode) ? static_cast<int>(n) : 0;
        } else if (isTarget) {
            turretHeldHome = false;
            targetSelectSeq++;
            selectTarget(static_cast<uint8_t>(n));
        } else if (isAutoPath) {
            requestAutoPath(static_cast<uint8_t>(n));
        } else if (ManualControlConstants::WIRED_MANUAL_ONLY) {
            sendEvent(RCMAP_EVENT_MODE_FULL_MANUAL);
        } else {
            pathStickSemi = false;
            setOperatorMode(static_cast<OperatorMode>(n));
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
    if (command[0] == 'F') {
        handleFire();
    } else {
        cycleOperatorMode();
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
    if (end == args || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    value = constrain(parsed, minimum, maximum);
    return true;
}

void ManualControl::resetWirelessTracking() {
    wirelessArmPrev = true;
    wirelessFirePrev = true;
    wirelessAutoPrev = 0xFF;
    wirelessHomePrev = true;
    wirelessTrimClearPrev = true;
    wirelessCapturePrev = true;
    wirelessStartPrev = true;
}

void ManualControl::processWirelessFrame() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) return;
    if (MechLink::signalLost || MechLink::frameCount == lastFrameCount) {
        return;
    }
    lastFrameCount = MechLink::frameCount;

    rcmap_command c;
    if (!rcmap_decode(MechLink::channels, RCMAP_CHANNEL_COUNT, &c)) {
        if (!rcmap_link_version_ok(MechLink::channels, RCMAP_CHANNEL_COUNT) &&
            millis() - lastVersionEventMs >= 1000) {
            lastVersionEventMs = millis();
            sendEvent(RCMAP_EVENT_LINK_VERSION_MISMATCH);
        }
        return;
    }
    lastValidWirelessMs = millis();

    const bool rising = c.armed && !wirelessArmPrev;
    const bool falling = !c.armed && wirelessArmPrev;
    wirelessArmPrev = c.armed;
    if (falling && owner == Owner::WIRELESS) {
        disarm(RCMAP_EVENT_OUTPUTS_DISABLED);
    }
    const uint8_t frameMode = rcmap_command_mode(&c);
    if (rising) {
        if (frameMode != RCMAP_MODE_FULL_MANUAL || c.flywheel > 0.0f) {
            sendEvent(RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL);
        } else {
            requestEnable(Owner::WIRELESS);
        }
    }

    const bool fireEdge = c.fire && !wirelessFirePrev;
    wirelessFirePrev = c.fire;
    const bool autoEdge = c.auto_path != RCMAP_AUTO_NONE && wirelessAutoPrev == RCMAP_AUTO_NONE;
    wirelessAutoPrev = c.auto_path;
    const bool homeEdge = c.turret_home && !wirelessHomePrev;
    wirelessHomePrev = c.turret_home;
    const bool trimClearEdge = c.trim_clear && !wirelessTrimClearPrev;
    wirelessTrimClearPrev = c.trim_clear;
    const bool captureEdge = c.capture_toggle && !wirelessCapturePrev;
    wirelessCapturePrev = c.capture_toggle;
    const bool startEdge = c.match_start && !wirelessStartPrev;
    wirelessStartPrev = c.match_start;

    if (owner == Owner::WIRELESS) {
        tankLeftCommand = constrain(c.forward - c.turn, -1.0, 1.0);
        tankRightCommand = constrain(c.forward + c.turn, -1.0, 1.0);
        angleManualCommand = c.nudge == RCMAP_NUDGE_UP ? 1.0 : (c.nudge == RCMAP_NUDGE_DOWN ? -1.0 : 0.0);
        rotateManualCommand = c.nudge == RCMAP_NUDGE_RIGHT ? 1.0 : (c.nudge == RCMAP_NUDGE_LEFT ? -1.0 : 0.0);
        OperatorMode wantedMode = static_cast<OperatorMode>(frameMode);
        if (pathStickSemi) {
            if (wantedMode == OperatorMode::FULL_AUTO) {
                wantedMode = OperatorMode::SEMI_AUTO;
            } else {
                pathStickSemi = false;
            }
        }
        setOperatorMode(wantedMode);
        flywheelCommand = c.flywheel;
        flywheelOffsetPct = c.flywheel_offset_pct;
        selectTarget(c.target);
        setBallColor(c.ball_color, false); // CH3 carries it in every frame
        if (c.half != RCMAP_HALF_KEEP) setHalf(c.half, false); // CH9 levels 0 / 2
        if (fireEdge) handleFire();
        if (autoEdge) requestAutoPath(c.auto_path);
        if (homeEdge) requestTurretHome(); // RB (CH9): back to 0 deg
        if (trimClearEdge) clearTrim();    // C (CH8 level 5)
        if (captureEdge) toggleCapture();  // P (CH9 level 3)
        if (startEdge) matchStart();       // Y (CH7 level 5)
    } else if (fireEdge || autoEdge || homeEdge || trimClearEdge || captureEdge) {
        sendEvent(owner == Owner::NONE ? RCMAP_EVENT_REJECTED_DISARMED
                                       : RCMAP_EVENT_REJECTED_NOT_OWNER);
    }
}

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
    Dribbler::setEnabled(true); // tower A15 HIGH -> LOW
    Chassis::setOutputsEnabled(true);
    Shooter::setOutputsEnabled(true);
    owner = source;
    timedOut = false;
    autoPathThisEnable = RCMAP_AUTO_NONE;
    target = RCMAP_TARGET_NONE;
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
    Dribbler::setEnabled(false); // tower A15 held HIGH while disabled
    sendEvent(reason);
}

void ManualControl::clearCommands() {
    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    flywheelCommand = 0.0;
    flywheelOffsetPct = 0;
    operatorMode = OperatorMode::FULL_MANUAL;
    fireRequested = false;
    clearFineAdjust();
    AutoPath::reset();
    pathStickSemi = false;
    lastPathState = static_cast<uint8_t>(AutoPath::State::NONE);
}

void ManualControl::clearFineAdjust() {
    turretHeldHome = false;
    aimJogging = false;
    aimJogSettling = false;
    aimHandHeld = false;
    rotateNudgePrev = NUDGE_NEEDS_RELEASE;
    angleNudgePrev = NUDGE_NEEDS_RELEASE;
}

namespace {
int signOf(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }
double clampAbs(double v, double limit) { return v > limit ? limit : (v < -limit ? -limit : v); }

bool nudgePressed(int now, int& prev) {
    const bool pressed = now != 0 && now != prev && prev != ManualControl::NUDGE_NEEDS_RELEASE;
    if (!(prev == ManualControl::NUDGE_NEEDS_RELEASE && now != 0)) prev = now;
    return pressed;
}
}

void ManualControl::updateFineAdjust() {
    using namespace TuningConstants::Auto;
    const int rotateNow = signOf(rotateManualCommand);
    const int angleNow = signOf(angleManualCommand);
    nudgePressed(rotateNow, rotateNudgePrev);
    nudgePressed(angleNow, angleNudgePrev);
    const unsigned long nowMs = millis();
    unsigned long jogElapsedMs = nowMs - elevationJogLastMs;
    if (jogElapsedMs > 100) jogElapsedMs = 100;
    elevationJogLastMs = nowMs;
    if (operatorMode == OperatorMode::FULL_MANUAL) {
        aimJogging = false;
        aimJogSettling = false;
        aimHandHeld = false;
        elevationJogPrev = false;
        return;
    }
    const bool jogNow = rotateNow != 0 && rotateNudgePrev != NUDGE_NEEDS_RELEASE;
    if (jogNow) {
        turretHeldHome = false;
        aimJogging = true;
        aimJogSettling = false;
        aimHandHeld = false;
    } else if (aimJogging) {
        aimJogging = false;
        if (AIM_OFFSET_ENABLED) {
            aimJogSettling = true;
            aimJogReleasedMs = millis();
        } else {
            aimHandHeld = true;
            handHeldAimTarget = Auto::getAimTarget();
        }
    }
    if (aimHandHeld && Auto::getAimTarget() != handHeldAimTarget) {
        aimHandHeld = false;
    }
    if (aimJogSettling && millis() - aimJogReleasedMs >= AIM_JOG_SETTLE_MS) {
        aimJogSettling = false;
        if (AIM_OFFSET_ENABLED && Vision::isValid()) {
            aimOffsetPx = clampAbs(-Vision::getTx(), AIM_OFFSET_MAX_PX);
            trimSeq++;
        }
    }
    const bool elevationJogNow = angleNow != 0 && angleNudgePrev != NUDGE_NEEDS_RELEASE;
    if (ELEVATION_TRIM_ENABLED && elevationJogNow && autoOwnsElevation()) {
        const int up = TuningConstants::Angle::INVERTED ? -1 : 1;
        elevationOffsetUs = clampAbs(
            elevationOffsetUs + angleNow * up * ELEVATION_JOG_US_PER_SECOND * jogElapsedMs / 1000.0,
            ELEVATION_OFFSET_MAX_US);
    }
    if (ELEVATION_TRIM_ENABLED && elevationJogPrev && !elevationJogNow && autoOwnsElevation()) {
        trimSeq++;
    }
    elevationJogPrev = elevationJogNow;
}

void ManualControl::clearTrim() {
    aimOffsetPx = 0.0;
    elevationOffsetUs = 0.0;
    aimJogSettling = false;
    trimSeq++;
}

void ManualControl::reportPathState() {
    const uint8_t now = static_cast<uint8_t>(AutoPath::getState());
    if (now == lastPathState) return;
    lastPathState = now;
    if (now == static_cast<uint8_t>(AutoPath::State::DONE)) {
        sendEvent(RCMAP_EVENT_AUTO_PATH_DONE);
    } else if (now == static_cast<uint8_t>(AutoPath::State::ABORTED)) {
        sendEvent(RCMAP_EVENT_AUTO_PATH_ABORTED);
    }
}

void ManualControl::toggleCapture() {
    captureOn = !captureOn;
    sendEvent(captureOn ? RCMAP_EVENT_CAPTURE_ON : RCMAP_EVENT_CAPTURE_OFF);
}

bool ManualControl::isCaptureOn() {
    return captureOn;
}

uint16_t ManualControl::getTrimSeq() {
    return trimSeq;
}

void ManualControl::setTrim(double aimOffset, double elevationOffset) {
    using namespace TuningConstants::Auto;
    aimOffsetPx = clampAbs(aimOffset, AIM_OFFSET_MAX_PX);
    elevationOffsetUs = clampAbs(elevationOffset, ELEVATION_OFFSET_MAX_US);
}

int ManualControl::getFlywheelOffsetPct() {
    return flywheelOffsetPct;
}

double ManualControl::getAimOffsetPx() {
    return TuningConstants::Auto::AIM_OFFSET_ENABLED ? aimOffsetPx : 0.0;
}

void ManualControl::requestTurretHome() {
    Shooter::requestTurretReturn();
    turretHeldHome = true;
    aimHandHeld = false;
}

uint16_t ManualControl::getTargetSelectSeq() {
    return targetSelectSeq;
}

bool ManualControl::isTurretHeldHome() {
    return turretHeldHome;
}

bool ManualControl::isAimJogActive() {
    return aimJogging || aimJogSettling || aimHandHeld;
}

bool ManualControl::isAimHandHeld() {
    return aimHandHeld;
}

double ManualControl::getAimJogCommand() {
    if (!aimJogging) return 0.0;
    const double jogSign = TuningConstants::Rotate::MANUAL_JOG_INVERTED ? -1.0 : 1.0;
    return jogSign * rotateManualCommand * TuningConstants::Auto::AIM_JOG_COMMAND;
}

double ManualControl::getElevationOffsetUs() {
    return TuningConstants::Auto::ELEVATION_TRIM_ENABLED ? elevationOffsetUs : 0.0;
}

void ManualControl::triggerFeed() {
    sendEvent(Dribbler::requestFeed() ? RCMAP_EVENT_FEED_TRIGGER_SENT : RCMAP_EVENT_FEED_BUSY);
}

void ManualControl::handleFire() {
    if (!isAutoMode(operatorMode)) {
        triggerFeed();
    } else {
        fireRequested = true;
    }
}

void ManualControl::matchStart() {
    matchStartRequested = true;
    sendEvent(RCMAP_EVENT_MATCH_STARTED);
}

bool ManualControl::takeMatchStart() {
    const bool was = matchStartRequested;
    matchStartRequested = false;
    return was;
}

bool ManualControl::isSecondHalf() {
    return half == RCMAP_HALF_SECOND;
}

void ManualControl::setHalf(uint8_t newHalf, bool announceSame) {
    if (newHalf != RCMAP_HALF_FIRST && newHalf != RCMAP_HALF_SECOND) return;
    if (newHalf == half && !announceSame) return;
    half = newHalf;
    sendEvent(half == RCMAP_HALF_SECOND ? RCMAP_EVENT_HALF_SECOND : RCMAP_EVENT_HALF_FIRST);
}

uint8_t ManualControl::getBallColor() {
    return ballColor;
}

void ManualControl::setBallColor(uint8_t color, bool announceSame) {
    if (color >= RCMAP_BALL_COLORS) return;
    if (color == ballColor && !announceSame) return;
    ballColor = color;
    sendEvent(static_cast<rcmap_event>(RCMAP_EVENT_BALL_YELLOW + color));
}

void ManualControl::selectTarget(uint8_t hole) {
    if (hole > RCMAP_TARGET_MAX || hole == target) {
        return;
    }
    target = hole;
    targetSelectSeq++;
    clearFineAdjust();
    sendEvent(RCMAP_EVENT_TARGET_SELECTED);
}

void ManualControl::requestAutoPath(uint8_t path) {
    if (ManualControlConstants::WIRED_MANUAL_ONLY || path < 1 || path > RCMAP_AUTO_PATHS ||
        autoPathThisEnable != RCMAP_AUTO_NONE) {
        sendEvent(RCMAP_EVENT_AUTO_PATH_REJECTED);
        return;
    }
    autoPathThisEnable = path;
    autoPathsUsedMask = static_cast<uint8_t>(autoPathsUsedMask | (1u << path));
    AutoPath::request(path, millis());
    lastPathState = static_cast<uint8_t>(AutoPath::getState());
    sendEvent(RCMAP_EVENT_AUTO_PATH_STARTED);
}

bool ManualControl::autoOwnsElevation() {
    return isAutoMode(operatorMode) && TuningConstants::ShotTable::ELEVATION_MEASURED;
}

void ManualControl::cycleOperatorMode() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) {
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
    turretHeldHome = false;
    aimHandHeld = false;
    rotateManualCommand = 0.0;
    flywheelCommand = 0.0;
    flywheelOffsetPct = 0;
    fireRequested = false;
    sendEvent(mode == OperatorMode::FULL_MANUAL ? RCMAP_EVENT_MODE_FULL_MANUAL
              : mode == OperatorMode::SEMI_AUTO ? RCMAP_EVENT_MODE_SEMI_AUTO
              : mode == OperatorMode::TEST      ? RCMAP_EVENT_MODE_TEST
                                                : RCMAP_EVENT_MODE_FULL_AUTO);
}

void ManualControl::applyOutputs() {
    using namespace ManualControlConstants;
    if (AutoPath::isActive() &&
        (fabs(tankLeftCommand) > TuningConstants::AutoPath::STICK_TAKEOVER ||
         fabs(tankRightCommand) > TuningConstants::AutoPath::STICK_TAKEOVER)) {
        AutoPath::abort();
        lastPathState = static_cast<uint8_t>(AutoPath::State::ABORTED);
        pathStickSemi = true;
        setOperatorMode(OperatorMode::SEMI_AUTO);
        sendEvent(RCMAP_EVENT_AUTO_PATH_TAKEOVER);
    }
    double pathLeft = 0.0;
    double pathRight = 0.0;
    const bool pathDrives = AutoPath::update(operatorMode, millis(), pathLeft, pathRight);
    reportPathState();
    if (pathDrives) {
        Chassis::setTankOpenLoop(pathLeft, pathRight);
    } else {
        Chassis::setTankOpenLoop(
            tankLeftCommand * CHASSIS_MANUAL_SCALE,
            tankRightCommand * CHASSIS_MANUAL_SCALE);
    }

    if (!autoOwnsElevation()) {
        Shooter::setAngleOpenLoop(angleManualCommand * ANGLE_MANUAL_SCALE);
    }

    if (isAutoMode(operatorMode)) {
        return;
    }
    Shooter::setFlywheelOpenLoop(flywheelCommand);
    if (operatorMode == OperatorMode::TEST) {
        return;
    }
    const double jogSign = TuningConstants::Rotate::MANUAL_JOG_INVERTED ? -1.0 : 1.0;
    Shooter::setRotateOpenLoop(jogSign * rotateManualCommand * ROTATE_MANUAL_SCALE);
}

uint8_t ManualControl::statusFlags() {
    uint8_t flags = 0;
    if (owner != Owner::NONE) flags |= RCMAP_STATUS_ARMED;
    if (owner == Owner::WIRELESS) flags |= RCMAP_STATUS_OWNER_WIRELESS;
    if (owner == Owner::WIRED) flags |= RCMAP_STATUS_OWNER_WIRED;
    if (PIN_TABLE_CONFIRMED) flags |= RCMAP_STATUS_PINS_CONFIRMED;
    if (Dribbler::isBusy()) flags |= RCMAP_STATUS_FEED_BUSY;
    if (getOperatorMode() == OperatorMode::FULL_MANUAL) flags |= RCMAP_STATUS_ROTATE_MANUAL;
    if (!MechLink::signalLost) flags |= RCMAP_STATUS_RC_LINK_OK;
    if (getOperatorMode() == OperatorMode::TEST) flags |= RCMAP_STATUS_TEST_MODE;
    return flags;
}

void ManualControl::sendStatusIfDue() {
    const unsigned long now = millis();
    if (now - lastStatusMs < ManualControlConstants::STATUS_PERIOD_MS) {
        return;
    }
    lastStatusMs = now;
    const uint8_t slot = statusSlot;
    statusSlot = static_cast<uint8_t>((statusSlot + 1) % 6);
    if (slot == 1) {
        AimUplink::send();
        return;
    }
    if (slot == 3) {
        AimUplink::sendVision();
        return;
    }
    if (slot == 4) {
        AimUplink::sendSeen();
        return;
    }
    if (slot == 5) {
        AimUplink::sendBalls();
        return;
    }
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
