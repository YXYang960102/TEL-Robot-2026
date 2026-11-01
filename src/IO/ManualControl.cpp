#include "ManualControl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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
unsigned long ManualControl::aimJogReleasedMs = 0;
int ManualControl::angleNudgePrev = 0;

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
    statusSlot = 0; // status first
}

void ManualControl::update() {
    // Watchdogs run before new input is read: a heartbeat or frame that
    // only shows up after the deadline must not rescue an expired session.
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

    // TZERO (2026-10-31): the turret is at its home point now -- angle 0,
    // saved to EEPROM. Only while disarmed: the operator sets the turret
    // there by hand first (Dashboard 「設為歸位點」, wired).
    if (strcmp(command, "TZERO") == 0) {
        const bool ok = owner == Owner::NONE && TurretAngle::setZeroHere();
        sendEvent(ok ? RCMAP_EVENT_TURRET_ZERO_SET : RCMAP_EVENT_TURRET_ZERO_REJECTED);
        return;
    }

    const bool isTank = strncmp(command, "T,", 2) == 0;
    const bool isRotate = strncmp(command, "ROT,", 4) == 0;
    const bool isAngle = strncmp(command, "ANG,", 4) == 0;
    const bool isFlywheel = strncmp(command, "FLY,", 4) == 0;
    // Elevation as an absolute angle (Dashboard slider / number box):
    // ANGT,<deg> = go to. The arrows use the ANG hold-to-move jog.
    const bool isAngleTarget = strncmp(command, "ANGT,", 5) == 0;
    // TGT,<0..12>: the target hole (keys 1-9 the 3x3 grid, 10-12 the bonus
    // holes, keys Q W E; 0 = none).
    const bool isTarget = strncmp(command, "TGT,", 4) == 0;
    // AUTO,<1..4>: start auto path 1-4 (the Dashboard's path menu sends it
    // right after Enable).
    const bool isAutoPath = strncmp(command, "AUTO,", 5) == 0;
    // MODE,<0..3>: go straight to an operator mode (0 full auto, 1 semi
    // auto, 2 full manual, 3 test) -- the Dashboard's mode menu, sent right
    // after Enable. Enable itself still always starts in full manual.
    const bool isModeSet = strncmp(command, "MODE,", 5) == 0;
    // FLYO,<-20..20>: the autos' flywheel offset in percent (LT / RT on the
    // pad), added to the shot table. Absolute, so a resend is harmless.
    const bool isFlywheelOffset = strncmp(command, "FLYO,", 5) == 0;
    const bool isSingle = command[1] == '\0' && (command[0] == 'F' || command[0] == 'M');
    // THOME (2026-10-31, RB): drive the turret back to 0 deg.
    const bool isTurretHome = strcmp(command, "THOME") == 0;

    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isSingle && !isAngleTarget &&
        !isTarget && !isAutoPath && !isModeSet && !isFlywheelOffset && !isTurretHome) {
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

    if (isTurretHome) {
        refreshWiredWatchdog();
        requestTurretHome();
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
    if (isTarget || isAutoPath || isModeSet || isFlywheelOffset) {
        const char* digits = command + (isTarget ? 4 : 5);
        char* end = nullptr;
        const long n = strtol(digits, &end, 10);
        const long maximum = isTarget ? RCMAP_TARGET_MAX
                           : isAutoPath ? RCMAP_AUTO_PATHS
                           : isFlywheelOffset ? RCMAP_AUTO_OFFSET_MAX_PCT
                                        : static_cast<long>(OperatorMode::TEST);
        const long minimum = isAutoPath ? 1 : isFlywheelOffset ? -RCMAP_AUTO_OFFSET_MAX_PCT : 0;
        if (end == digits || *end != '\0' || n < minimum || n > maximum) {
            sendEvent(RCMAP_EVENT_BAD_COMMAND);
            return;
        }
        refreshWiredWatchdog();
        if (isFlywheelOffset) {
            // Only meaningful in the autos (full manual and test have the
            // slider).
            flywheelOffsetPct = isAutoMode(operatorMode) ? static_cast<int>(n) : 0;
        } else if (isTarget) {
            turretHeldHome = false; // a hole key, even the same one: follow again
            targetSelectSeq++;      // ...and the row cycle starts over
            selectTarget(static_cast<uint8_t>(n));
        } else if (isAutoPath) {
            requestAutoPath(static_cast<uint8_t>(n));
        } else if (ManualControlConstants::WIRED_MANUAL_ONLY) {
            sendEvent(RCMAP_EVENT_MODE_FULL_MANUAL); // benches stay manual, like M
        } else {
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
    // Require arm, fire and auto to be seen released before they can act
    // again: nothing held through an outage is ever replayed.
    wirelessArmPrev = true;
    wirelessFirePrev = true;
    wirelessAutoPrev = 0xFF; // "unknown": needs a NONE first
    wirelessHomePrev = true;
}

void ManualControl::processWirelessFrame() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) return;
    if (MechLink::signalLost || MechLink::frameCount == lastFrameCount) {
        return;
    }
    lastFrameCount = MechLink::frameCount;

    rcmap_command c;
    if (!rcmap_decode(MechLink::channels, RCMAP_CHANNEL_COUNT, &c)) {
        // A ground on another channel table (CH10, RcControlMap.h): say so,
        // once a second, so "Enable never confirms" has a visible reason.
        if (!rcmap_link_version_ok(MechLink::channels, RCMAP_CHANNEL_COUNT) &&
            millis() - lastVersionEventMs >= 1000) {
            lastVersionEventMs = millis();
            sendEvent(RCMAP_EVENT_LINK_VERSION_MISMATCH);
        }
        return; // invalid frame: ignored; if it persists the watchdog disarms
    }
    lastValidWirelessMs = millis();

    const bool rising = c.armed && !wirelessArmPrev;
    const bool falling = !c.armed && wirelessArmPrev;
    wirelessArmPrev = c.armed;
    if (falling && owner == Owner::WIRELESS) {
        disarm(RCMAP_EVENT_OUTPUTS_DISABLED);
    }
    // CH4's mode, or TEST (CH4 full manual + CH9 high).
    const uint8_t frameMode = rcmap_command_mode(&c);
    if (rising) {
        // An Enable must never start in an auto mode or with the flywheel
        // on: the ground station resets CH4 to manual, flywheel 0, when it
        // raises the arm channel, so anything else here is stale or corrupt.
        if (frameMode != RCMAP_MODE_FULL_MANUAL || c.flywheel > 0.0f) {
            sendEvent(RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL);
        } else {
            requestEnable(Owner::WIRELESS);
        }
    }

    // Edges are tracked on every frame, so a press made while not owning
    // the robot can't act later.
    const bool fireEdge = c.fire && !wirelessFirePrev;
    wirelessFirePrev = c.fire;
    const bool autoEdge = c.auto_path != RCMAP_AUTO_NONE && wirelessAutoPrev == RCMAP_AUTO_NONE;
    wirelessAutoPrev = c.auto_path;
    const bool homeEdge = c.turret_home && !wirelessHomePrev;
    wirelessHomePrev = c.turret_home;

    if (owner == Owner::WIRELESS) {
        // Arcade -> tank with the same polarity as the Dashboard: turn +1
        // (stick right) = left wheel back, right wheel forward.
        tankLeftCommand = constrain(c.forward - c.turn, -1.0, 1.0);
        tankRightCommand = constrain(c.forward + c.turn, -1.0, 1.0);
        // Nudge (arrows): one direction at a time; held = hold-to-move jog.
        angleManualCommand = c.nudge == RCMAP_NUDGE_UP ? 1.0 : (c.nudge == RCMAP_NUDGE_DOWN ? -1.0 : 0.0);
        rotateManualCommand = c.nudge == RCMAP_NUDGE_RIGHT ? 1.0 : (c.nudge == RCMAP_NUDGE_LEFT ? -1.0 : 0.0);
        setOperatorMode(static_cast<OperatorMode>(frameMode));
        // The Dashboard's Falcon slider, on CH4 with the mode (decode gives
        // 0 outside full manual / test; applyOutputs only uses it there).
        flywheelCommand = c.flywheel;
        // ...and in the autos the LT / RT flywheel offset (0 in manual).
        flywheelOffsetPct = c.flywheel_offset_pct;
        selectTarget(c.target);
        if (fireEdge) handleFire();
        if (autoEdge) requestAutoPath(c.auto_path);
        if (homeEdge) requestTurretHome(); // RB (CH9): back to 0 deg
    } else if (fireEdge || autoEdge || homeEdge) {
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
    Dribbler::setEnabled(true); // tower line: held HIGH -> idle LOW
    Chassis::setOutputsEnabled(true);
    Shooter::setOutputsEnabled(true);
    owner = source;
    timedOut = false;
    // One auto path per Enable; the since-boot record (mask) is kept.
    autoPathThisEnable = RCMAP_AUTO_NONE;
    // Every Enable starts on the Orin's own pick (Jeremy, 2026-10-16:
    // semi-auto first locks whatever the AI picks, digit keys switch, 0 goes
    // back). Silent: the Enable event already says what happened.
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
    Dribbler::setEnabled(false); // tower line held HIGH while disabled
    sendEvent(reason);
}

void ManualControl::clearCommands() {
    tankLeftCommand = 0.0;
    tankRightCommand = 0.0;
    rotateManualCommand = 0.0;
    angleManualCommand = 0.0;
    flywheelCommand = 0.0;
    flywheelOffsetPct = 0;
    // Disarm and every Enable start over in full manual, with no stale F.
    operatorMode = OperatorMode::FULL_MANUAL;
    fireRequested = false;
    clearFineAdjust();
}

void ManualControl::clearFineAdjust() {
    aimOffsetPx = 0.0;
    elevationOffsetUs = 0.0;
    turretHeldHome = false; // a new hole, Enable, disarm
    aimJogging = false;
    aimJogSettling = false;
    // An arrow still held from before needs a release before it counts.
    rotateNudgePrev = NUDGE_NEEDS_RELEASE;
    angleNudgePrev = NUDGE_NEEDS_RELEASE;
}

namespace {
int signOf(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }
double clampAbs(double v, double limit) { return v > limit ? limit : (v < -limit ? -limit : v); }

// One arrow press: the direction changes to a non-zero one (from 0, or
// straight from the other side). Holding repeats the same value -- the
// Dashboard resends ROT,1 every heartbeat, the radio repeats CH7 every
// frame -- and adds nothing. After a clear, a held arrow first has to be
// released.
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
    nudgePressed(rotateNow, rotateNudgePrev); // keeps the release-first rule
    const bool anglePress = nudgePressed(angleNow, angleNudgePrev);
    if (operatorMode == OperatorMode::FULL_MANUAL) {
        aimJogging = false;
        aimJogSettling = false;
        return; // manual: the arrows are the hold-to-move jog
    }
    // Autos and test: left / right held = turn the turret by hand (Auto
    // asks isAimJogActive()); after release and AIM_JOG_SETTLE_MS still,
    // the hole's place in the image becomes the aim offset. Example: let
    // go with the hole 120 px left of center (tx = -120) -> offset +120,
    // the camera keeps the hole 120 px left. Up / down: see below.
    const bool jogNow = rotateNow != 0 && rotateNudgePrev != NUDGE_NEEDS_RELEASE;
    if (jogNow) {
        turretHeldHome = false; // the hand turn takes the turret back
        aimJogging = true;
        aimJogSettling = false;
    } else if (aimJogging) {
        aimJogging = false;
        aimJogSettling = true;
        aimJogReleasedMs = millis();
    }
    if (aimJogSettling && millis() - aimJogReleasedMs >= AIM_JOG_SETTLE_MS) {
        aimJogSettling = false;
        if (Vision::isValid()) {
            aimOffsetPx = clampAbs(-Vision::getTx(), AIM_OFFSET_MAX_PX);
        }
    }
    // Up/down only adjust Auto's elevation once Auto owns it; until the
    // shot table is measured they stay the manual jog (applyOutputs).
    if (anglePress && autoOwnsElevation()) {
        // Same direction as the jog: "up" moves the pulse the way ANG,1 does.
        const int up = TuningConstants::Angle::INVERTED ? -1 : 1;
        elevationOffsetUs = clampAbs(elevationOffsetUs + angleNow * up * ELEVATION_OFFSET_STEP_US,
                                     ELEVATION_OFFSET_MAX_US);
    }
}

int ManualControl::getFlywheelOffsetPct() {
    return flywheelOffsetPct;
}

double ManualControl::getAimOffsetPx() {
    return aimOffsetPx;
}

// RB (wired THOME or the radio's CH9): back to 0 deg (the turret guard),
// then held there in the autos / test.
void ManualControl::requestTurretHome() {
    Shooter::requestTurretReturn();
    turretHeldHome = true;
}

uint16_t ManualControl::getTargetSelectSeq() {
    return targetSelectSeq;
}

bool ManualControl::isTurretHeldHome() {
    return turretHeldHome;
}

bool ManualControl::isAimJogActive() {
    return aimJogging || aimJogSettling;
}

double ManualControl::getAimJogCommand() {
    if (!aimJogging) return 0.0; // the wait: stand still
    const double jogSign = TuningConstants::Rotate::MANUAL_JOG_INVERTED ? -1.0 : 1.0;
    return jogSign * rotateManualCommand * ManualControlConstants::ROTATE_MANUAL_SCALE;
}

double ManualControl::getElevationOffsetUs() {
    return elevationOffsetUs;
}

void ManualControl::triggerFeed() {
    // No sensor feedback from the tower: "trigger sent" is all the Mega can
    // honestly report. Busy = still inside the tower's lockout; refused,
    // never queued.
    sendEvent(Dribbler::requestFeed() ? RCMAP_EVENT_FEED_TRIGGER_SENT : RCMAP_EVENT_FEED_BUSY);
}

void ManualControl::handleFire() {
    if (!isAutoMode(operatorMode)) {
        // Full manual and test: the operator decides, the tower fires now.
        triggerFeed();
    } else {
        // Auto decides whether the shot can go now (semi-auto: only when the
        // turret is locked and the flywheel is up; it reports FIRE_NOT_READY
        // otherwise). Full-auto fires by itself, so the request is harmless.
        fireRequested = true;
    }
}

void ManualControl::selectTarget(uint8_t hole) {
    if (hole > RCMAP_TARGET_MAX || hole == target) {
        return;
    }
    // main.cpp hands it to Vision (MEGA_TARGET -> the Orin locks on this
    // hole) and Auto looks it up in that hole's shot table.
    target = hole;
    targetSelectSeq++;
    // Offsets belong to the hole they were set for.
    clearFineAdjust();
    sendEvent(RCMAP_EVENT_TARGET_SELECTED);
}

void ManualControl::requestAutoPath(uint8_t path) {
    // Once per Enable (any of the four), and only where Auto exists.
    if (ManualControlConstants::WIRED_MANUAL_ONLY || path < 1 || path > RCMAP_AUTO_PATHS ||
        autoPathThisEnable != RCMAP_AUTO_NONE) {
        sendEvent(RCMAP_EVENT_AUTO_PATH_REJECTED);
        return;
    }
    // Phase 1: the trigger, the once-per-Enable rule and the record are
    // real; the path itself (driving) needs chassis localization first.
    autoPathThisEnable = path;
    autoPathsUsedMask = static_cast<uint8_t>(autoPathsUsedMask | (1u << path));
    sendEvent(RCMAP_EVENT_AUTO_PATH_STARTED);
}

bool ManualControl::autoOwnsElevation() {
    // Test mode keeps the elevation with the operator (that is the point).
    return isAutoMode(operatorMode) && TuningConstants::ShotTable::ELEVATION_MEASURED;
}

void ManualControl::cycleOperatorMode() {
    if (ManualControlConstants::WIRED_MANUAL_ONLY) {
        // Wired benches have no camera/auto path: stay manual.
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
    // Whoever had the turret/flywheel before the switch gives them up from
    // zero: leaving auto must not jump the flywheel back to an old slider
    // value, and entering it must not carry a manual jog.
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
    Chassis::setTankOpenLoop(
        tankLeftCommand * CHASSIS_MANUAL_SCALE,
        tankRightCommand * CHASSIS_MANUAL_SCALE);

    // Elevation: the operator's, unless Auto sets it from the shot table
    // (auto mode and a measured table). The open-loop call also holds an
    // absolute ANGT target (zero jog = stay put).
    if (!autoOwnsElevation()) {
        Shooter::setAngleOpenLoop(angleManualCommand * ANGLE_MANUAL_SCALE);
    }

    if (isAutoMode(operatorMode)) {
        return; // turret and flywheel belong to Auto (runs right after this)
    }
    // Full manual and test: the operator's flywheel. In test the turret is
    // Auto's (camera follow, runs right after this).
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
    // Slots: status, aim (distance, elevation pulse, target), status,
    // vision (tx / ty / distance) -- status every 200 ms, aim and vision
    // every 400 ms each (2026-10-17, with the ELRS TLM ratio raised to 1:4;
    // it was status, status, aim at 1:32). One line per 100 ms, so the
    // robot bridge (which forwards the latest line every 100 ms) passes
    // them all.
    const uint8_t slot = statusSlot;
    // 2026-10-27: a fifth slot, the seen frame (Orin v3 detected holes).
    statusSlot = static_cast<uint8_t>((statusSlot + 1) % 5);
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
