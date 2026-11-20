#include "Auto.h"

#include <Arduino.h>

#include "AutoPath.h"
#include "../Constants/ShooterConstants.h"
#include "../Constants/TuningConstants.h"
#include "../Control/ShotDistanceFilter.h"
#include "../Control/ShotTableLookup.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/ManualControl.h"
#include "../IO/OperatorMode.h"
#include "../Vision/OperatorTarget.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"

Auto::FireState Auto::fireState = Auto::FireState::IDLE;
unsigned long Auto::fireStateEnteredMs = 0;
bool Auto::haveLastShot = false;
double Auto::lastShotFlywheel = 0.0;
double Auto::lastShotElevationUs = 0.0;
unsigned long Auto::lastShotMs = 0;
uint8_t Auto::lastShotSelection = 0;
unsigned long Auto::lastElevationSlewMs = 0;
int Auto::lastShotTargetId = 0;
uint8_t Auto::readyBits = 0;
uint16_t Auto::fireBlockers = 0;
bool Auto::semiFirePending = false;
unsigned long Auto::semiFireUntilMs = 0;
double Auto::keptFlywheel = -1.0;
double Auto::keptElevationUs = -1.0;
double Auto::keptDistanceM = 0.0;
uint8_t Auto::keptHole = 0;
unsigned long Auto::elevationArrivedSinceMs = 0;
bool Auto::rowActive = false;
uint8_t Auto::rowStartHole = 0;
uint8_t Auto::farColumn = 0xFF;
uint8_t Auto::rowStep = 0;
uint8_t Auto::rowShots = 0;
bool Auto::rowDone = false;
uint8_t Auto::cycleHoles[12] = {0};
uint8_t Auto::cycleLength = 0;
uint8_t Auto::lastCycleHole = 0;
bool Auto::litActive = false;
uint8_t Auto::litHole = 0;
uint8_t Auto::litShots = 0;
uint8_t Auto::litServedMask = 0;
bool Auto::holeSwitchPending = false;
uint8_t Auto::notFoundAim = 0;
unsigned long Auto::notFoundSinceMs = 0;
bool Auto::holeSwitchFromLit = false;
unsigned long Auto::holeSwitchStartMs = 0;
unsigned long Auto::litLastSeenMs[4] = {0, 0, 0, 0};
uint8_t Auto::lastAimTarget = 0;
unsigned long Auto::aimChangedMs = 0;
uint16_t Auto::rowSelectSeq = 0;
OperatorMode Auto::rowMode = OperatorMode::FULL_MANUAL;
bool Auto::rowSecondHalf = false;
bool Auto::pathStartUsed = false;

namespace {
ShotDistanceFilter<TuningConstants::Auto::DISTANCE_MEDIAN_FRAMES> distanceFilter(
    TuningConstants::Auto::DISTANCE_LATCH_BAND_M, TuningConstants::Auto::DISTANCE_LATCH_SETTLE_MS);
unsigned long lastFilterFrame = 0;

void aimTurret(bool visionHealthy, bool sweep = false) {
    Shooter::setRotateSweepAllowed(false);
    if (ManualControl::isAimJogActive()) {
        Shooter::setRotateOpenLoop(ManualControl::getAimJogCommand());
    } else if (AutoPath::holdsTurret() || (AutoPath::isActive() && !Vision::isValid())) {
        Shooter::disableRotate();
    } else if (ManualControl::isTurretHeldHome()) {
        Shooter::disableRotate(); // RB: stay at 0 deg (the guard drove it there)
    } else if (visionHealthy) {
        Shooter::setRotateVisionFollow();
        Shooter::setRotateAimOffsetPx(ManualControl::getAimOffsetPx());
        Shooter::setRotateSweepAllowed(sweep && !AutoPath::isActive());
    } else {
        Shooter::disableRotate();
    }
}
uint8_t filterSelection = 0;
int filterTargetId = 0;
bool tableElevationValid = false;
double tableElevationUs = 0.0;

bool lookUpShot(unsigned long now, ShotTableLookup::Result& shot) {
    const uint8_t selection = Vision::getSelectedTarget();
    const bool frameHasDepth = Vision::isValid() && Vision::getDistance() > 0.0;
    if (selection != filterSelection ||
        (Vision::isValid() && Vision::getTargetId() != filterTargetId)) {
        distanceFilter.reset();
        filterSelection = selection;
        if (Vision::isValid()) filterTargetId = Vision::getTargetId();
    }
    const unsigned long frame = Vision::getFrameCount();
    if (frameHasDepth && frame != lastFilterFrame) {
        distanceFilter.addSample(Vision::getDistance() / 1000.0, now);
    }
    lastFilterFrame = frame;
    return frameHasDepth && distanceFilter.latched() > 0.0 &&
        ShotTableLookup::lookupTarget(Vision::getTargetId(), distanceFilter.latched(), shot,
                                      ManualControl::getBallColor());
}
}

double Auto::getShotDistanceM() {
    return distanceFilter.latched();
}

uint16_t Auto::getFireBlockers() {
    return fireBlockers;
}

uint8_t Auto::getReadyBits() {
    return readyBits;
}

namespace {
bool hasShotTable(uint8_t hole) {
    return TuningConstants::ShotTable::tableForVisionId(OperatorTarget::visionIdForHole(hole),
                                                        ManualControl::getBallColor()).entries != nullptr;
}

bool excludedInFullAuto(uint8_t hole) {
    return hole <= 15 && (TuningConstants::Auto::FULL_AUTO_EXCLUDED_HOLES & (1u << hole)) != 0;
}
}

uint8_t Auto::getAimTarget() {
    if (pathStartUsed && AutoPath::isActive()) {
        const TuningConstants::AutoPath::PathConfig* c = AutoPath::config(AutoPath::getPath());
        if (c != nullptr && c->driveAimHole != 0) return c->driveAimHole;
    }
    if (litActive) return litHole;
    if (rowActive) return rowDone ? lastCycleHole : cycleHoles[rowStep];
    return ManualControl::getTarget();
}

uint8_t Auto::getRowShots() {
    return litActive ? litShots : (rowActive ? rowShots : 0);
}

uint8_t Auto::getAimShots() {
    if (litActive) return TuningConstants::Auto::BONUS_SHOTS_PER_HOLE;
    if (rowActive && !rowDone) return shotsForHole(cycleHoles[rowStep]);
    return 0;
}

uint8_t Auto::shotsForHole(uint8_t hole) {
    using namespace TuningConstants::Auto;
    if (!ManualControl::isSecondHalf()) return SHOTS_PER_HOLE;
    if (SECOND_HALF_FAR_COLUMN_SHOTS > 0 && OperatorTarget::rowOf(hole) < 3 &&
        OperatorTarget::columnOf(hole) == farColumn) {
        return SECOND_HALF_FAR_COLUMN_SHOTS;
    }
    return SECOND_HALF_SHOTS_PER_HOLE;
}

bool Auto::isRowDone() {
    return rowDone;
}

uint8_t Auto::getLitPriorityHole() {
    return litActive ? litHole : 0;
}

void Auto::updateRowCycle(OperatorMode mode) {
    using namespace TuningConstants::Auto;
    const uint16_t seq = ManualControl::getTargetSelectSeq();
    const bool secondHalf = ManualControl::isSecondHalf();
    if (mode != rowMode || seq != rowSelectSeq || secondHalf != rowSecondHalf) {
        rowMode = mode;
        rowSelectSeq = seq;
        rowSecondHalf = secondHalf;
        holeSwitchPending = false;
        rowActive = false;
        rowDone = false;
        rowStep = 0;
        rowShots = 0;
        litActive = false;
        litServedMask = 0;
    }
    const unsigned long now = millis();
    const uint8_t litId = Vision::getLitBonusId();
    for (uint8_t id = 1; id <= 3; id++) {
        if (litId == id) {
            litLastSeenMs[id] = now;
        } else if ((litServedMask & (1u << id)) && now - litLastSeenMs[id] > LIT_DARK_MS) {
            litServedMask = static_cast<uint8_t>(litServedMask & ~(1u << id));
        }
    }
    if (mode != OperatorMode::FULL_AUTO) {
        holeSwitchPending = false;
        rowActive = false;
        rowDone = false;
        litActive = false;
        notFoundAim = 0;
        return;
    }
    const AutoPath::State pathState = AutoPath::getState();
    if (pathState == AutoPath::State::NONE) pathStartUsed = false;
    const uint8_t path = AutoPath::getPath();
    const TuningConstants::AutoPath::PathConfig* pathConfig = AutoPath::config(path);
    if (ROW_CYCLE_ENABLED && PATH_SETS_START_COLUMN && !pathStartUsed && pathConfig != nullptr &&
        pathState != AutoPath::State::NONE && ManualControl::getTarget() == OperatorTarget::NONE) {
        pathStartUsed = true;
        holeSwitchPending = false;
        rowDone = false;
        startCycle(pathConfig->startHole);
    }
    if (holeSwitchPending && now - holeSwitchStartMs >= HOLE_SWITCH_DELAY_MS) {
        finishHoleSwitch();
    }
    skipHoleNotFound(now);
    if (LIT_BONUS_PRIORITY && !holeSwitchPending && !litActive && litId >= 1 && litId <= 3 &&
        !(litServedMask & (1u << litId))) {
        const uint8_t hole = OperatorTarget::holeForVisionId(litId);
        if (hasShotTable(hole)) {
            litActive = true;
            litHole = hole;
            litShots = 0;
        }
    }
    if (ROW_CYCLE_ENABLED && !rowActive && !rowDone && ManualControl::getTarget() != OperatorTarget::NONE) {
        startCycle(ManualControl::getTarget());
    }
    if (ROW_CYCLE_ENABLED && CYCLE_BOTTOM_UP && !rowActive && !rowDone && !litActive) {
        const uint8_t near = OperatorTarget::holeForVisionId(Vision::getNearBottomId());
        if (near != OperatorTarget::NONE) {
            startCycle(near);
        } else if (Vision::isValid()) {
            const uint8_t picked = OperatorTarget::holeForVisionId(Vision::getTargetId());
            if (picked != OperatorTarget::NONE) {
                startCycle(OperatorTarget::holeAt(0, OperatorTarget::columnOf(picked)));
            }
        }
    }
    if (ROW_CYCLE_ENABLED && !rowActive && !rowDone && Vision::isValid()) {
        const uint8_t picked = OperatorTarget::holeForVisionId(Vision::getTargetId());
        if (CYCLE_SKIPS_BONUS_ROW && !litActive && picked != OperatorTarget::NONE &&
            OperatorTarget::rowOf(picked) == 3) {
            startCycle(picked);
        } else if (excludedInFullAuto(picked)) {
            const uint8_t instead = OperatorTarget::nearestAllowedInColumn(picked, FULL_AUTO_EXCLUDED_HOLES);
            if (instead != OperatorTarget::NONE) startCycle(instead);
        }
    }
}

void Auto::startCycle(uint8_t hole) {
    using namespace TuningConstants::Auto;
    if (CYCLE_SKIPS_BONUS_ROW && OperatorTarget::rowOf(hole) == 3) {
        hole = OperatorTarget::holeAt(2, OperatorTarget::columnOf(hole));
    }
    rowActive = true;
    rowStartHole = hole;
    rowStep = 0;
    rowShots = 0;
    lastCycleHole = hole;
    const uint16_t skipped = FULL_AUTO_EXCLUDED_HOLES | (CYCLE_SKIPS_BONUS_ROW ? BONUS_ROW_HOLES : 0);
    cycleLength = CYCLE_BOTTOM_UP
        ? OperatorTarget::buildBottomUpCycle(hole, skipped, cycleHoles)
        : OperatorTarget::buildCycle(hole, skipped, cycleHoles);
    farColumn = 0xFF;
    for (uint8_t i = 0; i < cycleLength; i++) {
        if (OperatorTarget::rowOf(cycleHoles[i]) < 3) farColumn = OperatorTarget::columnOf(cycleHoles[i]);
    }
    skipHolesWithoutTable();
}

void Auto::skipHolesWithoutTable() {
    while (rowStep < cycleLength &&
           (!hasShotTable(cycleHoles[rowStep]) || excludedInFullAuto(cycleHoles[rowStep]))) {
        rowStep++;
    }
    if (rowStep >= cycleLength) {
        rowDone = true;
    } else {
        lastCycleHole = cycleHoles[rowStep];
    }
}

void Auto::countRowShot() {
    using namespace TuningConstants::Auto;
    notFoundSinceMs = millis();
    if (litActive) {
        if (++litShots >= BONUS_SHOTS_PER_HOLE) {
            holeSwitchPending = true;
            holeSwitchFromLit = true;
            holeSwitchStartMs = millis();
        }
        return;
    }
    if (!ROW_CYCLE_ENABLED || rowDone) return;
    if (!rowActive) {
        const uint8_t hole = OperatorTarget::holeForVisionId(Vision::getTargetId());
        if (hole == OperatorTarget::NONE) return;
        startCycle(hole);
    }
    if (++rowShots < shotsForHole(cycleHoles[rowStep])) return;
    holeSwitchPending = true;
    holeSwitchFromLit = false;
    holeSwitchStartMs = millis();
}

void Auto::skipHoleNotFound(unsigned long now) {
    using namespace TuningConstants::Auto;
    const bool cycling = litActive || (rowActive && !rowDone);
    const uint8_t aim = cycling ? getAimTarget() : 0;
    if (aim != notFoundAim) {
        notFoundAim = aim;
        notFoundSinceMs = now;
    }
    if (aim == 0 || Shooter::isRotateReady() || holeSwitchPending || AutoPath::isActive()) {
        notFoundSinceMs = now;
        return;
    }
    if (HOLE_NOT_FOUND_SKIP_MS == 0 || now - notFoundSinceMs < HOLE_NOT_FOUND_SKIP_MS) return;
    if (litActive) {
        litActive = false;
        litServedMask = static_cast<uint8_t>(litServedMask | (1u << OperatorTarget::visionIdForHole(litHole)));
    } else {
        rowShots = 0;
        rowStep++;
        skipHolesWithoutTable();
    }
    notFoundAim = 0;
}

void Auto::finishHoleSwitch() {
    holeSwitchPending = false;
    if (holeSwitchFromLit) {
        litActive = false;
        litServedMask = static_cast<uint8_t>(litServedMask | (1u << OperatorTarget::visionIdForHole(litHole)));
        return;
    }
    rowShots = 0;
    rowStep++;
    skipHolesWithoutTable();
}

void Auto::init() {
    fireState = FireState::IDLE;
    fireStateEnteredMs = millis();
    haveLastShot = false;
    distanceFilter.reset();
    readyBits = 0;
    fireBlockers = 0;
    semiFirePending = false;
    keptFlywheel = -1.0;
    keptElevationUs = -1.0;
    keptHole = 0;
}

void Auto::enterFireState(FireState next) {
    fireState = next;
    fireStateEnteredMs = millis();
}

void Auto::update() {
    const OperatorMode mode = ManualControl::getOperatorMode();
    updateRowCycle(mode);
    tableElevationValid = false;
    if (mode == OperatorMode::FULL_MANUAL) {
        ManualControl::takeFireRequest();
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        distanceFilter.reset();
        readyBits = 0;
        fireBlockers = 0;
        semiFirePending = false;
        keptFlywheel = -1.0;
    keptElevationUs = -1.0;
    keptHole = 0;
        return;
    }

    const bool visionHealthy = Vision::isConnected() && Vision::isVisionReady();
    if (mode == OperatorMode::TEST) {
        ManualControl::takeFireRequest();
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        readyBits = 0;
        fireBlockers = 0;
        semiFirePending = false;
        aimTurret(visionHealthy);
        ShotTableLookup::Result shot = {0.0, 0.0};
        if (visionHealthy && lookUpShot(millis(), shot)) {
            tableElevationValid = true;
            tableElevationUs = shot.elevationPulseUs;
        }
        return;
    }

    const bool firePressed = ManualControl::takeFireRequest();
    if (!visionHealthy) {
        aimTurret(false);
        Shooter::setFlywheelOpenLoop(0.0);
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        distanceFilter.reset();
        readyBits = 0;
        fireBlockers = BLOCK_NO_CAMERA;
        semiFirePending = false;
        if (firePressed) ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
        return;
    }

    aimTurret(true, true);

    const unsigned long now = millis();
    const uint8_t selection = Vision::getSelectedTarget();
    ShotTableLookup::Result shot = {0.0, 0.0};
    const bool hasTarget = lookUpShot(now, shot);
    tableElevationValid = hasTarget;
    tableElevationUs = shot.elevationPulseUs;

    if (hasTarget) {
        haveLastShot = true;
        keptFlywheel = shot.flywheelCommand;
        keptElevationUs = shot.elevationPulseUs;
        keptDistanceM = distanceFilter.latched();
        keptHole = OperatorTarget::holeForVisionId(Vision::getTargetId());
        lastShotFlywheel = shot.flywheelCommand;
        lastShotElevationUs = shot.elevationPulseUs;
        lastShotMs = now;
        lastShotSelection = selection;
        lastShotTargetId = Vision::getTargetId();
    }
    const bool holding = !hasTarget && haveLastShot && selection == lastShotSelection &&
        now - lastShotMs <= TuningConstants::Auto::TARGET_HOLD_MS;
    if (!hasTarget && !holding) {
        haveLastShot = false;
        distanceFilter.reset();
    }

    double flywheelCommand = TuningConstants::Flywheel::SHOOT_COMMAND;
    bool elevationArrived = !TuningConstants::ShotTable::ELEVATION_MEASURED;
    if (hasTarget || holding) {
        flywheelCommand = constrain(
            lastShotFlywheel + ManualControl::getFlywheelOffsetPct() / 100.0, 0.0, 1.0);
        if (TuningConstants::ShotTable::ELEVATION_MEASURED) {
            elevationArrived = slewElevation(lastShotElevationUs + ManualControl::getElevationOffsetUs(), now);
        }
    } else if (TuningConstants::ShotTable::ELEVATION_MEASURED && AutoPath::isActive()) {
        ShotTableLookup::Result preset = {0.0, 0.0};
        if (ShotTableLookup::lookupTarget(
                OperatorTarget::visionIdForHole(TuningConstants::Auto::AUTO_PATH_ELEVATION_HOLE),
                TuningConstants::Auto::AUTO_PATH_ELEVATION_DISTANCE_M, preset,
                ManualControl::getBallColor())) {
            slewElevation(preset.elevationPulseUs + ManualControl::getElevationOffsetUs(), now);
        }
        ShotTableLookup::Result kept = {0.0, 0.0};
        if (noTargetShot(kept)) {
            flywheelCommand = constrain(kept.flywheelCommand + ManualControl::getFlywheelOffsetPct() / 100.0, 0.0, 1.0);
        }
    } else {
        ShotTableLookup::Result kept = {0.0, 0.0};
        if (noTargetShot(kept)) {
            flywheelCommand = constrain(kept.flywheelCommand + ManualControl::getFlywheelOffsetPct() / 100.0, 0.0, 1.0);
            if (TuningConstants::ShotTable::ELEVATION_MEASURED) {
                slewElevation(kept.elevationPulseUs + ManualControl::getElevationOffsetUs(), now);
            }
        }
    }
    lastElevationSlewMs = now;
    if (!elevationArrived) {
        elevationArrivedSinceMs = 0;
    } else if (elevationArrivedSinceMs == 0) {
        elevationArrivedSinceMs = now == 0 ? 1 : now;
    }
    const bool elevationSettled = elevationArrived &&
        (mode != OperatorMode::FULL_AUTO ||
         now - elevationArrivedSinceMs >= TuningConstants::Auto::ELEVATION_SETTLE_MS);
    if (AutoPath::holdsShooter()) flywheelCommand = 0.0;
    Shooter::setFlywheelOpenLoop(flywheelCommand);

    const bool shotAvailable = Vision::isValid() &&
        (hasTarget || (holding && Vision::getTargetId() == lastShotTargetId));
    const bool holeAllowed = mode != OperatorMode::FULL_AUTO ||
        !excludedInFullAuto(OperatorTarget::holeForVisionId(Vision::getTargetId()));
    const uint8_t aim = getAimTarget();
    if (aim != lastAimTarget) {
        lastAimTarget = aim;
        aimChangedMs = now;
    }
    const bool settledOnHole = now - aimChangedMs >= TuningConstants::Auto::AIM_SWITCH_SETTLE_MS;
    const bool towerReady = Dribbler::towerAllowsShot();
    const bool turretAimed = Shooter::isRotateReady() || ManualControl::isAimHandHeld();
    const bool ready = !AutoPath::holdsShooter() && !holeSwitchPending && turretAimed && Shooter::isFlywheelReady() && shotAvailable &&
                       elevationSettled && holeAllowed && settledOnHole && towerReady;
    bool fired = false;
    if (mode != OperatorMode::SEMI_AUTO) semiFirePending = false;
    if (firePressed && mode == OperatorMode::SEMI_AUTO) {
        semiFirePending = true;
        semiFireUntilMs = now + TuningConstants::Auto::SEMI_FIRE_HOLD_MS;
    }
    const bool fireWanted = firePressed || semiFirePending;

    if (fireState == FireState::IDLE) {
        enterFireState(FireState::SPINNING_UP);
    }
    if (fireState == FireState::COOLDOWN &&
        millis() - fireStateEnteredMs >= TuningConstants::Auto::COOLDOWN_MS) {
        enterFireState(FireState::SPINNING_UP);
    }
    if (fireState == FireState::SPINNING_UP && ready) {
        enterFireState(FireState::READY);
    }
    if (fireState == FireState::READY) {
        if (!ready) {
            enterFireState(FireState::SPINNING_UP);
        } else if ((mode == OperatorMode::FULL_AUTO && (!rowDone || litActive)) || fireWanted) {
            if (Dribbler::requestFeed()) {
                ManualControl::reportEvent(RCMAP_EVENT_FEED_TRIGGER_SENT);
                enterFireState(FireState::COOLDOWN);
                fired = true;
                semiFirePending = false;
                if (mode == OperatorMode::FULL_AUTO) countRowShot();
            } else if (fireWanted && mode != OperatorMode::FULL_AUTO) {
                ManualControl::reportEvent(RCMAP_EVENT_FEED_BUSY);
                fired = true;
                semiFirePending = false;
            }
        }
    }

    readyBits = static_cast<uint8_t>((turretAimed ? 1u : 0u) |
                                     (Shooter::isFlywheelReady() ? 2u : 0u) |
                                     (shotAvailable ? 4u : 0u) |
                                     (fireState == FireState::READY ? 8u : 0u) |
                                     (elevationSettled ? 16u : 0u) |
                                     (towerReady ? 32u : 0u));

    fireBlockers = static_cast<uint16_t>(
        (turretAimed ? 0u : BLOCK_TURRET) |
        (Shooter::isFlywheelReady() ? 0u : BLOCK_FLYWHEEL) |
        (shotAvailable ? 0u : BLOCK_NO_SHOT) |
        (elevationSettled ? 0u : BLOCK_ELEVATION) |
        (towerReady ? 0u : BLOCK_BALL) |
        (settledOnHole && !holeSwitchPending ? 0u : BLOCK_SWITCHING) |
        (AutoPath::holdsShooter() ? BLOCK_PATH : 0u) |
        (holeAllowed ? 0u : BLOCK_HOLE) |
        (fireState == FireState::COOLDOWN ? BLOCK_COOLDOWN : 0u) |
        (mode == OperatorMode::FULL_AUTO && rowDone && !litActive ? BLOCK_DONE : 0u));
    if (fired) fireBlockers = 0;

    if (semiFirePending && !fired && static_cast<long>(now - semiFireUntilMs) >= 0) {
        semiFirePending = false;
        ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
    }
}

bool Auto::noTargetShot(ShotTableLookup::Result& out) {
    const uint8_t color = ManualControl::getBallColor();
    const uint8_t aim = getAimTarget();
    if (keptFlywheel >= 0.0) {
        if (aim != 0 && aim != keptHole && keptDistanceM > 0.0 &&
            ShotTableLookup::lookupTarget(OperatorTarget::visionIdForHole(aim), keptDistanceM, out, color)) {
            return true;
        }
        out.flywheelCommand = keptFlywheel;
        out.elevationPulseUs = keptElevationUs;
        return keptElevationUs >= 0.0;
    }
    return ShotTableLookup::lookupTarget(
        OperatorTarget::visionIdForHole(TuningConstants::Auto::AUTO_PATH_ELEVATION_HOLE),
        TuningConstants::Auto::AUTO_PATH_ELEVATION_DISTANCE_M, out, color);
}

bool Auto::slewElevation(double targetUs, unsigned long now) {
    targetUs = constrain(targetUs, static_cast<double>(ShooterConstants::Angle::MIN_PULSE_US),
                         static_cast<double>(ShooterConstants::Angle::MAX_PULSE_US));
    unsigned long elapsed = now - lastElevationSlewMs;
    if (elapsed > 100) elapsed = 100;
    const double maxStep = TuningConstants::Auto::ELEVATION_SLEW_US_PER_SECOND * elapsed / 1000.0;
    const double currentUs = Shooter::getAnglePulseUs();
    const double delta = targetUs - currentUs;
    Shooter::setAnglePulseUs(delta > maxStep ? currentUs + maxStep
                             : delta < -maxStep ? currentUs - maxStep : targetUs);
    return fabs(delta) <= TuningConstants::Auto::ELEVATION_READY_TOLERANCE_US;
}

bool Auto::getTableElevationUs(double& us) {
    if (!tableElevationValid) return false;
    us = tableElevationUs;
    return true;
}
