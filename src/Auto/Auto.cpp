#include "Auto.h"

#include <Arduino.h>

#include "../Constants/TuningConstants.h"
#include "../Control/ShotDistanceFilter.h"
#include "../Control/ShotTableLookup.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/ManualControl.h"
#include "../IO/OperatorMode.h"
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

namespace {
ShotDistanceFilter<TuningConstants::Auto::DISTANCE_MEDIAN_FRAMES> distanceFilter(
    TuningConstants::Auto::DISTANCE_LATCH_BAND_M, TuningConstants::Auto::DISTANCE_LATCH_SETTLE_MS);
unsigned long lastFilterFrame = 0;

// The turret in the autos and test: the operator's hand turn (and its
// settle wait) wins; then RB's hold at 0 deg; otherwise the camera follow
// with the aim offset, or hold it with no camera.
void aimTurret(bool visionHealthy) {
    if (ManualControl::isAimJogActive()) {
        Shooter::setRotateOpenLoop(ManualControl::getAimJogCommand());
    } else if (ManualControl::isTurretHeldHome()) {
        Shooter::disableRotate(); // RB: stay at 0 deg (the guard drove it there)
    } else if (visionHealthy) {
        Shooter::setRotateVisionFollow();
        Shooter::setRotateAimOffsetPx(ManualControl::getAimOffsetPx());
    } else {
        Shooter::disableRotate();
    }
}
uint8_t filterSelection = 0;
int filterTargetId = 0;
}  // namespace

double Auto::getShotDistanceM() {
    return distanceFilter.latched();
}

uint8_t Auto::getReadyBits() {
    return readyBits;
}

void Auto::init() {
    fireState = FireState::IDLE;
    fireStateEnteredMs = millis();
    haveLastShot = false;
    distanceFilter.reset();
    readyBits = 0;
}

void Auto::enterFireState(FireState next) {
    fireState = next;
    fireStateEnteredMs = millis();
}

void Auto::update() {
    const OperatorMode mode = ManualControl::getOperatorMode();
    // getOperatorMode() is FULL_MANUAL whenever disarmed (and always on the
    // wired-manual benches), so this also covers "nobody armed".
    if (mode == OperatorMode::FULL_MANUAL) {
        ManualControl::takeFireRequest(); // never let a stale F carry over
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        distanceFilter.reset();
        readyBits = 0;
        return;
    }

    const bool visionHealthy = Vision::isConnected() && Vision::isVisionReady();
    if (mode == OperatorMode::TEST) {
        // Test mode (2026-10-26): only the turret is Auto's -- it follows
        // the selected hole (or the Orin's pick) plus the aim offset, the
        // same as in the autos. Elevation, flywheel and F stay with
        // ManualControl (F fires the tower directly there), so no shot
        // table, no fire sequence. No camera: hold the turret.
        ManualControl::takeFireRequest(); // never set in test; never carry one over
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        distanceFilter.reset();
        readyBits = 0;
        aimTurret(visionHealthy);
        return;
    }

    const bool firePressed = ManualControl::takeFireRequest();
    if (!visionHealthy) {
        // No camera: nothing to aim at. Hold the turret (the operator may
        // still turn it by hand), spin down.
        aimTurret(false);
        Shooter::setFlywheelOpenLoop(0.0);
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        distanceFilter.reset();
        readyBits = 0;
        if (firePressed) ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
        return;
    }

    aimTurret(true);

    // A target Auto may shoot at: a valid frame (with a hole selected, only
    // that hole's -- Vision drops every other id) that has a depth distance
    // and a shot table for its height. The frame's id says which hole it is
    // (the selected one, or the Orin's own pick with none selected), so the
    // table is that hole's row. valid=1 with distance 0 (the Orin's webcam
    // bench mode) or an id with no table can be aimed at but never fired at.
    //
    // The table is looked up at the STEADIED distance (2026-10-27,
    // ShotDistanceFilter): each new depth frame of this target feeds it,
    // and its latched value only moves for a real move, so standing still
    // the flywheel command and the elevation target stay put and the
    // shooter can count as ready. A new hole (selection, or the Orin's own
    // pick changing) starts the filter over.
    const unsigned long now = millis();
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
    ShotTableLookup::Result shot = {0.0, 0.0};
    const bool hasTarget = frameHasDepth && distanceFilter.latched() > 0.0 &&
        ShotTableLookup::lookupTarget(Vision::getTargetId(), distanceFilter.latched(), shot);

    // Target hold (2026-10-18): a single invalid / distance-0 / other-hole
    // frame used to drop the flywheel to the pre-spin for ~90 ms and back,
    // pulsing it and restarting its spin-up wait. The last target's shot
    // is kept for TARGET_HOLD_MS (not across a new hole selection). Firing
    // needs the hole in the current frame (Vision::isValid(): the turret is
    // locked on it) and a shot: this frame's, or the held one when only the
    // depth dropped out for a moment (same hole).
    if (hasTarget) {
        haveLastShot = true;
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

    // Flywheel from that table when there is (or was just) a target
    // (distance in mm -> m); otherwise the fixed SHOOT_COMMAND so it is
    // already spinning when a target appears.
    double flywheelCommand = TuningConstants::Flywheel::SHOOT_COMMAND;
    // Elevation at its table pulse (2026-10-31): true with no measured
    // table (the operator holds the elevation then).
    bool elevationArrived = !TuningConstants::ShotTable::ELEVATION_MEASURED;
    if (hasTarget || holding) {
        // Plus the operator's LT / RT offset (percent), kept within 0..1.
        flywheelCommand = constrain(
            lastShotFlywheel + ManualControl::getFlywheelOffsetPct() / 100.0, 0.0, 1.0);
        // Elevation from the table too (a measured servo pulse) plus the
        // operator's fine-adjust, once the table holds real measured pulses
        // (ShotTable::ELEVATION_MEASURED); until then the operator keeps
        // the elevation. No target: hold.
        if (TuningConstants::ShotTable::ELEVATION_MEASURED) {
            // Toward the target at ELEVATION_SLEW_US_PER_SECOND, not a jump
            // (current limiting, 2026-10-18).
            const double targetUs = lastShotElevationUs + ManualControl::getElevationOffsetUs();
            unsigned long elapsed = now - lastElevationSlewMs;
            if (elapsed > 100) elapsed = 100;
            const double maxStep = TuningConstants::Auto::ELEVATION_SLEW_US_PER_SECOND * elapsed / 1000.0;
            const double currentUs = Shooter::getAnglePulseUs();
            const double delta = targetUs - currentUs;
            // Where it is now (before this loop's step): one loop late at
            // most, never early.
            elevationArrived = fabs(delta) <= TuningConstants::Auto::ELEVATION_READY_TOLERANCE_US;
            Shooter::setAnglePulseUs(delta > maxStep ? currentUs + maxStep
                                     : delta < -maxStep ? currentUs - maxStep : targetUs);
        }
    }
    lastElevationSlewMs = now;
    Shooter::setFlywheelOpenLoop(flywheelCommand);

    // Turret locked on the target (|tx + aim offset| <= 5 px for 100 ms),
    // flywheel commanded at shoot speed long enough to be spun up, a shot
    // for the hole in this frame (its own, or the held one), and the
    // elevation at that shot's pulse.
    const bool shotAvailable = Vision::isValid() &&
        (hasTarget || (holding && Vision::getTargetId() == lastShotTargetId));
    const bool ready = Shooter::isReady() && shotAvailable && elevationArrived;
    bool fired = false;

    if (fireState == FireState::IDLE) {
        enterFireState(FireState::SPINNING_UP);
    }
    if (fireState == FireState::COOLDOWN &&
        millis() - fireStateEnteredMs >= TuningConstants::Auto::COOLDOWN_MS) {
        enterFireState(FireState::SPINNING_UP);
    }
    if (fireState == FireState::SPINNING_UP && ready) {
        enterFireState(FireState::READY); // a ready shooter can fire this same loop
    }
    if (fireState == FireState::READY) {
        if (!ready) {
            enterFireState(FireState::SPINNING_UP); // lost lock or spun down
        } else if (mode == OperatorMode::FULL_AUTO || firePressed) {
            if (Dribbler::requestFeed()) {
                ManualControl::reportEvent(RCMAP_EVENT_FEED_TRIGGER_SENT);
                enterFireState(FireState::COOLDOWN);
                fired = true;
            } else if (firePressed) {
                // Tower still in its own lockout. Full-auto just retries next
                // loop without reporting every attempt.
                ManualControl::reportEvent(RCMAP_EVENT_FEED_BUSY);
                fired = true; // the press was answered
            }
        }
    }

    // Why it can't fire yet, for the Dashboard (TEL slot 40).
    readyBits = static_cast<uint8_t>((Shooter::isRotateReady() ? 1u : 0u) |
                                     (Shooter::isFlywheelReady() ? 2u : 0u) |
                                     (shotAvailable ? 4u : 0u) |
                                     (fireState == FireState::READY ? 8u : 0u) |
                                     (elevationArrived ? 16u : 0u));

    // Semi-auto F while not ready (still aiming / spinning up / cooling
    // down) is refused, not queued: the operator presses again.
    if (firePressed && !fired && mode == OperatorMode::SEMI_AUTO) {
        ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
    }
}
