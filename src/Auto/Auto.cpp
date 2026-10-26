#include "Auto.h"

#include <Arduino.h>

#include "../Constants/TuningConstants.h"
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

void Auto::init() {
    fireState = FireState::IDLE;
    fireStateEnteredMs = millis();
    haveLastShot = false;
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
        if (visionHealthy) {
            Shooter::setRotateVisionFollow();
            Shooter::setRotateAimOffsetPx(ManualControl::getAimOffsetPx());
        } else {
            Shooter::disableRotate();
        }
        return;
    }

    const bool firePressed = ManualControl::takeFireRequest();
    if (!visionHealthy) {
        // No camera: nothing to aim at. Hold the turret, spin down.
        Shooter::disableRotate();
        Shooter::setFlywheelOpenLoop(0.0);
        enterFireState(FireState::IDLE);
        haveLastShot = false;
        if (firePressed) ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
        return;
    }

    Shooter::setRotateVisionFollow();
    Shooter::setRotateAimOffsetPx(ManualControl::getAimOffsetPx());

    // A target Auto may shoot at: a valid frame (with a hole selected, only
    // that hole's -- Vision drops every other id) that has a depth distance
    // and a shot table for its height. The frame's id says which hole it is
    // (the selected one, or the Orin's own pick with none selected), so the
    // table is that hole's row. valid=1 with distance 0 (the Orin's webcam
    // bench mode) or an id with no table can be aimed at but never fired at.
    ShotTableLookup::Result shot = {0.0, 0.0};
    const bool hasTarget = Vision::isValid() && Vision::getDistance() > 0.0 &&
        ShotTableLookup::lookupTarget(Vision::getTargetId(), Vision::getDistance() / 1000.0, shot);

    // Target hold (2026-10-18): a single invalid / distance-0 / other-hole
    // frame used to drop the flywheel to the pre-spin for ~90 ms and back,
    // pulsing it and restarting its spin-up wait. The last target's shot
    // is kept for TARGET_HOLD_MS (not across a new hole selection); firing
    // still needs a target in the current frame (hasTarget below).
    const unsigned long now = millis();
    const uint8_t selection = Vision::getSelectedTarget();
    if (hasTarget) {
        haveLastShot = true;
        lastShotFlywheel = shot.flywheelCommand;
        lastShotElevationUs = shot.elevationPulseUs;
        lastShotMs = now;
        lastShotSelection = selection;
    }
    const bool holding = !hasTarget && haveLastShot && selection == lastShotSelection &&
        now - lastShotMs <= TuningConstants::Auto::TARGET_HOLD_MS;
    if (!hasTarget && !holding) haveLastShot = false;

    // Flywheel from that table when there is (or was just) a target
    // (distance in mm -> m); otherwise the fixed SHOOT_COMMAND so it is
    // already spinning when a target appears.
    double flywheelCommand = TuningConstants::Flywheel::SHOOT_COMMAND;
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
            Shooter::setAnglePulseUs(delta > maxStep ? currentUs + maxStep
                                     : delta < -maxStep ? currentUs - maxStep : targetUs);
        }
    }
    lastElevationSlewMs = now;
    Shooter::setFlywheelOpenLoop(flywheelCommand);

    // Turret locked on the target (|tx + aim offset| <= 5 px for 100 ms),
    // flywheel commanded at shoot speed long enough to be spun up, and a
    // target with a distance.
    const bool ready = Shooter::isReady() && hasTarget;
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

    // Semi-auto F while not ready (still aiming / spinning up / cooling
    // down) is refused, not queued: the operator presses again.
    if (firePressed && !fired && mode == OperatorMode::SEMI_AUTO) {
        ManualControl::reportEvent(RCMAP_EVENT_FIRE_NOT_READY);
    }
}
