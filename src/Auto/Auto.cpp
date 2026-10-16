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

void Auto::init() {
    fireState = FireState::IDLE;
    fireStateEnteredMs = millis();
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
        return;
    }

    const bool firePressed = ManualControl::takeFireRequest();
    const bool visionHealthy = Vision::isConnected() && Vision::isVisionReady();
    if (!visionHealthy) {
        // No camera: nothing to aim at. Hold the turret, spin down.
        Shooter::disableRotate();
        Shooter::setFlywheelOpenLoop(0.0);
        enterFireState(FireState::IDLE);
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

    // Flywheel from that table when there is a target (distance in mm -> m);
    // otherwise the fixed SHOOT_COMMAND so it is already spinning when a
    // target appears.
    double flywheelCommand = TuningConstants::Flywheel::SHOOT_COMMAND;
    if (hasTarget) {
        flywheelCommand = shot.flywheelCommand;
        // Elevation from the table too (a measured servo pulse) plus the
        // operator's fine-adjust, once the table holds real measured pulses
        // (ShotTable::ELEVATION_MEASURED); until then the operator keeps
        // the elevation. No target: hold.
        if (TuningConstants::ShotTable::ELEVATION_MEASURED) {
            Shooter::setAnglePulseUs(shot.elevationPulseUs + ManualControl::getElevationOffsetUs());
        }
    }
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
