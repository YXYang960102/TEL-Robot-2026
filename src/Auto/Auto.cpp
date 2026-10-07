#include "Auto.h"

#include <Arduino.h>

#include "../Constants/TuningConstants.h"
#include "../Control/ShotTableLookup.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
#include "../IO/OperatorMode.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"

Auto::FireState Auto::fireState = Auto::FireState::IDLE;
unsigned long Auto::fireStateEnteredMs = 0;
bool Auto::fireButtonWasDown = false;

void Auto::init() {
    fireState = FireState::IDLE;
    fireStateEnteredMs = millis();
    fireButtonWasDown = false;
    Dribbler::setFeedAllowed(false);
}

void Auto::enterFireState(FireState next) {
    fireState = next;
    fireStateEnteredMs = millis();
}

Auto::FireState Auto::getFireState() {
    return fireState;
}

void Auto::update() {
    const OperatorMode mode = OperatorModeSelector::current();
    const bool visionHealthy = Vision::isConnected() && Vision::isVisionReady();

    // Rotate: full-auto and semi-auto both auto-aim continuously (they only
    // differ on whether firing needs a button press, handled below).
    // Full-manual, or a vision link that isn't actually healthy, disables
    // the axis -- no manual joystick channel is wired to Rotate yet (the
    // operator keypad doesn't exist), so a safe disable is the conservative
    // choice over guessing an unreviewed channel.
    if (mode == OperatorMode::FULL_MANUAL || !visionHealthy) {
        Shooter::disableRotate();
    } else {
        Shooter::setRotateVisionFollow();
    }

    // Momentary fire-confirm button (semi-auto only), edge-detected so a
    // held button fires once, not every loop. Provisional channel/threshold,
    // same caveat as OperatorMode's mode channel -- the real keypad doesn't
    // exist yet.
    const bool fireButtonDown = MechLink::fireChannel >= TuningConstants::Auto::FIRE_BUTTON_THRESHOLD;
    const bool fireButtonPressed = fireButtonDown && !fireButtonWasDown;
    fireButtonWasDown = fireButtonDown;

    const bool wantsToShoot = Dribbler::getShootRemaining() > 0;
    // Deliberately NOT gated on Shooter::isAngleReady() -- elevation
    // closed-loop can never arm yet (Angle::POSITION_LIMITS_CALIBRATED is
    // hardcoded false pending real mechanical-limit calibration). Wire
    // isAngleReady() in here once that calibration is done; see
    // docs/codex-handoff.md 2026-10-06.
    const bool canAutoFire =
        mode != OperatorMode::FULL_MANUAL && visionHealthy && wantsToShoot;

    if (!canAutoFire) {
        Dribbler::setFeedAllowed(false);
        Shooter::setFlywheelOpenLoop(0.0);
        enterFireState(FireState::IDLE);
        return;
    }

    // Distance-based shot table lookup, computed once per loop instead of
    // each state case re-deriving the same fixed SHOOT_COMMAND. Falls back
    // to the fixed command when the camera isn't actually locked onto a
    // valid target (canAutoFire only requires visionHealthy == the Orin
    // link being alive, not that it currently sees a target). angleDegrees
    // is computed but intentionally not driven anywhere yet -- Angle's
    // closed-loop calibration isn't done, see the comment above canAutoFire.
    double flywheelCommand = TuningConstants::Flywheel::SHOOT_COMMAND;
    if (Vision::isValid()) {
        const ShotTableLookup::Result shot = ShotTableLookup::lookup(
            Vision::getDistance() / 1000.0,  // mm -> m
            TuningConstants::ShotTable::TABLE,
            TuningConstants::ShotTable::TABLE_COUNT);
        flywheelCommand = shot.flywheelCommand;
    }

    switch (fireState) {
        case FireState::IDLE:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(flywheelCommand);
            enterFireState(FireState::SPINNING_UP);
            break;

        case FireState::SPINNING_UP:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(flywheelCommand);
            if (Shooter::isRotateReady() && Shooter::isFlywheelReady()) {
                enterFireState(FireState::READY_TO_FIRE);
            }
            break;

        case FireState::READY_TO_FIRE:
            Shooter::setFlywheelOpenLoop(flywheelCommand);
            if (!Shooter::isRotateReady() || !Shooter::isFlywheelReady()) {
                // Lost lock or spun down -- go back and wait again.
                enterFireState(FireState::SPINNING_UP);
                break;
            }
            if (mode == OperatorMode::FULL_AUTO || fireButtonPressed) {
                Dribbler::setFeedAllowed(true);
                enterFireState(FireState::FEEDING);
            }
            break;

        case FireState::FEEDING:
            Shooter::setFlywheelOpenLoop(flywheelCommand);
            if (millis() - fireStateEnteredMs >= TuningConstants::Auto::FEED_DURATION_MS) {
                Dribbler::setFeedAllowed(false);
                enterFireState(FireState::COOLDOWN);
            }
            break;

        case FireState::COOLDOWN:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(flywheelCommand);
            if (millis() - fireStateEnteredMs >= TuningConstants::Auto::COOLDOWN_MS) {
                enterFireState(FireState::IDLE);
            }
            break;
    }
}
