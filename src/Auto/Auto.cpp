#include "Auto.h"

#include <Arduino.h>

#include "../Constants/AutoConstants.h"
#include "../Constants/ShooterConstants.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/OperatorMode.h"
#include "../IO/SBUS.h"
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
    const bool fireButtonDown = SBUS::fireChannel >= AutoConstants::FIRE_BUTTON_THRESHOLD;
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

    switch (fireState) {
        case FireState::IDLE:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(ShooterConstants::Flywheel::SHOOT_COMMAND);
            enterFireState(FireState::SPINNING_UP);
            break;

        case FireState::SPINNING_UP:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(ShooterConstants::Flywheel::SHOOT_COMMAND);
            if (Shooter::isRotateReady() && Shooter::isFlywheelReady()) {
                enterFireState(FireState::READY_TO_FIRE);
            }
            break;

        case FireState::READY_TO_FIRE:
            Shooter::setFlywheelOpenLoop(ShooterConstants::Flywheel::SHOOT_COMMAND);
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
            Shooter::setFlywheelOpenLoop(ShooterConstants::Flywheel::SHOOT_COMMAND);
            if (millis() - fireStateEnteredMs >= AutoConstants::FEED_DURATION_MS) {
                Dribbler::setFeedAllowed(false);
                enterFireState(FireState::COOLDOWN);
            }
            break;

        case FireState::COOLDOWN:
            Dribbler::setFeedAllowed(false);
            Shooter::setFlywheelOpenLoop(ShooterConstants::Flywheel::SHOOT_COMMAND);
            if (millis() - fireStateEnteredMs >= AutoConstants::COOLDOWN_MS) {
                enterFireState(FireState::IDLE);
            }
            break;
    }
}
