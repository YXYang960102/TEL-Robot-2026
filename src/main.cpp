#include <Arduino.h>

#include "IO/SBUS.h"
#include "IO/OperatorMode.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"
#include "Chassis/Chassis.h"

void setup() {
    Serial.begin(115200);

    SBUS::init();
    Vision::init();
    Shooter::init();
    Dribbler::init();
    Chassis::init();

    Dribbler::setShootRequest(3);
}

void loop() {

    SBUS::update();
    Vision::update();

    // Full-auto and semi-auto both auto-aim (they only differ once a
    // fire-confirmation step exists, which isn't built yet) -> VISION_FOLLOW
    // for both. Full-manual, or a vision link that isn't actually healthy,
    // disables the axis: no manual joystick channel is wired to Rotate yet
    // (the operator keypad doesn't exist), so defaulting to a safe disable
    // is the conservative choice rather than guessing an unreviewed channel.
    const OperatorMode mode = OperatorModeSelector::current();
    const bool visionHealthy = Vision::isConnected() && Vision::isVisionReady();
    if (mode == OperatorMode::FULL_MANUAL || !visionHealthy) {
        Shooter::disableRotate();
    } else {
        Shooter::setRotateVisionFollow();
    }

    Shooter::update();
    Dribbler::update();

    Chassis::update();

    delay(10);
}