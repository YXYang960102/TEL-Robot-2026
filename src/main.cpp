#include <Arduino.h>

#include "Auto/Auto.h"
#include "IO/SBUS.h"
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
    Auto::init();

    // Shooting stays disarmed until something actually requests shots.
    // Previously defaulted to 3 -- now that Auto::update() auto-fires
    // whenever a shot is queued and the axis is locked+spun-up, leaving
    // this nonzero would auto-fire on every boot with no operator action.
    Dribbler::setShootRequest(0);
}

void loop() {

    SBUS::update();
    Vision::update();

    // Rotate mode arbitration and the auto-fire sequence both live in
    // Auto::update() now (moved out of here 2026-10-06) so "what mode is
    // each subsystem in" is decided in one place.
    Auto::update();

    Shooter::update();
    Dribbler::update();

    Chassis::update();

    delay(10);
}
