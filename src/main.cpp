#include <Arduino.h>

#include "IO/SBUS.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"
#include "Chassis/Chassis.h"
#include "Telemetry/Telemetry.h"

void setup() {
    Serial.begin(115200);

    SBUS::init();
    Vision::init();
    Shooter::init();
    Dribbler::init();
    Chassis::init();
    Telemetry::init();

    // Shooting stays disarmed until the mode and shot-confirmation flow enables it.
    Dribbler::setShootRequest(0);
}

void loop() {

    SBUS::update();
    Vision::update();

    if (SBUS::isHealthy()) {
        Chassis::setOpenLoop(SBUS::getDriveForward(), SBUS::getDriveTurn());
    } else {
        Chassis::stop();
    }

    Shooter::update();
    Dribbler::update();  

    Chassis::update();
    Telemetry::update();

    delay(10);
}
