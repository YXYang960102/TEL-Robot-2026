#include <Arduino.h>

#include "Auto/Auto.h"
#include "IO/MechLink.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"
#include "Chassis/Chassis.h"
#include "Telemetry/Telemetry.h"

void setup() {
    Serial.begin(115200);

    MechLink::init();
    Vision::init();
    Shooter::init();
    Dribbler::init();
    Chassis::init();
    Telemetry::init();
    Auto::init();

    // Shooting stays disarmed until the mode and shot-confirmation flow enables it.
    Dribbler::setShootRequest(0);
}

void loop() {

    MechLink::update();
    Vision::update();
    Auto::update();

    if (MechLink::isHealthy()) {
        Chassis::setOutputsEnabled(true);
        Chassis::setOpenLoop(MechLink::getDriveForward(), MechLink::getDriveTurn());
    } else {
        Chassis::setOutputsEnabled(false);
        Chassis::stop();
    }

    Shooter::update();
    Dribbler::update();

    Chassis::update();
    Telemetry::update();

    delay(10);
}
