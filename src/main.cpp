#include <Arduino.h>

#include "IO/MechLink.h"
#include "Vision/Vision.h"
#include "Shooter/Shooter.h"
#include "Dribbler/Dribbler.h"
#include "Chassis/Chassis.h"

void setup() {
    Serial.begin(115200);

    MechLink::init();
    Vision::init();
    Shooter::init();
    Dribbler::init();
    Chassis::init();

    // Shooting stays disarmed until the mode and shot-confirmation flow enables it.
    Dribbler::setShootRequest(0);
}

void loop() {

    MechLink::update();
    Vision::update();

    if (MechLink::isHealthy()) {
        Chassis::setDriveCommand(MechLink::getDriveForward(), MechLink::getDriveTurn());
    } else {
        Chassis::stop();
    }

    Shooter::update();
    Dribbler::update();

    Chassis::update();

    delay(10);
}
