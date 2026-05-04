#include <Arduino.h>

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

    Dribbler::setShootRequest(3); 
}

void loop() {

    SBUS::update();
    Vision::update();

    Shooter::update();   
    Dribbler::update();  

    Chassis::update();

    delay(10);
}