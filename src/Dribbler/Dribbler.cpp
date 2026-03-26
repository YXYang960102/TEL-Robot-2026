#include "Dribbler.h"
#include "../Constants/RobotConstants.h"

void Dribbler::begin() {
    using namespace RobotConfig::Dribbler;
    
    motor.attach(PIN_MOTOR);
    sensorPin = PIN_SENSOR;
    pinMode(sensorPin, INPUT);
    
    stop();
}

void Dribbler::update(bool manualIntake, bool manualOuttake) {
    using namespace RobotConfig::Dribbler;

    if (manualIntake) {
        intake();
    } 
    else if (manualOuttake) {
        outtake();
    }
    else {
        if (hasBall()) {
            stop(); 
        } else {
            stop();
        }
    }
}

bool Dribbler::hasBall() {
    return (digitalRead(sensorPin) == LOW); 
}

void Dribbler::intake() {
    motor.writeMicroseconds(RobotConfig::Dribbler::SPEED_INTAKE);
}

void Dribbler::outtake() {
    motor.writeMicroseconds(RobotConfig::Dribbler::SPEED_OUT);
}

void Dribbler::stop() {
    motor.writeMicroseconds(RobotConfig::Dribbler::SPEED_STOP);
}