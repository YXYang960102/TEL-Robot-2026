#include "Chassis.h"
#include "../Constants/Pins.h"
#include "../Constants/ChassisConstants.h"

#include <Arduino.h>

Servo Chassis::fr;
Servo Chassis::fl;
double Chassis::forwardCommand = 0.0;
double Chassis::turnCommand = 0.0;

void Chassis::init() {
    fr.attach(PIN_FR);
    fl.attach(PIN_FL);
    stop();
}

void Chassis::update() {
    double FR = forwardCommand - turnCommand;
    double FL = forwardCommand + turnCommand;

    double maxVal = max(abs(FR), abs(FL));

    if(maxVal > 1){
        FR/=maxVal; FL/=maxVal;
    }

    fr.writeMicroseconds(constrain(
        ChassisConst::ESC_STOP_US + static_cast<int>(FR * ChassisConst::ESC_RANGE_US),
        ChassisConst::ESC_MIN_US, ChassisConst::ESC_MAX_US));
    fl.writeMicroseconds(constrain(
        ChassisConst::ESC_STOP_US + static_cast<int>(FL * ChassisConst::ESC_RANGE_US),
        ChassisConst::ESC_MIN_US, ChassisConst::ESC_MAX_US));
}

void Chassis::setDriveCommand(double forward, double turn) {
    forwardCommand = constrain(forward, -1.0, 1.0);
    turnCommand = constrain(turn, -1.0, 1.0);

    if (abs(forwardCommand) < ChassisConst::COMMAND_DEADBAND) {
        forwardCommand = 0.0;
    }
    if (abs(turnCommand) < ChassisConst::COMMAND_DEADBAND) {
        turnCommand = 0.0;
    }
}

void Chassis::stop() {
    forwardCommand = 0.0;
    turnCommand = 0.0;

    if (fr.attached()) {
        fr.writeMicroseconds(ChassisConst::ESC_STOP_US);
    }
    if (fl.attached()) {
        fl.writeMicroseconds(ChassisConst::ESC_STOP_US);
    }
}
