#include "Chassis.h"
#include "../Constants/Pins.h"
#include "../IO/SBUS.h"

Servo Chassis::fr;
Servo Chassis::fl;

void Chassis::init() {
    fr.attach(PIN_FR);
    fl.attach(PIN_FL);
}

void Chassis::update() {
    double vx = (SBUS::ch0 - 1500) / 500.0;
    double w  = (SBUS::ch3 - 1500) / 500.0;

    double FR = vx - w;
    double FL = vx + w;

    double maxVal = max(abs(FR), abs(FL));

    if(maxVal > 1){
        FR/=maxVal; FL/=maxVal;
    }

    fr.writeMicroseconds(1500 + FR*500);
    fl.writeMicroseconds(1500 + FL*500);
}