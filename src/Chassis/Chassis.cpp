#include "Chassis.h"
#include "../Constants/Pins.h"
#include "../IO/MechLink.h"

Servo Chassis::fr;
Servo Chassis::fl;
Servo Chassis::br;
Servo Chassis::bl;

void Chassis::init() {
    fr.attach(PIN_FR);
    fl.attach(PIN_FL);
    br.attach(PIN_BR);
    bl.attach(PIN_BL);
}

void Chassis::update() {
    double vx = (MechLink::ch0 - 1500) / 500.0;
    double vy = (MechLink::ch1 - 1500) / 500.0;
    double w  = (MechLink::ch3 - 1500) / 500.0;

    double FR = vx - vy - w;
    double FL = vx + vy + w;
    double BR = vx + vy - w;
    double BL = vx - vy + w;

    double maxVal = max(max(abs(FR),abs(FL)),max(abs(BR),abs(BL)));

    if(maxVal > 1){
        FR/=maxVal; FL/=maxVal; BR/=maxVal; BL/=maxVal;
    }

    fr.writeMicroseconds(1500 + FR*500);
    fl.writeMicroseconds(1500 + FL*500);
    br.writeMicroseconds(1500 + BR*500);
    bl.writeMicroseconds(1500 + BL*500);
}