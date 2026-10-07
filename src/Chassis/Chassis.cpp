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
    int FR = MechLink::ch0 - MechLink::ch1 + MechLink::ch3;
    int BR = MechLink::ch0 - MechLink::ch1 - MechLink::ch3;
    int FL = MechLink::ch0 + MechLink::ch1 - MechLink::ch3;
    int BL = MechLink::ch0 + MechLink::ch1 + MechLink::ch3;

    fr.writeMicroseconds(FR);
    br.writeMicroseconds(BR);
    fl.writeMicroseconds(FL);
    bl.writeMicroseconds(BL);
}