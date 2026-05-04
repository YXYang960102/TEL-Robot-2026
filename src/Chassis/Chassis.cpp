#include "Chassis.h"
#include "../Constants/Pins.h"
#include "../IO/SBUS.h"

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
    int FR = SBUS::ch0 - SBUS::ch1 + SBUS::ch3;
    int BR = SBUS::ch0 - SBUS::ch1 - SBUS::ch3;
    int FL = SBUS::ch0 + SBUS::ch1 - SBUS::ch3;
    int BL = SBUS::ch0 + SBUS::ch1 + SBUS::ch3;

    fr.writeMicroseconds(FR);
    br.writeMicroseconds(BR);
    fl.writeMicroseconds(FL);
    bl.writeMicroseconds(BL);
}