#include "Shooter.h"
#include "../Constants/Pins.h"
#include "../Constants/PIDConfig.h"
#include "../Constants/ShooterConstants.h"
#include "../Vision/Vision.h"
#include "../Dribbler/Dribbler.h"
#include <PID_v1.h>

using namespace ShooterConst;

Servo Shooter::turretServo;
Servo Shooter::escV;
Servo Shooter::falcon;

bool Shooter::readyH = false;
bool Shooter::readyV = false;

// PID
double inH, outH, setH;
double inV, outV, setV;
double inA, outA, setA;

PID pidH(&inH,&outH,&setH,Kp1,Ki1,Kd1,DIRECT);
PID pidV(&inV,&outV,&setV,Kp2,Ki2,Kd2,DIRECT);
PID pidA(&inA,&outA,&setA,Kp3,Ki3,Kd3,DIRECT);

void Shooter::init() {
    turretServo.attach(PIN_ESC_HORI);
    escV.attach(PIN_ESC_VER);
    falcon.attach(PIN_FALCON);

    pidH.SetMode(AUTOMATIC);
    pidV.SetMode(AUTOMATIC);
    pidA.SetMode(AUTOMATIC);
    pidA.SetOutputLimits(-MG996_MAX_SPEED_OFFSET_US, MG996_MAX_SPEED_OFFSET_US);

    turretServo.writeMicroseconds(MG996_STOP_US);
}

void Shooter::update() {

    int pot = analogRead(PIN_POT);

    // auto aim
    inA = Vision::getXPred();
    setA = 0;

    pidA.Compute();

    // Ready
    if (abs(setA - inA) < MG996_AIM_DEADBAND) {
        turretServo.writeMicroseconds(MG996_STOP_US);
        readyH = true;
    } else {
        int turretPulse = MG996_STOP_US + (int)outA;
        turretPulse = constrain(turretPulse, MG996_MIN_US, MG996_MAX_US);
        turretServo.writeMicroseconds(turretPulse);
        readyH = false;
    }

    
    inV = pot;
    setV = 2000;

    pidV.Compute();
    escV.writeMicroseconds(1500 + outV);

    if (abs(setV - inV) < 40) {
        readyV = true;
    } else {
        readyV = false;
    }

    // Shooter + Dribbler
    if (readyH && readyV && Dribbler::getShootRemaining() > 0) {
        falcon.writeMicroseconds(1800); // 發射
    } else {
        falcon.writeMicroseconds(1500);
    }
}

bool Shooter::isReady() {
    return readyH && readyV;
}
