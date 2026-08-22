#include "Shooter.h"
#include "../Constants/Pins.h"
#include "../Constants/PIDConfig.h"
#include "../Vision/Vision.h"
#include "../Dribbler/Dribbler.h"
#include <PID_v1.h>

Servo Shooter::escH;
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
    escH.attach(PIN_ESC_HORI);
    escV.attach(PIN_ESC_VER);
    falcon.attach(PIN_FALCON);

    pidH.SetMode(AUTOMATIC);
    pidV.SetMode(AUTOMATIC);
    pidA.SetMode(AUTOMATIC);
}

void Shooter::update() {
    // Boot-safe gate: without a fresh Orin VISION_READY, hold every output
    // neutral instead of running the PID loops. The vertical PID's fixed
    // setV=2000 target is unreachable by the 0-1023 ADC input, so left
    // unconditional it saturates its output (and moves escV) from the very
    // first loop() iteration, independent of SBUS/vision validity.
    if (!Vision::isVisionReady()) {
        escH.writeMicroseconds(1500);
        escV.writeMicroseconds(1500);
        falcon.writeMicroseconds(1500);
        readyH = false;
        readyV = false;
        return;
    }

    int pot = analogRead(PIN_POT);

    // auto aim
    if (Vision::isValid()) {
        inA = Vision::getXPred();
        setA = 0;

        pidA.Compute();
        escH.writeMicroseconds(1500 + outA);
        readyH = abs(setA - inA) < 10;
    } else {
        escH.writeMicroseconds(1500);
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
    if (Vision::isValid() && readyH && readyV && Dribbler::getShootRemaining() > 0) {
        falcon.writeMicroseconds(1800); // 發射
    } else {
        falcon.writeMicroseconds(1500);
    }
}

bool Shooter::isReady() {
    return readyH && readyV;
}
