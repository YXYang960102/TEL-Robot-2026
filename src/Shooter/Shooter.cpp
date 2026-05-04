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

    int pot = analogRead(PIN_POT);

    // ===== 水平自動 =====
    inA = Vision::getXPred();
    setA = 0;

    pidA.Compute();
    escH.writeMicroseconds(1500 + outA);

    // ===== Ready判定 =====
    if (abs(setA - inA) < 10) {
        readyH = true;
    } else {
        readyH = false;
    }

    // ===== 垂直 =====
    inV = pot;
    setV = 2000;

    pidV.Compute();
    escV.writeMicroseconds(1500 + outV);

    if (abs(setV - inV) < 40) {
        readyV = true;
    } else {
        readyV = false;
    }

    // ===== Shooter + Dribbler同步 =====
    if (readyH && readyV && Dribbler::getShootRemaining() > 0) {
        falcon.writeMicroseconds(1800); // 發射
    } else {
        falcon.writeMicroseconds(1500);
    }
}

bool Shooter::isReady() {
    return readyH && readyV;
}