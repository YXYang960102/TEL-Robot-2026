// Fake Arduino core + recording stand-ins for the subsystems ManualControl
// and Auto drive (Chassis/Shooter/Vision) and for MechLink (the test injects
// RC frames directly and reads back status lines). Dribbler and Auto are NOT
// stubbed: the real src/Dribbler/Dribbler.cpp and src/Auto/Auto.cpp run
// against the fake pins here.
#include "host_stubs.h"

#include "../../src/Chassis/Chassis.h"
#include "../../src/IO/MechLink.h"
#include "../../src/Shooter/Shooter.h"
#include "../../src/Vision/Vision.h"

namespace hoststub {
unsigned long nowMs = 0;
int pinLevel[128] = {};
int digitalOut[128] = {};
int analogOut[128] = {};
Recorder rec;
bool visionReady = false;
bool visionValid = false;
double visionDistanceMm = 0.0;
bool shooterReady = false;
}

HostSerial Serial;

unsigned long millis() { return hoststub::nowMs; }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value) { hoststub::digitalOut[pin] = value; }
int digitalRead(uint8_t pin) { return hoststub::pinLevel[pin]; }
void analogWrite(uint8_t pin, int value) { hoststub::analogOut[pin] = value; }

using hoststub::rec;

void Chassis::setOutputsEnabled(bool enabled) {
    rec.chassisEnabled = enabled;
    if (!enabled) { rec.chassisLeft = 0.0; rec.chassisRight = 0.0; }
}
void Chassis::setTankOpenLoop(double left, double right) {
    rec.chassisLeft = left;
    rec.chassisRight = right;
}

void Shooter::setOutputsEnabled(bool enabled) {
    rec.shooterEnabled = enabled;
    if (!enabled) stopAll();
}
void Shooter::stopAll() {
    rec.rotateMode = "disabled";
    rec.rotateCommand = 0.0;
    rec.angleCommand = 0.0;
    rec.flywheelCommand = 0.0;
}
void Shooter::setRotateOpenLoop(double command) {
    rec.rotateMode = "manual";
    rec.rotateCommand = command;
}
void Shooter::setRotateVisionFollow() { rec.rotateMode = "vision"; }
void Shooter::disableRotate() {
    rec.rotateMode = "disabled";
    rec.rotateCommand = 0.0;
}
void Shooter::setAngleOpenLoop(double command) { rec.angleCommand = command; }
bool Shooter::setAngleTargetDegrees(double degrees) {
    rec.angleTargetDegrees = degrees;
    rec.angleTargetSets++;
    return true;
}
bool Shooter::setAnglePulseUs(double pulseUs) {
    rec.anglePulseUs = pulseUs;
    rec.anglePulseSets++;
    return true;
}
void Shooter::setFlywheelOpenLoop(double command) { rec.flywheelCommand = command; }

bool Shooter::isReady() { return hoststub::shooterReady; }

bool Vision::isConnected() { return hoststub::visionReady; }
bool Vision::isVisionReady() { return hoststub::visionReady; }
bool Vision::isValid() { return hoststub::visionValid; }
double Vision::getDistance() { return hoststub::visionDistanceMm; }

int MechLink::channels[RCMAP_CHANNEL_COUNT];
unsigned long MechLink::frameCount = 0;
bool MechLink::signalLost = true;
int MechLink::modeChannel = RCMAP_MODE_FULL_MANUAL_US;

void MechLink::sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode) {
    rec.statusCount++;
    rec.statusFlags = flags;
    rec.statusSeq = eventSeq;
    rec.statusCode = eventCode;
}
