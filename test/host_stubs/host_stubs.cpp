// Fake Arduino core + recording stand-ins for the subsystems ManualControl
// drives (Chassis/Shooter/Vision) and for MechLink (the test injects RC
// frames directly and reads back status lines). Dribbler is NOT stubbed:
// the real src/Dribbler/Dribbler.cpp runs against the fake pins here.
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
void Shooter::setFlywheelOpenLoop(double command) { rec.flywheelCommand = command; }

bool Vision::isConnected() { return hoststub::visionReady; }
bool Vision::isVisionReady() { return hoststub::visionReady; }

int MechLink::channels[RCMAP_CHANNEL_COUNT];
unsigned long MechLink::frameCount = 0;
bool MechLink::signalLost = true;
int MechLink::modeChannel = RCMAP_MODE_FULL_MANUAL_US;
int MechLink::fireChannel = 1000;

void MechLink::sendStatus(uint8_t flags, unsigned long eventSeq, uint16_t eventCode) {
    rec.statusCount++;
    rec.statusFlags = flags;
    rec.statusSeq = eventSeq;
    rec.statusCode = eventCode;
}
