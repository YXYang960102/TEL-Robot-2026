#include "Dribbler.h"

using namespace DribblerConstants;

Servo Dribbler::conveyorEsc;
bool Dribbler::conveyorOn = false;

void Dribbler::setConveyor(bool on) {
    conveyorOn = on;
    if (!Conveyor::CONFIGURED || !conveyorEsc.attached()) return;
    conveyorEsc.writeMicroseconds(on ? Conveyor::RUN_PULSE_US : Conveyor::STOP_PULSE_US);
}

bool Dribbler::isConveyorOn() { return conveyorOn; }

bool Dribbler::pulseActive = false;
bool Dribbler::everTriggered = false;
unsigned long Dribbler::pulseStartMs = 0;
unsigned long Dribbler::triggerCount = 0;
int Dribbler::shootRemaining = 0;
bool Dribbler::feedAllowed = false;

void Dribbler::init() {
    setConveyor(false);
    if (Conveyor::CONFIGURED) {
        conveyorEsc.attach(Conveyor::SIGNAL_PIN);
        setConveyor(false);
    }
    // Drive the line inactive before switching the pin to output, so it
    // never glitches active during boot. (Before this runs the pin floats:
    // the tower side should have a pull-down.)
    writeLine(false);
#ifndef TEL_DRIVE_CONVEYOR_ONLY
    pinMode(Tower::TRIGGER_PIN, OUTPUT);
#endif
    writeLine(false);
    pulseActive = false;
    everTriggered = false;
    triggerCount = 0;
    shootRemaining = 0;
    feedAllowed = false;
}

void Dribbler::update() {
    if (pulseActive && millis() - pulseStartMs >= Tower::PULSE_MS) {
        writeLine(false);
        pulseActive = false;
    }

    if (feedAllowed && shootRemaining > 0 && !isBusy()) {
        if (requestFeed()) {
            shootRemaining--;
        }
    }
}

void Dribbler::stop() {
    setConveyor(false);
    writeLine(false);
    pulseActive = false;
    shootRemaining = 0;
}

bool Dribbler::requestFeed() {
#ifdef TEL_DRIVE_CONVEYOR_ONLY
    return false; // Tower not configured for this bench.
#endif
    if (isBusy()) {
        return false;
    }
    pulseStartMs = millis();
    pulseActive = true;
    everTriggered = true;
    triggerCount++;
    writeLine(true);
    return true;
}

bool Dribbler::isPulseActive() {
    return pulseActive;
}

bool Dribbler::isBusy() {
    if (pulseActive) {
        return true;
    }
    return everTriggered && millis() - pulseStartMs < Tower::COOLDOWN_MS;
}

unsigned long Dribbler::getTriggerCount() {
    return triggerCount;
}

void Dribbler::setShootRequest(int count) {
    shootRemaining = constrain(count, Request::MIN_COUNT, Request::MAX_COUNT);
}

int Dribbler::getShootRemaining() {
    return shootRemaining;
}

void Dribbler::setFeedAllowed(bool allowed) {
    feedAllowed = allowed;
}

void Dribbler::writeLine(bool active) {
#ifdef TEL_DRIVE_CONVEYOR_ONLY
    (void)active;
    return;
#endif
    digitalWrite(Tower::TRIGGER_PIN, active == Tower::TRIGGER_ACTIVE_HIGH ? HIGH : LOW);
}
