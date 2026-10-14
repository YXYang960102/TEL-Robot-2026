#include "Dribbler.h"

#include "../Constants/BuildProfile.h"

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
    if (BuildProfile::DRIVES_FEED_TOWER) {
        pinMode(Tower::TRIGGER_PIN, OUTPUT);
    }
    writeLine(false);
    pulseActive = false;
    everTriggered = false;
    triggerCount = 0;
}

void Dribbler::update() {
    if (pulseActive && millis() - pulseStartMs >= Tower::PULSE_MS) {
        writeLine(false);
        pulseActive = false;
    }
}

void Dribbler::stop() {
    setConveyor(false);
    writeLine(false);
    pulseActive = false;
}

bool Dribbler::requestFeed() {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return false; // Tower not wired on this bench.
    }
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

void Dribbler::writeLine(bool active) {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return;
    }
    digitalWrite(Tower::TRIGGER_PIN, active == Tower::TRIGGER_ACTIVE_HIGH ? HIGH : LOW);
}
