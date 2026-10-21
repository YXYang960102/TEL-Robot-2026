#include "Dribbler.h"

#include "../Constants/BuildProfile.h"

using namespace DribblerConstants;

bool Dribbler::enabled = false;
bool Dribbler::pulseActive = false;
bool Dribbler::everTriggered = false;
unsigned long Dribbler::pulseStartMs = 0;
unsigned long Dribbler::triggerCount = 0;

void Dribbler::init() {
    // Boot = disabled: the line is held HIGH (DribblerConstants.h). Preload
    // the level before switching the pin to output. (Before this runs the
    // pin floats.)
    enabled = false;
    writeLine(true);
    if (BuildProfile::DRIVES_FEED_TOWER) {
        pinMode(Tower::TRIGGER_PIN, OUTPUT);
    }
    writeLine(true);
    pulseActive = false;
    everTriggered = false;
    triggerCount = 0;
}

void Dribbler::update() {
    if (pulseActive && millis() - pulseStartMs >= Tower::PULSE_MS) {
        pulseActive = false;
        writeLine(!enabled); // back to idle: LOW enabled, HIGH disabled
    }
}

void Dribbler::setEnabled(bool on) {
    enabled = on;
    stop();
}

bool Dribbler::isEnabled() {
    return enabled;
}

void Dribbler::stop() {
    pulseActive = false;
    writeLine(!enabled); // idle level: LOW enabled, HIGH (held) disabled
}

bool Dribbler::requestFeed() {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return false; // Tower not wired on this bench.
    }
    if (!enabled || isBusy()) {
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
