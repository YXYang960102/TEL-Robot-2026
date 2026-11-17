#include "Dribbler.h"

#include "../Constants/BuildProfile.h"

using namespace DribblerConstants;

bool Dribbler::enabled = false;
bool Dribbler::pulseActive = false;
bool Dribbler::everTriggered = false;
unsigned long Dribbler::pulseStartMs = 0;
unsigned long Dribbler::triggerCount = 0;
bool Dribbler::ballReady = false;
unsigned long Dribbler::loadedCount = 0;
bool Dribbler::readyHigh = false;
bool Dribbler::readyCounted = false;
unsigned long Dribbler::readyHighSinceMs = 0;

void Dribbler::init() {
    enabled = false;
    writeDisable(true);
    writeTrigger(false);
    if (BuildProfile::DRIVES_FEED_TOWER) {
        pinMode(Tower::DISABLE_PIN, OUTPUT);
        pinMode(Tower::TRIGGER_PIN, OUTPUT);
        pinMode(Tower::READY_PIN, INPUT);
    }
    writeDisable(true);
    writeTrigger(false);
    pulseActive = false;
    everTriggered = false;
    triggerCount = 0;
    ballReady = false;
    loadedCount = 0;
    readyHigh = false;
    readyCounted = false;
}

void Dribbler::update() {
    if (pulseActive && millis() - pulseStartMs >= Tower::PULSE_MS) {
        pulseActive = false;
        writeTrigger(false);
    }
    pollReady();
}

void Dribbler::pollReady() {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return;
    }
    const unsigned long now = millis();
    if (digitalRead(Tower::READY_PIN) != HIGH) {
        readyHigh = false;
        readyCounted = false;
        return;
    }
    if (!readyHigh) {
        readyHigh = true;
        readyHighSinceMs = now;
    }
    if (!readyCounted && now - readyHighSinceMs >= Tower::READY_MIN_HIGH_MS) {
        readyCounted = true;
        if (!ballReady) loadedCount++;
        ballReady = true;
    }
}

void Dribbler::setEnabled(bool on) {
    enabled = on;
    writeDisable(!on);
    stop();
}

bool Dribbler::isEnabled() {
    return enabled;
}

void Dribbler::stop() {
    pulseActive = false;
    writeTrigger(false);
}

bool Dribbler::requestFeed() {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return false;
    }
    if (!enabled || isBusy()) {
        return false;
    }
    pulseStartMs = millis();
    pulseActive = true;
    everTriggered = true;
    triggerCount++;
    ballReady = false; // this ball is on its way; wait for the next A14 pulse
    writeTrigger(true);
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

bool Dribbler::hasBall() {
    return ballReady;
}

unsigned long Dribbler::getLoadedCount() {
    return loadedCount;
}

bool Dribbler::towerAllowsShot() {
    return !Tower::READY_REQUIRED || ballReady;
}

void Dribbler::writeTrigger(bool active) {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return;
    }
    digitalWrite(Tower::TRIGGER_PIN, active == Tower::TRIGGER_ACTIVE_HIGH ? HIGH : LOW);
}

void Dribbler::writeDisable(bool disabled) {
    if (!BuildProfile::DRIVES_FEED_TOWER) {
        return;
    }
    digitalWrite(Tower::DISABLE_PIN, disabled ? HIGH : LOW);
}
