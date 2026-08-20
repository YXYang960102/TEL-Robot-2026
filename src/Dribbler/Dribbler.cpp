#include "Dribbler.h"
#include "../Constants/Pins.h"
#include "../Constants/DribblerConstants.h"

int Dribbler::shootRemaining = 0;
unsigned long Dribbler::completedShotCount = 0;
bool Dribbler::feeding = false;
bool Dribbler::rawSensorBlocked = false;
bool Dribbler::stableSensorBlocked = false;
bool Dribbler::ballSeenDuringRequest = false;
unsigned long Dribbler::rawSensorChangedMs = 0;

void Dribbler::init() {
    pinMode(PIN_DRIBBLE_UP, OUTPUT);
    pinMode(PIN_DRIBBLE_DOWN, OUTPUT);
    pinMode(PIN_DRIBBLE_EXIT_SENSOR, INPUT_PULLUP);

    rawSensorBlocked = readExitSensorBlocked();
    stableSensorBlocked = rawSensorBlocked;
    rawSensorChangedMs = millis();
    stop();
}

void Dribbler::setShootRequest(int count) {
    shootRemaining = constrain(
        count, DribblerConst::MIN_SHOT_REQUEST, DribblerConst::MAX_SHOT_REQUEST);
    ballSeenDuringRequest = false;
}

int Dribbler::getShootRemaining() {
    return shootRemaining;
}

unsigned long Dribbler::getCompletedShotCount() {
    return completedShotCount;
}

bool Dribbler::isExitSensorBlocked() {
    return stableSensorBlocked;
}

bool Dribbler::isFeeding() {
    return feeding;
}

void Dribbler::update() {
    updateExitSensor();

    if (shootRemaining > 0) {
        runFeed();
    } else {
        stop();
    }
}

void Dribbler::runFeed() {
    digitalWrite(
        PIN_DRIBBLE_DOWN, DribblerConst::FEED_GATE_ACTIVE_HIGH ? HIGH : LOW);
    analogWrite(PIN_DRIBBLE_UP, DribblerConst::FEED_PWM);
    feeding = true;
}

void Dribbler::stop() {
    digitalWrite(
        PIN_DRIBBLE_DOWN, DribblerConst::FEED_GATE_ACTIVE_HIGH ? LOW : HIGH);
    analogWrite(PIN_DRIBBLE_UP, 0);
    feeding = false;
    ballSeenDuringRequest = false;
}

void Dribbler::updateExitSensor() {
    const bool nextRawBlocked = readExitSensorBlocked();
    const unsigned long now = millis();

    if (nextRawBlocked != rawSensorBlocked) {
        rawSensorBlocked = nextRawBlocked;
        rawSensorChangedMs = now;
    }

    if (rawSensorBlocked == stableSensorBlocked ||
        now - rawSensorChangedMs < DribblerConst::SENSOR_DEBOUNCE_MS) {
        return;
    }

    stableSensorBlocked = rawSensorBlocked;

    if (shootRemaining <= 0) {
        ballSeenDuringRequest = false;
        return;
    }

    if (stableSensorBlocked) {
        ballSeenDuringRequest = true;
        return;
    }

    if (ballSeenDuringRequest) {
        shootRemaining--;
        completedShotCount++;
        ballSeenDuringRequest = false;
    }
}

bool Dribbler::readExitSensorBlocked() {
    const bool pinLow = digitalRead(PIN_DRIBBLE_EXIT_SENSOR) == LOW;
    return DribblerConst::EXIT_SENSOR_BLOCKED_LOW ? pinLow : !pinLow;
}
