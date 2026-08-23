#include "Dribbler.h"

#include "../Constants/DribblerConstants.h"

using namespace DribblerConstants;

DribblerControlMode Dribbler::controlMode = DribblerControlMode::DISABLED;
int Dribbler::shootRemaining = 0;
unsigned long Dribbler::completedShotCount = 0;
bool Dribbler::feeding = false;
double Dribbler::feedCommand = 0.0;
int Dribbler::feedPwm = Feeder::MIN_PWM;
bool Dribbler::rawSensorBlocked = false;
bool Dribbler::stableSensorBlocked = false;
bool Dribbler::ballSeenDuringRequest = false;
unsigned long Dribbler::rawSensorChangedMs = 0;

void Dribbler::init() {
    pinMode(Feeder::PWM_PIN, OUTPUT);
    pinMode(Feeder::ENABLE_PIN, OUTPUT);
    pinMode(ExitSensor::SIGNAL_PIN, INPUT_PULLUP);

    rawSensorBlocked = readExitSensorBlocked();
    stableSensorBlocked = rawSensorBlocked;
    rawSensorChangedMs = millis();
    stop();
}

void Dribbler::update() {
    updateExitSensor();

    if (controlMode == DribblerControlMode::MANUAL_OPEN_LOOP) {
        writeFeedOutput(feedCommand);
        return;
    }

    if (controlMode == DribblerControlMode::COUNTED_FEED &&
        shootRemaining > 0) {
        writeFeedOutput(Feeder::DEFAULT_FEED_COMMAND);
        return;
    }

    stop();
}

void Dribbler::stop() {
    controlMode = DribblerControlMode::DISABLED;
    shootRemaining = 0;
    feedCommand = 0.0;
    ballSeenDuringRequest = false;
    writeFeedOutput(0.0);
}

void Dribbler::setFeedAction(FeedAction action) {
    if (action == FeedAction::FEED) {
        setFeedOpenLoop(Feeder::DEFAULT_FEED_COMMAND);
    } else {
        stop();
    }
}

void Dribbler::setFeedOpenLoop(double command) {
    feedCommand = constrain(command, 0.0, 1.0);
    shootRemaining = 0;
    ballSeenDuringRequest = false;
    controlMode = feedCommand > 0.0
        ? DribblerControlMode::MANUAL_OPEN_LOOP
        : DribblerControlMode::DISABLED;
    writeFeedOutput(feedCommand);
}

void Dribbler::setShootRequest(int count) {
    shootRemaining = constrain(count, Request::MIN_COUNT, Request::MAX_COUNT);
    feedCommand = 0.0;
    ballSeenDuringRequest = false;
    controlMode = shootRemaining > 0
        ? DribblerControlMode::COUNTED_FEED
        : DribblerControlMode::DISABLED;
}

DribblerControlMode Dribbler::getControlMode() {
    return controlMode;
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

double Dribbler::getFeedCommand() {
    return feeding ? feedCommand : 0.0;
}

int Dribbler::getFeedPwm() {
    return feedPwm;
}

void Dribbler::writeFeedOutput(double command) {
    const double boundedCommand = constrain(command, 0.0, 1.0);
    feedCommand = boundedCommand;
    feedPwm = constrain(
        static_cast<int>(boundedCommand * Feeder::MAX_PWM + 0.5),
        Feeder::MIN_PWM,
        Feeder::MAX_PWM);
    feeding = feedPwm > Feeder::MIN_PWM;

    digitalWrite(
        Feeder::ENABLE_PIN,
        feeding == Feeder::ENABLE_ACTIVE_HIGH ? HIGH : LOW);
    analogWrite(Feeder::PWM_PIN, feedPwm);
}

void Dribbler::updateExitSensor() {
    const bool nextRawBlocked = readExitSensorBlocked();
    const unsigned long now = millis();

    if (nextRawBlocked != rawSensorBlocked) {
        rawSensorBlocked = nextRawBlocked;
        rawSensorChangedMs = now;
    }

    if (rawSensorBlocked == stableSensorBlocked ||
        now - rawSensorChangedMs < ExitSensor::DEBOUNCE_MS) {
        return;
    }

    stableSensorBlocked = rawSensorBlocked;

    if (controlMode != DribblerControlMode::COUNTED_FEED ||
        shootRemaining <= 0) {
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
        if (shootRemaining == 0) {
            controlMode = DribblerControlMode::DISABLED;
        }
    }
}

bool Dribbler::readExitSensorBlocked() {
    const bool pinLow = digitalRead(ExitSensor::SIGNAL_PIN) == LOW;
    return ExitSensor::BLOCKED_LOW ? pinLow : !pinLow;
}
