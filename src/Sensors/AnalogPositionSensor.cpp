#include "AnalogPositionSensor.h"

AnalogPositionSensor::AnalogPositionSensor(
    const AnalogPositionSensorConfig& configValue)
    : config(configValue) {}

void AnalogPositionSensor::begin() {
    pinMode(config.signalPin, INPUT);
    initialized = false;
    latestSampleValid = false;
    lastSampleMs = 0;
    lastValidSampleMs = 0;
    update();
}

bool AnalogPositionSensor::update() {
    const unsigned long nowMs = millis();
    if (initialized && nowMs - lastSampleMs < config.samplePeriodMs) {
        return isValid();
    }
    lastSampleMs = nowMs;

    raw = analogRead(config.signalPin);
    latestSampleValid =
        raw >= config.validMinimumRaw &&
        raw <= config.validMaximumRaw;
    if (!latestSampleValid) {
        return false;
    }

    if (!initialized) {
        filteredRaw = raw;
        initialized = true;
    } else {
        filteredRaw += config.filterAlpha * (raw - filteredRaw);
    }
    lastValidSampleMs = nowMs;
    return true;
}

bool AnalogPositionSensor::isValid() const {
    return initialized && latestSampleValid &&
           millis() - lastValidSampleMs <= config.sampleTimeoutMs;
}

int AnalogPositionSensor::getRaw() const {
    return raw;
}

double AnalogPositionSensor::getFilteredRaw() const {
    return filteredRaw;
}
