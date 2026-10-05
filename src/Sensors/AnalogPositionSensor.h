#pragma once

#include <Arduino.h>

struct AnalogPositionSensorConfig {
    constexpr AnalogPositionSensorConfig(
        uint8_t signalPinValue,
        int validMinimumRawValue,
        int validMaximumRawValue,
        double filterAlphaValue,
        unsigned long samplePeriodMsValue,
        unsigned long sampleTimeoutMsValue)
        : signalPin(signalPinValue),
          validMinimumRaw(validMinimumRawValue),
          validMaximumRaw(validMaximumRawValue),
          filterAlpha(filterAlphaValue),
          samplePeriodMs(samplePeriodMsValue),
          sampleTimeoutMs(sampleTimeoutMsValue) {}

    uint8_t signalPin;
    int validMinimumRaw;
    int validMaximumRaw;
    double filterAlpha;
    unsigned long samplePeriodMs;
    unsigned long sampleTimeoutMs;
};

class AnalogPositionSensor {
public:
    explicit AnalogPositionSensor(const AnalogPositionSensorConfig& config);

    void begin();
    bool update();

    bool isValid() const;
    int getRaw() const;
    double getFilteredRaw() const;

private:
    const AnalogPositionSensorConfig config;
    int raw = 0;
    double filteredRaw = 0.0;
    unsigned long lastSampleMs = 0;
    unsigned long lastValidSampleMs = 0;
    bool initialized = false;
    bool latestSampleValid = false;
};
