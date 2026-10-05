#pragma once

#include <Arduino.h>
#include <AS5600.h>

struct AS5600EncoderConfig {
    constexpr AS5600EncoderConfig(
        int countsPerRevolutionValue,
        int maximumAcceptedDeltaCountsValue,
        unsigned long sampleTimeoutMsValue)
        : countsPerRevolution(countsPerRevolutionValue),
          halfCountsPerRevolution(countsPerRevolutionValue / 2),
          maximumAcceptedDeltaCounts(maximumAcceptedDeltaCountsValue),
          sampleTimeoutMs(sampleTimeoutMsValue) {}

    int countsPerRevolution;
    int halfCountsPerRevolution;
    int maximumAcceptedDeltaCounts;
    unsigned long sampleTimeoutMs;
};

class AS5600Encoder {
public:
    explicit AS5600Encoder(const AS5600EncoderConfig& config);

    void begin();
    bool update();
    void zero();

    bool isValid() const;
    bool isMagnetDetected();
    uint16_t getRawCounts() const;
    uint16_t getZeroRawCounts() const;
    long getRelativeCounts() const;
    int getLastDeltaCounts() const;
    unsigned long getRejectedSampleCount() const;

    int shortestDelta(uint16_t previousRaw, uint16_t currentRaw) const;

private:
    const AS5600EncoderConfig config;
    AMS_5600 encoder;
    uint16_t rawCounts = 0;
    uint16_t zeroRawCounts = 0;
    uint16_t previousRawCounts = 0;
    long relativeCounts = 0;
    int lastDeltaCounts = 0;
    unsigned long rejectedSampleCount = 0;
    unsigned long lastValidSampleMs = 0;
    bool initialized = false;
    bool magnetDetected = false;
};
