#pragma once

#include <Arduino.h>
#include <AS5600.h>

class AS5600Encoder {
public:
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

    static int shortestDelta(uint16_t previousRaw, uint16_t currentRaw);

private:
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

