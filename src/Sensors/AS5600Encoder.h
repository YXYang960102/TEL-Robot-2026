#pragma once

#include <Arduino.h>
#include <AS5600.h>
#include "../Constants/As5600ServoTestConstants.h"

class AS5600Encoder {
public:
    void begin();
    bool update();
    void zero();

    uint16_t readRaw();
    uint16_t getRaw() const;
    uint16_t getZeroRaw() const;
    long getRelativeCounts() const;
    int getLastDeltaCounts() const;
    int getLastRejectedDeltaCounts() const;
    unsigned int getRejectedSampleCount() const;
    bool isMagnetDetected();

    void setMaxAcceptedDeltaCounts(int maxDeltaCounts);

    static int signedShortestError(uint16_t targetRaw, uint16_t currentRaw);
    static int signedShortestDelta(uint16_t previousRaw, uint16_t currentRaw);

private:
    AMS_5600 encoder;
    uint16_t raw = 0;
    uint16_t zeroRaw = 0;
    uint16_t lastRaw = 0;
    long relativeCounts = 0;
    int lastDeltaCounts = 0;
    int lastRejectedDeltaCounts = 0;
    int maxAcceptedDeltaCounts = As5600ServoTestConst::Encoder::MAX_ACCEPTED_DELTA_COUNTS;
    unsigned int rejectedSampleCount = 0;
    bool initialized = false;
};
