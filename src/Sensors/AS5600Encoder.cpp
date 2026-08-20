#include "AS5600Encoder.h"

#include "../Constants/ShooterConstants.h"

#include <Wire.h>

using namespace ShooterConst;

void AS5600Encoder::begin() {
    Wire.begin();
    initialized = false;
    update();
}

bool AS5600Encoder::update() {
    magnetDetected = isMagnetDetected();
    if (!magnetDetected) {
        lastDeltaCounts = 0;
        return false;
    }

    const uint16_t nextRawCounts = encoder.getRawAngle();

    if (!initialized) {
        rawCounts = nextRawCounts;
        zeroRawCounts = nextRawCounts;
        previousRawCounts = nextRawCounts;
        relativeCounts = 0;
        lastDeltaCounts = 0;
        lastValidSampleMs = millis();
        initialized = true;
        return true;
    }

    const int nextDeltaCounts = shortestDelta(previousRawCounts, nextRawCounts);
    if (abs(nextDeltaCounts) > Encoder::MAX_ACCEPTED_DELTA_COUNTS) {
        lastDeltaCounts = 0;
        rejectedSampleCount++;
        return false;
    }

    lastDeltaCounts = nextDeltaCounts;
    relativeCounts += nextDeltaCounts;
    previousRawCounts = nextRawCounts;
    rawCounts = nextRawCounts;
    lastValidSampleMs = millis();
    return true;
}

void AS5600Encoder::zero() {
    if (!magnetDetected) {
        return;
    }

    zeroRawCounts = rawCounts;
    previousRawCounts = rawCounts;
    relativeCounts = 0;
    lastDeltaCounts = 0;
    rejectedSampleCount = 0;
}

bool AS5600Encoder::isValid() const {
    return initialized && magnetDetected &&
           millis() - lastValidSampleMs <= Encoder::SAMPLE_TIMEOUT_MS;
}

bool AS5600Encoder::isMagnetDetected() {
    return encoder.detectMagnet() == 1;
}

uint16_t AS5600Encoder::getRawCounts() const {
    return rawCounts;
}

uint16_t AS5600Encoder::getZeroRawCounts() const {
    return zeroRawCounts;
}

long AS5600Encoder::getRelativeCounts() const {
    return relativeCounts;
}

int AS5600Encoder::getLastDeltaCounts() const {
    return lastDeltaCounts;
}

unsigned long AS5600Encoder::getRejectedSampleCount() const {
    return rejectedSampleCount;
}

int AS5600Encoder::shortestDelta(uint16_t previousRaw, uint16_t currentRaw) {
    int delta = static_cast<int>(currentRaw) - static_cast<int>(previousRaw);

    if (delta > Encoder::HALF_COUNTS_PER_REV) {
        delta -= Encoder::COUNTS_PER_REV;
    } else if (delta < -Encoder::HALF_COUNTS_PER_REV) {
        delta += Encoder::COUNTS_PER_REV;
    }

    return delta;
}

