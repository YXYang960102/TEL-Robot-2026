#include "AS5600Encoder.h"

#include <Wire.h>

void AS5600Encoder::begin() {
    Wire.begin();
    zero();
}

bool AS5600Encoder::update() {
    const bool magnetDetected = isMagnetDetected();
    const uint16_t newRaw = readRaw();

    if (!magnetDetected) {
        lastDeltaCounts = 0;
        return false;
    }

    if (!initialized) {
        raw = newRaw;
        zeroRaw = newRaw;
        lastRaw = newRaw;
        relativeCounts = 0;
        lastDeltaCounts = 0;
        initialized = true;
        return true;
    }

    const int newDeltaCounts = signedShortestDelta(lastRaw, newRaw);
    if (abs(newDeltaCounts) > maxAcceptedDeltaCounts) {
        lastDeltaCounts = 0;
        lastRejectedDeltaCounts = newDeltaCounts;
        rejectedSampleCount++;
        return true;
    }

    lastDeltaCounts = newDeltaCounts;
    lastRejectedDeltaCounts = 0;
    relativeCounts += lastDeltaCounts;
    lastRaw = newRaw;
    raw = newRaw;
    return true;
}

void AS5600Encoder::zero() {
    raw = readRaw();
    zeroRaw = raw;
    lastRaw = raw;
    relativeCounts = 0;
    lastDeltaCounts = 0;
    lastRejectedDeltaCounts = 0;
    rejectedSampleCount = 0;
    initialized = true;
}

uint16_t AS5600Encoder::readRaw() {
    return encoder.getRawAngle();
}

uint16_t AS5600Encoder::getRaw() const {
    return raw;
}

uint16_t AS5600Encoder::getZeroRaw() const {
    return zeroRaw;
}

long AS5600Encoder::getRelativeCounts() const {
    return relativeCounts;
}

int AS5600Encoder::getLastDeltaCounts() const {
    return lastDeltaCounts;
}

int AS5600Encoder::getLastRejectedDeltaCounts() const {
    return lastRejectedDeltaCounts;
}

unsigned int AS5600Encoder::getRejectedSampleCount() const {
    return rejectedSampleCount;
}

bool AS5600Encoder::isMagnetDetected() {
    return encoder.detectMagnet() == 1;
}

void AS5600Encoder::setMaxAcceptedDeltaCounts(int maxDeltaCounts) {
    maxAcceptedDeltaCounts = maxDeltaCounts;
}

int AS5600Encoder::signedShortestError(uint16_t targetRaw, uint16_t currentRaw) {
    int error = static_cast<int>(targetRaw) - static_cast<int>(currentRaw);

    if (error > As5600ServoTestConst::Encoder::HALF_COUNTS_PER_REV) {
        error -= As5600ServoTestConst::Encoder::COUNTS_PER_REV;
    } else if (error < -As5600ServoTestConst::Encoder::HALF_COUNTS_PER_REV) {
        error += As5600ServoTestConst::Encoder::COUNTS_PER_REV;
    }

    return error;
}

int AS5600Encoder::signedShortestDelta(uint16_t previousRaw, uint16_t currentRaw) {
    int delta = static_cast<int>(currentRaw) - static_cast<int>(previousRaw);

    if (delta > As5600ServoTestConst::Encoder::HALF_COUNTS_PER_REV) {
        delta -= As5600ServoTestConst::Encoder::COUNTS_PER_REV;
    } else if (delta < -As5600ServoTestConst::Encoder::HALF_COUNTS_PER_REV) {
        delta += As5600ServoTestConst::Encoder::COUNTS_PER_REV;
    }

    return delta;
}
