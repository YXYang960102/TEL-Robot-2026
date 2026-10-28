#include "TurretAngle.h"

#include <Arduino.h>
#include <Wire.h>

#include "../Constants/TurretAngleConstants.h"

using namespace TurretAngleConstants;

namespace {
uint16_t lastRaw = 0;
long accumulated = 0;
uint8_t statusBits = TurretAngle::NEVER_READ;
bool haveReading = false;
bool lost = false;
unsigned long lastReadMs = 0;
unsigned long lastGoodMs = 0;
}  // namespace

void TurretAngle::init() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    // Give up (and reset the bus) instead of hanging the loop.
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
    statusBits = NEVER_READ;
    haveReading = false;
    lost = false;
    accumulated = 0;
}

bool TurretAngle::readSensor(uint8_t& statusReg, uint16_t& rawAngle) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(REG_STATUS);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(3)) != 3) {
        return false;
    }
    statusReg = static_cast<uint8_t>(Wire.read());
    const uint8_t high = static_cast<uint8_t>(Wire.read());
    const uint8_t low = static_cast<uint8_t>(Wire.read());
    rawAngle = static_cast<uint16_t>(((high & 0x0F) << 8) | low);
    return true;
}

void TurretAngle::update() {
    const unsigned long now = millis();
    if (now - lastReadMs < READ_PERIOD_MS) {
        return;
    }
    lastReadMs = now;

    uint8_t statusReg = 0;
    uint16_t rawAngle = 0;
    const bool ok = readSensor(statusReg, rawAngle);
    if (!ok) {
        Wire.clearWireTimeoutFlag();
        statusBits = static_cast<uint8_t>(statusBits & ~READ_OK);
        if (haveReading && now - lastGoodMs > LOST_AFTER_MS) {
            lost = true;
        }
        if (lost) statusBits |= LOST;
        return;
    }

    if (!haveReading) {
        // First reading: where the turret is, relative to straight ahead.
        accumulated = ZERO_RAW < 0 ? 0 : foldDelta(rawAngle, ZERO_RAW);
        haveReading = true;
    } else {
        if (now - lastGoodMs > LOST_AFTER_MS) {
            lost = true; // too long without a read to know the turns
        }
        accumulated += foldDelta(rawAngle, lastRaw);
    }
    lastRaw = rawAngle;
    lastGoodMs = now;

    statusBits = READ_OK;
    if (statusReg & 0x20) statusBits |= MAGNET;
    if (statusReg & 0x10) statusBits |= TOO_WEAK;
    if (statusReg & 0x08) statusBits |= TOO_STRONG;
    if (lost) statusBits |= LOST;
}

uint16_t TurretAngle::raw() {
    return lastRaw;
}

uint8_t TurretAngle::status() {
    return statusBits;
}

long TurretAngle::counts() {
    return accumulated;
}

double TurretAngle::angleDegrees() {
    const double deg = accumulated / COUNTS_PER_TURRET_DEG;
    return INVERTED ? -deg : deg;
}

bool TurretAngle::isHealthy() {
    return (statusBits & READ_OK) && (statusBits & MAGNET) &&
           !(statusBits & (TOO_WEAK | TOO_STRONG | LOST));
}
