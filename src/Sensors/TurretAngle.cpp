#include "TurretAngle.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

#include "../Constants/TurretAngleConstants.h"
#include "TurretStore.h"

using namespace TurretAngleConstants;

namespace {
uint16_t lastRaw = 0;
long accumulated = 0;
uint8_t statusBits = TurretAngle::NEVER_READ;
bool haveReading = false;
bool lost = false;
bool absolute = false;
unsigned long lastReadMs = 0;
unsigned long lastGoodMs = 0;
int agcValue = -1;
int magnitudeValue = -1;
unsigned long lastStrengthMs = 0;

// EEPROM: the newest record found at boot, and the one being written.
// A record goes out one byte per loop, only when the EEPROM is idle, so
// the ~3.3 ms write never blocks the loop; 12 bytes take ~120 ms.
TurretStore::Record stored = {0, 0, 0};
bool haveStored = false;
uint8_t nextSlot = 0;
uint8_t writeBytes[TurretStore::RECORD_BYTES];
uint8_t writeIndex = TurretStore::RECORD_BYTES; // == RECORD_BYTES: idle
int writeAddress = 0;
bool saveForced = false;
uint16_t homeRawToSave = 0;
bool outsideHome = false;

double countsToDegrees(long counts) {
    const double deg = counts / COUNTS_PER_TURRET_DEG;
    return INVERTED ? -deg : deg;
}
}  // namespace

void TurretAngle::init() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    // Give up (and reset the bus) instead of hanging the loop.
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
    statusBits = NEVER_READ;
    haveReading = false;
    lost = false;
    absolute = false;
    accumulated = 0;
    uint8_t slot = 0;
    haveStored = TurretStore::findNewest([](int address) { return EEPROM.read(address); }, stored, slot);
    nextSlot = haveStored ? static_cast<uint8_t>(slot + 1) : 0;
    writeIndex = TurretStore::RECORD_BYTES;
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

// First reading: the turret was turned back near home by hand, so its
// angle is how far the magnet is from the home raw, folded into half a
// magnet turn either way. Example: home raw 3918, boot raw 522 -> fold
// 522 - 3918 = -3396 -> +700 counts -> -700 / 56.11 = -12.5 deg (INVERTED)
// -> within 20 deg: absolute, the first Enable turns it 12.5 deg right.
void TurretAngle::bootFromHome(uint16_t rawAngle) {
    if (!haveStored) {
        accumulated = 0; // no home point yet: limits can't be trusted
        absolute = false;
        outsideHome = false;
        return;
    }
    accumulated = foldDelta(rawAngle, stored.raw);
    absolute = fabs(countsToDegrees(accumulated)) <= HOME_WINDOW_DEG;
    outsideHome = !absolute;
}

void TurretAngle::update() {
    const unsigned long now = millis();
    writeStoreStep();
    readStrength(now);
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
            absolute = false; // turns unknown
        }
        if (lost) statusBits |= LOST;
        return;
    }

    if (!haveReading) {
        bootFromHome(rawAngle);
        haveReading = true;
    } else {
        if (now - lastGoodMs > LOST_AFTER_MS) {
            lost = true; // too long without a read to know the turns
            absolute = false;
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
    if (absolute) statusBits |= ABSOLUTE;
    if (outsideHome) statusBits |= OUTSIDE_HOME;

    saveHomeIfDue();
}

// The home point goes to EEPROM once, after 設為歸位點, as soon as the
// reading is healthy and the writer idle (the raw taken at the press).
void TurretAngle::saveHomeIfDue() {
    if (!saveForced || !isHealthy() || writeIndex < TurretStore::RECORD_BYTES) {
        return;
    }
    TurretStore::Record r;
    r.seq = haveStored ? stored.seq + 1 : 1;
    r.counts = 0;
    r.raw = homeRawToSave;
    TurretStore::encode(r, writeBytes);
    writeAddress = TurretStore::slotAddress(nextSlot);
    writeIndex = 0;
    nextSlot = static_cast<uint8_t>((nextSlot + 1) % TurretStore::SLOTS);
    stored = r;
    haveStored = true;
    saveForced = false;
}

void TurretAngle::writeStoreStep() {
    if (writeIndex >= TurretStore::RECORD_BYTES || !eeprom_is_ready()) {
        return;
    }
    EEPROM.update(writeAddress + writeIndex, writeBytes[writeIndex]); // starts it, doesn't wait
    writeIndex++;
}

bool TurretAngle::setZeroHere() {
    if (!haveReading || !(statusBits & READ_OK) || !(statusBits & MAGNET) ||
        (statusBits & TOO_STRONG) || (WEAK_MAGNET_LOCKS && (statusBits & TOO_WEAK))) {
        return false;
    }
    accumulated = 0;
    lost = false;
    absolute = true;
    outsideHome = false;
    statusBits = static_cast<uint8_t>((statusBits & ~(LOST | OUTSIDE_HOME)) | ABSOLUTE);
    homeRawToSave = lastRaw;
    saveForced = true; // goes out as soon as the EEPROM writer is idle
    return true;
}

// AGC + MAGNITUDE in one 3-byte read, every AGC_READ_PERIOD_MS.
void TurretAngle::readStrength(unsigned long now) {
    if (now - lastStrengthMs < AGC_READ_PERIOD_MS) {
        return;
    }
    lastStrengthMs = now;
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(REG_AGC);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(3)) != 3) {
        Wire.clearWireTimeoutFlag();
        agcValue = -1;
        magnitudeValue = -1;
        return;
    }
    agcValue = Wire.read();
    const uint8_t high = static_cast<uint8_t>(Wire.read());
    const uint8_t low = static_cast<uint8_t>(Wire.read());
    magnitudeValue = ((high & 0x0F) << 8) | low;
}

int TurretAngle::agc() {
    return agcValue;
}

int TurretAngle::magnitude() {
    return magnitudeValue;
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
    return countsToDegrees(accumulated);
}

bool TurretAngle::isHealthy() {
    return (statusBits & READ_OK) && (statusBits & MAGNET) &&
           !(statusBits & (TOO_STRONG | LOST)) &&
           !(WEAK_MAGNET_LOCKS && (statusBits & TOO_WEAK));
}

bool TurretAngle::isAbsolute() {
    return absolute && !lost;
}
