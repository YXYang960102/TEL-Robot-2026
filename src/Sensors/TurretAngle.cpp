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

// EEPROM: the newest record found at boot, and the one being written.
// A record goes out one byte per loop, only when the EEPROM is idle, so
// the ~3.3 ms write never blocks the loop; 12 bytes take ~120 ms.
TurretStore::Record stored = {0, 0, 0};
bool haveStored = false;
uint8_t nextSlot = 0;
long savedCounts = 0;
uint8_t writeBytes[TurretStore::RECORD_BYTES];
uint8_t writeIndex = TurretStore::RECORD_BYTES; // == RECORD_BYTES: idle
int writeAddress = 0;
bool saveForced = false;
long stillRefCounts = 0;
unsigned long stillSinceMs = 0;

long degreesToCounts(double deg) {
    return lround(deg * COUNTS_PER_TURRET_DEG);
}

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

// First reading: the saved position plus how far the magnet turned since
// (folded: right only if that was less than half a magnet turn).
// Example: saved +20.0 deg (counts -1122 with INVERTED), raw 1500; now raw
// 1781 -> fold +281 -> counts -841 -> +15.0 deg.
void TurretAngle::restoreFromStore(uint16_t rawAngle) {
    if (!haveStored) {
        accumulated = 0; // relative to boot: limits can't be trusted
        absolute = false;
        return;
    }
    accumulated = stored.counts + foldDelta(rawAngle, stored.raw);
    const double deg = countsToDegrees(accumulated);
    absolute = deg >= LIMIT_LEFT_DEG - RESTORE_MARGIN_DEG && deg <= LIMIT_RIGHT_DEG + RESTORE_MARGIN_DEG;
    savedCounts = accumulated;
    stillRefCounts = accumulated;
}

void TurretAngle::update() {
    const unsigned long now = millis();
    writeStoreStep();
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
            absolute = false; // turns unknown: never saved from here on
        }
        if (lost) statusBits |= LOST;
        return;
    }

    if (!haveReading) {
        restoreFromStore(rawAngle);
        haveReading = true;
        stillSinceMs = now;
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

    saveIfDue(now);
}

// Save when moved SAVE_STEP_DEG, or still for SAVE_STILL_MS after a small
// change; only an absolute, healthy position.
void TurretAngle::saveIfDue(unsigned long now) {
    if (labs(accumulated - stillRefCounts) > degreesToCounts(SAVE_STILL_DEG)) {
        stillRefCounts = accumulated;
        stillSinceMs = now;
    }
    if (!absolute || !isHealthy() || writeIndex < TurretStore::RECORD_BYTES) {
        return;
    }
    const long change = labs(accumulated - savedCounts);
    const bool due = saveForced || change >= degreesToCounts(SAVE_STEP_DEG) ||
                     (now - stillSinceMs >= SAVE_STILL_MS && change >= degreesToCounts(SAVE_MIN_CHANGE_DEG));
    if (!due) {
        return;
    }
    TurretStore::Record r;
    r.seq = haveStored ? stored.seq + 1 : 1;
    r.counts = static_cast<int32_t>(accumulated);
    r.raw = lastRaw;
    TurretStore::encode(r, writeBytes);
    writeAddress = TurretStore::slotAddress(nextSlot);
    writeIndex = 0;
    nextSlot = static_cast<uint8_t>((nextSlot + 1) % TurretStore::SLOTS);
    stored = r;
    haveStored = true;
    savedCounts = accumulated;
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
        (statusBits & (TOO_WEAK | TOO_STRONG))) {
        return false;
    }
    accumulated = 0;
    lost = false;
    absolute = true;
    statusBits = static_cast<uint8_t>((statusBits & ~LOST) | ABSOLUTE);
    saveForced = true; // goes out as soon as the EEPROM writer is idle
    return true;
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
           !(statusBits & (TOO_WEAK | TOO_STRONG | LOST));
}

bool TurretAngle::isAbsolute() {
    return absolute && !lost;
}
