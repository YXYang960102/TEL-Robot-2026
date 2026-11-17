#include "AimTrimStore.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

#include "ManualControl.h"

namespace {
uint8_t lastSeq = 0;
uint8_t nextSlot = 0;
uint16_t savedTrimSeq = 0;
uint8_t writeBytes[AimTrimStore::RECORD_BYTES];
uint8_t writeIndex = AimTrimStore::RECORD_BYTES;
int writeAddress = 0;

int slotAddress(uint8_t slot) {
    return AimTrimStore::BASE_ADDRESS + slot * AimTrimStore::SLOT_STRIDE;
}

int16_t toTenths(double v) {
    return static_cast<int16_t>(lround(v * 10.0));
}
}

void AimTrimStore::init() {
    bool found = false;
    Record newest = {0, 0, 0};
    for (uint8_t slot = 0; slot < SLOTS; slot++) {
        uint8_t bytes[RECORD_BYTES];
        for (uint8_t i = 0; i < RECORD_BYTES; i++) bytes[i] = EEPROM.read(slotAddress(slot) + i);
        Record r;
        if (decode(bytes, r) && (!found || static_cast<int8_t>(r.seq - newest.seq) > 0)) {
            found = true;
            newest = r;
            nextSlot = static_cast<uint8_t>((slot + 1) % SLOTS);
        }
    }
    if (found) {
        lastSeq = newest.seq;
        ManualControl::setTrim(newest.aimTenthsPx / 10.0, newest.elevationTenthsUs / 10.0);
    }
    savedTrimSeq = ManualControl::getTrimSeq();
}

void AimTrimStore::update() {
    if (writeIndex >= RECORD_BYTES && ManualControl::getTrimSeq() != savedTrimSeq) {
        savedTrimSeq = ManualControl::getTrimSeq();
        Record r;
        r.seq = static_cast<uint8_t>(lastSeq + 1);
        r.aimTenthsPx = toTenths(ManualControl::getAimOffsetPx());
        r.elevationTenthsUs = toTenths(ManualControl::getElevationOffsetUs());
        encode(r, writeBytes);
        writeAddress = slotAddress(nextSlot);
        writeIndex = 0;
        nextSlot = static_cast<uint8_t>((nextSlot + 1) % SLOTS);
        lastSeq = r.seq;
    }
    writeStep();
}

void AimTrimStore::writeStep() {
    if (writeIndex >= RECORD_BYTES || !eeprom_is_ready()) {
        return;
    }
    EEPROM.update(writeAddress + writeIndex, writeBytes[writeIndex]);
    writeIndex++;
}
