#pragma once

#include <stdint.h>

#include "../Sensors/TurretStore.h"

// The operator's aim / elevation trim (ManualControl::getAimOffsetPx() /
// getElevationOffsetUs()), kept in the Mega's EEPROM so it survives a power
// cycle (2026-11-08, Jeremy: "AI locked on, but the shots go off to one
// side" -- fine-adjust once, and every hole keeps that correction until C
// clears it).
//
// One record = 7 bytes, in two slots used in turn (a record cut off by a
// power loss fails its CRC and the other slot is used):
//   [0] MAGIC  [1] seq  [2..3] aim px x 10  [4..5] elevation us x 10
//   [6] CRC-8 of [0..5]                              (int16, little endian)
// EEPROM 400..413: after TurretStore's 0..383.
class AimTrimStore {
public:
    static constexpr uint8_t MAGIC = 0xB1;
    static constexpr uint8_t RECORD_BYTES = 7;
    static constexpr uint8_t SLOTS = 2;
    static constexpr int BASE_ADDRESS = 400;
    static constexpr int SLOT_STRIDE = 8;

    static void init();   // after ManualControl::init(): load the saved trim
    static void update(); // after ManualControl::update(): save a new one

    struct Record {
        uint8_t seq;
        int16_t aimTenthsPx;
        int16_t elevationTenthsUs;
    };

    static void encode(const Record& r, uint8_t out[RECORD_BYTES]) {
        const uint16_t aim = static_cast<uint16_t>(r.aimTenthsPx);
        const uint16_t elevation = static_cast<uint16_t>(r.elevationTenthsUs);
        out[0] = MAGIC;
        out[1] = r.seq;
        out[2] = static_cast<uint8_t>(aim);
        out[3] = static_cast<uint8_t>(aim >> 8);
        out[4] = static_cast<uint8_t>(elevation);
        out[5] = static_cast<uint8_t>(elevation >> 8);
        out[6] = TurretStore::crc8(out, RECORD_BYTES - 1);
    }

    static bool decode(const uint8_t in[RECORD_BYTES], Record& r) {
        if (in[0] != MAGIC || TurretStore::crc8(in, RECORD_BYTES - 1) != in[6]) return false;
        r.seq = in[1];
        r.aimTenthsPx = static_cast<int16_t>(in[2] | (in[3] << 8));
        r.elevationTenthsUs = static_cast<int16_t>(in[4] | (in[5] << 8));
        return true;
    }

private:
    static void writeStep();
};
