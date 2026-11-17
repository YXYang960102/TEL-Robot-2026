#pragma once

#include <stdint.h>

#include "../Sensors/TurretStore.h"

class AimTrimStore {
public:
    static constexpr uint8_t MAGIC = 0xB1;
    static constexpr uint8_t RECORD_BYTES = 7;
    static constexpr uint8_t SLOTS = 2;
    static constexpr int BASE_ADDRESS = 400;
    static constexpr int SLOT_STRIDE = 8;

    static void init();
    static void update();

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
