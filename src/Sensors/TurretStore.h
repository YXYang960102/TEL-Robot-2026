#pragma once

#include <stdint.h>

namespace TurretStore {

constexpr uint8_t MAGIC = 0xA8;
constexpr uint8_t RECORD_BYTES = 12;
constexpr uint8_t SLOTS = 32;
constexpr int BASE_ADDRESS = 0;

struct Record {
    uint32_t seq;
    int32_t counts;
    uint16_t raw;
};

inline uint8_t crc8(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        crc = static_cast<uint8_t>(crc ^ data[i]);
        for (uint8_t bit = 0; bit < 8; bit++) {
            const uint8_t shifted = static_cast<uint8_t>(crc << 1);
            crc = (crc & 0x80u) ? static_cast<uint8_t>(shifted ^ 0x07u) : shifted;
        }
    }
    return crc;
}

inline void encode(const Record& r, uint8_t out[RECORD_BYTES]) {
    const uint32_t counts = static_cast<uint32_t>(r.counts);
    out[0] = MAGIC;
    for (uint8_t i = 0; i < 4; i++) {
        out[1 + i] = static_cast<uint8_t>(r.seq >> (8 * i));
        out[5 + i] = static_cast<uint8_t>(counts >> (8 * i));
    }
    out[9] = static_cast<uint8_t>(r.raw);
    out[10] = static_cast<uint8_t>(r.raw >> 8);
    out[11] = crc8(out, RECORD_BYTES - 1);
}

inline bool decode(const uint8_t in[RECORD_BYTES], Record& r) {
    if (in[0] != MAGIC || crc8(in, RECORD_BYTES - 1) != in[11]) return false;
    uint32_t seq = 0;
    uint32_t counts = 0;
    for (uint8_t i = 0; i < 4; i++) {
        seq |= static_cast<uint32_t>(in[1 + i]) << (8 * i);
        counts |= static_cast<uint32_t>(in[5 + i]) << (8 * i);
    }
    const uint16_t raw = static_cast<uint16_t>(in[9] | (in[10] << 8));
    if (raw > 4095) return false;
    r.seq = seq;
    r.counts = static_cast<int32_t>(counts);
    r.raw = raw;
    return true;
}

inline int slotAddress(uint8_t slot) {
    return BASE_ADDRESS + static_cast<int>(slot % SLOTS) * RECORD_BYTES;
}

template <typename ReadByte>
bool findNewest(ReadByte readByte, Record& newest, uint8_t& newestSlot) {
    bool found = false;
    for (uint8_t slot = 0; slot < SLOTS; slot++) {
        uint8_t bytes[RECORD_BYTES];
        const int base = slotAddress(slot);
        for (uint8_t i = 0; i < RECORD_BYTES; i++) bytes[i] = readByte(base + i);
        Record r;
        if (decode(bytes, r) && (!found || r.seq > newest.seq)) {
            newest = r;
            newestSlot = slot;
            found = true;
        }
    }
    return found;
}

}
