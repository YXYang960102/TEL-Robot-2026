#pragma once

#include <stdint.h>

// The turret's last known position, kept in the Mega's EEPROM (2026-10-31,
// Jeremy). The AS5600 only knows the angle within one magnet turn (73 deg
// of turret), so the absolute turret angle is remembered instead:
//   saved:  counts (all turns, raw-sensor direction) + the raw reading then
//   boot:   counts = saved.counts + fold(raw now - saved.raw)
// which is right as long as the turret moved less than half a magnet turn
// (+-36.5 deg of turret) while the power was off.
//
// One record = 12 bytes:
//   [0] MAGIC  [1..4] seq  [5..8] counts  [9..10] raw  [11] CRC-8 of [0..10]
// (little endian). Records go round-robin through SLOTS slots, so no cell is
// rewritten every time; the newest valid one (highest seq) wins. A record
// cut off by a power loss fails its CRC and the one before it is used.
// No Arduino code here, so the encoding can be checked on a PC.
namespace TurretStore {

constexpr uint8_t MAGIC = 0xA7;
constexpr uint8_t RECORD_BYTES = 12;
constexpr uint8_t SLOTS = 32;
constexpr int BASE_ADDRESS = 0; // EEPROM 0..383; nothing else uses it

struct Record {
    uint32_t seq;
    int32_t counts;
    uint16_t raw;
};

// CRC-8, polynomial 0x07, init 0.
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

// readByte(address) -> the newest valid record and its slot. False = none
// (a new board, or never set: the turret position is unknown).
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

}  // namespace TurretStore
