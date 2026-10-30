#pragma once

#include <stdint.h>

// Turret angle from the AS5600 (TurretAngleConstants.h).
//
// The AS5600 gives the magnet's angle within one turn (0-4095). The turret
// turns ~4.93 magnet turns per turret turn, so whole turns are counted in
// software: between two reads (10 ms apart) the shaft moves far less than
// half a turn, so a jump of more than 2048 counts is a wrap, not a move:
//   delta = raw - lastRaw, folded into -2048..2047
//   counts += delta
//   degrees = counts / 56.11   (sign: TurretAngleConstants::INVERTED)
// Absolute (2026-10-31): counts are relative to the home point set with
// setZeroHere() (its raw saved to EEPROM, TurretStore.h). At boot the
// turret has been turned back near home by hand: counts = fold(raw now -
// home raw), absolute only within HOME_WINDOW_DEG. No home saved, a boot
// outside the window (OUTSIDE_HOME), or a read outage (LOST) = not
// absolute.
class TurretAngle {
public:
    // Status bits (TEL slot 43).
    static constexpr uint8_t READ_OK = 0x01;      // the last read worked
    static constexpr uint8_t MAGNET = 0x02;       // MD: magnet detected
    static constexpr uint8_t TOO_WEAK = 0x04;     // ML
    static constexpr uint8_t TOO_STRONG = 0x08;   // MH
    static constexpr uint8_t LOST = 0x10;         // turns no longer trusted
    static constexpr uint8_t NEVER_READ = 0x20;   // nothing since boot
    static constexpr uint8_t ABSOLUTE = 0x40;     // counts are from the home point
    static constexpr uint8_t OUTSIDE_HOME = 0x80; // booted outside HOME_WINDOW_DEG

    static void init();
    static void update();

    // Raw angle within the magnet turn, 0-4095 (last good read).
    static uint16_t raw();
    static uint8_t status();
    // Counts from straight ahead, signed, all turns.
    static long counts();
    // Turret degrees from straight ahead (+ = right once INVERTED is set).
    static double angleDegrees();
    // Magnet strength (bench aid for the magnet gap): AGC 0-255 (higher =
    // weaker field) and MAGNITUDE 0-4095 (higher = stronger); -1 = no read.
    static int agc();
    static int magnitude();
    // A good recent read, magnet in range, turns trusted ("too weak" only
    // counts when TurretAngleConstants::WEAK_MAGNET_LOCKS is set).
    static bool isHealthy();
    // The angle is from the home point (restored or set), not from boot.
    static bool isAbsolute();
    // The turret is at the home point now: angle 0, saved at once. Refused
    // (false) without a healthy reading.
    static bool setZeroHere();

    // -2048..2047: the shortest way from `from` to `to` on the 4096 circle.
    static int foldDelta(int to, int from) {
        int d = (to - from) % 4096;
        if (d >= 2048) d -= 4096;
        if (d < -2048) d += 4096;
        return d;
    }

private:
    static bool readSensor(uint8_t& statusReg, uint16_t& rawAngle);
    static void readStrength(unsigned long now);
    static void bootFromHome(uint16_t rawAngle);
    static void saveHomeIfDue();
    static void writeStoreStep();
};
