#pragma once

#include <stdint.h>

class TurretAngle {
public:
    static constexpr uint8_t READ_OK = 0x01;
    static constexpr uint8_t MAGNET = 0x02;
    static constexpr uint8_t TOO_WEAK = 0x04;
    static constexpr uint8_t TOO_STRONG = 0x08;
    static constexpr uint8_t LOST = 0x10;
    static constexpr uint8_t NEVER_READ = 0x20;
    static constexpr uint8_t ABSOLUTE = 0x40;
    static constexpr uint8_t OUTSIDE_HOME = 0x80;

    static void init();
    static void update();

    static uint16_t raw();
    static uint8_t status();
    static long counts();
    static double angleDegrees();
    static int agc();
    static int magnitude();
    static bool isHealthy();
    static bool isAbsolute();
    static bool setZeroHere();

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
