#pragma once

#include <stdint.h>

// AS5600 magnetic angle sensor on the turret motor shaft (2026-10-27,
// Jeremy: fitted, I2C on the Mega's D20 SDA / D21 SCL). READ ONLY for now:
// src/Sensors/TurretAngle reads it and reports it (TEL slots 42-44,
// Dashboard), nothing is limited or driven from it until the turret's
// left / right mechanical limits have been measured with it.
namespace TurretAngleConstants {

constexpr uint8_t I2C_ADDRESS = 0x36;
// STATUS (MD bit 5 = magnet detected, ML bit 4 = too weak, MH bit 3 = too
// strong), then RAW ANGLE high nibble / low byte: one 3-byte read.
constexpr uint8_t REG_STATUS = 0x0B;
constexpr uint32_t I2C_CLOCK_HZ = 100000;
// A missing / shorted sensor must never hang the main loop (the safety
// watchdogs run there): every I2C transfer gives up after this, us.
constexpr uint32_t I2C_TIMEOUT_US = 2000;
constexpr unsigned long READ_PERIOD_MS = 10;

// One magnet turn = 4096 counts. Belt 180:36.5: one turret turn is
// 180 / 36.5 = 4.9315 magnet turns, so
//   counts per turret degree = 4096 * 4.9315 / 360 = 56.11.
constexpr double BELT_RATIO = 180.0 / 36.5;
constexpr double COUNTS_PER_TURRET_DEG = 4096.0 * BELT_RATIO / 360.0;

// The raw reading (0-4095) with the turret pointing straight ahead.
// -1 = take the reading at boot as straight ahead (then the turret must be
// centered at power-up). Measure it: center the turret, read "raw" on the
// Dashboard, put it here. Either way the turret must be within half a
// magnet turn (+-36.5 deg of turret) of center at power-up: the same raw
// value repeats every 73 deg of turret travel.
constexpr int ZERO_RAW = -1;

// false: the angle grows the way the raw count grows. Set so that turning
// the turret right reads + (the same sign as the camera's tx).
constexpr bool INVERTED = false;

// No good read for this long and the turns count can't be trusted any
// more (the magnet may have passed half a turn unseen): reported as LOST
// until a reboot.
constexpr unsigned long LOST_AFTER_MS = 100;

}  // namespace TurretAngleConstants
