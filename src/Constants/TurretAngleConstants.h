#pragma once

#include <stdint.h>

namespace TurretAngleConstants {

constexpr uint8_t I2C_ADDRESS = 0x36;
constexpr uint8_t REG_STATUS = 0x0B; // STATUS + RAW ANGLE, one 3-byte read
constexpr uint32_t I2C_CLOCK_HZ = 100000;
constexpr uint8_t REG_AGC = 0x1A; // AGC 0-255, higher = weaker magnet
constexpr unsigned long AGC_READ_PERIOD_MS = 100;
constexpr uint32_t I2C_TIMEOUT_US = 2000;
constexpr unsigned long READ_PERIOD_MS = 10;

constexpr double BELT_RATIO = 180.0 / 36.5; // belt 180:36.5 = 4.93 magnet turns per turret turn
constexpr double COUNTS_PER_TURRET_DEG = 4096.0 * BELT_RATIO / 360.0; // 56.11

constexpr bool INVERTED = true; // + = turret right

constexpr bool GUARD_ENABLED = true; // soft limits, return to 0 deg, locks

constexpr double HOME_WINDOW_DEG = 25.0; // ± around the home point at boot
static_assert(HOME_WINDOW_DEG > 0.0 && HOME_WINDOW_DEG < 36.0,
              "the window must lie within half a magnet turn (36.5 deg)");

constexpr double LIMIT_RIGHT_DEG = 38.5; // from the home point
constexpr double LIMIT_LEFT_DEG = -166.0; // from the home point
constexpr double LIMIT_SLOW_ZONE_DEG = 10.0; // slow down this close to a limit
constexpr double LIMIT_SLOW_MIN_COMMAND = 0.2; // turret needs ~0.16 to move
constexpr double LIMIT_STOP_LOOKAHEAD_S = 0.15; // coast look-ahead for the limits

constexpr double RIGHT_COMMAND_SIGN = -1.0; // command sign that turns right

constexpr double RETURN_kP = 0.05;          // command per degree
constexpr double RETURN_MAX_COMMAND = 1.0;
constexpr double RETURN_MIN_COMMAND = 0.2; // min command outside the tolerance
constexpr double RETURN_TOLERANCE_DEG = 1.0;
constexpr unsigned long RETURN_SETTLE_MS = 200;
constexpr unsigned long RETURN_TIMEOUT_MS = 5000;

constexpr double DIRECTION_CHECK_MIN_COMMAND = 0.2; // wrong way > DIRECTION_CHECK_WRONG_DEG = lock
constexpr unsigned long DIRECTION_CHECK_GRACE_MS = 150;
constexpr unsigned long DIRECTION_CHECK_WINDOW_MS = 300;
constexpr double DIRECTION_CHECK_WRONG_DEG = 2.0;

static_assert(LIMIT_LEFT_DEG < 0.0 && LIMIT_RIGHT_DEG > 0.0, "home must lie between the limits");
static_assert(LIMIT_RIGHT_DEG - LIMIT_LEFT_DEG < 360.0, "limits less than a turn apart");
static_assert(RIGHT_COMMAND_SIGN == 1.0 || RIGHT_COMMAND_SIGN == -1.0, "a sign");
static_assert(RETURN_MIN_COMMAND > 0.0 && RETURN_MIN_COMMAND <= RETURN_MAX_COMMAND,
              "the return's minimum command fits under its maximum");

constexpr bool WEAK_MAGNET_LOCKS = false; // false = weak magnet only warns

constexpr unsigned long LOST_AFTER_MS = 100; // no good read this long = LOST until reboot

}
