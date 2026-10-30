#pragma once

#include <stdint.h>

// AS5600 magnetic angle sensor on the turret motor shaft (2026-10-27,
// Jeremy: fitted, I2C on the Mega's D20 SDA / D21 SCL). src/Sensors/
// TurretAngle reads it (TEL slots 42-44, Dashboard); since 2026-10-31 the
// absolute turret angle is kept in EEPROM (TurretStore.h) and the turret
// is held inside soft limits (src/Shooter/TurretGuard.h).
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

// The angle's sign: + = turret turned right (the same sign as the camera's
// tx). Measured 2026-10-31: turning right read negative with false.
constexpr bool INVERTED = true;

// ---- Absolute position (2026-10-31, Jeremy; AS5600 only, no switch, no
// end stop to push against: past the limits below the mechanism breaks).
// 0 deg = the home point, set once with the Dashboard's 「設為歸位點」
// (wired TZERO, disarmed). Saved to EEPROM whenever the turret has moved
// SAVE_STEP_DEG, or stood still SAVE_STILL_MS after a move of at least
// SAVE_MIN_CHANGE_DEG. At boot the saved angle + the AS5600's change since
// gives the angle back, if the turret was not turned more than ~36 deg by
// hand while off. A restored angle further than RESTORE_MARGIN_DEG outside
// the limits is not trusted (the turret stays locked until set again).
constexpr double SAVE_STEP_DEG = 10.0;
constexpr unsigned long SAVE_STILL_MS = 500;
constexpr double SAVE_STILL_DEG = 0.3;      // "still": moved less than this
constexpr double SAVE_MIN_CHANGE_DEG = 0.2;
constexpr double RESTORE_MARGIN_DEG = 10.0;

// Soft limits (Jeremy 2026-10-31, from the home point; the turret turns
// further but breaks things there). Measured: right -36.0 / left +136.0
// with INVERTED false, i.e. +36 / -136 now. Commands that would go past
// are dropped; within LIMIT_SLOW_ZONE_DEG of a limit the outward command
// is scaled down (to no less than LIMIT_SLOW_MIN_COMMAND) so the turret
// doesn't coast past it.
constexpr double LIMIT_RIGHT_DEG = 36.0;
constexpr double LIMIT_LEFT_DEG = -136.0;
constexpr double LIMIT_SLOW_ZONE_DEG = 10.0;
constexpr double LIMIT_SLOW_MIN_COMMAND = 0.15;
// The turret coasts after the command drops: the limits are checked
// against where it would be LIMIT_STOP_LOOKAHEAD_S later at its present
// speed (measured from the AS5600). PROVISIONAL: on a PC model of
// 120 deg/s with a 0.12 s lag, full right held from 0 stopped at +44.7 deg
// without it. Tune from how far the real turret coasts.
constexpr double LIMIT_STOP_LOOKAHEAD_S = 0.15;

// Turret command sign that turns it right. From the vision-follow
// direction validated on the bench (TuningConstants::Rotate): a target
// right of center (tx > 0) gives a negative command and the turret turns
// toward it. Checked live: a command that keeps turning the turret the
// other way locks it (DIRECTION_CHECK_*).
constexpr double RIGHT_COMMAND_SIGN = -1.0;

// Back to 0 deg on the first Enable after power-up (and RB on the wired
// Dashboard): a separate P(IDF) on the angle, its own gains. PROVISIONAL.
// Done within RETURN_TOLERANCE_DEG for RETURN_SETTLE_MS, or after
// RETURN_TIMEOUT_MS (the position is known anyway; it just stops trying).
constexpr double RETURN_kP = 0.03;          // command per degree
constexpr double RETURN_MAX_COMMAND = 0.4;
constexpr double RETURN_TOLERANCE_DEG = 1.0;
constexpr unsigned long RETURN_SETTLE_MS = 200;
constexpr unsigned long RETURN_TIMEOUT_MS = 5000;

// Direction check: a command of at least DIRECTION_CHECK_MIN_COMMAND held
// one way that moved the turret DIRECTION_CHECK_WRONG_DEG the other way
// (measured over DIRECTION_CHECK_WINDOW_MS windows, after a
// DIRECTION_CHECK_GRACE_MS grace for coasting when the direction changes)
// = RIGHT_COMMAND_SIGN or INVERTED is wrong: the limits would push the
// wrong way, so the turret locks (until reboot).
constexpr double DIRECTION_CHECK_MIN_COMMAND = 0.2;
constexpr unsigned long DIRECTION_CHECK_GRACE_MS = 150;
constexpr unsigned long DIRECTION_CHECK_WINDOW_MS = 300;
constexpr double DIRECTION_CHECK_WRONG_DEG = 2.0;

static_assert(LIMIT_LEFT_DEG < 0.0 && LIMIT_RIGHT_DEG > 0.0, "home must lie between the limits");
static_assert(LIMIT_RIGHT_DEG - LIMIT_LEFT_DEG < 360.0, "limits less than a turn apart");
static_assert(RIGHT_COMMAND_SIGN == 1.0 || RIGHT_COMMAND_SIGN == -1.0, "a sign");

// No good read for this long and the turns count can't be trusted any
// more (the magnet may have passed half a turn unseen): reported as LOST
// until a reboot.
constexpr unsigned long LOST_AFTER_MS = 100;

}  // namespace TurretAngleConstants
