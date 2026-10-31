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
// Magnet strength (2026-10-31, for setting the magnet gap on the bench):
// AGC (0x1A) = the chip's gain, 0-255 in 5 V mode / 0-128 in 3.3 V mode,
// higher = weaker field (magnet further away); MAGNITUDE (0x1B-0x1C,
// 12 bits) = the field it sees, higher = stronger. Read every
// AGC_READ_PERIOD_MS (TEL slots 50-51, Dashboard).
constexpr uint8_t REG_AGC = 0x1A;
constexpr unsigned long AGC_READ_PERIOD_MS = 100;
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

// Master switch for the turret guard (src/Shooter/TurretGuard.h: soft
// limits, return to 0 deg, locks). Jeremy 2026-10-31: was OFF until the
// magnet read reliably; ON again after the refitted magnet (magnitude
// ~1030), the home point set and the boot-window test passed. false = the
// turret runs as before the AS5600 (no limits: mind +38.5 / -166 deg by
// eye); the AS5600 is still read and shown either way.
constexpr bool GUARD_ENABLED = true;

// ---- Home point, by hand (2026-10-31, Jeremy; AS5600 only, no switch, no
// end stop to push against: past the limits below the mechanism breaks).
// The home point's raw reading is saved once in EEPROM with the
// Dashboard's 「設為歸位點」 (wired TZERO, disarmed). Before power-up the
// turret is turned by hand back to within HOME_WINDOW_DEG of its home mark
// (after a match, after work on the robot); at boot
//   angle = fold(raw - home raw) / 56.11   (one magnet turn = 73 deg)
// is the turret's real angle, and the first Enable turns it back to 0.
// Outside the window the turret stays locked (turn it back, reboot).
// NOT caught: a turret left a whole magnet turn away (about 73 or 146
// deg) reads like one near home -- turn it back to the mark every time.
constexpr double HOME_WINDOW_DEG = 25.0; // 20 until 2026-10-31 (Jeremy: +-25)
static_assert(HOME_WINDOW_DEG > 0.0 && HOME_WINDOW_DEG < 36.0,
              "the window must lie within half a magnet turn (36.5 deg)");

// Soft limits (Jeremy 2026-10-31, from the home point; the turret turns
// further but breaks things there). Measured: right -36.0 / left +136.0
// with INVERTED false, i.e. +36 / -136; 2026-10-31 later, after the belt
// skip and with the magnet refitted (home raw 1414): +38.5 deg (raw 3350)
// and -166 deg (raw 2528), Jeremy: these are the limits wanted. Both
// raws match 56.11 counts / deg to within 8 counts. Commands that would go past
// are dropped; within LIMIT_SLOW_ZONE_DEG of a limit the outward command
// is scaled down (to no less than LIMIT_SLOW_MIN_COMMAND) so the turret
// doesn't coast past it.
constexpr double LIMIT_RIGHT_DEG = 38.5;
constexpr double LIMIT_LEFT_DEG = -166.0;
constexpr double LIMIT_SLOW_ZONE_DEG = 10.0;
// 0.2 (was 0.15, 2026-10-31): the robot's turret (small pulley driving
// the big one) does not move below about 0.16 -- see RETURN_MIN_COMMAND.
constexpr double LIMIT_SLOW_MIN_COMMAND = 0.2;
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
// Outside the tolerance the command is at least this (2026-10-31, robot:
// from +20 deg the return stopped at 5.3 deg, where 0.03 x 5.3 = 0.16 no
// longer turned the turret). PROVISIONAL: raise it if the turret still
// stops short, lower it if it overshoots back and forth around 0.
constexpr double RETURN_MIN_COMMAND = 0.2;
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
static_assert(RETURN_MIN_COMMAND > 0.0 && RETURN_MIN_COMMAND <= RETURN_MAX_COMMAND,
              "the return's minimum command fits under its maximum");

// AS5600 "magnet too weak" (ML: its gain is at maximum). Jeremy
// 2026-10-31: the robot's magnet reads that way but the raw angle is
// smooth, so it is only a warning (Dashboard) -- the turret still runs and
// the home point can be set. No magnet at all (MD = 0), a failed read or
// "too strong" still lock the turret. Set true once the magnet sits at
// 0.5-3 mm, so a magnet working loose stops the turret.
constexpr bool WEAK_MAGNET_LOCKS = false;

// No good read for this long and the turns count can't be trusted any
// more (the magnet may have passed half a turn unseen): reported as LOST
// until a reboot.
constexpr unsigned long LOST_AFTER_MS = 100;

}  // namespace TurretAngleConstants
