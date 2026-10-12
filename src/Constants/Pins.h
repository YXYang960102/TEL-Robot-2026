#pragma once

// Gate for Enable from either operator source (src/IO/ManualControl.cpp):
// stays false until the real motor/sensor pin table has been checked on the
// robot against this file, ChassisConstants.h, DribblerConstants.h and
// ShooterConstants.h. While false, Enable is refused, so provisional pins
// can never be driven just because a comment says "provisional".
//
// TEL_HOST_TEST_PIN_TABLE_CONFIRMED exists only so the host test
// (test/manual_control_test.cpp) can exercise the armed paths; it is a hard
// compile error in any Arduino build, so firmware can't bypass the gate.
#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
#if defined(ARDUINO)
#error "TEL_HOST_TEST_PIN_TABLE_CONFIRMED is for host tests only"
#endif
constexpr bool PIN_TABLE_CONFIRMED = TEL_HOST_TEST_PIN_TABLE_CONFIRMED;
#elif defined(TEL_DRIVE_CONVEYOR_ONLY)
#ifndef TEL_WIRED_MANUAL_ONLY
#error "Partial pin confirmation is restricted to wired manual testing"
#endif
// Only drive/conveyor outputs exist in this test profile.
constexpr bool PIN_TABLE_CONFIRMED = true;
#else
constexpr bool PIN_TABLE_CONFIRMED = false;
#include "DribblerConstants.h"
static_assert(!PIN_TABLE_CONFIRMED || DribblerConstants::Conveyor::CONFIGURED,
              "Configure horizontal conveyor ESC signal before enabling outputs");
#endif

// Shooter
#define PIN_ESC_HORI 23
#define PIN_ESC_VER 24
#define PIN_FALCON 25

// Chassis and Dribbler pins moved to their own Constants files
// (ChassisConstants.h / DribblerConstants.h) as part of the 2026-10-11
// dev integration -- see docs/codex-handoff.md. Pins 40/41 (old BR/BL)
// are unused now that the chassis is two active wheels + passive casters,
// not four mecanum wheels; left free for the final pin table.

// Sensors
#define PIN_SAFE 28
#define PIN_POT A3

// Mechanism link (ELRS bridge via the robot-side STM32; replaces the
// former direct-SBUS wiring on this same hardware serial port). Not a
// GPIO pin number -- the Mega's hardware UART pins aren't user-mappable --
// but which Serial port this link uses is still a signal-wiring decision,
// so it's kept here with the rest of them. See src/IO/MechLink.h.
#define MECH_LINK_SERIAL_PORT Serial2

#include "ChassisConstants.h"
#include "DribblerConstants.h"
static_assert(ChassisConstants::RightDrive::SIGNAL_PIN != ChassisConstants::LeftDrive::SIGNAL_PIN,
              "Drive signals must use distinct pins");
static_assert(DribblerConstants::Conveyor::SIGNAL_PIN != ChassisConstants::RightDrive::SIGNAL_PIN &&
              DribblerConstants::Conveyor::SIGNAL_PIN != ChassisConstants::LeftDrive::SIGNAL_PIN,
              "Conveyor signal must not overlap drive signals");

#ifdef TEL_DRIVE_CONVEYOR_ONLY
static_assert(ChassisConstants::RightDrive::SIGNAL_PIN == 48 &&
              ChassisConstants::LeftDrive::SIGNAL_PIN == 46 &&
              DribblerConstants::Conveyor::SIGNAL_PIN == 45,
              "Reconfirm partial bench wiring before changing its pins");
#endif

#ifdef TEL_MANUAL_AXES_TEST
#include "ShooterConstants.h"
static_assert(ShooterConstants::Angle::LEFT_SIGNAL_PIN == 24 &&
              ShooterConstants::Rotate::SIGNAL_PIN == 23,
              "Reconfirm manual axis bench pins");
#endif

#ifdef TEL_MANUAL_FLYWHEEL_TEST
#ifndef TEL_MANUAL_AXES_TEST
#error "Flywheel bench requires manual axes profile"
#endif
static_assert(ShooterConstants::Flywheel::SIGNAL_PIN == 25, "Reconfirm flywheel pin");
#endif
