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

// Where each pin lives: every motor/sensor pin is defined exactly once, in
// its subsystem's Constants file (ChassisConstants.h, DribblerConstants.h,
// ShooterConstants.h) -- change it there and nowhere else. The only check
// is in src/main.cpp: two outputs on the same pin, or an output on a
// hardware serial pin, fails the build. (Removed 2026-10-14 at Jeremy's
// request: the second copy of each bench pin that had to be edited in step
// here, and the unused legacy PIN_ESC_HORI/VER, PIN_FALCON, PIN_SAFE and
// PIN_POT defines.)

// Mechanism link (ELRS bridge via the robot-side STM32; replaces the
// former direct-SBUS wiring on this same hardware serial port). Not a
// GPIO pin number -- the Mega's hardware UART pins aren't user-mappable --
// but which Serial port this link uses is still a signal-wiring decision,
// so it's kept here with the rest of them. See src/IO/MechLink.h.
#define MECH_LINK_SERIAL_PORT Serial2

#ifdef TEL_MANUAL_FLYWHEEL_TEST
#ifndef TEL_MANUAL_AXES_TEST
#error "Flywheel bench requires manual axes profile"
#endif
#endif
