#pragma once

// Gate for Enable from either operator source (src/IO/ManualControl.cpp):
// while false, Enable is refused (ENABLE_REJECTED_PINS_UNCONFIRMED), so
// unchecked pins can never be driven.
//
// CONFIRMED 2026-10-16 (Jeremy: these are the competition pins):
//   chassis right D48, left D46 (ChassisConstants.h)
//   turret D23, elevation D24, Falcon D25 (ShooterConstants.h)
//   feed tower trigger D22 (DribblerConstants.h; was D21 until 2026-10-17)
//   Orin Serial1 D18/D19, robot ELRS bridge Serial2 D16/D17
// If the wiring changes, change the pin in its Constants file (main.cpp's
// pincheck catches two outputs on one pin or an output on a serial pin);
// set this back to false if a pin is ever in doubt.
//
// TEL_HOST_TEST_PIN_TABLE_CONFIRMED exists only so the host test
// (test/manual_control_test.cpp) can exercise both the armed paths (true)
// and the refusal (false); it is a hard compile error in any Arduino build.
#include "BuildProfile.h"
#include "DribblerConstants.h"

#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
#if defined(ARDUINO)
#error "TEL_HOST_TEST_PIN_TABLE_CONFIRMED is for host tests only"
#endif
constexpr bool PIN_TABLE_CONFIRMED = TEL_HOST_TEST_PIN_TABLE_CONFIRMED;
#else
constexpr bool PIN_TABLE_CONFIRMED = true;
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
