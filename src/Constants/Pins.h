#pragma once

// Gate for the wired Dashboard's Enable command (src/IO/WiredControl.cpp):
// stays false until the real motor/sensor pin table has been checked on the
// robot against this file, ChassisConstants.h, DribblerConstants.h and
// ShooterConstants.h. While false, Enable is refused, so provisional pins
// can never be driven just because a comment says "provisional".
//
// TEL_HOST_TEST_PIN_TABLE_CONFIRMED exists only so the host test
// (test/wired_control_test.cpp) can exercise the armed paths; it is a hard
// compile error in any Arduino build, so firmware can't bypass the gate.
#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
#if defined(ARDUINO)
#error "TEL_HOST_TEST_PIN_TABLE_CONFIRMED is for host tests only"
#endif
constexpr bool PIN_TABLE_CONFIRMED = TEL_HOST_TEST_PIN_TABLE_CONFIRMED;
#else
constexpr bool PIN_TABLE_CONFIRMED = false;
#endif

// Shooter
#define PIN_ESC_HORI 29
#define PIN_ESC_VER 27
#define PIN_FALCON 49

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
