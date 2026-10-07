#pragma once

// Shooter
#define PIN_ESC_HORI 29
#define PIN_ESC_VER 27
#define PIN_FALCON 49

// Dribbler
#define PIN_DRIBBLE_UP 2
#define PIN_DRIBBLE_DOWN 51

// Sensors
#define PIN_SAFE 28
#define PIN_POT A3

// Mechanism link (ELRS bridge via the robot-side STM32; replaces the
// former direct-SBUS wiring on this same hardware serial port). Not a
// GPIO pin number -- the Mega's hardware UART pins aren't user-mappable --
// but which Serial port this link uses is still a signal-wiring decision,
// so it's kept here with the rest of them. See src/IO/MechLink.h.
#define MECH_LINK_SERIAL_PORT Serial2
