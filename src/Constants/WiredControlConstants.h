#pragma once

#include <stdint.h>

// Production wired Dashboard command protocol (USB Serial, separate from
// Serial2/MechLink's wireless ELRS link). Added 2026-10-11 dev integration,
// see docs/codex-handoff.md. Values match the existing bench convention
// (ShooterBenchConstants::COMMAND_TIMEOUT_MS/COMMAND_BUFFER_SIZE) rather
// than inventing new ones.
namespace WiredControlConstants {

constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr size_t COMMAND_BUFFER_SIZE = 24;

}
