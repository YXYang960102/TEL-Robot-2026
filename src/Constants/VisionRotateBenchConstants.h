#pragma once
#include <stdint.h>
#include "RotateBenchConstants.h"

// Standalone Orin-vision-follow bench for the A5 horizontal servo. Parallel
// to mega_rotate_only_test, does not touch it or production Vision/Shooter.
namespace VisionRotateBenchConstants {
// Reuse the mechanism constants already verified on the manual-jog bench —
// same motor, same limit switches, no reason to duplicate/risk drift.
using RotateBenchConstants::SIGNAL_PIN;
using RotateBenchConstants::LEFT_LIMIT_PIN;
using RotateBenchConstants::RIGHT_LIMIT_PIN;
using RotateBenchConstants::NEUTRAL_US;
using RotateBenchConstants::COMMAND_TIMEOUT_MS; // USB (browser tab) watchdog
using RotateBenchConstants::TELEMETRY_MS;
using RotateBenchConstants::BAUD; // USB Serial, for the browser page

// Independent of RotateBenchConstants::INVERTED (the manual-jog bench's
// already-verified-correct setting) — vision tx>0 (target right of center)
// was observed on real hardware 2026-09-25 to rotate the servo away from
// center instead of toward it, so this bench's sign is flipped on its own.
constexpr bool INVERTED = true;

constexpr int PROTOCOL_VERSION = 1; // must match Orin CONTROLLER_PROTOCOL_VERSION
constexpr unsigned long VISION_BAUD = 115200; // Serial1, must match Orin SERIAL_BAUD

constexpr int DEADBAND_PX = 25; // Provisional: tune against real target jitter.
constexpr int FOLLOW_JOG_OFFSET_US = 300; // Raised from 150 (2026-09-25) after
                                           // the first successful vision-follow
                                           // test with corrected direction.
                                           // Still below the 500us manual-jog
                                           // ceiling; staged the same way
                                           // JOG_OFFSET_US was raised — verify
                                           // stability at this speed before
                                           // going further.
constexpr uint32_t VISION_TIMEOUT_MS = 500; // Serial1 (Orin) link watchdog,
                                             // independent of COMMAND_TIMEOUT_MS.
constexpr uint32_t MEGA_HEARTBEAT_INTERVAL_MS = 200; // well under Orin's 1.0s
                                                       // MEGA_HEARTBEAT_TIMEOUT_SECONDS

static_assert(FOLLOW_JOG_OFFSET_US > 0 && FOLLOW_JOG_OFFSET_US <= 500,
              "Bench authority exceeded");
static_assert(DEADBAND_PX > 0, "Deadband must be positive");
}
