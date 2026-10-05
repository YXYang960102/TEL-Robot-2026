#pragma once
#include <stdint.h>

// Standalone Orin-vision-follow bench for the A5 horizontal servo. Does not
// touch production Shooter/Vision. Self-contained (no longer shares
// RotateBenchConstants.h with the manual-jog bench, which was removed
// 2026-10-06 once this vision-follow path became the maintained one).
namespace VisionRotateBenchConstants {
constexpr uint8_t SIGNAL_PIN = 59; // Mega A5 is digital pin 59.
constexpr uint8_t LEFT_LIMIT_PIN = 43;
constexpr uint8_t RIGHT_LIMIT_PIN = 42;
constexpr int NEUTRAL_US = 1500;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250; // USB (browser tab) watchdog
constexpr uint32_t TELEMETRY_MS = 100;
constexpr unsigned long BAUD = 115200; // USB Serial, for the browser page

// tx>0 (target right of center) was observed on real hardware 2026-09-25 to
// rotate the servo away from center instead of toward it.
constexpr bool INVERTED = true;

constexpr int PROTOCOL_VERSION = 1; // must match Orin CONTROLLER_PROTOCOL_VERSION
constexpr unsigned long VISION_BAUD = 115200; // Serial1, must match Orin SERIAL_BAUD

constexpr int DEADBAND_PX = 20; // Tightened from 25 (2026-10-05): real-hardware
                                 // testing found overall follow speed too slow;
                                 // picked the more conservative of the two
                                 // values Jeremy offered (20 or 15). Inside
                                 // this: stop.
constexpr int SLOW_ZONE_PX = 70; // Narrowed from 80 (2026-10-05), same reason
                                  // — reaches full speed sooner. Between
                                  // DEADBAND_PX and this:
                                  // creep instead of full speed. Added
                                  // 2026-09-25 because pure on/off bang-bang
                                  // control has no way to slow down before
                                  // the deadband, so control-loop latency +
                                  // servo momentum cause it to overshoot
                                  // center and correct back the other way —
                                  // worse at higher FOLLOW_JOG_OFFSET_US,
                                  // exactly as observed on real hardware.
constexpr int FOLLOW_CREEP_OFFSET_US = 150; // Speed used inside the slow zone.
constexpr int FOLLOW_JOG_OFFSET_US = 300; // Speed used beyond the slow zone.
                                           // Raised from 150 (2026-09-25) after
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
static_assert(FOLLOW_CREEP_OFFSET_US > 0 && FOLLOW_CREEP_OFFSET_US <= FOLLOW_JOG_OFFSET_US,
              "Creep speed must be positive and no faster than full speed");
static_assert(DEADBAND_PX > 0, "Deadband must be positive");
static_assert(SLOW_ZONE_PX > DEADBAND_PX, "Slow zone must be outside the deadband");
}
