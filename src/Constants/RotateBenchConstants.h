#pragma once
#include <stdint.h>

// Standalone A5 bench only; production Shooter pin mapping is unchanged.
namespace RotateBenchConstants {
// Mega A5 is digital pin 59. Keep this header Arduino-independent for tests.
constexpr uint8_t SIGNAL_PIN = 59;
constexpr uint8_t LEFT_LIMIT_PIN = 43;
constexpr uint8_t RIGHT_LIMIT_PIN = 42;
constexpr int NEUTRAL_US = 1500; // Provisional: verify physical stop before jogging.
constexpr int JOG_OFFSET_US = 500; // 100% of the historical 500 us half-span
                                    // (full authority: 1500+-500 = the servo's
                                    // full attached 1000-2000us range); NOT RPM.
                                    // Jeremy raised this from 250us (50%) to full
                                    // speed on 2026-09-19 after confirming the
                                    // Mega<->Orin wiring separately; re-verify
                                    // direction, stopping, and limit behavior at
                                    // full authority before trusting it unattended.
constexpr bool INVERTED = false;
constexpr uint32_t COMMAND_TIMEOUT_MS = 250; // Loss-of-link watchdog; stays even
                                              // without a jog cap (see below).
constexpr uint32_t TELEMETRY_MS = 100;
constexpr unsigned long BAUD = 115200;
static_assert(NEUTRAL_US >= 1400 && NEUTRAL_US <= 1600, "Review neutral calibration");
static_assert(JOG_OFFSET_US > 0 && JOG_OFFSET_US <= 500, "Bench authority exceeded");
}
