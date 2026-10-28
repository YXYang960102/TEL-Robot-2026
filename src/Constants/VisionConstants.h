#pragma once

#include <stddef.h>

namespace VisionConstants {

namespace Transport {
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long PACKET_TIMEOUT_MS = 300;
constexpr size_t PACKET_MAX_CHARS = 80;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 100;
// v2 (2026-10-16): adds MEGA_TARGET,2,<vision id> (the operator's chosen
// hole), sent on change and with every heartbeat. Must match the Orin's
// YOLO_Detect_single vision_main.main(); a v1 peer is refused both ways,
// so the Mega and the Orin are deployed together.
// v3 (2026-10-27): the frame gains a sixth field, the Orin's reliably
// detected hole ids as a bitmask (bit id - 1); valid = 2 is a PREDICTED
// position of the selected hole that is out of view (aim only: the turret
// turns toward it, never a lock or a shot); valid = 1 may come with
// distance 0 (hole seen, depth failed this frame); MEGA_TARGET may carry
// the bonus holes 1-3. Deploy with YOLO_Detect_single protocol v3.
constexpr int PROTOCOL_VERSION = 3;
// Follow predicted positions at most this long without the hole really
// coming into view (no turret angle limits yet: never turn on forever).
constexpr unsigned long PREDICTED_FOLLOW_MAX_MS = 3000;
}

namespace Validation {
constexpr double MAX_ABS_TX = 400.0;
// A predicted position may lie up to a frame width outside the image; it
// is accepted up to this and then clamped to MAX_ABS_TX for the turret.
constexpr double MAX_ABS_PREDICTED_TX = 4000.0;
constexpr long MAX_SEEN_MASK = 0x0FFF;
constexpr double MAX_ABS_TY = 1000.0;
constexpr double MAX_DISTANCE_MM = 20000.0;
constexpr int MIN_TARGET_ID = 0;
constexpr int MAX_TARGET_ID = 12;
}

namespace Legacy {
constexpr double Y_RANGE_SET[] = {
    96.1, 108.3, 135.6, 189.5, 157.0, 122.0, 170.5
};
constexpr int Y_COUNT = sizeof(Y_RANGE_SET) / sizeof(Y_RANGE_SET[0]);
constexpr double DERIVATIVE_TIME_SECONDS = 0.08;
constexpr double FILTER_TIME_CONSTANT_SECONDS = 0.10;
constexpr double MAX_X_VELOCITY = 800.0;
}

}
