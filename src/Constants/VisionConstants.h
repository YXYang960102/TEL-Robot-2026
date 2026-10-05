#pragma once

#include <stddef.h>

namespace VisionConstants {

namespace Transport {
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long PACKET_TIMEOUT_MS = 300;
constexpr size_t PACKET_MAX_CHARS = 80;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 100;
constexpr int PROTOCOL_VERSION = 1;
}

namespace Validation {
constexpr double MAX_ABS_TX = 400.0;
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
