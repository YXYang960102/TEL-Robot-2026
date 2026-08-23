#pragma once

#include <stdint.h>

namespace DribblerConstants {

namespace Feeder {
enum class Action : uint8_t {
    STOP = 0,
    FEED = 1
};

constexpr uint8_t PWM_PIN = 2;
constexpr uint8_t ENABLE_PIN = 51;
constexpr bool ENABLE_ACTIVE_HIGH = true;
constexpr int MIN_PWM = 0;
constexpr int MAX_PWM = 255;
constexpr int DEFAULT_FEED_PWM = 180;
constexpr double DEFAULT_FEED_COMMAND =
    DEFAULT_FEED_PWM / static_cast<double>(MAX_PWM);
}

namespace ExitSensor {
constexpr uint8_t SIGNAL_PIN = 45;
constexpr bool BLOCKED_LOW = true;
constexpr unsigned long DEBOUNCE_MS = 8;
}

namespace Request {
constexpr int MIN_COUNT = 0;
constexpr int MAX_COUNT = 12;
}

static_assert(
    Feeder::DEFAULT_FEED_PWM >= Feeder::MIN_PWM &&
        Feeder::DEFAULT_FEED_PWM <= Feeder::MAX_PWM,
    "Invalid dribbler feed PWM");

}
