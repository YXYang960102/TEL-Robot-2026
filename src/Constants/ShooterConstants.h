#pragma once

#include <stdint.h>

#include "../Control/ActuatorConfig.h"
#include "../Control/PidfController.h"
#include "../Sensors/AS5600Encoder.h"

namespace ShooterConstants {

namespace Angle {

enum class Action : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

constexpr uint8_t LEFT_SIGNAL_PIN = 27;
constexpr uint8_t RIGHT_SIGNAL_PIN = 26;

constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = true;
constexpr ServoMotorConfig LEFT_MOTOR(
    LEFT_INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);
constexpr ServoMotorConfig RIGHT_MOTOR(
    RIGHT_INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

constexpr int ENCODER_COUNTS_PER_REV = 4096;
constexpr int ENCODER_HALF_COUNTS_PER_REV = ENCODER_COUNTS_PER_REV / 2;
constexpr double COUNTS_TO_DEGREES = 360.0 / ENCODER_COUNTS_PER_REV;
constexpr int MAX_ACCEPTED_DELTA_COUNTS = 180;
constexpr unsigned long SENSOR_TIMEOUT_MS = 100;
constexpr AS5600EncoderConfig ENCODER(
    ENCODER_COUNTS_PER_REV,
    MAX_ACCEPTED_DELTA_COUNTS,
    SENSOR_TIMEOUT_MS);

constexpr long MIN_POSITION_COUNTS = 0;
constexpr long MAX_POSITION_COUNTS = 1024;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = true;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = true;
constexpr PositionLimitConfig SOFT_LIMIT(
    FORWARD_SOFT_LIMIT_ENABLED,
    REVERSE_SOFT_LIMIT_ENABLED,
    MAX_POSITION_COUNTS,
    MIN_POSITION_COUNTS);

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;
constexpr PidfConfig PIDF(kP, kI, kD, kIZone, kFF);

constexpr int MAX_CLOSED_LOOP_OFFSET_US = 80;
constexpr long READY_TOLERANCE_COUNTS = 1;
constexpr double READY_VELOCITY_TOLERANCE_COUNTS_PER_SECOND = 50.0;
constexpr unsigned long READY_SETTLE_TIME_MS = 100;
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long MAX_CONTROL_GAP_MS = 100;
constexpr long SETPOINT_RESET_THRESHOLD_COUNTS = 128;
constexpr double MIN_NORMALIZED_OUTPUT = -1.0;
constexpr double MAX_NORMALIZED_OUTPUT = 1.0;
constexpr double MIN_INTEGRAL_OUTPUT = -0.25;
constexpr double MAX_INTEGRAL_OUTPUT = 0.25;
constexpr double FEEDFORWARD_REFERENCE = 1.0;

}

namespace Rotate {

enum class Action : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

constexpr uint8_t SIGNAL_PIN = 29;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr bool INVERTED = false;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;
constexpr PidfConfig PIDF(kP, kI, kD, kIZone, kFF);

}

namespace Flywheel {

enum class Action : uint8_t {
    STOP = 0,
    FORWARD = 1
};

constexpr uint8_t SIGNAL_PIN = 49;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr bool INVERTED = false;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;
constexpr PidfConfig PIDF(kP, kI, kD, kIZone, kFF);

}

namespace Legacy {

constexpr int HALF_AUTO_VERTICAL[4][8] = {
    {1212, 1200, 1212, 1212, 1212, 1212, 1212},
    {1915, 1497, 1374, 1625, 1410, 1212, 1212},
    {1957, 2230, 2075, 2684, 2326, 1608, 2307},
    {2515, 2635, 2667, 3492, 2918, 2380, 2918}
};

}

static_assert(Angle::MIN_PULSE_US < Angle::NEUTRAL_US, "Invalid angle minimum PWM");
static_assert(Angle::NEUTRAL_US < Angle::MAX_PULSE_US, "Invalid angle maximum PWM");
static_assert(Rotate::MIN_PULSE_US < Rotate::NEUTRAL_US, "Invalid rotate minimum PWM");
static_assert(Rotate::NEUTRAL_US < Rotate::MAX_PULSE_US, "Invalid rotate maximum PWM");
static_assert(Flywheel::MIN_PULSE_US < Flywheel::NEUTRAL_US, "Invalid flywheel minimum PWM");
static_assert(Flywheel::NEUTRAL_US < Flywheel::MAX_PULSE_US, "Invalid flywheel maximum PWM");
static_assert(
    Angle::SOFT_LIMIT.reverseLimit < Angle::SOFT_LIMIT.forwardLimit,
    "Angle soft limits are reversed");

}
