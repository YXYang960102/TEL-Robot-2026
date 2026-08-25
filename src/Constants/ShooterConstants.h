#pragma once

#include <stdint.h>

#include "../Control/ActuatorConfig.h"
#include "../Control/PidfController.h"
#include "../Sensors/AnalogPositionSensor.h"
#include "../Sensors/AS5600Encoder.h"

namespace ShooterConstants {

// Angle Constants
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

// Legacy all_robot_017 wiring: HIGH means the beam input is triggered.
constexpr uint8_t UP_LIMIT_PIN = 52;
constexpr uint8_t DOWN_LIMIT_PIN = 53;
constexpr bool LIMIT_TRIGGERED_HIGH = true;
constexpr bool LIMIT_USE_INTERNAL_PULLUP = false;
constexpr unsigned long LIMIT_DEBOUNCE_MS = 8;
constexpr DigitalLimitSwitchConfig UP_LIMIT_SWITCH(
    UP_LIMIT_PIN,
    LIMIT_TRIGGERED_HIGH,
    LIMIT_USE_INTERNAL_PULLUP,
    LIMIT_DEBOUNCE_MS);
constexpr DigitalLimitSwitchConfig DOWN_LIMIT_SWITCH(
    DOWN_LIMIT_PIN,
    LIMIT_TRIGGERED_HIGH,
    LIMIT_USE_INTERNAL_PULLUP,
    LIMIT_DEBOUNCE_MS);

constexpr int ENCODER_COUNTS_PER_REV = 4096;
constexpr int ENCODER_HALF_COUNTS_PER_REV = ENCODER_COUNTS_PER_REV / 2;
constexpr double COUNTS_TO_DEGREES = 360.0 / ENCODER_COUNTS_PER_REV;
constexpr int MAX_ACCEPTED_DELTA_COUNTS = 180;
constexpr unsigned long SENSOR_TIMEOUT_MS = 100;
constexpr AS5600EncoderConfig ENCODER(
    ENCODER_COUNTS_PER_REV,
    MAX_ACCEPTED_DELTA_COUNTS,
    SENSOR_TIMEOUT_MS);

// Keep position control disabled until both mechanical limits are measured.
constexpr bool POSITION_LIMITS_CALIBRATED = false;
constexpr long MIN_POSITION_COUNTS = 0;
constexpr long MAX_POSITION_COUNTS = 0;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = false;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = false;
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


// Rotate Constants
namespace Rotate {

enum class Action : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

// pin pwm invert
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

// Position control remains disabled until all three potentiometer points are measured.
//sensor raw position values for left, center, right
constexpr uint8_t POSITION_SENSOR_PIN = A3;
constexpr int SENSOR_VALID_MINIMUM_RAW = 5;
constexpr int SENSOR_VALID_MAXIMUM_RAW = 1018;
constexpr double SENSOR_FILTER_ALPHA = 0.25;
constexpr unsigned long SENSOR_SAMPLE_PERIOD_MS = 5;
constexpr unsigned long SENSOR_TIMEOUT_MS = 100;
constexpr AnalogPositionSensorConfig POSITION_SENSOR(
    POSITION_SENSOR_PIN,
    SENSOR_VALID_MINIMUM_RAW,
    SENSOR_VALID_MAXIMUM_RAW,
    SENSOR_FILTER_ALPHA,
    SENSOR_SAMPLE_PERIOD_MS,
    SENSOR_TIMEOUT_MS);

constexpr bool POSITION_CALIBRATED = false;
constexpr int LEFT_POSITION_RAW = 0;
constexpr int CENTER_POSITION_RAW = 0;
constexpr int RIGHT_POSITION_RAW = 0;
constexpr double LEFT_POSITION_DEGREES = 0.0;
constexpr double CENTER_POSITION_DEGREES = 0.0;
constexpr double RIGHT_POSITION_DEGREES = 0.0;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = false;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = false;
constexpr PositionLimitConfig SOFT_LIMIT(
    FORWARD_SOFT_LIMIT_ENABLED,
    REVERSE_SOFT_LIMIT_ENABLED,
    RIGHT_POSITION_RAW,
    LEFT_POSITION_RAW);

// Legacy all_robot_017 wiring: HIGH means the beam input is triggered.
constexpr uint8_t LEFT_LIMIT_PIN = 43;
constexpr uint8_t RIGHT_LIMIT_PIN = 42;
constexpr bool LIMIT_TRIGGERED_HIGH = true;
constexpr bool LIMIT_USE_INTERNAL_PULLUP = false;
constexpr unsigned long LIMIT_DEBOUNCE_MS = 8;
constexpr DigitalLimitSwitchConfig LEFT_LIMIT_SWITCH(
    LEFT_LIMIT_PIN,
    LIMIT_TRIGGERED_HIGH,
    LIMIT_USE_INTERNAL_PULLUP,
    LIMIT_DEBOUNCE_MS);
constexpr DigitalLimitSwitchConfig RIGHT_LIMIT_SWITCH(
    RIGHT_LIMIT_PIN,
    LIMIT_TRIGGERED_HIGH,
    LIMIT_USE_INTERNAL_PULLUP,
    LIMIT_DEBOUNCE_MS);

constexpr double kP = 0.0;
constexpr double kI = 0.0;
constexpr double kD = 0.0;
constexpr double kIZone = 0.0;
constexpr double kFF = 0.0;
constexpr PidfConfig PIDF(kP, kI, kD, kIZone, kFF);

constexpr double PROFILE_CRUISE_COMMAND = 1.0;
constexpr double PROFILE_MINIMUM_APPROACH_RATIO = 0.10;
constexpr double PROFILE_DECELERATION_FRACTION = 0.20;
constexpr int READY_TOLERANCE_RAW = 2;
constexpr unsigned long READY_SETTLE_TIME_MS = 100;
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long MAX_CONTROL_GAP_MS = 100;
constexpr int SETPOINT_RESET_THRESHOLD_RAW = 10;
constexpr double MIN_NORMALIZED_OUTPUT = -1.0;
constexpr double MAX_NORMALIZED_OUTPUT = 1.0;
constexpr double MIN_INTEGRAL_OUTPUT = -0.25;
constexpr double MAX_INTEGRAL_OUTPUT = 0.25;

}

// Flywheel Constants
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
static_assert(!Rotate::POSITION_CALIBRATED ||
                  Rotate::LEFT_POSITION_RAW < Rotate::CENTER_POSITION_RAW,
              "Rotate left calibration must be below center");
static_assert(!Rotate::POSITION_CALIBRATED ||
                  Rotate::CENTER_POSITION_RAW < Rotate::RIGHT_POSITION_RAW,
              "Rotate center calibration must be below right");
static_assert(Rotate::PROFILE_DECELERATION_FRACTION > 0.0 &&
              Rotate::PROFILE_DECELERATION_FRACTION <= 1.0,
              "Invalid rotate deceleration fraction");
static_assert(Rotate::PROFILE_MINIMUM_APPROACH_RATIO >= 0.0 &&
              Rotate::PROFILE_MINIMUM_APPROACH_RATIO <= 1.0,
              "Invalid rotate minimum approach ratio");
static_assert(Flywheel::MIN_PULSE_US < Flywheel::NEUTRAL_US, "Invalid flywheel minimum PWM");
static_assert(Flywheel::NEUTRAL_US < Flywheel::MAX_PULSE_US, "Invalid flywheel maximum PWM");
static_assert(
    !(Angle::FORWARD_SOFT_LIMIT_ENABLED ||
      Angle::REVERSE_SOFT_LIMIT_ENABLED) ||
        Angle::POSITION_LIMITS_CALIBRATED,
    "Angle soft limits require calibrated positions");
static_assert(
    !(Angle::FORWARD_SOFT_LIMIT_ENABLED ||
      Angle::REVERSE_SOFT_LIMIT_ENABLED) ||
        Angle::SOFT_LIMIT.reverseLimit < Angle::SOFT_LIMIT.forwardLimit,
    "Angle soft limits are reversed");
static_assert(
    !(Rotate::FORWARD_SOFT_LIMIT_ENABLED ||
      Rotate::REVERSE_SOFT_LIMIT_ENABLED) ||
        Rotate::POSITION_CALIBRATED,
    "Rotate soft limits require calibrated positions");
static_assert(
    !(Rotate::FORWARD_SOFT_LIMIT_ENABLED ||
      Rotate::REVERSE_SOFT_LIMIT_ENABLED) ||
        Rotate::SOFT_LIMIT.reverseLimit < Rotate::SOFT_LIMIT.forwardLimit,
    "Rotate soft limits are reversed");

}
