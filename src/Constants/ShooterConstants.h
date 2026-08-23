#pragma once

#include "../Control/PidfController.h"

namespace ShooterConst {

struct ServoMotorConfig {
    constexpr ServoMotorConfig(
        bool invertedValue,
        int reversePulseUs,
        int neutralPulseUs,
        int forwardPulseUs)
        : inverted(invertedValue),
          reverseUs(reversePulseUs),
          neutralUs(neutralPulseUs),
          forwardUs(forwardPulseUs) {}

    bool inverted;
    int reverseUs;
    int neutralUs;
    int forwardUs;
};

struct PositionLimitConfig {
    constexpr PositionLimitConfig(
        bool forwardEnabledValue,
        bool reverseEnabledValue,
        long forwardLimitCountsValue,
        long reverseLimitCountsValue)
        : forwardEnabled(forwardEnabledValue),
          reverseEnabled(reverseEnabledValue),
          forwardLimitCounts(forwardLimitCountsValue),
          reverseLimitCounts(reverseLimitCountsValue) {}

    bool forwardEnabled;
    bool reverseEnabled;
    long forwardLimitCounts;
    long reverseLimitCounts;
};

constexpr double kRotatekP = 0.0;
constexpr double kRotatekI = 0.0;
constexpr double kRotatekD = 0.0;
constexpr double kRotatekIZone = 0.0;
constexpr double kRotatekFF = 0.0;

constexpr double kAnglekP = 0.0;
constexpr double kAnglekI = 0.0;
constexpr double kAnglekD = 0.0;
constexpr double kAnglekIZone = 0.0;
constexpr double kAnglekFF = 0.0;

constexpr double kFlywheelkP = 0.0;
constexpr double kFlywheelkI = 0.0;
constexpr double kFlywheelkD = 0.0;
constexpr double kFlywheelkIZone = 0.0;
constexpr double kFlywheelkFF = 0.0;

constexpr PidfConfig kRotatePidfConfig(
    kRotatekP,
    kRotatekI,
    kRotatekD,
    kRotatekIZone,
    kRotatekFF);
constexpr PidfConfig kAnglePidfConfig(
    kAnglekP,
    kAnglekI,
    kAnglekD,
    kAnglekIZone,
    kAnglekFF);
constexpr PidfConfig kFlywheelPidfConfig(
    kFlywheelkP,
    kFlywheelkI,
    kFlywheelkD,
    kFlywheelkIZone,
    kFlywheelkFF);

namespace Encoder {
constexpr int COUNTS_PER_REV = 4096;
constexpr int HALF_COUNTS_PER_REV = COUNTS_PER_REV / 2;
constexpr double COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REV;
constexpr int MAX_ACCEPTED_DELTA_COUNTS = 180;
constexpr unsigned long SAMPLE_TIMEOUT_MS = 100;
}

namespace Elevation {
constexpr int SERVO_ATTACH_MIN_US = 1000;
constexpr int SERVO_ATTACH_MAX_US = 2000;
constexpr int FORWARD_US = 2000;
constexpr int STOP_US = 1500;
constexpr int REVERSE_US = 1000;
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = true;
constexpr ServoMotorConfig LEFT_MOTOR_CONFIG(
    LEFT_INVERTED,
    REVERSE_US,
    STOP_US,
    FORWARD_US);
constexpr ServoMotorConfig RIGHT_MOTOR_CONFIG(
    RIGHT_INVERTED,
    REVERSE_US,
    STOP_US,
    FORWARD_US);
constexpr int MAX_CLOSED_LOOP_OFFSET_US = 80;
constexpr long MIN_TARGET_COUNTS = 0;
constexpr long MAX_TARGET_COUNTS = 1024;
constexpr bool FORWARD_SOFT_LIMIT_ENABLED = true;
constexpr bool REVERSE_SOFT_LIMIT_ENABLED = true;
constexpr PositionLimitConfig SOFT_LIMIT_CONFIG(
    FORWARD_SOFT_LIMIT_ENABLED,
    REVERSE_SOFT_LIMIT_ENABLED,
    MAX_TARGET_COUNTS,
    MIN_TARGET_COUNTS);
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

namespace Turret {
constexpr int ATTACH_MIN_US = 1000;
constexpr int ATTACH_MAX_US = 2000;
constexpr int FORWARD_US = 2000;
constexpr int STOP_US = 1500;
constexpr int REVERSE_US = 1000;
constexpr bool INVERTED = false;
constexpr ServoMotorConfig MOTOR_CONFIG(
    INVERTED,
    REVERSE_US,
    STOP_US,
    FORWARD_US);
}

namespace Flywheel {
constexpr int ATTACH_MIN_US = 1000;
constexpr int ATTACH_MAX_US = 2000;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr bool INVERTED = false;
constexpr ServoMotorConfig MOTOR_CONFIG(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);
}

static_assert(Elevation::REVERSE_US < Elevation::STOP_US, "Invalid elevation reverse PWM");
static_assert(Elevation::STOP_US < Elevation::FORWARD_US, "Invalid elevation forward PWM");
static_assert(Turret::REVERSE_US < Turret::STOP_US, "Invalid turret reverse PWM");
static_assert(Turret::STOP_US < Turret::FORWARD_US, "Invalid turret forward PWM");
static_assert(Flywheel::MIN_PULSE_US < Flywheel::NEUTRAL_US, "Invalid flywheel minimum PWM");
static_assert(Flywheel::NEUTRAL_US < Flywheel::MAX_PULSE_US, "Invalid flywheel maximum PWM");
static_assert(
    Elevation::SOFT_LIMIT_CONFIG.reverseLimitCounts <
        Elevation::SOFT_LIMIT_CONFIG.forwardLimitCounts,
    "Elevation soft limits are reversed");

const int HALF_AUTO_VER[4][8] = {
  {1212,1200,1212,1212,1212,1212,1212},
  {1915,1497,1374,1625,1410,1212,1212},
  {1957,2230,2075,2684,2326,1608,2307},
  {2515,2635,2667,3492,2918,2380,2918}
};

}
