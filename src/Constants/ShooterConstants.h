#pragma once

#include <stdint.h>

#include "../Control/ActuatorConfig.h"
#include "../Control/PidfController.h"
#include "../Control/VisionFollowController.h"
#include "../Sensors/AnalogPositionSensor.h"
#include "../Sensors/AS5600Encoder.h"
#include "TuningConstants.h"

namespace ShooterConstants {

// Angle Constants
namespace Angle {

using namespace TuningConstants::Angle;

enum class Action : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

// Single elevation MG996R. Set false only for a continuous-rotation unit.
constexpr bool POSITION_SERVO = true;
constexpr bool EXTERNAL_ENCODER_ENABLED = false;
// Provisional command travel, not calibrated mechanical limits.
constexpr double POSITION_INITIAL_US = 1500.0;
constexpr double POSITION_RATE_US_PER_SECOND = 1000.0;
constexpr uint8_t ANGLE_PIN = 24; // elevation servo signal

constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

// No elevation limit switches are installed (Jeremy, 2026-10-14). The
// legacy all_robot_017 up/down inputs on D52/D53 were removed: unwired and
// without a pull-up they floated and could randomly block one direction.

constexpr int ENCODER_COUNTS_PER_REV = 4096;
constexpr int ENCODER_HALF_COUNTS_PER_REV = ENCODER_COUNTS_PER_REV / 2;
constexpr double COUNTS_TO_DEGREES = 360.0 / ENCODER_COUNTS_PER_REV;
constexpr int MAX_ACCEPTED_DELTA_COUNTS = 180;
constexpr unsigned long SENSOR_TIMEOUT_MS = 100;
constexpr AS5600EncoderConfig ENCODER(
    ENCODER_COUNTS_PER_REV,
    MAX_ACCEPTED_DELTA_COUNTS,
    SENSOR_TIMEOUT_MS);

constexpr PositionLimitConfig SOFT_LIMIT(
    FORWARD_SOFT_LIMIT_ENABLED,
    REVERSE_SOFT_LIMIT_ENABLED,
    MAX_POSITION_COUNTS,
    MIN_POSITION_COUNTS);

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

using namespace TuningConstants::Rotate;

enum class Action : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

// pin pwm invert
constexpr uint8_t SIGNAL_PIN = 23;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
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

constexpr PidfConfig PIDF(kP, kI, kD, kIZone, kFF);

// No limit switches will be installed on the real turret mechanism (Jeremy,
// 2026-10-06) — LEFT/RIGHT_LIMIT_PIN stay defined for bench compatibility
// but are not wired on the competition robot. LIMIT_USE_INTERNAL_PULLUP is
// already false for this axis, so without this flag the pins would float
// (unpredictable, not fail-safe) instead of reading a real switch.
// isRotateLeftLimitTriggered()/isRotateRightLimitTriggered() return false
// unconditionally while this is false.
constexpr bool LIMIT_SWITCHES_INSTALLED = false;

// Vision-follow (RotateControlMode::VISION_FOLLOW): PIDF on image-space tx,
// gains in TuningConstants::Rotate (VISION_*), converted from the bench
// values Jeremy accepted on 2026-10-13. Sign: VisionFollowController's
// output is PIDF(setpoint 0, measurement tx), so tx > 0 gives a negative
// command, which with INVERTED = false is a pulse below 1500 us -- the
// direction validated on the bench with this same servo and mounting
// (Jeremy confirmed it is the D23 turret servo). If it is ever backwards,
// flip Rotate::INVERTED above; do not add a second inversion in the
// vision-follow path.
constexpr int READY_TOLERANCE_RAW = 2;
constexpr unsigned long READY_SETTLE_TIME_MS = 100;
constexpr unsigned long CONTROL_PERIOD_MS = 20;
constexpr unsigned long MAX_CONTROL_GAP_MS = 100;
constexpr int SETPOINT_RESET_THRESHOLD_RAW = 10;
constexpr double MIN_NORMALIZED_OUTPUT = -1.0;
constexpr double MAX_NORMALIZED_OUTPUT = 1.0;
constexpr double MIN_INTEGRAL_OUTPUT = -0.25;
constexpr double MAX_INTEGRAL_OUTPUT = 0.25;

constexpr VisionFollowConfig VISION_FOLLOW(
    PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
    VISION_MAX_COMMAND,
    VISION_MAX_INTEGRAL_COMMAND,
    VISION_READY_TOLERANCE_PX,
    READY_SETTLE_TIME_MS,
    MAX_CONTROL_GAP_MS);

}

// Flywheel Constants
namespace Flywheel {

using namespace TuningConstants::Flywheel;

enum class Action : uint8_t {
    STOP = 0,
    FORWARD = 1
};

constexpr uint8_t SIGNAL_PIN = 25;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

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
// Rotate calibration ordering, profile-shape, and vision-follow tuning
// static_asserts now live in TuningConstants.h next to the values they
// check (those values moved there; see Rotate::'s `using namespace` above).
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
