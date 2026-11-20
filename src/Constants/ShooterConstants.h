#pragma once

#include <stdint.h>

#include "../Control/ActuatorConfig.h"
#include "../Control/PidfController.h"
#include "../Control/ServoAngleMap.h"
#include "../Control/VisionFollowController.h"
#include "TuningConstants.h"

namespace ShooterConstants {

namespace Angle {

using namespace TuningConstants::Angle;

constexpr uint8_t ANGLE_PIN = 24; // elevation servo signal

constexpr bool POSITION_SERVO = true; // positional servo (false = continuous)
constexpr double POSITION_INITIAL_US = 2461.0; // first pulse after boot: top of travel
constexpr double POSITION_RATE_US_PER_SECOND = 1000.0;

constexpr int MIN_PULSE_US = 500;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2500; // was 2461 (2026-11-21)

constexpr double CAL_A_PULSE_US = 500.0; // pulse <-> deg, Dashboard readout only
constexpr double CAL_A_DEGREES = 0.0;
constexpr double CAL_B_PULSE_US = 2500.0;
constexpr double CAL_B_DEGREES = 270.0;
constexpr ServoAngleMap ANGLE_MAP{
    CAL_A_PULSE_US, CAL_A_DEGREES, CAL_B_PULSE_US, CAL_B_DEGREES,
    static_cast<double>(MIN_PULSE_US), static_cast<double>(MAX_PULSE_US)};

constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

}

namespace Rotate {

using namespace TuningConstants::Rotate;

constexpr uint8_t SIGNAL_PIN = 23; // turret
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

constexpr unsigned long READY_SETTLE_TIME_MS = 250; // |tx| in tolerance this long = locked
constexpr unsigned long MAX_CONTROL_GAP_MS = 100;

constexpr VisionFollowConfig VISION_FOLLOW(
    PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
    VISION_MAX_COMMAND,
    VISION_MAX_INTEGRAL_COMMAND,
    VISION_READY_TOLERANCE_PX,
    READY_SETTLE_TIME_MS,
    MAX_CONTROL_GAP_MS,
    VISION_FRAME_kD,
    VISION_STALL_COMMAND,
    VISION_STALL_RATE_PX_PER_S,
    VISION_STALL_DELAY_MS,
    VISION_STALL_PULSE_MS,
    VISION_HOLD_I_IN_TOLERANCE);

}

namespace Flywheel {

using namespace TuningConstants::Flywheel;

constexpr uint8_t SIGNAL_PIN = 25; // Falcon
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

}

static_assert(Angle::MIN_PULSE_US < Angle::NEUTRAL_US, "Invalid angle minimum PWM");
static_assert(Angle::CAL_A_PULSE_US != Angle::CAL_B_PULSE_US &&
                  Angle::CAL_A_DEGREES != Angle::CAL_B_DEGREES,
              "Elevation calibration needs two distinct points");
static_assert(Angle::NEUTRAL_US < Angle::MAX_PULSE_US, "Invalid angle maximum PWM");
static_assert(Angle::POSITION_INITIAL_US >= Angle::MIN_PULSE_US &&
                  Angle::POSITION_INITIAL_US <= Angle::MAX_PULSE_US,
              "Elevation boot pulse must be inside the pulse travel");
static_assert(Rotate::MIN_PULSE_US < Rotate::NEUTRAL_US, "Invalid rotate minimum PWM");
static_assert(Rotate::NEUTRAL_US < Rotate::MAX_PULSE_US, "Invalid rotate maximum PWM");
static_assert(Flywheel::MIN_PULSE_US < Flywheel::NEUTRAL_US, "Invalid flywheel minimum PWM");
static_assert(Flywheel::NEUTRAL_US < Flywheel::MAX_PULSE_US, "Invalid flywheel maximum PWM");
static_assert(Flywheel::STOP_PULSE_OFFSET_US >= -30 && Flywheel::STOP_PULSE_OFFSET_US <= 30,
              "Flywheel stop offset must stay near the Falcon's neutral deadband (~20 us)");

}
