#pragma once

#include <stdint.h>

#include "../Control/ActuatorConfig.h"
#include "../Control/PidfController.h"
#include "../Control/VisionFollowController.h"
#include "TuningConstants.h"

// The shooter head is three motors and no sensors (Jeremy, 2026-10-14):
// turret (rotate), elevation (angle) and flywheel. There is no encoder,
// potentiometer or limit switch anywhere on it; the turret is aimed by the
// camera (vision-follow PIDF on tx), the elevation is a manually jogged
// position servo, and the flywheel is an open-loop PWM command.
namespace ShooterConstants {

// Elevation
namespace Angle {

using namespace TuningConstants::Angle;

constexpr uint8_t ANGLE_PIN = 24; // elevation servo signal

// Single elevation MG996R. true = positional (180 deg) servo: jogging moves
// a target pulse, releasing the key holds it, disable removes the pulse.
// Set false only for a continuous-rotation unit: the command is then a
// speed and disable sends neutral.
constexpr bool POSITION_SERVO = true;
constexpr double POSITION_INITIAL_US = 1500.0;
constexpr double POSITION_RATE_US_PER_SECOND = 1000.0;

// Pulse travel. Widened 1000..2000 -> 500..2500 us on 2026-10-15 (Jeremy):
// at 1000..2000 this servo only turned ~90 deg, while a servo tester
// (500..2500 us) turns it ~270 deg. PROVISIONAL: these are the servo's
// limits, not the elevation mechanism's. There is no limit switch, so
// jog slowly the first time, note the pulse (Dashboard "elevation pulse")
// at each real mechanical end, and narrow MIN/MAX to those values.
// Both the jog target and Servo::attach() clamp to this range.
constexpr int MIN_PULSE_US = 500;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2500;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

}

// Turret
namespace Rotate {

using namespace TuningConstants::Rotate;

constexpr uint8_t SIGNAL_PIN = 23;
constexpr int MIN_PULSE_US = 1000;
constexpr int NEUTRAL_US = 1500;
constexpr int MAX_PULSE_US = 2000;
constexpr ServoMotorConfig MOTOR(
    INVERTED,
    MIN_PULSE_US,
    NEUTRAL_US,
    MAX_PULSE_US);

// Vision-follow (RotateControlMode::VISION_FOLLOW): PIDF on image-space tx,
// gains in TuningConstants::Rotate (VISION_*), converted from the bench
// values Jeremy accepted on 2026-10-13. Sign: VisionFollowController's
// output is PIDF(setpoint 0, measurement tx), so tx > 0 gives a negative
// command, which with INVERTED = false is a pulse below 1500 us -- the
// direction validated on the bench with this same servo and mounting
// (Jeremy confirmed it is the D23 turret servo). If it is ever backwards,
// flip Rotate::INVERTED; do not add a second inversion in the
// vision-follow path.
constexpr unsigned long READY_SETTLE_TIME_MS = 100;
constexpr unsigned long MAX_CONTROL_GAP_MS = 100;

constexpr VisionFollowConfig VISION_FOLLOW(
    PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
    VISION_MAX_COMMAND,
    VISION_MAX_INTEGRAL_COMMAND,
    VISION_READY_TOLERANCE_PX,
    READY_SETTLE_TIME_MS,
    MAX_CONTROL_GAP_MS);

}

// Flywheel (Falcon 500 over PWM, open loop)
namespace Flywheel {

using namespace TuningConstants::Flywheel;

constexpr uint8_t SIGNAL_PIN = 25;
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
static_assert(Angle::NEUTRAL_US < Angle::MAX_PULSE_US, "Invalid angle maximum PWM");
static_assert(Rotate::MIN_PULSE_US < Rotate::NEUTRAL_US, "Invalid rotate minimum PWM");
static_assert(Rotate::NEUTRAL_US < Rotate::MAX_PULSE_US, "Invalid rotate maximum PWM");
static_assert(Flywheel::MIN_PULSE_US < Flywheel::NEUTRAL_US, "Invalid flywheel minimum PWM");
static_assert(Flywheel::NEUTRAL_US < Flywheel::MAX_PULSE_US, "Invalid flywheel maximum PWM");

}
