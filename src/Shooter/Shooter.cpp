#include "Shooter.h"

#include "../Constants/BuildProfile.h"
#include "../Control/ServoPositionJog.h"
#include "../Control/VisionFollowController.h"
#include "../Vision/Vision.h"

using namespace ShooterConstants;

namespace {

// Which shooter outputs this build drives (production: all three).
constexpr bool DRIVES_AXES = BuildProfile::DRIVES_SHOOTER_AXES;
constexpr bool DRIVES_FLYWHEEL = BuildProfile::DRIVES_FLYWHEEL;

ServoPositionJog anglePosition(Angle::POSITION_INITIAL_US);
VisionFollowController rotateVisionFollow(Rotate::VISION_FOLLOW);
uint32_t angleJogLastMs = 0;

int commandToPulseUs(double command, const ServoMotorConfig& config) {
    command = constrain(command, -1.0, 1.0);
    if (config.inverted) {
        command = -command;
    }
    const double pulseUs = command >= 0.0
        ? config.neutralUs + command * (config.maximumUs - config.neutralUs)
        : config.neutralUs + (-command) * (config.minimumUs - config.neutralUs);
    return static_cast<int>(pulseUs + 0.5);
}

}

Servo Shooter::angleServo;
Servo Shooter::rotateServo;
Servo Shooter::flywheelOutput;
AngleControlMode Shooter::angleMode = AngleControlMode::DISABLED;
RotateControlMode Shooter::rotateMode = RotateControlMode::DISABLED;
bool Shooter::outputsEnabled = false;
bool Shooter::rotateReady = false;
double Shooter::angleManualCommand = 0.0;
double Shooter::rotateManualCommand = 0.0;
double Shooter::flywheelOpenLoopCommand = 0.0;
int Shooter::anglePulseUs = Angle::MOTOR.neutralUs;
int Shooter::rotatePulseUs = Rotate::MOTOR.neutralUs;
int Shooter::flywheelPulseUs = Flywheel::MOTOR.neutralUs;
unsigned long Shooter::flywheelSpinStartMs = 0;
bool Shooter::flywheelReady = false;

void Shooter::init() {
    // Preload neutral before attaching so the first pulse is never a stray
    // value. The positional elevation servo is attached lazily on its first
    // jog (see writeAngleCommand), so it gets no pulse until then.
    if (DRIVES_FLYWHEEL) {
        flywheelOutput.writeMicroseconds(Flywheel::NEUTRAL_US);
        flywheelOutput.attach(Flywheel::SIGNAL_PIN, Flywheel::MIN_PULSE_US, Flywheel::MAX_PULSE_US);
    }
    if (DRIVES_AXES) {
        rotateServo.writeMicroseconds(Rotate::NEUTRAL_US);
        rotateServo.attach(Rotate::SIGNAL_PIN, Rotate::MIN_PULSE_US, Rotate::MAX_PULSE_US);
        if (!Angle::POSITION_SERVO) {
            angleServo.writeMicroseconds(Angle::NEUTRAL_US);
            angleServo.attach(Angle::ANGLE_PIN, Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
        }
    }
    angleJogLastMs = millis();
    stopAll();
}

void Shooter::update() {
    if (!outputsEnabled) {
        stopAll();
        return;
    }
    updateAngle();
    updateRotate();
    writeFlywheelCommand(flywheelOpenLoopCommand);
    updateFlywheelReady();
}

void Shooter::stopAll() {
    angleMode = AngleControlMode::DISABLED;
    rotateMode = RotateControlMode::DISABLED;
    angleManualCommand = 0.0;
    rotateManualCommand = 0.0;
    flywheelOpenLoopCommand = 0.0;
    flywheelSpinStartMs = 0;
    flywheelReady = false;
    rotateReady = false;
    rotateVisionFollow.reset();
    releaseAngle();
    writeRotateCommand(0.0);
    writeFlywheelCommand(0.0);
}

void Shooter::setOutputsEnabled(bool enabled) {
    if (!DRIVES_AXES) {
        enabled = false; // drive+conveyor bench: the shooter never enables
    }
    outputsEnabled = enabled;
    if (!outputsEnabled) {
        stopAll();
    }
}

bool Shooter::areOutputsEnabled() {
    return outputsEnabled;
}

void Shooter::setAngleOpenLoop(double command) {
    angleManualCommand = constrain(command, -1.0, 1.0);
    angleMode = AngleControlMode::MANUAL_OPEN_LOOP;
}

bool Shooter::setAngleTargetDegrees(double degrees) {
    if (!Angle::POSITION_SERVO) return false;
    anglePosition.set(Angle::ANGLE_MAP.toPulse(degrees), Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
    anglePulseUs = static_cast<int>(anglePosition.value() + 0.5);
    angleMode = AngleControlMode::MANUAL_OPEN_LOOP; // updateAngle() writes it
    return true;
}

bool Shooter::stepAngleTargetDegrees(double deltaDegrees) {
    return setAngleTargetDegrees(getAngleTargetDegrees() + deltaDegrees);
}

double Shooter::getAngleTargetDegrees() {
    return Angle::ANGLE_MAP.toDegrees(anglePosition.value());
}

double Shooter::getAngleMinDegrees() {
    return Angle::ANGLE_MAP.minDegrees();
}

double Shooter::getAngleMaxDegrees() {
    return Angle::ANGLE_MAP.maxDegrees();
}

void Shooter::setRotateOpenLoop(double command) {
    rotateManualCommand = constrain(command, -1.0, 1.0);
    rotateMode = RotateControlMode::MANUAL_OPEN_LOOP;
    rotateReady = false;
    rotateVisionFollow.reset();
}

void Shooter::disableRotate() {
    rotateMode = RotateControlMode::DISABLED;
    rotateManualCommand = 0.0;
    rotateReady = false;
    rotateVisionFollow.reset();
    writeRotateCommand(0.0);
}

void Shooter::setRotateVisionFollow() {
    if (rotateMode != RotateControlMode::VISION_FOLLOW) {
        // Entering vision-follow never carries a lock or integral over.
        rotateReady = false;
        rotateVisionFollow.reset();
    }
    rotateMode = RotateControlMode::VISION_FOLLOW;
}

void Shooter::setFlywheelOpenLoop(double command) {
    flywheelOpenLoopCommand = constrain(command, 0.0, 1.0);
}

bool Shooter::isRotateReady() {
    return rotateReady;
}

bool Shooter::isFlywheelReady() {
    return flywheelReady;
}

bool Shooter::isReady() {
    return rotateReady && flywheelReady;
}

AngleControlMode Shooter::getAngleControlMode() {
    return angleMode;
}

RotateControlMode Shooter::getRotateControlMode() {
    return rotateMode;
}

int Shooter::getAnglePulseUs() {
    return anglePulseUs;
}

int Shooter::getRotatePulseUs() {
    return rotatePulseUs;
}

int Shooter::getFlywheelPulseUs() {
    return flywheelPulseUs;
}

double Shooter::getFlywheelCommand() {
    return flywheelOpenLoopCommand;
}

void Shooter::updateAngle() {
    if (angleMode == AngleControlMode::MANUAL_OPEN_LOOP) {
        writeAngleCommand(angleManualCommand);
    } else {
        releaseAngle();
    }
}

void Shooter::updateRotate() {
    if (rotateMode == RotateControlMode::MANUAL_OPEN_LOOP) {
        rotateReady = false;
        writeRotateCommand(rotateManualCommand);
        return;
    }
    if (rotateMode == RotateControlMode::VISION_FOLLOW) {
        // PIDF on image-space tx, same control law and gains as the A5
        // bench. An invalid/stale target commands 0 and clears the lock.
        const double command = rotateVisionFollow.update(
            Vision::getTx(),
            Vision::isValid(),
            Vision::getTargetId(),
            millis());
        rotateReady = rotateVisionFollow.isReady();
        writeRotateCommand(command);
        return;
    }
    rotateReady = false;
    writeRotateCommand(0.0);
}

void Shooter::updateFlywheelReady() {
    // No RPM feedback: "ready" means commanded at/above shoot speed for
    // long enough that it has probably spun up.
    if (flywheelOpenLoopCommand >= Flywheel::SHOOT_COMMAND) {
        if (flywheelSpinStartMs == 0) {
            flywheelSpinStartMs = millis();
        }
        flywheelReady = millis() - flywheelSpinStartMs >= Flywheel::SPIN_UP_MS;
    } else {
        flywheelSpinStartMs = 0;
        flywheelReady = false;
    }
}

void Shooter::writeAngleCommand(double command) {
    if (!DRIVES_AXES) return;
    command = constrain(command, -1.0, 1.0);
    if (Angle::POSITION_SERVO) {
        // Positional servo: the command moves a target pulse; releasing the
        // key (command 0) holds it.
        const uint32_t now = millis();
        const uint32_t elapsed = now - angleJogLastMs;
        angleJogLastMs = now;
        if (Angle::INVERTED) command = -command;
        const double target = anglePosition.step(command, elapsed,
            Angle::POSITION_RATE_US_PER_SECOND, Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
        anglePulseUs = static_cast<int>(target + 0.5);
        // Preload before attaching, so the first pulse uses the retained target.
        angleServo.writeMicroseconds(anglePulseUs);
        if (!angleServo.attached()) {
            angleServo.attach(Angle::ANGLE_PIN, Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
        }
    } else {
        anglePulseUs = commandToPulseUs(command, Angle::MOTOR);
        angleServo.writeMicroseconds(anglePulseUs);
    }
}

void Shooter::releaseAngle() {
    if (!DRIVES_AXES) return;
    if (Angle::POSITION_SERVO) {
        // Disable removes pulses; never command centre as a "stop" (that
        // would swing the elevation). The software target is kept.
        angleServo.detach();
        angleJogLastMs = millis();
    } else {
        anglePulseUs = Angle::MOTOR.neutralUs;
        angleServo.writeMicroseconds(anglePulseUs);
    }
}

void Shooter::writeRotateCommand(double command) {
    if (!DRIVES_AXES) return;
    rotatePulseUs = commandToPulseUs(command, Rotate::MOTOR);
    rotateServo.writeMicroseconds(rotatePulseUs);
}

void Shooter::writeFlywheelCommand(double command) {
    if (!DRIVES_FLYWHEEL) return;
    flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR);
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}
