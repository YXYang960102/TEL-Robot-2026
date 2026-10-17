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
double Shooter::rotateAimOffsetPx = 0.0;
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
        // Neutral for the very first pulse; stopAll() below moves it to the
        // stop-trimmed value right away.
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
    rotateAimOffsetPx = 0.0;
    rotateVisionFollow.reset();
    releaseAngle();
    writeRotateCommand(0.0);
    writeFlywheelCommand(0.0);
}

void Shooter::setOutputsEnabled(bool enabled) {
    if (!DRIVES_AXES) {
        enabled = false; // drive-only bench: the shooter never enables
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
    return setAnglePulseUs(Angle::ANGLE_MAP.toPulse(degrees));
}

bool Shooter::setAnglePulseUs(double pulseUs) {
    if (!Angle::POSITION_SERVO) return false;
    anglePosition.set(pulseUs, Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
    anglePulseUs = static_cast<int>(anglePosition.value() + 0.5);
    angleMode = AngleControlMode::MANUAL_OPEN_LOOP; // updateAngle() writes it
    return true;
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

void Shooter::setRotateAimOffsetPx(double offsetPx) {
    rotateAimOffsetPx = offsetPx;
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
        // Aim offset R: the loop drives tx + R to 0, i.e. holds the target
        // R px left of center, so the turret points R px right of it.
        // Example: R = +10, target at tx = -10 -> input 0 -> locked.
        const double command = rotateVisionFollow.update(
            Vision::getTx() + rotateAimOffsetPx,
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
    // Never ready while stopped, even with SHOOT_COMMAND at 0 (2026-10-17:
    // the autos run with the flywheel off for turret testing).
    if (flywheelOpenLoopCommand > 0.0 && flywheelOpenLoopCommand >= Flywheel::SHOOT_COMMAND) {
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
    // Stopped = STOP_TRIM_US toward the spin direction, not exactly 1500
    // (this Falcon creeps backwards at 1500). As a command it goes through
    // the same INVERTED handling: 10 us -> 0.02 -> 1490 us when inverted.
    const double stopCommand = static_cast<double>(Flywheel::STOP_TRIM_US) /
                               (Flywheel::MAX_PULSE_US - Flywheel::NEUTRAL_US);
    if (command <= 0.0) command = stopCommand;
    flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR);
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}
