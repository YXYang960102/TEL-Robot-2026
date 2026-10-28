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
double Shooter::flywheelSpinCommand = 0.0;
double Shooter::flywheelOutputCommand = 0.0;
unsigned long Shooter::flywheelRampLastMs = 0;
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
    flywheelOutputCommand = 0.0;
    flywheelRampLastMs = millis();
    stopAll();
}

void Shooter::update() {
    if (!outputsEnabled) {
        stopAll();
        // Disabled: the turret and elevation stop at once, but the flywheel
        // spins down along the ramp (Jeremy, 2026-10-18: this Falcon is in
        // Brake mode, so an instant neutral stops it hard). It is still
        // driven until the ramp reaches 0 -- at most 1 / RAMP_DOWN s.
        rampFlywheel();
        writeFlywheelCommand(flywheelOutputCommand);
        return;
    }
    updateAngle();
    updateRotate();
    rampFlywheel();
    writeFlywheelCommand(flywheelOutputCommand);
    updateFlywheelReady();
}

void Shooter::rampFlywheel() {
    const unsigned long now = millis();
    unsigned long elapsed = now - flywheelRampLastMs;
    flywheelRampLastMs = now;
    if (elapsed > 100) elapsed = 100; // a stalled loop must not jump
    if (flywheelOpenLoopCommand > flywheelOutputCommand) {
        const double step = Flywheel::RAMP_UP_PER_SECOND * elapsed / 1000.0;
        flywheelOutputCommand = flywheelOpenLoopCommand - flywheelOutputCommand > step
            ? flywheelOutputCommand + step : flywheelOpenLoopCommand;
    } else {
        // Down too (2026-10-18), including the spin-down after a disable.
        const double step = Flywheel::RAMP_DOWN_PER_SECOND * elapsed / 1000.0;
        flywheelOutputCommand = flywheelOutputCommand - flywheelOpenLoopCommand > step
            ? flywheelOutputCommand - step : flywheelOpenLoopCommand;
    }
}

void Shooter::stopAll() {
    angleMode = AngleControlMode::DISABLED;
    rotateMode = RotateControlMode::DISABLED;
    angleManualCommand = 0.0;
    rotateManualCommand = 0.0;
    flywheelOpenLoopCommand = 0.0; // the output ramps down to it (rampFlywheel)
    flywheelSpinStartMs = 0;
    flywheelReady = false;
    rotateReady = false;
    rotateAimOffsetPx = 0.0;
    rotateVisionFollow.reset();
    releaseAngle();
    writeRotateCommand(0.0);
    writeFlywheelCommand(flywheelOutputCommand); // no jump: update() ramps it down
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
    return flywheelOutputCommand; // what goes out now (ramped), like the pulse
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
        // Protocol v3: a PREDICTED position (the selected hole out of view)
        // is followed too, so the turret turns toward it, but it is never
        // a lock -- only the hole really in view can make the turret ready.
        const bool seen = Vision::isValid();
        const double command = rotateVisionFollow.update(
            Vision::getTx() + rotateAimOffsetPx,
            seen || Vision::isPredicted(),
            Vision::getTargetId(),
            millis());
        rotateReady = seen && rotateVisionFollow.isReady();
        writeRotateCommand(command);
        return;
    }
    rotateReady = false;
    writeRotateCommand(0.0);
}

void Shooter::updateFlywheelReady() {
    // No RPM feedback: "ready" means the ramped output has been non-zero and
    // steady long enough that the wheel has probably reached it. A change
    // of more than FLYWHEEL_READY_BAND (a new distance / table value)
    // restarts the wait. Never ready while stopped. (2026-10-18: was
    // "at/above SHOOT_COMMAND", which the measured 42-46% table values
    // could never reach.)
    constexpr double FLYWHEEL_READY_BAND = 0.02;
    if (flywheelOutputCommand > 0.0) {
        if (flywheelSpinStartMs == 0 ||
            fabs(flywheelOutputCommand - flywheelSpinCommand) > FLYWHEEL_READY_BAND) {
            flywheelSpinStartMs = millis();
            if (flywheelSpinStartMs == 0) flywheelSpinStartMs = 1; // 0 means "not spinning"
            flywheelSpinCommand = flywheelOutputCommand;
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
    // Stopped = 1500 + STOP_PULSE_OFFSET_US, an absolute pulse that does not
    // follow INVERTED (this Falcon creeps at plain 1500; -10 -> 1490 us).
    if (command <= 0.0) {
        flywheelPulseUs = Flywheel::NEUTRAL_US + Flywheel::STOP_PULSE_OFFSET_US;
    } else {
        flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR);
    }
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}
