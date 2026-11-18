#include "Shooter.h"

#include "../Constants/BuildProfile.h"
#include "../Control/ServoPositionJog.h"
#include "../Control/VisionFollowController.h"
#include "../Sensors/TurretAngle.h"
#include "../Vision/OperatorTarget.h"
#include "../Vision/Vision.h"
#include "TurretGuard.h"

using namespace ShooterConstants;

namespace {

constexpr bool DRIVES_AXES = BuildProfile::DRIVES_SHOOTER_AXES;
constexpr bool DRIVES_FLYWHEEL = BuildProfile::DRIVES_FLYWHEEL;

ServoPositionJog anglePosition(Angle::POSITION_INITIAL_US);
VisionFollowController rotateVisionFollow(Rotate::VISION_FOLLOW);
uint32_t angleJogLastMs = 0;
TurretGuard turretGuard;
bool turretReturnDone = false;
uint8_t searchSelection = 0;
unsigned long searchStartMs = 0;
bool searchFound = false;
int8_t lastSeenColumn = -1;
int8_t searchDir = 0;
bool haveLastSeen = false;
double lastSeenTx = 0.0;
unsigned long lastSeenMs = 0;
bool sweepAllowed = false;
int8_t sweepDir = -1;
unsigned long blindSinceMs = 0;
unsigned long confirmSinceMs = 0;
int confirmTargetId = 0;

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
int Shooter::anglePulseUs = Angle::POSITION_SERVO ? static_cast<int>(Angle::POSITION_INITIAL_US + 0.5)
                                                  : Angle::MOTOR.neutralUs;
int Shooter::rotatePulseUs = Rotate::MOTOR.neutralUs;
int Shooter::flywheelPulseUs = Flywheel::MOTOR.neutralUs;
unsigned long Shooter::flywheelSpinStartMs = 0;
double Shooter::flywheelSpinCommand = 0.0;
double Shooter::flywheelOutputCommand = 0.0;
unsigned long Shooter::flywheelRampLastMs = 0;
bool Shooter::flywheelReady = false;

void Shooter::init() {
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
    flywheelOutputCommand = 0.0;
    flywheelRampLastMs = millis();
    stopAll();
}

void Shooter::update() {
    if (!outputsEnabled) {
        stopAll();
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
    if (elapsed > 100) elapsed = 100;
    if (flywheelOpenLoopCommand > flywheelOutputCommand) {
        const double step = Flywheel::RAMP_UP_PER_SECOND * elapsed / 1000.0;
        flywheelOutputCommand = flywheelOpenLoopCommand - flywheelOutputCommand > step
            ? flywheelOutputCommand + step : flywheelOpenLoopCommand;
    } else {
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
    flywheelOpenLoopCommand = 0.0;
    flywheelSpinStartMs = 0;
    flywheelReady = false;
    rotateReady = false;
    rotateAimOffsetPx = 0.0;
    rotateVisionFollow.reset();
    releaseAngle();
    writeRotateCommand(0.0);
    writeFlywheelCommand(flywheelOutputCommand);
}

void Shooter::setOutputsEnabled(bool enabled) {
    if (!DRIVES_AXES) {
        enabled = false;
    }
    if (TurretAngleConstants::GUARD_ENABLED && enabled && !outputsEnabled && !turretReturnDone) {
        turretGuard.beginReturn(); // first Enable after power-up: back to 0 deg
    }
    outputsEnabled = enabled;
    if (!outputsEnabled) {
        turretGuard.onDisabled();
        stopAll();
    }
}

void Shooter::requestTurretReturn() {
    if (TurretAngleConstants::GUARD_ENABLED) turretGuard.beginReturn();
}

int8_t Shooter::getSearchDirection() {
    return searchDir;
}

uint8_t Shooter::getTurretState() {
    return TurretAngleConstants::GUARD_ENABLED ? turretGuard.state() : TurretGuard::OFF;
}

void Shooter::driveRotate(double command) {
    if (!TurretAngleConstants::GUARD_ENABLED) {
        writeRotateCommand(command);
        return;
    }
    TurretGuard::Inputs in;
    in.absolute = TurretAngle::isAbsolute();
    in.healthy = TurretAngle::isHealthy();
    in.angleDeg = TurretAngle::angleDegrees();
    in.nowMs = millis();
    const bool wasReturning = turretGuard.isReturning();
    const double out = turretGuard.filter(command, in);
    if (wasReturning && !turretGuard.isReturning() && turretGuard.allowsLock()) {
        turretReturnDone = true;
    }
    if (!turretGuard.allowsLock()) {
        rotateReady = false;
    }
    writeRotateCommand(out);
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
    angleMode = AngleControlMode::MANUAL_OPEN_LOOP;
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
        rotateReady = false;
        rotateVisionFollow.reset();
    }
    rotateMode = RotateControlMode::VISION_FOLLOW;
}

void Shooter::setRotateAimOffsetPx(double offsetPx) {
    rotateAimOffsetPx = offsetPx;
}

void Shooter::setRotateSweepAllowed(bool allowed) {
    sweepAllowed = allowed;
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
    return flywheelOutputCommand;
}

void Shooter::updateAngle() {
    if (angleMode == AngleControlMode::MANUAL_OPEN_LOOP) {
        writeAngleCommand(angleManualCommand);
    } else {
        releaseAngle();
    }
}

void Shooter::updateRotate() {
    if (rotateMode != RotateControlMode::VISION_FOLLOW || turretGuard.isReturning() || !sweepAllowed) {
        blindSinceMs = millis();
    }
    if (turretGuard.isReturning()) {
        rotateReady = false;
        rotateVisionFollow.reset();
        driveRotate(0.0);
        return;
    }
    if (rotateMode == RotateControlMode::MANUAL_OPEN_LOOP) {
        rotateReady = false;
        driveRotate(rotateManualCommand);
        return;
    }
    if (rotateMode == RotateControlMode::VISION_FOLLOW) {
        const unsigned long now = millis();
        const bool rawSeen = Vision::isValid();
        const bool rawPredicted = Vision::isPredicted();
        if (!(rawSeen || rawPredicted) || Vision::getTargetId() != confirmTargetId) {
            confirmSinceMs = rawSeen || rawPredicted ? (now == 0 ? 1 : now) : 0;
            confirmTargetId = rawSeen || rawPredicted ? Vision::getTargetId() : 0;
        }
        const bool confirmed = confirmSinceMs != 0 && now - confirmSinceMs >= Rotate::SEEN_CONFIRM_MS;
        const bool seen = rawSeen && confirmed;
        const bool predicted = rawPredicted && confirmed;
        const uint8_t selection = Vision::getSelectedTarget();
        if (selection != searchSelection) {
            searchSelection = selection;
            searchStartMs = now;
            searchFound = false;
            haveLastSeen = false;
        }
        if (seen) {
            lastSeenColumn = static_cast<int8_t>(
                OperatorTarget::columnOf(OperatorTarget::holeForVisionId(Vision::getTargetId())));
            haveLastSeen = true;
            lastSeenTx = Vision::getTx();
            lastSeenMs = now;
        }
        if (seen || predicted) searchFound = true;
        searchDir = 0;
        if (selection != OperatorTarget::NONE && !searchFound &&
            now - searchStartMs < Rotate::SEARCH_MAX_MS) {
            searchDir = OperatorTarget::searchDirection(
                OperatorTarget::holeForVisionId(selection),
                OperatorTarget::holesFromVisionMask(Vision::getSeenMask()), lastSeenColumn);
        }
        if (searchDir == 0 && !seen && !predicted && haveLastSeen &&
            now - lastSeenMs <= Rotate::LOST_TURN_MS && fabs(lastSeenTx) >= Rotate::LOST_EDGE_PX) {
            searchDir = lastSeenTx > 0.0 ? 1 : -1;
        }
        if (seen || predicted || searchDir != 0) {
            blindSinceMs = now;
            if (seen) sweepDir = -1;
        } else if (Rotate::SWEEP_ENABLED && sweepAllowed && TurretAngle::isHealthy() &&
                   TurretAngle::isAbsolute() && now - blindSinceMs >= Rotate::SWEEP_START_MS) {
            const double deg = TurretAngle::angleDegrees();
            if (deg <= Rotate::SWEEP_LEFT_DEG) sweepDir = 1;
            else if (deg >= Rotate::SWEEP_RIGHT_DEG) sweepDir = -1;
            searchDir = sweepDir;
        }
        if (searchDir != 0) {
            rotateReady = false;
            rotateVisionFollow.reset();
            driveRotate(searchDir * TurretAngleConstants::RIGHT_COMMAND_SIGN * Rotate::SEARCH_COMMAND);
            return;
        }
        const double command = rotateVisionFollow.update(
            Vision::getTx() + rotateAimOffsetPx,
            seen || predicted,
            Vision::getTargetId(),
            millis(),
            Vision::getFrameCount());
        rotateReady = seen && rotateVisionFollow.isReady();
        driveRotate(command);
        return;
    }
    rotateReady = false;
    driveRotate(0.0);
}

void Shooter::updateFlywheelReady() {
    constexpr double FLYWHEEL_READY_BAND = 0.02;
    if (flywheelOutputCommand > 0.0) {
        if (flywheelSpinStartMs == 0 ||
            fabs(flywheelOutputCommand - flywheelSpinCommand) > FLYWHEEL_READY_BAND) {
            flywheelSpinStartMs = millis();
            if (flywheelSpinStartMs == 0) flywheelSpinStartMs = 1;
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
        const uint32_t now = millis();
        const uint32_t elapsed = now - angleJogLastMs;
        angleJogLastMs = now;
        if (Angle::INVERTED) command = -command;
        const double target = anglePosition.step(command, elapsed,
            Angle::POSITION_RATE_US_PER_SECOND, Angle::MIN_PULSE_US, Angle::MAX_PULSE_US);
        anglePulseUs = static_cast<int>(target + 0.5);
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
    if (command <= 0.0) {
        flywheelPulseUs = Flywheel::NEUTRAL_US + Flywheel::STOP_PULSE_OFFSET_US;
    } else {
        flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR);
    }
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}
