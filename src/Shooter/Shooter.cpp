#include "Shooter.h"

using namespace ShooterConstants;

namespace {

double clampCommand(double command) {
    if (command < -1.0) {
        return -1.0;
    }
    if (command > 1.0) {
        return 1.0;
    }
    return command;
}

int commandToPulseUs(double command, const ServoMotorConfig& config) {
    command = clampCommand(command);
    if (config.inverted) {
        command = -command;
    }

    const double pulseUs = command >= 0.0
        ? config.neutralUs + command * (config.maximumUs - config.neutralUs)
        : config.neutralUs + (-command) * (config.minimumUs - config.neutralUs);
    return static_cast<int>(pulseUs + (pulseUs >= 0.0 ? 0.5 : -0.5));
}

int offsetToPulseUs(int offsetUs, const ServoMotorConfig& config) {
    const int appliedOffset = config.inverted ? -offsetUs : offsetUs;
    return constrain(
        config.neutralUs + appliedOffset,
        config.minimumUs,
        config.maximumUs);
}

}

Servo Shooter::angleLeftServo;
Servo Shooter::angleRightServo;
Servo Shooter::rotateServo;
Servo Shooter::flywheelOutput;
AS5600Encoder Shooter::angleEncoder(Angle::ENCODER);
AngleControlMode Shooter::angleMode = AngleControlMode::DISABLED;
bool Shooter::outputsEnabled = false;
bool Shooter::angleHomed = false;
bool Shooter::angleReady = false;
double Shooter::angleManualCommand = 0.0;
double Shooter::rotateManualCommand = 0.0;
double Shooter::flywheelOpenLoopCommand = 0.0;
double Shooter::angleSetpoint = 0.0;
PidfController Shooter::angleController(Angle::PIDF);
double Shooter::angleControllerOutput = 0.0;
unsigned long Shooter::lastAngleControlMs = 0;
unsigned long Shooter::angleReadySinceMs = 0;
int Shooter::leftAnglePulseUs = Angle::LEFT_MOTOR.neutralUs;
int Shooter::rightAnglePulseUs = Angle::RIGHT_MOTOR.neutralUs;
int Shooter::rotatePulseUs = Rotate::MOTOR.neutralUs;
int Shooter::flywheelPulseUs = Flywheel::MOTOR.neutralUs;

void Shooter::init() {
    angleLeftServo.attach(
        Angle::LEFT_SIGNAL_PIN,
        Angle::MIN_PULSE_US,
        Angle::MAX_PULSE_US);
    angleRightServo.attach(
        Angle::RIGHT_SIGNAL_PIN,
        Angle::MIN_PULSE_US,
        Angle::MAX_PULSE_US);
    rotateServo.attach(
        Rotate::SIGNAL_PIN,
        Rotate::MIN_PULSE_US,
        Rotate::MAX_PULSE_US);
    flywheelOutput.attach(
        Flywheel::SIGNAL_PIN,
        Flywheel::MIN_PULSE_US,
        Flywheel::MAX_PULSE_US);

    angleEncoder.begin();
    angleController.setOutputLimits(
        Angle::MIN_NORMALIZED_OUTPUT,
        Angle::MAX_NORMALIZED_OUTPUT);
    angleController.setIntegralOutputLimits(
        Angle::MIN_INTEGRAL_OUTPUT,
        Angle::MAX_INTEGRAL_OUTPUT);
    stopAll();
}

void Shooter::update() {
    angleEncoder.update();

    if (!outputsEnabled) {
        stopAll();
        return;
    }

    updateAngle();
    writeRotateCommand(rotateManualCommand);
    writeFlywheelCommand(flywheelOpenLoopCommand);
}

void Shooter::stopAll() {
    angleMode = AngleControlMode::DISABLED;
    angleManualCommand = 0.0;
    rotateManualCommand = 0.0;
    flywheelOpenLoopCommand = 0.0;
    angleReady = false;
    resetAngleController();
    writeAngleOffset(0);
    writeRotateCommand(0.0);
    writeFlywheelCommand(0.0);
}

void Shooter::setOutputsEnabled(bool enabled) {
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
    angleReady = false;
    resetAngleController();
}

void Shooter::setAngleAction(AngleAction action) {
    setAngleOpenLoop(static_cast<int8_t>(action));
}

bool Shooter::setAngleTargetCounts(long targetCounts) {
    if (!angleHomed || !angleEncoder.isValid()) {
        disableAngle();
        return false;
    }

    const double nextSetpoint = constrain(
        targetCounts,
        Angle::MIN_POSITION_COUNTS,
        Angle::MAX_POSITION_COUNTS);
    const bool targetJumped =
        angleMode != AngleControlMode::CLOSED_LOOP ||
        fabs(nextSetpoint - angleSetpoint) >=
            Angle::SETPOINT_RESET_THRESHOLD_COUNTS;
    angleSetpoint = nextSetpoint;
    if (targetJumped) {
        resetAngleController();
    }
    angleMode = AngleControlMode::CLOSED_LOOP;
    angleReady = false;
    angleReadySinceMs = 0;
    return true;
}

void Shooter::zeroAngleAtCurrentPosition() {
    if (!angleEncoder.isValid()) {
        angleHomed = false;
        disableAngle();
        return;
    }

    angleEncoder.zero();
    angleHomed = true;
    angleSetpoint = 0.0;
    disableAngle();
}

void Shooter::disableAngle() {
    angleMode = AngleControlMode::DISABLED;
    angleManualCommand = 0.0;
    angleReady = false;
    resetAngleController();
    writeAngleOffset(0);
}

void Shooter::setRotateOpenLoop(double command) {
    rotateManualCommand = constrain(command, -1.0, 1.0);
}

void Shooter::setRotateAction(RotateAction action) {
    setRotateOpenLoop(static_cast<int8_t>(action));
}

void Shooter::setFlywheelOpenLoop(double command) {
    flywheelOpenLoopCommand = constrain(command, 0.0, 1.0);
}

void Shooter::setFlywheelAction(FlywheelAction action) {
    setFlywheelOpenLoop(static_cast<uint8_t>(action));
}

bool Shooter::isAngleEncoderValid() {
    return angleEncoder.isValid();
}

bool Shooter::isAngleHomed() {
    return angleHomed;
}

bool Shooter::isAngleReady() {
    return angleReady;
}

bool Shooter::areAngleSoftLimitsActive() {
    return angleHomed && angleEncoder.isValid();
}

bool Shooter::isReady() {
    // Overall readiness also needs rotate-angle and flywheel-speed feedback.
    return false;
}

AngleControlMode Shooter::getAngleControlMode() {
    return angleMode;
}

uint16_t Shooter::getAngleRawCounts() {
    return angleEncoder.getRawCounts();
}

long Shooter::getAngleCounts() {
    return angleEncoder.getRelativeCounts();
}

long Shooter::getAngleTargetCounts() {
    return static_cast<long>(angleSetpoint);
}

double Shooter::getAngleDegrees() {
    return getAngleCounts() * Angle::COUNTS_TO_DEGREES;
}

int Shooter::getAngleErrorCounts() {
    return static_cast<int>(angleSetpoint - angleEncoder.getRelativeCounts());
}

int Shooter::getLeftAnglePulseUs() {
    return leftAnglePulseUs;
}

int Shooter::getRightAnglePulseUs() {
    return rightAnglePulseUs;
}

int Shooter::getRotatePulseUs() {
    return rotatePulseUs;
}

int Shooter::getFlywheelPulseUs() {
    return flywheelPulseUs;
}

double Shooter::getAngleControllerOutput() {
    return angleControllerOutput;
}

double Shooter::getAngleVelocityCountsPerSecond() {
    return angleController.getMeasurementRate();
}

double Shooter::getAngleProportionalTerm() {
    return angleController.getProportionalTerm();
}

double Shooter::getAngleIntegralTerm() {
    return angleController.getIntegralTerm();
}

double Shooter::getAngleDerivativeTerm() {
    return angleController.getDerivativeTerm();
}

double Shooter::getAngleFeedforwardTerm() {
    return angleController.getFeedforwardTerm();
}

bool Shooter::isAngleControllerSaturated() {
    return angleController.isSaturated();
}

unsigned long Shooter::getAngleRejectedSampleCount() {
    return angleEncoder.getRejectedSampleCount();
}

void Shooter::updateAngle() {
    if (angleMode == AngleControlMode::DISABLED) {
        writeAngleOffset(0);
        return;
    }

    if (angleMode == AngleControlMode::MANUAL_OPEN_LOOP) {
        writeAngleManualCommand(
            limitAngleCommand(angleManualCommand));
        angleReady = false;
        return;
    }

    if (!angleHomed || !angleEncoder.isValid()) {
        disableAngle();
        return;
    }

    const unsigned long nowMs = millis();
    const unsigned long elapsedMs = nowMs - lastAngleControlMs;
    if (elapsedMs < Angle::CONTROL_PERIOD_MS) {
        return;
    }

    const double measurement = angleEncoder.getRelativeCounts();
    if (lastAngleControlMs == 0 ||
        elapsedMs > Angle::MAX_CONTROL_GAP_MS) {
        angleController.reset(measurement);
        angleControllerOutput = 0.0;
        lastAngleControlMs = nowMs;
        angleReadySinceMs = 0;
        angleReady = false;
        writeAngleOffset(0);
        return;
    }

    const double dtSeconds = elapsedMs / 1000.0;
    angleControllerOutput = angleController.calculate(
        angleSetpoint,
        measurement,
        dtSeconds,
        Angle::FEEDFORWARD_REFERENCE);
    lastAngleControlMs = nowMs;

    const double limitedControllerOutput =
        limitAngleCommand(angleControllerOutput);
    if (limitedControllerOutput != angleControllerOutput) {
        angleController.reset(measurement);
        angleControllerOutput = 0.0;
        angleReadySinceMs = 0;
        angleReady = false;
        writeAngleOffset(0);
        return;
    }

    const bool positionReady =
        fabs(angleController.getError()) <=
        Angle::READY_TOLERANCE_COUNTS;
    const bool velocityReady =
        fabs(angleController.getMeasurementRate()) <=
        Angle::READY_VELOCITY_TOLERANCE_COUNTS_PER_SECOND;

    if (positionReady && velocityReady) {
        angleControllerOutput = 0.0;
        writeAngleOffset(0);
        if (angleReadySinceMs == 0) {
            angleReadySinceMs = nowMs;
        }
        angleReady =
            nowMs - angleReadySinceMs >=
            Angle::READY_SETTLE_TIME_MS;
        return;
    }

    angleReadySinceMs = 0;
    angleReady = false;
    const double scaledOffset =
        angleControllerOutput * Angle::MAX_CLOSED_LOOP_OFFSET_US;
    const int offsetUs = static_cast<int>(
        scaledOffset + (scaledOffset >= 0.0 ? 0.5 : -0.5));
    writeAngleOffset(offsetUs);
}

void Shooter::writeAngleManualCommand(double command) {
    leftAnglePulseUs = commandToPulseUs(
        command,
        Angle::LEFT_MOTOR);
    rightAnglePulseUs = commandToPulseUs(
        command,
        Angle::RIGHT_MOTOR);
    angleLeftServo.writeMicroseconds(leftAnglePulseUs);
    angleRightServo.writeMicroseconds(rightAnglePulseUs);
}

void Shooter::writeAngleOffset(int offsetUs) {
    offsetUs = constrain(
        offsetUs,
        -Angle::MAX_CLOSED_LOOP_OFFSET_US,
        Angle::MAX_CLOSED_LOOP_OFFSET_US);

    leftAnglePulseUs = offsetToPulseUs(
        offsetUs,
        Angle::LEFT_MOTOR);
    rightAnglePulseUs = offsetToPulseUs(
        offsetUs,
        Angle::RIGHT_MOTOR);
    angleLeftServo.writeMicroseconds(leftAnglePulseUs);
    angleRightServo.writeMicroseconds(rightAnglePulseUs);
}

void Shooter::writeRotateCommand(double command) {
    rotatePulseUs = commandToPulseUs(command, Rotate::MOTOR);
    rotateServo.writeMicroseconds(rotatePulseUs);
}

void Shooter::writeFlywheelCommand(double command) {
    flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR);
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}

void Shooter::resetAngleController() {
    const double measurement = angleEncoder.getRelativeCounts();
    angleController.reset(measurement);
    angleControllerOutput = 0.0;
    lastAngleControlMs = millis();
    angleReadySinceMs = 0;
}

double Shooter::limitAngleCommand(double command) {
    command = constrain(command, -1.0, 1.0);
    if (!areAngleSoftLimitsActive()) {
        return command;
    }

    const long counts = angleEncoder.getRelativeCounts();
    if (command > 0.0 &&
        Angle::SOFT_LIMIT.forwardEnabled &&
        counts >= Angle::SOFT_LIMIT.forwardLimit) {
        return 0.0;
    }
    if (command < 0.0 &&
        Angle::SOFT_LIMIT.reverseEnabled &&
        counts <= Angle::SOFT_LIMIT.reverseLimit) {
        return 0.0;
    }
    return command;
}
