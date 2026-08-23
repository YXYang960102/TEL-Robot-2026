#include "Shooter.h"

#include "../Constants/Pins.h"
#include "../Constants/ShooterConstants.h"

using namespace ShooterConst;

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
        ? config.neutralUs + command * (config.forwardUs - config.neutralUs)
        : config.neutralUs + (-command) * (config.reverseUs - config.neutralUs);
    return static_cast<int>(pulseUs + (pulseUs >= 0.0 ? 0.5 : -0.5));
}

int offsetToPulseUs(int offsetUs, const ServoMotorConfig& config) {
    const int appliedOffset = config.inverted ? -offsetUs : offsetUs;
    return constrain(
        config.neutralUs + appliedOffset,
        config.reverseUs,
        config.forwardUs);
}

}

Servo Shooter::elevationLeftServo;
Servo Shooter::elevationRightServo;
Servo Shooter::turretServo;
Servo Shooter::flywheelOutput;
AS5600Encoder Shooter::elevationEncoder;
ElevationControlMode Shooter::elevationMode = ElevationControlMode::DISABLED;
bool Shooter::outputsEnabled = false;
bool Shooter::elevationHomed = false;
bool Shooter::elevationReady = false;
double Shooter::elevationManualCommand = 0.0;
double Shooter::turretManualCommand = 0.0;
double Shooter::flywheelOpenLoopCommand = 0.0;
double Shooter::elevationSetpoint = 0.0;
PidfController Shooter::angleController(kAnglePidfConfig);
double Shooter::angleControllerOutput = 0.0;
unsigned long Shooter::lastAngleControlMs = 0;
unsigned long Shooter::angleReadySinceMs = 0;
int Shooter::leftElevationPulseUs = Elevation::LEFT_MOTOR_CONFIG.neutralUs;
int Shooter::rightElevationPulseUs = Elevation::RIGHT_MOTOR_CONFIG.neutralUs;
int Shooter::turretPulseUs = Turret::MOTOR_CONFIG.neutralUs;
int Shooter::flywheelPulseUs = Flywheel::MOTOR_CONFIG.neutralUs;

void Shooter::init() {
    elevationLeftServo.attach(
        PIN_SHOOTER_ELEVATION_LEFT,
        Elevation::SERVO_ATTACH_MIN_US,
        Elevation::SERVO_ATTACH_MAX_US);
    elevationRightServo.attach(
        PIN_SHOOTER_ELEVATION_RIGHT,
        Elevation::SERVO_ATTACH_MIN_US,
        Elevation::SERVO_ATTACH_MAX_US);
    turretServo.attach(
        PIN_SHOOTER_TURRET,
        Turret::ATTACH_MIN_US,
        Turret::ATTACH_MAX_US);
    flywheelOutput.attach(
        PIN_SHOOTER_FLYWHEEL,
        Flywheel::ATTACH_MIN_US,
        Flywheel::ATTACH_MAX_US);

    elevationEncoder.begin();
    angleController.setOutputLimits(
        Elevation::MIN_NORMALIZED_OUTPUT,
        Elevation::MAX_NORMALIZED_OUTPUT);
    angleController.setIntegralOutputLimits(
        Elevation::MIN_INTEGRAL_OUTPUT,
        Elevation::MAX_INTEGRAL_OUTPUT);
    stop();
}

void Shooter::update() {
    elevationEncoder.update();

    if (!outputsEnabled) {
        stop();
        return;
    }

    updateElevation();
    writeTurretCommand(turretManualCommand);
    writeFlywheelCommand(flywheelOpenLoopCommand);
}

void Shooter::stop() {
    elevationMode = ElevationControlMode::DISABLED;
    elevationManualCommand = 0.0;
    turretManualCommand = 0.0;
    flywheelOpenLoopCommand = 0.0;
    elevationReady = false;
    resetElevationController();
    writeElevationOffset(0);
    writeTurretCommand(0.0);
    writeFlywheelCommand(0.0);
}

void Shooter::setOutputsEnabled(bool enabled) {
    outputsEnabled = enabled;
    if (!outputsEnabled) {
        stop();
    }
}

bool Shooter::areOutputsEnabled() {
    return outputsEnabled;
}

void Shooter::setElevationManual(double command) {
    elevationManualCommand = constrain(command, -1.0, 1.0);
    elevationMode = ElevationControlMode::MANUAL_OPEN_LOOP;
    elevationReady = false;
    resetElevationController();
}

void Shooter::setAngleSpeed(OpenLoopAction action) {
    setElevationManual(actionToCommand(action));
}

bool Shooter::setElevationTargetCounts(long targetCounts) {
    if (!elevationHomed || !elevationEncoder.isValid()) {
        disableElevation();
        return false;
    }

    const double nextSetpoint = constrain(
        targetCounts,
        Elevation::MIN_TARGET_COUNTS,
        Elevation::MAX_TARGET_COUNTS);
    const bool targetJumped =
        elevationMode != ElevationControlMode::CLOSED_LOOP ||
        fabs(nextSetpoint - elevationSetpoint) >=
            Elevation::SETPOINT_RESET_THRESHOLD_COUNTS;
    elevationSetpoint = nextSetpoint;
    if (targetJumped) {
        resetElevationController();
    }
    elevationMode = ElevationControlMode::CLOSED_LOOP;
    elevationReady = false;
    angleReadySinceMs = 0;
    return true;
}

void Shooter::zeroElevationAtCurrentPosition() {
    if (!elevationEncoder.isValid()) {
        elevationHomed = false;
        disableElevation();
        return;
    }

    elevationEncoder.zero();
    elevationHomed = true;
    elevationSetpoint = 0.0;
    disableElevation();
}

void Shooter::disableElevation() {
    elevationMode = ElevationControlMode::DISABLED;
    elevationManualCommand = 0.0;
    elevationReady = false;
    resetElevationController();
    writeElevationOffset(0);
}

void Shooter::setTurretManual(double command) {
    turretManualCommand = constrain(command, -1.0, 1.0);
}

void Shooter::setRotateSpeed(OpenLoopAction action) {
    setTurretManual(actionToCommand(action));
}

void Shooter::setFlywheelOpenLoop(double command) {
    flywheelOpenLoopCommand = constrain(command, 0.0, 1.0);
}

void Shooter::setFlywheelSpeed(FlywheelAction action) {
    setFlywheelOpenLoop(static_cast<uint8_t>(action));
}

bool Shooter::isEncoderValid() {
    return elevationEncoder.isValid();
}

bool Shooter::isElevationHomed() {
    return elevationHomed;
}

bool Shooter::isElevationReady() {
    return elevationReady;
}

bool Shooter::areElevationSoftLimitsActive() {
    return elevationHomed && elevationEncoder.isValid();
}

bool Shooter::isReady() {
    // Overall readiness also needs turret-angle and flywheel-speed feedback.
    return false;
}

long Shooter::getElevationCounts() {
    return elevationEncoder.getRelativeCounts();
}

long Shooter::getElevationTargetCounts() {
    return static_cast<long>(elevationSetpoint);
}

double Shooter::getElevationDegrees() {
    return getElevationCounts() * Encoder::COUNTS_TO_DEGREES;
}

int Shooter::getElevationErrorCounts() {
    return static_cast<int>(elevationSetpoint - elevationEncoder.getRelativeCounts());
}

int Shooter::getLeftElevationPulseUs() {
    return leftElevationPulseUs;
}

int Shooter::getRightElevationPulseUs() {
    return rightElevationPulseUs;
}

int Shooter::getTurretPulseUs() {
    return turretPulseUs;
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

void Shooter::updateElevation() {
    if (elevationMode == ElevationControlMode::DISABLED) {
        writeElevationOffset(0);
        return;
    }

    if (elevationMode == ElevationControlMode::MANUAL_OPEN_LOOP) {
        writeElevationManualCommand(
            limitElevationCommand(elevationManualCommand));
        elevationReady = false;
        return;
    }

    if (!elevationHomed || !elevationEncoder.isValid()) {
        disableElevation();
        return;
    }

    const unsigned long nowMs = millis();
    const unsigned long elapsedMs = nowMs - lastAngleControlMs;
    if (elapsedMs < Elevation::CONTROL_PERIOD_MS) {
        return;
    }

    const double measurement = elevationEncoder.getRelativeCounts();
    if (lastAngleControlMs == 0 ||
        elapsedMs > Elevation::MAX_CONTROL_GAP_MS) {
        angleController.reset(measurement);
        angleControllerOutput = 0.0;
        lastAngleControlMs = nowMs;
        angleReadySinceMs = 0;
        elevationReady = false;
        writeElevationOffset(0);
        return;
    }

    const double dtSeconds = elapsedMs / 1000.0;
    angleControllerOutput = angleController.calculate(
        elevationSetpoint,
        measurement,
        dtSeconds,
        Elevation::FEEDFORWARD_REFERENCE);
    lastAngleControlMs = nowMs;

    const double limitedControllerOutput =
        limitElevationCommand(angleControllerOutput);
    if (limitedControllerOutput != angleControllerOutput) {
        angleController.reset(measurement);
        angleControllerOutput = 0.0;
        angleReadySinceMs = 0;
        elevationReady = false;
        writeElevationOffset(0);
        return;
    }

    const bool positionReady =
        fabs(angleController.getError()) <=
        Elevation::READY_TOLERANCE_COUNTS;
    const bool velocityReady =
        fabs(angleController.getMeasurementRate()) <=
        Elevation::READY_VELOCITY_TOLERANCE_COUNTS_PER_SECOND;

    if (positionReady && velocityReady) {
        angleControllerOutput = 0.0;
        writeElevationOffset(0);
        if (angleReadySinceMs == 0) {
            angleReadySinceMs = nowMs;
        }
        elevationReady =
            nowMs - angleReadySinceMs >=
            Elevation::READY_SETTLE_TIME_MS;
        return;
    }

    angleReadySinceMs = 0;
    elevationReady = false;
    const double scaledOffset =
        angleControllerOutput * Elevation::MAX_CLOSED_LOOP_OFFSET_US;
    const int offsetUs = static_cast<int>(
        scaledOffset + (scaledOffset >= 0.0 ? 0.5 : -0.5));
    writeElevationOffset(offsetUs);
}

void Shooter::writeElevationManualCommand(double command) {
    leftElevationPulseUs = commandToPulseUs(
        command,
        Elevation::LEFT_MOTOR_CONFIG);
    rightElevationPulseUs = commandToPulseUs(
        command,
        Elevation::RIGHT_MOTOR_CONFIG);
    elevationLeftServo.writeMicroseconds(leftElevationPulseUs);
    elevationRightServo.writeMicroseconds(rightElevationPulseUs);
}

void Shooter::writeElevationOffset(int offsetUs) {
    offsetUs = constrain(
        offsetUs,
        -Elevation::MAX_CLOSED_LOOP_OFFSET_US,
        Elevation::MAX_CLOSED_LOOP_OFFSET_US);

    leftElevationPulseUs = offsetToPulseUs(
        offsetUs,
        Elevation::LEFT_MOTOR_CONFIG);
    rightElevationPulseUs = offsetToPulseUs(
        offsetUs,
        Elevation::RIGHT_MOTOR_CONFIG);
    elevationLeftServo.writeMicroseconds(leftElevationPulseUs);
    elevationRightServo.writeMicroseconds(rightElevationPulseUs);
}

void Shooter::writeTurretCommand(double command) {
    turretPulseUs = commandToPulseUs(command, Turret::MOTOR_CONFIG);
    turretServo.writeMicroseconds(turretPulseUs);
}

void Shooter::writeFlywheelCommand(double command) {
    flywheelPulseUs = commandToPulseUs(command, Flywheel::MOTOR_CONFIG);
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}

void Shooter::resetElevationController() {
    const double measurement = elevationEncoder.getRelativeCounts();
    angleController.reset(measurement);
    angleControllerOutput = 0.0;
    lastAngleControlMs = millis();
    angleReadySinceMs = 0;
}

double Shooter::actionToCommand(OpenLoopAction action) {
    return static_cast<int8_t>(action);
}

double Shooter::limitElevationCommand(double command) {
    command = constrain(command, -1.0, 1.0);
    if (!areElevationSoftLimitsActive()) {
        return command;
    }

    const long counts = elevationEncoder.getRelativeCounts();
    if (command > 0.0 &&
        Elevation::SOFT_LIMIT_CONFIG.forwardEnabled &&
        counts >= Elevation::SOFT_LIMIT_CONFIG.forwardLimitCounts) {
        return 0.0;
    }
    if (command < 0.0 &&
        Elevation::SOFT_LIMIT_CONFIG.reverseEnabled &&
        counts <= Elevation::SOFT_LIMIT_CONFIG.reverseLimitCounts) {
        return 0.0;
    }
    return command;
}
