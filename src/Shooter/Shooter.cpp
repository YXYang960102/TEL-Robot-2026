#include "Shooter.h"

#include "../Constants/Pins.h"
#include "../Constants/ShooterConstants.h"

using namespace ShooterConst;

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
double Shooter::elevationInput = 0.0;
double Shooter::elevationOutput = 0.0;
double Shooter::elevationSetpoint = 0.0;
PID Shooter::elevationPid(
    &Shooter::elevationInput,
    &Shooter::elevationOutput,
    &Shooter::elevationSetpoint,
    Elevation::KP,
    Elevation::KI,
    Elevation::KD,
    DIRECT);
int Shooter::leftElevationPulseUs = Elevation::LEFT_STOP_US;
int Shooter::rightElevationPulseUs = Elevation::RIGHT_STOP_US;
int Shooter::turretPulseUs = Turret::STOP_US;
int Shooter::flywheelPulseUs = Flywheel::STOP_US;

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
    elevationPid.SetOutputLimits(
        -Elevation::MAX_OUTPUT_OFFSET_US,
        Elevation::MAX_OUTPUT_OFFSET_US);
    elevationPid.SetSampleTime(Elevation::PID_SAMPLE_TIME_MS);
    elevationPid.SetMode(MANUAL);
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

bool Shooter::setElevationTargetCounts(long targetCounts) {
    if (!elevationHomed || !elevationEncoder.isValid()) {
        disableElevation();
        return false;
    }

    elevationSetpoint = constrain(
        targetCounts,
        Elevation::MIN_TARGET_COUNTS,
        Elevation::MAX_TARGET_COUNTS);
    elevationInput = elevationEncoder.getRelativeCounts();
    elevationOutput = 0.0;
    elevationMode = ElevationControlMode::CLOSED_LOOP;
    elevationReady = false;
    elevationPid.SetMode(AUTOMATIC);
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

void Shooter::setFlywheelOpenLoop(double command) {
    flywheelOpenLoopCommand = constrain(command, 0.0, 1.0);
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

void Shooter::updateElevation() {
    if (elevationMode == ElevationControlMode::DISABLED) {
        writeElevationOffset(0);
        return;
    }

    if (elevationMode == ElevationControlMode::MANUAL_OPEN_LOOP) {
        const int offsetUs = static_cast<int>(
            elevationManualCommand * Elevation::MAX_OUTPUT_OFFSET_US);
        writeElevationOffset(offsetUs);
        elevationReady = false;
        return;
    }

    if (!elevationHomed || !elevationEncoder.isValid()) {
        disableElevation();
        return;
    }

    elevationInput = elevationEncoder.getRelativeCounts();
    const long errorCounts = static_cast<long>(elevationSetpoint - elevationInput);
    if (labs(errorCounts) <= Elevation::READY_TOLERANCE_COUNTS) {
        writeElevationOffset(0);
        elevationReady = true;
        return;
    }

    elevationReady = false;
    elevationPid.Compute();
    writeElevationOffset(static_cast<int>(elevationOutput));
}

void Shooter::writeElevationOffset(int offsetUs) {
    offsetUs = constrain(
        offsetUs,
        -Elevation::MAX_OUTPUT_OFFSET_US,
        Elevation::MAX_OUTPUT_OFFSET_US);

    leftElevationPulseUs = Elevation::LEFT_STOP_US +
                           Elevation::LEFT_OUTPUT_SIGN * offsetUs;
    rightElevationPulseUs = Elevation::RIGHT_STOP_US +
                            Elevation::RIGHT_OUTPUT_SIGN * offsetUs;
    elevationLeftServo.writeMicroseconds(leftElevationPulseUs);
    elevationRightServo.writeMicroseconds(rightElevationPulseUs);
}

void Shooter::writeTurretCommand(double command) {
    command = constrain(command, -1.0, 1.0);
    turretPulseUs = Turret::STOP_US + static_cast<int>(
        Turret::OUTPUT_SIGN * command * Turret::MAX_MANUAL_OFFSET_US);
    turretServo.writeMicroseconds(turretPulseUs);
}

void Shooter::writeFlywheelCommand(double command) {
    command = constrain(command, 0.0, 1.0);
    flywheelPulseUs = Flywheel::STOP_US + static_cast<int>(
        command * (Flywheel::MAX_OPEN_LOOP_US - Flywheel::STOP_US));
    flywheelOutput.writeMicroseconds(flywheelPulseUs);
}

void Shooter::resetElevationController() {
    elevationPid.SetMode(MANUAL);
    elevationInput = elevationEncoder.getRelativeCounts();
    elevationOutput = 0.0;
}
