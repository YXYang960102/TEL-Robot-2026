#include "Shooter.h"

#include "../Control/DirectionalLimit.h"
#include "../Control/PositionDecelerationProfile.h"

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
AnalogPositionSensor Shooter::rotatePositionSensor(Rotate::POSITION_SENSOR);
AngleControlMode Shooter::angleMode = AngleControlMode::DISABLED;
RotateControlMode Shooter::rotateMode = RotateControlMode::DISABLED;
bool Shooter::outputsEnabled = false;
bool Shooter::angleHomed = false;
bool Shooter::angleReady = false;
bool Shooter::rotateReady = false;
double Shooter::angleManualCommand = 0.0;
double Shooter::rotateManualCommand = 0.0;
double Shooter::flywheelOpenLoopCommand = 0.0;
double Shooter::angleSetpoint = 0.0;
double Shooter::rotateSetpointRaw = Rotate::CENTER_POSITION_RAW;
double Shooter::rotateInitialErrorMagnitudeRaw = 0.0;
double Shooter::rotateMaximumCommand = Rotate::PROFILE_CRUISE_COMMAND;
PidfController Shooter::angleController(Angle::PIDF);
PidfController Shooter::rotateController(Rotate::PIDF);
double Shooter::angleControllerOutput = 0.0;
double Shooter::rotateControllerOutput = 0.0;
double Shooter::rotateProfileEnvelope = 0.0;
unsigned long Shooter::lastAngleControlMs = 0;
unsigned long Shooter::lastRotateControlMs = 0;
unsigned long Shooter::angleReadySinceMs = 0;
unsigned long Shooter::rotateReadySinceMs = 0;
int Shooter::leftAnglePulseUs = Angle::LEFT_MOTOR.neutralUs;
int Shooter::rightAnglePulseUs = Angle::RIGHT_MOTOR.neutralUs;
int Shooter::rotatePulseUs = Rotate::MOTOR.neutralUs;
int Shooter::flywheelPulseUs = Flywheel::MOTOR.neutralUs;
Shooter::DebouncedLimitSwitchState Shooter::angleUpLimitState = {
    false, false, 0};
Shooter::DebouncedLimitSwitchState Shooter::angleDownLimitState = {
    false, false, 0};
Shooter::DebouncedLimitSwitchState Shooter::rotateLeftLimitState = {
    false, false, 0};
Shooter::DebouncedLimitSwitchState Shooter::rotateRightLimitState = {
    false, false, 0};

void Shooter::init() {
    initializeLimitSwitch(Angle::UP_LIMIT_SWITCH, angleUpLimitState);
    initializeLimitSwitch(Angle::DOWN_LIMIT_SWITCH, angleDownLimitState);
    initializeLimitSwitch(Rotate::LEFT_LIMIT_SWITCH, rotateLeftLimitState);
    initializeLimitSwitch(Rotate::RIGHT_LIMIT_SWITCH, rotateRightLimitState);

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
    rotatePositionSensor.begin();
    angleController.setOutputLimits(
        Angle::MIN_NORMALIZED_OUTPUT,
        Angle::MAX_NORMALIZED_OUTPUT);
    angleController.setIntegralOutputLimits(
        Angle::MIN_INTEGRAL_OUTPUT,
        Angle::MAX_INTEGRAL_OUTPUT);
    rotateController.setOutputLimits(
        Rotate::MIN_NORMALIZED_OUTPUT,
        Rotate::MAX_NORMALIZED_OUTPUT);
    rotateController.setIntegralOutputLimits(
        Rotate::MIN_INTEGRAL_OUTPUT,
        Rotate::MAX_INTEGRAL_OUTPUT);
    stopAll();
}

void Shooter::update() {
    angleEncoder.update();
    rotatePositionSensor.update();
    updateLimitSwitches();

    if (!outputsEnabled) {
        stopAll();
        return;
    }

    updateAngle();
    updateRotate();
    writeFlywheelCommand(flywheelOpenLoopCommand);
}

void Shooter::stopAll() {
    angleMode = AngleControlMode::DISABLED;
    angleManualCommand = 0.0;
    rotateManualCommand = 0.0;
    flywheelOpenLoopCommand = 0.0;
    angleReady = false;
    rotateReady = false;
    resetAngleController();
    resetRotateController();
    rotateMode = RotateControlMode::DISABLED;
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
    if (!Angle::POSITION_LIMITS_CALIBRATED ||
        !angleHomed ||
        !angleEncoder.isValid()) {
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
    rotateMode = RotateControlMode::MANUAL_OPEN_LOOP;
    rotateReady = false;
    resetRotateController();
}

void Shooter::setRotateAction(RotateAction action) {
    setRotateOpenLoop(static_cast<int8_t>(action));
}

bool Shooter::setRotateProfiledTargetRaw(
    int targetRaw,
    double maximumCommand) {
    return configureRotateTarget(
        targetRaw,
        RotateControlMode::PROFILED_POSITION,
        maximumCommand);
}

bool Shooter::setRotateTargetRaw(
    int targetRaw,
    double maximumCommand) {
    return configureRotateTarget(
        targetRaw,
        RotateControlMode::CLOSED_LOOP,
        maximumCommand);
}

void Shooter::disableRotate() {
    rotateMode = RotateControlMode::DISABLED;
    rotateManualCommand = 0.0;
    rotateReady = false;
    rotateProfileEnvelope = 0.0;
    resetRotateController();
    writeRotateCommand(0.0);
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
    return Angle::POSITION_LIMITS_CALIBRATED &&
           (Angle::FORWARD_SOFT_LIMIT_ENABLED ||
            Angle::REVERSE_SOFT_LIMIT_ENABLED) &&
           angleHomed &&
           angleEncoder.isValid();
}

bool Shooter::isAngleUpLimitTriggered() {
    return angleUpLimitState.stableTriggered;
}

bool Shooter::isAngleDownLimitTriggered() {
    return angleDownLimitState.stableTriggered;
}

bool Shooter::isRotateLeftLimitTriggered() {
    return rotateLeftLimitState.stableTriggered;
}

bool Shooter::isRotateRightLimitTriggered() {
    return rotateRightLimitState.stableTriggered;
}

bool Shooter::isRotatePositionSensorValid() {
    return rotatePositionSensor.isValid();
}

bool Shooter::isRotateReady() {
    return rotateReady;
}

bool Shooter::areRotateSoftLimitsActive() {
    return Rotate::POSITION_CALIBRATED &&
           (Rotate::FORWARD_SOFT_LIMIT_ENABLED ||
            Rotate::REVERSE_SOFT_LIMIT_ENABLED) &&
           rotatePositionSensor.isValid();
}

bool Shooter::isReady() {
    // Overall readiness also needs rotate-angle and flywheel-speed feedback.
    return false;
}

AngleControlMode Shooter::getAngleControlMode() {
    return angleMode;
}

RotateControlMode Shooter::getRotateControlMode() {
    return rotateMode;
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

int Shooter::getRotatePositionRaw() {
    return rotatePositionSensor.getRaw();
}

double Shooter::getRotatePositionFilteredRaw() {
    return rotatePositionSensor.getFilteredRaw();
}

double Shooter::getRotateDegrees() {
    return rotateRawToDegrees(getRotatePositionFilteredRaw());
}

int Shooter::getRotateTargetRaw() {
    return static_cast<int>(rotateSetpointRaw + 0.5);
}

double Shooter::getRotateTargetDegrees() {
    return rotateRawToDegrees(rotateSetpointRaw);
}

int Shooter::getRotateErrorRaw() {
    const double error =
        rotateSetpointRaw - getRotatePositionFilteredRaw();
    return static_cast<int>(error + (error >= 0.0 ? 0.5 : -0.5));
}

double Shooter::getRotateMoveProgress() {
    if (rotateInitialErrorMagnitudeRaw <= 0.0) {
        return fabs(
            rotateSetpointRaw - getRotatePositionFilteredRaw()) <=
            Rotate::READY_TOLERANCE_RAW
            ? 1.0
            : 0.0;
    }
    const double remaining = fabs(
        rotateSetpointRaw - getRotatePositionFilteredRaw());
    return constrain(
        1.0 - remaining / rotateInitialErrorMagnitudeRaw,
        0.0,
        1.0);
}

double Shooter::getRotateProfileEnvelope() {
    return rotateProfileEnvelope;
}

double Shooter::getRotateControllerOutput() {
    return rotateControllerOutput;
}

double Shooter::getRotateProportionalTerm() {
    return rotateController.getProportionalTerm();
}

double Shooter::getRotateIntegralTerm() {
    return rotateController.getIntegralTerm();
}

double Shooter::getRotateDerivativeTerm() {
    return rotateController.getDerivativeTerm();
}

double Shooter::getRotateFeedforwardTerm() {
    return rotateController.getFeedforwardTerm();
}

bool Shooter::isRotateControllerSaturated() {
    return rotateController.isSaturated();
}

void Shooter::initializeLimitSwitch(
    const DigitalLimitSwitchConfig& config,
    DebouncedLimitSwitchState& state) {
    pinMode(
        config.signalPin,
        config.useInternalPullup ? INPUT_PULLUP : INPUT);
    state.rawTriggered = readLimitSwitch(config);
    state.stableTriggered = state.rawTriggered;
    state.rawChangedMs = millis();
}

void Shooter::updateLimitSwitch(
    const DigitalLimitSwitchConfig& config,
    DebouncedLimitSwitchState& state,
    unsigned long nowMs) {
    const bool triggered = readLimitSwitch(config);
    if (triggered != state.rawTriggered) {
        state.rawTriggered = triggered;
        state.rawChangedMs = nowMs;
    }
    if (state.stableTriggered != state.rawTriggered &&
        nowMs - state.rawChangedMs >= config.debounceMs) {
        state.stableTriggered = state.rawTriggered;
    }
}

bool Shooter::readLimitSwitch(
    const DigitalLimitSwitchConfig& config) {
    const bool signalHigh = digitalRead(config.signalPin) == HIGH;
    return config.triggeredHigh ? signalHigh : !signalHigh;
}

void Shooter::updateLimitSwitches() {
    const unsigned long nowMs = millis();
    updateLimitSwitch(
        Angle::UP_LIMIT_SWITCH,
        angleUpLimitState,
        nowMs);
    updateLimitSwitch(
        Angle::DOWN_LIMIT_SWITCH,
        angleDownLimitState,
        nowMs);
    updateLimitSwitch(
        Rotate::LEFT_LIMIT_SWITCH,
        rotateLeftLimitState,
        nowMs);
    updateLimitSwitch(
        Rotate::RIGHT_LIMIT_SWITCH,
        rotateRightLimitState,
        nowMs);
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

void Shooter::updateRotate() {
    if (rotateMode == RotateControlMode::DISABLED) {
        writeRotateCommand(0.0);
        return;
    }

    if (rotateMode == RotateControlMode::MANUAL_OPEN_LOOP) {
        rotateControllerOutput = 0.0;
        rotateProfileEnvelope = 0.0;
        rotateReady = false;
        rotateReadySinceMs = 0;
        writeRotateCommand(rotateManualCommand);
        return;
    }

    if (!rotatePositionSensor.isValid()) {
        disableRotate();
        return;
    }

    const double measurement = rotatePositionSensor.getFilteredRaw();
    const double error = rotateSetpointRaw - measurement;
    const double errorMagnitude = fabs(error);
    const unsigned long nowMs = millis();

    if (errorMagnitude <= Rotate::READY_TOLERANCE_RAW) {
        rotateControllerOutput = 0.0;
        rotateProfileEnvelope = 0.0;
        rotateController.reset(measurement);
        lastRotateControlMs = nowMs;
        writeRotateCommand(0.0);
        if (rotateReadySinceMs == 0) {
            rotateReadySinceMs = nowMs;
        }
        rotateReady =
            nowMs - rotateReadySinceMs >=
            Rotate::READY_SETTLE_TIME_MS;
        return;
    }

    rotateReady = false;
    rotateReadySinceMs = 0;
    const double profileReference = max(
        rotateInitialErrorMagnitudeRaw,
        errorMagnitude);
    rotateProfileEnvelope =
        PositionDecelerationProfile::calculateMaximumCommand(
            profileReference,
            errorMagnitude,
            Rotate::PROFILE_DECELERATION_FRACTION,
            rotateMaximumCommand,
            rotateMaximumCommand *
                Rotate::PROFILE_MINIMUM_APPROACH_RATIO);

    if (rotateMode == RotateControlMode::PROFILED_POSITION) {
        rotateControllerOutput = error > 0.0
            ? rotateProfileEnvelope
            : -rotateProfileEnvelope;
        const double limitedOutput =
            limitRotateCommand(rotateControllerOutput);
        if (limitedOutput != rotateControllerOutput) {
            rotateControllerOutput = 0.0;
        }
        writeRotateCommand(rotateControllerOutput);
        return;
    }

    const unsigned long elapsedMs = nowMs - lastRotateControlMs;
    if (elapsedMs < Rotate::CONTROL_PERIOD_MS) {
        return;
    }
    if (lastRotateControlMs == 0 ||
        elapsedMs > Rotate::MAX_CONTROL_GAP_MS) {
        rotateController.reset(measurement);
        rotateControllerOutput = 0.0;
        lastRotateControlMs = nowMs;
        writeRotateCommand(0.0);
        return;
    }

    const double feedforwardReference = error > 0.0 ? 1.0 : -1.0;
    rotateControllerOutput = rotateController.calculate(
        rotateSetpointRaw,
        measurement,
        elapsedMs / 1000.0,
        feedforwardReference);
    lastRotateControlMs = nowMs;
    rotateControllerOutput = constrain(
        rotateControllerOutput,
        -rotateProfileEnvelope,
        rotateProfileEnvelope);

    const double limitedOutput =
        limitRotateCommand(rotateControllerOutput);
    if (limitedOutput != rotateControllerOutput) {
        rotateController.reset(measurement);
        rotateControllerOutput = 0.0;
        writeRotateCommand(0.0);
        return;
    }
    writeRotateCommand(rotateControllerOutput);
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
    command = limitRotateCommand(command);
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

void Shooter::resetRotateController() {
    const double measurement = rotatePositionSensor.getFilteredRaw();
    rotateController.reset(measurement);
    rotateControllerOutput = 0.0;
    lastRotateControlMs = millis();
    rotateReadySinceMs = 0;
}

double Shooter::limitAngleCommand(double command) {
    command = constrain(command, -1.0, 1.0);
    command = DirectionalLimit::apply(
        command,
        isAngleDownLimitTriggered(),
        isAngleUpLimitTriggered());
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

double Shooter::limitRotateCommand(double command) {
    command = constrain(command, -1.0, 1.0);
    command = DirectionalLimit::apply(
        command,
        isRotateLeftLimitTriggered(),
        isRotateRightLimitTriggered());
    if (!areRotateSoftLimitsActive()) {
        return command;
    }

    const double position = rotatePositionSensor.getFilteredRaw();
    if (command > 0.0 &&
        Rotate::SOFT_LIMIT.forwardEnabled &&
        position >= Rotate::SOFT_LIMIT.forwardLimit) {
        return 0.0;
    }
    if (command < 0.0 &&
        Rotate::SOFT_LIMIT.reverseEnabled &&
        position <= Rotate::SOFT_LIMIT.reverseLimit) {
        return 0.0;
    }
    return command;
}

bool Shooter::configureRotateTarget(
    int targetRaw,
    RotateControlMode mode,
    double maximumCommand) {
    if (!Rotate::POSITION_CALIBRATED ||
        !rotatePositionSensor.isValid() ||
        (mode != RotateControlMode::PROFILED_POSITION &&
         mode != RotateControlMode::CLOSED_LOOP)) {
        disableRotate();
        return false;
    }

    const double nextSetpoint = constrain(
        targetRaw,
        Rotate::LEFT_POSITION_RAW,
        Rotate::RIGHT_POSITION_RAW);
    const double nextMaximumCommand = constrain(
        fabs(maximumCommand),
        0.0,
        1.0);
    if (nextMaximumCommand <= 0.0) {
        disableRotate();
        return false;
    }

    const bool commandChanged =
        rotateMode != mode ||
        fabs(nextSetpoint - rotateSetpointRaw) >=
            Rotate::SETPOINT_RESET_THRESHOLD_RAW ||
        fabs(nextMaximumCommand - rotateMaximumCommand) > 0.0001;
    rotateSetpointRaw = nextSetpoint;
    rotateMaximumCommand = nextMaximumCommand;
    rotateMode = mode;
    rotateReady = false;
    rotateReadySinceMs = 0;
    if (commandChanged) {
        rotateInitialErrorMagnitudeRaw = fabs(
            rotateSetpointRaw -
            rotatePositionSensor.getFilteredRaw());
        rotateProfileEnvelope = 0.0;
        resetRotateController();
    }
    return true;
}

double Shooter::rotateRawToDegrees(double raw) {
    if (!Rotate::POSITION_CALIBRATED) {
        return 0.0;
    }

    if (raw <= Rotate::CENTER_POSITION_RAW) {
        return Rotate::LEFT_POSITION_DEGREES +
            (raw - Rotate::LEFT_POSITION_RAW) *
            (Rotate::CENTER_POSITION_DEGREES -
             Rotate::LEFT_POSITION_DEGREES) /
            (Rotate::CENTER_POSITION_RAW -
             Rotate::LEFT_POSITION_RAW);
    }
    return Rotate::CENTER_POSITION_DEGREES +
        (raw - Rotate::CENTER_POSITION_RAW) *
        (Rotate::RIGHT_POSITION_DEGREES -
         Rotate::CENTER_POSITION_DEGREES) /
        (Rotate::RIGHT_POSITION_RAW -
         Rotate::CENTER_POSITION_RAW);
}
