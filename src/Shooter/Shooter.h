#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Constants/ShooterConstants.h"
#include "../Control/PidfController.h"
#include "../Sensors/AnalogPositionSensor.h"
#include "../Sensors/AS5600Encoder.h"

enum class AngleControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    CLOSED_LOOP
};

enum class RotateControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    PROFILED_POSITION,
    CLOSED_LOOP
};

class Shooter {
public:
    using AngleAction = ShooterConstants::Angle::Action;
    using RotateAction = ShooterConstants::Rotate::Action;
    using FlywheelAction = ShooterConstants::Flywheel::Action;

    // Lifecycle and safety
    static void init();
    static void update();
    static void stopAll();
    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();

    // Manual open-loop control
    static void setAngleAction(AngleAction action);
    static void setAngleOpenLoop(double command);
    static void setRotateAction(RotateAction action);
    static void setRotateOpenLoop(double command);
    static void setFlywheelAction(FlywheelAction action);
    static void setFlywheelOpenLoop(double command);

    // Angle closed-loop control and homing
    static bool setAngleTargetCounts(long targetCounts);
    static void zeroAngleAtCurrentPosition();
    static void disableAngle();

    // Rotate position control with potentiometer feedback
    static bool setRotateProfiledTargetRaw(
        int targetRaw,
        double maximumCommand =
            ShooterConstants::Rotate::PROFILE_CRUISE_COMMAND);
    static bool setRotateTargetRaw(
        int targetRaw,
        double maximumCommand =
            ShooterConstants::Rotate::PROFILE_CRUISE_COMMAND);
    static void disableRotate();

    // Status
    static bool isAngleEncoderValid();
    static bool isAngleHomed();
    static bool isAngleReady();
    static bool areAngleSoftLimitsActive();
    static bool isAngleUpLimitTriggered();
    static bool isAngleDownLimitTriggered();
    static bool isRotateLeftLimitTriggered();
    static bool isRotateRightLimitTriggered();
    static bool isRotatePositionSensorValid();
    static bool isRotateReady();
    static bool areRotateSoftLimitsActive();
    static bool isReady();
    static AngleControlMode getAngleControlMode();
    static RotateControlMode getRotateControlMode();

    // Telemetry
    static uint16_t getAngleRawCounts();
    static long getAngleCounts();
    static long getAngleTargetCounts();
    static double getAngleDegrees();
    static int getAngleErrorCounts();
    static int getLeftAnglePulseUs();
    static int getRightAnglePulseUs();
    static int getRotatePulseUs();
    static int getFlywheelPulseUs();
    static double getAngleControllerOutput();
    static double getAngleVelocityCountsPerSecond();
    static double getAngleProportionalTerm();
    static double getAngleIntegralTerm();
    static double getAngleDerivativeTerm();
    static double getAngleFeedforwardTerm();
    static bool isAngleControllerSaturated();
    static unsigned long getAngleRejectedSampleCount();
    static int getRotatePositionRaw();
    static double getRotatePositionFilteredRaw();
    static double getRotateDegrees();
    static int getRotateTargetRaw();
    static double getRotateTargetDegrees();
    static int getRotateErrorRaw();
    static double getRotateMoveProgress();
    static double getRotateProfileEnvelope();
    static double getRotateControllerOutput();
    static double getRotateProportionalTerm();
    static double getRotateIntegralTerm();
    static double getRotateDerivativeTerm();
    static double getRotateFeedforwardTerm();
    static bool isRotateControllerSaturated();

private:
    struct DebouncedLimitSwitchState {
        bool rawTriggered;
        bool stableTriggered;
        unsigned long rawChangedMs;
    };

    static void updateAngle();
    static void updateRotate();
    static void initializeLimitSwitch(
        const DigitalLimitSwitchConfig& config,
        DebouncedLimitSwitchState& state);
    static void updateLimitSwitch(
        const DigitalLimitSwitchConfig& config,
        DebouncedLimitSwitchState& state,
        unsigned long nowMs);
    static bool readLimitSwitch(
        const DigitalLimitSwitchConfig& config);
    static void updateLimitSwitches();
    static void writeAngleManualCommand(double command);
    static void writeAngleOffset(int offsetUs);
    static void writeRotateCommand(double command);
    static void writeFlywheelCommand(double command);
    static void resetAngleController();
    static void resetRotateController();
    static double limitAngleCommand(double command);
    static double limitRotateCommand(double command);
    static bool configureRotateTarget(
        int targetRaw,
        RotateControlMode mode,
        double maximumCommand);
    static double rotateRawToDegrees(double raw);

    static Servo angleLeftServo;
    static Servo angleRightServo;
    static Servo rotateServo;
    static Servo flywheelOutput;
    static AS5600Encoder angleEncoder;
    static AnalogPositionSensor rotatePositionSensor;
    static AngleControlMode angleMode;
    static RotateControlMode rotateMode;
    static bool outputsEnabled;
    static bool angleHomed;
    static bool angleReady;
    static bool rotateReady;
    static double angleManualCommand;
    static double rotateManualCommand;
    static double flywheelOpenLoopCommand;
    static double angleSetpoint;
    static double rotateSetpointRaw;
    static double rotateInitialErrorMagnitudeRaw;
    static double rotateMaximumCommand;
    static PidfController angleController;
    static PidfController rotateController;
    static double angleControllerOutput;
    static double rotateControllerOutput;
    static double rotateProfileEnvelope;
    static unsigned long lastAngleControlMs;
    static unsigned long lastRotateControlMs;
    static unsigned long angleReadySinceMs;
    static unsigned long rotateReadySinceMs;
    static int leftAnglePulseUs;
    static int rightAnglePulseUs;
    static int rotatePulseUs;
    static int flywheelPulseUs;
    static DebouncedLimitSwitchState angleUpLimitState;
    static DebouncedLimitSwitchState angleDownLimitState;
    static DebouncedLimitSwitchState rotateLeftLimitState;
    static DebouncedLimitSwitchState rotateRightLimitState;
};
