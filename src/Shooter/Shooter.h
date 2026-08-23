#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Constants/ShooterConstants.h"
#include "../Control/PidfController.h"
#include "../Sensors/AS5600Encoder.h"

enum class AngleControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
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

    // Status
    static bool isAngleEncoderValid();
    static bool isAngleHomed();
    static bool isAngleReady();
    static bool areAngleSoftLimitsActive();
    static bool isReady();

    // Telemetry
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

private:
    static void updateAngle();
    static void writeAngleManualCommand(double command);
    static void writeAngleOffset(int offsetUs);
    static void writeRotateCommand(double command);
    static void writeFlywheelCommand(double command);
    static void resetAngleController();
    static double limitAngleCommand(double command);

    static Servo angleLeftServo;
    static Servo angleRightServo;
    static Servo rotateServo;
    static Servo flywheelOutput;
    static AS5600Encoder angleEncoder;
    static AngleControlMode angleMode;
    static bool outputsEnabled;
    static bool angleHomed;
    static bool angleReady;
    static double angleManualCommand;
    static double rotateManualCommand;
    static double flywheelOpenLoopCommand;
    static double angleSetpoint;
    static PidfController angleController;
    static double angleControllerOutput;
    static unsigned long lastAngleControlMs;
    static unsigned long angleReadySinceMs;
    static int leftAnglePulseUs;
    static int rightAnglePulseUs;
    static int rotatePulseUs;
    static int flywheelPulseUs;
};
