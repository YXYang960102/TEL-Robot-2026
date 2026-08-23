#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Control/PidfController.h"
#include "../Sensors/AS5600Encoder.h"

enum class ElevationControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    CLOSED_LOOP
};

enum class OpenLoopAction : int8_t {
    REVERSE = -1,
    STOP = 0,
    FORWARD = 1
};

enum class FlywheelAction : uint8_t {
    STOP = 0,
    FORWARD = 1
};

class Shooter {
public:
    static void init();
    static void update();
    static void stop();

    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();
    static void setElevationManual(double command);
    static void setAngleSpeed(OpenLoopAction action);
    static bool setElevationTargetCounts(long targetCounts);
    static void zeroElevationAtCurrentPosition();
    static void disableElevation();
    static void setTurretManual(double command);
    static void setRotateSpeed(OpenLoopAction action);
    static void setFlywheelOpenLoop(double command);
    static void setFlywheelSpeed(FlywheelAction action);

    static bool isEncoderValid();
    static bool isElevationHomed();
    static bool isElevationReady();
    static bool areElevationSoftLimitsActive();
    static bool isReady();
    static long getElevationCounts();
    static long getElevationTargetCounts();
    static double getElevationDegrees();
    static int getElevationErrorCounts();
    static int getLeftElevationPulseUs();
    static int getRightElevationPulseUs();
    static int getTurretPulseUs();
    static int getFlywheelPulseUs();
    static double getAngleControllerOutput();
    static double getAngleVelocityCountsPerSecond();
    static double getAngleProportionalTerm();
    static double getAngleIntegralTerm();
    static double getAngleDerivativeTerm();
    static double getAngleFeedforwardTerm();
    static bool isAngleControllerSaturated();

private:
    static void updateElevation();
    static void writeElevationManualCommand(double command);
    static void writeElevationOffset(int offsetUs);
    static void writeTurretCommand(double command);
    static void writeFlywheelCommand(double command);
    static void resetElevationController();
    static double actionToCommand(OpenLoopAction action);
    static double limitElevationCommand(double command);

    static Servo elevationLeftServo;
    static Servo elevationRightServo;
    static Servo turretServo;
    static Servo flywheelOutput;
    static AS5600Encoder elevationEncoder;
    static ElevationControlMode elevationMode;
    static bool outputsEnabled;
    static bool elevationHomed;
    static bool elevationReady;
    static double elevationManualCommand;
    static double turretManualCommand;
    static double flywheelOpenLoopCommand;
    static double elevationSetpoint;
    static PidfController angleController;
    static double angleControllerOutput;
    static unsigned long lastAngleControlMs;
    static unsigned long angleReadySinceMs;
    static int leftElevationPulseUs;
    static int rightElevationPulseUs;
    static int turretPulseUs;
    static int flywheelPulseUs;
};
