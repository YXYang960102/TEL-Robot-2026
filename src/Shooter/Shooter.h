#pragma once

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>

#include "../Sensors/AS5600Encoder.h"

enum class ElevationControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    CLOSED_LOOP
};

class Shooter {
public:
    static void init();
    static void update();
    static void stop();

    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();
    static void setElevationManual(double command);
    static bool setElevationTargetCounts(long targetCounts);
    static void zeroElevationAtCurrentPosition();
    static void disableElevation();
    static void setTurretManual(double command);
    static void setFlywheelOpenLoop(double command);

    static bool isEncoderValid();
    static bool isElevationHomed();
    static bool isElevationReady();
    static bool isReady();
    static long getElevationCounts();
    static long getElevationTargetCounts();
    static double getElevationDegrees();
    static int getElevationErrorCounts();
    static int getLeftElevationPulseUs();
    static int getRightElevationPulseUs();
    static int getTurretPulseUs();
    static int getFlywheelPulseUs();

private:
    static void updateElevation();
    static void writeElevationOffset(int offsetUs);
    static void writeTurretCommand(double command);
    static void writeFlywheelCommand(double command);
    static void resetElevationController();

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
    static double elevationInput;
    static double elevationOutput;
    static double elevationSetpoint;
    static PID elevationPid;
    static int leftElevationPulseUs;
    static int rightElevationPulseUs;
    static int turretPulseUs;
    static int flywheelPulseUs;
};
