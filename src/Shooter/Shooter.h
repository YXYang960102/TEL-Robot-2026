#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "../Constants/ShooterConstants.h"

enum class AngleControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP
};

enum class RotateControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    VISION_FOLLOW
};

class Shooter {
public:
    static void init();
    static void update();
    static void stopAll();
    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();

    static void setAngleOpenLoop(double command);
    static bool setAngleTargetDegrees(double degrees);
    static bool setAnglePulseUs(double pulseUs);
    static double getAngleTargetDegrees();
    static double getAngleMinDegrees();
    static double getAngleMaxDegrees();
    static void setRotateOpenLoop(double command);
    static void setFlywheelOpenLoop(double command);
    static void disableRotate();

    static void setRotateVisionFollow();
    static void setRotateAimOffsetPx(double offsetPx);
    static void setRotateSweepAllowed(bool allowed);

    static bool isRotateReady();
    static bool isFlywheelReady();
    static bool isReady();
    static AngleControlMode getAngleControlMode();
    static RotateControlMode getRotateControlMode();

    static void requestTurretReturn();
    static uint8_t getTurretState();
    static int8_t getSearchDirection();

    static int getAnglePulseUs();
    static int getRotatePulseUs();
    static int getFlywheelPulseUs();
    static double getFlywheelCommand();

private:
    static void updateAngle();
    static void updateRotate();
    static void updateFlywheelReady();
    static void writeAngleCommand(double command);
    static void releaseAngle();
    static void writeRotateCommand(double command);
    static void driveRotate(double command);
    static void writeFlywheelCommand(double command);

    static Servo angleServo;
    static Servo rotateServo;
    static Servo flywheelOutput;
    static AngleControlMode angleMode;
    static RotateControlMode rotateMode;
    static bool outputsEnabled;
    static bool rotateReady;
    static double rotateAimOffsetPx;
    static double angleManualCommand;
    static double rotateManualCommand;
    static double flywheelOpenLoopCommand;
    static int anglePulseUs;
    static int rotatePulseUs;
    static int flywheelPulseUs;
    static unsigned long flywheelSpinStartMs;
    static double flywheelSpinCommand;
    static double flywheelOutputCommand;
    static unsigned long flywheelRampLastMs;
    static void rampFlywheel();
    static bool flywheelReady;
};
