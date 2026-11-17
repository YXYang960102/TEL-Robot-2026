#pragma once

#include <Servo.h>

class Chassis {
public:
    static void init();
    static void update();
    static void setOutputsEnabled(bool enabled);
    static bool areOutputsEnabled();

    static void setOpenLoop(double forward, double turn);
    static void setTankOpenLoop(double left, double right);
    static void stop();

    static double getForwardCommand();
    static double getTurnCommand();
    static double getLeftCommand();
    static double getRightCommand();
    static int getRightPulseUs();
    static int getLeftPulseUs();

private:
    static double applyDeadband(double command);
    static int commandToPulseUs(double command, bool inverted);
    static double filterTowards(double target, double& state, double dtMs);

    static Servo rightDrive;
    static Servo leftDrive;
    static double forwardCommand;
    static double turnCommand;
    static double leftCommand;
    static double rightCommand;
    static bool outputsEnabled;
    static int rightPulseUs;
    static int leftPulseUs;

    static double filteredLeftCommand;
    static double filteredRightCommand;
    static unsigned long lastUpdateMs;
    static bool hasLastUpdateMs;
};
