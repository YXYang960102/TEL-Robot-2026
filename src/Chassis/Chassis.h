#pragma once

#include <Servo.h>

class Chassis {
public:
    // Lifecycle
    static void init();
    static void update();

    // Manual open-loop control
    static void setOpenLoop(double forward, double turn);
    static void stop();

    // Telemetry
    static double getForwardCommand();
    static double getTurnCommand();
    static int getRightPulseUs();
    static int getLeftPulseUs();

private:
    static double applyDeadband(double command);
    static int commandToPulseUs(double command, bool inverted);

    static Servo rightDrive;
    static Servo leftDrive;
    static double forwardCommand;
    static double turnCommand;
    static int rightPulseUs;
    static int leftPulseUs;
};
