#pragma once
#include <Servo.h>

class Chassis {
public:
    static void init();
    static void update();
    static void setDriveCommand(double forward, double turn);
    static void stop();

private:
    static Servo fr, fl;
    static double forwardCommand;
    static double turnCommand;
};
