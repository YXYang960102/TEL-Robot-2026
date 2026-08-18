#pragma once
#include <Servo.h>

class Chassis {
public:
    static void init();
    static void update();

private:
    static Servo fr, fl;
};
