#ifndef CHASSIS_H
#define CHASSIS_H

#include <Servo.h>

class Chassis {
public:
    static void init();
    static void update();

private:
    static Servo fr, fl, br, bl;
};
#endif