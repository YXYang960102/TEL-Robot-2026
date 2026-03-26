#ifndef CHASSIS_H
#define CHASSIS_H

#include <Servo.h>

class Chassis {
public:
    void begin(int fr_pin, int br_pin, int fl_pin, int bl_pin);
    void drive(int x, int y, int rotate);
    void stop();

private:
    Servo motor_fr, motor_br, motor_fl, motor_bl;
};

#endif