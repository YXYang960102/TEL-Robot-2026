#ifndef SHOOTER_SHOOTER_H
#define SHOOTER_SHOOTER_H

#include <Servo.h>

class Shooter {
public:
    static void init();
    static void update();

    static bool isReady();

private:
    static Servo turretServo;
    static Servo escV;
    static Servo falcon;

    static bool readyH;
    static bool readyV;
};

#endif
