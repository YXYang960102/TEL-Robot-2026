#pragma once

#include <Servo.h>

class Shooter {
public:
    static void init();
    static void update();

    static bool isReady();

private:
    static Servo escH;
    static Servo escV;
    static Servo falcon;

    static bool readyH;
    static bool readyV;
};

