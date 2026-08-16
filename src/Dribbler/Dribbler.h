#pragma once

#include <Arduino.h>

class Dribbler {
public:
    static void init();
    static void update();

    static void setShootRequest(int count);
    static int getShootRemaining();

private:
    static int shoot_pice;

    static void run();
    static void stop();

    static bool sensorUp;
};
