#pragma once

#include <stdint.h>

class Servo {
public:
    uint8_t attach(int pin) { active = true; lastPin = pin; return 0; }
    uint8_t attach(int pin, int, int) { return attach(pin); }
    void writeMicroseconds(int pulse) { lastPulse = pulse; }
    bool attached() { return active; }
    inline static int lastPin = -1;
    inline static int lastPulse = -1;
private:
    bool active = false;
};
