#pragma once

#include <stdint.h>

class Servo {
public:
    uint8_t attach(int) { return 0; }
    uint8_t attach(int, int, int) { return 0; }
    void writeMicroseconds(int) {}
    bool attached() { return false; }
};
