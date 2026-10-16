#pragma once

// Minimal host-side stand-in for the Arduino core, just enough to compile
// the real src/IO/ManualControl.cpp, src/Auto/Auto.cpp,
// src/Dribbler/Dribbler.cpp and the gamepad motor bench on a PC (see
// test/README). Time, pin levels, and the USB Serial port are all fake and
// driven by the test.

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include <deque>
#include <string>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A5 59
#define F(text) (text)

template <typename T, typename Low, typename High>
T constrain(T value, Low low, High high) {
    if (value < low) return static_cast<T>(low);
    if (value > high) return static_cast<T>(high);
    return value;
}

namespace hoststub {
extern unsigned long nowMs;
extern int pinLevel[128];     // what digitalRead() returns
extern int digitalOut[128];   // last digitalWrite() per pin
extern int analogOut[128];    // last analogWrite() per pin
}

unsigned long millis();
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);

class String {
public:
    String() {}
    String(const char*) {}
};

class HostSerial {
public:
    void begin(unsigned long) {}
    int available() { return static_cast<int>(input.size()); }
    int read() {
        if (input.empty()) return -1;
        const char c = input.front();
        input.pop_front();
        return static_cast<unsigned char>(c);
    }
    void print(const char* text) { output += text; }
    void println(const char* text) { output += text; output += "\n"; }
    void print(char c) { output += c; }
    void print(int v) { output += std::to_string(v); }
    void print(long v) { output += std::to_string(v); }
    void print(unsigned long v) { output += std::to_string(v); }
    void println(int v) { print(v); output += "\n"; }
    void println(unsigned long v) { print(v); output += "\n"; }

    void send(const std::string& text) {
        for (char c : text) input.push_back(c);
    }

    std::deque<char> input;
    std::string output;
};

extern HostSerial Serial;
