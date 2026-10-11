#pragma once

#include <Arduino.h>

#include <string>

namespace hoststub {

// What the fake subsystems were last told to do.
struct Recorder {
    bool chassisEnabled = false;
    double chassisLeft = 0.0;
    double chassisRight = 0.0;
    bool shooterEnabled = false;
    std::string rotateMode = "disabled";
    double rotateCommand = 0.0;
    double angleCommand = 0.0;
    double flywheelCommand = 0.0;
    unsigned long statusCount = 0;
    uint8_t statusFlags = 0;
    unsigned long statusSeq = 0;
    uint16_t statusCode = 0;
};

extern Recorder rec;
extern bool visionReady;

}
