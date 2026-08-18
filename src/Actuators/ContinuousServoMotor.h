#pragma once

#include <Arduino.h>
#include "../Constants/As5600ServoTestConstants.h"
#include <Servo.h>

struct ContinuousServoConfig {
    uint8_t signalPin = As5600ServoTestConst::Servo::SIGNAL_PIN;
    int stopUs = As5600ServoTestConst::Servo::STOP_US;
    int clockwiseUs = As5600ServoTestConst::Servo::CLOCKWISE_US;
    int counterClockwiseUs = As5600ServoTestConst::Servo::COUNTER_CLOCKWISE_US;
    int minUs = As5600ServoTestConst::Servo::MIN_US;
    int maxUs = As5600ServoTestConst::Servo::MAX_US;
};

class ContinuousServoMotor {
public:
    void begin(const ContinuousServoConfig& config);
    void stop();
    void runClockwise();
    void runCounterClockwise();
    void writeMicroseconds(int pulseUs);
    int getLastPulseUs() const;

private:
    ContinuousServoConfig config;
    Servo servo;
    int lastPulseUs = As5600ServoTestConst::Servo::STOP_US;
};
