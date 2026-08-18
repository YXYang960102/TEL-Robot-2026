#include "ContinuousServoMotor.h"

void ContinuousServoMotor::begin(const ContinuousServoConfig& newConfig) {
    config = newConfig;
    servo.attach(config.signalPin);
    stop();
}

void ContinuousServoMotor::stop() {
    writeMicroseconds(config.stopUs);
}

void ContinuousServoMotor::runClockwise() {
    writeMicroseconds(config.clockwiseUs);
}

void ContinuousServoMotor::runCounterClockwise() {
    writeMicroseconds(config.counterClockwiseUs);
}

void ContinuousServoMotor::writeMicroseconds(int pulseUs) {
    lastPulseUs = constrain(pulseUs, config.minUs, config.maxUs);
    servo.writeMicroseconds(lastPulseUs);
}

int ContinuousServoMotor::getLastPulseUs() const {
    return lastPulseUs;
}
