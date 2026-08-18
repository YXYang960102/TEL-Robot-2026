#pragma once

#include <Arduino.h>
#include "AS5600Encoder.h"
#include "ContinuousServoMotor.h"
#include "../Constants/As5600ServoTestConstants.h"

enum class As5600ServoTestMode {
    MANUAL_OPEN_LOOP,
    PID_CLOSED_LOOP
};

struct As5600ServoTestConfig {
    ContinuousServoConfig servo;
    long targetRelativeCounts = As5600ServoTestConst::Control::TARGET_RELATIVE_COUNTS;
    int stopToleranceCounts = As5600ServoTestConst::Control::STOP_TOLERANCE_COUNTS;
    int manualOffsetUs = As5600ServoTestConst::Servo::MANUAL_OFFSET_US;
    double kp = As5600ServoTestConst::Control::KP;
    double ki = As5600ServoTestConst::Control::KI;
    double kd = As5600ServoTestConst::Control::KD;
    int maxOutputOffsetUs = As5600ServoTestConst::Control::MAX_OUTPUT_OFFSET_US;
    int minMovingOffsetUs = As5600ServoTestConst::Control::MIN_MOVING_OFFSET_US;
    int maxEncoderDeltaCounts = As5600ServoTestConst::Encoder::MAX_ACCEPTED_DELTA_COUNTS;
    double integralLimit = As5600ServoTestConst::Control::INTEGRAL_LIMIT;
    int outputSign = As5600ServoTestConst::Control::OUTPUT_SIGN;
    unsigned long printIntervalMs = As5600ServoTestConst::SerialLog::PRINT_INTERVAL_MS;
};

class As5600ServoTestSubsystem {
public:
    void begin(const As5600ServoTestConfig& config);
    void update();
    bool isStoppedAtTarget() const;

private:
    void handleSerialCommands();
    void setManualStop();
    void setManualForward();
    void setManualReverse();
    void setMode(As5600ServoTestMode newMode);
    int computePidOutput(long error);
    void resetController();
    void printStatus(long error, bool magnetDetected);
    void printHelp();

    As5600ServoTestConfig config;
    AS5600Encoder encoder;
    ContinuousServoMotor servo;
    As5600ServoTestMode mode = As5600ServoTestMode::MANUAL_OPEN_LOOP;
    bool stoppedAtTarget = true;
    bool hasLastError = false;
    long lastError = 0;
    double errorIntegral = 0.0;
    int lastPidOutputUs = 0;
    int manualOffsetUs = As5600ServoTestConst::Servo::MANUAL_OFFSET_US;
    unsigned long lastUpdateMs = 0;
    unsigned long lastPrintMs = 0;
};
