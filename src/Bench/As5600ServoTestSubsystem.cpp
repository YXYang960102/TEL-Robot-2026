#include "As5600ServoTestSubsystem.h"

void As5600ServoTestSubsystem::begin(const As5600ServoTestConfig& newConfig) {
    config = newConfig;
    manualOffsetUs = config.manualOffsetUs;

    encoder.begin();
    encoder.setMaxAcceptedDeltaCounts(config.maxEncoderDeltaCounts);
    servo.begin(config.servo);
    servo.stop();
    delay(1500);

    Serial.println("AS5600 + continuous servo UNO subsystem test");
    Serial.println("AS5600: VCC=5V, GND=GND, SDA=A4, SCL=A5");
    Serial.println("Servo signal: D9. Servo power must be external. Share GND with UNO.");
    Serial.print("Boot zero raw count: ");
    Serial.println(encoder.getZeroRaw());
    Serial.println("Relative multi-turn mode: boot position is 0, counts keep accumulating across 4095/0.");
    Serial.println("PID target relative count: 1024. Stop tolerance: +/-1 count.");
    Serial.println("Large encoder jumps are rejected before they affect relative position.");
    printHelp();
}

void As5600ServoTestSubsystem::update() {
    handleSerialCommands();

    const bool magnetDetected = encoder.update();
    const long relativeCounts = encoder.getRelativeCounts();
    const long error = config.targetRelativeCounts - relativeCounts;

    if (mode == As5600ServoTestMode::MANUAL_OPEN_LOOP) {
        stoppedAtTarget = servo.getLastPulseUs() == config.servo.stopUs;
        resetController();
        printStatus(error, magnetDetected);
        return;
    }

    if (!magnetDetected) {
        stoppedAtTarget = true;
        resetController();
        servo.stop();
        printStatus(error, false);
        return;
    }

    if (labs(error) <= config.stopToleranceCounts) {
        stoppedAtTarget = true;
        resetController();
        servo.stop();
    } else {
        stoppedAtTarget = false;
        const int pidOutputUs = computePidOutput(error);
        servo.writeMicroseconds(config.servo.stopUs + pidOutputUs);
    }

    printStatus(error, true);
}

bool As5600ServoTestSubsystem::isStoppedAtTarget() const {
    return stoppedAtTarget;
}

void As5600ServoTestSubsystem::handleSerialCommands() {
    while (Serial.available() > 0) {
        const char command = Serial.read();

        switch (command) {
            case 'm':
            case 'M':
                setMode(As5600ServoTestMode::MANUAL_OPEN_LOOP);
                setManualStop();
                break;
            case 'p':
            case 'P':
                setMode(As5600ServoTestMode::PID_CLOSED_LOOP);
                break;
            case 's':
            case 'S':
                setMode(As5600ServoTestMode::MANUAL_OPEN_LOOP);
                setManualStop();
                break;
            case 'f':
            case 'F':
                setMode(As5600ServoTestMode::MANUAL_OPEN_LOOP);
                setManualForward();
                break;
            case 'r':
            case 'R':
                setMode(As5600ServoTestMode::MANUAL_OPEN_LOOP);
                setManualReverse();
                break;
            case '+':
                manualOffsetUs = constrain(manualOffsetUs + 10, 0, config.maxOutputOffsetUs);
                break;
            case '-':
                manualOffsetUs = constrain(manualOffsetUs - 10, 0, config.maxOutputOffsetUs);
                break;
            case 'z':
            case 'Z':
                encoder.zero();
                resetController();
                break;
            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;
            default:
                break;
        }
    }
}

void As5600ServoTestSubsystem::setManualStop() {
    servo.stop();
    stoppedAtTarget = true;
}

void As5600ServoTestSubsystem::setManualForward() {
    servo.writeMicroseconds(config.servo.stopUs + manualOffsetUs);
    stoppedAtTarget = false;
}

void As5600ServoTestSubsystem::setManualReverse() {
    servo.writeMicroseconds(config.servo.stopUs - manualOffsetUs);
    stoppedAtTarget = false;
}

void As5600ServoTestSubsystem::setMode(As5600ServoTestMode newMode) {
    if (mode == newMode) {
        return;
    }

    mode = newMode;
    resetController();
}

int As5600ServoTestSubsystem::computePidOutput(long error) {
    const unsigned long now = millis();
    double dt = 0.02;

    if (lastUpdateMs != 0 && now > lastUpdateMs) {
        dt = (now - lastUpdateMs) / 1000.0;
    }
    lastUpdateMs = now;

    errorIntegral += error * dt;
    errorIntegral = constrain(errorIntegral, -config.integralLimit, config.integralLimit);

    double derivative = 0.0;
    if (hasLastError && dt > 0.0) {
        derivative = (error - lastError) / dt;
    }

    hasLastError = true;
    lastError = error;

    double output = config.kp * error + config.ki * errorIntegral + config.kd * derivative;
    output *= config.outputSign;
    output = constrain(output, -config.maxOutputOffsetUs, config.maxOutputOffsetUs);

    if (output > 0.0 && output < config.minMovingOffsetUs) {
        output = config.minMovingOffsetUs;
    } else if (output < 0.0 && output > -config.minMovingOffsetUs) {
        output = -config.minMovingOffsetUs;
    }

    lastPidOutputUs = static_cast<int>(output);
    return lastPidOutputUs;
}

void As5600ServoTestSubsystem::resetController() {
    hasLastError = false;
    lastError = 0;
    errorIntegral = 0.0;
    lastPidOutputUs = 0;
    lastUpdateMs = 0;
}

void As5600ServoTestSubsystem::printStatus(long error, bool magnetDetected) {
    const unsigned long now = millis();
    if (now - lastPrintMs < config.printIntervalMs) {
        return;
    }

    lastPrintMs = now;

    Serial.print("mode=");
    Serial.print(mode == As5600ServoTestMode::MANUAL_OPEN_LOOP ? "manual" : "pid");
    Serial.print(", ");
    Serial.print("raw=");
    Serial.print(encoder.getRaw());
    Serial.print(", zero_raw=");
    Serial.print(encoder.getZeroRaw());
    Serial.print(", delta=");
    Serial.print(encoder.getLastDeltaCounts());
    Serial.print(", rejected_delta=");
    Serial.print(encoder.getLastRejectedDeltaCounts());
    Serial.print(", rejected_count=");
    Serial.print(encoder.getRejectedSampleCount());
    Serial.print(", relative=");
    Serial.print(encoder.getRelativeCounts());
    Serial.print(", target_relative=");
    Serial.print(config.targetRelativeCounts);
    Serial.print(", error=");
    Serial.print(error);
    Serial.print(", abs_error=");
    Serial.print(labs(error));
    Serial.print(", error_deg=");
    Serial.print(labs(error) * As5600ServoTestConst::Encoder::COUNTS_TO_DEGREES, 3);
    Serial.print(", pid_us=");
    Serial.print(lastPidOutputUs);
    Serial.print(", pulse_us=");
    Serial.print(servo.getLastPulseUs());
    Serial.print(", manual_offset_us=");
    Serial.print(manualOffsetUs);
    Serial.print(", stopped=");
    Serial.print(stoppedAtTarget ? 1 : 0);
    Serial.print(", magnet=");
    Serial.println(magnetDetected ? 1 : 0);
}

void As5600ServoTestSubsystem::printHelp() {
    Serial.println("Commands: m=manual, p=PID, s=stop, f=forward, r=reverse, +=more, -=less, z=zero, h=help");
    Serial.println("Start in manual open-loop mode. Use PID only after motor motion changes encoder readings.");
}
