#include "SBUS.h"
#include "../Constants/SBUSConstants.h"

#include <Arduino.h>

bfs::SbusRx SBUS::sbus(&Serial2);
bfs::SbusData SBUS::data;

int SBUS::ch0 = 1500;
int SBUS::ch1 = 1500;
int SBUS::ch2 = 1500;
int SBUS::ch3 = 1500;
int SBUS::ch8 = 1500;
unsigned long SBUS::lastValidFrameMs = 0;
bool SBUS::hasValidFrame = false;

void SBUS::init() {
    sbus.Begin();
    setNeutral();
}

void SBUS::update() {
    if (sbus.Read()) {
        data = sbus.data();

        if (data.failsafe || data.lost_frame) {
            hasValidFrame = false;
            setNeutral();
            return;
        }

        ch0 = map(data.ch[0], SBUSConst::RAW_MIN, SBUSConst::RAW_MAX,
                  SBUSConst::PULSE_MIN_US, SBUSConst::PULSE_MAX_US);
        ch1 = map(data.ch[3], SBUSConst::RAW_MIN, SBUSConst::RAW_MAX, 1200, 1800);
        ch2 = map(data.ch[2], SBUSConst::RAW_MIN, SBUSConst::RAW_MAX,
                  SBUSConst::PULSE_MIN_US, SBUSConst::PULSE_MAX_US);
        ch3 = map(data.ch[1], SBUSConst::RAW_MIN, SBUSConst::RAW_MAX,
                  SBUSConst::PULSE_MAX_US, SBUSConst::PULSE_MIN_US);
        ch8 = map(data.ch[8], SBUSConst::RAW_MIN, SBUSConst::RAW_MAX,
                  SBUSConst::PULSE_NEUTRAL_US, SBUSConst::PULSE_MAX_US);

        lastValidFrameMs = millis();
        hasValidFrame = true;
    }

    if (!isHealthy()) {
        setNeutral();
    }
}

bool SBUS::isHealthy() {
    return hasValidFrame && millis() - lastValidFrameMs <= SBUSConst::FRAME_TIMEOUT_MS;
}

unsigned long SBUS::getFrameAgeMs() {
    return hasValidFrame ? millis() - lastValidFrameMs : 0xFFFFFFFFUL;
}

double SBUS::getDriveForward() {
    return isHealthy() ? normalizedPulse(ch0) : 0.0;
}

double SBUS::getDriveTurn() {
    return isHealthy() ? normalizedPulse(ch3) : 0.0;
}

void SBUS::setNeutral() {
    ch0 = SBUSConst::PULSE_NEUTRAL_US;
    ch1 = SBUSConst::PULSE_NEUTRAL_US;
    ch2 = SBUSConst::PULSE_NEUTRAL_US;
    ch3 = SBUSConst::PULSE_NEUTRAL_US;
    ch8 = SBUSConst::PULSE_NEUTRAL_US;
}

double SBUS::normalizedPulse(int pulseUs) {
    const int constrainedPulse = constrain(
        pulseUs, SBUSConst::PULSE_MIN_US, SBUSConst::PULSE_MAX_US);
    return (constrainedPulse - SBUSConst::PULSE_NEUTRAL_US) /
           static_cast<double>(SBUSConst::PULSE_MAX_US - SBUSConst::PULSE_NEUTRAL_US);
}
