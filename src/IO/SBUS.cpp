#include "SBUS.h"

#include "../Constants/IOConstants.h"

#include <Arduino.h>

using namespace IOConstants;

bfs::SbusRx SBUS::receiver(&Serial2);
bfs::SbusData SBUS::data;
int SBUS::driveForwardPulseUs = Sbus::PULSE_NEUTRAL_US;
int SBUS::auxiliaryPulseUs = Sbus::PULSE_NEUTRAL_US;
int SBUS::mechanismPulseUs = Sbus::PULSE_NEUTRAL_US;
int SBUS::driveTurnPulseUs = Sbus::PULSE_NEUTRAL_US;
int SBUS::modePulseUs = Sbus::PULSE_NEUTRAL_US;
unsigned long SBUS::lastValidFrameMs = 0;
bool SBUS::hasValidFrame = false;

void SBUS::init() {
    receiver.Begin();
    setNeutral();
}

void SBUS::update() {
    if (receiver.Read()) {
        data = receiver.data();

        if (data.failsafe || data.lost_frame) {
            hasValidFrame = false;
            setNeutral();
            return;
        }

        driveForwardPulseUs = map(
            data.ch[Sbus::DRIVE_FORWARD_CHANNEL],
            Sbus::RAW_MIN,
            Sbus::RAW_MAX,
            Sbus::PULSE_MIN_US,
            Sbus::PULSE_MAX_US);
        auxiliaryPulseUs = map(
            data.ch[Sbus::AUXILIARY_CHANNEL],
            Sbus::RAW_MIN,
            Sbus::RAW_MAX,
            Sbus::AUXILIARY_MIN_US,
            Sbus::AUXILIARY_MAX_US);
        mechanismPulseUs = map(
            data.ch[Sbus::MECHANISM_CHANNEL],
            Sbus::RAW_MIN,
            Sbus::RAW_MAX,
            Sbus::PULSE_MIN_US,
            Sbus::PULSE_MAX_US);
        driveTurnPulseUs = map(
            data.ch[Sbus::DRIVE_TURN_CHANNEL],
            Sbus::RAW_MIN,
            Sbus::RAW_MAX,
            Sbus::PULSE_MAX_US,
            Sbus::PULSE_MIN_US);
        modePulseUs = map(
            data.ch[Sbus::MODE_CHANNEL],
            Sbus::RAW_MIN,
            Sbus::RAW_MAX,
            Sbus::PULSE_NEUTRAL_US,
            Sbus::PULSE_MAX_US);

        lastValidFrameMs = millis();
        hasValidFrame = true;
    }

    if (!isHealthy()) {
        setNeutral();
    }
}

bool SBUS::isHealthy() {
    return hasValidFrame &&
           millis() - lastValidFrameMs <= Sbus::FRAME_TIMEOUT_MS;
}

unsigned long SBUS::getFrameAgeMs() {
    return hasValidFrame ? millis() - lastValidFrameMs : 0xFFFFFFFFUL;
}

double SBUS::getDriveForward() {
    return isHealthy() ? normalizedPulse(driveForwardPulseUs) : 0.0;
}

double SBUS::getDriveTurn() {
    return isHealthy() ? normalizedPulse(driveTurnPulseUs) : 0.0;
}

int SBUS::getDriveForwardPulseUs() {
    return driveForwardPulseUs;
}

int SBUS::getAuxiliaryPulseUs() {
    return auxiliaryPulseUs;
}

int SBUS::getMechanismPulseUs() {
    return mechanismPulseUs;
}

int SBUS::getDriveTurnPulseUs() {
    return driveTurnPulseUs;
}

int SBUS::getModePulseUs() {
    return modePulseUs;
}

void SBUS::setNeutral() {
    driveForwardPulseUs = Sbus::PULSE_NEUTRAL_US;
    auxiliaryPulseUs = Sbus::PULSE_NEUTRAL_US;
    mechanismPulseUs = Sbus::PULSE_NEUTRAL_US;
    driveTurnPulseUs = Sbus::PULSE_NEUTRAL_US;
    modePulseUs = Sbus::PULSE_NEUTRAL_US;
}

double SBUS::normalizedPulse(int pulseUs) {
    const int constrainedPulse = constrain(
        pulseUs,
        Sbus::PULSE_MIN_US,
        Sbus::PULSE_MAX_US);
    return (constrainedPulse - Sbus::PULSE_NEUTRAL_US) /
           static_cast<double>(Sbus::PULSE_MAX_US - Sbus::PULSE_NEUTRAL_US);
}
