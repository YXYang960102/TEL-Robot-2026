#include "Art.h"

#include <Arduino.h>

#include "../Auto/Auto.h"
#include "../Constants/ArtConstants.h"
#include "../Constants/BuildProfile.h"
#include "../Dribbler/Dribbler.h"
#include "../Vision/OperatorTarget.h"
#include "../Vision/Vision.h"

using namespace ArtConstants;

Servo Art::linkMotor;
int Art::linkPulseUs = Link::NEUTRAL_US;
bool Art::linkRunning = false;
unsigned long Art::linkStartMs = 0;
unsigned long Art::lastTriggerCount = 0;
uint8_t Art::lastShotHole = 0;
bool Art::waveActive = false;
unsigned long Art::waveStartMs = 0;
bool Art::laughActive = false;
unsigned long Art::laughStartMs = 0;
bool Art::laughPending = false;
unsigned long Art::laughDueMs = 0;

void Art::init() {
    linkRunning = false;
    linkPulseUs = Link::NEUTRAL_US;
    waveActive = false;
    laughActive = false;
    laughPending = false;
    lastShotHole = 0;
    lastTriggerCount = Dribbler::getTriggerCount();
    if (!BuildProfile::DRIVES_ART) {
        return;
    }
    // Sound lines idle HIGH: preload, then drive.
    const uint8_t soundPins[] = {Sound::WAVE_PIN, Sound::LAUGH_PIN, Sound::SPARE_PIN};
    for (uint8_t pin : soundPins) {
        digitalWrite(pin, HIGH);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
    linkMotor.attach(Link::SIGNAL_PIN, Link::MIN_PULSE_US, Link::MAX_PULSE_US);
    linkMotor.writeMicroseconds(Link::NEUTRAL_US); // the ESC arms on a steady stop
}

void Art::update() {
    const unsigned long now = millis();
    const unsigned long shots = Dribbler::getTriggerCount();
    if (shots != lastTriggerCount) {
        lastTriggerCount = shots;
        onShot(now);
    }

    // Link motor: one direction for RUN_MS, only while enabled.
    if (linkRunning && (!Dribbler::isEnabled() || now - linkStartMs >= Link::RUN_MS)) {
        linkRunning = false;
    }
    if (linkRunning) {
        const int sign = Link::INVERTED ? -1 : 1;
        linkPulseUs = Link::NEUTRAL_US +
                      sign * static_cast<int>(Link::COMMAND * (Link::MAX_PULSE_US - Link::NEUTRAL_US) + 0.5);
    } else {
        linkPulseUs = Link::NEUTRAL_US;
    }

    if (laughPending && static_cast<long>(now - laughDueMs) >= 0) {
        laughPending = false;
        startSound(Sound::LAUGH_PIN, now, laughStartMs, laughActive);
    }
    // Release each sound line after PULSE_MS.
    if (waveActive && now - waveStartMs >= Sound::PULSE_MS) {
        waveActive = false;
        if (BuildProfile::DRIVES_ART) digitalWrite(Sound::WAVE_PIN, HIGH);
    }
    if (laughActive && now - laughStartMs >= Sound::PULSE_MS) {
        laughActive = false;
        if (BuildProfile::DRIVES_ART) digitalWrite(Sound::LAUGH_PIN, HIGH);
    }

    if (BuildProfile::DRIVES_ART) {
        linkMotor.writeMicroseconds(linkPulseUs);
    }
}

void Art::onShot(unsigned long now) {
    // A trigger is only ever sent while enabled, but check anyway: the
    // motor must never start on a disabled robot.
    if (Dribbler::isEnabled()) {
        linkRunning = true;
        linkStartMs = now;
    }
    startSound(Sound::WAVE_PIN, now, waveStartMs, waveActive);
    const uint8_t hole = shotHole();
    if (hole != OperatorTarget::NONE && hole != lastShotHole) {
        lastShotHole = hole;
        laughPending = true;
        laughDueMs = now + Sound::LAUGH_DELAY_MS;
    }
}

void Art::startSound(uint8_t pin, unsigned long now, unsigned long& startMs, bool& active) {
    active = true;
    startMs = now;
    if (BuildProfile::DRIVES_ART) digitalWrite(pin, LOW);
}

// The hole this shot went to (Dashboard numbering): the one the camera is
// locked on, else the one Auto / the operator aims at; 0 = unknown (full
// manual without a target in view), which never plays the laugh.
uint8_t Art::shotHole() {
    if (Vision::isValid()) {
        const uint8_t hole = OperatorTarget::holeForVisionId(Vision::getTargetId());
        if (hole != OperatorTarget::NONE) return hole;
    }
    return Auto::getAimTarget();
}

bool Art::isLinkRunning() {
    return linkRunning;
}

int Art::getLinkPulseUs() {
    return linkPulseUs;
}
