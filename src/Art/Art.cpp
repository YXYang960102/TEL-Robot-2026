#include "Art.h"

#include <Arduino.h>

#include "../Auto/Auto.h"
#include "../Constants/ArtConstants.h"
#include "../Constants/BuildProfile.h"
#include "../Dribbler/Dribbler.h"
#include "../Vision/OperatorTarget.h"
#include "../Vision/Vision.h"

using namespace ArtConstants;

namespace {
uint8_t soundLevel(bool active) {
    return active == Sound::ACTIVE_HIGH ? HIGH : LOW;
}
}

Servo Art::linkMotor;
int Art::linkPulseUs = Link::NEUTRAL_US;
bool Art::linkRunning = false;
unsigned long Art::linkStartMs = 0;
unsigned long Art::lastTriggerCount = 0;
uint8_t Art::lastShotHole = 0;
Art::SoundLine Art::wave = {Sound::WAVE_PIN, false, 0, false, 0};
Art::SoundLine Art::laugh = {Sound::LAUGH_PIN, false, 0, false, 0};

void Art::init() {
    linkRunning = false;
    linkPulseUs = Link::NEUTRAL_US;
    wave.active = wave.pending = false;
    laugh.active = laugh.pending = false;
    lastShotHole = 0;
    lastTriggerCount = Dribbler::getTriggerCount();
    if (!BuildProfile::DRIVES_ART) {
        return;
    }
    // Sound lines at their idle level: preload, then drive.
    const uint8_t soundPins[] = {Sound::WAVE_PIN, Sound::LAUGH_PIN, Sound::SPARE_PIN};
    for (uint8_t pin : soundPins) {
        digitalWrite(pin, soundLevel(false));
        pinMode(pin, OUTPUT);
        digitalWrite(pin, soundLevel(false));
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

    updateSound(wave, now);
    updateSound(laugh, now);

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
    triggerSound(wave, now, 0);
    const uint8_t hole = shotHole();
    if (hole != OperatorTarget::NONE && hole != lastShotHole) {
        lastShotHole = hole;
        triggerSound(laugh, now, Sound::LAUGH_DELAY_MS);
    }
}

void Art::triggerSound(SoundLine& line, unsigned long now, unsigned long delayMs) {
    if (line.active) {
        // Still playing the last one: back to idle first, then a fresh edge.
        line.active = false;
        writeSound(line, false);
        if (delayMs < Sound::REARM_MS) delayMs = Sound::REARM_MS;
    }
    line.pending = true;
    line.dueMs = now + delayMs;
    updateSound(line, now);
}

void Art::updateSound(SoundLine& line, unsigned long now) {
    if (line.pending && static_cast<long>(now - line.dueMs) >= 0) {
        line.pending = false;
        line.active = true;
        line.startMs = now;
        writeSound(line, true);
    }
    if (line.active && now - line.startMs >= Sound::PULSE_MS) {
        line.active = false;
        writeSound(line, false);
    }
}

void Art::writeSound(const SoundLine& line, bool active) {
    if (BuildProfile::DRIVES_ART) digitalWrite(line.pin, soundLevel(active));
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
