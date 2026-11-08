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
uint8_t Art::lastAimedHole = OperatorTarget::NONE;
bool Art::laughWanted = false;
bool Art::waveEverPlayed = false;
unsigned long Art::waveStartMs = 0;
unsigned long Art::readyLastStartMs = 0;
Art::SoundLine Art::wave = {Sound::WAVE_PIN, false, 0, false, 0, Sound::PULSE_MS};
Art::SoundLine Art::laugh = {Sound::LAUGH_PIN, false, 0, false, 0, Sound::PULSE_MS};
Art::SoundLine Art::ready = {Sound::READY_PIN, false, 0, false, 0, Sound::PULSE_MS};
uint8_t Art::readyPlays = 0;
uint8_t Art::readyStutterLeft = 0;

void Art::init() {
    linkRunning = false;
    linkPulseUs = Link::NEUTRAL_US;
    wave.active = wave.pending = false;
    laugh.active = laugh.pending = false;
    ready.active = ready.pending = false;
    readyPlays = readyStutterLeft = 0;
    lastAimedHole = OperatorTarget::NONE;
    laughWanted = waveEverPlayed = false;
    lastTriggerCount = Dribbler::getTriggerCount();
    if (!BuildProfile::DRIVES_ART) {
        return;
    }
    // Sound lines at their idle level: preload, then drive.
    const uint8_t soundPins[] = {Sound::WAVE_PIN, Sound::LAUGH_PIN, Sound::READY_PIN};
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

    // 哇哈哈: once each time the aimed hole changes (the operator picks
    // another one, or full auto moves on), only while enabled. Locking on
    // and fine corrections at the same hole never play it. It waits until
    // the 動感光波 clip is over (WAVE_CLIP_MS): the module plays one clip at
    // a time, and a new trigger cuts the one playing.
    const uint8_t hole = aimedHole();
    if (hole != OperatorTarget::NONE && hole != lastAimedHole) {
        lastAimedHole = hole;
        laughWanted = Dribbler::isEnabled();
    }
    if (!Dribbler::isEnabled()) {
        laughWanted = false;
    }
    const bool wavePlaying = waveEverPlayed && now - waveStartMs < Sound::WAVE_CLIP_MS;
    if (laughWanted && !wavePlaying && !laugh.active && !laugh.pending) {
        laughWanted = false;
        triggerSound(laugh, now, Sound::REARM_MS, Sound::PULSE_MS);
    }

    updateReady(now);

    updateSound(wave, now);
    updateSound(laugh, now);
    updateSound(ready, now);

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
    triggerSound(wave, now, 0, Sound::PULSE_MS);
    waveEverPlayed = true;
    waveStartMs = now;
}

// 準備好了 while disabled, cut off at once on Enable. The first play starts
// at once, each later one READY_PERIOD_MS after the last one started. Plays
// 1..READY_NORMAL_PLAYS are normal (HIGH PULSE_MS); every later one
// stutters: READY_STUTTER_CUTS
// triggers of READY_STUTTER_HIT_MS, each REARM_MS after the last, so the
// module restarts the clip and drops the previous one (準 準 準 準), then a
// last full PULSE_MS one (準備好了).
void Art::updateReady(unsigned long now) {
    if (Dribbler::isEnabled()) {
        if (ready.active || ready.pending) {
            ready.active = ready.pending = false;
            writeSound(ready, false);
        }
        readyPlays = readyStutterLeft = 0; // the next disable starts over
        return;
    }
    if (ready.active || ready.pending) return;

    if (readyStutterLeft > 0) {
        // Next trigger of this stutter; the last one plays the whole clip.
        readyStutterLeft--;
        triggerSound(ready, now, Sound::REARM_MS,
                     readyStutterLeft > 0 ? Sound::READY_STUTTER_HIT_MS : Sound::PULSE_MS);
        return;
    }

    // A new play: at once if it's the first since disable, else one period
    // after the last one started.
    unsigned long delayMs = Sound::REARM_MS;
    if (readyPlays > 0) {
        const unsigned long sinceLast = now - readyLastStartMs;
        if (sinceLast < Sound::READY_PERIOD_MS) delayMs = Sound::READY_PERIOD_MS - sinceLast;
        if (delayMs < Sound::REARM_MS) delayMs = Sound::REARM_MS;
    }
    if (readyPlays < Sound::READY_NORMAL_PLAYS) {
        readyPlays++;
        triggerSound(ready, now, delayMs, Sound::PULSE_MS);
    } else {
        // Stutter: this short trigger, then READY_STUTTER_CUTS - 1 more
        // short ones and the full one.
        readyStutterLeft = Sound::READY_STUTTER_CUTS;
        triggerSound(ready, now, delayMs, Sound::READY_STUTTER_HIT_MS);
    }
    readyLastStartMs = ready.dueMs;
}

void Art::triggerSound(SoundLine& line, unsigned long now, unsigned long delayMs,
                       unsigned long pulseMs) {
    if (line.active) {
        // Still playing the last one: back to idle first, then a fresh edge.
        line.active = false;
        writeSound(line, false);
        if (delayMs < Sound::REARM_MS) delayMs = Sound::REARM_MS;
    }
    line.pending = true;
    line.dueMs = now + delayMs;
    line.pulseMs = pulseMs;
    updateSound(line, now);
}

void Art::updateSound(SoundLine& line, unsigned long now) {
    if (line.pending && static_cast<long>(now - line.dueMs) >= 0) {
        line.pending = false;
        line.active = true;
        line.startMs = now;
        writeSound(line, true);
    }
    if (line.active && now - line.startMs >= line.pulseMs) {
        line.active = false;
        writeSound(line, false);
    }
}

void Art::writeSound(const SoundLine& line, bool active) {
    if (BuildProfile::DRIVES_ART) digitalWrite(line.pin, soundLevel(active));
}

// The hole being aimed at (Dashboard numbering): the one Auto / the
// operator aims at, else the one the camera is locked on (the Orin's own
// pick); 0 = none, which never changes lastAimedHole.
uint8_t Art::aimedHole() {
    const uint8_t target = Auto::getAimTarget();
    if (target != OperatorTarget::NONE) return target;
    return Vision::isValid() ? OperatorTarget::holeForVisionId(Vision::getTargetId())
                             : OperatorTarget::NONE;
}

bool Art::isLinkRunning() {
    return linkRunning;
}

int Art::getLinkPulseUs() {
    return linkPulseUs;
}
