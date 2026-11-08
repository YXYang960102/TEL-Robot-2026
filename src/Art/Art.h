#pragma once

#include <Servo.h>

// Creative design show (ArtConstants.h): on every shot the Mega sends (a
// new Dribbler trigger), the link motor turns for Link::RUN_MS and the
// 動感光波 clip plays. While enabled, each change of the aimed hole plays
// the laugh clip once, after the 動感光波 clip is over.
// The motor only runs while the tower is enabled (the robot is enabled) and
// stops at once on disable. While disabled, the 準備好了 clip repeats every
// Sound::READY_PERIOD_MS: the first READY_NORMAL_PLAYS are
// normal, every later one stutters (準準準準準備好了). It never affects
// whether or when a shot is fired.
class Art {
public:
    static void init();   // motor at stop, sound lines at their idle level
    static void update(); // after Shooter::update() and Dribbler::update()

    static bool isLinkRunning();
    static int getLinkPulseUs();

private:
    // One sound trigger line: idle, a pending start, or active for pulseMs.
    struct SoundLine {
        uint8_t pin;
        bool active;
        unsigned long startMs;
        bool pending;
        unsigned long dueMs;
        unsigned long pulseMs;
    };

    static void onShot(unsigned long now);
    static void triggerSound(SoundLine& line, unsigned long now, unsigned long delayMs,
                             unsigned long pulseMs);
    static void updateReady(unsigned long now);
    static void updateSound(SoundLine& line, unsigned long now);
    static void writeSound(const SoundLine& line, bool active);
    static uint8_t aimedHole();

    static Servo linkMotor;
    static int linkPulseUs;
    static bool linkRunning;
    static unsigned long linkStartMs;
    static unsigned long lastTriggerCount;
    static uint8_t lastAimedHole;
    static bool laughWanted;      // a hole change waiting for its laugh
    static bool waveEverPlayed;
    static unsigned long waveStartMs;
    static unsigned long readyLastStartMs;
    static SoundLine wave;
    static SoundLine laugh;
    static SoundLine ready;
    static uint8_t readyPlays;       // plays started in this disabled period
    static uint8_t readyStutterLeft; // triggers still to come in a stutter
};
