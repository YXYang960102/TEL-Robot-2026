#pragma once

#include <Servo.h>

// Creative design show (ArtConstants.h): on every shot the Mega sends (a
// new Dribbler trigger), the link motor turns for Link::RUN_MS and the
// 動感光波 clip plays; the first shot at a hole different from the last
// one also plays the laugh clip after Sound::LAUGH_DELAY_MS. The motor
// only runs while the tower is enabled (the robot is enabled) and stops at
// once on disable. It never affects whether or when a shot is fired.
class Art {
public:
    static void init();   // motor at stop, sound lines at their idle level
    static void update(); // after Dribbler::update()

    static bool isLinkRunning();
    static int getLinkPulseUs();

private:
    // One sound trigger line: idle, a pending start, or active for PULSE_MS.
    struct SoundLine {
        uint8_t pin;
        bool active;
        unsigned long startMs;
        bool pending;
        unsigned long dueMs;
    };

    static void onShot(unsigned long now);
    static void triggerSound(SoundLine& line, unsigned long now, unsigned long delayMs);
    static void updateSound(SoundLine& line, unsigned long now);
    static void writeSound(const SoundLine& line, bool active);
    static uint8_t shotHole();

    static Servo linkMotor;
    static int linkPulseUs;
    static bool linkRunning;
    static unsigned long linkStartMs;
    static unsigned long lastTriggerCount;
    static uint8_t lastShotHole;
    static SoundLine wave;
    static SoundLine laugh;
};
