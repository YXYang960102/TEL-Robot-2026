#pragma once

#include <Servo.h>

class Art {
public:
    static void init();
    static void update();

    static void playMatchStart();

    static bool isLinkRunning();
    static int getLinkPulseUs();

private:
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
    static bool laughWanted;
    static bool waveEverPlayed;
    static unsigned long waveStartMs;
    static unsigned long readyLastStartMs;
    static SoundLine wave;
    static SoundLine laugh;
    static SoundLine ready;
    static SoundLine start;
    static uint8_t readyPlays;
    static uint8_t readyStutterLeft;
};
