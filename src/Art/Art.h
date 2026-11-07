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
    static void init();   // motor at stop, sound lines HIGH
    static void update(); // after Dribbler::update()

    static bool isLinkRunning();
    static int getLinkPulseUs();

private:
    static void onShot(unsigned long now);
    static void startSound(uint8_t pin, unsigned long now, unsigned long& startMs, bool& active);
    static uint8_t shotHole();

    static Servo linkMotor;
    static int linkPulseUs;
    static bool linkRunning;
    static unsigned long linkStartMs;
    static unsigned long lastTriggerCount;
    static uint8_t lastShotHole;
    static bool waveActive;
    static unsigned long waveStartMs;
    static bool laughActive;
    static unsigned long laughStartMs;
    static bool laughPending;
    static unsigned long laughDueMs;
};
