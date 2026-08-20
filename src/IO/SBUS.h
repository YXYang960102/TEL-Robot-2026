#pragma once
#include <sbus.h>



class SBUS {
public:
    static void init();
    static void update();

    static bool isHealthy();
    static unsigned long getFrameAgeMs();
    static double getDriveForward();
    static double getDriveTurn();

    static int ch0, ch1, ch2, ch3, ch8;

private:
    static void setNeutral();
    static double normalizedPulse(int pulseUs);

    static bfs::SbusRx sbus;
    static bfs::SbusData data;
    static unsigned long lastValidFrameMs;
    static bool hasValidFrame;
};
