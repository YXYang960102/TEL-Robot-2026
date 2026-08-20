#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Arduino.h>

class Dribbler {
public:
    static void init();
    static void update();

    static void setShootRequest(int count);
    static int getShootRemaining();
    static unsigned long getCompletedShotCount();
    static bool isExitSensorBlocked();
    static bool isFeeding();
    static void stop();

private:
    static int shootRemaining;
    static unsigned long completedShotCount;
    static bool feeding;

    static void runFeed();
    static void updateExitSensor();
    static bool readExitSensorBlocked();

    static bool rawSensorBlocked;
    static bool stableSensorBlocked;
    static bool ballSeenDuringRequest;
    static unsigned long rawSensorChangedMs;
};
#endif
