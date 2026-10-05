#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <Arduino.h>

class Dribbler {
public:
    static void init();
    static void update();

    static void setShootRequest(int count);
    static int getShootRemaining();

    // Gates whether update() is allowed to actually run the feed motor, even
    // if a shot is queued (shoot_pice > 0). Added for the Auto fire sequence
    // (src/Auto/Auto.cpp): without this, Dribbler used to feed as soon as
    // anyone called setShootRequest(), regardless of whether Shooter was
    // even aimed yet. Existing (admittedly imperfect — see Dribbler.cpp)
    // ball-count logic is untouched; this only adds an external permission
    // gate on top of it.
    static void setFeedAllowed(bool allowed);

private:
    static int shoot_pice;

    static void run();
    static void stop();

    static bool sensorUp;
    static bool feedAllowed;
};
#endif