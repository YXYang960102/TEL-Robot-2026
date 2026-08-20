#pragma once
#include <Arduino.h>

class Vision {
public:
    static void init();
    static void update();

    static double getXPred();
    static double getTx();
    static double getTy();
    static double getDistance();
    static int getTargetId();
    static bool isValid();
    static bool isConnected();
    static unsigned long getPacketAgeMs();

private:
    static bool parse(const String& s);
    static void updatePrediction(double tx, double ty, double distance, int targetId, bool valid);
    static void invalidateTarget();

    static String rx;
    static double tx;
    static double ty;
    static double distance;
    static int targetId;
    static bool valid;
    static unsigned long lastPacketMs;
    static bool hasPacket;
};
