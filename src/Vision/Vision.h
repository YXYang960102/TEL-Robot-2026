#pragma once
#include <Arduino.h>

class Vision {
public:
    // Mirrors Codex's Orin-side lifecycle states exactly (VISION_STANDBY /
    // VISION_STARTING / VISION_READY / VISION_ERROR). UNKNOWN means "nothing
    // heard yet, or the last state line went stale."
    enum class OrinState { UNKNOWN, STANDBY, STARTING, READY, ERROR };

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
    static OrinState getOrinState();
    static bool isVisionReady();

private:
    static bool parse(const String& s);
    static bool parseOrinControl(const String& s);
    static void updatePrediction(double tx, double ty, double distance, int targetId, bool valid);
    static void invalidateTarget();
    static void sendHeartbeat();

    static String rx;
    static double tx;
    static double ty;
    static double distance;
    static int targetId;
    static bool valid;
    static unsigned long lastPacketMs;
    static bool hasPacket;
    static bool sentReady;
    static unsigned long lastHeartbeatMs;
    static OrinState orinState;
    static unsigned long lastOrinStateMs;
};
