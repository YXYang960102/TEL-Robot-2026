#pragma once
#include <Arduino.h>

class Vision {
public:
    // Mirrors Codex's Orin-side lifecycle states exactly (VISION_STANDBY /
    // VISION_STARTING / VISION_READY / VISION_ERROR). UNKNOWN means "nothing
    // heard yet, or the last state line went stale."
    enum class OrinState { UNKNOWN, STANDBY, STARTING, READY, ERROR };

    // Lifecycle
    static void init();
    static void update();

    // Target facts from YOLO (latest valid frame; zeroed when invalid).
    static double getTx();
    static double getTy();
    static double getDistance();
    static int getTargetId();
    static bool isValid();

    // The operator's chosen hole, in the Orin's numbering (0 = none, the
    // Orin picks; 4..12 = OperatorTarget::visionIdForHole()). Sent to the
    // Orin as MEGA_TARGET on change and with every heartbeat. While a hole
    // is selected, a frame for any other id counts as "no target", and a
    // change drops the current frame at once.
    static void setSelectedTarget(uint8_t visionId);
    static uint8_t getSelectedTarget();

    // Transport and Orin lifecycle status
    static bool isConnected();
    static unsigned long getPacketAgeMs();
    static OrinState getOrinState();
    static bool isVisionReady();

private:
    static bool parse(const String& s);
    static bool parseOrinControl(const String& s);
    static void storeTarget(
        double tx,
        double ty,
        double distance,
        int targetId,
        bool valid);
    static void invalidateTarget();
    static void sendHeartbeat();
    static void sendSelection();

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
    static uint8_t selectedTargetId;
    static bool selectionDirty;
    static OrinState orinState;
    static unsigned long lastOrinStateMs;
};
