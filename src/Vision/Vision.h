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
    // Protocol v3: the selected hole is out of view and this is where the
    // Orin predicts it (getTx() / getTy() are that position, clamped);
    // aim only -- never counts as isValid(), never a distance. False again
    // after PREDICTED_FOLLOW_MAX_MS of predictions without the real hole.
    static bool isPredicted();
    // The Orin's reliably detected hole ids in the last frame (bit id - 1).
    static uint16_t getSeenMask();
    // The target's depth sampling ring (2026-10-28, Dashboard only): the
    // share of its pixels with usable depth, 0-100 (-1 = not reported or
    // no valid target), and which of its 8 sectors have depth (bit k =
    // sector k, k = 0 right, then clockwise on screen: 2 below, 4 left,
    // 6 above).
    static int8_t getRingPercent();
    // The lit bonus hole the Orin sees (vision id 1-3; 0 = none, or an
    // Orin without frame field 9), from the latest frame whatever its hole.
    static uint8_t getLitBonusId();
    static uint8_t getRingSectors();
    // Bumped once per target frame from the Orin (valid or not), so a
    // consumer can take each frame exactly once (Auto's distance filter).
    static unsigned long getFrameCount();

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
        int validCode,
        uint16_t seenMask);
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
    static unsigned long frameCount;
    static bool predicted;
    static unsigned long predictedSinceMs;
    static uint16_t seenMask;
    static int8_t ringPercent;
    static uint8_t litBonusId;
    static uint8_t ringSectors;
    static bool hasPacket;
    static bool sentReady;
    static unsigned long lastHeartbeatMs;
    static uint8_t selectedTargetId;
    static bool selectionDirty;
    static OrinState orinState;
    static unsigned long lastOrinStateMs;
};
