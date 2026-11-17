#pragma once
#include <Arduino.h>

class Vision {
public:
    enum class OrinState { UNKNOWN, STANDBY, STARTING, READY, ERROR };

    static void init();
    static void update();

    static double getTx();
    static double getTy();
    static double getDistance();
    static int getTargetId();
    static bool isValid();
    static bool isPredicted();
    static uint16_t getSeenMask();
    static int8_t getRingPercent();
    static uint8_t getLitBonusId();
    static uint8_t getNearBottomId();
    static uint8_t getRingSectors();
    static unsigned long getFrameCount();

    static void setSelectedTarget(uint8_t visionId);
    static uint8_t getSelectedTarget();
    static void setCaptureRequested(bool on);

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
    static void sendCapture();
    static bool captureRequested;
    static bool captureDirty;

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
    static uint8_t nearBottomId;
    static uint8_t ringSectors;
    static bool hasPacket;
    static bool sentReady;
    static unsigned long lastHeartbeatMs;
    static uint8_t selectedTargetId;
    static bool selectionDirty;
    static OrinState orinState;
    static unsigned long lastOrinStateMs;
};
