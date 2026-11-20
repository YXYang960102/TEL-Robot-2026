#pragma once

#include <Arduino.h>

#include "../Constants/ManualControlConstants.h"
#include "OperatorMode.h"
#include "RcControlMap.h"

class ManualControl {
public:
    enum class Owner : uint8_t { NONE = 0, WIRED, WIRELESS };

    static void init();
    static void update();

    static bool isActive();
    static bool isArmed();
    static Owner getOwner();
    static bool isConnected();
    static bool didTimeOut();
    static double getFlywheelCommand();

    static OperatorMode getOperatorMode();
    static bool takeFireRequest();
    static void reportEvent(rcmap_event event);

    static uint8_t getTarget();
    static uint8_t getBallColor();
    static bool isSecondHalf();
    static bool takeMatchStart();
    static uint16_t getTargetSelectSeq();
    static uint8_t getAutoPathThisEnable();
    static uint8_t getAutoPathsUsedMask();
    static double getAimOffsetPx();
    static double getElevationOffsetUs();
    static uint16_t getTrimSeq();
    static bool isCaptureOn();
    static void setTrim(double aimOffsetPx, double elevationOffsetUs);
    static bool isAimJogActive();
    static double getAimJogCommand();
    static bool isAimHandHeld();
    static bool isTurretHeldHome();
    static int getFlywheelOffsetPct();
    static constexpr int NUDGE_NEEDS_RELEASE = 2;

private:
    static void readIncomingLines();
    static void processCommand(char* command);
    static bool parseTank(char* args, double& left, double& right);
    static bool parseValue(char* args, double minimum, double maximum, double& value);

    static void processWirelessFrame();
    static void resetWirelessTracking();

    static void requestEnable(Owner source);
    static void enforceWatchdogs();
    static void disarm(rcmap_event reason);
    static void clearCommands();
    static void triggerFeed();
    static void selectTarget(uint8_t hole);
    static void updateFineAdjust();
    static void clearFineAdjust();
    static void clearTrim();
    static void toggleCapture();
    static void reportPathState();
    static bool pathStickSemi;
    static uint8_t lastPathState;
    static bool captureOn;
    static void requestAutoPath(uint8_t path);
    static void cycleOperatorMode();
    static void setOperatorMode(OperatorMode mode);
    static void handleFire();
    static bool autoOwnsElevation();
    static void applyOutputs();
    static void sendStatusIfDue();
    static void sendEvent(rcmap_event event);
    static void refreshWiredWatchdog();
    static uint8_t statusFlags();

    static char commandBuffer[ManualControlConstants::COMMAND_BUFFER_SIZE];
    static size_t commandLength;
    static bool discardingLine;

    static Owner owner;
    static bool everConnected;
    static bool timedOut;
    static unsigned long lastWiredCommandMs;

    static unsigned long lastFrameCount;
    static unsigned long lastValidWirelessMs;
    static bool wirelessArmPrev;
    static bool wirelessFirePrev;
    static uint8_t wirelessAutoPrev;
    static bool wirelessHomePrev;
    static bool wirelessCapturePrev;
    static bool wirelessStartPrev;
    static bool matchStartRequested;
    static void matchStart();
    static unsigned long lastVersionEventMs;

    static double tankLeftCommand;
    static double tankRightCommand;
    static double rotateManualCommand;
    static double angleManualCommand;
    static double flywheelCommand;
    static OperatorMode operatorMode;
    static bool fireRequested;
    static uint8_t target;
    static uint8_t ballColor;
    static uint8_t half;
    static void setHalf(uint8_t newHalf, bool announceSame);
    static void setBallColor(uint8_t color, bool announceSame);
    static uint8_t autoPathThisEnable;
    static uint8_t autoPathsUsedMask;
    static double aimOffsetPx;
    static double elevationOffsetUs;
    static int flywheelOffsetPct;
    static int rotateNudgePrev;
    static bool turretHeldHome;
    static uint16_t targetSelectSeq;
    static void requestTurretHome();
    static bool aimJogging;
    static bool aimJogSettling;
    static bool aimHandHeld;
    static uint8_t handHeldAimTarget;
    static unsigned long aimJogReleasedMs;
    static int angleNudgePrev;
    static unsigned long elevationJogLastMs;
    static bool elevationJogPrev;
    static uint16_t trimSeq;

    static unsigned long eventSeq;
    static uint16_t lastEventCode;
    static unsigned long lastStatusMs;
    static uint8_t statusSlot;
};
