#pragma once

#include <Arduino.h>

#include "../Constants/ManualControlConstants.h"
#include "RcControlMap.h"

// Operator control of the robot, from two sources that share one set of
// rules:
//   - Wired: line protocol on USB `Serial` from tools/dashboard/index.html
//     (E/X/K, T,<l>,<r>, ROT,<d>, ANG,<d>, FLY,<0..1>, F, L).
//   - Wireless: CRSF channels from the ELRS link via MechLink, decoded with
//     RcControlMap (arm level on CH5, axes, F/L event counters).
//
// Control authority: exactly one source owns the robot while armed. Enable
// from the other source is refused (ENABLE_REJECTED_OTHER_SOURCE), and its
// motion/F/L commands are refused (REJECTED_NOT_OWNER). An explicit stop
// (wired X) always disarms whoever owns it; the wireless arm level going
// low only disarms a wireless session (low is the ground station's idle
// state, so it can't mean "stop the wired session").
//
// Wireless enable is edge-triggered: CH5 has to be seen low and then high.
// After an RF dropout the edge detector re-arms only once it sees CH5 low
// again, so recovering the link never re-enables on its own.
//
// While armed (either source), Auto::update() stays out of the way.
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

private:
    // Wired line protocol
    static void readIncomingLines();
    static void processCommand(char* command);
    static bool parseTank(char* args, double& left, double& right);
    static bool parseValue(char* args, double minimum, double maximum, double& value);

    // Wireless
    static void processWirelessFrame();
    static void resetWirelessTracking();

    // Shared
    static void requestEnable(Owner source);
    static void enforceWatchdogs();
    static void disarm(rcmap_event reason);
    static void clearCommands();
    static void triggerFeed();
    static void toggleRotateMode();
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
    static rcmap_counter feedCounter;
    static rcmap_counter lmodeCounter;

    static double tankLeftCommand;
    static double tankRightCommand;
    static double rotateManualCommand;
    static double angleManualCommand;
    static double flywheelCommand;
    static bool rotateManualActive; // L: false = vision-follow, true = manual jog

    static unsigned long eventSeq;
    static uint16_t lastEventCode;
    static unsigned long lastStatusMs;
};
