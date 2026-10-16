#pragma once

#include <Arduino.h>

#include "../Constants/ManualControlConstants.h"
#include "OperatorMode.h"
#include "RcControlMap.h"

// Operator control of the robot, from two sources that share one set of
// rules:
//   - Wired: line protocol on USB `Serial` from tools/dashboard/index.html
//     (E/X/K, T,<l>,<r>, ROT,<d>, ANG,<d>, ANGT,<deg>, FLY,<0..1>,
//     TGT,<0..9>, AUTO,<1..4>, F, M).
//   - Wireless: CRSF channels from the ELRS link via MechLink, decoded with
//     RcControlMap table v2 (forward/turn, target hole, mode, fire, arm,
//     nudge, auto path).
//
// Control authority: exactly one source owns the robot while armed. Enable
// from the other source is refused (ENABLE_REJECTED_OTHER_SOURCE), and its
// motion/F/TGT/AUTO commands are refused (REJECTED_NOT_OWNER). An explicit stop
// (wired X) always disarms whoever owns it; the wireless arm level going
// low only disarms a wireless session (low is the ground station's idle
// state, so it can't mean "stop the wired session").
//
// Wireless enable is edge-triggered: CH6 has to be seen low and then high.
// Fire (CH5) and auto path (CH8) act on their rising edge too. After an RF
// dropout every edge detector re-arms only once it sees the channel
// released again, so recovering the link never re-enables or fires on its
// own.
//
// Target hole (TGT / CH3): main.cpp passes it to Vision (the Orin locks on
// that hole) and Auto uses that hole's shot table. The auto path
// (AUTO / CH8) can start once per Enable, and every path started since
// boot is remembered for the Dashboard; the paths themselves are not built
// yet.
//
// Fine-adjust in the auto modes: each arrow press (ROT/ANG going from 0 to
// +-1; wireless: CH7 leaving center) adds a step to the aim offset
// (left/right) or, once the shot table's elevation is measured, the
// elevation offset (up/down). A new hole, Enable and disarm clear both.
//
// Operator mode (OperatorMode.h): M cycles FULL_MANUAL -> SEMI_AUTO ->
// FULL_AUTO while armed (wireless: CH4 level). It is FULL_MANUAL whenever
// disarmed and at every Enable; a wireless Enable whose CH4 isn't manual is
// refused. In the auto modes this class still owns safety, the chassis
// and elevation, but leaves the turret and flywheel to Auto, and
// F becomes a fire request that Auto takes (takeFireRequest()).
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

    // FULL_MANUAL while disarmed.
    static OperatorMode getOperatorMode();
    // True once per F press made in an auto mode (cleared by reading it).
    static bool takeFireRequest();
    // Lets Auto report what it did on the same "$EVENT" / status channel.
    static void reportEvent(rcmap_event event);

    static uint8_t getTarget();               // 0 = none, 1..9
    static uint8_t getAutoPathThisEnable();   // 0 = none, 1..4
    static uint8_t getAutoPathsUsedMask();    // bit n = path n started since boot
    // Fine-adjust (TuningConstants::Auto): + = aim right / elevation up.
    static double getAimOffsetPx();
    static double getElevationOffsetUs();
    static constexpr int NUDGE_NEEDS_RELEASE = 2; // see nudgePressed()

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
    static void selectTarget(uint8_t hole);
    static void updateFineAdjust();
    static void clearFineAdjust();
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

    static double tankLeftCommand;
    static double tankRightCommand;
    static double rotateManualCommand;
    static double angleManualCommand;
    static double flywheelCommand;
    static OperatorMode operatorMode;
    static bool fireRequested;
    static uint8_t target;
    static uint8_t autoPathThisEnable;
    static uint8_t autoPathsUsedMask;
    static double aimOffsetPx;
    static double elevationOffsetUs;
    static int rotateNudgePrev; // sign of the last ROT/nudge, for press edges
    static int angleNudgePrev;

    static unsigned long eventSeq;
    static uint16_t lastEventCode;
    static unsigned long lastStatusMs;
};
