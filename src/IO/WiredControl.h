#pragma once

#include <Arduino.h>

#include "../Constants/WiredControlConstants.h"

// Production wired-Dashboard command source: USB `Serial`, parsing a
// line-based ASCII protocol from tools/dashboard/index.html. Entirely
// separate from MechLink (Serial2/ELRS wireless). See docs/codex-handoff.md
// 2026-10-11 for the full command table.
//
// Control-source rule: wired and wireless never drive outputs at the same
// time. Enable is refused while MechLink reports a live link, and an armed
// wired session disarms itself the moment one appears. While armed (which
// therefore implies no RC link, so Auto's auto-fire can't run anyway),
// WiredControl owns Chassis, Dribbler, and all three Shooter axes, and
// Auto::update() stays out of their way via isActive().
class WiredControl {
public:
    static void init();
    static void update();

    // True while the wired Dashboard is armed and therefore the sole owner
    // of Chassis/Dribbler/Shooter outputs. Auto::update() skips its own
    // Rotate arbitration, flywheel zeroing, and feed-gate writes when true.
    static bool isActive();

    // Telemetry
    static bool isArmed();
    static bool isConnected();
    static bool didTimeOut();
    static double getFlywheelCommand();

private:
    static void readIncomingLines();
    static void processCommand(char* command);
    static bool parseTank(char* args, double& left, double& right);
    static bool parseValue(char* args, double minimum, double maximum, double& value);
    static void requestEnable();
    static void enforceWatchdog();
    static void disarm(const char* reasonEvent);
    static void clearCommands();
    static void sendEvent(const char* name);
    static void refreshWatchdog();

    static char commandBuffer[WiredControlConstants::COMMAND_BUFFER_SIZE];
    static size_t commandLength;
    static bool discardingLine;  // inside an oversized line, waiting for '\n'

    static bool armed;
    static bool everConnected;
    static bool timedOut;
    static unsigned long lastCommandMs;

    static double tankLeftCommand;
    static double tankRightCommand;
    static double rotateManualCommand;
    static double angleManualCommand;
    static double flywheelCommand;  // 0.0-1.0 open-loop ratio, not RPM
    static bool rotateManualActive; // L: false=vision-follow, true=manual jog
    static bool continuousFeedOn;   // R's logical toggle state
    static bool singleShotPending;  // F request still in flight
};
