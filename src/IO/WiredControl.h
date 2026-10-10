#pragma once

#include <Arduino.h>

#include "../Constants/WiredControlConstants.h"

// Production wired-Dashboard command source: USB `Serial`, parsing a
// line-based ASCII protocol from tools/dashboard/index.html's keyboard
// controls. Entirely separate from MechLink (Serial2/ELRS wireless) --
// this is the "no manual joystick channel wired yet" gap several comments
// in Auto.cpp/Shooter.h called out before this existed. See
// docs/codex-handoff.md 2026-10-11 for the full command table.
//
// Chassis/Dribbler open-loop commands are applied directly from here.
// Shooter's Rotate axis and Dribbler's COUNTED_FEED gate are arbitrated
// with Auto.cpp's existing auto-fire sequence via the isXAuthorityActive()
// flags below, rather than Auto.cpp unconditionally overwriting them every
// tick the way it did before any manual input source existed.
class WiredControl {
public:
    static void init();
    static void update();

    // True whenever WiredControl should be considered the owner of
    // Shooter's Rotate axis this tick (armed and connected) -- it decides
    // internally whether that means a manual jog (L) or vision-follow,
    // calling Shooter::setRotateOpenLoop()/setRotateVisionFollow()/
    // disableRotate() itself. Auto::update() checks this and skips its own
    // Rotate arbitration when true.
    static bool isRotateAuthorityActive();

    // True only while a wired F (single-ball) request is in flight --
    // WiredControl asserts Dribbler::setFeedAllowed(true) for exactly that
    // window. Auto::update() checks this and skips forcing
    // setFeedAllowed(false) when true, so a wired F request isn't starved
    // by Auto's own auto-fire permission gate (which is otherwise false
    // whenever there's no RC signal, i.e. always, during wired-only
    // testing). R (continuous feed) does NOT need this -- it runs through
    // Dribbler's MANUAL_OPEN_LOOP mode, which isn't gated by
    // setFeedAllowed() at all.
    static bool isDribblerAuthorityActive();

    // Telemetry
    static bool isArmed();
    static bool isConnected();
    static bool didTimeOut();

private:
    static void readIncomingLines();
    static void processCommand(char* command);
    static void handleTankCommand(char* args);
    static void handleAxisCommand(char* args, double& target);
    static void enforceWatchdog();
    static void setEnabled(bool enabled);
    static void sendEvent(const char* name);
    static void refreshWatchdog();

    static char commandBuffer[WiredControlConstants::COMMAND_BUFFER_SIZE];
    static size_t commandLength;

    static bool armed;
    static bool everConnected;
    static bool timedOut;
    static unsigned long lastCommandMs;

    static double tankLeftCommand;
    static double tankRightCommand;
    static double rotateManualCommand;
    static double angleManualCommand;
    static bool rotateManualActive; // L: false=vision-follow, true=manual jog
    static bool continuousFeedOn;   // R's logical toggle state
    static bool singleShotPending;  // F request still in flight
};
