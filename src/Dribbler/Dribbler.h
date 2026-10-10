#pragma once

#include <Arduino.h>

#include "../Constants/DribblerConstants.h"

enum class DribblerControlMode {
    DISABLED,
    MANUAL_OPEN_LOOP,
    COUNTED_FEED
};

class Dribbler {
public:
    using FeedAction = DribblerConstants::Feeder::Action;

    // Lifecycle
    static void init();
    static void update();
    static void stop();

    // Manual open-loop control (wired R routes through this -- continuous,
    // not sensor-gated).
    static void setFeedAction(FeedAction action);
    static void setFeedOpenLoop(double command);

    // Counted feeding (wired F, and Auto's auto-fire sequence, both route
    // through this -- sensor-gated, decrements as balls are confirmed
    // exiting).
    static void setShootRequest(int count);

    // Gates whether update() is allowed to actually run the feed motors in
    // COUNTED_FEED mode, even if a shot is queued. Kept from the pre-merge
    // dev Dribbler -- Auto::update() uses this as its auto-fire permission
    // gate, and src/IO/WiredControl.cpp uses it the same way for a wired F
    // request, so neither has to duplicate COUNTED_FEED/sensor logic.
    static void setFeedAllowed(bool allowed);

    // Status and telemetry
    static DribblerControlMode getControlMode();
    static int getShootRemaining();
    static unsigned long getCompletedShotCount();
    static bool isExitSensorBlocked();
    static bool isFeeding();
    static double getFeedCommand();
    static int getFeedPwm();

    // True once, the first time it's read after a COUNTED_FEED request
    // timed out without the ExitSensor seeing a ball pass (see
    // DribblerConstants::Jam::TIMEOUT_MS). Clears itself on read so the
    // caller (WiredControl, for $EVENT,FEED_JAM) sees each jam exactly
    // once, the same edge-detected-flag pattern Auto.cpp uses for its fire
    // button.
    static bool consumeJamEvent();

private:
    static void writeFeedOutput(double command);
    static void updateExitSensor();
    static bool readExitSensorBlocked();

    static DribblerControlMode controlMode;
    static int shootRemaining;
    static unsigned long completedShotCount;
    static bool feedAllowed;
    static bool feeding;
    static double feedCommand;
    static int feedPwm;
    static bool rawSensorBlocked;
    static bool stableSensorBlocked;
    static bool ballSeenDuringRequest;
    static unsigned long rawSensorChangedMs;
    static unsigned long feedStartedMs;
    static bool jamEventPending;
};
