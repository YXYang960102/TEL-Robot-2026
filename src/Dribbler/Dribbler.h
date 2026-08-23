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

    // Manual open-loop control
    static void setFeedAction(FeedAction action);
    static void setFeedOpenLoop(double command);

    // Counted feeding
    static void setShootRequest(int count);

    // Status and telemetry
    static DribblerControlMode getControlMode();
    static int getShootRemaining();
    static unsigned long getCompletedShotCount();
    static bool isExitSensorBlocked();
    static bool isFeeding();
    static double getFeedCommand();
    static int getFeedPwm();

private:
    static void writeFeedOutput(double command);
    static void updateExitSensor();
    static bool readExitSensorBlocked();

    static DribblerControlMode controlMode;
    static int shootRemaining;
    static unsigned long completedShotCount;
    static bool feeding;
    static double feedCommand;
    static int feedPwm;
    static bool rawSensorBlocked;
    static bool stableSensorBlocked;
    static bool ballSeenDuringRequest;
    static unsigned long rawSensorChangedMs;
};
