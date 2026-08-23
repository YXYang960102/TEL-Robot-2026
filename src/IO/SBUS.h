#pragma once

#include <sbus.h>

class SBUS {
public:
    // Lifecycle
    static void init();
    static void update();

    // Link status
    static bool isHealthy();
    static unsigned long getFrameAgeMs();

    // Chassis commands
    static double getDriveForward();
    static double getDriveTurn();

    // Receiver telemetry. Names describe the current firmware mapping.
    static int getDriveForwardPulseUs();
    static int getAuxiliaryPulseUs();
    static int getMechanismPulseUs();
    static int getDriveTurnPulseUs();
    static int getModePulseUs();

private:
    static void setNeutral();
    static double normalizedPulse(int pulseUs);

    static bfs::SbusRx receiver;
    static bfs::SbusData data;
    static int driveForwardPulseUs;
    static int auxiliaryPulseUs;
    static int mechanismPulseUs;
    static int driveTurnPulseUs;
    static int modePulseUs;
    static unsigned long lastValidFrameMs;
    static bool hasValidFrame;
};
