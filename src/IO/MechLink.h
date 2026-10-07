#pragma once

#include <Arduino.h>

// Replaces SBUS.h/.cpp. The Mega no longer parses SBUS: a dedicated STM32
// (tools/stm32_elrs_robot_bridge) decodes CRSF from the ES900RX radio
// module and forwards a plain ASCII line
//   "CH,fwd,turn,mech,aux,fire,mode\n"
// over Pins.h's MECH_LINK_SERIAL_PORT. Field order/meaning is
// tools/stm32_elrs_common/src/tel_channel_map.h's TEL_CH_* assignment.
// This class keeps the same public shape the old SBUS class had so call
// sites only needed a class-name/include change, not a logic change. The
// "fire" field has no consumer on this branch yet (that lives in Auto.cpp
// on the Shooter branch) and is parsed but not exposed here.
class MechLink {
public:
    static void init();
    static void update();

    static bool isHealthy();
    static unsigned long getFrameAgeMs();

    // Normalized -1.0..1.0 drive commands, same convention the old SBUS
    // class's getDriveForward()/getDriveTurn() used.
    static double getDriveForward();
    static double getDriveTurn();

    // Raw pulse-us values, same names/semantics the old SBUS class exposed.
    static int getDriveForwardPulseUs();
    static int getAuxiliaryPulseUs();
    static int getMechanismPulseUs();
    static int getDriveTurnPulseUs();
    static int getModePulseUs();

    // Forwards the Mega's "TEL,..." dashboard line (Telemetry.cpp) to the
    // robot-side STM32 over the same serial port, for it to relay back to
    // the ground-station dashboard over the ELRS backlink (see
    // tools/stm32_elrs_robot_bridge/README.md). Telemetry.cpp still also
    // prints this same line to the Mega's own USB Serial directly -- that
    // wired path stays useful for bench debugging.
    static void sendTelemetryLine(const String& line);

private:
    static void parseLine(const String& line);
    static void setNeutral();
    static double normalizedPulse(int pulseUs);

    static int driveForwardPulseUs;
    static int auxiliaryPulseUs;
    static int mechanismPulseUs;
    static int driveTurnPulseUs;
    static int modePulseUs;
    static unsigned long lastValidFrameMs;
    static bool hasValidFrame;
};
