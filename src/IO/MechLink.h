#pragma once

#include <Arduino.h>

// Replaces SBUS.h/.cpp. The Mega no longer parses SBUS: a dedicated STM32
// (tools/stm32_elrs_robot_bridge) decodes CRSF from the ES900RX radio
// module and forwards a plain ASCII line
//   "CH,fwd,turn,mech,aux,fire,mode\n"
// over Pins.h's MECH_LINK_SERIAL_PORT. Field order/meaning is
// tools/stm32_elrs_common/src/tel_channel_map.h's TEL_CH_* assignment.
// This class keeps the same public shape the old SBUS class had
// (ch0/ch1/ch2/ch3/ch8/isHealthy/getFrameAgeMs/getDriveForward/
// getDriveTurn) so call sites only needed a class-name/include change.
class MechLink {
public:
    static void init();
    static void update();

    static bool isHealthy();
    static unsigned long getFrameAgeMs();
    static double getDriveForward();
    static double getDriveTurn();

    static int ch0, ch1, ch2, ch3, ch8;

private:
    static void parseLine(const String& line);
    static void setNeutral();
    static double normalizedPulse(int pulseUs);

    static unsigned long lastValidFrameMs;
    static bool hasValidFrame;
};
