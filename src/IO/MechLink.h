#pragma once

#include <Arduino.h>

// Replaces SBUS.h/.cpp. The Mega no longer parses SBUS: a dedicated STM32
// (tools/stm32_elrs_robot_bridge) decodes CRSF from the ES900RX radio
// module and forwards a plain ASCII line
//   "CH,fwd,turn,mech,aux,fire,mode\n"
// over Pins.h's MECH_LINK_SERIAL_PORT. Field order/meaning is
// tools/stm32_elrs_common/src/tel_channel_map.h's TEL_CH_* assignment.
// Keeps the old SBUS class's ch0/ch1/ch2/ch3/ch8 public shape so
// Chassis.cpp/Telemetry.cpp only needed a class-name/include change.
//
// Adds a signal-loss safeguard the old SBUS class never had (it updated
// ch0..ch8 only when a frame arrived and otherwise silently left them at
// their last value forever, with no detection of a dropped/absent link --
// every other branch's MechLink already resets to neutral on signal loss,
// see e.g. the Shooter branch's SBUS.h history). Not a new feature so much
// as closing a known gap while touching this file anyway.
class MechLink {
public:
    static void init();
    static void update();

    static int ch0, ch1, ch2, ch3, ch8;

private:
    static void parseLine(const String& line);
    static void setNeutral();

    static unsigned long lastValidLineMs;
    static bool everReceivedLine;
};
