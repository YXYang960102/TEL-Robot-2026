#pragma once

// Replaces SBUS.h/.cpp. The Mega no longer parses SBUS: a dedicated STM32
// (tools/stm32_elrs_robot_bridge) decodes CRSF from the ES900RX radio
// module and forwards a plain ASCII line
//   "CH,fwd,turn,mech,aux,fire,mode\n"
// over Pins.h's MECH_LINK_SERIAL_PORT. Field order/meaning is
// tools/stm32_elrs_common/src/tel_channel_map.h's TEL_CH_* assignment.
// This class keeps the same public shape the old SBUS class had
// (ch0/ch1/ch3/signalLost/modeChannel/fireChannel) so call sites only
// needed a class-name/include change, not a logic change.
class MechLink {
public:
    static void init();
    static void update();

    // driveForward, driveTurn, auxiliary (pulse-us convention, same
    // 1000-2000us-ish range the retired SBUS fields of the same name used).
    // Consumed by the mecanum mixing in Chassis.cpp.
    static int ch0, ch1, ch3;

    // True until the first valid "CH,..." line ever arrives, and again
    // whenever one hasn't arrived recently -- mirrors the old SBUS class's
    // fail-safe behavior (lost_frame/failsafe/timeout all meant the same
    // thing: do not trust these values for autonomous motion).
    static bool signalLost;

    // Dedicated operator-mode switch input (full-auto/semi-auto/full-manual
    // keypad). Provisional: the real keypad doesn't exist yet, see
    // tools/stm32_elrs_ground_bridge/README.md.
    static int modeChannel;

    // Momentary fire-confirm button (semi-auto only, see Auto::update()).
    static int fireChannel;

private:
    static void parseLine(const String& line);
    static void setNeutral();
};
