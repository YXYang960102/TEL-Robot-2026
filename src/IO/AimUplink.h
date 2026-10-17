#pragma once

// Aim frame for the wireless status uplink (RcControlMap.h, "Aim frames"):
// the camera distance, the elevation pulse, the hole as sent to the Orin,
// the frame's hole / validity and the aim offset, so the Dashboard shows
// them through the ground station too (2026-10-17). Plus the vision frame
// (tx / ty / distance). ManualControl's 100 ms status slots go status, aim,
// status, vision.
class AimUplink {
public:
    static void send();
    static void sendVision();
};
