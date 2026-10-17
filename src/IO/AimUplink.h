#pragma once

// Aim frame for the wireless status uplink (RcControlMap.h, "Aim frames"):
// the camera distance, the elevation pulse, the hole as sent to the Orin,
// the frame's hole / validity and the aim offset, so the Dashboard shows
// them through the ground station too (2026-10-17). ManualControl sends
// it in every third 100 ms status slot.
class AimUplink {
public:
    static void send();
};
