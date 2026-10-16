#include "AimUplink.h"

#include <math.h>

#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"
#include "ManualControl.h"
#include "MechLink.h"
#include "RcControlMap.h"

namespace {
long clampLong(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

void AimUplink::send() {
    rcmap_aim a = {};
    const bool valid = Vision::isValid();
    const double mm = Vision::getDistance();
    // 0 = no distance (no valid target, or the webcam's distance 0).
    a.distance_cm = static_cast<uint16_t>(valid && mm > 0.0 ? clampLong(lround(mm / 10.0), 1, 0x7FFF) : 0);
    a.elevation_pulse_us = static_cast<uint16_t>(clampLong(Shooter::getAnglePulseUs(), 0, 0x0FFF));
    a.selected_id = Vision::getSelectedTarget();
    a.target_id = static_cast<uint8_t>(valid ? clampLong(Vision::getTargetId(), 0, 15) : 0);
    a.valid = valid;
    a.aim_offset_px = static_cast<int8_t>(clampLong(lround(ManualControl::getAimOffsetPx()), -127, 127));

    uint8_t flags = 0;
    uint32_t seq = 0;
    uint16_t code = 0;
    rcmap_aim_to_status(&a, &flags, &seq, &code);
    MechLink::sendStatus(flags, seq, code);
}
