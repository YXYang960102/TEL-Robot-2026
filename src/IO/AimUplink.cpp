#include "AimUplink.h"

#include <math.h>

#include "../Sensors/TurretAngle.h"
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
    a.distance_cm = static_cast<uint16_t>(valid && mm > 0.0 ? clampLong(lround(mm / 10.0), 1, 0x3FFF) : 0);
    a.elevation_pulse_us = static_cast<uint16_t>(clampLong(Shooter::getAnglePulseUs(), 0, 0x0FFF));
    a.selected_id = Vision::getSelectedTarget();
    a.target_id = static_cast<uint8_t>(valid ? clampLong(Vision::getTargetId(), 0, 15) : 0);
    a.valid = valid;
    // 4 px units since 2026-10-30 (the offset now reaches 300 px; the
    // Dashboard multiplies by 4; the ground passes it through unchanged).
    a.aim_offset_px = static_cast<int8_t>(clampLong(lround(ManualControl::getAimOffsetPx() / 4.0), -127, 127));
    a.turret_state = Shooter::getTurretState(); // 2026-10-31

    uint8_t flags = 0;
    uint32_t seq = 0;
    uint16_t code = 0;
    rcmap_aim_to_status(&a, &flags, &seq, &code);
    MechLink::sendStatus(flags, seq, code);
}

void AimUplink::sendSeen() {
    rcmap_seen v = {};
    v.seen_mask = Vision::getSeenMask();
    v.state = Vision::isValid() ? RCMAP_VISION_STATE_SEEN
            : Vision::isPredicted() ? RCMAP_VISION_STATE_PREDICTED
                                    : RCMAP_VISION_STATE_NONE;
    // 2026-10-28: the turret angle and the depth ring ride along, so the
    // wireless Dashboard shows them too (every 500 ms).
    v.turret_deg10 = static_cast<int16_t>(clampLong(lround(TurretAngle::angleDegrees() * 10.0), -2047, 2047));
    v.turret_status = TurretAngle::status();
    v.ring_percent = Vision::getRingPercent();
    v.ring_sectors = Vision::getRingSectors();
    uint8_t flags = 0;
    uint32_t seq = 0;
    uint16_t code = 0;
    rcmap_seen_to_status(&v, &flags, &seq, &code);
    MechLink::sendStatus(flags, seq, code);
}

void AimUplink::sendVision() {
    rcmap_vision v = {};
    const bool valid = Vision::isValid();
    v.valid = valid;
    // The Falcon's actual pulse (any mode), for the wireless Dashboard.
    v.flywheel_pulse_us = static_cast<uint16_t>(clampLong(Shooter::getFlywheelPulseUs(), 0, 0x3FFF));
    v.tx_px = static_cast<float>(valid ? Vision::getTx() : 0.0);
    v.ty_px = static_cast<float>(valid ? Vision::getTy() : 0.0);
    v.target_id = static_cast<uint8_t>(valid ? clampLong(Vision::getTargetId(), 0, 15) : 0);

    uint8_t flags = 0;
    uint32_t seq = 0;
    uint16_t code = 0;
    rcmap_vision_to_status(&v, &flags, &seq, &code);
    MechLink::sendStatus(flags, seq, code);
}
