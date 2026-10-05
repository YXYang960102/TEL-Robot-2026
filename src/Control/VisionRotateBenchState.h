#pragma once
#include <stdint.h>
#include <string.h>
#include "../Constants/VisionRotateBenchConstants.h"
#include "PositionDecelerationProfile.h"

// Pure state machine, shared by the firmware and host regression tests.
// Direction comes from Orin vision data (onVision), never from the browser —
// the browser only arms/disarms (E/E_NO_LIMITS/X) and proves its tab is
// still alive (P), the same way the keyboard bench proves liveness via
// repeated M commands.
class VisionRotateBenchState {
public:
    bool enabled = false;
    bool noLimits = false;
    const char* stopReason = "BOOT";
    int direction = 0;
    int speedOffsetUs = 0; // magnitude selected by onVision's zone logic; 0 when stopped
    uint32_t lastUsbPing = 0;
    uint32_t lastVisionRx = 0;

    void stop(const char* reason="OPERATOR") {
        enabled = false; direction = 0; speedOffsetUs = 0; noLimits = false; stopReason = reason;
    }

    void applyLimits(bool left, bool right) {
        if (!noLimits && ((direction < 0 && left) || (direction > 0 && right))) stop("LIMIT");
    }

    void tick(uint32_t now) {
        using namespace VisionRotateBenchConstants;
        // Two independent watchdogs: the browser tab (USB "P" pings) and the
        // Orin vision link (Serial1 lines) must both stay alive, or we stop.
        if (enabled && uint32_t(now - lastUsbPing) > COMMAND_TIMEOUT_MS) { stop("TIMEOUT"); return; }
        if (enabled && uint32_t(now - lastVisionRx) > VISION_TIMEOUT_MS) { stop("VISION_TIMEOUT"); return; }
    }

    bool command(const char* text, uint32_t now) {
        tick(now); // A late ping/enable cannot revive already-timed-out movement.
        if (!strcmp(text, "X")) { stop(); return true; }
        if (!strcmp(text, "E") || !strcmp(text, "E_NO_LIMITS")) {
            noLimits = !strcmp(text, "E_NO_LIMITS");
            stopReason = "NONE";
            direction = 0; enabled = true; lastUsbPing = now; lastVisionRx = now;
            return true;
        }
        if (!strcmp(text, "P")) {
            if (enabled) lastUsbPing = now;
            return true;
        }
        stop("BAD_COMMAND");
        return false;
    }

    // Called for every decoded Serial1 line from Orin, regardless of `enabled`
    // (link liveness is independent of the motion-armed gate). Inside the
    // deadband: stop. Outside it: speed comes from PositionDecelerationProfile
    // (the same profiled-deceleration utility already used for Shooter
    // elevation) — a smooth ramp from FOLLOW_CREEP_OFFSET_US up to
    // FOLLOW_JOG_OFFSET_US across the slow zone, floored at the creep speed,
    // instead of a hand-rolled step between two fixed speeds. This is what
    // gradually slows the servo down as it approaches center rather than
    // slamming into it at full speed and overshooting back and forth.
    void onVision(int tx, bool valid, uint32_t now) {
        lastVisionRx = now;
        if (!enabled || !valid) { direction = 0; speedOffsetUs = 0; return; }
        using namespace VisionRotateBenchConstants;
        const int absTx = tx < 0 ? -tx : tx;
        if (absTx <= DEADBAND_PX) {
            direction = 0; speedOffsetUs = 0;
        } else {
            direction = tx > 0 ? 1 : -1;
            const double command = PositionDecelerationProfile::calculateMaximumCommand(
                SLOW_ZONE_PX, absTx, 1.0, FOLLOW_JOG_OFFSET_US, FOLLOW_CREEP_OFFSET_US);
            speedOffsetUs = (int)(command + 0.5);
        }
    }

    int pulse() const {
        using namespace VisionRotateBenchConstants;
        const int signedOffset = INVERTED ? -speedOffsetUs : speedOffsetUs;
        return NEUTRAL_US + (enabled ? direction : 0) * signedOffset;
    }
};
