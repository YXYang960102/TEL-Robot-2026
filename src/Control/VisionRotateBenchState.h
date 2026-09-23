#pragma once
#include <stdint.h>
#include <string.h>
#include "../Constants/VisionRotateBenchConstants.h"

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
    uint32_t lastUsbPing = 0;
    uint32_t lastVisionRx = 0;

    void stop(const char* reason="OPERATOR") {
        enabled = false; direction = 0; noLimits = false; stopReason = reason;
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
    // (link liveness is independent of the motion-armed gate).
    void onVision(int tx, bool valid, uint32_t now) {
        lastVisionRx = now;
        if (!enabled || !valid) { direction = 0; return; }
        using namespace VisionRotateBenchConstants;
        if (tx > DEADBAND_PX) direction = 1;
        else if (tx < -DEADBAND_PX) direction = -1;
        else direction = 0;
    }

    int pulse() const {
        using namespace VisionRotateBenchConstants;
        return NEUTRAL_US + (enabled ? direction : 0) *
            (INVERTED ? -FOLLOW_JOG_OFFSET_US : FOLLOW_JOG_OFFSET_US);
    }
};
