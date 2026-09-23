#pragma once
#include <stdint.h>
#include <string.h>
#include "../Constants/RotateBenchConstants.h"

// Pure state machine, shared by the firmware and host regression tests.
class RotateBenchState {
public:
    bool enabled = false;
    bool noLimits = false;
    const char* stopReason = "BOOT";
    int direction = 0;
    uint32_t lastCommand = 0;
    void stop(const char* reason="OPERATOR") { enabled = false; direction = 0; noLimits = false; stopReason=reason; }
    void applyLimits(bool left, bool right) {
        if (!noLimits && ((direction < 0 && left) || (direction > 0 && right))) stop("LIMIT");
    }
    void tick(uint32_t now) {
        using namespace RotateBenchConstants;
        // JOG_CAP (max continuous-hold duration) removed 2026-09-19 at Jeremy's
        // explicit request for supervised manual testing (motor + camera only,
        // operator present). COMMAND_TIMEOUT_MS stays: it is the loss-of-link
        // watchdog (stale/missing commands, e.g. USB drop or page crash), a
        // different failure mode from "held a key too long while watching it".
        if(enabled && uint32_t(now-lastCommand)>COMMAND_TIMEOUT_MS)stop("TIMEOUT");
    }
    bool command(const char* text, uint32_t now) {
        tick(now); // A late heartbeat cannot revive timed-out movement.
        if (!strcmp(text, "X")) { stop(); return true; }
        if (!strcmp(text, "E") || !strcmp(text, "E_NO_LIMITS")) {
            noLimits = !strcmp(text, "E_NO_LIMITS");
            stopReason = "NONE";
            direction = 0; enabled = true; lastCommand = now; return true;
        }
        int next;
        if (!strcmp(text, "M,0,-1")) next = -1;
        else if (!strcmp(text, "M,0,0")) next = 0;
        else if (!strcmp(text, "M,0,1")) next = 1;
        else { stop("BAD_COMMAND"); return false; }
        if (!enabled) return true;
        lastCommand = now;
        direction = next;
        return true;
    }
    int pulse() const {
        using namespace RotateBenchConstants;
        return NEUTRAL_US + (enabled ? direction : 0) *
            (INVERTED ? -JOG_OFFSET_US : JOG_OFFSET_US);
    }
};
