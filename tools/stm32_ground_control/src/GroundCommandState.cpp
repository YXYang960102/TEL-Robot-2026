#include "GroundCommandState.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace {

bool parseValue(const char* text, float minimum, float maximum, float& out) {
    char* end = nullptr;
    const double parsed = strtod(text, &end);
    // Same rule as the Mega: nan/inf spellings and overflow are rejected,
    // not clamped (clamping can't remove NaN).
    if (end == text || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    float v = static_cast<float>(parsed);
    out = v < minimum ? minimum : (v > maximum ? maximum : v);
    return true;
}

int8_t toDirection(float v) {
    return v > 0.5f ? 1 : (v < -0.5f ? -1 : 0);
}

void emitLine(GroundEmitFn emit, void* ctx, const char* text) {
    if (emit) emit(ctx, text);
}

}  // namespace

void GroundCommandState::reset(uint32_t nowMs) {
    cmd = rcmap_command{};
    cmd.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    rotate = 0;
    angle = 0;
    length = 0;
    discarding = false;
    lastPcMs = nowMs;
    pcHold = false;
    enabled = false;
    armLowSinceMs = nowMs - ARM_LOW_MS; // CH6 counts as low long enough
}

void GroundCommandState::release(uint32_t nowMs) {
    // Arm off and every motion neutral, fire / auto released. The selected
    // target hole is a choice, not a motion, so it is kept until the next E.
    if (enabled) epoch++;
    enabled = false;
    if (cmd.armed) armLowSinceMs = nowMs;
    cmd.armed = false;
    cmd.forward = 0.0f;
    cmd.turn = 0.0f;
    cmd.fire = false;
    cmd.auto_path = RCMAP_AUTO_NONE;
    rotate = 0;
    angle = 0;
    updateNudge();
    // Back to full manual, flywheel off: the robot refuses an Enable whose
    // CH4 isn't exactly that, and every release ends any auto mode.
    cmd.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    cmd.flywheel = 0.0f;
    cmd.flywheel_offset_pct = 0;
    pcHold = false;
}

void GroundCommandState::raiseArmIfReady(uint32_t nowMs) {
    if (enabled && !cmd.armed && nowMs - armLowSinceMs >= ARM_LOW_MS) {
        cmd.armed = true;
    }
}

void GroundCommandState::holdMotion() {
    // Stage 1 of the PC watchdog: everything that moves goes neutral; the
    // enable, mode and target stay. Fire / auto pulses end on their own.
    cmd.forward = 0.0f;
    cmd.turn = 0.0f;
    cmd.flywheel = 0.0f;
    rotate = 0;
    angle = 0;
    updateNudge();
    pcHold = true;
}

void GroundCommandState::notePcLine(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (pcHold) {
        // How long the PC was silent: the evidence for page vs USB stalls.
        char text[48];
        snprintf(text, sizeof(text), "$GROUND_EVENT,PC_RESUMED,%lu",
                 static_cast<unsigned long>(nowMs - lastPcMs));
        emitLine(emit, ctx, text);
        pcHold = false;
    }
    lastPcMs = nowMs;
}

void GroundCommandState::updateNudge() {
    // One direction at a time on the radio; elevation wins if both are held.
    cmd.nudge = angle > 0 ? RCMAP_NUDGE_UP
              : angle < 0 ? RCMAP_NUDGE_DOWN
              : rotate < 0 ? RCMAP_NUDGE_LEFT
              : rotate > 0 ? RCMAP_NUDGE_RIGHT
                           : RCMAP_NUDGE_CENTER;
}

void GroundCommandState::rxByte(char c, uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (c == '\r') return;
    if (c == '\n') {
        if (discarding) {
            discarding = false;
        } else {
            line[length] = '\0';
            processLine(nowMs, emit, ctx);
        }
        length = 0;
        return;
    }
    if (discarding) return;
    if (length + 1 < MAX_LINE_LENGTH) {
        line[length++] = c;
        return;
    }
    // Oversized line: drop it through the next '\n' (its tail must never
    // be parsed as a command), and treat a corrupt PC stream as lost.
    length = 0;
    discarding = true;
    if (enabled) {
        release(nowMs);
        emitLine(emit, ctx, "$GROUND_EVENT,RELEASED_CORRUPT_COMMAND");
    }
    emitLine(emit, ctx, "$GROUND_EVENT,COMMAND_TOO_LONG");
}

void GroundCommandState::processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (line[0] == '\0') return;

    if (strcmp(line, "E") == 0) {
        release(nowMs);
        // Every Enable starts on the Orin's own pick, like the robot (which
        // resets it too -- but would take this held CH3 straight back).
        cmd.target = RCMAP_TARGET_NONE;
        enabled = true;
        epoch++;
        // CH6 high now if it has been low long enough, else from tick().
        raiseArmIfReady(nowMs);
        lastPcMs = nowMs;
        emitLine(emit, ctx, "$GROUND_EVENT,ENABLE_REQUESTED");
        return;
    }
    if (strcmp(line, "X") == 0) {
        release(nowMs);
        emitLine(emit, ctx, "$GROUND_EVENT,ENABLE_RELEASED");
        return;
    }
    if (strcmp(line, "K") == 0) {
        notePcLine(nowMs, emit, ctx);
        return;
    }

    const bool isTank = strncmp(line, "T,", 2) == 0;
    const bool isRotate = strncmp(line, "ROT,", 4) == 0;
    const bool isAngle = strncmp(line, "ANG,", 4) == 0;
    // FLY,<0..1>: the Falcon slider, sent on CH4 with the mode (full
    // manual only; see RcControlMap.h).
    const bool isFlywheel = strncmp(line, "FLY,", 4) == 0;
    // No radio channel: accepted so the Dashboard's heartbeat keeps the
    // link alive, but nothing is sent for it.
    const bool isNoChannel = strncmp(line, "ANGT,", 5) == 0;
    const bool isTarget = strncmp(line, "TGT,", 4) == 0;
    const bool isAutoPath = strncmp(line, "AUTO,", 5) == 0;
    const bool isModeSet = strncmp(line, "MODE,", 5) == 0;
    // FLYO,<-20..20>: the autos' flywheel offset (LT / RT), on CH4 with the
    // auto level; dropped in full manual and on every mode change.
    const bool isFlywheelOffset = strncmp(line, "FLYO,", 5) == 0;
    const bool isSingle = line[1] == '\0' && (line[0] == 'F' || line[0] == 'M');
    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isNoChannel && !isTarget &&
        !isAutoPath && !isModeSet && !isFlywheelOffset && !isSingle) {
        emitLine(emit, ctx, "$GROUND_EVENT,UNKNOWN_COMMAND");
        return;
    }
    if (!enabled) {
        // Nothing to send without an enable request; the robot would refuse
        // it anyway.
        emitLine(emit, ctx, "$GROUND_EVENT,REJECTED_NOT_ENABLED");
        return;
    }

    if (isTank) {
        char* comma = strchr(line + 2, ',');
        if (comma == nullptr || strchr(comma + 1, ',') != nullptr) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        *comma = '\0';
        float left = 0.0f;
        float right = 0.0f;
        if (!parseValue(line + 2, -1.0f, 1.0f, left) ||
            !parseValue(comma + 1, -1.0f, 1.0f, right)) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        notePcLine(nowMs, emit, ctx);
        cmd.forward = (left + right) / 2.0f;
        cmd.turn = (right - left) / 2.0f;
        return;
    }
    if (isFlywheel) {
        float v = 0.0f;
        if (!parseValue(line + 4, 0.0f, 1.0f, v)) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        notePcLine(nowMs, emit, ctx);
        // Kept only in full manual (the robot ignores it otherwise, and a
        // mode change starts the flywheel from 0 on both ends).
        cmd.flywheel = rcmap_mode_from_us(cmd.mode_us) == RCMAP_MODE_FULL_MANUAL ? v : 0.0f;
        return;
    }
    if (isRotate || isAngle) {
        float v = 0.0f;
        if (!parseValue(line + 4, -1.0f, 1.0f, v)) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        notePcLine(nowMs, emit, ctx);
        if (isRotate) rotate = toDirection(v);
        else angle = toDirection(v);
        updateNudge();
        return;
    }
    if (isTarget || isAutoPath || isModeSet || isFlywheelOffset) {
        const char* digits = line + (isTarget ? 4 : 5);
        char* end = nullptr;
        const long n = strtol(digits, &end, 10);
        const long minimum = isAutoPath ? 1 : isFlywheelOffset ? -RCMAP_AUTO_OFFSET_MAX_PCT : 0;
        const long maximum = isTarget ? RCMAP_TARGET_MAX
                           : isAutoPath ? RCMAP_AUTO_PATHS
                           : isFlywheelOffset ? RCMAP_AUTO_OFFSET_MAX_PCT
                                        : RCMAP_MODE_FULL_MANUAL;
        if (end == digits || *end != '\0' || n < minimum || n > maximum) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        notePcLine(nowMs, emit, ctx);
        if (isFlywheelOffset) {
            cmd.flywheel_offset_pct = rcmap_mode_from_us(cmd.mode_us) == RCMAP_MODE_FULL_MANUAL
                ? 0 : static_cast<int8_t>(n);
        } else if (isTarget) {
            cmd.target = static_cast<uint8_t>(n);
        } else if (isModeSet) {
            // The Dashboard's mode menu: CH4 straight to that level.
            const uint8_t mode = static_cast<uint8_t>(n);
            if (mode != rcmap_mode_from_us(cmd.mode_us)) {
                cmd.flywheel = 0.0f;
                cmd.flywheel_offset_pct = 0;
            }
            cmd.mode_us = rcmap_mode_to_us(mode);
            emitModeEvent(mode, emit, ctx);
        } else if (cmd.auto_path == RCMAP_AUTO_NONE) {
            // A new press only after the previous pulse ended (the robot
            // needs to see "none" between presses).
            cmd.auto_path = static_cast<uint8_t>(n);
            autoStartMs = nowMs;
        }
        return;
    }

    notePcLine(nowMs, emit, ctx);
    if (isNoChannel) {
        return;
    }
    if (line[0] == 'F') {
        if (!cmd.fire) {
            cmd.fire = true;
            fireStartMs = nowMs;
        }
    } else {
        // Same cycle as the Mega's wired M: manual -> semi -> full -> manual.
        const uint8_t mode = rcmap_mode_from_us(cmd.mode_us);
        const uint8_t next = mode == RCMAP_MODE_FULL_MANUAL ? RCMAP_MODE_SEMI_AUTO
                           : mode == RCMAP_MODE_SEMI_AUTO   ? RCMAP_MODE_FULL_AUTO
                                                            : RCMAP_MODE_FULL_MANUAL;
        cmd.mode_us = rcmap_mode_to_us(next);
        cmd.flywheel = 0.0f;
        cmd.flywheel_offset_pct = 0;
        emitModeEvent(next, emit, ctx);
    }
}

void GroundCommandState::emitModeEvent(uint8_t mode, GroundEmitFn emit, void* ctx) {
    emitLine(emit, ctx, mode == RCMAP_MODE_FULL_MANUAL ? "$GROUND_EVENT,MODE_FULL_MANUAL"
                      : mode == RCMAP_MODE_SEMI_AUTO   ? "$GROUND_EVENT,MODE_SEMI_AUTO"
                                                       : "$GROUND_EVENT,MODE_FULL_AUTO");
}

void GroundCommandState::tick(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (cmd.fire && nowMs - fireStartMs >= FIRE_PULSE_MS) {
        cmd.fire = false;
    }
    if (cmd.auto_path != RCMAP_AUTO_NONE && nowMs - autoStartMs >= AUTO_PULSE_MS) {
        cmd.auto_path = RCMAP_AUTO_NONE;
    }
    raiseArmIfReady(nowMs);
    if (enabled && nowMs - lastPcMs > PC_TIMEOUT_MS) {
        release(nowMs);
        emitLine(emit, ctx, "$GROUND_EVENT,PC_TIMEOUT");
    } else if (enabled && !pcHold && nowMs - lastPcMs > PC_HOLD_MS) {
        holdMotion();
        emitLine(emit, ctx, "$GROUND_EVENT,PC_HOLD");
    }
}

void RobotStatusTracker::reset(uint32_t nowMs) {
    haveStatus = false;
    last = rcmap_status{};
    lastStatusMs = nowMs;
    haveAim = false;
    lastAim = rcmap_aim{};
    lastAimMs = nowMs;
    haveVision = false;
    lastVision = rcmap_vision{};
    lastVisionMs = nowMs;
    lastReportMs = nowMs;
}

void RobotStatusTracker::onStatus(const rcmap_status& s, uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    rcmap_vision vision;
    if (rcmap_vision_from_status(&s, &vision)) {
        lastVision = vision;
        haveVision = true;
        lastVisionMs = nowMs;
        return;
    }
    rcmap_aim aim;
    if (rcmap_aim_from_status(&s, &aim)) {
        lastAim = aim;
        haveAim = true;
        lastAimMs = nowMs;
        return;
    }
    // A status whose event isn't a known event code is corrupt (seen as
    // UNKNOWN_EVENT_CODE on the 2026-10-17 bench): drop the whole frame
    // rather than trust its flags or report a bogus event.
    if (rcmap_status_code_event(s.event_code) >= RCMAP_EVENT_COUNT) {
        return;
    }
    const bool newEvent = haveStatus && s.event_seq != last.event_seq;
    last = s;
    haveStatus = true;
    invalidated = false;
    lastStatusMs = nowMs;
    const uint16_t event = rcmap_status_code_event(s.event_code);
    if (newEvent && event != RCMAP_EVENT_NONE) {
        char text[64];
        snprintf(text, sizeof(text), "$EVENT,%s", rcmap_event_name(event));
        emitLine(emit, ctx, text);
    }
}

bool RobotStatusTracker::fresh(uint32_t nowMs) const {
    return haveStatus && !invalidated && nowMs - lastStatusMs <= STALE_MS;
}

void RobotStatusTracker::tick(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (nowMs - lastReportMs < REPORT_PERIOD_MS) return;
    lastReportMs = nowMs;
    const bool isFresh = fresh(nowMs);
    const uint8_t f = isFresh ? last.flags : 0;
    const int owner = (f & RCMAP_STATUS_OWNER_WIRED) ? 1 : ((f & RCMAP_STATUS_OWNER_WIRELESS) ? 2 : 0);
    // Operator mode: 0 full auto, 1 semi, 2 full manual, -1 not reported.
    const uint8_t mode = isFresh ? rcmap_status_code_mode(last.event_code) : RCMAP_MODE_UNKNOWN;
    char text[96];
    snprintf(text, sizeof(text), "$WIRELESS,%d,%d,%d,%d,%d,%d,%d,%lu,%d",
             isFresh ? 1 : 0,
             (f & RCMAP_STATUS_ARMED) ? 1 : 0,
             owner,
             (f & RCMAP_STATUS_PINS_CONFIRMED) ? 1 : 0,
             (f & RCMAP_STATUS_FEED_BUSY) ? 1 : 0,
             (f & RCMAP_STATUS_ROTATE_MANUAL) ? 1 : 0,
             (f & RCMAP_STATUS_RC_LINK_OK) ? 1 : 0,
             haveStatus ? static_cast<unsigned long>(nowMs - lastStatusMs) : 0UL,
             mode == RCMAP_MODE_UNKNOWN ? -1 : static_cast<int>(mode));
    emitLine(emit, ctx, text);

    const bool aimFresh = haveAim && nowMs - lastAimMs <= STALE_MS;
    const rcmap_aim a = aimFresh ? lastAim : rcmap_aim{};
    snprintf(text, sizeof(text), "$AIM,%d,%u,%u,%u,%u,%d,%d",
             aimFresh ? 1 : 0,
             static_cast<unsigned>(a.distance_cm),
             static_cast<unsigned>(a.elevation_pulse_us),
             static_cast<unsigned>(a.selected_id),
             static_cast<unsigned>(a.target_id),
             a.valid ? 1 : 0,
             static_cast<int>(a.aim_offset_px));
    emitLine(emit, ctx, text);

    // tx / ty in half pixels (no float printf on this board): the page
    // divides by 2.
    const bool visionFresh = haveVision && nowMs - lastVisionMs <= STALE_MS;
    const rcmap_vision v = visionFresh ? lastVision : rcmap_vision{};
    snprintf(text, sizeof(text), "$VISION,%d,%u,%d,%d,%d,%u",
             visionFresh ? 1 : 0,
             static_cast<unsigned>(v.flywheel_pulse_us),
             static_cast<int>(lroundf(v.tx_px * 2.0f)),
             static_cast<int>(lroundf(v.ty_px * 2.0f)),
             v.valid ? 1 : 0,
             static_cast<unsigned>(v.target_id));
    emitLine(emit, ctx, text);
}
