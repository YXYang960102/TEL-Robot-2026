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
    length = 0;
    discarding = false;
    lastPcMs = nowMs;
}

void GroundCommandState::release() {
    // Arm off and every value neutral; event counters keep their level so
    // the robot never sees a spurious step.
    cmd.armed = false;
    cmd.conveyor = false;
    cmd.left = 0.0f;
    cmd.right = 0.0f;
    cmd.flywheel = 0.0f;
    cmd.rotate = 0;
    cmd.angle = 0;
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
    if (cmd.armed) {
        release();
        emitLine(emit, ctx, "$GROUND_EVENT,RELEASED_CORRUPT_COMMAND");
    }
    emitLine(emit, ctx, "$GROUND_EVENT,COMMAND_TOO_LONG");
}

void GroundCommandState::processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (line[0] == '\0') return;

    if (strcmp(line, "E") == 0) {
        release();
        cmd.armed = true;
        lastPcMs = nowMs;
        emitLine(emit, ctx, "$GROUND_EVENT,ENABLE_REQUESTED");
        return;
    }
    if (strcmp(line, "X") == 0) {
        release();
        emitLine(emit, ctx, "$GROUND_EVENT,ENABLE_RELEASED");
        return;
    }
    if (strcmp(line, "K") == 0) {
        lastPcMs = nowMs;
        return;
    }

    const bool isTank = strncmp(line, "T,", 2) == 0;
    const bool isRotate = strncmp(line, "ROT,", 4) == 0;
    const bool isAngle = strncmp(line, "ANG,", 4) == 0;
    const bool isFlywheel = strncmp(line, "FLY,", 4) == 0;
    const bool isSingle = line[1] == '\0' && (line[0] == 'F' || line[0] == 'L' || line[0] == 'R');
    if (!isTank && !isRotate && !isAngle && !isFlywheel && !isSingle) {
        emitLine(emit, ctx, "$GROUND_EVENT,UNKNOWN_COMMAND");
        return;
    }
    if (!cmd.armed) {
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
        lastPcMs = nowMs;
        cmd.left = left;
        cmd.right = right;
        return;
    }
    if (isRotate || isAngle || isFlywheel) {
        float v = 0.0f;
        if (!parseValue(line + 4, isFlywheel ? 0.0f : -1.0f, 1.0f, v)) {
            emitLine(emit, ctx, "$GROUND_EVENT,BAD_COMMAND");
            return;
        }
        lastPcMs = nowMs;
        if (isRotate) cmd.rotate = toDirection(v);
        else if (isAngle) cmd.angle = toDirection(v);
        else cmd.flywheel = v;
        return;
    }

    lastPcMs = nowMs;
    if (line[0] == 'R') {
        cmd.conveyor = !cmd.conveyor;
    } else if (line[0] == 'F') {
        cmd.feed_counter = static_cast<uint8_t>((cmd.feed_counter + 1u) % RCMAP_COUNTER_LEVELS);
    } else {
        cmd.lmode_counter = static_cast<uint8_t>((cmd.lmode_counter + 1u) % RCMAP_COUNTER_LEVELS);
    }
}

void GroundCommandState::tick(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (cmd.armed && nowMs - lastPcMs > PC_TIMEOUT_MS) {
        release();
        emitLine(emit, ctx, "$GROUND_EVENT,PC_TIMEOUT");
    }
}

void RobotStatusTracker::reset(uint32_t nowMs) {
    haveStatus = false;
    last = rcmap_status{};
    lastStatusMs = nowMs;
    lastReportMs = nowMs;
}

void RobotStatusTracker::onStatus(const rcmap_status& s, uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    const bool newEvent = haveStatus && s.event_seq != last.event_seq;
    last = s;
    haveStatus = true;
    lastStatusMs = nowMs;
    if (newEvent && s.event_code != RCMAP_EVENT_NONE) {
        char text[64];
        snprintf(text, sizeof(text), "$EVENT,%s", rcmap_event_name(s.event_code));
        emitLine(emit, ctx, text);
    }
}

bool RobotStatusTracker::fresh(uint32_t nowMs) const {
    return haveStatus && nowMs - lastStatusMs <= STALE_MS;
}

void RobotStatusTracker::tick(uint32_t nowMs, GroundEmitFn emit, void* ctx) {
    if (nowMs - lastReportMs < REPORT_PERIOD_MS) return;
    lastReportMs = nowMs;
    const bool isFresh = fresh(nowMs);
    const uint8_t f = isFresh ? last.flags : 0;
    const int owner = (f & RCMAP_STATUS_OWNER_WIRED) ? 1 : ((f & RCMAP_STATUS_OWNER_WIRELESS) ? 2 : 0);
    char text[96];
    snprintf(text, sizeof(text), "$WIRELESS,%d,%d,%d,%d,%d,%d,%d,%lu",
             isFresh ? 1 : 0,
             (f & RCMAP_STATUS_ARMED) ? 1 : 0,
             owner,
             (f & RCMAP_STATUS_PINS_CONFIRMED) ? 1 : 0,
             (f & RCMAP_STATUS_FEED_BUSY) ? 1 : 0,
             (f & RCMAP_STATUS_ROTATE_MANUAL) ? 1 : 0,
             (f & RCMAP_STATUS_RC_LINK_OK) ? 1 : 0,
             haveStatus ? static_cast<unsigned long>(nowMs - lastStatusMs) : 0UL);
    emitLine(emit, ctx, text);
}
