#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RcControlMap.h"

// Ground-station side of wireless control: turns the PC Dashboard's line
// protocol (the same one the Mega speaks over USB: E, X, K, T,<l>,<r>,
// ROT,<d>, ANG,<d>, FLY,<0..1>, F, L, R, M) into the RC channel state
// (rcmap_command) that goes out over ELRS, and turns the robot's status
// coming back (rcmap_status in Battery frames) into lines for the PC.
//
// No Arduino here, so it runs unchanged in the host test.
using GroundEmitFn = void (*)(void* ctx, const char* line);

class GroundCommandState {
public:
    static constexpr uint32_t PC_TIMEOUT_MS = 250;
    static constexpr size_t MAX_LINE_LENGTH = 24;

    void reset(uint32_t nowMs);
    void rxByte(char c, uint32_t nowMs, GroundEmitFn emit, void* ctx);
    // Watchdog: no line from the PC for PC_TIMEOUT_MS while an enable is
    // requested -> release the arm channel and zero everything.
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    const rcmap_command& command() const { return cmd; }

private:
    void processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx);
    void release();

    rcmap_command cmd{};
    char line[MAX_LINE_LENGTH]{};
    size_t length = 0;
    bool discarding = false;
    uint32_t lastPcMs = 0;
};

class RobotStatusTracker {
public:
    static constexpr uint32_t STALE_MS = 1000;
    static constexpr uint32_t REPORT_PERIOD_MS = 250;

    void reset(uint32_t nowMs);
    // New status from a Battery frame. Emits "$EVENT,<name>" when the
    // robot's event sequence advanced (the first status only sets the
    // baseline, so an old event isn't replayed on connect).
    void onStatus(const rcmap_status& s, uint32_t nowMs, GroundEmitFn emit, void* ctx);
    // Every REPORT_PERIOD_MS: "$WIRELESS,<fresh>,<armed>,<owner>,<pins>,
    // <feedBusy>,<rotateManual>,<rcLink>,<ageMs>,<mode>" where owner is 0
    // none, 1 wired, 2 wireless, and mode is 0 full auto, 1 semi auto,
    // 2 full manual, -1 not reported.
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    bool fresh(uint32_t nowMs) const;
    uint8_t flags() const { return last.flags; }

private:
    rcmap_status last{};
    bool haveStatus = false;
    uint32_t lastStatusMs = 0;
    uint32_t lastReportMs = 0;
};
