#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RcControlMap.h"

// Ground-station side of wireless control: turns the PC Dashboard's line
// protocol (the same one the Mega speaks over USB: E, X, K, T,<l>,<r>,
// ROT,<d>, ANG,<d>, TGT,<0..9>, AUTO,<1..4>, MODE,<0..2>, F, M; FLY and ANGT are
// accepted but have no radio channel) into the RC channel state
// (rcmap_command, table v2) that goes out over ELRS, and turns the robot's
// status coming back (rcmap_status in Battery frames) into lines for the PC.
//
//   T,<l>,<r>   -> forward = (l + r) / 2, turn = (r - l) / 2 (CH1/CH2)
//   ROT / ANG   -> nudge (CH7), held while non-zero; elevation wins if both
//   F           -> fire (CH5) high for FIRE_PULSE_MS
//   TGT,<n>     -> target hole (CH3), kept until changed or the next E
//   MODE,<n>    -> operator mode (CH4) straight to that level
//   AUTO,<n>    -> auto path (CH8) for AUTO_PULSE_MS, then none
//
// No Arduino here, so it runs unchanged in the host test.
using GroundEmitFn = void (*)(void* ctx, const char* line);

class GroundCommandState {
public:
    static constexpr uint32_t PC_TIMEOUT_MS = 250;
    static constexpr size_t MAX_LINE_LENGTH = 24;
    // The robot acts on the rising edge; holding the level this long makes
    // sure it is seen (CH5 is in every packet; CH8 is round-robin).
    static constexpr uint32_t FIRE_PULSE_MS = 100;
    static constexpr uint32_t AUTO_PULSE_MS = 200;

    void reset(uint32_t nowMs);
    void rxByte(char c, uint32_t nowMs, GroundEmitFn emit, void* ctx);
    // Watchdog: no line from the PC for PC_TIMEOUT_MS while an enable is
    // requested -> release the arm channel and zero everything. Also ends
    // the fire / auto-path pulses.
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    const rcmap_command& command() const { return cmd; }

private:
    void processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx);
    static void emitModeEvent(uint8_t mode, GroundEmitFn emit, void* ctx);
    void release();

    void updateNudge();

    rcmap_command cmd{};
    int8_t rotate = 0;
    int8_t angle = 0;
    uint32_t fireStartMs = 0;
    uint32_t autoStartMs = 0;
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
    // baseline, so an old event isn't replayed on connect). An aim frame
    // (RcControlMap.h) is kept apart and never touches the status.
    void onStatus(const rcmap_status& s, uint32_t nowMs, GroundEmitFn emit, void* ctx);
    // Every REPORT_PERIOD_MS: "$WIRELESS,<fresh>,<armed>,<owner>,<pins>,
    // <feedBusy>,<rotateManual>,<rcLink>,<ageMs>,<mode>" where owner is 0
    // none, 1 wired, 2 wireless, and mode is 0 full auto, 1 semi auto,
    // 2 full manual, -1 not reported.
    // Then "$AIM,<fresh>,<distanceCm>,<elevationPulseUs>,<selectedId>,
    // <targetId>,<valid>,<aimOffsetPx>" from the latest aim frame (fresh 0
    // and zeros when none came within STALE_MS).
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    bool fresh(uint32_t nowMs) const;
    uint8_t flags() const { return last.flags; }

private:
    rcmap_status last{};
    bool haveStatus = false;
    uint32_t lastStatusMs = 0;
    rcmap_aim lastAim{};
    bool haveAim = false;
    uint32_t lastAimMs = 0;
    uint32_t lastReportMs = 0;
};
