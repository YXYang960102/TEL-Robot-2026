#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RcControlMap.h"

// Ground-station side of wireless control: turns the PC Dashboard's line
// protocol (the same one the Mega speaks over USB: E, X, K, T,<l>,<r>,
// ROT,<d>, ANG,<d>, TGT,<0..9>, AUTO,<1..4>, MODE,<0..2>, FLY,<0..1>, F, M;
// ANGT is accepted but has no radio channel) into the RC channel state
// (rcmap_command, table v2) that goes out over ELRS, and turns the robot's
// status coming back (rcmap_status in Battery frames) into lines for the PC.
//
//   T,<l>,<r>   -> forward = (l + r) / 2, turn = (r - l) / 2 (CH1/CH2)
//   ROT / ANG   -> nudge (CH7), held while non-zero; elevation wins if both
//   F           -> fire (CH5) high for FIRE_PULSE_MS
//   TGT,<n>     -> target hole (CH3), kept until changed or the next E
//   MODE,<n>    -> operator mode (CH4) straight to that level
//   FLY,<f>     -> Falcon 0..1, on CH4 with the mode (full manual only;
//                  a mode change, E, X and a PC timeout set it to 0)
//   AUTO,<n>    -> auto path (CH8) for AUTO_PULSE_MS, then none
//
// No Arduino here, so it runs unchanged in the host test.
using GroundEmitFn = void (*)(void* ctx, const char* line);

class GroundCommandState {
public:
    // Two-stage PC watchdog (2026-10-17). The Dashboard sends a line at
    // least every 75 ms, but page / USB hiccups of 0.5 s+ kept releasing the
    // enable mid-match (it was one 500 ms stage, 250 ms before that).
    //   PC_HOLD_MS:    motion to neutral (drive, nudge, flywheel), enable,
    //                  mode and target kept; the next line resumes, and the
    //                  Dashboard resends every non-neutral output each tick.
    //   PC_TIMEOUT_MS: release the enable, as before.
    // A dead page now stops the robot sooner (300 ms) and disarms at 1.5 s.
    static constexpr uint32_t PC_HOLD_MS = 300;
    static constexpr uint32_t PC_TIMEOUT_MS = 1500;
    static constexpr size_t MAX_LINE_LENGTH = 24;
    // The robot acts on the rising edge; holding the level this long makes
    // sure it is seen (CH5 is in every packet; CH8 is round-robin).
    static constexpr uint32_t FIRE_PULSE_MS = 100;
    static constexpr uint32_t AUTO_PULSE_MS = 200;

    void reset(uint32_t nowMs);
    void rxByte(char c, uint32_t nowMs, GroundEmitFn emit, void* ctx);
    // Watchdog, while an enable is requested: no line from the PC for
    // PC_HOLD_MS -> motion neutral ("$GROUND_EVENT,PC_HOLD"; the next line
    // prints "$GROUND_EVENT,PC_RESUMED,<gap ms>"); for PC_TIMEOUT_MS ->
    // release the arm channel and zero everything. Also ends the fire /
    // auto-path pulses.
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    const rcmap_command& command() const { return cmd; }

private:
    void processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx);
    static void emitModeEvent(uint8_t mode, GroundEmitFn emit, void* ctx);
    void release();
    void holdMotion();
    void notePcLine(uint32_t nowMs, GroundEmitFn emit, void* ctx);

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
    bool pcHold = false;
};

class RobotStatusTracker {
public:
    // 3 s (was 1 s): the ELRS telemetry slots are sparse (default ratio
    // about 1:64), and status / aim share them, so status can be ~1 s apart
    // on the bench (2026-10-17). Display only -- the robot disarms itself on
    // RC loss regardless.
    static constexpr uint32_t STALE_MS = 3000;
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
