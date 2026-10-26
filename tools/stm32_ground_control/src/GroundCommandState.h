#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RcControlMap.h"

// Ground-station side of wireless control: turns the PC Dashboard's line
// protocol (the same one the Mega speaks over USB: E, X, K, T,<l>,<r>,
// ROT,<d>, ANG,<d>, TGT,<0..9>, AUTO,<1..4>, MODE,<0..3>, FLY,<0..1>, F, M;
// ANGT is accepted but has no radio channel) into the RC channel state
// (rcmap_command, table v2) that goes out over ELRS, and turns the robot's
// status coming back (rcmap_status in Battery frames) into lines for the PC.
//
//   T,<l>,<r>   -> forward = (l + r) / 2, turn = (r - l) / 2 (CH1/CH2)
//   ROT / ANG   -> nudge (CH7), held while non-zero; elevation wins if both
//   F           -> fire (CH5) high for FIRE_PULSE_MS
//   TGT,<n>     -> target hole (CH3), kept until changed or the next E
//   MODE,<n>    -> operator mode (CH4) straight to that level; 3 = TEST:
//                  CH4 full manual (with its Falcon) + CH9 high
//   FLY,<f>     -> Falcon 0..1, on CH4 with the mode (full manual and
//                  test only; a mode change, E, X and a PC timeout set it
//                  to 0)
//   FLYO,<n>    -> the autos' flywheel offset -20..20 %, on CH4 with the
//                  auto level (same resets; dropped in full manual)
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
    // The robot enables only on a low->high edge of CH6 (2026-10-17). If an
    // E arrives while CH6 is still high (the robot disarmed itself, but the
    // ground kept holding the arm channel), CH6 first goes low for this long
    // so the low actually crosses the radio (CH6 is round-robin) and the
    // bridge's 20 ms forwarding, then high. Without it that Enable never
    // confirmed ("no confirmation") and only a second press worked.
    static constexpr uint32_t ARM_LOW_MS = 200;
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
    // E received and not released since. CH6 (command().armed) follows it,
    // up to ARM_LOW_MS later.
    bool isEnabled() const { return enabled; }
    // Bumped on every E and every release of an enable: the robot status
    // received before it says nothing about the new state.
    uint32_t enableEpoch() const { return epoch; }

private:
    void processLine(uint32_t nowMs, GroundEmitFn emit, void* ctx);
    static void emitModeEvent(uint8_t mode, GroundEmitFn emit, void* ctx);
    void release(uint32_t nowMs);
    void raiseArmIfReady(uint32_t nowMs);
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
    bool enabled = false;
    uint32_t armLowSinceMs = 0;
    uint32_t epoch = 0;
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
    // 2 full manual, 3 test, -1 not reported.
    // Then "$AIM,<fresh>,<distanceCm>,<elevationPulseUs>,<selectedId>,
    // <targetId>,<valid>,<aimOffsetPx>" from the latest aim frame (fresh 0
    // and zeros when none came within STALE_MS).
    // Then "$VISION,<fresh>,<flywheelUs>,<tx*2>,<ty*2>,<valid>,<targetId>"
    // from the latest vision frame (tx / ty in half pixels; the Falcon's
    // actual PWM pulse), same rule.
    void tick(uint32_t nowMs, GroundEmitFn emit, void* ctx);

    bool fresh(uint32_t nowMs) const;
    // The last status predates an E / release: report "not fresh" until the
    // next status arrives, so a page can't confirm (or doubt) an enable
    // from the previous one.
    void invalidate() { invalidated = true; }
    uint8_t flags() const { return last.flags; }

private:
    rcmap_status last{};
    bool haveStatus = false;
    bool invalidated = false;
    uint32_t lastStatusMs = 0;
    rcmap_aim lastAim{};
    bool haveAim = false;
    uint32_t lastAimMs = 0;
    rcmap_vision lastVision{};
    bool haveVision = false;
    uint32_t lastVisionMs = 0;
    uint32_t lastReportMs = 0;
};
