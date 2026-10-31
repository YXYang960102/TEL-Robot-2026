#pragma once

#include <stdint.h>

#include "../IO/OperatorMode.h"

// Semi-auto / full-auto shooting (OperatorMode.h). Runs right after
// ManualControl::update() each loop, and only while an operator has the
// robot armed in SEMI_AUTO, FULL_AUTO or TEST (TEST: the turret's camera
// follow only, see Auto::update()): ManualControl keeps safety
// (Enable, watchdogs, link loss), the chassis and elevation;
// this class drives the turret (camera vision-follow), the flywheel (shot
// table by distance) and decides when to fire the feed tower.
//
// Target: the operator's hole (keys 1-9) in both auto modes; Vision only
// passes on that hole's frames (the Orin is told which via MEGA_TARGET).
// No hole selected = the Orin's own pick. Full auto cycle (2026-10-31,
// TuningConstants::Auto::ROW_CYCLE_ENABLED): 3 balls at a hole, then the
// next in OperatorTarget::cycleHole order (that row to the right /
// wrapping, the row below, the row above, then the bonus row), skipping
// holes without a shot table, then stop -- starting from the chosen hole,
// or from the first hole shot at when the Orin picks. A lit bonus hole
// (Vision::getLitBonusId) is served first, 3 balls (LIT_BONUS_PRIORITY).
// getAimTarget() is handed to Vision by main.cpp. Either way the frame's
// id picks
// the shot table for that hole's height (TuningConstants::ShotTable: bottom,
// middle, top, bonus) plus the hole's elevation trim. A target needs a
// depth distance (> 0) and a table to be shot at.
//
// Elevation: from the shot table too (measured servo pulses), plus the
// operator's fine-adjust, once TuningConstants::ShotTable::
// ELEVATION_MEASURED is set; until then it stays with the operator. The
// turret aims at the hole plus the operator's aim offset.
//
//   SPINNING_UP --(turret locked && flywheel up && distance)--> READY
//   READY --(FULL_AUTO, or F in SEMI_AUTO)--> fire one ball --> COOLDOWN
//   COOLDOWN --(Auto::COOLDOWN_MS)--> SPINNING_UP
//
// In FULL_MANUAL, or disarmed, it touches nothing.
class Auto {
public:
    static void init();
    static void update();

    // The steadied distance the shot table is looked up at, meters (0 =
    // none), and why Auto can't fire yet: bit 0 turret locked, 1 flywheel
    // spun up, 2 a shot for the hole in this frame, 3 READY, 4 (2026-10-31)
    // the elevation at the table's pulse. Both 0 outside
    // the autos. For the Dashboard (TEL slots 40-41).
    static double getShotDistanceM();
    static uint8_t getReadyBits();

    // The hole Auto aims at (Dashboard numbering, 0 = the Orin's pick):
    // the operator's, or in full auto the row cycle's current hole
    // (TuningConstants::Auto::ROW_CYCLE_ENABLED). main.cpp hands it to
    // Vision. Shots at that hole in this cycle, and whether the row is
    // done (full auto holds fire), for the Dashboard (TEL slots 53-55).
    static uint8_t getAimTarget();
    static uint8_t getRowShots();
    static bool isRowDone();      // the whole cycle (not one row)
    // A lit bonus hole being served first (Dashboard hole 10-12, 0 = none).
    static uint8_t getLitPriorityHole();

private:
    enum class FireState {
        IDLE,
        SPINNING_UP,
        READY,
        COOLDOWN
    };

    static void enterFireState(FireState next);
    static void updateRowCycle(OperatorMode mode);
    static void countRowShot();
    static void skipHolesWithoutTable();

    static bool rowActive;
    static uint8_t rowStartHole;
    static uint8_t rowStep;      // index into OperatorTarget::cycleHole
    static uint8_t rowShots;     // at the current hole
    static bool rowDone;
    static uint8_t lastCycleHole;
    static uint16_t rowSelectSeq;
    static bool litActive;
    static uint8_t litHole;
    static uint8_t litShots;
    static uint8_t litServedMask; // bit vision id 1-3: served, waiting to go dark
    static unsigned long litLastSeenMs[4];
    static OperatorMode rowMode;

    static FireState fireState;
    static unsigned long fireStateEnteredMs;
    // The last target's shot, kept for TuningConstants::Auto::TARGET_HOLD_MS
    // after the target drops (a one-frame vision dropout must not drop the
    // flywheel to the pre-spin and restart its spin-up wait).
    static bool haveLastShot;
    static double lastShotFlywheel;
    static double lastShotElevationUs;
    static unsigned long lastShotMs;
    static uint8_t lastShotSelection;
    static unsigned long lastElevationSlewMs;
    static int lastShotTargetId;
    static uint8_t readyBits;
};
