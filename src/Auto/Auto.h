#pragma once

#include <stdint.h>

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
// No hole selected = the Orin's own pick. Either way the frame's id picks
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

private:
    enum class FireState {
        IDLE,
        SPINNING_UP,
        READY,
        COOLDOWN
    };

    static void enterFireState(FireState next);

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
};
