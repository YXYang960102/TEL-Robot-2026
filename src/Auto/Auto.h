#pragma once

// Semi-auto / full-auto shooting (OperatorMode.h). Runs right after
// ManualControl::update() each loop, and only while an operator has the
// robot armed in SEMI_AUTO or FULL_AUTO: ManualControl keeps safety
// (Enable, watchdogs, link loss), the chassis, elevation and conveyor;
// this class drives the turret (camera vision-follow), the flywheel (shot
// table by distance) and decides when to fire the feed tower.
//
// Elevation: from the shot table too (measured servo pulses), once
// TuningConstants::ShotTable::ELEVATION_MEASURED is set; until then it
// stays with the operator.
//
//   SPINNING_UP --(turret locked && flywheel up)--> READY
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
};
