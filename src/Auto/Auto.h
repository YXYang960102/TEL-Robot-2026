#pragma once

// Orchestrates behavior that spans more than one subsystem: which
// OperatorMode-driven mode Rotate should be in, and the auto-fire sequence
// (flywheel spin-up -> locked+spun-up -> feed -> cooldown). Kept separate
// from Shooter/Dribbler so each of those stays a single-subsystem API; this
// is the one place that decides *when* to use them together.
class Auto {
public:
    static void init();
    static void update();

private:
    enum class FireState {
        IDLE,
        SPINNING_UP,
        READY_TO_FIRE,
        FEEDING,
        COOLDOWN
    };

    static void enterFireState(FireState next);

    static FireState fireState;
    static unsigned long fireStateEnteredMs;
    // Edge-detection for the semi-auto fire-confirm button, so holding it
    // down doesn't keep re-triggering every loop.
    static bool fireButtonWasDown;
};
