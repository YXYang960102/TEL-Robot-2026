#pragma once

#include <stdint.h>

#include "../IO/OperatorMode.h"

// The four auto paths (TuningConstants::AutoPath; 2026-11-07 design, built
// 2026-11-14): open-loop timed moves, picked in the Dashboard's Match Setup
// and requested as AUTO,n with Y. ManualControl owns it: request() when
// AUTO,n is accepted, update() every loop while armed (it hands the chassis
// over while driving), takeover / abort on the stick or a mode change,
// reset() on Enable / disarm. Auto keeps the flywheel at 0 and never fires
// while holdsShooter(), and holds the turret still while holdsTurret().
class AutoPath {
public:
    enum class State : uint8_t {
        NONE,     // no path this Enable
        PENDING,  // requested, waiting for full auto (PENDING_MS)
        DRIVING,
        DONE,     // reached the end: full auto goes on from here
        ABORTED   // mode change, stick takeover, or full auto never came
    };

    static void reset();
    static void request(uint8_t path, unsigned long nowMs);
    // One loop. Returns true while the path drives the chassis, with the
    // tank commands in left / right. State changes: see getState().
    static bool update(OperatorMode mode, unsigned long nowMs, double& left, double& right);
    static void abort();

    static State getState();
    static uint8_t getPath();
    static bool isActive();      // PENDING or DRIVING
    static bool holdsShooter();  // == isActive(): flywheel 0, no shot
    static bool holdsTurret();   // driving (any path); after a right path until the hole is seen

private:
    struct Move {
        int8_t kind;        // 0 drive, 1 turn
        int8_t direction;   // drive: +1 ahead; turn: +1 right, -1 left
        unsigned long ms;   // TuningConstants::AutoPath::*_MS
    };
    static uint8_t buildMoves(uint8_t path, Move* out);

    static State state;
    static uint8_t path;
    static unsigned long requestedMs;
    static uint8_t moveIndex;
    static uint8_t moveCount;
    static unsigned long moveStartMs;
    static bool turretHold;
    static Move moves[4];
};
