#pragma once

#include <stdint.h>

#include "../IO/OperatorMode.h"

class AutoPath {
public:
    enum class State : uint8_t {
        NONE,
        PENDING,
        DRIVING,
        DONE,
        ABORTED
    };

    static void reset();
    static void request(uint8_t path, unsigned long nowMs);
    static bool update(OperatorMode mode, unsigned long nowMs, double& left, double& right);
    static void abort();

    static State getState();
    static uint8_t getPath();
    static bool isActive();
    static bool holdsShooter();
    static bool holdsTurret();

private:
    struct Move {
        int8_t kind;
        int8_t direction;
        unsigned long ms;
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
