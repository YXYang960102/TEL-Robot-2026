#pragma once

#include <stdint.h>

#include "../Constants/TuningConstants.h"
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
    static const TuningConstants::AutoPath::PathConfig* config(uint8_t path);

private:
    static const TuningConstants::AutoPath::Move& currentMove();

    static State state;
    static uint8_t path;
    static unsigned long requestedMs;
    static uint8_t moveIndex;
    static uint8_t moveCount;
    static unsigned long moveStartMs;
    static bool turretHold;
};
