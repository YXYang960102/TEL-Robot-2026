#include "AutoPath.h"

#include <Arduino.h>

#include "../Constants/TuningConstants.h"
#include "../IO/RcControlMap.h"
#include "../Vision/Vision.h"

using namespace TuningConstants::AutoPath;

static_assert(PATH_COUNT >= RCMAP_AUTO_PATHS, "radio paths 1..RCMAP_AUTO_PATHS, wired 1..PATH_COUNT");

AutoPath::State AutoPath::state = AutoPath::State::NONE;
uint8_t AutoPath::path = 0;
unsigned long AutoPath::requestedMs = 0;
uint8_t AutoPath::moveIndex = 0;
uint8_t AutoPath::moveCount = 0;
unsigned long AutoPath::moveStartMs = 0;
bool AutoPath::turretHold = false;

void AutoPath::reset() {
    state = State::NONE;
    path = 0;
    moveIndex = 0;
    moveCount = 0;
    turretHold = false;
}

void AutoPath::request(uint8_t newPath, unsigned long nowMs) {
    const PathConfig* c = config(newPath);
    if (c == nullptr) return;
    moveCount = 0;
    for (uint8_t i = 0; i < MAX_MOVES; i++) {
        if (c->moves[i].ms > 0) moveCount++;
    }
    if (moveCount == 0) return;
    path = newPath;
    state = State::PENDING;
    requestedMs = nowMs;
    moveIndex = 0;
    turretHold = c->holdTurretUntilSeen;
}

const PathConfig* AutoPath::config(uint8_t p) {
    return p >= 1 && p <= PATH_COUNT ? &PATHS[p - 1] : nullptr;
}

const Move& AutoPath::currentMove() {
    const PathConfig* c = config(path);
    uint8_t seen = 0;
    for (uint8_t i = 0; i < MAX_MOVES; i++) {
        if (c->moves[i].ms == 0) continue;
        if (seen++ == moveIndex) return c->moves[i];
    }
    return c->moves[0];
}

bool AutoPath::update(OperatorMode mode, unsigned long nowMs, double& left, double& right) {
    left = right = 0.0;
    if (turretHold && Vision::isValid()) turretHold = false;
    if (state == State::PENDING) {
        if (mode == OperatorMode::FULL_AUTO) {
            state = State::DRIVING;
            moveStartMs = nowMs;
        } else if (nowMs - requestedMs >= PENDING_MS) {
            state = State::ABORTED;
            turretHold = false;
            return false;
        } else {
            return false;
        }
    }
    if (state != State::DRIVING) return false;
    if (mode != OperatorMode::FULL_AUTO) {
        abort();
        return false;
    }

    const Move& move = currentMove();
    const unsigned long elapsed = nowMs - moveStartMs;
    const unsigned long runMs = move.ms;
    if (elapsed >= runMs + PAUSE_MS) {
        moveIndex++;
        moveStartMs = nowMs;
        if (moveIndex >= moveCount) {
            state = State::DONE;
            return false;
        }
        return true;
    }
    if (elapsed < runMs) {
        if (move.kind == DRIVE) {
            left = right = move.direction * move.command;
        } else {
            const double rightTurn = move.direction;
            const double leftSign = RIGHT_TURN_LEFT_WHEEL_BACK ? -rightTurn : rightTurn;
            left = leftSign * move.command;
            right = -leftSign * move.command;
        }
    }
    return true;
}

void AutoPath::abort() {
    if (state == State::PENDING || state == State::DRIVING) {
        state = State::ABORTED;
        turretHold = false;
    }
}

AutoPath::State AutoPath::getState() {
    return state;
}

uint8_t AutoPath::getPath() {
    return path;
}

bool AutoPath::isActive() {
    return state == State::PENDING || state == State::DRIVING;
}

bool AutoPath::holdsShooter() {
    return isActive();
}

bool AutoPath::holdsTurret() {
    return turretHold && isActive();
}
