#include "AutoPath.h"

#include <Arduino.h>

#include "../Constants/TuningConstants.h"
#include "../Vision/Vision.h"

using namespace TuningConstants::AutoPath;

AutoPath::State AutoPath::state = AutoPath::State::NONE;
uint8_t AutoPath::path = 0;
unsigned long AutoPath::requestedMs = 0;
uint8_t AutoPath::moveIndex = 0;
uint8_t AutoPath::moveCount = 0;
unsigned long AutoPath::moveStartMs = 0;
bool AutoPath::turretHold = false;
AutoPath::Move AutoPath::moves[4] = {};

void AutoPath::reset() {
    state = State::NONE;
    path = 0;
    moveIndex = 0;
    moveCount = 0;
    turretHold = false;
}

void AutoPath::request(uint8_t newPath, unsigned long nowMs) {
    moveCount = buildMoves(newPath, moves);
    if (moveCount == 0) return;
    path = newPath;
    state = State::PENDING;
    requestedMs = nowMs;
    moveIndex = 0;
    turretHold = RIGHT_PATHS_HOLD_TURRET_UNTIL_SEEN && (newPath == 3 || newPath == 4);
}

uint8_t AutoPath::buildMoves(uint8_t p, Move* out) {
    const int8_t side = (p == 3 || p == 4) ? -1 : 1;
    Move all[4];
    uint8_t n = 0;
    if (p == 1 || p == 3) {
        all[n++] = {1, side, DIAGONAL_TURN_MS};
        all[n++] = {0, 1, DIAGONAL_DRIVE_MS};
        if (p == 1) all[n++] = {1, LEFT1_LAST_TURN_DIRECTION, LEFT1_LAST_TURN_MS};
        else        all[n++] = {1, RIGHT1_LAST_TURN_DIRECTION, RIGHT1_LAST_TURN_MS};
    } else if (p == 2 || p == 4) {
        all[n++] = {0, 1, FORWARD_DRIVE_MS};
        all[n++] = {1, side, SIDE_TURN_MS};
        all[n++] = {0, 1, SIDE_DRIVE_MS};
        all[n++] = {1, static_cast<int8_t>(-side), SIDE_TURN_BACK_MS};
    }
    uint8_t count = 0;
    for (uint8_t i = 0; i < n; i++) {
        if (all[i].ms > 0) out[count++] = all[i];
    }
    return count;
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

    const Move& move = moves[moveIndex];
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
        if (move.kind == 0) {
            left = right = move.direction * DRIVE_COMMAND;
        } else {
            const double rightTurn = move.direction;
            const double leftSign = RIGHT_TURN_LEFT_WHEEL_BACK ? -rightTurn : rightTurn;
            left = leftSign * TURN_COMMAND;
            right = -leftSign * TURN_COMMAND;
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
