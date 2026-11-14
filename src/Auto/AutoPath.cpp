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

// left 1 / left 2 turn right first; right 1 / right 2 are their mirror.
uint8_t AutoPath::buildMoves(uint8_t p, Move* out) {
    const double side = (p == 3 || p == 4) ? -1.0 : 1.0; // + = right
    if (p == 1 || p == 3) {
        out[0] = {1, side * DIAGONAL_TURN_DEG};
        out[1] = {0, DIAGONAL_CM};
        out[2] = {1, -side * DIAGONAL_TURN_DEG};
        return 3;
    }
    if (p == 2 || p == 4) {
        out[0] = {0, FORWARD_CM};
        out[1] = {1, side * 90.0};
        out[2] = {0, SIDE_CM};
        out[3] = {1, -side * 90.0};
        return 4;
    }
    return 0;
}

// Time = distance / rate. Example: left 2's 365 cm at 120 cm/s = 3042 ms.
unsigned long AutoPath::moveMs(const Move& move) {
    const double rate = move.kind == 0 ? DRIVE_CM_PER_S : TURN_DEG_PER_S;
    return static_cast<unsigned long>(fabs(move.amount) / rate * 1000.0 + 0.5);
}

bool AutoPath::update(OperatorMode mode, unsigned long nowMs, double& left, double& right) {
    left = right = 0.0;
    if (turretHold && Vision::isValid()) turretHold = false; // seen once: aim
    if (state == State::PENDING) {
        if (mode == OperatorMode::FULL_AUTO) {
            state = State::DRIVING;
            moveStartMs = nowMs;
        } else if (nowMs - requestedMs >= PENDING_MS) {
            state = State::ABORTED; // full auto never came (semi: refused)
            turretHold = false;
            return false;
        } else {
            return false;
        }
    }
    if (state != State::DRIVING) return false;
    if (mode != OperatorMode::FULL_AUTO) {
        abort(); // the operator switched away: the path stops
        return false;
    }

    // Each move: its time at the command, then PAUSE_MS standing still.
    const Move& move = moves[moveIndex];
    const unsigned long elapsed = nowMs - moveStartMs;
    const unsigned long runMs = moveMs(move);
    if (elapsed >= runMs + PAUSE_MS) {
        moveIndex++;
        moveStartMs = nowMs;
        if (moveIndex >= moveCount) {
            state = State::DONE;
            return false;
        }
        return true; // stand still this loop; the next move starts next loop
    }
    if (elapsed < runMs) {
        if (move.kind == 0) {
            const double sign = move.amount >= 0.0 ? 1.0 : -1.0;
            left = right = sign * DRIVE_COMMAND;
        } else {
            // + amount = turn right. RIGHT_TURN_LEFT_WHEEL_BACK: left back,
            // right forward (the stick's polarity since 2026-11-08).
            const double rightTurn = move.amount >= 0.0 ? 1.0 : -1.0;
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
    return turretHold && state != State::ABORTED;
}
