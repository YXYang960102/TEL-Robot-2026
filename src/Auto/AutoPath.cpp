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
// Each move runs its *_MS time (tuned on the floor, 2026-11-14).
uint8_t AutoPath::buildMoves(uint8_t p, Move* out) {
    const int8_t side = (p == 3 || p == 4) ? -1 : 1; // + = right
    if (p == 1 || p == 3) {
        out[0] = {1, side, DIAGONAL_TURN_MS};
        out[1] = {0, 1, DIAGONAL_DRIVE_MS};
        out[2] = {1, static_cast<int8_t>(-side), DIAGONAL_TURN_BACK_MS};
        return 3;
    }
    if (p == 2 || p == 4) {
        out[0] = {0, 1, FORWARD_DRIVE_MS};
        out[1] = {1, side, SIDE_TURN_MS};
        out[2] = {0, 1, SIDE_DRIVE_MS};
        out[3] = {1, static_cast<int8_t>(-side), SIDE_TURN_BACK_MS};
        return 4;
    }
    return 0;
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
    const unsigned long runMs = move.ms;
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
            left = right = move.direction * DRIVE_COMMAND;
        } else {
            // +1 = turn right. RIGHT_TURN_LEFT_WHEEL_BACK: left back, right
            // forward (the stick's polarity since 2026-11-08).
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

// The turret does not follow the camera while any path runs (2026-11-14,
// Jeremy: lock only once the robot has stopped), and after a right path
// not until the hole has been seen (turretHold).
bool AutoPath::holdsTurret() {
    return isActive() || (turretHold && state == State::DONE);
}
