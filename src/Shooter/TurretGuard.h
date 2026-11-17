#pragma once

#include <math.h>
#include <stdint.h>

#include "../Constants/TurretAngleConstants.h"
#include "../Control/PidfController.h"

class TurretGuard {
public:
    enum State : uint8_t {
        UNKNOWN = 0,
        SENSOR = 1,
        DIRECTION = 2,
        RETURNING = 3,   // driving back to 0 deg
        READY = 4,
        AT_RIGHT = 5,
        AT_LEFT = 6,
        OFF = 7,
    };

    struct Inputs {
        bool absolute;
        bool healthy;
        double angleDeg;
        unsigned long nowMs;
    };

    TurretGuard() : returnPid(PidfConfig(TurretAngleConstants::RETURN_kP)) {
        returnPid.setOutputLimits(-TurretAngleConstants::RETURN_MAX_COMMAND,
                                  TurretAngleConstants::RETURN_MAX_COMMAND);
    }

    void beginReturn() {
        returning = true;
        returnStarted = false;
    }

    void onDisabled() {
        if (returning) returnStarted = false;
        windowActive = false;
    }

    bool isReturning() const { return returning; }
    State state() const { return current; }
    bool allowsLock() const { return current == READY || current == AT_RIGHT || current == AT_LEFT; }

    double filter(double command, const Inputs& in) {
        using namespace TurretAngleConstants;
        if (directionFault) {
            current = DIRECTION;
            return 0.0;
        }
        if (!in.absolute) {
            current = UNKNOWN;
            return 0.0;
        }
        if (!in.healthy) {
            current = SENSOR;
            windowActive = false;
            return 0.0;
        }

        trackSpeed(in);
        double out = command;
        if (returning) {
            out = returnCommand(in);
        }
        if (returning) {
            current = RETURNING;
        } else {
            current = in.angleDeg >= LIMIT_RIGHT_DEG ? AT_RIGHT
                    : in.angleDeg <= LIMIT_LEFT_DEG  ? AT_LEFT
                                                     : READY;
        }
        out = limit(out, stoppingAngle(out));
        checkDirection(out, in);
        if (directionFault) {
            current = DIRECTION;
            returning = false;
            return 0.0;
        }
        return out;
    }

    static double limit(double command, double angleDeg) {
        using namespace TurretAngleConstants;
        const double rightward = command * RIGHT_COMMAND_SIGN;
        if (rightward == 0.0) return command;
        const double room = rightward > 0.0 ? LIMIT_RIGHT_DEG - angleDeg : angleDeg - LIMIT_LEFT_DEG;
        if (room <= 0.0) return 0.0;
        if (room >= LIMIT_SLOW_ZONE_DEG) return command;
        double scale = room / LIMIT_SLOW_ZONE_DEG;
        if (scale < LIMIT_SLOW_MIN_COMMAND / fabs(command)) scale = LIMIT_SLOW_MIN_COMMAND / fabs(command);
        if (scale > 1.0) scale = 1.0;
        return command * scale;
    }

private:
    void trackSpeed(const Inputs& in) {
        if (haveLast && in.nowMs > lastMs && in.nowMs - lastMs <= 200) {
            const double dt = (in.nowMs - lastMs) / 1000.0;
            const double raw = (in.angleDeg - lastAngle) / dt;
            const double k = dt / 0.05 > 1.0 ? 1.0 : dt / 0.05;
            speedDegPerS += (raw - speedDegPerS) * k;
        } else if (!haveLast || in.nowMs - lastMs > 200) {
            speedDegPerS = 0.0;
        }
        haveLast = true;
        lastMs = in.nowMs;
        lastAngle = in.angleDeg;
    }

    double stoppingAngle(double command) const {
        using namespace TurretAngleConstants;
        const double rightward = command * RIGHT_COMMAND_SIGN;
        const double coast = speedDegPerS * LIMIT_STOP_LOOKAHEAD_S;
        if (rightward > 0.0 && coast > 0.0) return lastAngle + coast;
        if (rightward < 0.0 && coast < 0.0) return lastAngle + coast;
        return lastAngle;
    }

    double returnCommand(const Inputs& in) {
        using namespace TurretAngleConstants;
        if (!returnStarted) {
            returnStarted = true;
            returnStartMs = in.nowMs;
            settledSinceMs = 0;
            lastPidMs = in.nowMs;
            returnPid.reset(in.angleDeg);
        }
        const bool within = fabs(in.angleDeg) <= RETURN_TOLERANCE_DEG;
        if (within) {
            if (settledSinceMs == 0) settledSinceMs = in.nowMs == 0 ? 1 : in.nowMs;
        } else {
            settledSinceMs = 0;
        }
        if ((within && in.nowMs - settledSinceMs >= RETURN_SETTLE_MS) ||
            in.nowMs - returnStartMs >= RETURN_TIMEOUT_MS) {
            returning = false;
            return 0.0;
        }
        const double dt = (in.nowMs - lastPidMs) / 1000.0;
        lastPidMs = in.nowMs;
        double towardRight = returnPid.calculate(0.0, in.angleDeg, dt);
        if (!within && fabs(towardRight) < RETURN_MIN_COMMAND) {
            towardRight = in.angleDeg > 0.0 ? -RETURN_MIN_COMMAND : RETURN_MIN_COMMAND;
        }
        return towardRight * RIGHT_COMMAND_SIGN;
    }

    void checkDirection(double command, const Inputs& in) {
        using namespace TurretAngleConstants;
        const int sign = command * RIGHT_COMMAND_SIGN >= DIRECTION_CHECK_MIN_COMMAND ? 1
                       : command * RIGHT_COMMAND_SIGN <= -DIRECTION_CHECK_MIN_COMMAND ? -1 : 0;
        if (sign == 0 || !windowActive || sign != windowSign) {
            windowActive = sign != 0;
            windowSign = sign;
            windowStartMs = in.nowMs;
            refTaken = false;
            return;
        }
        if (!refTaken) {
            if (in.nowMs - windowStartMs >= DIRECTION_CHECK_GRACE_MS) {
                refTaken = true;
                windowStartMs = in.nowMs;
                windowStartDeg = in.angleDeg;
            }
            return;
        }
        const double moved = (in.angleDeg - windowStartDeg) * windowSign;
        if (moved <= -DIRECTION_CHECK_WRONG_DEG) {
            directionFault = true;
            return;
        }
        if (in.nowMs - windowStartMs >= DIRECTION_CHECK_WINDOW_MS) {
            windowStartMs = in.nowMs;
            windowStartDeg = in.angleDeg;
        }
    }

    PidfController returnPid;
    State current = UNKNOWN;
    bool returning = false;
    bool returnStarted = false;
    bool directionFault = false;
    unsigned long returnStartMs = 0;
    unsigned long settledSinceMs = 0;
    unsigned long lastPidMs = 0;
    bool windowActive = false;
    bool refTaken = false;
    int windowSign = 0;
    unsigned long windowStartMs = 0;
    double windowStartDeg = 0.0;
    bool haveLast = false;
    unsigned long lastMs = 0;
    double lastAngle = 0.0;
    double speedDegPerS = 0.0;
};
