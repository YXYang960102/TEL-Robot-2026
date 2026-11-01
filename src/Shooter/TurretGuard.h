#pragma once

#include <math.h>
#include <stdint.h>

#include "../Constants/TurretAngleConstants.h"
#include "../Control/PidfController.h"

// The last stop before the turret motor (2026-10-31, Jeremy): every turret
// command -- manual jog, camera follow, predicted position, the hand turn
// in the autos -- goes through filter(), which
//   - locks the turret (command 0) while its absolute angle is unknown, the
//     AS5600 is unhealthy, or a direction fault was seen;
//   - drops the part of a command that would go past the soft limits
//     (TurretAngleConstants::LIMIT_*), and slows it near them;
//   - after beginReturn() (first Enable after power-up, RB) drives the
//     turret back to 0 deg itself, ignoring the command;
//   - locks the turret if a held command turns it the opposite way.
// Angles: + = right, from the home point. No Arduino calls (time comes in),
// so it can be checked on a PC.
class TurretGuard {
public:
    // Reported state (TEL slot 49, radio aim frame bits 21-23).
    enum State : uint8_t {
        UNKNOWN = 0,     // no absolute angle: locked (set the home point)
        SENSOR = 1,      // AS5600 not healthy: locked
        DIRECTION = 2,   // a command turned the turret the wrong way: locked until reboot
        RETURNING = 3,   // driving back to 0 deg
        READY = 4,
        AT_RIGHT = 5,    // ready, at the right limit
        AT_LEFT = 6,     // ready, at the left limit
        OFF = 7,         // guard switched off (TurretAngleConstants::GUARD_ENABLED)
    };

    struct Inputs {
        bool absolute;     // TurretAngle::isAbsolute()
        bool healthy;      // TurretAngle::isHealthy()
        double angleDeg;   // TurretAngle::angleDegrees()
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

    // Outputs stopped (disarm): a return not finished starts over on the
    // next Enable; the direction check forgets its window.
    void onDisabled() {
        if (returning) returnStarted = false;
        windowActive = false;
    }

    bool isReturning() const { return returning; }
    State state() const { return current; }
    // Camera-follow may report "locked on" only while the guard isn't
    // overriding or holding the command.
    bool allowsLock() const { return current == READY || current == AT_RIGHT || current == AT_LEFT; }

    // command: -1..1 as it would go to the motor. Returns what may go.
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

    // Soft limits, as a pure function: + = right in degrees, command sign
    // per RIGHT_COMMAND_SIGN. Example: at +33.5 deg (5 deg from +38.5), a
    // full right command 1.0 becomes 5 / 10 = 0.5; at +38.5 or beyond, 0; any
    // left command passes unchanged.
    static double limit(double command, double angleDeg) {
        using namespace TurretAngleConstants;
        const double rightward = command * RIGHT_COMMAND_SIGN; // > 0: turning right
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
    // Turret speed, deg/s (+ = right), from successive angles, smoothed.
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

    // Where the turret would stop if the command were dropped now: the
    // present angle plus LIMIT_STOP_LOOKAHEAD_S of its present speed, only
    // in the commanded direction. Example: at +28 deg moving right at 60
    // deg/s -> +37 (limit +38.5): a full right command is cut to 1.5 / 10
    // = 0.15, raised to LIMIT_SLOW_MIN_COMMAND (0.2).
    double stoppingAngle(double command) const {
        using namespace TurretAngleConstants;
        const double rightward = command * RIGHT_COMMAND_SIGN;
        const double coast = speedDegPerS * LIMIT_STOP_LOOKAHEAD_S;
        if (rightward > 0.0 && coast > 0.0) return lastAngle + coast;
        if (rightward < 0.0 && coast < 0.0) return lastAngle + coast;
        return lastAngle;
    }

    // P(IDF) toward 0 deg in the right-positive frame, then to motor sign.
    // Example: at +36 deg: error -36 -> 0.03 * -36 = -1.08 -> clamped -0.4
    // (turn left) -> motor command -0.4 * RIGHT_COMMAND_SIGN = +0.4.
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
        // Outside the tolerance, never less than what still turns the
        // turret. Example: at +5.3 deg, 0.03 x -5.3 = -0.16 -> -0.2.
        if (!within && fabs(towardRight) < RETURN_MIN_COMMAND) {
            towardRight = in.angleDeg > 0.0 ? -RETURN_MIN_COMMAND : RETURN_MIN_COMMAND;
        }
        return towardRight * RIGHT_COMMAND_SIGN;
    }

    // A command held one way (|command| >= MIN) must not move the turret
    // DIRECTION_CHECK_WRONG_DEG the other way. After the direction changes
    // there is a grace period (the turret may still coast the old way);
    // then the reference angle is re-taken every window.
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
        const double moved = (in.angleDeg - windowStartDeg) * windowSign; // + = the commanded way
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
