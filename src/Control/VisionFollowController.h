#pragma once

#include <math.h>

#include "PidfController.h"

// Production turret vision-follow: image-space tx (px, + = target right of
// center) -> normalized turret command (-1..1), with the same control law
// Jeremy tuned on the A5 bench (VisionRotateBenchState, 2026-10-13):
// PIDF with setpoint 0 and measurement tx, I only inside kIZone, I capped
// separately. The bench works in microseconds of servo offset; production
// works in normalized command where 1.0 = 500 us, so every bench gain is
// divided by 500 (see TuningConstants::Rotate).
//
// Lock rules (also the bench's): no valid target -> stop and forget; the
// first frame of a new or switched target only re-primes (no output that
// tick, no derivative/integral carried across targets); a control gap
// longer than maxGapMs re-primes too, so a stalled loop can't dump one big
// dt into the integral.
//
// Pure logic, no Arduino, so it is host-tested directly.
struct VisionFollowConfig {
    constexpr VisionFollowConfig(
        const PidfConfig& pidfConfig,
        double maximumOutput,
        double maximumIntegral,
        double readyTolerance,
        unsigned long readySettle,
        unsigned long maximumGap)
        : pidf(pidfConfig),
          outputLimit(maximumOutput),
          integralLimit(maximumIntegral),
          readyTolerancePx(readyTolerance),
          readySettleMs(readySettle),
          maxGapMs(maximumGap) {}

    PidfConfig pidf;
    double outputLimit;
    double integralLimit;
    double readyTolerancePx;
    unsigned long readySettleMs;
    unsigned long maxGapMs;
};

class VisionFollowController {
public:
    explicit VisionFollowController(const VisionFollowConfig& followConfig)
        : config(followConfig), pidf(followConfig.pidf) {
        pidf.setOutputLimits(-config.outputLimit, config.outputLimit);
        pidf.setIntegralOutputLimits(-config.integralLimit, config.integralLimit);
    }

    void reset() {
        pidf.reset(0.0);
        haveTarget = false;
        output = 0.0;
        ready = false;
        inToleranceSince = false;
    }

    // Call every control tick. Returns the command to apply (0 when there
    // is nothing valid to follow).
    double update(double tx, bool valid, int targetId, unsigned long nowMs) {
        if (!valid || !isfinite(tx)) {
            reset();
            return 0.0;
        }
        if (!haveTarget || targetId != lastTargetId ||
            nowMs - lastUpdateMs > config.maxGapMs) {
            reset();
            pidf.reset(tx);
            haveTarget = true;
            lastTargetId = targetId;
            lastUpdateMs = nowMs;
            return 0.0;
        }

        const double dtSeconds = (nowMs - lastUpdateMs) / 1000.0;
        lastUpdateMs = nowMs;
        output = pidf.calculate(0.0, tx, dtSeconds);

        if (fabs(tx) <= config.readyTolerancePx) {
            if (!inToleranceSince) {
                inToleranceSince = true;
                toleranceStartMs = nowMs;
            }
            ready = nowMs - toleranceStartMs >= config.readySettleMs;
        } else {
            inToleranceSince = false;
            ready = false;
        }
        return output;
    }

    bool isReady() const { return ready; }
    double getOutput() const { return output; }
    double getProportionalTerm() const { return pidf.getProportionalTerm(); }
    double getIntegralTerm() const { return pidf.getIntegralTerm(); }
    bool isSaturated() const { return pidf.isSaturated(); }

private:
    VisionFollowConfig config;
    PidfController pidf;
    bool haveTarget = false;
    int lastTargetId = 0;
    unsigned long lastUpdateMs = 0;
    double output = 0.0;
    bool ready = false;
    bool inToleranceSince = false;
    unsigned long toleranceStartMs = 0;
};
