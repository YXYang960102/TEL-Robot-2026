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
// D (2026-11-01, Jeremy: the turret swung back and forth before settling):
// the loop runs every 10 ms but a new tx only comes with a camera frame
// (~33 ms), so a plain derivative would be 0, 0, then a spike. The rate is
// taken frame to frame instead -- (tx - previous frame's tx) / time between
// the frames, smoothed 50 / 50 with the last rate, held between frames --
// and the term -frameKd * rate is added to the PI output (derivative on the
// measurement: no kick when the target switches). Off (frameKd 0) unless
// the caller passes frame numbers.
//
// Pure logic, no Arduino, so it is host-tested directly.
struct VisionFollowConfig {
    constexpr VisionFollowConfig(
        const PidfConfig& pidfConfig,
        double maximumOutput,
        double maximumIntegral,
        double readyTolerance,
        unsigned long readySettle,
        unsigned long maximumGap,
        double frameDerivative = 0.0)
        : pidf(pidfConfig),
          outputLimit(maximumOutput),
          integralLimit(maximumIntegral),
          readyTolerancePx(readyTolerance),
          readySettleMs(readySettle),
          maxGapMs(maximumGap),
          frameKd(frameDerivative) {}

    PidfConfig pidf;
    double outputLimit;
    double integralLimit;
    double readyTolerancePx;
    unsigned long readySettleMs;
    unsigned long maxGapMs;
    double frameKd;   // command per (px/s), frame-to-frame rate of tx
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
        haveFrame = false;
        txRate = 0.0;
        derivativeTerm = 0.0;
    }

    // Call every control tick. Returns the command to apply (0 when there
    // is nothing valid to follow).
    // frame: the camera frame number (Vision::getFrameCount()); 0 = not
    // given (no D term).
    double update(double tx, bool valid, int targetId, unsigned long nowMs, unsigned long frame = 0) {
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
            noteFrame(tx, nowMs, frame);
            return 0.0;
        }

        const double dtSeconds = (nowMs - lastUpdateMs) / 1000.0;
        lastUpdateMs = nowMs;
        noteFrame(tx, nowMs, frame);
        // Example: tx 40 px closing at 600 px/s (rate -600), kd 0.0005:
        // PI -0.40, D -0.0005 * -600 = +0.30 -> -0.10, braking before 0.
        derivativeTerm = -config.frameKd * txRate;
        output = pidf.calculate(0.0, tx, dtSeconds) + derivativeTerm;
        if (output > config.outputLimit) output = config.outputLimit;
        if (output < -config.outputLimit) output = -config.outputLimit;

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
    double getDerivativeTerm() const { return derivativeTerm; }
    double getTxRate() const { return txRate; }

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
    bool haveFrame = false;
    unsigned long lastFrame = 0;
    double lastFrameTx = 0.0;
    unsigned long lastFrameMs = 0;
    double txRate = 0.0;           // px/s, smoothed, held between frames
    double derivativeTerm = 0.0;

    // A new camera frame: update the tx rate from the previous frame.
    void noteFrame(double tx, unsigned long nowMs, unsigned long frame) {
        if (frame == 0 || (haveFrame && frame == lastFrame)) return;
        if (haveFrame && nowMs > lastFrameMs) {
            const double raw = (tx - lastFrameTx) * 1000.0 / static_cast<double>(nowMs - lastFrameMs);
            txRate = 0.5 * txRate + 0.5 * raw;
        }
        haveFrame = true;
        lastFrame = frame;
        lastFrameTx = tx;
        lastFrameMs = nowMs;
    }
};
