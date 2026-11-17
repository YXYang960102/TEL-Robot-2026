#pragma once

#include <math.h>

#include "PidfController.h"

struct VisionFollowConfig {
    constexpr VisionFollowConfig(
        const PidfConfig& pidfConfig,
        double maximumOutput,
        double maximumIntegral,
        double readyTolerance,
        unsigned long readySettle,
        unsigned long maximumGap,
        double frameDerivative = 0.0,
        double stallPushCommand = 0.0,
        double stallRate = 0.0,
        unsigned long stallDelay = 0,
        unsigned long stallPulse = 0,
        bool holdIntegralInTolerance = false)
        : pidf(pidfConfig),
          outputLimit(maximumOutput),
          integralLimit(maximumIntegral),
          readyTolerancePx(readyTolerance),
          readySettleMs(readySettle),
          maxGapMs(maximumGap),
          frameKd(frameDerivative),
          stallCommand(stallPushCommand),
          stallRatePxPerS(stallRate),
          stallDelayMs(stallDelay),
          stallPulseMs(stallPulse),
          integralHoldInTolerance(holdIntegralInTolerance) {}

    PidfConfig pidf;
    double outputLimit;
    double integralLimit;
    double readyTolerancePx;
    unsigned long readySettleMs;
    unsigned long maxGapMs;
    double frameKd;
    double stallCommand;
    double stallRatePxPerS;
    unsigned long stallDelayMs;
    unsigned long stallPulseMs;
    bool integralHoldInTolerance;
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
        stalled = false;
        stallPulsing = false;
        stallPushing = false;
    }

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
        derivativeTerm = -config.frameKd * txRate;
        pidf.setIntegralHold(config.integralHoldInTolerance &&
                             fabs(tx) <= config.readyTolerancePx);
        output = pidf.calculate(0.0, tx, dtSeconds) + derivativeTerm;
        if (output > config.outputLimit) output = config.outputLimit;
        if (output < -config.outputLimit) output = -config.outputLimit;

        const double toward = tx > 0.0 ? -1.0 : 1.0;
        const bool stillOutside = config.stallCommand > 0.0 && haveFrame &&
            fabs(tx) > config.readyTolerancePx && fabs(txRate) < config.stallRatePxPerS;
        if (!stillOutside) {
            stalled = false;
            stallPulsing = false;
        } else if (!stalled) {
            stalled = true;
            stallStartMs = nowMs;
        }
        if (stallPulsing && config.stallPulseMs > 0 &&
            nowMs - stallPulseStartMs >= config.stallPulseMs) {
            stallPulsing = false;
            stallStartMs = nowMs;
        }
        if (stalled && !stallPulsing && nowMs - stallStartMs >= config.stallDelayMs) {
            stallPulsing = true;
            stallPulseStartMs = nowMs;
        }
        stallPushing = stallPulsing && output * toward < config.stallCommand;
        if (stallPushing) output = toward * config.stallCommand;

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
    bool isStallPushing() const { return stallPushing; }

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
    double txRate = 0.0;
    double derivativeTerm = 0.0;
    bool stalled = false;
    unsigned long stallStartMs = 0;
    bool stallPulsing = false;
    unsigned long stallPulseStartMs = 0;
    bool stallPushing = false;

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
