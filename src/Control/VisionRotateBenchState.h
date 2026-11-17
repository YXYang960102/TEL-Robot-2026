#pragma once
#include <stdint.h>
#include <string.h>
#include "../Constants/VisionRotateBenchConstants.h"
#include "PidfController.h"

class VisionRotateBenchState {
public:
    bool enabled = false;
    const char* stopReason = "BOOT";
    int direction = 0;
    int speedOffsetUs = 0;
    uint32_t lastUsbPing = 0;
    uint32_t lastVisionRx = 0;

    uint32_t lastDtMs = 0;
    int lastPTermUs = 0;
    int lastITermUs = 0;
    int lastDTermUs = 0;
    int lastOutputRawUs = 0;
    bool lastSaturated = false;
    double filteredRate = 0.0;
    bool rateFilterInitialized = false;

    VisionRotateBenchState() {
        pidf.setOutputLimits(
            -VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US,
            VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US);
        pidf.setIntegralOutputLimits(
            -VisionRotateBenchConstants::PIDF_INTEGRAL_LIMIT_US,
            VisionRotateBenchConstants::PIDF_INTEGRAL_LIMIT_US);
    }

    void stop(const char* reason="OPERATOR") {
        enabled = false; direction = 0; speedOffsetUs = 0; stopReason = reason;
        pidf.reset(0.0); lastPidUpdateMs = 0;
        resetRateFilter();
    }

    void tick(uint32_t now) {
        using namespace VisionRotateBenchConstants;
        if (enabled && uint32_t(now - lastUsbPing) > COMMAND_TIMEOUT_MS) { stop("TIMEOUT"); return; }
        if (enabled && uint32_t(now - lastVisionRx) > VISION_TIMEOUT_MS) { stop("VISION_TIMEOUT"); return; }
    }

    bool command(const char* text, uint32_t now) {
        tick(now);
        if (!strcmp(text, "X")) { stop(); return true; }
        if (!strcmp(text, "E")) {
            stopReason = "NONE";
            direction = 0; enabled = true; lastUsbPing = now; lastVisionRx = now;
            resetRateFilter();
            return true;
        }
        if (!strcmp(text, "P")) {
            if (enabled) lastUsbPing = now;
            return true;
        }
        stop("BAD_COMMAND");
        return false;
    }

    void onVision(int tx, bool valid, int targetId, uint32_t now) {
        lastVisionRx = now;
        const bool targetChanged = valid && targetId != lastTargetIdSeen;
        if (valid) lastTargetIdSeen = targetId;

        if (!enabled || !valid || targetChanged) {
            direction = 0; speedOffsetUs = 0;
            pidf.reset(tx); lastPidUpdateMs = now;
            resetRateFilter();
            return;
        }

        const double dtSeconds = (lastPidUpdateMs == 0) ? 0.0 : (now - lastPidUpdateMs) / 1000.0;
        lastPidUpdateMs = now;

        pidf.calculate(0.0, (double)tx, dtSeconds);

        if (dtSeconds > 0.0) {
            const double rawRate = pidf.getMeasurementRate();
            const double alpha = dtSeconds / (VisionRotateBenchConstants::kD_FILTER_TAU_S + dtSeconds);
            filteredRate = rateFilterInitialized
                ? (alpha * rawRate + (1.0 - alpha) * filteredRate)
                : rawRate;
            rateFilterInitialized = true;
        }

        const double filteredDerivativeUs = -VisionRotateBenchConstants::kD * filteredRate;
        const double rawCombinedUs =
            pidf.getProportionalTerm() + pidf.getIntegralTerm() +
            pidf.getFeedforwardTerm() + filteredDerivativeUs;
        const double clampedUs = clamp(
            rawCombinedUs,
            -VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US,
            VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US);

        lastDtMs = dtSeconds > 0.0 ? (uint32_t)(dtSeconds * 1000.0 + 0.5) : 0;
        lastPTermUs = roundToInt(pidf.getProportionalTerm());
        lastITermUs = roundToInt(pidf.getIntegralTerm());
        lastDTermUs = roundToInt(filteredDerivativeUs);
        lastOutputRawUs = roundToInt(rawCombinedUs);
        lastSaturated = clampedUs != rawCombinedUs;

        const int pidOutputUs = roundToInt(clampedUs);
        speedOffsetUs = VisionRotateBenchConstants::INVERTED ? pidOutputUs : -pidOutputUs;
        direction = speedOffsetUs > 0 ? 1 : (speedOffsetUs < 0 ? -1 : 0);
    }

    int pulse() const {
        using namespace VisionRotateBenchConstants;
        return NEUTRAL_US + (enabled ? speedOffsetUs : 0);
    }

private:
    PidfController pidf{PidfConfig(
        VisionRotateBenchConstants::kP,
        VisionRotateBenchConstants::kI,
        0.0,
        VisionRotateBenchConstants::kIZone,
        VisionRotateBenchConstants::kFF)};
    uint32_t lastPidUpdateMs = 0;
    int lastTargetIdSeen = 0;

    void resetRateFilter() {
        filteredRate = 0.0;
        rateFilterInitialized = false;
        lastDtMs = 0;
        lastPTermUs = 0;
        lastITermUs = 0;
        lastDTermUs = 0;
        lastOutputRawUs = 0;
        lastSaturated = false;
    }

    static double clamp(double value, double minimum, double maximum) {
        if (value < minimum) return minimum;
        if (value > maximum) return maximum;
        return value;
    }

    static int roundToInt(double value) {
        return (int)(value >= 0 ? value + 0.5 : value - 0.5);
    }
};
