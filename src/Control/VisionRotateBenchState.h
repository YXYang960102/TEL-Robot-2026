#pragma once
#include <stdint.h>
#include <string.h>
#include "../Constants/VisionRotateBenchConstants.h"
#include "PidfController.h"

// Pure state machine, shared by the firmware and host regression tests.
// Direction comes from Orin vision data (onVision), never from the browser —
// the browser only arms/disarms (E/E_NO_LIMITS/X) and proves its tab is
// still alive (P), the same way the keyboard bench proves liveness via
// repeated M commands.
class VisionRotateBenchState {
public:
    bool enabled = false;
    bool noLimits = false;
    const char* stopReason = "BOOT";
    int direction = 0;
    // Signed pulse offset from NEUTRAL_US, already INVERTED-applied -- this
    // is what pulse() adds directly, not a magnitude paired with `direction`.
    int speedOffsetUs = 0;
    uint32_t lastUsbPing = 0;
    uint32_t lastVisionRx = 0;

    // Telemetry snapshot from the most recent onVision() call, zeroed
    // alongside direction/speedOffsetUs on stop/disable/invalid/target-switch
    // (2026-10-10, Codex-reviewed PIDF stabilization pass -- see
    // docs/codex-handoff.md). dtMs/pTermUs/dTermUs/outputRawUs/saturated let
    // VisionRotateTest.cpp report the actual per-sample dt and P/D
    // components instead of just the final clamped pulse, so a given
    // "kP=X" test run is reproducible from the wire data, not just a
    // compile-time constant someone might not have actually reflashed.
    uint32_t lastDtMs = 0;
    int lastPTermUs = 0;
    int lastITermUs = 0; // v5 telemetry (2026-10-13): I added inside kIZone
    int lastDTermUs = 0;
    int lastOutputRawUs = 0;
    bool lastSaturated = false;
    // Public (like direction/speedOffsetUs above) specifically so host tests
    // can verify the filter's own behavior (burst/first-sample/reset
    // handling) directly -- lastDTermUs alone can't do that whenever kD
    // happens to be tuned to 0.0 live (the project's standard starting
    // value), since -kD*filteredRate would always read 0 regardless of
    // what the filter itself is actually doing.
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
        enabled = false; direction = 0; speedOffsetUs = 0; noLimits = false; stopReason = reason;
        pidf.reset(0.0); lastPidUpdateMs = 0; // no stale integral across a re-enable
        resetRateFilter();
    }

    void applyLimits(bool left, bool right) {
        if (!noLimits && ((direction < 0 && left) || (direction > 0 && right))) stop("LIMIT");
    }

    void tick(uint32_t now) {
        using namespace VisionRotateBenchConstants;
        // Two independent watchdogs: the browser tab (USB "P" pings) and the
        // Orin vision link (Serial1 lines) must both stay alive, or we stop.
        if (enabled && uint32_t(now - lastUsbPing) > COMMAND_TIMEOUT_MS) { stop("TIMEOUT"); return; }
        if (enabled && uint32_t(now - lastVisionRx) > VISION_TIMEOUT_MS) { stop("VISION_TIMEOUT"); return; }
    }

    bool command(const char* text, uint32_t now) {
        tick(now); // A late ping/enable cannot revive already-timed-out movement.
        if (!strcmp(text, "X")) { stop(); return true; }
        if (!strcmp(text, "E") || !strcmp(text, "E_NO_LIMITS")) {
            noLimits = !strcmp(text, "E_NO_LIMITS");
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

    // Called for every decoded Serial1 line from Orin, regardless of `enabled`
    // (link liveness is independent of the motion-armed gate). Pure PIDF:
    // setpoint=0, measurement=tx, straight into PidfController (same utility
    // Shooter's Angle/Rotate/Flywheel loops use) for P(+I+FF, both 0 here).
    //
    // D is NOT PidfController's own instantaneous measurementRate -- pidf is
    // always constructed with kD=0 (see the private member below), and D is
    // computed here from a time-constant-filtered rate instead, per the
    // 2026-10-10 Codex-reviewed stabilization pass (docs/codex-handoff.md):
    // a single noisy/bursty sample must not dominate D the way an unfiltered
    // instantaneous rate would, and PidfController itself must stay
    // untouched so Shooter's Angle/Rotate/Flywheel loops keep their existing
    // unfiltered behavior.
    //
    // The deadband (previously DEADBAND_PX) was retired the same session:
    // measured oscillation never got near its 1px boundary, so it wasn't
    // the problem, and Jeremy's explicit call -- overriding Codex's "don't
    // loosen the deadband" note -- was to drop it entirely and let
    // output=kP*error(+filtered D) settle to zero on its own, FRC-bench
    // style. Whether tx=0 is actually silent depends on the servo's own
    // minimum-movable-offset floor; see the neutral-point measurement
    // procedure in docs/codex-handoff.md.
    void onVision(int tx, bool valid, int targetId, uint32_t now) {
        lastVisionRx = now;
        const bool targetChanged = valid && targetId != lastTargetIdSeen;
        if (valid) lastTargetIdSeen = targetId;

        if (!enabled || !valid || targetChanged) {
            direction = 0; speedOffsetUs = 0;
            pidf.reset(tx); lastPidUpdateMs = now; // avoid a derivative kick on re-enable/re-lock/re-target
            resetRateFilter();
            return;
        }

        const double dtSeconds = (lastPidUpdateMs == 0) ? 0.0 : (now - lastPidUpdateMs) / 1000.0;
        lastPidUpdateMs = now;

        // P(+I+FF, both 0 here) via PidfController; pidf's own kD is 0, so
        // this never contributes a D term on its own.
        pidf.calculate(0.0, (double)tx, dtSeconds);

        if (dtSeconds > 0.0) {
            // Same-tick duplicate/burst samples (dtSeconds<=0, e.g. two
            // Serial1 lines processed within one loop() iteration before
            // millis() ticks) are NOT blended in here -- that would inject
            // a bogus zero-rate sample into the filter. The filter simply
            // holds its previous value until a sample with real dt arrives.
            const double rawRate = pidf.getMeasurementRate();
            const double alpha = dtSeconds / (VisionRotateBenchConstants::kD_FILTER_TAU_S + dtSeconds);
            filteredRate = rateFilterInitialized
                ? (alpha * rawRate + (1.0 - alpha) * filteredRate)
                : rawRate; // first sample after a reset: snap to the raw rate, no fake ramp-in
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
        // PidfController uses the standard error=setpoint-measurement convention
        // (output already carries the "cancel the error" sign), which is the
        // opposite sign convention the retired direction*magnitude scheme used
        // (that one computed direction from tx's own sign, then applied INVERTED
        // only to the magnitude). Flipping once here before applying INVERTED
        // reproduces the exact same real-hardware-validated tx-to-pulse mapping
        // confirmed 2026-09-25 -- verified against that mapping with a
        // nonzero-kP sign check before trusting this, not just by inspection.
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
        0.0, // D is applied externally to a filtered rate, see onVision()
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
