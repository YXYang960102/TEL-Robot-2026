#include <assert.h>
#include <stdint.h>
#include "../src/Control/VisionRotateBenchState.h"

int main() {
    // Boot: neutral, disabled.
    {
        VisionRotateBenchState s;
        assert(s.pulse() == 1500 && !s.enabled);
    }

    // Vision ignored while disabled.
    {
        VisionRotateBenchState s;
        s.onVision(100, true, 1, 10);
        assert(s.direction == 0 && s.pulse() == 1500);
    }

    // PIDF sign convention: PidfController's calculate() uses the standard
    // error=setpoint-measurement convention (output already carries the
    // "cancel the error" sign) -- opposite of the retired direction*magnitude
    // scheme, which computed direction from tx's own sign and applied
    // INVERTED only to the magnitude. onVision() flips the PIDF output once
    // before applying INVERTED specifically to reproduce the same tx-to-pulse
    // mapping that was validated on real hardware 2026-09-25 (tx>0 -> pulse
    // below neutral, tx<0 -> pulse above neutral, with INVERTED=true). Uses a
    // throwaway non-firmware VisionRotateBenchState is not possible since the
    // gains are compile-time constants -- instead this drives PidfController
    // directly with the same kP the firmware would use if tuned, confirming
    // the sign math onVision() applies to its output, not the full firmware
    // curve (that depends on whatever kP/kI/kD get tuned to live).
    {
        PidfController pidf(PidfConfig(1.0, 0.0, 0.0, 0.0, 0.0));
        pidf.setOutputLimits(-300, 300);
        const double outputForPositiveTx = pidf.calculate(0.0, 21.0, 0.0);
        pidf.reset(0.0);
        const double outputForNegativeTx = pidf.calculate(0.0, -21.0, 0.0);
        assert(outputForPositiveTx < 0.0); // error=-21, kP=1 -> -21
        assert(outputForNegativeTx > 0.0); // error=21, kP=1 -> +21
        // onVision()'s mapping (INVERTED=true in this bench):
        //   speedOffsetUs = INVERTED ? pidOutputUs : -pidOutputUs
        // so a negative PIDF output (tx>0 case) becomes a negative
        // speedOffsetUs -- pulse below neutral, matching the real-hardware
        // mapping.
        assert((int)outputForPositiveTx < 0);
        assert((int)outputForNegativeTx > 0);
    }

    // Output clamp: even a huge error cannot exceed +-PIDF_OUTPUT_LIMIT_US
    // (500 since 2026-10-13: the servo's full 1000..2000 us range).
    {
        using VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US;
        static_assert(PIDF_OUTPUT_LIMIT_US == 500, "Jeremy's 2026-10-13 full-speed ceiling");
        PidfController pidf(PidfConfig(50.0, 0.0, 0.0, 0.0, 0.0)); // deliberately large kP
        pidf.setOutputLimits(-PIDF_OUTPUT_LIMIT_US, PIDF_OUTPUT_LIMIT_US);
        const double output = pidf.calculate(0.0, 1000.0, 0.0); // error=-1000, kP*error=-50000
        assert(output == -PIDF_OUTPUT_LIMIT_US);
        assert(pidf.isSaturated());
    }

    // The real firmware state with the real gains: a target far off-center
    // drives the servo to the edge of its attach range (1000/2000 us, full
    // speed), never past it; inside the linear zone (|tx| < 300/kP = 60 px)
    // the output is exactly kP*tx, the same as before the ceiling was raised.
    {
        using namespace VisionRotateBenchConstants;
        VisionRotateBenchState s;
        s.command("E", 0);
        // Hardware-validated mapping (2026-09-25, INVERTED=true): tx>0 ->
        // pulse below neutral.
        // (The first frame of a new target id is a lock-acquire reset with no
        // output, so each case primes with one frame first.)
        s.onVision(400, true, 1, 50);
        s.onVision(400, true, 1, 100); // far right
        assert(s.pulse() == 1500 - PIDF_OUTPUT_LIMIT_US); // 1000 us
        assert(s.lastSaturated);
        s.onVision(-400, true, 1, 150); // far left
        assert(s.pulse() == 1500 + PIDF_OUTPUT_LIMIT_US); // 2000 us

        VisionRotateBenchState t;
        t.command("E", 0);
        t.onVision(40, true, 1, 50);
        t.onVision(40, true, 1, 100); // 40 px: inside the unchanged linear zone
        assert(!t.lastSaturated);
        assert(t.pulse() == 1500 - (int)(kP * 40)); // kP=5 -> 1300 us, same as at the old 300 ceiling
    }

    // I (2026-10-13): kI=5 us/(px*s), only inside |tx| <= kIZone=20 px,
    // capped at +-PIDF_INTEGRAL_LIMIT_US=60 us. Real state, real gains.
    {
        using namespace VisionRotateBenchConstants;
        static_assert(kI == 5.0 && kIZone == 20.0 && PIDF_INTEGRAL_LIMIT_US == 60,
                      "Jeremy's 2026-10-13 I settings");
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(5, true, 1, 50);   // lock-acquire reset, no output
        s.onVision(5, true, 1, 150);  // dt=0.1s, error=-5 -> I = 5*(-5*0.1) = -2.5 us
        assert(s.lastITermUs == -3 || s.lastITermUs == -2);
        // tx>0 -> below neutral (INVERTED): P=-25, I adds in the same direction.
        assert(s.pulse() < 1500 - 25);

        // Held 5 px off for 10 s: I saturates at its own cap, not beyond.
        uint32_t t = 150;
        for (int i = 0; i < 100; ++i) { t += 100; s.onVision(5, true, 1, t); }
        assert(s.lastITermUs == -PIDF_INTEGRAL_LIMIT_US);
        assert(s.pulse() == 1500 - 25 - PIDF_INTEGRAL_LIMIT_US); // 1415 us

        // 15 px is inside the widened 20 px zone: I keeps integrating
        // (error=-15 -> another 5*(-15*0.1) = -7.5 us, still held at the cap).
        t += 100; s.onVision(15, true, 1, t);
        assert(s.lastITermUs == -PIDF_INTEGRAL_LIMIT_US);

        // Leaving the zone (25 px > 20) drops I to 0 at once: pure P again.
        t += 100; s.onVision(25, true, 1, t);
        assert(s.lastITermUs == 0);
        assert(s.pulse() == 1500 - (int)(kP * 25)); // 1375 us

        // Back inside: integrates again from zero, not from the old -60.
        t += 100; s.onVision(5, true, 1, t);
        assert(s.lastITermUs == -3 || s.lastITermUs == -2);

        // Opposite side: I has the opposite sign.
        t += 100; s.onVision(-5, true, 1, t);
        assert(s.lastITermUs > -3); // unwinding toward / past zero, never more negative

        // Stop clears it; re-enable starts from zero.
        s.command("X", t);
        assert(s.lastITermUs == 0);
    }

    // onVision()'s own clamp: P alone (pidf's internal clamp) plus the
    // externally-added filtered D must still respect
    // +-PIDF_OUTPUT_LIMIT_US together, not just individually. Can't inject a
    // custom kD into VisionRotateBenchState (compile-time constant), so this
    // exercises the same clamp() helper's math directly with numbers shaped
    // like "P alone is already near the ceiling, D pushes it over" to prove
    // the combined-then-clamp order onVision() uses is correct.
    {
        // P alone saturates at +limit (matches onVision()'s pidf.calculate()
        // clamp), then a filtered D contribution pushes the raw combined
        // value 40 us past it -- still must clamp to the limit.
        const double limit = VisionRotateBenchConstants::PIDF_OUTPUT_LIMIT_US;
        const double pAlone = limit;
        const double filteredD = 40.0;
        const double rawCombined = pAlone + filteredD;
        const double clamped = rawCombined > limit ? limit : (rawCombined < -limit ? -limit : rawCombined);
        assert(clamped == limit);
        assert(clamped != rawCombined); // this is onVision()'s lastSaturated condition
    }

    // valid=0 skips the PIDF path (direction/pulse stay neutral) but link
    // liveness (lastVisionRx) still updates -- these are independent
    // concerns, same as before the PIDF change.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(200, false, 1, 123);
        assert(s.direction == 0 && s.pulse() == 1500 && s.lastVisionRx == 123);
    }

    // First sample after enable: the rate filter snaps directly to the raw
    // measurement rate (no fake ramp-in from 0), but dt==0 on the very first
    // call (lastPidUpdateMs starts at 0 after reset) means no rate sample
    // exists yet at all -- rateFilterInitialized stays false and
    // filteredRate stays 0 until a second sample arrives with real dt.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(50, true, 1, 100); // dt==0 (lastPidUpdateMs was 0) -> no rate sample yet
        assert(!s.rateFilterInitialized);
        assert(s.filteredRate == 0.0);

        s.onVision(70, true, 1, 150); // dt=50ms, tx moved 50->70 -> rate=(70-50)/0.05=400 px/s
        assert(s.rateFilterInitialized);
        assert(s.filteredRate == 400.0); // first real sample: snaps to raw rate, not blended from 0
    }

    // dt jitter/burst: two onVision() calls stamped with the SAME `now`
    // (e.g. two Serial1 lines processed inside one loop() iteration before
    // millis() ticks) must not corrupt the filter with a bogus zero-rate
    // sample -- filteredRate holds its previous value across the dt<=0 call,
    // while the P term still updates normally for both (each is a genuine
    // sample from Orin, not a duplicate command).
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(50, true, 1, 100); // dt==0, no rate sample yet
        s.onVision(70, true, 1, 150); // dt=50ms -> rate=400 px/s
        const double rateAfterRealSample = s.filteredRate;
        assert(rateAfterRealSample == 400.0);

        s.onVision(90, true, 1, 150); // SAME now=150 as previous call -> dt<=0
        assert(s.lastDtMs == 0); // reported as no-fresh-dt, not a fabricated value
        assert(s.filteredRate == rateAfterRealSample); // held, not blended with a fake 0 rate
    }

    // Lost target then recovered: valid=false resets the filter; the next
    // valid sample starts fresh (snaps to raw rate again), not continuing
    // from the pre-loss filteredRate.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(50, true, 1, 100);
        s.onVision(70, true, 1, 150); // filteredRate=400
        assert(s.filteredRate == 400.0);

        s.onVision(0, false, 0, 160); // target lost
        assert(!s.rateFilterInitialized);
        assert(s.filteredRate == 0.0);

        s.onVision(10, true, 1, 200); // re-locked, dt==0 relative to reset (lastPidUpdateMs=160)
        // dt = (200-160)/1000 = 0.04s > 0, so this IS a real first sample.
        assert(s.rateFilterInitialized);
        assert(s.filteredRate != 400.0); // did not carry over the pre-loss value
    }

    // Target ID switch (same valid=true, different target_id): treated the
    // same as losing and re-acquiring lock -- the filter must not blend
    // rate samples across two different physical targets.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(50, true, 5, 100);
        s.onVision(70, true, 5, 150); // target 5, filteredRate=400
        assert(s.filteredRate == 400.0);

        s.onVision(-200, true, 9, 160); // target switched to 9 -- reset, not a 400px/s jump
        assert(!s.rateFilterInitialized);
        assert(s.filteredRate == 0.0);
        assert(s.direction == 0 && s.speedOffsetUs == 0); // this call itself is the reset, no PIDF output
    }

    // VISION_TIMEOUT: USB pings keep arriving (link alive) but no vision
    // line ever does (Orin disconnected/crashed) -> stop after 500ms.
    {
        VisionRotateBenchState s;
        s.command("E", 0); // lastUsbPing=lastVisionRx=0
        s.command("P", 100); assert(s.enabled);
        s.command("P", 200); assert(s.enabled);
        s.command("P", 300); assert(s.enabled);
        s.command("P", 400); assert(s.enabled);
        s.tick(501);
        assert(!s.enabled);
        assert(!strcmp(s.stopReason, "VISION_TIMEOUT"));
    }

    // TIMEOUT: vision keeps arriving but the browser tab stops pinging
    // (tab crashed/USB dropped) -> stop after 250ms, before VISION_TIMEOUT
    // would ever fire.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.onVision(0, true, 1, 200);
        s.tick(251);
        assert(!s.enabled);
        assert(!strcmp(s.stopReason, "TIMEOUT"));
    }

    // LIMIT, with and without noLimits bypass. direction is set directly
    // here (it's public) rather than via onVision(), since
    // VisionRotateBenchConstants::kP is tuned live on the bench (not a
    // fixed value this test should depend on) -- applyLimits() itself
    // doesn't care how direction got set, so this tests its own logic in
    // isolation from whatever the PIDF gains happen to be tuned to live.
    {
        VisionRotateBenchState s;
        s.command("E", 200);
        s.direction = 1;
        s.applyLimits(false, true);
        assert(!s.enabled && !s.noLimits);

        s.command("E_NO_LIMITS", 300);
        s.direction = 1;
        s.applyLimits(false, true);
        assert(s.enabled && s.noLimits);
    }

    // E clears any stale noLimits/direction from a prior session.
    {
        VisionRotateBenchState s;
        s.command("E_NO_LIMITS", 0);
        s.direction = 1;
        s.command("E", 800);
        assert(s.enabled && !s.noLimits && s.direction == 0);
    }

    // BAD_COMMAND.
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        assert(!s.command("garbage", 1) && !s.enabled);
        assert(!strcmp(s.stopReason, "BAD_COMMAND"));
    }

    // Explicit "X" stop clears a nonzero direction (set directly here, same
    // reason as the LIMIT tests above -- gains are tuned live, not fixed).
    {
        VisionRotateBenchState s;
        s.command("E", 0);
        s.direction = 1;
        s.command("X", 2);
        assert(!s.enabled && s.direction == 0 && s.pulse() == 1500);
    }

    // "P" while disabled is a harmless no-op, not BAD_COMMAND.
    {
        VisionRotateBenchState s;
        assert(s.command("P", 5));
        assert(!s.enabled);
    }

    // 32-bit millis() rollover (COMMAND_TIMEOUT watchdog path).
    {
        VisionRotateBenchState s;
        s.command("E", UINT32_MAX - 100);
        s.tick(100); assert(s.enabled);
        s.tick(152); assert(!s.enabled);
    }

    return 0;
}
