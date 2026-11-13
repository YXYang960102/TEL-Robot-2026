// Production turret vision-follow (src/Control/VisionFollowController.h)
// with the real production gains (TuningConstants::Rotate::VISION_*), and
// a side-by-side check against the A5 bench state machine Jeremy tuned
// (VisionRotateBenchState), so production behaves like what was accepted
// on hardware on 2026-10-13.
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../src/Constants/TuningConstants.h"
#include "../src/Constants/VisionRotateBenchConstants.h"
#include "../src/Control/VisionFollowController.h"
#include "../src/Control/VisionRotateBenchState.h"

namespace {

using namespace TuningConstants::Rotate;

// Same composition as ShooterConstants::Rotate::VISION_FOLLOW (that header
// needs Arduino; its MAX_CONTROL_GAP_MS is 100 and READY_SETTLE_TIME_MS 250
// since 2026-11-01 -- the bench comparison below keeps the bench's 100).
const VisionFollowConfig CONFIG(
    PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
    VISION_MAX_COMMAND,
    VISION_MAX_INTEGRAL_COMMAND,
    VISION_READY_TOLERANCE_PX,
    100,
    100);

// Rotate::INVERTED is false, so the servo pulse is 1500 + command * 500.
int pulseFor(double command) {
    return (int)lround(1500.0 + command * 500.0);
}

bool near(double a, double b, double eps = 1e-9) { return fabs(a - b) <= eps; }

}  // namespace

int main() {
    // Production gains are the bench gains / 500 us (1.0 = full 500 us),
    // except kI: twice the bench's since 2026-11-09 (Jeremy: stuck just
    // outside the lock).
    {
        namespace B = VisionRotateBenchConstants;
        assert(near(VISION_kP, B::kP / 500.0));
        assert(near(VISION_kI, 2.0 * B::kI / 500.0));
        assert(near(VISION_kD, B::kD / 500.0));
        assert(near(VISION_kIZONE_PX, B::kIZone));
        assert(near(VISION_MAX_COMMAND, B::PIDF_OUTPUT_LIMIT_US / 500.0));
        assert(near(VISION_MAX_INTEGRAL_COMMAND, B::PIDF_INTEGRAL_LIMIT_US / 500.0));
        assert(near(VISION_READY_TOLERANCE_PX, 6.0)); // 5 until 2026-11-09
    }

    // First frame of a target only primes; then P alone inside the linear
    // zone: tx = 40 -> -0.40 -> 1300 us (tx > 0 -> below 1500, as on the bench).
    {
        VisionFollowController c(CONFIG);
        assert(c.update(40.0, true, 1, 1000) == 0.0);
        const double cmd = c.update(40.0, true, 1, 1010);
        assert(near(cmd, -0.40));
        assert(pulseFor(cmd) == 1300);
    }

    // Far off: full speed, saturated at the servo's end (1000 / 2000 us).
    {
        VisionFollowController c(CONFIG);
        c.update(400.0, true, 1, 0);
        assert(pulseFor(c.update(400.0, true, 1, 10)) == 1000);
        assert(c.isSaturated());
        assert(pulseFor(c.update(-400.0, true, 1, 20)) == 2000);
    }

    // I inside 20 px: held 5 px off for 10 s, I tops out at -0.12;
    // P -0.05 + I -0.12 = -0.17 -> 1415 us (the bench gives the same).
    {
        VisionFollowController c(CONFIG);
        unsigned long t = 0;
        c.update(5.0, true, 1, t);
        double cmd = 0.0;
        for (int i = 0; i < 1000; ++i) { t += 10; cmd = c.update(5.0, true, 1, t); }
        assert(near(c.getIntegralTerm(), -VISION_MAX_INTEGRAL_COMMAND));
        assert(pulseFor(cmd) == 1415);

        // 15 px: still inside the zone, I stays (capped).
        t += 10; c.update(15.0, true, 1, t);
        assert(near(c.getIntegralTerm(), -VISION_MAX_INTEGRAL_COMMAND));

        // 25 px: outside the zone -> I dropped, pure P: -0.25 -> 1375 us.
        t += 10; cmd = c.update(25.0, true, 1, t);
        assert(c.getIntegralTerm() == 0.0);
        assert(pulseFor(cmd) == 1375);
    }

    // Ready for Auto: |tx| <= 6 px (5 until 2026-11-09) continuously for
    // 100 ms.
    {
        VisionFollowController c(CONFIG);
        c.update(3.0, true, 1, 0);
        c.update(3.0, true, 1, 10);   // tolerance window starts here
        c.update(3.0, true, 1, 100);
        assert(!c.isReady());          // 90 ms
        c.update(-4.0, true, 1, 110);
        assert(c.isReady());           // 100 ms
        c.update(7.0, true, 1, 120);   // 7 px > 6: not locked any more
        assert(!c.isReady());
        c.update(6.0, true, 1, 130);
        assert(!c.isReady());          // window restarts
    }

    // Lost target / switched target / control gap / garbage: command 0 and
    // the lock (and its integral) start over.
    {
        VisionFollowController c(CONFIG);
        c.update(5.0, true, 1, 0);
        for (unsigned long t = 10; t <= 2000; t += 10) c.update(5.0, true, 1, t);
        assert(c.getIntegralTerm() < 0.0);

        assert(c.update(5.0, false, 1, 2010) == 0.0);
        assert(c.getIntegralTerm() == 0.0 && !c.isReady());
        assert(c.update(5.0, true, 1, 2020) == 0.0);   // re-prime
        assert(c.update(5.0, true, 1, 2030) != 0.0);

        assert(c.update(5.0, true, 2, 2040) == 0.0);   // switched target
        assert(c.update(5.0, true, 2, 2050) != 0.0);

        assert(c.update(5.0, true, 2, 2200) == 0.0);   // 150 ms gap > 100
        assert(c.update(5.0, true, 2, 2210) != 0.0);

        assert(c.update(NAN, true, 2, 2220) == 0.0);
        assert(c.update(INFINITY, true, 2, 2230) == 0.0);

        c.reset();
        assert(c.update(5.0, true, 2, 2240) == 0.0);    // reset() forgets the lock
    }

    // Side by side with the bench state machine Jeremy tuned: same tx
    // sequence at the bench's ~10 fps, production pulse == bench pulse --
    // with the bench's kI (production's is twice it since 2026-11-09), so
    // this still checks the controller's math against the bench.
    {
        const VisionFollowConfig benchGains(
            PidfConfig(VISION_kP, VisionRotateBenchConstants::kI / 500.0, VISION_kD,
                       VISION_kIZONE_PX, VISION_kFF),
            VISION_MAX_COMMAND, VISION_MAX_INTEGRAL_COMMAND, VISION_READY_TOLERANCE_PX,
            100, 100);
        VisionFollowController prod(benchGains);
        VisionRotateBenchState bench;
        bench.command("E", 0);
        const int txs[] = {180, 150, 110, 80, 60, 40, 25, 18, 12, 8, 6, 5, 4, 3, 3, 2, 2, 1,
                           -2, -6, -15, -30, 10, 10, 10, 10, 10, 10};
        unsigned long t = 100;
        for (int tx : txs) {
            bench.onVision(tx, true, 1, t);
            bench.command("P", t);
            const double cmd = prod.update(tx, true, 1, t);
            const int diff = pulseFor(cmd) - bench.pulse();
            assert(diff >= -1 && diff <= 1);  // rounding only
            t += 100;
        }
    }

    // Frame-to-frame D (2026-11-01): only with frame numbers; the rate is
    // taken between camera frames, held between them, and brakes.
    {
        const VisionFollowConfig withD(
            PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
            VISION_MAX_COMMAND, VISION_MAX_INTEGRAL_COMMAND, VISION_READY_TOLERANCE_PX,
            100, 100, 0.0005);
        VisionFollowController c(withD);
        c.update(40.0, true, 1, 0, 1);                    // prime, frame 1
        c.update(40.0, true, 1, 10, 1);                   // same frame: no rate
        assert(c.getTxRate() == 0.0 && c.getDerivativeTerm() == 0.0);
        const double cmd = c.update(20.0, true, 1, 40, 2); // frame 2: 20 px in 40 ms
        // raw -500 px/s, smoothed 50/50 with 0 -> -250; D = +0.125 (brakes)
        assert(near(c.getTxRate(), -250.0) && near(c.getDerivativeTerm(), 0.125));
        assert(cmd > -0.20 && cmd < 0.0);                  // PI alone would be about -0.20
        c.update(20.0, true, 1, 50, 2);                   // still frame 2: rate held
        assert(near(c.getTxRate(), -250.0));
        c.update(20.0, true, 2, 60, 3);                   // switched target: re-primed
        assert(c.getTxRate() == 0.0 && c.getDerivativeTerm() == 0.0);
        // Without frame numbers (frame 0): no D at all.
        VisionFollowController plain(withD);
        plain.update(40.0, true, 1, 0);
        plain.update(20.0, true, 1, 40);
        assert(plain.getDerivativeTerm() == 0.0);
    }

    // Stall push (2026-11-01): outside the 6 px tolerance (5 until
    // 2026-11-09, so 7 px here was 6) and still for
    // 200 ms -> 0.17 toward the target for an 80 ms pulse, then PI alone and
    // another 200 ms still before the next; never inside the tolerance,
    // where the I term is held; off without frame numbers or with 0.
    {
        const VisionFollowConfig withPush(
            PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
            VISION_MAX_COMMAND, VISION_MAX_INTEGRAL_COMMAND, VISION_READY_TOLERANCE_PX,
            100, 100, VISION_FRAME_kD,
            VISION_STALL_COMMAND, VISION_STALL_RATE_PX_PER_S, VISION_STALL_DELAY_MS,
            VISION_STALL_PULSE_MS, VISION_HOLD_I_IN_TOLERANCE);
        assert(VISION_STALL_DELAY_MS == 200 && VISION_STALL_PULSE_MS == 80);
        VisionFollowController c(withPush);
        unsigned long frame = 1;
        double cmd = c.update(7.0, true, 1, 0, frame);           // prime
        // Still at 7 px (a new frame every 60 ms, same tx). Still from 10 ms:
        // no push to 200, pulse 210-280, PI alone 290-480, next pulse 490.
        for (unsigned long t = 10; t <= 490; t += 10) {
            if (t % 60 == 0) ++frame;
            cmd = c.update(7.0, true, 1, t, frame);
            const bool pulse = (t >= 210 && t <= 280) || t == 490;
            assert(c.isStallPushing() == pulse);
            if (pulse) assert(near(cmd, -VISION_STALL_COMMAND));
            else assert(cmd > -VISION_STALL_COMMAND && cmd < 0.0); // PI alone, under the push
        }
        // In tolerance (4 px): no push, and the I term is held -- 5 s at
        // 4 px and P + I stays where it came in, under the turret's 0.16.
        unsigned long t = 500;
        ++frame; cmd = c.update(4.0, true, 1, t, frame);
        const double heldI = c.getIntegralTerm();
        for (int i = 0; i < 500; ++i) {
            t += 10;
            if (t % 60 == 0) ++frame;
            cmd = c.update(4.0, true, 1, t, frame);
            assert(!c.isStallPushing() && fabs(cmd) < 0.16);
        }
        assert(near(c.getIntegralTerm(), heldI));
        // Out of the tolerance again: I grows again.
        t += 10; ++frame; c.update(8.0, true, 1, t, frame);
        t += 10; c.update(8.0, true, 1, t, frame);
        assert(c.getIntegralTerm() < heldI);
        // Moving (a frame 15 px apart -> rate well over 30 px/s): no push.
        t += 60; ++frame; c.update(-8.0, true, 1, t, frame);
        assert(!c.isStallPushing());
        // The rate decays, then 200 ms still -> a pulse toward + (some frame).
        bool pushedPlus = false;
        for (int i = 0; i < 80; ++i) {
            t += 10;
            if (i % 6 == 0) ++frame;
            cmd = c.update(-8.0, true, 1, t, frame);
            if (c.isStallPushing()) { pushedPlus = true; assert(near(cmd, VISION_STALL_COMMAND)); }
        }
        assert(pushedPlus);

        // Pulse 0 = the continuous push: still pushing long after the delay.
        const VisionFollowConfig continuous(
            PidfConfig(VISION_kP, VISION_kI, VISION_kD, VISION_kIZONE_PX, VISION_kFF),
            VISION_MAX_COMMAND, VISION_MAX_INTEGRAL_COMMAND, VISION_READY_TOLERANCE_PX,
            100, 100, VISION_FRAME_kD,
            VISION_STALL_COMMAND, VISION_STALL_RATE_PX_PER_S, VISION_STALL_DELAY_MS, 0);
        VisionFollowController cont(continuous);
        cont.update(7.0, true, 1, 0, 1);
        for (unsigned long u = 10; u <= 600; u += 10) cont.update(7.0, true, 1, u, 1 + u / 60);
        assert(cont.isStallPushing());

        // No frame numbers: no rate, no push.
        VisionFollowController plain(withPush);
        plain.update(7.0, true, 1, 0);
        for (unsigned long u = 10; u <= 1000; u += 10) cmd = plain.update(7.0, true, 1, u);
        assert(!plain.isStallPushing());
        // The production gains without a push stay as they were.
        VisionFollowController off(CONFIG);
        off.update(7.0, true, 1, 0, 1);
        for (unsigned long u = 10; u <= 1000; u += 10) off.update(7.0, true, 1, u, 1 + u / 60);
        assert(!off.isStallPushing());
    }

    printf("vision_follow_controller_test: all asserts passed\n");
    return 0;
}
