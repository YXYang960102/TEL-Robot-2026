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
// needs Arduino; its READY_SETTLE_TIME_MS / MAX_CONTROL_GAP_MS are 100).
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
    // Production gains are the bench gains / 500 us (1.0 = full 500 us).
    {
        namespace B = VisionRotateBenchConstants;
        assert(near(VISION_kP, B::kP / 500.0));
        assert(near(VISION_kI, B::kI / 500.0));
        assert(near(VISION_kD, B::kD / 500.0));
        assert(near(VISION_kIZONE_PX, B::kIZone));
        assert(near(VISION_MAX_COMMAND, B::PIDF_OUTPUT_LIMIT_US / 500.0));
        assert(near(VISION_MAX_INTEGRAL_COMMAND, B::PIDF_INTEGRAL_LIMIT_US / 500.0));
        assert(near(VISION_READY_TOLERANCE_PX, 5.0));
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

    // Ready for Auto: |tx| <= 5 px continuously for 100 ms.
    {
        VisionFollowController c(CONFIG);
        c.update(3.0, true, 1, 0);
        c.update(3.0, true, 1, 10);   // tolerance window starts here
        c.update(3.0, true, 1, 100);
        assert(!c.isReady());          // 90 ms
        c.update(-4.0, true, 1, 110);
        assert(c.isReady());           // 100 ms
        c.update(6.0, true, 1, 120);   // 6 px > 5: not locked any more
        assert(!c.isReady());
        c.update(5.0, true, 1, 130);
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
    // sequence at the bench's ~10 fps, production pulse == bench pulse.
    {
        VisionFollowController prod(CONFIG);
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

    printf("vision_follow_controller_test: all asserts passed\n");
    return 0;
}
