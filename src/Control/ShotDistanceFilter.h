#pragma once

#include <stdint.h>

// The camera distance the autos look the shot table up with, steadied
// (2026-10-27). Standing still, the depth reading still wanders a few cm
// frame to frame; on a steep part of the table that moved the flywheel
// command by more than Shooter's 2% "spun up" band (restarting its 1.5 s
// wait every time) and kept the elevation hunting, so the autos never
// fired. Widening those bands would only make the shots less accurate.
// Instead:
//   1. median of the last FRAMES depth frames (one bad frame can't move it);
//   2. the latched shot distance follows that median only once it has
//      been more than BAND_M away for SETTLE_MS (driving to a new spot),
//      and otherwise stays put -- so the table values, the flywheel
//      command and the elevation target are constant while standing still.
// reset() on a new hole or a lost target; the first frame after a reset
// latches at once. Pure logic (no Arduino), host-testable.
template <uint8_t FRAMES>
class ShotDistanceFilter {
public:
    ShotDistanceFilter(double bandM, unsigned long settleMs) : bandM(bandM), settleMs(settleMs) {}

    void reset() {
        count = 0;
        next = 0;
        latchedM = 0.0;
        pending = false;
    }

    // One new depth frame, meters (> 0).
    void addSample(double meters, unsigned long nowMs) {
        samples[next] = meters;
        next = static_cast<uint8_t>((next + 1) % FRAMES);
        if (count < FRAMES) count++;
        const double m = median();
        if (latchedM <= 0.0) {
            latchedM = m;
            pending = false;
            return;
        }
        const double diff = m > latchedM ? m - latchedM : latchedM - m;
        if (diff <= bandM) {
            pending = false; // back inside the band: the move was noise
            return;
        }
        if (!pending) {
            pending = true;
            pendingSinceMs = nowMs;
        } else if (nowMs - pendingSinceMs >= settleMs) {
            latchedM = m;
            pending = false;
        }
    }

    // The distance to shoot at, meters; 0 = none since the last reset.
    double latched() const { return latchedM; }

    // Median of the frames in hand (0 when none).
    double median() const {
        if (count == 0) return 0.0;
        double sorted[FRAMES];
        for (uint8_t i = 0; i < count; i++) sorted[i] = samples[i];
        for (uint8_t i = 1; i < count; i++) { // insertion sort, FRAMES is tiny
            const double v = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > v) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = v;
        }
        return count % 2 ? sorted[count / 2] : (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0;
    }

private:
    double bandM;
    unsigned long settleMs;
    double samples[FRAMES] = {};
    uint8_t count = 0;
    uint8_t next = 0;
    double latchedM = 0.0;
    bool pending = false;
    unsigned long pendingSinceMs = 0;
};
