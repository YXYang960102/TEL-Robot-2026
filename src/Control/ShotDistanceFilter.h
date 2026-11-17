#pragma once

#include <stdint.h>

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
            pending = false;
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

    double latched() const { return latchedM; }

    double median() const {
        if (count == 0) return 0.0;
        double sorted[FRAMES];
        for (uint8_t i = 0; i < count; i++) sorted[i] = samples[i];
        for (uint8_t i = 1; i < count; i++) {
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
