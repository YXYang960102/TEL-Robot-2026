#pragma once
#include <stdint.h>

// Target pulse only: this is not measured shaft position or speed.
class ServoPositionJog {
public:
    explicit ServoPositionJog(double initialUs) : target(initialUs) {}
    double step(double command, uint32_t elapsedMs, double rateUsPerSecond,
                double minUs, double maxUs) {
        // Do not jump after a stalled loop or integrate invalid input.
        if (elapsedMs > 100 || !(command >= -1.0 && command <= 1.0)) return target;
        target += command * rateUsPerSecond * elapsedMs / 1000.0;
        if (target < minUs) target = minUs;
        if (target > maxUs) target = maxUs;
        return target;
    }
    // Absolute target (e.g. "go to 45 deg"), clamped like step().
    double set(double targetUs, double minUs, double maxUs) {
        if (!(targetUs == targetUs)) return target; // NaN: keep the old target
        target = targetUs < minUs ? minUs : (targetUs > maxUs ? maxUs : targetUs);
        return target;
    }
    double value() const { return target; }
private:
    double target;
};
