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
    double value() const { return target; }
private:
    double target;
};
