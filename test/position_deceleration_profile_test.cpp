#include <assert.h>
#include <math.h>

#include "../src/Control/PositionDecelerationProfile.h"

namespace {

bool near(double actual, double expected) {
    return fabs(actual - expected) <= 1e-9;
}

}

int main() {
    assert(near(
        PositionDecelerationProfile::calculateMaximumCommand(
            100.0, 100.0, 0.20, 1.0, 0.10),
        1.0));
    assert(near(
        PositionDecelerationProfile::calculateMaximumCommand(
            100.0, 20.0, 0.20, 1.0, 0.10),
        1.0));
    assert(near(
        PositionDecelerationProfile::calculateMaximumCommand(
            100.0, 10.0, 0.20, 1.0, 0.10),
        0.5));
    assert(near(
        PositionDecelerationProfile::calculateMaximumCommand(
            100.0, 1.0, 0.20, 1.0, 0.10),
        0.10));
    assert(near(
        PositionDecelerationProfile::calculateMaximumCommand(
            100.0, 1.0, 0.20, 0.10, 0.01),
        0.01));
    assert(near(
        PositionDecelerationProfile::calculateProfiledCommand(
            -10.0, 100.0, 0.20, 1.0, 0.10),
        -0.5));
    assert(near(
        PositionDecelerationProfile::calculateProfiledCommand(
            0.0, 100.0, 0.20, 1.0, 0.10),
        0.0));
    return 0;
}
