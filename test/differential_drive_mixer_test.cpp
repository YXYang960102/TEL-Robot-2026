#include <assert.h>
#include <math.h>

#include "../src/Chassis/DifferentialDriveMixer.h"

namespace {

bool near(double actual, double expected) {
    return fabs(actual - expected) <= 1e-9;
}

void expectOutput(
    double forward,
    double turn,
    double expectedLeft,
    double expectedRight) {
    const DifferentialDriveOutput output =
        DifferentialDriveMixer::mixArcade(forward, turn);
    assert(near(output.left, expectedLeft));
    assert(near(output.right, expectedRight));
}

}

int main() {
    expectOutput(1.0, 0.0, 1.0, 1.0);
    expectOutput(-1.0, 0.0, -1.0, -1.0);
    expectOutput(0.0, -1.0, -1.0, 1.0);
    expectOutput(0.0, 1.0, 1.0, -1.0);
    expectOutput(1.0, 1.0, 1.0, 0.0);
    return 0;
}
