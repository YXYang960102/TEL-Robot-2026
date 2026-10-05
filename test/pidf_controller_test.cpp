#include "../src/Control/PidfController.h"

#include <cmath>
#include <iostream>

namespace {

int failures = 0;

void expectNear(const char* name, double actual, double expected) {
    if (std::fabs(actual - expected) <= 1e-9) {
        return;
    }

    std::cerr << name << ": expected " << expected
              << ", got " << actual << '\n';
    failures++;
}

void expectTrue(const char* name, bool value) {
    if (value) {
        return;
    }

    std::cerr << name << ": expected true\n";
    failures++;
}

void testDefaultsAreInactive() {
    PidfController controller;
    controller.reset(25.0);
    expectNear(
        "default controller output",
        controller.calculate(100.0, 25.0, 0.02, 1.0),
        0.0);
}

void testPidAndFeedforwardTerms() {
    PidfController controller(PidfConfig(0.1, 0.0, 0.01, 0.0, 0.2));
    controller.reset(10.0);

    const double output = controller.calculate(20.0, 12.0, 0.1, 0.5);
    expectNear("proportional term", controller.getProportionalTerm(), 0.8);
    expectNear("derivative term", controller.getDerivativeTerm(), -0.2);
    expectNear("feedforward term", controller.getFeedforwardTerm(), 0.1);
    expectNear("combined output", output, 0.7);
}

void testIntegralZoneResetsAccumulation() {
    PidfController controller(PidfConfig(0.0, 0.5, 0.0, 2.0, 0.0));
    controller.reset(0.0);

    controller.calculate(10.0, 0.0, 1.0);
    expectNear("integral outside IZone", controller.getIntegralTerm(), 0.0);

    controller.calculate(1.0, 0.0, 1.0);
    expectNear("integral inside IZone", controller.getIntegralTerm(), 0.5);

    controller.calculate(10.0, 0.0, 1.0);
    expectNear("integral reset after leaving IZone", controller.getIntegralTerm(), 0.0);
}

void testZeroIntegralZoneDisablesIntegral() {
    PidfController controller(PidfConfig(0.0, 1.0, 0.0, 0.0, 0.0));
    controller.reset(0.0);

    controller.calculate(1.0, 0.0, 1.0);
    expectNear("zero IZone disables integral", controller.getIntegralTerm(), 0.0);
}

void testSaturationRejectsFurtherIntegralWindup() {
    PidfController controller(PidfConfig(1.0, 1.0, 0.0, 10.0, 0.0));
    controller.setOutputLimits(-1.0, 1.0);
    controller.setIntegralOutputLimits(-0.5, 0.5);
    controller.reset(0.0);

    const double saturatedOutput = controller.calculate(2.0, 0.0, 0.1);
    expectNear("saturated output", saturatedOutput, 1.0);
    expectNear("rejected windup term", controller.getIntegralTerm(), 0.0);
    expectTrue("saturation flag", controller.isSaturated());

    const double recoveredOutput = controller.calculate(0.0, 0.0, 0.1);
    expectNear("output after saturation clears", recoveredOutput, 0.0);
}

}  // namespace

int main() {
    testDefaultsAreInactive();
    testPidAndFeedforwardTerms();
    testIntegralZoneResetsAccumulation();
    testZeroIntegralZoneDisablesIntegral();
    testSaturationRejectsFurtherIntegralWindup();

    if (failures != 0) {
        return 1;
    }

    std::cout << "PidfController tests passed\n";
    return 0;
}
