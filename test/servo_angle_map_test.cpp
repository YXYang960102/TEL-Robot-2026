// Elevation angle <-> pulse map (src/Control/ServoAngleMap.h) with the real
// ShooterConstants::Angle calibration, plus ServoPositionJog::set().
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../src/Constants/ShooterConstants.h"
#include "../src/Control/ServoAngleMap.h"
#include "../src/Control/ServoPositionJog.h"

static bool near(double a, double b) { return fabs(a - b) < 1e-9; }

int main() {
    using namespace ShooterConstants;

    // Provisional calibration = the servo spec: 500 us = 0 deg, 2500 us = 270 deg.
    const ServoAngleMap& m = Angle::ANGLE_MAP;
    assert(near(m.toPulse(0.0), 500.0));
    assert(near(m.toPulse(270.0), 2500.0));
    assert(near(m.toPulse(135.0), 1500.0));          // middle = neutral
    assert(near(m.toPulse(45.0), 500.0 + 45.0 * 2000.0 / 270.0));
    assert(near(m.toDegrees(1500.0), 135.0));
    assert(near(m.toDegrees(m.toPulse(47.5)), 47.5)); // round trip
    // Out of range clamps to the pulse limits (= the servo's ends).
    assert(near(m.toPulse(-20.0), Angle::MIN_PULSE_US));
    assert(near(m.toPulse(400.0), Angle::MAX_PULSE_US));
    assert(near(m.minDegrees(), 0.0) && near(m.maxDegrees(), 270.0));

    // A real elevation calibration, e.g. level (0 deg) at 1900 us and the
    // top stop (60 deg) at 1100 us -- mounted so a higher angle is a
    // shorter pulse -- with travel limited to 1050..1950 us.
    const ServoAngleMap real{1900.0, 0.0, 1100.0, 60.0, 1050.0, 1950.0};
    assert(near(real.toPulse(0.0), 1900.0));
    assert(near(real.toPulse(60.0), 1100.0));
    assert(near(real.toPulse(30.0), 1500.0));
    assert(near(real.toPulse(90.0), 1050.0));        // clamped
    assert(near(real.minDegrees(), real.toDegrees(1950.0)));  // ascending either way
    assert(near(real.maxDegrees(), real.toDegrees(1050.0)));
    assert(real.minDegrees() < real.maxDegrees());

    // ServoPositionJog::set(): absolute target, clamped; NaN keeps the old one.
    ServoPositionJog jog(1500.0);
    assert(near(jog.set(2000.0, 500.0, 2500.0), 2000.0));
    assert(near(jog.set(3000.0, 500.0, 2500.0), 2500.0));
    assert(near(jog.set(NAN, 500.0, 2500.0), 2500.0));
    // A zero jog afterwards holds it (what the manual loop does every tick).
    assert(near(jog.step(0.0, 10, 1000.0, 500.0, 2500.0), 2500.0));

    printf("servo_angle_map_test: all asserts passed\n");
    return 0;
}
