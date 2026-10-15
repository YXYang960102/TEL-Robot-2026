#pragma once

// Linear map between a positional servo's pulse and an angle, defined by
// two calibration points (pulseA <-> degA, pulseB <-> degB), with the pulse
// clamped to [minUs, maxUs]. Pure logic, host-tested
// (test/servo_angle_map_test.cpp).
struct ServoAngleMap {
    double pulseA;
    double degA;
    double pulseB;
    double degB;
    double minUs;
    double maxUs;

    double clampPulse(double us) const {
        return us < minUs ? minUs : (us > maxUs ? maxUs : us);
    }
    double toPulse(double degrees) const {
        return clampPulse(pulseA + (degrees - degA) * (pulseB - pulseA) / (degB - degA));
    }
    double toDegrees(double us) const {
        return degA + (us - pulseA) * (degB - degA) / (pulseB - pulseA);
    }
    // The reachable angle range (the ends of the clamped pulse range), in
    // ascending order whichever way the servo is mounted.
    double minDegrees() const {
        const double a = toDegrees(minUs), b = toDegrees(maxUs);
        return a < b ? a : b;
    }
    double maxDegrees() const {
        const double a = toDegrees(minUs), b = toDegrees(maxUs);
        return a < b ? b : a;
    }
};
