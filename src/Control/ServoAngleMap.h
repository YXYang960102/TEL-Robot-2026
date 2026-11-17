#pragma once

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
    double minDegrees() const {
        const double a = toDegrees(minUs), b = toDegrees(maxUs);
        return a < b ? a : b;
    }
    double maxDegrees() const {
        const double a = toDegrees(minUs), b = toDegrees(maxUs);
        return a < b ? b : a;
    }
};
