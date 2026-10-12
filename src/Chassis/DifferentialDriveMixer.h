#pragma once

#include <math.h>

struct DifferentialDriveOutput {
    double left;
    double right;
};

class DifferentialDriveMixer {
public:
    static DifferentialDriveOutput mixArcade(double forward, double turn) {
        DifferentialDriveOutput output = {
            forward + turn,
            forward - turn
        };
        const double maximumMagnitude =
            fmax(fabs(output.left), fabs(output.right));
        if (maximumMagnitude > 1.0) {
            output.left /= maximumMagnitude;
            output.right /= maximumMagnitude;
        }
        return output;
    }
};
