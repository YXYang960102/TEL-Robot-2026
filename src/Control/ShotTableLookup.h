#pragma once

#include "../Constants/TuningConstants.h"

// Looks up the distance -> {flywheelCommand, elevationPulseUs} shot table
// (TuningConstants::ShotTable) with clamped linear interpolation between
// the two nearest entries. Header-only static method -- pure logic, no
// Arduino dependency, native-host-testable.
class ShotTableLookup {
public:
    struct Result {
        double flywheelCommand;
        double elevationPulseUs;
    };

    static Result lookup(
        double distanceMeters,
        const TuningConstants::ShotTable::Entry* table,
        int count) {
        if (count <= 0) {
            return {0.0, 0.0};
        }
        if (count == 1 || distanceMeters <= table[0].distanceMeters) {
            return {table[0].flywheelCommand, table[0].elevationPulseUs};
        }
        if (distanceMeters >= table[count - 1].distanceMeters) {
            return {table[count - 1].flywheelCommand, table[count - 1].elevationPulseUs};
        }

        for (int i = 0; i < count - 1; i++) {
            const auto& lower = table[i];
            const auto& upper = table[i + 1];
            if (distanceMeters > upper.distanceMeters) {
                continue;
            }

            const double span = upper.distanceMeters - lower.distanceMeters;
            const double t = span > 0.0
                ? (distanceMeters - lower.distanceMeters) / span
                : 0.0;
            return {
                lower.flywheelCommand + (upper.flywheelCommand - lower.flywheelCommand) * t,
                lower.elevationPulseUs + (upper.elevationPulseUs - lower.elevationPulseUs) * t,
            };
        }

        // Unreachable given the bounds checks above; satisfies -Werror on a
        // missing return without masking a real bug if the table were ever
        // passed unsorted.
        return {table[count - 1].flywheelCommand, table[count - 1].elevationPulseUs};
    }
};
