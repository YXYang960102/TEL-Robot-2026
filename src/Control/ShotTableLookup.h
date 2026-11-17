#pragma once

#include "../Constants/TuningConstants.h"

class ShotTableLookup {
public:
    struct Result {
        double flywheelCommand;
        double elevationPulseUs;
    };

    static Result lookup(
        double distanceMeters,
        const TuningConstants::ShotTable::Entry* table,
        int count,
        bool extrapolate = TuningConstants::ShotTable::EXTRAPOLATE_PAST_LAST_ENTRY) {
        if (table == nullptr || count <= 0) {
            return {0.0, 0.0};
        }
        if (count == 1 || distanceMeters <= meters(table[0])) {
            return {flywheel(table[0]), elevation(table[0])};
        }
        if (!extrapolate && distanceMeters >= meters(table[count - 1])) {
            return {flywheel(table[count - 1]), elevation(table[count - 1])};
        }

        int i = 0;
        while (i < count - 2 && distanceMeters > meters(table[i + 1])) {
            i++;
        }
        const auto& lower = table[i];
        const auto& upper = table[i + 1];
        const double span = meters(upper) - meters(lower);
        const double t = span > 0.0 ? (distanceMeters - meters(lower)) / span : 0.0;
        const double fly = flywheel(lower) + (flywheel(upper) - flywheel(lower)) * t;
        return {
            fly < 0.0 ? 0.0 : (fly > 1.0 ? 1.0 : fly),
            elevation(lower) + (elevation(upper) - elevation(lower)) * t,
        };
    }

    static double meters(const TuningConstants::ShotTable::Entry& e) { return e.distanceCm / 100.0; }
    static double flywheel(const TuningConstants::ShotTable::Entry& e) { return e.flywheelPermille / 1000.0; }
    static double elevation(const TuningConstants::ShotTable::Entry& e) { return e.elevationUs; }

    static bool lookupTarget(int visionId, double distanceMeters, Result& out, uint8_t color = 0) {
        const TuningConstants::ShotTable::Table table =
            TuningConstants::ShotTable::tableForVisionId(visionId, color);
        if (table.entries == nullptr) {
            return false;
        }
        out = lookup(distanceMeters, table.entries, table.count);
        out.elevationPulseUs += TuningConstants::ShotTable::elevationTrimUs(visionId);
        return true;
    }
};
