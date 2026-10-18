#pragma once

#include "../Constants/TuningConstants.h"

// Looks up a distance -> {flywheelCommand, elevationPulseUs} shot table
// (TuningConstants::ShotTable) with linear interpolation between the two
// nearest entries:
//   below the first entry: the first entry's values (clamped);
//   beyond the last entry: the last entry's values, or -- with
//     extrapolate (ShotTable::EXTRAPOLATE_PAST_LAST_ENTRY) -- extended
//     along the last two entries' trend (Jeremy, 2026-10-16: past 8.5 m,
//     predict rather than freeze; held instead while the tables are short,
//     2026-10-18). The flywheel command is kept within 0..1; the elevation
//     pulse is limited by Shooter::setAnglePulseUs().
// Header-only static methods -- pure logic, no Arduino dependency,
// native-host-testable.
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
        // table[i]..table[i+1] brackets the distance, or (beyond the last
        // entry) is the last segment, which t > 1 then extends.
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

    // The generated table's integer units (ShotTableData::Point).
    static double meters(const TuningConstants::ShotTable::Entry& e) { return e.distanceCm / 100.0; }
    static double flywheel(const TuningConstants::ShotTable::Entry& e) { return e.flywheelPermille / 1000.0; }
    static double elevation(const TuningConstants::ShotTable::Entry& e) { return e.elevationUs; }

    // The shot for one Orin target: its height's table
    // (ShotTable::tableForVisionId: the hole's, or its row center's) plus
    // that hole's elevation trim.
    // False when the id has no table (0 / unknown): don't shoot at it.
    static bool lookupTarget(int visionId, double distanceMeters, Result& out) {
        const TuningConstants::ShotTable::Table table =
            TuningConstants::ShotTable::tableForVisionId(visionId);
        if (table.entries == nullptr) {
            return false;
        }
        out = lookup(distanceMeters, table.entries, table.count);
        out.elevationPulseUs += TuningConstants::ShotTable::elevationTrimUs(visionId);
        return true;
    }
};
