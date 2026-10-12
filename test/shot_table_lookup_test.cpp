#include <assert.h>
#include <math.h>

#include "../src/Control/ShotTableLookup.h"

namespace {

bool near(double actual, double expected) {
    return fabs(actual - expected) <= 1e-9;
}

constexpr TuningConstants::ShotTable::Entry kTable[] = {
    {2.0, 0.70, 15.0},
    {3.0, 0.80, 25.0},
    {4.0, 0.90, 35.0},
};
constexpr int kCount = sizeof(kTable) / sizeof(kTable[0]);

}  // namespace

int main() {
    // Exact hits on table entries.
    {
        auto r = ShotTableLookup::lookup(2.0, kTable, kCount);
        assert(near(r.flywheelCommand, 0.70));
        assert(near(r.angleDegrees, 15.0));
    }
    {
        auto r = ShotTableLookup::lookup(3.0, kTable, kCount);
        assert(near(r.flywheelCommand, 0.80));
        assert(near(r.angleDegrees, 25.0));
    }

    // Linear interpolation midway between two entries.
    {
        auto r = ShotTableLookup::lookup(2.5, kTable, kCount);
        assert(near(r.flywheelCommand, 0.75));
        assert(near(r.angleDegrees, 20.0));
    }
    // Interpolation at a non-midpoint fraction.
    {
        auto r = ShotTableLookup::lookup(3.25, kTable, kCount);
        assert(near(r.flywheelCommand, 0.825));
        assert(near(r.angleDegrees, 27.5));
    }

    // Below the first entry: clamp, do not extrapolate.
    {
        auto r = ShotTableLookup::lookup(0.5, kTable, kCount);
        assert(near(r.flywheelCommand, 0.70));
        assert(near(r.angleDegrees, 15.0));
    }
    // Above the last entry: clamp, do not extrapolate.
    {
        auto r = ShotTableLookup::lookup(20.0, kTable, kCount);
        assert(near(r.flywheelCommand, 0.90));
        assert(near(r.angleDegrees, 35.0));
    }

    // count <= 1 is a defensive no-op case.
    {
        auto r = ShotTableLookup::lookup(5.0, kTable, 0);
        assert(near(r.flywheelCommand, 0.0));
        assert(near(r.angleDegrees, 0.0));
    }
    {
        auto r = ShotTableLookup::lookup(5.0, kTable, 1);
        assert(near(r.flywheelCommand, kTable[0].flywheelCommand));
        assert(near(r.angleDegrees, kTable[0].angleDegrees));
    }

    // The real project table: sanity-check it is sorted and interpolates
    // cleanly across its full range without special-casing any entry.
    {
        using TuningConstants::ShotTable::TABLE;
        using TuningConstants::ShotTable::TABLE_COUNT;
        for (int i = 0; i < TABLE_COUNT - 1; i++) {
            assert(TABLE[i].distanceMeters < TABLE[i + 1].distanceMeters);
        }
        auto r = ShotTableLookup::lookup(5.25, TABLE, TABLE_COUNT);
        assert(r.flywheelCommand > TABLE[6].flywheelCommand &&
               r.flywheelCommand < TABLE[7].flywheelCommand);
    }

    return 0;
}
