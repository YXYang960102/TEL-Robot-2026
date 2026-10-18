#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <initializer_list>

#include "../src/Control/ShotTableLookup.h"

namespace {

bool near(double actual, double expected) {
    return fabs(actual - expected) <= 1e-9;
}

// {distance cm, flywheel per mille, elevation us} (ShotTableData::Point).
constexpr TuningConstants::ShotTable::Entry kTable[] = {
    {200, 700, 1800},
    {300, 800, 1600},
    {400, 900, 1400},
};
constexpr int kCount = sizeof(kTable) / sizeof(kTable[0]);

}  // namespace

int main() {
    // Exact hits on table entries.
    {
        auto r = ShotTableLookup::lookup(2.0, kTable, kCount);
        assert(near(r.flywheelCommand, 0.70));
        assert(near(r.elevationPulseUs, 1800.0));
    }
    {
        auto r = ShotTableLookup::lookup(3.0, kTable, kCount);
        assert(near(r.flywheelCommand, 0.80));
        assert(near(r.elevationPulseUs, 1600.0));
    }

    // Linear interpolation midway between two entries.
    {
        auto r = ShotTableLookup::lookup(2.5, kTable, kCount);
        assert(near(r.flywheelCommand, 0.75));
        assert(near(r.elevationPulseUs, 1700.0));
    }
    // Interpolation at a non-midpoint fraction.
    {
        auto r = ShotTableLookup::lookup(3.25, kTable, kCount);
        assert(near(r.flywheelCommand, 0.825));
        assert(near(r.elevationPulseUs, 1550.0));
    }

    // Below the first entry: clamp, do not extrapolate.
    {
        auto r = ShotTableLookup::lookup(0.5, kTable, kCount);
        assert(near(r.flywheelCommand, 0.70));
        assert(near(r.elevationPulseUs, 1800.0));
    }
    // Beyond the last entry: extrapolated along the last segment's trend
    // (3.0 -> 4.0 m: +0.10 flywheel, -200 us per m). 4.5 m -> 0.95, 1300 us.
    {
        auto r = ShotTableLookup::lookup(4.5, kTable, kCount, true);
        assert(near(r.flywheelCommand, 0.95));
        assert(near(r.elevationPulseUs, 1300.0));
    }
    // ...or held at the last entry (EXTRAPOLATE_PAST_LAST_ENTRY false).
    {
        auto r = ShotTableLookup::lookup(4.5, kTable, kCount, false);
        assert(near(r.flywheelCommand, 0.90));
        assert(near(r.elevationPulseUs, 1400.0));
    }
    // ...but the flywheel command never goes past 100% (5 m would be 1.0,
    // 20 m would be 2.5). The elevation keeps going; Shooter clamps it to
    // the servo's limits.
    {
        auto r = ShotTableLookup::lookup(20.0, kTable, kCount, true);
        assert(near(r.flywheelCommand, 1.0));
        assert(near(r.elevationPulseUs, 1400.0 - 200.0 * 16.0));
    }
    // A null table is a no-op too.
    {
        auto r = ShotTableLookup::lookup(5.0, nullptr, 3);
        assert(near(r.flywheelCommand, 0.0) && near(r.elevationPulseUs, 0.0));
    }

    // count <= 1 is a defensive no-op case.
    {
        auto r = ShotTableLookup::lookup(5.0, kTable, 0);
        assert(near(r.flywheelCommand, 0.0));
        assert(near(r.elevationPulseUs, 0.0));
    }
    {
        auto r = ShotTableLookup::lookup(5.0, kTable, 1);
        assert(near(r.flywheelCommand, ShotTableLookup::flywheel(kTable[0])));
        assert(near(r.elevationPulseUs, ShotTableLookup::elevation(kTable[0])));
    }

    // The generated tables (tools/shot_table): every hole that resolves to a
    // table is sorted and in range; the midpoint of its first two entries
    // interpolates to their average.
    {
        using namespace TuningConstants::ShotTable;
        for (int id = 4; id <= 12; id++) {
            const Table t = tableForVisionId(id);
            if (t.entries == nullptr) continue;
            assert(t.count >= 1);
            for (int i = 0; i < t.count; i++) {
                if (i + 1 < t.count) {
                    assert(t.entries[i].distanceCm < t.entries[i + 1].distanceCm);
                }
                assert(t.entries[i].flywheelPermille <= 1000);
                assert(t.entries[i].elevationUs >= 500 && t.entries[i].elevationUs <= 2500);
            }
            if (t.count < 2) continue;
            const double mid = (ShotTableLookup::meters(t.entries[0]) + ShotTableLookup::meters(t.entries[1])) / 2.0;
            auto r = ShotTableLookup::lookup(mid, t.entries, t.count);
            assert(fabs(r.flywheelCommand - (ShotTableLookup::flywheel(t.entries[0]) +
                                             ShotTableLookup::flywheel(t.entries[1])) / 2.0) < 1e-6);
        }
        // Bonus holes: no data yet, no table.
        for (int id : {1, 2, 3}) assert(tableForVisionId(id).entries == nullptr);
    }

    // lookupTarget: the id's table plus that hole's elevation trim; no
    // table for id 0, the bonus holes 1-3 or unknown ids.
    {
        using namespace TuningConstants::ShotTable;
        for (int id = 4; id <= 12; id++) {
            ShotTableLookup::Result r = {0.0, 0.0};
            assert(ShotTableLookup::lookupTarget(id, 4.0, r));
            const Table t = tableForVisionId(id);
            const auto plain = ShotTableLookup::lookup(4.0, t.entries, t.count);
            assert(near(r.flywheelCommand, plain.flywheelCommand));
            assert(near(r.elevationPulseUs, plain.elevationPulseUs + ELEVATION_TRIM_US[id]));
        }
        ShotTableLookup::Result r = {0.0, 0.0};
        assert(!ShotTableLookup::lookupTarget(0, 4.0, r));
        assert(!ShotTableLookup::lookupTarget(2, 4.0, r));
        assert(!ShotTableLookup::lookupTarget(13, 4.0, r));
    }

    printf("shot_table_lookup_test: all asserts passed\n");
    return 0;
}
