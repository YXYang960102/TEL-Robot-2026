// Host test for src/Vision/OperatorTarget.h: Dashboard hole (keys 1-9) ->
// Orin vision id, and which Orin frames may be used while a hole is
// selected. Jeremy's mapping (2026-10-16): keys 3/6/9 -> 4/5/6,
// 2/5/8 -> 7/8/9, 1/4/7 -> 10/11/12; vision 1..3 are bonus holes.
#include <assert.h>
#include <stdio.h>

#include <initializer_list>

#include "../src/Vision/OperatorTarget.h"

using OperatorTarget::frameMatchesSelection;
using OperatorTarget::NONE;
using OperatorTarget::visionIdForHole;

static bool cycleIs(uint8_t start, uint16_t excluded, const uint8_t* expected, uint8_t n) {
    uint8_t out[OperatorTarget::MAX_CYCLE];
    if (OperatorTarget::buildCycle(start, excluded, out) != n) return false;
    for (uint8_t i = 0; i < n; i++) if (out[i] != expected[i]) return false;
    return true;
}

int main() {
    // Full auto's order (2026-11-01): never two columns in one move.
    {
        const uint16_t none = 0;
        const uint16_t bottom = (1u << 1) | (1u << 4) | (1u << 7);
        const uint8_t from5[] = {5, 8, 7, 9, 6, 4, 1, 2, 3, 10, 11, 12};
        assert(cycleIs(5, none, from5, 12));
        const uint8_t from4[] = {4, 7, 8, 9, 6, 5, 2, 1, 3, 10, 11, 12};
        assert(cycleIs(4, none, from4, 12));
        const uint8_t from3[] = {3, 2, 1, 4, 5, 6, 9, 8, 7, 12, 11, 10};
        assert(cycleIs(3, none, from3, 12));
        const uint8_t from7[] = {7, 8, 9, 6, 5, 4, 1, 2, 3, 10, 11, 12};
        assert(cycleIs(7, none, from7, 12));
        const uint8_t fromW[] = {11, 12, 10};
        assert(cycleIs(11, none, fromW, 3));
        // The bottom row out: left out.
        const uint8_t from5b[] = {5, 8, 9, 6, 3, 2, 10, 11, 12};
        assert(cycleIs(5, bottom, from5b, 9));
        const uint8_t from9b[] = {9, 8, 5, 6, 3, 2, 10, 11, 12};
        assert(cycleIs(9, bottom, from9b, 9));
        const uint8_t from2b[] = {2, 3, 6, 5, 8, 9, 12, 11, 10};
        assert(cycleIs(2, bottom, from2b, 9));
        const uint8_t from4b[] = {4, 8, 9, 6, 5, 2, 3, 10, 11, 12}; // an excluded start stays first (Auto skips it)
        assert(cycleIs(4, bottom, from4b, 10));
        // Never two columns in one move, any start, either mask.
        for (uint8_t start = 1; start <= 9; start++) {
            for (uint16_t mask : {none, bottom}) {
                uint8_t cyc[OperatorTarget::MAX_CYCLE];
                const uint8_t n = OperatorTarget::buildCycle(start, mask, cyc);
                for (uint8_t i = 1; i < n; i++) {
                    const int dc = OperatorTarget::columnOf(cyc[i]) - OperatorTarget::columnOf(cyc[i - 1]);
                    assert(dc >= -1 && dc <= 1);
                }
            }
        }
        uint8_t out[OperatorTarget::MAX_CYCLE];
        assert(OperatorTarget::buildCycle(0, none, out) == 0 && OperatorTarget::buildCycle(13, none, out) == 0);
        // An excluded pick -> the nearest allowed hole in its column.
        assert(OperatorTarget::nearestAllowedInColumn(4, bottom) == 5);
        assert(OperatorTarget::nearestAllowedInColumn(1, bottom) == 2);
        assert(OperatorTarget::nearestAllowedInColumn(7, bottom) == 8);
        assert(OperatorTarget::nearestAllowedInColumn(9, (1u << 9) | (1u << 8)) == 7);
        assert(OperatorTarget::nearestAllowedInColumn(10, bottom) == NONE);
    }

    // Row by row as Jeremy listed it (top, middle, bottom; left to right).
    const unsigned keys[9] = {3, 6, 9, 2, 5, 8, 1, 4, 7};
    for (unsigned i = 0; i < 9; i++) {
        assert(visionIdForHole(static_cast<uint8_t>(keys[i])) == 4 + i);
    }
    // Every key maps to a distinct grid id; never a bonus hole (1..3).
    unsigned seen = 0;
    for (uint8_t k = 1; k <= 9; k++) {
        const uint8_t id = visionIdForHole(k);
        assert(id >= 4 && id <= 12);
        assert(!(seen & (1u << id)));
        seen |= 1u << id;
    }
    // The bonus holes (2026-10-27): Dashboard 10/11/12 -> vision 1/2/3.
    assert(visionIdForHole(10) == 1 && visionIdForHole(11) == 2 && visionIdForHole(12) == 3);
    // Nothing selected / out of range -> NONE (the Orin's own pick).
    assert(visionIdForHole(0) == NONE);
    assert(visionIdForHole(13) == NONE);
    assert(visionIdForHole(255) == NONE);

    // No selection: any frame is the Orin's pick, including bonus holes.
    for (long id = 0; id <= 12; id++) assert(frameMatchesSelection(NONE, id));
    // Selected hole: only exactly that id.
    assert(frameMatchesSelection(10, 10));
    for (long id = 0; id <= 12; id++) {
        if (id != 10) assert(!frameMatchesSelection(10, id));
    }
    assert(!frameMatchesSelection(10, -10));
    assert(!frameMatchesSelection(10, 266)); // no uint8 wrap-around

    printf("operator_target_test: all asserts passed\n");
    return 0;
}
