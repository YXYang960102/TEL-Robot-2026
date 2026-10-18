// Host test for src/Vision/OperatorTarget.h: Dashboard hole (keys 1-9) ->
// Orin vision id, and which Orin frames may be used while a hole is
// selected. Jeremy's mapping (2026-10-16): keys 3/6/9 -> 4/5/6,
// 2/5/8 -> 7/8/9, 1/4/7 -> 10/11/12; vision 1..3 are bonus holes.
#include <assert.h>
#include <stdio.h>

#include "../src/Vision/OperatorTarget.h"

using OperatorTarget::frameMatchesSelection;
using OperatorTarget::NONE;
using OperatorTarget::visionIdForHole;

int main() {
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
    // Nothing selected / out of range -> NONE (the Orin's own pick).
    assert(visionIdForHole(0) == NONE);
    assert(visionIdForHole(10) == NONE);
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
