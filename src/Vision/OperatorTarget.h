#pragma once

#include <stdint.h>

// The operator's chosen hole (Dashboard keys 1-9 / wireless CH3) in the
// Orin's numbering, and the rule for which Orin frames may be used.
// Pure, header-only: host-tested in test/operator_target_test.cpp.
//
// Two numberings of the same holes (Jeremy, 2026-10-16):
//   Dashboard hole / key k (1..9): columns left to right, each bottom-up
//     (1 left-bottom, 2 left-mid, 3 left-top ... 9 right-top).
//   Orin vision id (4..12): rows from the top, each left to right
//     (top 4 5 6, middle 7 8 9, bottom 10 11 12).
// The bonus small holes above the grid (2026-10-27, keys Q W E): Dashboard
// hole 10 / 11 / 12 = Orin vision id 1 / 2 / 3, left to right.
// This is the ONLY place that converts between them; the Orin does no
// second conversion (YOLO_Detect_single docs/operator-selection-v2.md).
namespace OperatorTarget {

constexpr uint8_t NONE = 0;
constexpr uint8_t FIRST_BONUS_HOLE = 10; // Dashboard holes 10..12
constexpr uint8_t LAST_HOLE = 12;

// Grid key k (1..9): col = (k-1)/3 from the left, rowFromBottom = (k-1)%3,
// vision id = 4 + (2 - rowFromBottom)*3 + col. Example: key 1 -> 10.
// Bonus hole 10..12 -> vision id 1..3. Anything else -> NONE (the Orin's
// automatic choice).
constexpr uint8_t visionIdForHole(uint8_t hole) {
    return (hole >= FIRST_BONUS_HOLE && hole <= LAST_HOLE)
               ? static_cast<uint8_t>(hole - FIRST_BONUS_HOLE + 1)
           : (hole < 1 || hole > 9)
               ? NONE
               : static_cast<uint8_t>(4 + (2 - (hole - 1) % 3) * 3 + (hole - 1) / 3);
}

// With a hole selected, only a frame for exactly that hole may be used:
// a frame for any other id (an old frame from before a switch, or an Orin
// that missed the selection) is treated as no target -- never aim or fire
// at a hole the operator didn't choose. NONE accepts the Orin's own pick.
constexpr bool frameMatchesSelection(uint8_t selectedVisionId, long frameTargetId) {
    return selectedVisionId == NONE || frameTargetId == selectedVisionId;
}

static_assert(visionIdForHole(1) == 10 && visionIdForHole(2) == 7 && visionIdForHole(3) == 4,
              "left column: bottom, middle, top");
static_assert(visionIdForHole(4) == 11 && visionIdForHole(5) == 8 && visionIdForHole(6) == 5,
              "middle column");
static_assert(visionIdForHole(7) == 12 && visionIdForHole(8) == 9 && visionIdForHole(9) == 6,
              "right column");
static_assert(visionIdForHole(10) == 1 && visionIdForHole(11) == 2 && visionIdForHole(12) == 3,
              "bonus row, left to right");
static_assert(visionIdForHole(13) == NONE, "nothing past the bonus row");

}  // namespace OperatorTarget
