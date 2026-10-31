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

// Orin vision id -> Dashboard hole (the inverse; NONE for 0 / unknown).
constexpr uint8_t holeForVisionId(long visionId, uint8_t hole = 1) {
    return hole > LAST_HOLE ? NONE
         : visionIdForHole(hole) == visionId && visionId != NONE ? hole
         : holeForVisionId(visionId, static_cast<uint8_t>(hole + 1));
}

// The same row (same height), `step` holes to the right of `hole`,
// wrapping from the right column back to the left (Auto's row cycle,
// 2026-10-31). Grid key k: column (k-1)/3, row (k-1)%3; the bonus row
// 10 11 12. Example: 4 (middle bottom) -> step 1: 7, step 2: 1.
constexpr uint8_t rowNeighbor(uint8_t hole, uint8_t step) {
    return (hole >= FIRST_BONUS_HOLE && hole <= LAST_HOLE)
               ? static_cast<uint8_t>(FIRST_BONUS_HOLE + (hole - FIRST_BONUS_HOLE + step) % 3)
           : (hole < 1 || hole > 9)
               ? NONE
               : static_cast<uint8_t>(1 + (hole - 1) % 3 + 3 * (((hole - 1) / 3 + step) % 3));
}

static_assert(holeForVisionId(10) == 1 && holeForVisionId(8) == 5 && holeForVisionId(2) == 11 &&
              holeForVisionId(0) == NONE && holeForVisionId(13) == NONE,
              "inverse of visionIdForHole");
static_assert(rowNeighbor(4, 1) == 7 && rowNeighbor(4, 2) == 1 && rowNeighbor(1, 1) == 4 &&
              rowNeighbor(9, 1) == 3 && rowNeighbor(11, 1) == 12 && rowNeighbor(12, 1) == 10 &&
              rowNeighbor(5, 0) == 5 && rowNeighbor(0, 1) == NONE,
              "row neighbours: same height, left to right, wrapping");

}  // namespace OperatorTarget
