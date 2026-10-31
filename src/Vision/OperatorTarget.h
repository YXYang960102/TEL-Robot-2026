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

// Full auto's shooting order (Auto's cycle, 2026-10-31, Jeremy): a snake,
// so each move is one hole. The first row from the first hole: from the
// middle column right then the far left (5 8 2), from an edge column
// straight across (1 4 7, 9 6 3). Each next row starts in the column the
// previous one ended in -- the hole right above / below -- and sweeps
// across: rows in order start, the adjacent row below first, the other,
// then the bonus row. From 5: 5 8 2, 1 4 7, 9 6 3, Q W E (12 holes).
// Starting on a bonus hole: the bonus row only (3 holes).
// Grid key k: column (k-1)/3 (0 left), row (k-1)%3 (0 bottom); row 3 = bonus.
constexpr uint8_t rowOf(uint8_t hole) {
    return hole >= FIRST_BONUS_HOLE ? 3 : static_cast<uint8_t>((hole - 1) % 3);
}
constexpr uint8_t columnOf(uint8_t hole) {
    return hole >= FIRST_BONUS_HOLE ? static_cast<uint8_t>(hole - FIRST_BONUS_HOLE)
                                    : static_cast<uint8_t>((hole - 1) / 3);
}
constexpr uint8_t holeAt(uint8_t row, uint8_t column) {
    return row == 3 ? static_cast<uint8_t>(FIRST_BONUS_HOLE + column)
                    : static_cast<uint8_t>(1 + row + 3 * column);
}
// The i-th row visited from a grid start row: start, adjacent below
// first, then the rest; i = 3 is the bonus row.
constexpr uint8_t cycleRow(uint8_t startRow, uint8_t i) {
    return i == 3 ? 3
         : i == 0 ? startRow
         : startRow == 1 ? (i == 1 ? 0 : 2)
         : startRow == 0 ? i
                         : static_cast<uint8_t>(2 - i);
}
// Column of the j-th hole (0..2) of the k-th row from start column c0.
// Row 0: middle -> 1 2 0, left -> 0 1 2, right -> 2 1 0; it ends at the
// left edge unless it started there. Later rows start where the previous
// ended (rows 1, 3: that edge; row 2: the other) and sweep across.
constexpr uint8_t firstRowEndColumn(uint8_t c0) {
    return c0 == 0 ? 2 : 0;
}
constexpr uint8_t cycleColumn(uint8_t c0, uint8_t k, uint8_t j) {
    return k == 0 ? (c0 == 1 ? static_cast<uint8_t>((1 + j) % 3)
                    : c0 == 0 ? j
                              : static_cast<uint8_t>(2 - j))
         : ((k % 2 == 1 ? firstRowEndColumn(c0) : static_cast<uint8_t>(2 - firstRowEndColumn(c0))) == 0
                ? j
                : static_cast<uint8_t>(2 - j));
}
constexpr uint8_t cycleLength(uint8_t start) {
    return (start >= FIRST_BONUS_HOLE && start <= LAST_HOLE) ? 3 : (start >= 1 && start <= 9) ? 12 : 0;
}
constexpr uint8_t cycleHole(uint8_t start, uint8_t index) {
    return index >= cycleLength(start) ? NONE
         : holeAt(rowOf(start) == 3 ? 3 : cycleRow(rowOf(start), static_cast<uint8_t>(index / 3)),
                  cycleColumn(columnOf(start), static_cast<uint8_t>(index / 3), static_cast<uint8_t>(index % 3)));
}

static_assert(holeForVisionId(10) == 1 && holeForVisionId(8) == 5 && holeForVisionId(2) == 11 &&
              holeForVisionId(0) == NONE && holeForVisionId(13) == NONE,
              "inverse of visionIdForHole");
static_assert(cycleHole(5, 0) == 5 && cycleHole(5, 1) == 8 && cycleHole(5, 2) == 2 &&
              cycleHole(5, 3) == 1 && cycleHole(5, 4) == 4 && cycleHole(5, 5) == 7 &&
              cycleHole(5, 6) == 9 && cycleHole(5, 7) == 6 && cycleHole(5, 8) == 3 &&
              cycleHole(5, 9) == 10 && cycleHole(5, 10) == 11 && cycleHole(5, 11) == 12 &&
              cycleHole(5, 12) == NONE,
              "from 5: 5 8 2, 1 4 7, 9 6 3, Q W E -- one hole per move");
static_assert(cycleHole(4, 0) == 4 && cycleHole(4, 2) == 1 && cycleHole(4, 3) == 2 &&
              cycleHole(4, 5) == 8 && cycleHole(4, 6) == 9 && cycleHole(4, 8) == 3 &&
              cycleHole(4, 9) == 10,
              "from 4: 4 7 1, 2 5 8, 9 6 3, Q W E");
static_assert(cycleHole(3, 0) == 3 && cycleHole(3, 2) == 9 && cycleHole(3, 3) == 8 &&
              cycleHole(3, 5) == 2 && cycleHole(3, 6) == 1 && cycleHole(3, 8) == 7 &&
              cycleHole(3, 9) == 12 && cycleHole(3, 11) == 10,
              "from 3: 3 6 9, 8 5 2, 1 4 7, E W Q");
static_assert(cycleHole(7, 0) == 7 && cycleHole(7, 2) == 1 && cycleHole(7, 3) == 2 &&
              cycleHole(7, 6) == 9,
              "from 7: 7 4 1, 2 5 8, 9 6 3");
static_assert(cycleHole(11, 0) == 11 && cycleHole(11, 1) == 12 && cycleHole(11, 2) == 10 &&
              cycleHole(11, 3) == NONE && cycleHole(0, 0) == NONE,
              "a bonus start: the bonus row only");

}  // namespace OperatorTarget
