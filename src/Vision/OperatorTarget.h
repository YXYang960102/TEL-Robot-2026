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

constexpr uint8_t MAX_CYCLE = 12;

// The cycle from `start` into out[] (returns its length). Rows whose three
// holes are all in excludedHoles (bit n = hole n, e.g. full auto's
// TuningConstants::Auto::FULL_AUTO_EXCLUDED_HOLES) are dropped and the
// snake carries on from the row before, so moves stay one hole: with the
// bottom row out, from 5: 5 8 2, 3 6 9, E W Q. Single excluded holes in a
// kept row stay listed (Auto skips them like holes without a table).
inline uint8_t buildCycle(uint8_t start, uint16_t excludedHoles, uint8_t out[MAX_CYCLE]) {
    if (start < 1 || start > LAST_HOLE) return 0;
    const uint8_t startRow = rowOf(start);
    uint8_t rows[4];
    uint8_t rowCount = 0;
    const uint8_t candidates = startRow == 3 ? 1 : 4;
    for (uint8_t i = 0; i < candidates; i++) {
        const uint8_t row = startRow == 3 ? 3 : cycleRow(startRow, i);
        bool anyAllowed = i == 0; // the start row always
        for (uint8_t c = 0; c < 3; c++) {
            if (!(excludedHoles & (1u << holeAt(row, c)))) anyAllowed = true;
        }
        if (anyAllowed) rows[rowCount++] = row;
    }
    uint8_t count = 0;
    const uint8_t c0 = columnOf(start);
    // Row 0: middle -> 1 2 0, left -> 0 1 2, right -> 2 1 0.
    for (uint8_t j = 0; j < 3; j++) {
        const uint8_t col = c0 == 1 ? static_cast<uint8_t>((1 + j) % 3)
                          : c0 == 0 ? j : static_cast<uint8_t>(2 - j);
        out[count++] = holeAt(rows[0], col);
    }
    uint8_t endColumn = c0 == 0 ? 2 : 0;
    // Later rows: from the column the previous one ended in, across.
    for (uint8_t k = 1; k < rowCount; k++) {
        for (uint8_t j = 0; j < 3; j++) {
            out[count++] = holeAt(rows[k], endColumn == 0 ? j : static_cast<uint8_t>(2 - j));
        }
        endColumn = static_cast<uint8_t>(2 - endColumn);
    }
    return count;
}

// The hole to start from instead of an excluded one the Orin picked: the
// same column, the nearest allowed grid row (above first); NONE if none.
// With the bottom row out: 4 -> 5, 1 -> 2, 7 -> 8.
inline uint8_t nearestAllowedInColumn(uint8_t hole, uint16_t excludedHoles) {
    if (hole < 1 || hole > 9) return NONE;
    const uint8_t row = rowOf(hole);
    const uint8_t col = columnOf(hole);
    const int8_t order[4] = {1, 2, -1, -2};
    for (uint8_t i = 0; i < 4; i++) {
        const int r = row + order[i];
        if (r < 0 || r > 2) continue;
        const uint8_t candidate = holeAt(static_cast<uint8_t>(r), col);
        if (!(excludedHoles & (1u << candidate))) return candidate;
    }
    return NONE;
}

static_assert(holeForVisionId(10) == 1 && holeForVisionId(8) == 5 && holeForVisionId(2) == 11 &&
              holeForVisionId(0) == NONE && holeForVisionId(13) == NONE,
              "inverse of visionIdForHole");

}  // namespace OperatorTarget
