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

// Full auto's shooting order (Auto's cycle; Jeremy 2026-10-31 / 11-01).
// The turret only turns sideways, so the order never jumps two columns:
// a column is a move of the turret, a row only of the elevation. Columns
// from the first hole's: from an edge column straight across (left, middle,
// right or the reverse); from the middle column the first hole, then the
// right column, back to the rest of the middle, then the left -- never
// right -> left in one go. Inside a column, the holes nearest the height
// it came in at first (lower on a tie). Then the bonus row from the column
// the grid ended in. Excluded holes (bit n = hole n) are left out.
// Bottom row out, from 5: 5, 8 9, 6, 3 2, Q W E. From 2: 2 3, 6 5, 8 9,
// E W Q. All rows, from 5: 5, 8 7 9, 6 4, 1 2 3, Q W E.
// Starting on a bonus hole: the bonus row only (from W: W E Q).
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

constexpr uint8_t MAX_CYCLE = 12;

// The cycle from `start` into out[] (returns its length).
inline uint8_t buildCycle(uint8_t start, uint16_t excludedHoles, uint8_t out[MAX_CYCLE]) {
    if (start < 1 || start > LAST_HOLE) return 0;
    uint8_t count = 0;
    // A bonus row from column c0: middle -> 1 2 0, left -> 0 1 2, right -> 2 1 0.
    auto bonusRow = [&](uint8_t c0) {
        for (uint8_t j = 0; j < 3; j++) {
            const uint8_t col = c0 == 1 ? static_cast<uint8_t>((1 + j) % 3)
                              : c0 == 0 ? j : static_cast<uint8_t>(2 - j);
            const uint8_t hole = holeAt(3, col);
            if (hole == start || !(excludedHoles & (1u << hole))) out[count++] = hole;
        }
    };
    if (rowOf(start) == 3) {
        bonusRow(columnOf(start));
        return count;
    }
    out[count++] = start;
    uint8_t row = rowOf(start);
    // The grid holes of one column not yet listed, nearest `row` first.
    auto column = [&](uint8_t col) {
        const uint8_t entry = row;
        for (uint8_t d = 0; d <= 2; d++) {
            for (int8_t sign = -1; sign <= 1; sign += 2) {
                if (d == 0 && sign > 0) continue;
                const int r = entry + sign * d;
                if (r < 0 || r > 2) continue;
                const uint8_t hole = holeAt(static_cast<uint8_t>(r), col);
                if (excludedHoles & (1u << hole)) continue;
                bool listed = false;
                for (uint8_t i = 0; i < count; i++) listed = listed || out[i] == hole;
                if (listed) continue;
                out[count++] = hole;
                row = static_cast<uint8_t>(r);
            }
        }
    };
    const uint8_t c0 = columnOf(start);
    if (c0 == 1) {
        column(2);
        column(1);
        column(0);
    } else {
        column(c0);
        column(1);
        column(static_cast<uint8_t>(2 - c0));
    }
    bonusRow(columnOf(out[count - 1]));
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

// The Orin's detected-hole mask (bit vision id - 1) as Dashboard holes
// (bit n = hole n).
inline uint16_t holesFromVisionMask(uint16_t visionMask) {
    uint16_t holes = 0;
    for (uint8_t id = 1; id <= 12; id++) {
        if (visionMask & (1u << (id - 1))) holes = static_cast<uint16_t>(holes | (1u << holeForVisionId(id)));
    }
    return holes;
}

// Which way the turret should turn to bring a hole that is out of view into
// it (2026-11-01, Jeremy): -1 left, +1 right, 0 = don't know. The holes
// the camera does see say where it points: a target column left of all of
// them -> left. With none seen, the column of the hole last locked on
// (lastColumn, -1 = none). Example: the camera sees 7 8 9 (right column),
// the target is 1 (left column) -> -1.
inline int8_t searchDirection(uint8_t targetHole, uint16_t seenHoles, int8_t lastColumn) {
    if (targetHole < 1 || targetHole > LAST_HOLE) return 0;
    const int8_t target = static_cast<int8_t>(columnOf(targetHole));
    int8_t lowest = 3, highest = -1;
    for (uint8_t hole = 1; hole <= LAST_HOLE; hole++) {
        if (!(seenHoles & (1u << hole))) continue;
        const int8_t col = static_cast<int8_t>(columnOf(hole));
        if (col < lowest) lowest = col;
        if (col > highest) highest = col;
    }
    if (highest >= 0) {
        return target < lowest ? -1 : target > highest ? 1 : 0;
    }
    if (lastColumn < 0) return 0;
    return target < lastColumn ? -1 : target > lastColumn ? 1 : 0;
}

}  // namespace OperatorTarget
