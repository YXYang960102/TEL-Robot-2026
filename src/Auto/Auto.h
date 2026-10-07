#pragma once

#include "../Constants/Mode.h"

// Latches which side of the field this robot starts on, once, from the
// operator's keypad (MechLink's auxiliary channel) -- see Mode.h's
// StartingSide and docs/autonomous-zone-strategy.md. This is the one
// piece of cross-subsystem orchestration that exists on this branch so
// far; same role Auto plays on the Shooter branch (a place for behavior
// that isn't owned by any single subsystem).
class Auto {
public:
    static void init();
    static void update();

    static StartingSide getStartingSide();
    static bool isStartingSideLatched();

private:
    static StartingSide startingSide;
    static bool latched;
};
