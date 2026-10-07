#pragma once

enum RobotMode {
    MANUAL = 0,
    SEMI_AUTO = 1,
    AUTO = 2
};

// Which side of the field this robot starts on, latched once at boot from
// the operator's keypad (see src/Auto/Auto.h). Feeds the future field-zone/
// pose logic docs/autonomous-zone-strategy.md describes as its first step
// ("choose left or right start box") -- that full system isn't built yet,
// this only latches and exposes the choice.
enum class StartingSide { LEFT, RIGHT };
