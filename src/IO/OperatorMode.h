#pragma once

// Top-level mode selected by the operator's keypad over the ELRS/CRSF
// mechanism link (MechLink; see src/IO/MechLink.h -- this no longer goes
// through SBUS). Rotate's vision-follow logic treats FULL_AUTO and
// SEMI_AUTO identically — both auto-aim continuously. Semi-auto only
// changes whether firing needs an explicit button press before it
// executes; that trigger/flywheel logic doesn't exist yet and is out of
// scope here. Only FULL_MANUAL disables auto-aim and hands the axis back
// to direct keypad/joystick control.
enum class OperatorMode { FULL_AUTO, SEMI_AUTO, FULL_MANUAL };

class OperatorModeSelector {
public:
    // Loss of RC signal always resolves to FULL_MANUAL — losing the link
    // must never leave an axis armed for autonomous motion.
    static OperatorMode current();
};
