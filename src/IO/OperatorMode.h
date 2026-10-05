#pragma once

// Top-level mode selected by the operator's keypad over SBUS (ELRS link).
// Rotate's vision-follow logic treats FULL_AUTO and SEMI_AUTO identically —
// both auto-aim continuously. Semi-auto only changes whether firing needs an
// explicit button press before it executes; that trigger/flywheel logic
// doesn't exist yet and is out of scope here. Only FULL_MANUAL disables
// auto-aim and hands the axis back to direct keypad/joystick control.
enum class OperatorMode { FULL_AUTO, SEMI_AUTO, FULL_MANUAL };

namespace OperatorModeConstants {
// Provisional: SBUS::modeChannel currently aliases ch8 (mapped 1500-2000).
// These thresholds split that range into thirds. The real keypad doesn't
// exist yet — re-measure its actual output against these once it's built
// and update here; nothing else needs to change.
constexpr int SEMI_AUTO_THRESHOLD = 1667;
constexpr int FULL_MANUAL_THRESHOLD = 1833;
}

class OperatorModeSelector {
public:
    // Loss of RC signal always resolves to FULL_MANUAL — losing the link
    // must never leave an axis armed for autonomous motion.
    static OperatorMode current();
};
