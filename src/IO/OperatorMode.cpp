#include "OperatorMode.h"
#include "SBUS.h"

OperatorMode OperatorModeSelector::current() {
    if (SBUS::signalLost) return OperatorMode::FULL_MANUAL;

    using namespace OperatorModeConstants;
    if (SBUS::modeChannel >= FULL_MANUAL_THRESHOLD) return OperatorMode::FULL_MANUAL;
    if (SBUS::modeChannel >= SEMI_AUTO_THRESHOLD) return OperatorMode::SEMI_AUTO;
    return OperatorMode::FULL_AUTO;
}
