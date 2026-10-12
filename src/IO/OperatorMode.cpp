#include "OperatorMode.h"
#include "MechLink.h"
#include "../Constants/TuningConstants.h"

OperatorMode OperatorModeSelector::current() {
    if (MechLink::signalLost) return OperatorMode::FULL_MANUAL;

    using namespace TuningConstants::OperatorMode;
    if (MechLink::modeChannel >= FULL_MANUAL_THRESHOLD) return OperatorMode::FULL_MANUAL;
    if (MechLink::modeChannel >= SEMI_AUTO_THRESHOLD) return OperatorMode::SEMI_AUTO;
    return OperatorMode::FULL_AUTO;
}
