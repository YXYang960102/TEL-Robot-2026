#include "Auto.h"

#include "../Constants/TuningConstants.h"
#include "../IO/MechLink.h"

StartingSide Auto::startingSide = StartingSide::LEFT;
bool Auto::latched = false;

void Auto::init() {
    latched = false;
    startingSide = StartingSide::LEFT;
}

void Auto::update() {
    if (latched || !MechLink::isHealthy()) {
        // Wait for the first healthy MechLink frame after boot before
        // latching -- MechLink::init() runs in setup(), before any frame
        // has actually arrived, so reading the auxiliary channel at that
        // exact moment would only see its neutral default, not whatever
        // the operator has the switch set to.
        return;
    }

    startingSide = MechLink::getAuxiliaryPulseUs() >= TuningConstants::StartingSide::THRESHOLD_US
        ? StartingSide::RIGHT
        : StartingSide::LEFT;
    latched = true;
}

StartingSide Auto::getStartingSide() {
    return startingSide;
}

bool Auto::isStartingSideLatched() {
    return latched;
}
