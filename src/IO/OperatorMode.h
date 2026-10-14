#pragma once

#include <stdint.h>

#include "RcControlMap.h"

// Operator mode, chosen with M (wired Dashboard) or CH4 (wireless ground
// station) while armed; src/IO/ManualControl.cpp owns it and Auto reads it.
//   FULL_MANUAL: the operator drives everything (F fires the tower directly).
//   SEMI_AUTO:   turret follows the camera, flywheel from the shot table;
//                F fires once the turret is locked and the flywheel is up.
//   FULL_AUTO:   same aiming, and it fires by itself whenever ready.
// Chassis and elevation stay with the operator in every mode. Disarm, link
// loss and every new Enable go back to FULL_MANUAL.
//
// Values match the RCMAP_MODE_* ids carried on CH4 and in the status uplink.
enum class OperatorMode : uint8_t {
    FULL_AUTO = RCMAP_MODE_FULL_AUTO,
    SEMI_AUTO = RCMAP_MODE_SEMI_AUTO,
    FULL_MANUAL = RCMAP_MODE_FULL_MANUAL
};
