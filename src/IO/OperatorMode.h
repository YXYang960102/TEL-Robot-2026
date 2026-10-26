#pragma once

#include <stdint.h>

#include "RcControlMap.h"

// Operator mode, chosen with M (wired Dashboard) or CH4 (wireless ground
// station) while armed; src/IO/ManualControl.cpp owns it and Auto reads it.
//   FULL_MANUAL: the operator drives everything (F fires the tower directly).
//   SEMI_AUTO:   turret follows the camera, flywheel from the shot table;
//                F fires once the turret is locked and the flywheel is up.
//   FULL_AUTO:   same aiming, and it fires by itself whenever ready.
//   TEST:        (2026-10-26, for measuring the shot table) the turret
//                follows the camera like the autos; elevation, flywheel
//                and F stay the operator's, exactly as in full manual.
// Chassis stays with the operator in every mode. Disarm, link loss and
// every new Enable go back to FULL_MANUAL.
//
// Values match the RCMAP_MODE_* ids (CH4 + CH9 on the radio, the status
// uplink, TEL slot 13).
enum class OperatorMode : uint8_t {
    FULL_AUTO = RCMAP_MODE_FULL_AUTO,
    SEMI_AUTO = RCMAP_MODE_SEMI_AUTO,
    FULL_MANUAL = RCMAP_MODE_FULL_MANUAL,
    TEST = RCMAP_MODE_TEST
};

// SEMI_AUTO / FULL_AUTO: Auto runs the flywheel (shot table) and the fire
// decision. Not TEST: there the operator does.
inline bool isAutoMode(OperatorMode mode) {
    return mode == OperatorMode::SEMI_AUTO || mode == OperatorMode::FULL_AUTO;
}
