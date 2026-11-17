#pragma once

#include <stdint.h>

#include "RcControlMap.h"

enum class OperatorMode : uint8_t {
    FULL_AUTO = RCMAP_MODE_FULL_AUTO,
    SEMI_AUTO = RCMAP_MODE_SEMI_AUTO,
    FULL_MANUAL = RCMAP_MODE_FULL_MANUAL,
    TEST = RCMAP_MODE_TEST
};

inline bool isAutoMode(OperatorMode mode) {
    return mode == OperatorMode::SEMI_AUTO || mode == OperatorMode::FULL_AUTO;
}
