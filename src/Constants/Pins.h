#pragma once

#include "BuildProfile.h"
#include "DribblerConstants.h"

#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
#if defined(ARDUINO)
#error "TEL_HOST_TEST_PIN_TABLE_CONFIRMED is for host tests only"
#endif
constexpr bool PIN_TABLE_CONFIRMED = TEL_HOST_TEST_PIN_TABLE_CONFIRMED;
#else
constexpr bool PIN_TABLE_CONFIRMED = true;
#endif

#define MECH_LINK_SERIAL_PORT Serial2
