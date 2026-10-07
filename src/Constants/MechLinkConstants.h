#pragma once

// Renamed from SBUSConstants.h -- RAW_MIN/RAW_MAX (the old raw SBUS tick
// range) are gone, since the robot-side STM32 (tools/stm32_elrs_robot_bridge)
// now sends pulse-microsecond values directly; nothing on the Mega side
// decodes raw SBUS ticks anymore. The pulse-us range and timeout are still
// meaningful and used by src/IO/MechLink.cpp.
namespace MechLinkConst {

constexpr int PULSE_MIN_US = 1000;
constexpr int PULSE_NEUTRAL_US = 1500;
constexpr int PULSE_MAX_US = 2000;
constexpr unsigned long FRAME_TIMEOUT_MS = 100;

}
