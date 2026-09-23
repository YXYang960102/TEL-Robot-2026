#pragma once
#include <stdint.h>
namespace UartBareBenchConstants {
constexpr uint8_t RX_PIN = 19;
constexpr unsigned long BAUD = 115200;
constexpr uint32_t PHASE_MS = 10000;
constexpr uint32_t TX_PERIOD_MS = 200;
constexpr uint32_t REPORT_MS = 1000;
constexpr uint8_t SAMPLE_SIZE = 16;
constexpr char PROBE[] = "MEGA_HEARTBEAT,1\r\n";
}
