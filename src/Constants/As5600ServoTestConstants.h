#pragma once

#include <Arduino.h>

namespace As5600ServoTestConst {

namespace Encoder {
const int COUNTS_PER_REV = 4096;
const int HALF_COUNTS_PER_REV = COUNTS_PER_REV / 2;
const double COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REV;
const int MAX_ACCEPTED_DELTA_COUNTS = 180;
}

namespace Servo {
const uint8_t SIGNAL_PIN = 9;
const int STOP_US = 1500;
const int CLOCKWISE_US = 1600;
const int COUNTER_CLOCKWISE_US = 1400;
const int MIN_US = 1000;
const int MAX_US = 2000;
const int MANUAL_OFFSET_US = 120;
}

namespace Control {
const long TARGET_RELATIVE_COUNTS = 1024;
const int STOP_TOLERANCE_COUNTS = 1;
const double KP = 0.18;
const double KI = 0.0;
const double KD = 0.02;
const int MAX_OUTPUT_OFFSET_US = 220;
const int MIN_MOVING_OFFSET_US = 80;
const double INTEGRAL_LIMIT = 400.0;
const int OUTPUT_SIGN = 1;
}

namespace SerialLog {
const unsigned long PRINT_INTERVAL_MS = 100;
}

}
