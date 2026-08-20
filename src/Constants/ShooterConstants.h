#pragma once

namespace ShooterConst {

namespace Encoder {
constexpr int COUNTS_PER_REV = 4096;
constexpr int HALF_COUNTS_PER_REV = COUNTS_PER_REV / 2;
constexpr double COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REV;
constexpr int MAX_ACCEPTED_DELTA_COUNTS = 180;
constexpr unsigned long SAMPLE_TIMEOUT_MS = 100;
}

namespace Elevation {
constexpr int LEFT_STOP_US = 1500;
constexpr int RIGHT_STOP_US = 1500;
constexpr int SERVO_ATTACH_MIN_US = 1000;
constexpr int SERVO_ATTACH_MAX_US = 2000;
constexpr int MAX_OUTPUT_OFFSET_US = 80;
constexpr int LEFT_OUTPUT_SIGN = 1;
constexpr int RIGHT_OUTPUT_SIGN = -1;
constexpr long MIN_TARGET_COUNTS = 0;
constexpr long MAX_TARGET_COUNTS = 1024;
constexpr long READY_TOLERANCE_COUNTS = 4;
constexpr double KP = 0.18;
constexpr double KI = 0.0;
constexpr double KD = 0.02;
constexpr int PID_SAMPLE_TIME_MS = 20;
}

namespace Turret {
constexpr int STOP_US = 1500;
constexpr int ATTACH_MIN_US = 1000;
constexpr int ATTACH_MAX_US = 2000;
constexpr int MAX_MANUAL_OFFSET_US = 120;
constexpr int OUTPUT_SIGN = 1;
}

namespace Flywheel {
constexpr int STOP_US = 1500;
constexpr int ATTACH_MIN_US = 1000;
constexpr int ATTACH_MAX_US = 2000;
constexpr int MAX_OPEN_LOOP_US = 2000;
}

const int HALF_AUTO_VER[4][8] = {
  {1212,1200,1212,1212,1212,1212,1212},
  {1915,1497,1374,1625,1410,1212,1212},
  {1957,2230,2075,2684,2326,1608,2307},
  {2515,2635,2667,3492,2918,2380,2918}
};

}
