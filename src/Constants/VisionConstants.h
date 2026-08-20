#pragma once

namespace VisionConst {

const double Y_RANGE_SET[] = {
  96.1,108.3,135.6,189.5,157,122,170.5
};

const int Y_COUNT = sizeof(Y_RANGE_SET)/sizeof(Y_RANGE_SET[0]);

const double Td = 0.08;
const double tau = 0.10;
const double VX_MAX = 800;
const double X_MAX = 400;
const double TY_MAX = 1000;
const double DISTANCE_MAX_MM = 20000;
const int TARGET_ID_MIN = 0;
const int TARGET_ID_MAX = 12;
const unsigned long SERIAL_BAUD = 115200;
const unsigned long PACKET_TIMEOUT_MS = 300;
const size_t PACKET_MAX_CHARS = 80;

}
