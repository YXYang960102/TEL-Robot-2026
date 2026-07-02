#ifndef SHOOTER_CONSTANTS_H
#define SHOOTER_CONSTANTS_H

namespace ShooterConst {

// MG996 360-degree continuous rotation servo for the shooter turret test branch.
// 1500us is stop. Values below/above 1500us rotate in opposite directions.
const int MG996_STOP_US = 1500;
const int MG996_MIN_US = 1300;
const int MG996_MAX_US = 1700;
const int MG996_MAX_SPEED_OFFSET_US = 180;
const int MG996_AIM_DEADBAND = 10;

const int HORI_MIN = 90;
const int HORI_MAX = 500;

const int VER_MIN = 1200;
const int VER_MAX = 3500;

const int HALF_AUTO_VER[4][8] = {
  {1212,1200,1212,1212,1212,1212,1212},
  {1915,1497,1374,1625,1410,1212,1212},
  {1957,2230,2075,2684,2326,1608,2307},
  {2515,2635,2667,3492,2918,2380,2918}
};

}
#endif
