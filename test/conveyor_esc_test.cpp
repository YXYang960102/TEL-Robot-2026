#include <cassert>
#include "../src/Dribbler/Dribbler.h"
int main() {
 hoststub::digitalOut[DribblerConstants::Tower::TRIGGER_PIN] = 7;
 Dribbler::init();
 assert(Servo::lastPin == 45 && Servo::lastPulse == 1500);
 Dribbler::setConveyor(true);
 assert(Dribbler::isConveyorOn() && Servo::lastPulse == 1750);
 Dribbler::setConveyor(false);
 assert(!Dribbler::isConveyorOn() && Servo::lastPulse == 1500);
 Dribbler::setConveyor(true);
 Dribbler::stop();
 assert(!Dribbler::isConveyorOn() && Servo::lastPulse == 1500);
#ifdef TEL_DRIVE_CONVEYOR_ONLY
 assert(!Dribbler::requestFeed());
 assert(hoststub::digitalOut[DribblerConstants::Tower::TRIGGER_PIN] == 7);
#endif
}

