#include "Telemetry.h"

#include <Arduino.h>

#include "../Chassis/Chassis.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
#include "../IO/WiredControl.h"
#include "../Shooter/Shooter.h"
#include "../Vision/Vision.h"

unsigned long Telemetry::lastSendMs = 0;

void Telemetry::init() {
    lastSendMs = 0;
}

void Telemetry::update() {
    const unsigned long now = millis();
    if (now - lastSendMs < 100) {
        return;
    }
    lastSendMs = now;

    int warn = 0;
    if (!Vision::isValid()) {
        warn |= 1;
    }
    if (Vision::isValid() && Vision::getDistance() > 0 && Vision::getDistance() < 900) {
        warn |= 2;
    }
    if (Vision::isValid() && abs(Vision::getTx()) > 120) {
        warn |= 4;
    }

    Serial.print("TEL,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(Vision::getTx(), 2);
    Serial.print(",");
    Serial.print(Vision::getTy(), 2);
    Serial.print(",");
    Serial.print(Vision::getDistance(), 2);
    Serial.print(",");
    Serial.print(Vision::getTargetId());
    Serial.print(",");
    Serial.print(Vision::isValid() ? 1 : 0);
    Serial.print(",");
    // MechLink replaced SBUS (see src/IO/MechLink.h) -- it only exposes
    // ch0/ch1/ch3 plus the dedicated modeChannel/fireChannel, not a ch2/ch8
    // pair, so those two slots are repurposed rather than left as stale
    // SBUS field names (which no longer even compiled, see
    // docs/codex-handoff.md 2026-10-11).
    Serial.print(MechLink::ch0);
    Serial.print(",");
    Serial.print(MechLink::ch1);
    Serial.print(",");
    Serial.print(MechLink::modeChannel);
    Serial.print(",");
    Serial.print(MechLink::ch3);
    Serial.print(",");
    Serial.print(MechLink::fireChannel);
    Serial.print(",");
    Serial.print(Shooter::isReady() ? 1 : 0);
    Serial.print(",");
    Serial.print(Dribbler::getShootRemaining());
    Serial.print(",");
    Serial.print(warn);
    Serial.print(",");
    // robotX,robotY,heading: no odometry source exists yet. Printed as
    // literal NaN (not left empty) so the dashboard's
    // Number.isFinite(...) pose check reads false, not a false "at 0,0".
    Serial.print("NaN,NaN,NaN,");
    // Wired Dashboard + Chassis/Dribbler/Shooter manual-control fields,
    // added for the 2026-10-11 dev integration (docs/codex-handoff.md).
    Serial.print(WiredControl::isArmed() ? 1 : 0);
    Serial.print(",");
    Serial.print(Chassis::getLeftPulseUs());
    Serial.print(",");
    Serial.print(Chassis::getRightPulseUs());
    Serial.print(",");
    Serial.print(static_cast<int>(Dribbler::getControlMode()));
    Serial.print(",");
    Serial.print(Dribbler::isFeeding() ? 1 : 0);
    Serial.print(",");
    Serial.print(Dribbler::isExitSensorBlocked() ? 1 : 0);
    Serial.print(",");
    Serial.print(static_cast<int>(Shooter::getRotateControlMode()));
    Serial.print(",");
    Serial.print(static_cast<int>(Shooter::getAngleControlMode()));
    Serial.print(",");
    Serial.print(Shooter::getFlywheelPulseUs());
    Serial.print(",");
    // Command ratio, not RPM -- this Falcon 500 has no speed feedback on
    // this PWM path (see Shooter.h's getFlywheelCommand() comment).
    Serial.println(Shooter::getFlywheelCommand() * 100.0, 1);
}
