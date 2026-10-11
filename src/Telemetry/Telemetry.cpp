#include "Telemetry.h"

#include <Arduino.h>

#include "../Chassis/Chassis.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
#include "../IO/ManualControl.h"
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
    // The five legacy SBUS-era slots now carry the wireless channels the
    // operator actually controls (table in src/IO/RcControlMap.h): left
    // drive, right drive, operator mode, arm, F counter.
    Serial.print(MechLink::channels[RCMAP_CH_LEFT]);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_RIGHT]);
    Serial.print(",");
    Serial.print(MechLink::modeChannel);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_ARM]);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_FEED_COUNTER]);
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
    // Operator-control + Chassis/Dribbler/Shooter fields (docs/codex-handoff.md
    // 2026-10-11). Slot 18 = armed; slots 21-23 = feed tower trigger state
    // (it has no ball sensor feedback to report).
    Serial.print(ManualControl::isArmed() ? 1 : 0);
    Serial.print(",");
    Serial.print(Chassis::getLeftPulseUs());
    Serial.print(",");
    Serial.print(Chassis::getRightPulseUs());
    Serial.print(",");
    Serial.print(static_cast<int>(ManualControl::getOwner()));
    Serial.print(",");
    Serial.print(Dribbler::isBusy() ? 1 : 0);
    Serial.print(",");
    Serial.print(Dribbler::getTriggerCount());
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
