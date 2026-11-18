#include "Telemetry.h"

#include <Arduino.h>

#include "../Chassis/Chassis.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
#include "../Sensors/TurretAngle.h"
#include "../Auto/Auto.h"
#include "../IO/ManualControl.h"
#include "../Shooter/Shooter.h"
#include "../Vision/OperatorTarget.h"
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
    Serial.print(MechLink::channels[RCMAP_CH_FORWARD]);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_TURN]);
    Serial.print(",");
    Serial.print(MechLink::modeChannel);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_ARM]);
    Serial.print(",");
    Serial.print(MechLink::channels[RCMAP_CH_FIRE]);
    Serial.print(",");
    Serial.print(Shooter::isReady() ? 1 : 0);
    Serial.print(",");
    Serial.print(static_cast<int>(ManualControl::getOperatorMode()) + 1);
    Serial.print(",");
    Serial.print(warn);
    Serial.print(",");
    Serial.print("NaN,NaN,NaN,");
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
    Serial.print(Shooter::getFlywheelCommand() * 100.0, 1);
    Serial.print(",");
    Serial.print(Shooter::getAnglePulseUs());
    Serial.print(",");
    Serial.print(Shooter::getAngleTargetDegrees(), 1);
    Serial.print(",");
    Serial.print(Shooter::getAngleMinDegrees(), 1);
    Serial.print(",");
    Serial.print(Shooter::getAngleMaxDegrees(), 1);
    Serial.print(",");
    Serial.print(ManualControl::getTarget());
    Serial.print(",");
    Serial.print(ManualControl::getAutoPathThisEnable());
    Serial.print(",");
    Serial.print(ManualControl::getAutoPathsUsedMask());
    Serial.print(",");
    Serial.print(Vision::getSelectedTarget());
    Serial.print(",");
    Serial.print(ManualControl::getAimOffsetPx(), 0);
    Serial.print(",");
    Serial.print(ManualControl::getElevationOffsetUs(), 0);
    Serial.print(",");
    Serial.print(MechLink::lastLineAgeMs());
    Serial.print(",");
    Serial.print(MechLink::rejectedLines);
    Serial.print(",");
    Serial.print(Auto::getReadyBits());
    Serial.print(",");
    Serial.print(static_cast<long>(Auto::getShotDistanceM() * 100.0 + 0.5));
    Serial.print(",");
    Serial.print(TurretAngle::raw());
    Serial.print(",");
    Serial.print(TurretAngle::status());
    Serial.print(",");
    Serial.print(static_cast<long>(lround(TurretAngle::angleDegrees() * 10.0)));
    Serial.print(",");
    Serial.print(Vision::getSeenMask());
    Serial.print(",");
    Serial.print(Vision::isValid() ? 1 : (Vision::isPredicted() ? 2 : 0));
    Serial.print(",");
    Serial.print(Vision::getRingPercent());
    Serial.print(",");
    Serial.print(Vision::getRingSectors());
    Serial.print(",");
    Serial.print(Shooter::getTurretState());
    Serial.print(",");
    Serial.print(TurretAngle::agc());
    Serial.print(",");
    Serial.print(TurretAngle::magnitude());
    Serial.print(",");
    Serial.print(ManualControl::isTurretHeldHome() ? 1 : 0);
    Serial.print(",");
    Serial.print(Auto::getAimTarget());
    Serial.print(",");
    Serial.print(Auto::getRowShots());
    Serial.print(",");
    Serial.print(Auto::isRowDone() ? 1 : 0);
    Serial.print(",");
    Serial.print(OperatorTarget::holeForVisionId(Vision::getLitBonusId()));
    Serial.print(",");
    Serial.print(Auto::getLitPriorityHole());
    Serial.print(",");
    Serial.print(Shooter::getSearchDirection());
    Serial.print(",");
    Serial.print(Dribbler::hasBall() ? 1 : 0);
    Serial.print(",");
    Serial.print(DribblerConstants::Tower::READY_REQUIRED ? 1 : 0);
    Serial.print(",");
    Serial.print(Dribbler::getLoadedCount());
    Serial.print(",");
    Serial.print(Auto::getFireBlockers());
    Serial.print(",");
    Serial.println(Auto::getAimShots());
}
