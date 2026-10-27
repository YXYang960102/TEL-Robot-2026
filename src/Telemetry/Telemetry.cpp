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
    // Slots 7-11: raw wireless channels (table v2 in src/IO/RcControlMap.h):
    // forward, turn, operator mode, arm, fire.
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
    // Slot 13 (was the retired shot-queue count, always 0): operator mode
    // + 1 -- 1 = full auto, 2 = semi auto, 3 = full manual (OperatorMode.h).
    // The +1 keeps an older firmware's 0 from reading as "full auto".
    Serial.print(static_cast<int>(ManualControl::getOperatorMode()) + 1);
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
    Serial.print(Shooter::getFlywheelCommand() * 100.0, 1);
    Serial.print(",");
    // Slot 28 (appended 2026-10-15): elevation servo target pulse, for
    // finding the mechanism's real end points. Slots 29-31: the same target
    // as an angle, and the reachable angle range (Dashboard slider limits).
    Serial.print(Shooter::getAnglePulseUs());
    Serial.print(",");
    Serial.print(Shooter::getAngleTargetDegrees(), 1);
    Serial.print(",");
    Serial.print(Shooter::getAngleMinDegrees(), 1);
    Serial.print(",");
    Serial.print(Shooter::getAngleMaxDegrees(), 1);
    Serial.print(",");
    // Slots 32-34 (appended 2026-10-16): selected target hole (0 = none,
    // 1..9), the auto path started this Enable (0 = none, 1..4), and a bit
    // mask of every auto path started since boot (bit n = path n).
    Serial.print(ManualControl::getTarget());
    Serial.print(",");
    Serial.print(ManualControl::getAutoPathThisEnable());
    Serial.print(",");
    Serial.print(ManualControl::getAutoPathsUsedMask());
    Serial.print(",");
    // Slots 35-37 (appended 2026-10-16, Orin protocol v2): the hole as sent
    // to the Orin (vision id 0 / 4..12), and the operator's fine-adjust
    // (aim px, + = right; elevation us). Slots 5/6 (target id / valid)
    // already only show the selected hole's frame.
    Serial.print(Vision::getSelectedTarget());
    Serial.print(",");
    Serial.print(ManualControl::getAimOffsetPx(), 0);
    Serial.print(",");
    Serial.print(ManualControl::getElevationOffsetUs(), 0);
    Serial.print(",");
    // Slot 38 (appended 2026-10-17): ms since the last valid "RC,..." line
    // from the robot STM32 bridge, so the real ELRS/bridge update rate can
    // be read live (Dashboard) instead of assumed -- compare against
    // MechLink's own LINE_TIMEOUT_MS (200) and
    // ManualControlConstants::COMMAND_TIMEOUT_MS (250). Bench-check this
    // with the chassis output-disabled or the drive motors unpowered.
    Serial.print(MechLink::lastLineAgeMs());
    Serial.print(",");
    // Slot 39 (appended 2026-10-27): lines from the bridge dropped since boot
    // (bad "*HH" check, malformed, oversize). Should stay at or near 0.
    Serial.println(MechLink::rejectedLines);
}
