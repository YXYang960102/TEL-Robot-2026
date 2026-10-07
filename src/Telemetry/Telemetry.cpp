#include "Telemetry.h"

#include <Arduino.h>

#include "../Auto/Auto.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
#include "../IO/OperatorMode.h"
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

    // Length-17 extension (base 14 fields + operator_mode,fire_state, no
    // pose -- this branch has no chassis odometry): see
    // tools/dashboard/README.md's parseTelemetry() length table.
    // Dashboard's "ch2"/"ch8" slots carry MechLink::fireChannel/modeChannel
    // here -- this branch never had separate ch2/ch8 values, those were
    // always just aliases for the same fire/mode signals.
    String line = "TEL,";
    line += now;
    line += ","; line += String(Vision::getTx(), 2);
    line += ","; line += String(Vision::getTy(), 2);
    line += ","; line += String(Vision::getDistance(), 2);
    line += ","; line += Vision::getTargetId();
    line += ","; line += Vision::isValid() ? 1 : 0;
    line += ","; line += MechLink::ch0;
    line += ","; line += MechLink::ch1;
    line += ","; line += MechLink::fireChannel;
    line += ","; line += MechLink::ch3;
    line += ","; line += MechLink::modeChannel;
    line += ","; line += Shooter::isReady() ? 1 : 0;
    line += ","; line += Dribbler::getShootRemaining();
    line += ","; line += warn;
    line += ","; line += static_cast<int>(OperatorModeSelector::current());
    line += ","; line += static_cast<int>(Auto::getFireState());

    Serial.println(line);
    MechLink::sendTelemetryLine(line);
}
