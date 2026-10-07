#include "Telemetry.h"

#include <Arduino.h>

#include "../Auto/Auto.h"
#include "../Constants/Mode.h"
#include "../Dribbler/Dribbler.h"
#include "../IO/MechLink.h"
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

    String line = "TEL,";
    line += now;
    line += ",";
    line += String(Vision::getTx(), 2);
    line += ",";
    line += String(Vision::getTy(), 2);
    line += ",";
    line += String(Vision::getDistance(), 2);
    line += ",";
    line += Vision::getTargetId();
    line += ",";
    line += Vision::isValid() ? 1 : 0;
    line += ",";
    line += MechLink::getDriveForwardPulseUs();
    line += ",";
    line += MechLink::getAuxiliaryPulseUs();
    line += ",";
    line += MechLink::getMechanismPulseUs();
    line += ",";
    line += MechLink::getDriveTurnPulseUs();
    line += ",";
    line += MechLink::getModePulseUs();
    line += ",";
    line += Shooter::isReady() ? 1 : 0;
    line += ",";
    line += Dribbler::getShootRemaining();
    line += ",";
    line += warn;

    Serial.println(line);
    MechLink::sendTelemetryLine(line);

    // Not part of the TEL,... line the dashboard parses (that format is
    // fixed, see tools/dashboard/README.md) -- a separate human-readable
    // line so the operator can confirm over USB Serial, before a match,
    // that the starting-side switch was actually read and latched to the
    // side they expect.
    Serial.print("StartSide: ");
    Serial.print(Auto::getStartingSide() == StartingSide::RIGHT ? "RIGHT" : "LEFT");
    Serial.println(Auto::isStartingSideLatched() ? " (locked)" : " (pending)");
}
