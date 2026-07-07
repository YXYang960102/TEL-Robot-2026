#include "Telemetry.h"

#include <Arduino.h>

#include "../Dribbler/Dribbler.h"
#include "../IO/SBUS.h"
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
    Serial.print(SBUS::ch0);
    Serial.print(",");
    Serial.print(SBUS::ch1);
    Serial.print(",");
    Serial.print(SBUS::ch2);
    Serial.print(",");
    Serial.print(SBUS::ch3);
    Serial.print(",");
    Serial.print(SBUS::ch8);
    Serial.print(",");
    Serial.print(Shooter::isReady() ? 1 : 0);
    Serial.print(",");
    Serial.print(Dribbler::getShootRemaining());
    Serial.print(",");
    Serial.println(warn);
}
