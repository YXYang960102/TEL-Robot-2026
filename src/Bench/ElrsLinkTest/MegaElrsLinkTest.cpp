// ELRS downlink bench test (env: mega_elrs_link_test).
// Ground STM32 -> ES900TX ))) ES900RX -> robot STM32 bridge -> Mega Serial2.
// Uses the production MechLink parser (src/IO/MechLink.cpp) unchanged, so
// what this prints is exactly what the robot build would see. No motors or
// other subsystems are built in.
//
// Prints a $ELRS line 4x per second with the decoded channels, and an event
// whenever MechLink's link state flips, with how long the previous state
// lasted. The robot bridge goes silent when RF is lost, so turning the
// ground side off should produce LINK_LOST within ~0.7 s.
#include <Arduino.h>

#include "../../IO/MechLink.h"

namespace {

constexpr unsigned long PRINT_PERIOD_MS = 250;

bool lastLost = true;
unsigned long stateSinceMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lostCount = 0;
int lastCh0 = 1500;
unsigned long ch0Changes = 0;

void printState(unsigned long now) {
    Serial.print(F("$ELRS,t="));
    Serial.print(now);
    Serial.print(F(",link="));
    Serial.print(MechLink::signalLost ? F("LOST") : F("OK"));
    Serial.print(F(",fwd="));
    Serial.print(MechLink::ch0);
    Serial.print(F(",turn="));
    Serial.print(MechLink::ch1);
    Serial.print(F(",aux="));
    Serial.print(MechLink::ch3);
    Serial.print(F(",fire="));
    Serial.print(MechLink::fireChannel);
    Serial.print(F(",mode="));
    Serial.print(MechLink::modeChannel);
    Serial.print(F(",fwdChanges="));
    Serial.println(ch0Changes);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    MechLink::init();
    stateSinceMs = millis();
    Serial.println(F("$ELRS_BOOT mega_elrs_link_test: MechLink on Serial2 D16/D17 @115200"));
    Serial.println(F("$ELRS_BOOT link=LOST until the first valid CH line arrives"));
}

void loop() {
    MechLink::update();
    const unsigned long now = millis();

    if (MechLink::signalLost != lastLost) {
        Serial.print(MechLink::signalLost ? F("$ELRS_EVENT,LINK_LOST,t=") : F("$ELRS_EVENT,LINK_OK,t="));
        Serial.print(now);
        Serial.print(F(",previous_state_ms="));
        Serial.println(now - stateSinceMs);
        if (MechLink::signalLost) lostCount++;
        lastLost = MechLink::signalLost;
        stateSinceMs = now;
    }

    if (!MechLink::signalLost && MechLink::ch0 != lastCh0) {
        ch0Changes++;
        lastCh0 = MechLink::ch0;
    }

    if (now - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = now;
        printState(now);
    }
}
