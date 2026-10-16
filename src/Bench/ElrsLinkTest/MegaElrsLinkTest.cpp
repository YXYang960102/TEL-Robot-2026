// ELRS round-trip bench test (env: mega_elrs_link_test).
// PC Dashboard -> ground STM32 -> ELRS TX ))) ELRS RX -> robot STM32
// bridge -> Mega Serial2, and the status uplink back the same way.
// Uses the production MechLink parser and the shared rcmap codec
// unchanged, so what this prints is exactly what the robot build would
// see. No motors or other subsystems are built in -- nothing can move.
//
// Downlink: prints a $ELRS line 4x per second with the raw channels and
// the decoded operator command, and an event whenever MechLink's link
// state flips (the robot bridge goes silent on RF loss, so turning the
// ground side off should produce LINK_LOST within ~0.7 s).
//
// Uplink: sends the same "ST,..." status the production firmware sends,
// every 100 ms, with "armed" mirroring the arm channel (so the Dashboard's
// Enable handshake completes against this sketch even though nothing is
// actually enabled). Each fire press (CH5 low->high), auto-path press and
// arm change bumps the event sequence, so the ground station should print
// "$EVENT,FEED_TRIGGER_SENT" / "$EVENT,AUTO_PATH_STARTED" /
// "$EVENT,OUTPUTS_ENABLED" / "$EVENT,OUTPUTS_DISABLED" -- that proves the
// Battery-frame telemetry path end to end before any motor is wired.
#include <Arduino.h>

#include "../../IO/MechLink.h"
#include "../../IO/RcControlMap.h"

namespace {

constexpr unsigned long PRINT_PERIOD_MS = 250;
constexpr unsigned long STATUS_PERIOD_MS = 100;

bool lastLost = true;
unsigned long stateSinceMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastFrame = 0;
unsigned long badFrames = 0;
unsigned long fireEvents = 0;

rcmap_command cmd{};
bool cmdValid = false;
bool armedPrev = false;
bool firePrev = true;   // a press needs a release first (also after a dropout)
uint8_t autoPrev = 0xFF;

unsigned long eventSeq = 0;
uint16_t lastEventCode = RCMAP_EVENT_NONE;

void recordEvent(uint16_t code) {
    eventSeq++;
    lastEventCode = code;
    Serial.print(F("$ELRS_EVENT,SENT_UP,"));
    Serial.println(rcmap_event_name(code));
}

void printState(unsigned long now) {
    Serial.print(F("$ELRS,t="));
    Serial.print(now);
    Serial.print(F(",link="));
    Serial.print(MechLink::signalLost ? F("LOST") : F("OK"));
    Serial.print(F(",raw="));
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        if (i) Serial.print('/');
        Serial.print(MechLink::channels[i]);
    }
    Serial.print(F(",valid="));
    Serial.print(cmdValid ? 1 : 0);
    Serial.print(F(",fwd="));
    Serial.print(cmd.forward, 2);
    Serial.print(F(",turn="));
    Serial.print(cmd.turn, 2);
    Serial.print(F(",target="));
    Serial.print(cmd.target);
    Serial.print(F(",arm="));
    Serial.print(cmd.armed ? 1 : 0);
    Serial.print(F(",fire="));
    Serial.print(cmd.fire ? 1 : 0);
    Serial.print(F(",nudge="));
    Serial.print(cmd.nudge);
    Serial.print(F(",auto="));
    Serial.print(cmd.auto_path);
    Serial.print(F(",F="));
    Serial.print(fireEvents);
    Serial.print(F(",bad="));
    Serial.println(badFrames);
}

}  // namespace

void setup() {
    Serial.begin(115200);
    MechLink::init();
    stateSinceMs = millis();
    Serial.println(F("$ELRS_BOOT mega_elrs_link_test: MechLink on Serial2 D16/D17 @115200, no outputs"));
    Serial.println(F("$ELRS_BOOT link=LOST until the first valid RC line arrives"));
}

void loop() {
    MechLink::update();
    const unsigned long now = millis();

    if (MechLink::signalLost != lastLost) {
        Serial.print(MechLink::signalLost ? F("$ELRS_EVENT,LINK_LOST,t=") : F("$ELRS_EVENT,LINK_OK,t="));
        Serial.print(now);
        Serial.print(F(",previous_state_ms="));
        Serial.println(now - stateSinceMs);
        lastLost = MechLink::signalLost;
        stateSinceMs = now;
        if (MechLink::signalLost) {
            // Same rule as production: after a reconnect fire/auto must be
            // seen released before they count. (The "armed" reported here
            // is just the arm channel level -- this sketch enables nothing.)
            firePrev = true;
            autoPrev = 0xFF;
            armedPrev = false;
            cmdValid = false;
        }
    }

    if (MechLink::frameCount != lastFrame) {
        lastFrame = MechLink::frameCount;
        rcmap_command decoded;
        cmdValid = rcmap_decode(MechLink::channels, RCMAP_CHANNEL_COUNT, &decoded);
        if (!cmdValid) {
            badFrames++;
        } else {
            cmd = decoded;
            if (cmd.armed != armedPrev) {
                recordEvent(cmd.armed ? RCMAP_EVENT_OUTPUTS_ENABLED : RCMAP_EVENT_OUTPUTS_DISABLED);
                armedPrev = cmd.armed;
            }
            if (cmd.fire && !firePrev) {
                fireEvents++;
                recordEvent(RCMAP_EVENT_FEED_TRIGGER_SENT);
            }
            firePrev = cmd.fire;
            if (cmd.auto_path != RCMAP_AUTO_NONE && autoPrev == RCMAP_AUTO_NONE) {
                recordEvent(RCMAP_EVENT_AUTO_PATH_STARTED);
            }
            autoPrev = cmd.auto_path;
        }
    }

    if (now - lastStatusMs >= STATUS_PERIOD_MS) {
        lastStatusMs = now;
        uint8_t flags = 0;
        if (cmdValid && cmd.armed) flags |= RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS;
        if (!MechLink::signalLost) flags |= RCMAP_STATUS_RC_LINK_OK;
        MechLink::sendStatus(flags, eventSeq, lastEventCode);
    }

    if (now - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = now;
        printState(now);
    }
}
