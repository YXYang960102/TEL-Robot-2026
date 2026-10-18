// Ground-station STM32 for wireless robot control. See README.md.
//
//   PC Dashboard --USB CDC (Serial)--> this board --CRSF, PA9 half-duplex
//   400000 baud--> ELRS TX module ))) robot
//
// The PC speaks exactly the Mega's wired line protocol; GroundCommandState
// turns it into RC channel state (src/IO/RcControlMap.h), sent every
// RC_PERIOD_MS. The robot's status comes back from the TX module as CRSF
// Battery frames on the same wire and is reported to the PC as "$EVENT,..."
// and "$WIRELESS,..." lines.
#include <Arduino.h>

extern "C" {
#include "crsf.h"
}
#include "GroundCommandState.h"

namespace {

// Same 100 Hz cadence as the retired stm32_elrs_bidirectional_test bench, whose CRSF
// link to the TX module on this exact board/pin is hardware-confirmed.
constexpr uint32_t RC_PERIOD_MS = 10;
constexpr uint32_t RF_REPORT_PERIOD_MS = 250;
constexpr uint32_t RF_STATS_STALE_MS = 1000;
constexpr uint8_t CRSF_FRAME_LINK_STATISTICS = 0x14;

Uart crsfPort(PA9); // single pin = half-duplex

GroundCommandState state;
RobotStatusTracker tracker;
CrsfParser parser;

uint32_t lastRcMs = 0;
uint32_t lastRfReportMs = 0;
uint32_t lastLinkStatsMs = 0;
bool haveLinkStats = false;
uint8_t uplinkLq = 0;
uint8_t downlinkLq = 0;
uint8_t uplinkRssi = 0;
uint32_t seenEnableEpoch = 0;

void emitToPc(void*, const char* line) {
    Serial.println(line);
}

void handleFrame(const CrsfFrame& frame, uint32_t now) {
    if (frame.type == CRSF_FRAME_BATTERY && frame.payload_len == 8) {
        const uint16_t voltage = (uint16_t)((frame.payload[0] << 8) | frame.payload[1]);
        const uint16_t current = (uint16_t)((frame.payload[2] << 8) | frame.payload[3]);
        const uint32_t capacity = ((uint32_t)frame.payload[4] << 16) |
                                  ((uint32_t)frame.payload[5] << 8) | frame.payload[6];
        rcmap_status status;
        if (rcmap_status_from_battery(voltage, current, capacity, frame.payload[7], &status)) {
            tracker.onStatus(status, now, emitToPc, nullptr);
        }
    } else if (frame.type == CRSF_FRAME_LINK_STATISTICS && frame.payload_len >= 10) {
        uplinkRssi = frame.payload[0];
        uplinkLq = frame.payload[2];
        downlinkLq = frame.payload[8];
        haveLinkStats = true;
        lastLinkStatsMs = now;
    }
}

void reportRfLink(uint32_t now) {
    if (now - lastRfReportMs < RF_REPORT_PERIOD_MS) return;
    lastRfReportMs = now;
    const bool fresh = haveLinkStats && now - lastLinkStatsMs <= RF_STATS_STALE_MS;
    char text[48];
    // uplink = ground -> robot, downlink = robot -> ground; RSSI in -dBm.
    snprintf(text, sizeof(text), "$RFLINK,%d,%u,%u,%u", fresh ? 1 : 0,
             fresh ? uplinkLq : 0, fresh ? downlinkLq : 0, fresh ? uplinkRssi : 0);
    Serial.println(text);
}

void updateLed(uint32_t now) {
    // PC13 is active low. Off = idle; slow blink = enable requested but no
    // fresh robot status; solid = robot reports armed. A convenience only:
    // the Dashboard's numbers are the real evidence.
    bool on = false;
    if (state.isEnabled()) {
        on = (tracker.fresh(now) && (tracker.flags() & RCMAP_STATUS_ARMED))
                 ? true
                 : (now % 1000) < 500;
    }
    digitalWrite(PC13, on ? LOW : HIGH);
}

}  // namespace

void setup() {
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);
    Serial.begin(115200);
    crsfPort.begin(400000);
    crsf_parser_init(&parser);
    const uint32_t now = millis();
    state.reset(now);
    tracker.reset(now);
}

void loop() {
    const uint32_t now = millis();

    while (Serial.available() > 0) {
        state.rxByte(static_cast<char>(Serial.read()), now, emitToPc, nullptr);
    }
    state.tick(now, emitToPc, nullptr);
    // An E or a release makes the last robot status out of date.
    if (state.enableEpoch() != seenEnableEpoch) {
        seenEnableEpoch = state.enableEpoch();
        tracker.invalidate();
    }

    if (now - lastRcMs >= RC_PERIOD_MS) {
        lastRcMs = now;
        int ch[16];
        rcmap_encode(&state.command(), ch);
        uint8_t frame[CRSF_RC_FRAME_SIZE];
        crsf_build_rc_frame(frame, ch);
        crsfPort.write(frame, sizeof(frame));
        // Wait for the frame to leave, then listen on the same wire for the
        // TX module's telemetry reply.
        crsfPort.enableHalfDuplexRx();
    }

    while (crsfPort.available() > 0) {
        CrsfFrame frame;
        if (crsf_parser_push(&parser, static_cast<uint8_t>(crsfPort.read()), &frame)) {
            handleFrame(frame, now);
        }
    }

    tracker.tick(now, emitToPc, nullptr);
    reportRfLink(now);
    updateLed(now);
}
