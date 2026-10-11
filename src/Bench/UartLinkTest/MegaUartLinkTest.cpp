// Standalone Mega <-> STM32 UART link test (env: mega_uart_link_test).
// Serial2 (TX D16 -> STM32 PA10, RX D17 <- STM32 PA9), 115200 8N1.
// Serial1 (D18/D19, Orin) is left untouched. No motors, no other
// subsystems are compiled into this build. Protocol: ulink.h.
//
// USB (115200) commands, one per line:
//   RUN,<seconds>  start a run, stop automatically after <seconds>, report
//   START          start a run with no time limit (for unplug/replug tests)
//   STOP           stop the current run, drain, report
//   STATUS         print the current counters now
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "ulink.h"

namespace {

constexpr unsigned long PROGRESS_PERIOD_MS = 1000;
constexpr unsigned long PEER_SILENT_MS = 1000;
constexpr size_t USB_LINE_MAX = 32;

ulink_endpoint link;
uint16_t lastRun = 0;
unsigned long runLimitMs = 0;        // 0 = no limit
unsigned long runStartedMs = 0;      // when RUNNING began
unsigned long runStoppedMs = 0;      // when STOP was requested
ulink_run_state lastState = ULINK_IDLE;
unsigned long lastProgressMs = 0;
bool peerAliveReported = false;

char usbLine[USB_LINE_MAX];
size_t usbLen = 0;
bool usbDiscarding = false;

void writeToStm32(void*, const char* data, uint16_t len) {
    Serial2.write(reinterpret_cast<const uint8_t*>(data), len);
}

void printRtt(const ulink_stats& s) {
    Serial.print(F("min="));
    Serial.print(s.rtt_min_ms);
    Serial.print(F(" avg="));
    if (s.ack_ok > 0) {
        Serial.print(static_cast<double>(s.rtt_sum_ms) / s.ack_ok, 2);
    } else {
        Serial.print(F("-"));
    }
    Serial.print(F(" max="));
    Serial.print(s.rtt_max_ms);
    Serial.print(F(" ms"));
}

void printDirection(const __FlashStringHelper* label, const ulink_stats& sender,
                    const ulink_stats& receiver) {
    Serial.print(label);
    Serial.print(F(" sent="));
    Serial.print(sender.tx);
    Serial.print(F(" acked="));
    Serial.print(sender.ack_ok);
    Serial.print(F(" timeout="));
    Serial.print(sender.timeout);
    Serial.print(F(" lateAck="));
    Serial.print(sender.ack_stale);
    Serial.print(F(" dupAck="));
    Serial.print(sender.ack_dup);
    Serial.print(F(" badAck="));
    Serial.print(sender.ack_mismatch);
    Serial.print(F(" | received="));
    Serial.print(receiver.rx_data);
    Serial.print(F(" lost(gap)="));
    Serial.print(receiver.rx_gap);
    Serial.print(F(" dup="));
    Serial.println(receiver.rx_dup);
}

bool peerStatusFresh() {
    return link.peer_status_seen && millis() - link.peer_status_ms <= PEER_SILENT_MS;
}

void printProgress() {
    const ulink_stats& m = link.st;
    const ulink_stats& s = link.peer;
    Serial.print(F("$ULINK,t="));
    Serial.print(millis() - runStartedMs);
    Serial.print(F(",link="));
    Serial.print(link.link_up ? F("UP") : F("DOWN"));
    Serial.print(F(",M>S="));
    Serial.print(m.ack_ok);
    Serial.print('/');
    Serial.print(m.tx);
    Serial.print(F(",S>M="));
    Serial.print(s.ack_ok);
    Serial.print('/');
    Serial.print(s.tx);
    Serial.print(F(",Mtimeout="));
    Serial.print(m.timeout);
    Serial.print(F(",Stimeout="));
    Serial.print(s.timeout);
    Serial.print(F(",Mbad="));
    Serial.print(m.bad + m.overflow);
    Serial.print(F(",Sbad="));
    Serial.print(s.bad + s.overflow);
    Serial.print(F(",stm32Status="));
    Serial.println(peerStatusFresh() ? F("fresh") : F("STALE"));
}

void printReport() {
    const ulink_stats& m = link.st;
    const ulink_stats& s = link.peer;
    const bool peerFinal = link.drain_confirmed;
    const bool pass = peerFinal && m.tx > 0 && s.tx > 0 &&
        m.ack_ok == m.tx && s.ack_ok == s.tx &&
        s.rx_data == m.tx && m.rx_data == s.tx &&
        m.timeout == 0 && s.timeout == 0 &&
        m.ack_stale == 0 && s.ack_stale == 0 && m.ack_dup == 0 && s.ack_dup == 0 &&
        m.ack_mismatch == 0 && s.ack_mismatch == 0 &&
        m.rx_gap == 0 && s.rx_gap == 0 && m.rx_dup == 0 && s.rx_dup == 0 &&
        m.bad == 0 && s.bad == 0 && m.overflow == 0 && s.overflow == 0;

    Serial.println();
    Serial.print(F("=== UART link report, run "));
    Serial.print(link.run);
    Serial.println(F(" ==="));
    Serial.print(F("duration_ms="));
    Serial.print(runStoppedMs - runStartedMs);
    Serial.print(F("  drain="));
    Serial.println(peerFinal ? F("confirmed (STM32 final counters)")
                             : F("NOT CONFIRMED (STM32 numbers below may be stale)"));
    printDirection(F("Mega->STM32:"), m, s);
    printDirection(F("STM32->Mega:"), s, m);
    Serial.print(F("RTT seen by Mega:  "));
    printRtt(m);
    Serial.println();
    Serial.print(F("RTT seen by STM32: "));
    printRtt(s);
    Serial.println();
    Serial.print(F("line errors: Mega bad="));
    Serial.print(m.bad);
    Serial.print(F(" overflow="));
    Serial.print(m.overflow);
    Serial.print(F(" | STM32 bad="));
    Serial.print(s.bad);
    Serial.print(F(" overflow="));
    Serial.println(s.overflow);
    Serial.print(F("RESULT: "));
    Serial.println(pass ? F("PASS (every packet both ways acked, zero errors)")
                        : F("FAIL (see counters above)"));

    // Machine-readable copy of the same numbers.
    const uint32_t fields[] = {
        link.run, runStoppedMs - runStartedMs, peerFinal ? 1u : 0u, pass ? 1u : 0u,
        m.tx, m.ack_ok, m.timeout, m.ack_stale, m.ack_dup, m.ack_mismatch,
        s.rx_data, s.rx_gap, s.rx_dup,
        s.tx, s.ack_ok, s.timeout, s.ack_stale, s.ack_dup, s.ack_mismatch,
        m.rx_data, m.rx_gap, m.rx_dup,
        m.bad, m.overflow, s.bad, s.overflow,
        m.rtt_min_ms, m.rtt_max_ms, m.rtt_sum_ms, s.rtt_min_ms, s.rtt_max_ms, s.rtt_sum_ms,
    };
    Serial.print(F("$ULINK_RESULT"));
    for (uint32_t v : fields) {
        Serial.print(',');
        Serial.print(v);
    }
    Serial.println();
}

void startRun(unsigned long limitMs) {
    if (link.state == ULINK_STARTING || link.state == ULINK_RUNNING ||
        link.state == ULINK_STOPPING) {
        Serial.println(F("$ULINK_EVENT,BUSY (STOP first)"));
        return;
    }
    uint16_t run = static_cast<uint16_t>((micros() ^ (millis() << 5)) & 0xFFFFu);
    if (run == 0 || run == lastRun) run = static_cast<uint16_t>(lastRun + 1u);
    if (run == 0) run = 1;
    lastRun = run;
    runLimitMs = limitMs;
    ulink_master_start(&link, run, millis());
    Serial.print(F("$ULINK_EVENT,STARTING,run="));
    Serial.print(run);
    Serial.print(F(",limit_ms="));
    Serial.println(limitMs);
}

void handleUsbCommand(const char* line) {
    if (strncmp(line, "RUN,", 4) == 0) {
        char* end = nullptr;
        const long seconds = strtol(line + 4, &end, 10);
        if (end == line + 4 || *end != '\0' || seconds < 1 || seconds > 3600) {
            Serial.println(F("$ULINK_EVENT,BAD_COMMAND (RUN,<1..3600>)"));
            return;
        }
        startRun(static_cast<unsigned long>(seconds) * 1000UL);
    } else if (strcmp(line, "START") == 0) {
        startRun(0);
    } else if (strcmp(line, "STOP") == 0) {
        if (link.state != ULINK_STARTING && link.state != ULINK_RUNNING) {
            Serial.println(F("$ULINK_EVENT,NOT_RUNNING"));
            return;
        }
        runStoppedMs = millis();
        ulink_master_stop(&link, runStoppedMs);
        Serial.println(F("$ULINK_EVENT,STOPPING (draining)"));
    } else if (strcmp(line, "STATUS") == 0) {
        printProgress();
    } else if (line[0] != '\0') {
        Serial.println(F("$ULINK_EVENT,UNKNOWN_COMMAND (RUN,<s> | START | STOP | STATUS)"));
    }
}

void readUsb() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;
        if (c == '\n') {
            if (!usbDiscarding) {
                usbLine[usbLen] = '\0';
                handleUsbCommand(usbLine);
            }
            usbLen = 0;
            usbDiscarding = false;
            continue;
        }
        if (usbDiscarding) continue;
        if (usbLen + 1 < USB_LINE_MAX) {
            usbLine[usbLen++] = c;
        } else {
            usbLen = 0;
            usbDiscarding = true;
            Serial.println(F("$ULINK_EVENT,COMMAND_TOO_LONG"));
        }
    }
}

void reportStateChanges() {
    const unsigned long now = millis();
    const uint8_t events = ulink_take_events(&link);
    if (events & ULINK_EVENT_LINK_DOWN) {
        Serial.print(F("$ULINK_EVENT,LINK_DOWN,t="));
        Serial.println(now - runStartedMs);
    }
    if (events & ULINK_EVENT_LINK_UP) {
        Serial.print(F("$ULINK_EVENT,LINK_UP,t="));
        Serial.println(now - runStartedMs);
    }

    const bool alive = peerStatusFresh();
    if (alive != peerAliveReported) {
        peerAliveReported = alive;
        Serial.println(alive ? F("$ULINK_EVENT,STM32_STATUS_RECEIVED")
                             : F("$ULINK_EVENT,STM32_SILENT (no status line for 1 s)"));
    }

    if (link.state != lastState) {
        if (link.state == ULINK_RUNNING) {
            runStartedMs = now;
            lastProgressMs = now;
            Serial.println(F("$ULINK_EVENT,RUNNING (STM32 confirmed)"));
        } else if (link.state == ULINK_DONE) {
            printReport();
        }
        lastState = link.state;
    }

    if (link.state == ULINK_STARTING && now - link.state_since_ms >= 3000 &&
        now - lastProgressMs >= 3000) {
        lastProgressMs = now;
        Serial.println(F("$ULINK_EVENT,WAITING_FOR_STM32 (no start confirmation yet)"));
    }

    if (link.state == ULINK_RUNNING) {
        if (runLimitMs != 0 && now - runStartedMs >= runLimitMs) {
            runStoppedMs = now;
            ulink_master_stop(&link, now);
            Serial.println(F("$ULINK_EVENT,TIME_LIMIT_REACHED (draining)"));
        } else if (now - lastProgressMs >= PROGRESS_PERIOD_MS) {
            lastProgressMs = now;
            printProgress();
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    ulink_init(&link, 'M', writeToStm32, nullptr, millis());
    Serial.println(F("$ULINK_BOOT mega_uart_link_test: Serial2 D16 TX / D17 RX @115200"));
    Serial.println(F("$ULINK_BOOT commands: RUN,<seconds> | START | STOP | STATUS"));
}

void loop() {
    while (Serial2.available() > 0) {
        ulink_rx_byte(&link, static_cast<char>(Serial2.read()), millis());
    }
    ulink_poll(&link, millis());
    readUsb();
    reportStateChanges();
}
