// Serial1 (D18 TX1 / D19 RX1) check for one Mega (env: mega_serial1_test).
// Nothing else is initialized: no motors, no Serial2, no outputs.
//
// D18: sends "MEGA_HEARTBEAT,2" every 100 ms -- the line an Orin in
//   WAIT_MEGA is waiting for, so wired to the Orin its log should show
//   "MEGA_READY received" (D18 -> level shifter -> Orin RX works).
// D19: everything received is counted; once a second USB reports
//   S1TEST,t=<ms>,sent=<lines>,rx_bytes=<n>,rx_lines=<n>,last=<last line>
//   Wired to an Orin in WAIT_MEGA, "last" should be VISION_STANDBY,2
//   (Orin TX -> level shifter -> D19 works).
// Loopback: jumper D18 -> D19 (level shifter wires off those pins) and
//   "last" is this Mega's own MEGA_HEARTBEAT,2.
#include <Arduino.h>

namespace {

constexpr unsigned long BAUD = 115200;
constexpr unsigned long SEND_MS = 100;
constexpr unsigned long REPORT_MS = 1000;
constexpr size_t LINE_MAX = 48;

unsigned long lastSendMs = 0;
unsigned long lastReportMs = 0;
unsigned long sentLines = 0;
unsigned long rxBytes = 0;
unsigned long rxLines = 0;
char line[LINE_MAX];
size_t lineLength = 0;
char lastLine[LINE_MAX] = "(none)";

}  // namespace

void setup() {
    Serial.begin(BAUD);
    Serial1.begin(BAUD);
    Serial.println(F("$SERIAL1_TEST_READY: D18 sends MEGA_HEARTBEAT,2 every 100 ms; D19 received bytes reported every 1 s"));
}

void loop() {
    const unsigned long now = millis();

    while (Serial1.available() > 0) {
        const char c = static_cast<char>(Serial1.read());
        rxBytes++;
        if (c == '\r') continue;
        if (c == '\n') {
            line[lineLength] = '\0';
            if (lineLength > 0) {
                rxLines++;
                memcpy(lastLine, line, lineLength + 1);
            }
            lineLength = 0;
        } else if (lineLength + 1 < LINE_MAX) {
            // Non-printable bytes shown as '?', so noise is visible too.
            line[lineLength++] = (c >= 32 && c < 127) ? c : '?';
        }
    }

    if (now - lastSendMs >= SEND_MS) {
        lastSendMs = now;
        Serial1.print(F("MEGA_HEARTBEAT,2\n"));
        sentLines++;
    }

    if (now - lastReportMs >= REPORT_MS) {
        lastReportMs = now;
        Serial.print(F("S1TEST,t="));
        Serial.print(now);
        Serial.print(F(",sent="));
        Serial.print(sentLines);
        Serial.print(F(",rx_bytes="));
        Serial.print(rxBytes);
        Serial.print(F(",rx_lines="));
        Serial.print(rxLines);
        Serial.print(F(",last="));
        Serial.println(lastLine);
    }
}
