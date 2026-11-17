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

}

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
