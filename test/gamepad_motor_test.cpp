// Host test for the single-motor gamepad bench
// (src/Bench/GamepadMotorTest/GamepadMotorTest.cpp), compiled as-is against
// the fake Arduino core in test/host_stubs/.
#include <assert.h>
#include <stdio.h>

#include <sstream>
#include <string>
#include <vector>

#include "host_stubs/host_stubs.h"

#include "../src/Bench/GamepadMotorTest/GamepadMotorTest.cpp"

using hoststub::nowMs;

namespace {

void send(const std::string& line) { Serial.send(line + "\n"); }
void tick(unsigned long ms = 10) { nowMs += ms; loop(); }
bool saw(const std::string& event) {
    return Serial.output.find("$EVENT," + event + "\n") != std::string::npos;
}
int pulse() { return Servo::lastPulse; }

std::vector<std::string> lastTel() {
    const size_t at = Serial.output.rfind("TEL,");
    assert(at != std::string::npos);
    const std::string line = Serial.output.substr(at, Serial.output.find('\n', at) - at);
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string f;
    while (std::getline(ss, f, ',')) fields.push_back(f);
    return fields;
}

}  // namespace

int main() {
    nowMs = 1000;
    setup();
    assert(pulse() == 1500 && Servo::lastPin == A5);

    // Disarmed: motion refused, motor stays neutral.
    send("T,-1,1"); tick();
    assert(saw("REJECTED_DISARMED") && pulse() == 1500);

    send("E"); tick();
    assert(saw("OUTPUTS_ENABLED") && pulse() == 1500);
    send("T,-1,1"); tick();       // D / stick full right
    assert(pulse() == 2000);
    send("T,1,-1"); tick();       // A / stick full left
    assert(pulse() == 1000);
    send("T,-0.5,0.5"); tick();   // stick half right
    assert(pulse() == 1750);
    send("T,1,1"); tick();        // forward only: no turn
    assert(pulse() == 1500);
    send("T,0.3,0.9"); tick();    // forward + a bit right: turn 0.3
    assert(pulse() == 1650);
    send("T,-4,4"); tick();       // out of range clamps
    assert(pulse() == 2000);

    // Telemetry in the production layout: 28 fields, armed, pulse, owner.
    tick(100);
    std::vector<std::string> tel = lastTel();
    assert(tel.size() == 28);
    assert(tel[18] == "1" && tel[19] == "2000" && tel[21] == "1" && tel[13] == "3");

    // Other Dashboard commands keep the link alive but drive nothing.
    for (int i = 0; i < 10; i++) { send(i % 2 ? "K" : "FLY,0.5"); tick(50); }
    assert(pulse() == 2000);

    // Watchdog: nothing for >250 ms -> stop.
    tick(300);
    assert(saw("COMMAND_TIMEOUT") && pulse() == 1500);
    tick(100);
    tel = lastTel();
    assert(tel[18] == "0" && tel[19] == "1500");

    // Bad input.
    send("E"); tick();
    send("T,nan,0"); tick();
    assert(saw("BAD_COMMAND") && pulse() == 1500);
    send("HELLO"); tick();
    assert(saw("UNKNOWN_COMMAND"));
    send("T,-1,1"); tick();
    assert(pulse() == 2000);
    send(std::string(80, '9')); tick();   // oversized line: stop
    assert(saw("DISABLED_CORRUPT_COMMAND") && pulse() == 1500);

    // X stops.
    send("E"); send("T,-1,1"); tick();
    assert(pulse() == 2000);
    send("X"); tick();
    assert(saw("OUTPUTS_DISABLED") && pulse() == 1500);

    printf("gamepad_motor_test: all asserts passed\n");
    return 0;
}
