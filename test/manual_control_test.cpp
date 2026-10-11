// Host regression test for operator control: compiles the REAL
// src/IO/ManualControl.cpp, src/Dribbler/Dribbler.cpp and
// src/IO/RcControlMap.c against fake Arduino time/pins/Serial
// (test/host_stubs/), with Chassis/Shooter/Vision replaced by recorders and
// MechLink replaced by a stub the test feeds RC frames into.
//
// Covers the wired USB path (everything Codex's reviews of e4a70a5/afed238
// asked for), the wireless ELRS path, control authority between the two,
// and the feed-tower trigger pulse.
//
// Build twice (see test/README):
//   with -DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=true  -> full armed-path suite
//   without it                                     -> pin-table gate only
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <string>

#include "host_stubs/host_stubs.h"

#include "../src/Constants/DribblerConstants.h"
#include "../src/Dribbler/Dribbler.h"
#include "../src/IO/ManualControl.h"
#include "../src/IO/MechLink.h"
#include "../src/IO/RcControlMap.h"

using hoststub::digitalOut;
using hoststub::nowMs;
using hoststub::rec;
using Owner = ManualControl::Owner;

namespace {

constexpr uint8_t TRIGGER_PIN = DribblerConstants::Tower::TRIGGER_PIN;

rcmap_command wireless; // what the "ground station" is currently sending

void reset() {
    nowMs = 1000;
    for (int& v : digitalOut) v = 0;
    rec = hoststub::Recorder{};
    hoststub::visionReady = false;
    MechLink::signalLost = true;
    MechLink::frameCount = 0;
    wireless = rcmap_command{};
    Serial.input.clear();
    Serial.output.clear();
    ManualControl::init();
    Dribbler::init();
    nowMs += DribblerConstants::Tower::COOLDOWN_MS; // start outside any lockout
}

// Deliver one RC frame encoding `wireless`, as MechLink would.
void rcFrame() {
    int ch[16];
    rcmap_encode(&wireless, ch);
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) MechLink::channels[i] = ch[i];
    MechLink::frameCount++;
    MechLink::signalLost = false;
}

// One main-loop tick in production order (ManualControl before Dribbler).
void step(unsigned long ms = 10) {
    nowMs += ms;
    ManualControl::update();
    Dribbler::update();
}

// A tick with a fresh RC frame, like the bridge forwarding every 20 ms.
void rcStep(unsigned long ms = 20) {
    rcFrame();
    step(ms);
}

void send(const std::string& line) { Serial.send(line + "\n"); }

bool saw(const std::string& event) {
    return Serial.output.find("$EVENT," + event + "\n") != std::string::npos;
}

[[maybe_unused]] bool triggerHigh() { return digitalOut[TRIGGER_PIN] == HIGH; }

[[maybe_unused]] void runWithHeartbeat(unsigned long totalMs) {
    for (unsigned long t = 0; t < totalMs; t += 10) {
        if (t % 50 == 0) send("K");
        step(10);
    }
}

[[maybe_unused]] void runWireless(unsigned long totalMs) {
    for (unsigned long t = 0; t < totalMs; t += 20) rcStep(20);
}

[[maybe_unused]] void armWired() {
    send("E");
    step();
    assert(ManualControl::getOwner() == Owner::WIRED);
    Serial.output.clear();
}

[[maybe_unused]] void armWireless() {
    wireless.armed = false;
    rcStep();
    wireless.armed = true;
    rcStep();
    assert(ManualControl::getOwner() == Owner::WIRELESS);
    Serial.output.clear();
}

[[maybe_unused]] bool nearly(double a, double b) { return fabs(a - b) < 1e-6; }

#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)

// ------------------------------------------------------------------ wired

void testWiredRefusedWhileDisarmed() {
    reset();
    for (const char* c : {"F", "L", "T,1,1", "ROT,1", "ANG,1", "FLY,0.5"}) {
        Serial.output.clear();
        send(c);
        step();
        assert(saw("REJECTED_DISARMED"));
    }
    runWithHeartbeat(2000);
    assert(!ManualControl::isArmed() && !triggerHigh());
    assert(nearly(rec.chassisLeft, 0.0) && nearly(rec.flywheelCommand, 0.0));
    assert(Dribbler::getTriggerCount() == 0);
}

void testRIsGone() {
    reset();
    armWired();
    send("R");
    step();
    assert(saw("UNKNOWN_COMMAND"));
}

void testEnableDisableDrivesEverySubsystem() {
    reset();
    armWired();
    assert(rec.chassisEnabled && rec.shooterEnabled);
    send("FLY,0.6");
    send("T,1,1");
    step();
    assert(nearly(rec.flywheelCommand, 0.6) && rec.chassisLeft > 0.0);
    send("X");
    step();
    assert(saw("OUTPUTS_DISABLED"));
    assert(!rec.chassisEnabled && !rec.shooterEnabled && nearly(rec.flywheelCommand, 0.0));
}

void testWiredWatchdog() {
    reset();
    armWired();
    send("T,1,1");
    step();
    for (int i = 0; i < 30; i++) step(10);
    assert(saw("COMMAND_TIMEOUT") && !ManualControl::isArmed() && ManualControl::didTimeOut());
    assert(!rec.chassisEnabled && !rec.shooterEnabled);

    reset();
    armWired();
    nowMs += 300;
    send("K"); // too late to rescue
    ManualControl::update();
    assert(saw("COMMAND_TIMEOUT") && !ManualControl::isArmed());

    reset();
    armWired();
    runWithHeartbeat(2000);
    assert(ManualControl::isArmed());
}

void testWiredScalingAndParsing() {
    reset();
    armWired();
    send("T,1,-1");
    send("ROT,1");
    send("ANG,-1");
    send("FLY,1.5");
    step();
    using namespace ManualControlConstants;
    assert(nearly(rec.chassisLeft, CHASSIS_MANUAL_SCALE) && nearly(rec.chassisRight, -CHASSIS_MANUAL_SCALE));
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, ROTATE_MANUAL_SCALE));
    assert(nearly(rec.angleCommand, -ANGLE_MANUAL_SCALE) && nearly(rec.flywheelCommand, 1.0));
    for (const char* bad : {"FLY,nan", "FLY,inf", "ROT,-1e999", "T,2", "T,1,1,1", "FLY,abc"}) {
        Serial.output.clear();
        send(bad);
        ManualControl::update();
        assert(saw("BAD_COMMAND"));
    }
    step();
    assert(nearly(rec.flywheelCommand, 1.0)); // bad values changed nothing
    Serial.output.clear();
    send("Q");
    step();
    assert(saw("UNKNOWN_COMMAND"));
}

void testWiredOversizedLine() {
    reset();
    send(std::string(24, 'z') + "E");
    step();
    assert(saw("COMMAND_TOO_LONG") && !ManualControl::isArmed());
    send("E");
    step();
    assert(ManualControl::isArmed());
    Serial.output.clear();
    send(std::string(30, 'z') + "F");
    step();
    assert(saw("DISABLED_CORRUPT_COMMAND") && !ManualControl::isArmed());
    assert(Dribbler::getTriggerCount() == 0);
}

void testLToggle() {
    reset();
    armWired();
    send("ROT,1");
    send("L");
    step();
    assert(saw("ROTATE_VISION_FOLLOW") && rec.rotateMode == "disabled"); // vision not ready
    hoststub::visionReady = true;
    send("K");
    step();
    assert(rec.rotateMode == "vision");
    send("L");
    step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, 0.0));
}

// ------------------------------------------------------------- feed tower

void testFeedPulseAndLockout() {
    reset();
    armWired();
    send("F");
    step(1);
    assert(saw("FEED_TRIGGER_SENT") && triggerHigh() && Dribbler::getTriggerCount() == 1);
    // HIGH for exactly PULSE_MS, then LOW.
    for (int i = 0; i < 9; i++) {
        send("K");
        step(10);
    }
    assert(triggerHigh()); // ~91 ms in
    send("K");
    step(10);
    assert(!triggerHigh()); // past 100 ms
    // Second F inside the lockout: refused, not queued.
    Serial.output.clear();
    send("F");
    step();
    assert(saw("FEED_BUSY") && Dribbler::getTriggerCount() == 1);
    runWithHeartbeat(DribblerConstants::Tower::COOLDOWN_MS + 50);
    assert(Dribbler::getTriggerCount() == 1 && !triggerHigh()); // nothing fired later
    send("F");
    step();
    assert(Dribbler::getTriggerCount() == 2 && triggerHigh());
}

void testDisarmCutsPulse() {
    reset();
    armWired();
    send("F");
    step();
    assert(triggerHigh());
    send("X");
    step();
    assert(!triggerHigh());
}

// --------------------------------------------------------------- wireless

void testWirelessEnableNeedsEdge() {
    reset();
    // Arm already high in the very first frame (e.g. link came up while the
    // operator had Enable on): no enable.
    wireless.armed = true;
    runWireless(200);
    assert(!ManualControl::isArmed());
    wireless.armed = false;
    rcStep();
    wireless.armed = true;
    rcStep();
    assert(saw("OUTPUTS_ENABLED") && ManualControl::getOwner() == Owner::WIRELESS);
    assert(rec.chassisEnabled && rec.shooterEnabled);
}

void testWirelessCommands() {
    reset();
    armWireless();
    wireless.left = 1.0f;
    wireless.right = -1.0f;
    wireless.rotate = 1;
    wireless.angle = -1;
    wireless.flywheel = 0.5f;
    rcStep();
    using namespace ManualControlConstants;
    assert(nearly(rec.chassisLeft, CHASSIS_MANUAL_SCALE) && nearly(rec.chassisRight, -CHASSIS_MANUAL_SCALE));
    assert(nearly(rec.rotateCommand, ROTATE_MANUAL_SCALE) && nearly(rec.angleCommand, -ANGLE_MANUAL_SCALE));
    assert(nearly(rec.flywheelCommand, 0.5));
    // Arm released by the ground: disarm.
    wireless.armed = false;
    rcStep();
    assert(saw("OUTPUTS_DISABLED") && !ManualControl::isArmed() && !rec.chassisEnabled);
}

void testWirelessCountersNoRepeatNoReplay() {
    reset();
    armWireless();
    wireless.feed_counter = (wireless.feed_counter + 1) % RCMAP_COUNTER_LEVELS;
    runWireless(200); // the same level repeats in every frame
    assert(Dribbler::getTriggerCount() == 1);
    wireless.lmode_counter = 1;
    runWireless(100);
    assert(saw("ROTATE_VISION_FOLLOW"));

    // Presses during an RF outage are not replayed after it.
    MechLink::signalLost = true;
    step(300);
    assert(saw("DISABLED_RC_LINK_LOST") && !ManualControl::isArmed());
    wireless.feed_counter = (wireless.feed_counter + 1) % RCMAP_COUNTER_LEVELS;
    runWireless(200);
    assert(Dribbler::getTriggerCount() == 1);
    // ...and recovering the link does not re-enable on its own, even with
    // the ground still holding Enable.
    assert(!ManualControl::isArmed());
    armWireless();
    runWireless(DribblerConstants::Tower::COOLDOWN_MS);
    wireless.feed_counter = (wireless.feed_counter + 2) % RCMAP_COUNTER_LEVELS; // jump of 2
    rcStep();
    assert(saw("EVENT_SKIPPED") && Dribbler::getTriggerCount() == 1);
    wireless.feed_counter = (wireless.feed_counter + 1) % RCMAP_COUNTER_LEVELS;
    rcStep();
    assert(Dribbler::getTriggerCount() == 2);
}

void testWirelessPressWhileDisarmedNotLatched() {
    reset();
    wireless.armed = false;
    rcStep();
    wireless.feed_counter = 1;
    rcStep();
    assert(saw("REJECTED_DISARMED"));
    armWireless();
    runWireless(200);
    assert(Dribbler::getTriggerCount() == 0);
}

void testWirelessInvalidFramesTimeOut() {
    reset();
    armWireless();
    for (int i = 0; i < 20; i++) {
        rcFrame();
        MechLink::channels[RCMAP_CH_FEED_COUNTER] = 1100; // between counter levels
        step(20);
    }
    assert(saw("COMMAND_TIMEOUT") && !ManualControl::isArmed());
}

// -------------------------------------------------------------- authority

void testAuthority() {
    // Wired owns: wireless enable refused, wireless presses refused.
    reset();
    wireless.armed = false;
    rcStep();
    armWired();
    wireless.armed = true;
    rcStep();
    assert(saw("ENABLE_REJECTED_OTHER_SOURCE") && ManualControl::getOwner() == Owner::WIRED);
    wireless.feed_counter = 1;
    rcStep();
    assert(saw("REJECTED_NOT_OWNER") && Dribbler::getTriggerCount() == 0);
    // The ground's idle (arm low) never stops the wired session.
    wireless.armed = false;
    send("K");
    rcStep();
    assert(ManualControl::getOwner() == Owner::WIRED);

    // Wireless owns: wired enable and motion refused...
    reset();
    armWireless();
    send("E");
    send("T,1,1");
    rcStep();
    assert(saw("ENABLE_REJECTED_OTHER_SOURCE") && saw("REJECTED_NOT_OWNER"));
    assert(ManualControl::getOwner() == Owner::WIRELESS && nearly(rec.chassisLeft, 0.0));
    // ...but a wired stop always stops it.
    send("X");
    rcStep();
    assert(!ManualControl::isArmed() && !rec.chassisEnabled);
    // And it stays stopped until the ground releases and re-presses Enable.
    runWireless(200);
    assert(!ManualControl::isArmed());
}

void testAutoStepsAsideWhileArmed() {
    reset();
    assert(!ManualControl::isActive());
    armWireless();
    assert(ManualControl::isActive());
}

// ---------------------------------------------------------------- uplink

void testStatusUplink() {
    reset();
    step(150);
    assert(rec.statusCount >= 1);
    assert(rec.statusFlags & RCMAP_STATUS_PINS_CONFIRMED);
    assert(!(rec.statusFlags & RCMAP_STATUS_ARMED));
    armWireless();
    step(120);
    assert(rec.statusFlags & RCMAP_STATUS_ARMED);
    assert(rec.statusFlags & RCMAP_STATUS_OWNER_WIRELESS);
    assert(rec.statusFlags & RCMAP_STATUS_RC_LINK_OK);
    assert(rec.statusCode == RCMAP_EVENT_OUTPUTS_ENABLED);
    const unsigned long seq = rec.statusSeq;
    wireless.feed_counter = 1;
    rcStep(120);
    assert(rec.statusCode == RCMAP_EVENT_FEED_TRIGGER_SENT && rec.statusSeq == seq + 1);
    assert(rec.statusFlags & RCMAP_STATUS_FEED_BUSY);
}

#else

void testEnableRefusedUntilPinTableConfirmed() {
    reset();
    send("E");
    step();
    assert(saw("ENABLE_REJECTED_PINS_UNCONFIRMED") && !ManualControl::isArmed());
    wireless.armed = false;
    rcStep();
    wireless.armed = true;
    rcStep();
    assert(!ManualControl::isArmed() && !rec.chassisEnabled && !rec.shooterEnabled);
    step(150);
    assert(!(rec.statusFlags & RCMAP_STATUS_PINS_CONFIRMED));
}

#endif

}  // namespace

int main() {
#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
    testWiredRefusedWhileDisarmed();
    testRIsGone();
    testEnableDisableDrivesEverySubsystem();
    testWiredWatchdog();
    testWiredScalingAndParsing();
    testWiredOversizedLine();
    testLToggle();
    testFeedPulseAndLockout();
    testDisarmCutsPulse();
    testWirelessEnableNeedsEdge();
    testWirelessCommands();
    testWirelessCountersNoRepeatNoReplay();
    testWirelessPressWhileDisarmedNotLatched();
    testWirelessInvalidFramesTimeOut();
    testAuthority();
    testAutoStepsAsideWhileArmed();
    testStatusUplink();
    printf("manual_control_test (pin table confirmed): all asserts passed\n");
#else
    testEnableRefusedUntilPinTableConfirmed();
    printf("manual_control_test (pin table unconfirmed gate): all asserts passed\n");
#endif
    return 0;
}
