// Host regression test for the wired Dashboard firmware path: compiles the
// REAL src/IO/WiredControl.cpp and src/Dribbler/Dribbler.cpp against fake
// Arduino time/pins/Serial (test/host_stubs/), with Chassis/Shooter/Vision/
// MechLink replaced by recorders. Covers the issues from Codex's review of
// e4a70a5 plus the control semantics agreed with Jeremy on 2026-10-11.
//
// Build twice (see test/README):
//   with -DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=true  -> full armed-path suite
//   without it                                     -> pin-table gate only
#include <assert.h>
#include <stdio.h>

#include <string>

#include "host_stubs/host_stubs.h"

#include "../src/Constants/DribblerConstants.h"
#include "../src/Dribbler/Dribbler.h"
#include "../src/IO/MechLink.h"
#include "../src/IO/WiredControl.h"

using hoststub::analogOut;
using hoststub::nowMs;
using hoststub::pinLevel;
using hoststub::rec;

namespace {

constexpr uint8_t FEED_PWM_PIN = DribblerConstants::Feeder::PWM_PINS[0];
constexpr uint8_t EXIT_PIN = DribblerConstants::ExitSensor::SIGNAL_PIN;
[[maybe_unused]] constexpr int SENSOR_BLOCKED = LOW;   // ExitSensor::BLOCKED_LOW
constexpr int SENSOR_CLEAR = HIGH;

void reset(int exitSensorLevel) {
    nowMs = 1000;
    for (int& v : pinLevel) v = HIGH;
    for (int& v : analogOut) v = 0;
    pinLevel[EXIT_PIN] = exitSensorLevel;
    rec = hoststub::Recorder{};
    hoststub::visionReady = false;
    MechLink::signalLost = true;
    Serial.input.clear();
    Serial.output.clear();
    WiredControl::init();
    Dribbler::init();
    Dribbler::setFeedAllowed(false);
    Dribbler::consumeJamEvent();
}

// One main-loop tick in production order (WiredControl before Dribbler).
void step(unsigned long ms = 10) {
    nowMs += ms;
    WiredControl::update();
    Dribbler::update();
}

void send(const std::string& line) { Serial.send(line + "\n"); }

bool saw(const std::string& event) {
    return Serial.output.find("$EVENT," + event + "\n") != std::string::npos;
}

bool feedRunning() { return analogOut[FEED_PWM_PIN] > 0; }

// Advance time while the page keeps sending heartbeats every 50ms.
[[maybe_unused]] void runWithHeartbeat(unsigned long totalMs) {
    for (unsigned long t = 0; t < totalMs; t += 10) {
        if (t % 50 == 0) send("K");
        step(10);
    }
}

[[maybe_unused]] void arm() {
    send("E");
    step();
    assert(WiredControl::isArmed());
    Serial.output.clear();
}

bool nearly(double a, double b) { return a - b < 1e-9 && b - a < 1e-9; }

#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)

void testEverythingRefusedWhileDisarmed() {
    reset(SENSOR_CLEAR);
    const char* commands[] = {"R", "F", "L", "T,1,1", "ROT,1", "ANG,1", "FLY,0.5"};
    for (const char* c : commands) {
        Serial.output.clear();
        send(c);
        step();
        assert(saw("REJECTED_DISARMED"));
    }
    runWithHeartbeat(4000);
    assert(!WiredControl::isArmed());
    assert(!feedRunning());
    assert(Dribbler::getControlMode() == DribblerControlMode::DISABLED);
    assert(nearly(rec.chassisLeft, 0.0) && nearly(rec.flywheelCommand, 0.0));
}

// Codex repro: "X後R，armed=0 feeding=1；4秒後仍armed=0 feeding=1".
void testRAfterDisableStaysOff() {
    reset(SENSOR_CLEAR);
    arm();
    send("X");
    step();
    assert(!WiredControl::isArmed());
    send("R");
    step();
    assert(saw("REJECTED_DISARMED"));
    runWithHeartbeat(4000);
    assert(!WiredControl::isArmed());
    assert(!feedRunning());
}

void testDisarmedDefenseStopsStrayManualFeed() {
    reset(SENSOR_CLEAR);
    Dribbler::setFeedOpenLoop(0.7);  // something left the feed on
    assert(feedRunning());
    step();
    assert(!feedRunning());
}

void testEnableDisableDrivesEverySubsystem() {
    reset(SENSOR_CLEAR);
    arm();
    assert(rec.chassisEnabled && rec.shooterEnabled);
    send("R");
    send("FLY,0.6");
    step();
    assert(feedRunning() && nearly(rec.flywheelCommand, 0.6));
    send("X");  // Enter
    step();
    assert(saw("OUTPUTS_DISABLED"));
    assert(!rec.chassisEnabled && !rec.shooterEnabled);
    assert(!feedRunning() && nearly(rec.flywheelCommand, 0.0));
}

void testWatchdogTimeoutStopsEverything() {
    reset(SENSOR_CLEAR);
    arm();
    send("T,1,1");
    send("FLY,0.5");
    send("R");
    step();
    assert(feedRunning() && rec.chassisLeft > 0.0);
    for (int i = 0; i < 30; i++) step(10);  // 300ms of silence
    assert(saw("COMMAND_TIMEOUT"));
    assert(!WiredControl::isArmed() && WiredControl::didTimeOut());
    assert(!rec.chassisEnabled && !rec.shooterEnabled && !feedRunning());
    assert(nearly(rec.flywheelCommand, 0.0));
}

void testLateHeartbeatCannotRescueExpiredSession() {
    reset(SENSOR_CLEAR);
    arm();
    nowMs += 300;  // past the 250ms deadline...
    send("K");     // ...and the heartbeat only shows up now
    WiredControl::update();
    assert(saw("COMMAND_TIMEOUT"));
    assert(!WiredControl::isArmed());
}

void testHeartbeatKeepsSessionAlive() {
    reset(SENSOR_CLEAR);
    arm();
    runWithHeartbeat(2000);
    assert(WiredControl::isArmed());
}

void testRcLinkTakesPriority() {
    reset(SENSOR_CLEAR);
    MechLink::signalLost = false;
    send("E");
    step();
    assert(saw("ENABLE_REJECTED_RC_LINK_ACTIVE") && !WiredControl::isArmed());

    MechLink::signalLost = true;
    arm();
    send("R");
    step();
    MechLink::signalLost = false;  // RC link comes up mid-session
    step();
    assert(saw("DISABLED_RC_LINK_ACTIVE"));
    assert(!WiredControl::isArmed() && !WiredControl::isActive());
    assert(!rec.chassisEnabled && !rec.shooterEnabled && !feedRunning());
}

void testManualAxesAreScaled() {
    reset(SENSOR_CLEAR);
    arm();
    send("T,1,-1");  // A held: left forward, right backward
    send("ROT,1");
    send("ANG,-1");
    step();
    using namespace WiredControlConstants;
    assert(nearly(rec.chassisLeft, CHASSIS_MANUAL_SCALE));
    assert(nearly(rec.chassisRight, -CHASSIS_MANUAL_SCALE));
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, ROTATE_MANUAL_SCALE));
    assert(nearly(rec.angleCommand, -ANGLE_MANUAL_SCALE));
}

void testFlywheelRangeAndParsing() {
    reset(SENSOR_CLEAR);
    arm();
    send("FLY,1.5");
    step();
    assert(nearly(rec.flywheelCommand, 1.0));
    send("FLY,-0.3");
    step();
    assert(nearly(rec.flywheelCommand, 0.0));
    send("FLY,abc");
    step();
    assert(saw("BAD_COMMAND"));
    Serial.output.clear();
    send("T,2");
    step();
    assert(saw("BAD_COMMAND"));
    Serial.output.clear();
    send("Q");
    step();
    assert(saw("UNKNOWN_COMMAND"));
}

void testLTogglesAndReenableReturnsToManual() {
    reset(SENSOR_CLEAR);
    arm();
    send("ROT,1");
    send("L");
    step();
    assert(saw("ROTATE_VISION_FOLLOW"));
    assert(rec.rotateMode == "disabled");  // vision not ready -> safe disable
    hoststub::visionReady = true;
    send("K");
    step();
    assert(rec.rotateMode == "vision");
    send("L");
    step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, 0.0));  // stale jog cleared
    send("L");
    send("X");
    step();
    arm();
    send("K");
    step();
    assert(rec.rotateMode == "manual");
}

// Codex P1: a ball already in the exit beam when F is pressed is the ball
// that goes out; its clear edge must end the request.
void testSingleShotWithBallAlreadyBlocking() {
    reset(SENSOR_BLOCKED);
    arm();
    const unsigned long shotsBefore = Dribbler::getCompletedShotCount();
    send("F");
    step();
    assert(saw("FEED_SINGLE_START") && feedRunning());
    pinLevel[EXIT_PIN] = SENSOR_CLEAR;
    send("K");
    step();
    step();  // past the 8ms debounce
    assert(Dribbler::getShootRemaining() == 0);
    assert(Dribbler::getCompletedShotCount() == shotsBefore + 1);
    assert(!feedRunning());
    send("K");
    step();
    assert(saw("FEED_SINGLE_DONE"));
}

void testSingleShotFromClearBeam() {
    reset(SENSOR_CLEAR);
    arm();
    const unsigned long shotsBefore = Dribbler::getCompletedShotCount();
    send("F");
    step();
    assert(feedRunning());
    pinLevel[EXIT_PIN] = SENSOR_BLOCKED;
    step();
    step();
    assert(feedRunning() && Dribbler::getShootRemaining() == 1);  // in the beam, not out yet
    pinLevel[EXIT_PIN] = SENSOR_CLEAR;
    send("K");
    step();
    step();
    assert(Dribbler::getShootRemaining() == 0);
    assert(Dribbler::getCompletedShotCount() == shotsBefore + 1);
    assert(!feedRunning());
}

void testJamTimeoutAndBusyFDoesNotRestartIt() {
    reset(SENSOR_CLEAR);
    arm();
    send("F");
    step();
    const unsigned long started = nowMs;
    runWithHeartbeat(1000);
    Serial.output.clear();
    send("F");  // pressed again while the first is still in flight
    step();
    assert(saw("FEED_BUSY"));
    assert(Dribbler::getShootRemaining() == 1);
    while (nowMs < started + DribblerConstants::Jam::TIMEOUT_MS - 20) {
        if ((nowMs / 10) % 5 == 0) send("K");
        step();
    }
    assert(feedRunning());  // not yet
    runWithHeartbeat(60);
    assert(saw("FEED_JAM"));  // jam clock was NOT reset by the busy F
    assert(!feedRunning() && Dribbler::getShootRemaining() == 0);
    assert(WiredControl::isArmed());
}

void testFInterruptsR() {
    reset(SENSOR_CLEAR);
    arm();
    send("R");
    step();
    assert(Dribbler::getControlMode() == DribblerControlMode::MANUAL_OPEN_LOOP);
    send("F");
    step();
    assert(Dribbler::getControlMode() == DribblerControlMode::COUNTED_FEED);
    assert(Dribbler::getShootRemaining() == 1);
    // After the single shot completes, R does not resume.
    pinLevel[EXIT_PIN] = SENSOR_BLOCKED;
    step(); step();
    pinLevel[EXIT_PIN] = SENSOR_CLEAR;
    send("K");
    step(); step();
    runWithHeartbeat(200);
    assert(!feedRunning());
    assert(Dribbler::getControlMode() == DribblerControlMode::DISABLED);
}

#else

void testEnableRefusedUntilPinTableConfirmed() {
    reset(SENSOR_CLEAR);
    send("E");
    step();
    assert(saw("ENABLE_REJECTED_PINS_UNCONFIRMED"));
    assert(!WiredControl::isArmed());
    assert(!rec.chassisEnabled && !rec.shooterEnabled);
    send("R");
    send("T,1,1");
    step();
    assert(!feedRunning() && nearly(rec.chassisLeft, 0.0));
}

#endif

}  // namespace

int main() {
#if defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
    testEverythingRefusedWhileDisarmed();
    testRAfterDisableStaysOff();
    testDisarmedDefenseStopsStrayManualFeed();
    testEnableDisableDrivesEverySubsystem();
    testWatchdogTimeoutStopsEverything();
    testLateHeartbeatCannotRescueExpiredSession();
    testHeartbeatKeepsSessionAlive();
    testRcLinkTakesPriority();
    testManualAxesAreScaled();
    testFlywheelRangeAndParsing();
    testLTogglesAndReenableReturnsToManual();
    testSingleShotWithBallAlreadyBlocking();
    testSingleShotFromClearBeam();
    testJamTimeoutAndBusyFDoesNotRestartIt();
    testFInterruptsR();
    printf("wired_control_test (pin table confirmed): all asserts passed\n");
#else
    testEnableRefusedUntilPinTableConfirmed();
    printf("wired_control_test (pin table unconfirmed gate): all asserts passed\n");
#endif
    return 0;
}
