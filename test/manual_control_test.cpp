// Host regression test for operator control: compiles the REAL
// src/IO/ManualControl.cpp, src/Auto/Auto.cpp, src/Dribbler/Dribbler.cpp and
// src/IO/RcControlMap.c against fake Arduino time/pins/Serial
// (test/host_stubs/), with Chassis/Shooter/Vision replaced by recorders and
// MechLink replaced by a stub the test feeds RC frames into.
//
// Covers the wired USB path (everything Codex's reviews of e4a70a5/afed238
// asked for), the wireless ELRS path, control authority between the two,
// the feed-tower trigger pulse, and the operator modes (M / semi-auto F /
// full-auto firing).
//
// Build twice (see test/README):
//   with -DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=true  -> full armed-path suite
//   without it                                     -> pin-table gate only
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <string>

#include "host_stubs/host_stubs.h"

#include "../src/Auto/Auto.h"
#include "../src/Constants/DribblerConstants.h"
#include "../src/Constants/ShooterConstants.h"
#include "../src/Constants/TuningConstants.h"
#include "../src/Dribbler/Dribbler.h"
#include "../src/IO/ManualControl.h"
#include "../src/IO/MechLink.h"
#include "../src/Control/ShotTableLookup.h"
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
    hoststub::visionValid = false;
    hoststub::visionDistanceMm = 0.0;
    hoststub::shooterReady = false;
    MechLink::signalLost = true;
    MechLink::frameCount = 0;
    wireless = rcmap_command{};
    Serial.input.clear();
    Serial.output.clear();
    ManualControl::init();
    Auto::init();
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

// One main-loop tick in production order (ManualControl, Auto, Dribbler).
void step(unsigned long ms = 10) {
    nowMs += ms;
    ManualControl::update();
    Auto::update();
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
    for (const char* c : {"R", "F", "L", "T,1,1", "ROT,1", "ANG,1", "FLY,0.5"}) {
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

void testConveyorIndependent() {
    reset();
    armWired();
    send("R");
    step();
    assert(Dribbler::isConveyorOn() && saw("CONVEYOR_ON"));
    send("F"); step();
    assert(triggerHigh() && Dribbler::isConveyorOn());
    send("R"); step();
    assert(!Dribbler::isConveyorOn() && triggerHigh());
    send("R"); step();
    send("X"); step();
    assert(!Dribbler::isConveyorOn() && !triggerHigh());
    armWired(); send("R"); step(); step(300);
    assert(!Dribbler::isConveyorOn());
    reset(); wireless.armed = false; rcStep();
    wireless.armed = true; rcStep();
    wireless.conveyor = true; rcStep(); rcStep();
    assert(Dribbler::isConveyorOn());
    MechLink::signalLost = true; step();
    assert(!Dribbler::isConveyorOn());
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
    // The code field carries the event and, in its top bits, the mode.
    assert(rcmap_status_code_event(rec.statusCode) == RCMAP_EVENT_OUTPUTS_ENABLED);
    assert(rcmap_status_code_mode(rec.statusCode) == RCMAP_MODE_FULL_MANUAL);
    const unsigned long seq = rec.statusSeq;
    wireless.feed_counter = 1;
    rcStep(120);
    assert(rcmap_status_code_event(rec.statusCode) == RCMAP_EVENT_FEED_TRIGGER_SENT &&
           rec.statusSeq == seq + 1);
    assert(rec.statusFlags & RCMAP_STATUS_FEED_BUSY);
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep(120);
    assert(rcmap_status_code_mode(rec.statusCode) == RCMAP_MODE_SEMI_AUTO);
}

// ----------------------------------------------------------- operator modes

// Keep the wired session alive over `totalMs` while Auto runs.
void runArmed(unsigned long totalMs) {
    for (unsigned long t = 0; t < totalMs; t += 10) {
        if (t % 50 == 0) send("K");
        step(10);
    }
}

unsigned long shotsDuring(unsigned long totalMs) {
    const unsigned long before = Dribbler::getTriggerCount();
    runArmed(totalMs);
    return Dribbler::getTriggerCount() - before;
}

void testModeKeyWired() {
    reset();
    send("M"); step();
    assert(saw("REJECTED_DISARMED"));                      // disarmed: refused
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);

    armWired();
    send("FLY,0.6"); step();
    assert(nearly(rec.flywheelCommand, 0.6));
    send("M"); step();
    assert(saw("MODE_SEMI_AUTO"));
    assert(ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    send("M"); step();
    assert(saw("MODE_FULL_AUTO") && ManualControl::getOperatorMode() == OperatorMode::FULL_AUTO);
    send("M"); step();
    assert(saw("MODE_FULL_MANUAL") && ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    // Leaving auto never jumps the flywheel back to the old slider value.
    assert(rec.flywheelCommand == 0.0);

    // Any disarm returns to manual, and a new Enable starts manual.
    send("M"); step();
    assert(ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    send("X"); step();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    armWired();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    send("M"); step();
    runWithHeartbeat(0);
    step(ManualControlConstants::COMMAND_TIMEOUT_MS + 20);  // watchdog
    assert(!ManualControl::isArmed());
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
}

void testSemiAutoFiresOnlyWhenReady() {
    reset();
    hoststub::visionReady = true;
    armWired();
    send("M"); step();                                    // semi-auto
    // Auto has the turret and flywheel; the slider is ignored.
    assert(rec.rotateMode == "vision");
    assert(nearly(rec.flywheelCommand, TuningConstants::Flywheel::SHOOT_COMMAND));
    send("FLY,0.2"); step();
    assert(nearly(rec.flywheelCommand, TuningConstants::Flywheel::SHOOT_COMMAND));
    // The operator still drives the chassis and elevation.
    send("T,1,1"); send("ANG,-1"); step();
    assert(nearly(rec.chassisLeft, 1.0) && nearly(rec.angleCommand, -1.0));

    // Not locked yet: F is refused, not queued.
    Serial.output.clear();
    send("F"); step();
    assert(saw("FIRE_NOT_READY") && !triggerHigh());
    runArmed(200);
    assert(Dribbler::getTriggerCount() == 0);              // no late shot

    // Locked and spun up: F fires exactly one ball.
    hoststub::shooterReady = true;
    step();
    Serial.output.clear();
    send("F"); step();
    assert(saw("FEED_TRIGGER_SENT") && triggerHigh());
    assert(Dribbler::getTriggerCount() == 1);
    // Right after a shot: refused again (cooldown / tower lockout).
    Serial.output.clear();
    send("F"); step();
    assert(saw("FIRE_NOT_READY") || saw("FEED_BUSY"));
    assert(Dribbler::getTriggerCount() == 1);
    // Without F, semi-auto never fires on its own.
    assert(shotsDuring(3000) == 0);
}

void testFullAutoFiresByItself() {
    reset();
    hoststub::visionReady = true;
    armWired();
    send("M"); send("M"); step();                         // full auto
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_AUTO);
    assert(shotsDuring(500) == 0);                         // not ready: holds fire
    hoststub::shooterReady = true;
    // Fires whenever ready, paced by the tower's 1000 ms lockout (longer
    // than Auto's own 300 ms cooldown): ~3 shots in 2.5 s.
    const unsigned long shots = shotsDuring(2500);
    assert(shots >= 2 && shots <= 3);
    // Losing the lock stops it.
    hoststub::shooterReady = false;
    assert(shotsDuring(2000) == 0);
    // Back to manual: Auto lets go, nothing fires by itself.
    hoststub::shooterReady = true;
    send("M"); step();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    assert(shotsDuring(2500) == 0);
}

void testAutoWithoutCamera() {
    reset();
    hoststub::visionReady = false;
    hoststub::shooterReady = true;
    armWired();
    send("M"); send("M"); step();                         // full auto, no camera
    assert(rec.rotateMode == "disabled" && rec.flywheelCommand == 0.0);
    assert(shotsDuring(2000) == 0);
    send("M"); send("M"); step();                         // -> semi
    Serial.output.clear();
    send("F"); step();
    assert(saw("FIRE_NOT_READY") && Dribbler::getTriggerCount() == 0);
}

void testShotTableDrivesFlywheel() {
    reset();
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 5000.0;  // 5 m
    armWired();
    send("M"); step();
    const ShotTableLookup::Result expected = ShotTableLookup::lookup(
        5.0, TuningConstants::ShotTable::TABLE, TuningConstants::ShotTable::TABLE_COUNT);
    assert(nearly(rec.flywheelCommand, expected.flywheelCommand));
}

void testElevationTargetCommands() {
    reset();
    send("ANGT,90"); step();
    assert(saw("REJECTED_DISARMED") && rec.angleTargetSets == 0);
    armWired();
    send("ANGT,47.5"); step();
    assert(nearly(rec.angleTargetDegrees, 47.5));
    // Bad numbers are refused, not clamped into a move; the old step
    // command no longer exists.
    const unsigned long sets = rec.angleTargetSets;
    send("ANGT,nan"); send("ANGT,"); send("ANGT,abc"); step();
    assert(saw("BAD_COMMAND") && rec.angleTargetSets == sets);
    send("ANGS,1"); step();
    assert(saw("UNKNOWN_COMMAND") && rec.angleTargetSets == sets);
    // The arrows' hold-to-move jog is unchanged: ANG -> open-loop jog.
    send("ANG,1"); step();
    assert(nearly(rec.angleCommand, 1.0 * ManualControlConstants::ANGLE_MANUAL_SCALE));
    send("ANG,0"); step();
    assert(nearly(rec.angleCommand, 0.0));

    // The shot table's elevation pulses are placeholders until measured:
    // in auto with a target in view, Auto must not move the elevation, and
    // the operator keeps it.
    static_assert(!TuningConstants::ShotTable::ELEVATION_MEASURED,
                  "update this test when the shot table's elevation is measured");
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 4000.0;
    send("M"); step();
    runArmed(200);
    assert(rec.anglePulseSets == 0);
    send("ANGT,100"); step();
    assert(nearly(rec.angleTargetDegrees, 100.0) && !saw("REJECTED_AUTO_MODE"));
}

void testWirelessMode() {
    reset();
    hoststub::visionReady = true;
    armWireless();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep();
    assert(saw("MODE_SEMI_AUTO") && ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    // F (counter step) in semi-auto goes through Auto: not ready -> refused.
    Serial.output.clear();
    wireless.feed_counter = 1;
    rcStep();
    assert(saw("FIRE_NOT_READY") && Dribbler::getTriggerCount() == 0);
    hoststub::shooterReady = true;
    rcStep();
    wireless.feed_counter = 2;
    rcStep();
    assert(Dribbler::getTriggerCount() == 1);
    // Arm low disarms and resets to manual.
    wireless.armed = false;
    rcStep();
    assert(!ManualControl::isArmed() &&
           ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    // An Enable whose CH4 still says auto is refused.
    Serial.output.clear();
    wireless.mode_us = RCMAP_MODE_FULL_AUTO_US;
    wireless.armed = true;
    rcStep();
    assert(saw("ENABLE_REJECTED_NOT_MANUAL") && !ManualControl::isArmed());
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
#if defined(TEL_WIRED_MANUAL_ONLY)
    reset();
    wireless.armed = false; rcStep(); wireless.armed = true; rcStep();
    assert(!ManualControl::isArmed());
    armWired(); send("L"); step();
    assert(rec.rotateMode == "manual");
    send("T,1,-1"); send("ROT,1"); send("ANG,-1"); send("FLY,0.6"); step();
    assert(nearly(rec.chassisLeft, 1.0) && nearly(rec.chassisRight, -1.0));
    assert(nearly(rec.rotateCommand, 1.0) && nearly(rec.angleCommand, -1.0));
    assert(nearly(rec.flywheelCommand, 0.6));
    // No auto modes on the wired benches: M stays in full manual.
    send("M"); step();
    assert(saw("MODE_FULL_MANUAL") &&
           ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    send("X"); step(); assert(!ManualControl::isArmed());
    printf("wired manual profile: all asserts passed\n");
#elif defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED)
    testWiredRefusedWhileDisarmed();
    testConveyorIndependent();
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
    testStatusUplink();
    testModeKeyWired();
    testSemiAutoFiresOnlyWhenReady();
    testFullAutoFiresByItself();
    testAutoWithoutCamera();
    testShotTableDrivesFlywheel();
    testWirelessMode();
    testElevationTargetCommands();
    printf("manual_control_test (pin table confirmed): all asserts passed\n");
#else
    testEnableRefusedUntilPinTableConfirmed();
    printf("manual_control_test (pin table unconfirmed gate): all asserts passed\n");
#endif
    return 0;
}
