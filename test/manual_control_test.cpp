// Host regression test for operator control: compiles the REAL
// src/IO/ManualControl.cpp, src/Auto/Auto.cpp, src/Dribbler/Dribbler.cpp and
// src/IO/RcControlMap.c against fake Arduino time/pins/Serial
// (test/host_stubs/), with Chassis/Shooter/Vision replaced by recorders and
// MechLink replaced by a stub the test feeds RC frames into.
//
// Covers the wired USB path (everything Codex's reviews of e4a70a5/afed238
// asked for), the wireless ELRS path (channel table v2: forward/turn, fire
// edge, 3x3 target, nudge, auto-path edge), control authority between the
// two, the feed-tower trigger pulse, the operator modes (M / semi-auto F /
// full-auto firing), and the phase-1 target / auto-path records.
//
// Build twice (see test/README):
//   -DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=true   -> full armed-path suite
//   -DTEL_HOST_TEST_PIN_TABLE_CONFIRMED=false  -> pin-table gate only
//   (without it, like the firmware since 2026-10-16: armed suite)
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
    hoststub::visionTargetId = 8;
    hoststub::visionTx = 0.0;
    hoststub::anglePulseUs = 1500;
    hoststub::visionTy = 0.0;
    hoststub::shooterReady = false;
    hoststub::turretZeroOk = true;
    hoststub::turretState = 4;
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

// The operator's turret jog direction (TuningConstants::Rotate::
// MANUAL_JOG_INVERTED, flipped after the 2026-10-17 robot test): ROT +1 /
// nudge right reaches Shooter as this sign.
[[maybe_unused]] constexpr double JOG = TuningConstants::Rotate::MANUAL_JOG_INVERTED ? -1.0 : 1.0;
static_assert(TuningConstants::Rotate::MANUAL_JOG_INVERTED, "robot-verified 2026-10-17");

// The camera sees the (selected) hole with a depth distance: 4 m (the
// shot table interpolates there; tests look the expected value up).
[[maybe_unused]] void targetInView() {
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 4000.0;
}

#if !defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED) || TEL_HOST_TEST_PIN_TABLE_CONFIRMED

// ------------------------------------------------------------------ wired

void testWiredRefusedWhileDisarmed() {
    reset();
    for (const char* c : {"F", "T,1,1", "ROT,1", "ANG,1", "FLY,0.5", "TGT,5", "AUTO,1"}) {
        Serial.output.clear();
        send(c);
        step();
        assert(saw("REJECTED_DISARMED"));
    }
    runWithHeartbeat(2000);
    // Disabled: the tower line is held HIGH (and no trigger was counted).
    assert(!ManualControl::isArmed() && triggerHigh());
    assert(nearly(rec.chassisLeft, 0.0) && nearly(rec.flywheelCommand, 0.0));
    assert(Dribbler::getTriggerCount() == 0);
    assert(ManualControl::getTarget() == RCMAP_TARGET_NONE);
    assert(ManualControl::getAutoPathsUsedMask() == 0);
}

// R (conveyor) and L (vision-follow toggle) were retired with table v2.
void testRetiredKeys() {
    reset();
    armWired();
    for (const char* c : {"R", "L"}) {
        Serial.output.clear();
        send(c);
        step();
        assert(saw("UNKNOWN_COMMAND") && ManualControl::isArmed());
    }
    assert(rec.rotateMode == "manual");
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
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, JOG * ROTATE_MANUAL_SCALE));
    assert(nearly(rec.angleCommand, -ANGLE_MANUAL_SCALE) && nearly(rec.flywheelCommand, 1.0));
    for (const char* bad : {"FLY,nan", "FLY,inf", "ROT,-1e999", "T,2", "T,1,1,1", "FLY,abc",
                            "TGT,13", "TGT,-1", "TGT,", "TGT,5x", "AUTO,0", "AUTO,5", "AUTO,"}) {
        Serial.output.clear();
        send(bad);
        ManualControl::update();
        assert(saw("BAD_COMMAND"));
    }
    step();
    assert(nearly(rec.flywheelCommand, 1.0)); // bad values changed nothing
    assert(ManualControl::getTarget() == RCMAP_TARGET_NONE);
    assert(ManualControl::getAutoPathThisEnable() == RCMAP_AUTO_NONE);
    Serial.output.clear();
    send("Q"); // the Dashboard maps the Q key to AUTO,1; a bare Q is unknown
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

// ------------------------------------------------- target / auto-path keys

void testTargetSelection() {
    reset();
    armWired();
    send("TGT,5");
    step();
    assert(saw("TARGET_SELECTED") && ManualControl::getTarget() == 5);
    // Same hole again: no new event.
    Serial.output.clear();
    send("TGT,5");
    step();
    assert(!saw("TARGET_SELECTED") && ManualControl::getTarget() == 5);
    send("TGT,9");
    step();
    assert(saw("TARGET_SELECTED") && ManualControl::getTarget() == 9);
    // The bonus holes (Q W E, 2026-10-27).
    for (uint8_t hole = 10; hole <= 12; hole++) {
        Serial.output.clear();
        send("TGT," + std::to_string(hole));
        step();
        assert(saw("TARGET_SELECTED") && ManualControl::getTarget() == hole);
    }
    send("TGT,0");
    step();
    assert(ManualControl::getTarget() == RCMAP_TARGET_NONE);
    // Phase 1 only records it: nothing moves.
    assert(nearly(rec.chassisLeft, 0.0) && nearly(rec.rotateCommand, 0.0) &&
           Dribbler::getTriggerCount() == 0);
}

void testAutoPathOncePerEnable() {
    reset();
    armWired();
    send("AUTO,1");
    step();
    assert(saw("AUTO_PATH_STARTED") && ManualControl::getAutoPathThisEnable() == 1);
    assert(ManualControl::getAutoPathsUsedMask() == (1u << 1));
    // A second path (or the same one) in the same Enable: refused.
    Serial.output.clear();
    send("AUTO,2");
    send("AUTO,1");
    step();
    assert(saw("AUTO_PATH_REJECTED") && !saw("AUTO_PATH_STARTED"));
    assert(ManualControl::getAutoPathThisEnable() == 1 &&
           ManualControl::getAutoPathsUsedMask() == (1u << 1));
    // A new Enable allows one more; the since-boot record is kept.
    send("X");
    step();
    assert(ManualControl::getAutoPathsUsedMask() == (1u << 1));
    armWired();
    assert(ManualControl::getAutoPathThisEnable() == RCMAP_AUTO_NONE);
    send("AUTO,4");
    step();
    assert(saw("AUTO_PATH_STARTED") && ManualControl::getAutoPathThisEnable() == 4);
    assert(ManualControl::getAutoPathsUsedMask() == ((1u << 1) | (1u << 4)));
    // Only a reboot clears the record.
    ManualControl::init();
    assert(ManualControl::getAutoPathsUsedMask() == 0);
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
    // Disabled: held HIGH from now on, and no new trigger is accepted.
    assert(triggerHigh() && !Dribbler::isEnabled());
    const unsigned long count = Dribbler::getTriggerCount();
    runWithHeartbeat(300);
    assert(triggerHigh() && Dribbler::getTriggerCount() == count);
    // Enable again: idle LOW.
    armWired();
    assert(!triggerHigh() && Dribbler::isEnabled());
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
    using namespace ManualControlConstants;
    // Stick Y forward: both sides forward.
    wireless.forward = 1.0f;
    rcStep();
    assert(nearly(rec.chassisLeft, CHASSIS_MANUAL_SCALE) && nearly(rec.chassisRight, CHASSIS_MANUAL_SCALE));
    // Stick X right: spin right (left back, right forward), as the Dashboard's T,-1,1.
    wireless.forward = 0.0f;
    wireless.turn = 1.0f;
    rcStep();
    assert(nearly(rec.chassisLeft, -CHASSIS_MANUAL_SCALE) && nearly(rec.chassisRight, CHASSIS_MANUAL_SCALE));
    // Both full: clamped, not over-driven.
    wireless.forward = 1.0f;
    rcStep();
    assert(nearly(rec.chassisLeft, 0.0) && nearly(rec.chassisRight, CHASSIS_MANUAL_SCALE));
    wireless.forward = 0.0f;
    wireless.turn = 0.0f;
    // Nudge: one direction at a time, hold-to-move.
    wireless.nudge = RCMAP_NUDGE_UP;
    rcStep();
    assert(nearly(rec.angleCommand, ANGLE_MANUAL_SCALE) && nearly(rec.rotateCommand, 0.0));
    wireless.nudge = RCMAP_NUDGE_DOWN;
    rcStep();
    assert(nearly(rec.angleCommand, -ANGLE_MANUAL_SCALE));
    wireless.nudge = RCMAP_NUDGE_RIGHT;
    rcStep();
    assert(nearly(rec.rotateCommand, JOG * ROTATE_MANUAL_SCALE) && nearly(rec.angleCommand, 0.0));
    wireless.nudge = RCMAP_NUDGE_LEFT;
    rcStep();
    assert(nearly(rec.rotateCommand, -JOG * ROTATE_MANUAL_SCALE));
    wireless.nudge = RCMAP_NUDGE_CENTER;
    rcStep();
    assert(nearly(rec.rotateCommand, 0.0) && nearly(rec.angleCommand, 0.0));
    // Flywheel off unless the ground sends it (CH4, see testWirelessFlywheel).
    assert(nearly(rec.flywheelCommand, 0.0));
    // Arm released by the ground: disarm.
    wireless.armed = false;
    rcStep();
    assert(saw("OUTPUTS_DISABLED") && !ManualControl::isArmed() && !rec.chassisEnabled);
}

void testWirelessFireEdgeNoRepeatNoReplay() {
    reset();
    armWireless();
    wireless.fire = true;
    runWireless(200); // held: the same "on" repeats in every frame
    assert(Dribbler::getTriggerCount() == 1);
    wireless.fire = false;
    runWireless(DribblerConstants::Tower::COOLDOWN_MS);
    wireless.fire = true;
    rcStep();
    assert(Dribbler::getTriggerCount() == 2);

    // Fire held through an RF outage is not replayed after it.
    MechLink::signalLost = true;
    step(300);
    assert(saw("DISABLED_RC_LINK_LOST") && !ManualControl::isArmed());
    // ...and recovering the link does not re-enable on its own, even with
    // the ground still holding Enable.
    runWireless(200);
    assert(!ManualControl::isArmed());
    armWireless();
    runWireless(DribblerConstants::Tower::COOLDOWN_MS);
    assert(Dribbler::getTriggerCount() == 2);
    // A fresh release + press fires.
    wireless.fire = false;
    rcStep();
    wireless.fire = true;
    rcStep();
    assert(Dribbler::getTriggerCount() == 3);
}

void testWirelessPressWhileDisarmedNotLatched() {
    reset();
    wireless.armed = false;
    rcStep();
    wireless.fire = true;
    rcStep();
    assert(saw("REJECTED_DISARMED"));
    armWireless();
    runWireless(200);
    assert(Dribbler::getTriggerCount() == 0);
}

void testWirelessTargetAndAutoPath() {
    reset();
    armWireless();
    wireless.target = 7;
    runWireless(100); // repeated every frame: one event
    assert(ManualControl::getTarget() == 7);
    assert(Serial.output.find("$EVENT,TARGET_SELECTED\n") ==
           Serial.output.rfind("$EVENT,TARGET_SELECTED\n"));
    wireless.auto_path = 3;
    runWireless(200); // the ground's 200 ms pulse: starts once
    assert(saw("AUTO_PATH_STARTED") && ManualControl::getAutoPathThisEnable() == 3);
    assert(!saw("AUTO_PATH_REJECTED"));
    wireless.auto_path = RCMAP_AUTO_NONE;
    rcStep();
    Serial.output.clear();
    wireless.auto_path = 4;
    rcStep();
    assert(saw("AUTO_PATH_REJECTED") && ManualControl::getAutoPathThisEnable() == 3);
    assert(ManualControl::getAutoPathsUsedMask() == (1u << 3));

    // An auto press held through a link outage does not start after it.
    MechLink::signalLost = true;
    step(300);
    armWireless();
    runWireless(200);
    assert(!saw("AUTO_PATH_STARTED") && ManualControl::getAutoPathThisEnable() == RCMAP_AUTO_NONE);
    wireless.auto_path = RCMAP_AUTO_NONE;
    rcStep();
    wireless.auto_path = 1;
    rcStep();
    assert(saw("AUTO_PATH_STARTED") && ManualControl::getAutoPathThisEnable() == 1);
    assert(ManualControl::getAutoPathsUsedMask() == ((1u << 1) | (1u << 3)));
}

void testWirelessInvalidFramesTimeOut() {
    reset();
    armWireless();
    for (int i = 0; i < 20; i++) {
        rcFrame();
        MechLink::channels[RCMAP_CH_TARGET] = 1120; // between target levels (1080 / 1160)
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
    wireless.fire = true;
    rcStep();
    assert(saw("REJECTED_NOT_OWNER") && Dribbler::getTriggerCount() == 0);
    Serial.output.clear();
    wireless.auto_path = 2;
    rcStep();
    assert(saw("REJECTED_NOT_OWNER") && ManualControl::getAutoPathsUsedMask() == 0);
    wireless.fire = false;
    wireless.auto_path = RCMAP_AUTO_NONE;
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
    runWireless(320); // status: slots 0 and 2 of 5 (100 ms each), so at most 300 ms apart
    assert(rec.statusFlags & RCMAP_STATUS_ARMED);
    assert(rec.statusFlags & RCMAP_STATUS_OWNER_WIRELESS);
    assert(rec.statusFlags & RCMAP_STATUS_RC_LINK_OK);
    // The code field carries the event and, in its top bits, the mode.
    assert(rcmap_status_code_event(rec.statusCode) == RCMAP_EVENT_OUTPUTS_ENABLED);
    assert(rcmap_status_code_mode(rec.statusCode) == RCMAP_MODE_FULL_MANUAL);
    const unsigned long seq = rec.statusSeq;
    wireless.fire = true;
    runWireless(320);
    assert(rcmap_status_code_event(rec.statusCode) == RCMAP_EVENT_FEED_TRIGGER_SENT &&
           rec.statusSeq == seq + 1);
    assert(rec.statusFlags & RCMAP_STATUS_FEED_BUSY);
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    runWireless(320);
    assert(rcmap_status_code_mode(rec.statusCode) == RCMAP_MODE_SEMI_AUTO);
}

// Aim and vision frames (2026-10-17): the 100 ms status slots go status,
// aim, status, vision, for the wireless Dashboard.
rcmap_aim lastAim() {
    rcmap_status st = {};
    st.flags = rec.aimFlags;
    st.event_seq = rec.aimSeq;
    st.event_code = rec.aimCode;
    rcmap_aim a = {};
    assert(rcmap_aim_from_status(&st, &a));
    return a;
}

void testAimUplink() {
    reset();
    // Status, aim, status, vision, seen (2026-10-27); one line per 100 ms.
    step(100);
    assert(rec.statusCount == 1 && rec.aimCount == 0 && rec.visionCount == 0);
    step(100);
    assert(rec.statusCount == 1 && rec.aimCount == 1 && rec.visionCount == 0);
    step(100);
    assert(rec.statusCount == 2 && rec.aimCount == 1 && rec.visionCount == 0);
    step(100);
    assert(rec.statusCount == 2 && rec.aimCount == 1 && rec.visionCount == 1);
    step(100);
    assert(rec.seenCount == 1 && rec.statusCount == 2);
    for (int i = 0; i < 7; i++) step(100);
    assert(rec.statusCount == 5 && rec.aimCount == 3 && rec.visionCount == 2 && rec.seenCount == 2);
    {
        rcmap_status st = {};
        st.event_code = rec.seenCode;
        rcmap_seen seen = {};
        assert(rcmap_seen_from_status(&st, &seen) && seen.seen_mask == 0 &&
               seen.state == RCMAP_VISION_STATE_NONE);
    }
    // Nothing in view: no distance, not valid.
    rcmap_aim a = lastAim();
    assert(a.distance_cm == 0 && !a.valid && a.target_id == 0 && a.elevation_pulse_us == 1500);

    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 3524.0;   // -> 352 cm
    hoststub::visionTargetId = 10;
    hoststub::visionSelectedId = 10;      // key 1
    hoststub::anglePulseUs = 1620;
    armWired();
    send("M"); step();                    // semi auto: arrows are fine-adjust
    send("ROT,1"); step();
    hoststub::visionTx = -80.0;           // let go with the hole 80 px left
    send("ROT,0"); step();
    runWithHeartbeat(550);                // an aim frame every 500 ms (5 slots)
    a = lastAim();
    // The elevation servo's pulse when the frame went out (Auto slews it
    // toward the table, so it may trail the current one by one aim period).
    assert(a.distance_cm == 352);
    assert(abs(static_cast<int>(a.elevation_pulse_us) - hoststub::anglePulseUs) <=
           static_cast<int>(TuningConstants::Auto::ELEVATION_SLEW_US_PER_SECOND * 0.51) + 1);
    assert(a.selected_id == 10 && a.target_id == 10 && a.valid);
    assert(a.aim_offset_px == 20);        // +80 px in 4 px units (2026-10-30)
    hoststub::visionTx = 0.0;
    // The status frames in between are untouched by it.
    assert(rec.statusFlags & RCMAP_STATUS_ARMED);
    assert(!rcmap_status_is_aim(rec.statusCode));
    assert(!rcmap_status_is_vision(rec.statusCode));
}

rcmap_vision lastVision() {
    rcmap_status st = {};
    st.flags = rec.visionFlags;
    st.event_seq = rec.visionSeq;
    st.event_code = rec.visionCode;
    rcmap_vision v = {};
    assert(rcmap_vision_from_status(&st, &v));
    return v;
}

void testVisionUplink() {
    reset();
    for (int i = 0; i < 5; i++) step(100);   // one vision frame per 5 slots
    // Nothing in view: zeros, not valid.
    rcmap_vision v = lastVision();
    assert(!v.valid && v.tx_px == 0.0f && v.ty_px == 0.0f);
    assert(v.flywheel_pulse_us == 1490);                   // the Falcon's pulse, any time
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 3524.0;   // -> 352 cm
    hoststub::visionTargetId = 10;
    hoststub::visionTx = -62.4;           // -> -62.5 (0.5 px steps)
    hoststub::visionTy = 20.5;
    for (int i = 0; i < 5; i++) step(100);   // one vision frame per 5 slots
    v = lastVision();
    assert(v.valid && v.target_id == 10);
    assert(v.tx_px == -62.5f && v.ty_px == 20.5f);
    // Lost target: tx / ty back to 0 (not a stale value).
    hoststub::visionValid = false;
    for (int i = 0; i < 5; i++) step(100);   // one vision frame per 5 slots
    v = lastVision();
    assert(!v.valid && v.tx_px == 0.0f);
    hoststub::visionTx = 0.0;
    hoststub::anglePulseUs = 1500;
    hoststub::visionTy = 0.0;
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
    targetInView();
    armWired();
    send("M"); step();                                    // semi-auto
    // Auto has the turret, flywheel and (measured table) elevation; the
    // slider is ignored. The flywheel is that hole's table value at 4 m.
    ShotTableLookup::Result shot = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(hoststub::visionTargetId, 4.0, shot));
    assert(rec.rotateMode == "vision");
    assert(nearly(rec.flywheelCommand, shot.flywheelCommand));
    send("FLY,0.2"); step();
    assert(nearly(rec.flywheelCommand, shot.flywheelCommand));
    // LT / RT offset (FLYO, percent) is added to the table value; bad
    // values are refused; a mode change drops it.
    send("FLYO,5"); step();
    assert(ManualControl::getFlywheelOffsetPct() == 5);
    assert(nearly(rec.flywheelCommand, constrain(shot.flywheelCommand + 0.05, 0.0, 1.0)));
    send("FLYO,21"); send("FLYO,x"); step();
    assert(saw("BAD_COMMAND") && ManualControl::getFlywheelOffsetPct() == 5);
    send("FLYO,0"); step();
    assert(nearly(rec.flywheelCommand, shot.flywheelCommand));
    // Target hold: a short dropout keeps the shot; a long one falls back
    // to the pre-spin.
    hoststub::visionValid = false;
    runWithHeartbeat(TuningConstants::Auto::TARGET_HOLD_MS - 100);
    assert(nearly(rec.flywheelCommand, shot.flywheelCommand));
    runWithHeartbeat(200);
    assert(nearly(rec.flywheelCommand, TuningConstants::Flywheel::SHOOT_COMMAND));
    hoststub::visionValid = true;
    step();
    assert(nearly(rec.flywheelCommand, shot.flywheelCommand));
    // The operator still drives the chassis.
    send("T,1,1"); step();
    assert(nearly(rec.chassisLeft, 1.0));

    // Not locked yet: F is refused, not queued.
    Serial.output.clear();
    send("F"); step();
    assert(saw("FIRE_NOT_READY") && !triggerHigh());
    runArmed(200);
    assert(Dribbler::getTriggerCount() == 0);              // no late shot

    // Locked and spun up, but the elevation still slewing to the table's
    // pulse (400 us/s): refused (2026-10-31).
    hoststub::shooterReady = true;
    step();
    if (fabs(hoststub::anglePulseUs - shot.elevationPulseUs) >
        TuningConstants::Auto::ELEVATION_READY_TOLERANCE_US) {
        Serial.output.clear();
        send("F"); step();
        assert(saw("FIRE_NOT_READY") && !triggerHigh());
    }
    runArmed(6000);
    assert(fabs(hoststub::anglePulseUs - shot.elevationPulseUs) <=
           TuningConstants::Auto::ELEVATION_READY_TOLERANCE_US);
    // Locked, spun up, elevation there: F fires exactly one ball.
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
    targetInView();
    armWired();
    send("M"); send("M"); step();                         // full auto
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_AUTO);
    assert(shotsDuring(500) == 0);                         // not ready: holds fire
    // Ready, but the elevation still slewing to the table's pulse
    // (400 us/s): holds fire until it gets there (2026-10-31).
    hoststub::shooterReady = true;
    ShotTableLookup::Result shot = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(hoststub::visionTargetId, 4.0, shot));
    while (fabs(hoststub::anglePulseUs - shot.elevationPulseUs) >
           TuningConstants::Auto::ELEVATION_READY_TOLERANCE_US + 50.0) {
        assert(shotsDuring(100) == 0);
    }
    hoststub::shooterReady = false;
    runArmed(1000);                                       // there now
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
    ShotTableLookup::Result expected = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(hoststub::visionTargetId, 5.0, expected));
    assert(nearly(rec.flywheelCommand, expected.flywheelCommand));
}

// Standing still, the depth wanders a few cm frame to frame (2026-10-27):
// the shot table is looked up at the steadied distance, so the flywheel
// command and the elevation target stay put and the shooter can get ready.
void testSteadyShotDistance() {
    reset();
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 4000.0;
    armWired();
    send("M"); step();                                   // semi-auto
    const double fly = rec.flywheelCommand;
    ShotTableLookup::Result expected = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(hoststub::visionTargetId, 4.0, expected));
    assert(nearly(fly, expected.flywheelCommand));
    const double jitter[] = {4040, 3960, 4030, 3950, 4045, 3970, 4020, 3940};
    for (int rep = 0; rep < 10; rep++) {
        for (double mm : jitter) {
            hoststub::visionDistanceMm = mm;
            send("K");
            step(33);
            assert(nearly(rec.flywheelCommand, fly));     // unchanged, every frame
        }
    }
    assert(nearly(Auto::getShotDistanceM(), 4.0));
    // A real move (4.0 -> 4.5 m) is followed after DISTANCE_LATCH_SETTLE_MS.
    hoststub::visionDistanceMm = 4500.0;
    for (int i = 0; i < 20; i++) { send("K"); step(33); }
    assert(nearly(Auto::getShotDistanceM(), 4.5));
    // Ready bits: the shot is there (bit 2), the stub shooter not ready.
    assert((Auto::getReadyBits() & 4u) && !(Auto::getReadyBits() & 1u));
    send("X"); step();
    assert(Auto::getReadyBits() == 0 && Auto::getShotDistanceM() == 0.0);
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

    // The shot table's elevation is measured (2026-10-18): in auto with a
    // target in view, Auto sets the elevation pulse from the table, and an
    // absolute ANGT is refused.
    static_assert(TuningConstants::ShotTable::ELEVATION_MEASURED,
                  "with placeholder pulses, Auto must not move the elevation: restore the old check");
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 4000.0;
    send("M"); step();
    ShotTableLookup::Result expected = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(hoststub::visionTargetId, 4.0, expected));
    // Slewed, not a jump: 200 ms in it has moved, but well short of the
    // target (400 us/s; the first step may count up to 100 ms).
    const double startUs = 1500.0;
    runArmed(200);
    assert(rec.anglePulseSets > 0 && fabs(rec.anglePulseUs - startUs) > 0.0);
    assert(fabs(rec.anglePulseUs - startUs) <=
           TuningConstants::Auto::ELEVATION_SLEW_US_PER_SECOND * 0.31 + 1.0);
    assert(fabs(rec.anglePulseUs - startUs) < fabs(expected.elevationPulseUs - startUs));
    runArmed(3000);                                       // ...then arrives
    assert(nearly(rec.anglePulseUs, expected.elevationPulseUs));
    Serial.output.clear();
    send("ANGT,100"); step();
    assert(saw("REJECTED_AUTO_MODE"));
}

void testWirelessMode() {
    reset();
    hoststub::visionReady = true;
    targetInView();
    armWireless();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep();
    assert(saw("MODE_SEMI_AUTO") && ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    // Fire (CH5 edge) in semi-auto goes through Auto: not ready -> refused.
    Serial.output.clear();
    wireless.fire = true;
    rcStep();
    assert(saw("FIRE_NOT_READY") && Dribbler::getTriggerCount() == 0);
    hoststub::shooterReady = true;
    wireless.fire = false;
    runWireless(6000);                                    // the elevation slews to the table first
    wireless.fire = true;
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

// The Dashboard's Falcon slider over the radio (CH4 with the mode).
void testWirelessFlywheel() {
    reset();
    armWireless();
    wireless.flywheel = 0.42f;
    rcStep();
    assert(fabs(rec.flywheelCommand - 0.42) < 1e-4);
    wireless.flywheel = 1.0f;
    rcStep();
    assert(fabs(rec.flywheelCommand - 1.0) < 1e-4);
    wireless.flywheel = 0.0f;
    rcStep();
    assert(nearly(rec.flywheelCommand, 0.0));
    // Semi auto: the slider value never reaches the robot (Auto owns the
    // flywheel there).
    wireless.flywheel = 0.7f;
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep();
    assert(ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    assert(nearly(ManualControl::getFlywheelCommand(), 0.0));
    // Link loss: disarmed, flywheel off.
    wireless.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    rcStep();
    assert(fabs(rec.flywheelCommand - 0.7) < 1e-4);
    MechLink::signalLost = true;
    step();
    assert(!ManualControl::isArmed() && nearly(rec.flywheelCommand, 0.0));
    // An Enable whose CH4 carries a flywheel speed is refused.
    Serial.output.clear();
    wireless.armed = false;
    rcStep();
    wireless.armed = true;
    rcStep();
    assert(saw("ENABLE_REJECTED_NOT_MANUAL") && !ManualControl::isArmed());
    assert(nearly(rec.flywheelCommand, 0.0));
}

// ------------------------------------------------ test mode (2026-10-26)

// MODE,3: the turret follows the camera; elevation, flywheel and F stay the
// operator's, exactly as in full manual.
void testTestModeWired() {
    using namespace TuningConstants::Auto;
    reset();
    hoststub::visionReady = true;
    targetInView();
    armWired();
    send("MODE,3"); step();
    assert(saw("MODE_TEST") && ManualControl::getOperatorMode() == OperatorMode::TEST);
    assert(rec.rotateMode == "vision");
    // Flywheel: the slider, not the shot table (no pre-spin either).
    assert(nearly(rec.flywheelCommand, 0.0));
    send("FLY,0.46"); step();
    assert(nearly(rec.flywheelCommand, 0.46) && rec.rotateMode == "vision");
    // Elevation: the hold-to-move jog and ANGT, never the table.
    const unsigned long tablePulses = rec.anglePulseSets;
    send("ANG,1"); step();
    assert(nearly(rec.angleCommand, 1.0) && rec.anglePulseSets == tablePulses);
    assert(nearly(ManualControl::getElevationOffsetUs(), 0.0));
    send("ANG,0"); step();
    send("ANGT,120"); step();
    assert(!saw("REJECTED_AUTO_MODE"));
    // Left / right (2026-10-30): held = turn by hand, then the camera
    // holds the new place as the aim offset.
    send("ROT,1"); step();
    assert(rec.rotateMode == "manual" && fabs(rec.rotateCommand) > 0.5);
    hoststub::visionTx = -60.0;
    send("ROT,0"); step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, 0.0));
    runWithHeartbeat(TuningConstants::Auto::AIM_JOG_SETTLE_MS);
    assert(nearly(ManualControl::getAimOffsetPx(), 60.0));
    assert(nearly(rec.aimOffsetPx, 60.0) && rec.rotateMode == "vision");
    hoststub::visionTx = 0.0;
    // F fires the tower right away, locked or not (no FIRE_NOT_READY).
    hoststub::shooterReady = false;
    Serial.output.clear();
    send("F"); step();
    assert(saw("FEED_TRIGGER_SENT") && !saw("FIRE_NOT_READY") && Dribbler::getTriggerCount() == 1);
    // FLYO is an autos-only offset: ignored here.
    send("FLYO,10"); step();
    assert(ManualControl::getFlywheelOffsetPct() == 0 && nearly(rec.flywheelCommand, 0.46));
    // Target holes still select (the camera locks the chosen one).
    send("TGT,5"); step();
    assert(saw("TARGET_SELECTED") && ManualControl::getTarget() == 5);
    // No camera: the turret holds; the operator keeps the flywheel.
    hoststub::visionReady = false;
    send("K"); step();
    assert(rec.rotateMode == "disabled" && nearly(rec.flywheelCommand, 0.46));
    // Status: reported as full manual + the TEST flag (vision frames own
    // the code's mode + 1 = 4 pattern).
    runWithHeartbeat(500);
    assert(rcmap_status_mode(rec.statusCode, rec.statusFlags) == RCMAP_MODE_TEST);
    assert(!rcmap_status_is_vision(rec.statusCode) && !rcmap_status_is_aim(rec.statusCode));
    assert(!(rec.statusFlags & RCMAP_STATUS_ROTATE_MANUAL));
    // Leaving test zeroes the flywheel like every mode change.
    send("MODE,2"); step();
    assert(saw("MODE_FULL_MANUAL") && nearly(rec.flywheelCommand, 0.0) && rec.rotateMode == "manual");
    // Disarm ends it; a new Enable starts in full manual.
    send("MODE,3"); step();
    send("X"); step();
    assert(!ManualControl::isArmed() && ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
}

// A ground on another channel table (CH10): every frame rejected, no
// Enable, and LINK_VERSION_MISMATCH at most once a second.
void testWirelessLinkVersion() {
    reset();
    for (int i = 0; i < 60; i++) {           // 1.2 s of frames, arm low then high
        wireless.armed = i >= 5;
        rcFrame();
        MechLink::channels[RCMAP_CH_VERSION] = 1500;   // an older ground
        step(20);
    }
    assert(!ManualControl::isArmed());
    size_t count = 0;
    for (size_t at = 0; (at = Serial.output.find("$EVENT,LINK_VERSION_MISMATCH\n", at)) != std::string::npos; at++) count++;
    assert(count >= 1 && count <= 2);
    // The matching table enables as usual.
    Serial.output.clear();
    armWireless();
}

// Over the radio: CH4 full manual (+ Falcon) and CH9 high.
void testTestModeWireless() {
    reset();
    hoststub::visionReady = true;
    targetInView();
    armWireless();
    wireless.test_mode = true;
    rcStep();
    assert(saw("MODE_TEST") && ManualControl::getOperatorMode() == OperatorMode::TEST);
    assert(rec.rotateMode == "vision");
    wireless.flywheel = 0.55f;
    rcStep();
    assert(fabs(rec.flywheelCommand - 0.55) < 1e-4);
    wireless.nudge = RCMAP_NUDGE_UP;
    rcStep();
    assert(nearly(rec.angleCommand, 1.0));
    wireless.nudge = RCMAP_NUDGE_CENTER;
    rcStep();
    Serial.output.clear();
    wireless.fire = true;
    rcStep();
    assert(saw("FEED_TRIGGER_SENT") && Dribbler::getTriggerCount() == 1);
    wireless.fire = false;
    rcStep();
    // CH4 to semi auto: semi auto at once, whatever CH9 still says.
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep();
    assert(ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    // Back, then CH9 low = full manual.
    wireless.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    rcStep();
    assert(ManualControl::getOperatorMode() == OperatorMode::TEST);
    wireless.test_mode = false;
    rcStep();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    // An Enable with CH9 high is refused, like any non-manual CH4.
    wireless.test_mode = true;
    wireless.armed = false;
    rcStep();
    assert(!ManualControl::isArmed());
    Serial.output.clear();
    wireless.armed = true;
    rcStep();
    assert(saw("ENABLE_REJECTED_NOT_MANUAL") && !ManualControl::isArmed());
}

// ------------------------------------------- vision v2: target, fine-adjust

void testAutoNeedsDistance() {
    // A valid frame with no depth distance (the Orin's webcam bench mode)
    // can be aimed at but never fired at: nothing to look up.
    reset();
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 0.0;
    hoststub::shooterReady = true;
    armWired();
    send("M"); send("M"); step();                         // full auto
    assert(shotsDuring(2500) == 0);
    assert(nearly(rec.flywheelCommand, TuningConstants::Flywheel::SHOOT_COMMAND));
    hoststub::visionDistanceMm = 4000.0;
    assert(shotsDuring(5000) >= 2);                       // after the elevation slew (2026-10-31)
    // Semi-auto F without a distance: refused.
    hoststub::visionDistanceMm = 0.0;
    send("M"); send("M"); step();                         // full auto -> manual -> semi
    assert(ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    Serial.output.clear();
    runArmed(1500);
    send("F"); step();
    assert(saw("FIRE_NOT_READY"));
}

void testShotTableByHeight() {
    using namespace TuningConstants::ShotTable;
    // Orin vision id -> its Dashboard hole's generated table, or that row's
    // center hole (keys 4 / 5 / 6; the bonus row's W = 11, 2026-10-31) when
    // the hole has no points; nothing for 0 or unknown ids.
    for (uint8_t hole = 1; hole <= 12; hole++) {
        const int id = OperatorTarget::visionIdForHole(hole);
        const uint8_t center = hole >= 10 ? 11 : static_cast<uint8_t>(4 + (hole - 1) % 3);
        const ShotTableData::Point* expected =
            ShotTableData::HOLE_COUNTS[hole] > 0 ? ShotTableData::HOLE_POINTS[hole]
                                                 : ShotTableData::HOLE_POINTS[center];
        assert(tableForVisionId(id).entries == expected);
    }
    for (int id : {0, 13, -1, 99}) assert(tableForVisionId(id).entries == nullptr);

    // Auto uses the frame's hole: the flywheel follows that hole's table.
    reset();
    hoststub::visionReady = true;
    hoststub::visionValid = true;
    hoststub::visionDistanceMm = 5000.0;
    hoststub::visionTargetId = 5;                         // top row, key 6
    armWired();
    send("TGT,6"); send("M"); step();
    ShotTableLookup::Result expected = {0.0, 0.0};
    assert(ShotTableLookup::lookupTarget(5, 5.0, expected));
    assert(nearly(rec.flywheelCommand, expected.flywheelCommand));
    // A frame with no table (id 0) is never fired at, even "ready".
    hoststub::visionTargetId = 0;
    hoststub::shooterReady = true;
    send("M"); step();                                    // full auto
    assert(shotsDuring(2500) == 0);
    assert(nearly(rec.flywheelCommand, TuningConstants::Flywheel::SHOOT_COMMAND));
    hoststub::visionTargetId = 13;                        // an unknown id: no table
    assert(shotsDuring(2500) == 0);
    hoststub::visionTargetId = 11;                        // bottom row: fired at
    assert(shotsDuring(4000) >= 2);                       // once the elevation is there (2026-10-31)
}

void testFineAdjustWired() {
    using namespace TuningConstants::Auto;
    reset();
    hoststub::visionReady = true;
    armWired();
    // Full manual: the arrows are the jog, no offset.
    send("ROT,1"); step();
    assert(nearly(ManualControl::getAimOffsetPx(), 0.0) && rec.rotateMode == "manual");
    send("ROT,0"); send("M"); step();                     // semi-auto
    hoststub::visionValid = true;
    hoststub::visionTx = 0.0;
    // 2026-10-30: held = the turret turns by hand (same jog as full
    // manual), the camera lets go; heartbeat resends keep it going.
    const double jog = (TuningConstants::Rotate::MANUAL_JOG_INVERTED ? -1.0 : 1.0) *
                       ManualControlConstants::ROTATE_MANUAL_SCALE;
    send("ROT,1"); step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, jog));
    send("ROT,1"); step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, jog));
    assert(nearly(ManualControl::getAimOffsetPx(), 0.0));
    // Released: stands still AIM_JOG_SETTLE_MS, then the hole's place in
    // the image (tx = -120: 120 px left of center) becomes offset +120.
    hoststub::visionTx = -120.0;
    send("ROT,0"); step();
    assert(ManualControl::isAimJogActive() && rec.rotateMode == "manual" && nearly(rec.rotateCommand, 0.0));
    runWithHeartbeat(AIM_JOG_SETTLE_MS - 20);
    assert(ManualControl::isAimJogActive() && nearly(ManualControl::getAimOffsetPx(), 0.0));
    runWithHeartbeat(20);
    assert(!ManualControl::isAimJogActive());
    assert(nearly(ManualControl::getAimOffsetPx(), 120.0));
    assert(nearly(rec.aimOffsetPx, 120.0) && rec.rotateMode == "vision"); // Auto hands it to the turret
    // The other way, straight from one side to the other, and clamped.
    send("ROT,1"); step();
    send("ROT,-1"); step();
    assert(rec.rotateMode == "manual" && nearly(rec.rotateCommand, -jog));
    hoststub::visionTx = 500.0;
    send("ROT,0"); step(); runWithHeartbeat(AIM_JOG_SETTLE_MS);
    assert(nearly(ManualControl::getAimOffsetPx(), -AIM_OFFSET_MAX_PX));
    // No hole in view at the end of the wait: the old offset stays.
    hoststub::visionValid = false;
    send("ROT,1"); step(); send("ROT,0"); step(); runWithHeartbeat(AIM_JOG_SETTLE_MS);
    assert(nearly(ManualControl::getAimOffsetPx(), -AIM_OFFSET_MAX_PX));
    hoststub::visionValid = true;
    hoststub::visionTx = -40.0;
    send("ROT,1"); step(); send("ROT,0"); step(); runWithHeartbeat(AIM_JOG_SETTLE_MS);
    assert(nearly(ManualControl::getAimOffsetPx(), 40.0));
    // A new hole clears it (and stops a hand turn), and a key still held
    // then needs a release.
    send("ROT,1"); step();
    send("TGT,4"); step();
    assert(nearly(ManualControl::getAimOffsetPx(), 0.0) && !ManualControl::isAimJogActive());
    assert(rec.rotateMode == "vision");
    send("ROT,1"); step(); send("ROT,1"); step();
    assert(rec.rotateMode == "vision");
    send("ROT,0"); step(); send("ROT,1"); step();
    assert(rec.rotateMode == "manual");
    send("ROT,0"); step(); runWithHeartbeat(AIM_JOG_SETTLE_MS);
    assert(nearly(ManualControl::getAimOffsetPx(), 40.0));
    // The same hole again is not a change: kept.
    send("TGT,4"); step();
    assert(nearly(ManualControl::getAimOffsetPx(), 40.0));
    hoststub::visionTx = 0.0;
    // Up/down: Auto owns the elevation now (measured table), so each press
    // adds one elevation-offset step instead of jogging.
    static_assert(TuningConstants::ShotTable::ELEVATION_MEASURED,
                  "with placeholder pulses, up/down stay the manual jog: restore the old check");
    const double up = TuningConstants::Angle::INVERTED ? -1.0 : 1.0;
    send("ANG,1"); step();
    assert(nearly(ManualControl::getElevationOffsetUs(), up * ELEVATION_OFFSET_STEP_US));
    send("ANG,0"); step();
    // Disarm clears; so does a new Enable.
    send("X"); step();
    assert(nearly(ManualControl::getAimOffsetPx(), 0.0));
}

void testFineAdjustWireless() {
    using namespace TuningConstants::Auto;
    reset();
    hoststub::visionReady = true;
    armWireless();
    wireless.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcStep();
    hoststub::visionValid = true;
    wireless.nudge = RCMAP_NUDGE_RIGHT;
    runWireless(200);                                     // held: every frame repeats it
    assert(rec.rotateMode == "manual" && fabs(rec.rotateCommand) > 0.5);
    hoststub::visionTx = 90.0;
    wireless.nudge = RCMAP_NUDGE_CENTER;
    runWireless(AIM_JOG_SETTLE_MS + 20);
    assert(nearly(ManualControl::getAimOffsetPx(), -90.0) && rec.rotateMode == "vision");
    wireless.nudge = RCMAP_NUDGE_LEFT; rcStep();
    assert(rec.rotateMode == "manual");
    hoststub::visionTx = 30.0;
    wireless.nudge = RCMAP_NUDGE_CENTER;
    runWireless(AIM_JOG_SETTLE_MS + 20);
    assert(nearly(ManualControl::getAimOffsetPx(), -30.0));
    hoststub::visionTx = 0.0;
    // Switching hole on CH3 clears it.
    wireless.nudge = RCMAP_NUDGE_CENTER;
    wireless.target = 2;
    rcStep();
    assert(nearly(ManualControl::getAimOffsetPx(), 0.0) && ManualControl::getTarget() == 2);
}

// Turret home point and return (2026-10-31): TZERO only while disarmed,
// THOME only for the wired owner; the guard's state rides the aim frame.
void testTurretHomeCommands() {
    reset();
    send("TZERO"); step();
    assert(saw("TURRET_ZERO_SET"));
    Serial.output.clear();
    hoststub::turretZeroOk = false;                       // no healthy AS5600 reading
    send("TZERO"); step();
    assert(saw("TURRET_ZERO_REJECTED") && !saw("TURRET_ZERO_SET"));
    hoststub::turretZeroOk = true;
    send("THOME"); step();
    assert(saw("REJECTED_DISARMED") && rec.turretReturns == 0);
    armWired();
    send("TZERO"); step();                                // armed: refused
    assert(saw("TURRET_ZERO_REJECTED"));
    send("THOME"); step();
    assert(rec.turretReturns == 1);
    hoststub::turretState = 5;                            // AT_RIGHT
    runWithHeartbeat(550);
    assert(lastAim().turret_state == 5);

    // RB over the radio (CH9 home level, 2026-10-31): one return per rising
    // edge, only for the wireless owner.
    reset();
    armWireless();
    wireless.turret_home = true; rcStep(); rcStep(); rcStep();
    assert(rec.turretReturns == 1);
    wireless.turret_home = false; rcStep();
    wireless.turret_home = true; rcStep();
    assert(rec.turretReturns == 2);
    wireless.turret_home = false; rcStep();
    send("X"); step();                                    // disarmed: refused
    Serial.output.clear();
    wireless.armed = false; rcStep();
    wireless.turret_home = true; rcStep();
    assert(rec.turretReturns == 2 && saw("REJECTED_DISARMED"));

    // RB in the autos / test (2026-10-31): back to 0 deg and held there --
    // no camera follow -- until a hole key (even the same one), a mode
    // change or the hand turn. Full manual ignores the hold.
    reset();
    hoststub::visionReady = true;
    armWired();
    send("M"); step();                                    // semi-auto
    assert(rec.rotateMode == "vision");
    send("THOME"); step();
    assert(ManualControl::isTurretHeldHome() && rec.rotateMode == "disabled");
    runWithHeartbeat(500);
    assert(rec.rotateMode == "disabled");                 // still held
    send("TGT,0"); step();                                // same hole again: follow
    assert(!ManualControl::isTurretHeldHome() && rec.rotateMode == "vision");
    send("THOME"); step();
    send("MODE,3"); step();                               // mode change: follow
    assert(!ManualControl::isTurretHeldHome() && rec.rotateMode == "vision");
    send("THOME"); step();
    assert(rec.rotateMode == "disabled");
    send("ROT,1"); step();                                // hand turn takes it back
    assert(!ManualControl::isTurretHeldHome() && rec.rotateMode == "manual");
    send("ROT,0"); step();
    runWithHeartbeat(TuningConstants::Auto::AIM_JOG_SETTLE_MS);
    assert(rec.rotateMode == "vision");
    send("THOME"); step();
    send("X"); step();                                    // disarm clears it
    assert(!ManualControl::isTurretHeldHome());
}

// Dashboard menus (2026-10-16): MODE,<n> sets the mode directly, and every
// Enable starts on the Orin's own pick (target 0).
void testModeCommandAndTargetReset() {
    reset();
    send("MODE,1"); step();
    assert(saw("REJECTED_DISARMED"));
    armWired();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    send("MODE,0"); step();
    assert(saw("MODE_FULL_AUTO") && ManualControl::getOperatorMode() == OperatorMode::FULL_AUTO);
    send("MODE,1"); step();
    assert(saw("MODE_SEMI_AUTO") && ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    Serial.output.clear();
    for (const char* bad : {"MODE,4", "MODE,-1", "MODE,", "MODE,2x"}) { send(bad); step(); }
    assert(saw("BAD_COMMAND") && ManualControl::getOperatorMode() == OperatorMode::SEMI_AUTO);
    send("M"); step();                                    // M still cycles from there
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_AUTO);
    send("MODE,2"); step();
    assert(ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);

    // Target: kept while enabled, back to 0 (the AI's pick) at every Enable.
    send("TGT,5"); step();
    assert(ManualControl::getTarget() == 5);
    send("X"); step();
    armWired();
    assert(ManualControl::getTarget() == RCMAP_TARGET_NONE);
    send("TGT,0"); step();                                // key 0: back to the AI's pick
    assert(ManualControl::getTarget() == RCMAP_TARGET_NONE);
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
    armWired();
    assert(rec.rotateMode == "manual");
    // Target is recorded; auto paths don't exist on the wired benches.
    send("TGT,3"); step();
    assert(saw("TARGET_SELECTED") && ManualControl::getTarget() == 3);
    send("AUTO,1"); step();
    assert(saw("AUTO_PATH_REJECTED") && ManualControl::getAutoPathsUsedMask() == 0);
    send("MODE,0"); step();                              // no auto modes on the benches
    assert(saw("MODE_FULL_MANUAL") && ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    send("T,1,-1"); send("ROT,1"); send("ANG,-1"); send("FLY,0.6"); step();
    assert(nearly(rec.chassisLeft, 1.0) && nearly(rec.chassisRight, -1.0));
    assert(nearly(rec.rotateCommand, JOG * 1.0) && nearly(rec.angleCommand, -1.0));
    assert(nearly(rec.flywheelCommand, 0.6));
    // No auto modes on the wired benches: M stays in full manual.
    send("M"); step();
    assert(saw("MODE_FULL_MANUAL") &&
           ManualControl::getOperatorMode() == OperatorMode::FULL_MANUAL);
    send("X"); step(); assert(!ManualControl::isArmed());
    printf("wired manual profile: all asserts passed\n");
#elif !defined(TEL_HOST_TEST_PIN_TABLE_CONFIRMED) || TEL_HOST_TEST_PIN_TABLE_CONFIRMED
    testWiredRefusedWhileDisarmed();
    testRetiredKeys();
    testEnableDisableDrivesEverySubsystem();
    testWiredWatchdog();
    testWiredScalingAndParsing();
    testWiredOversizedLine();
    testTargetSelection();
    testAutoPathOncePerEnable();
    testFeedPulseAndLockout();
    testDisarmCutsPulse();
    testWirelessEnableNeedsEdge();
    testWirelessCommands();
    testWirelessFireEdgeNoRepeatNoReplay();
    testWirelessPressWhileDisarmedNotLatched();
    testWirelessTargetAndAutoPath();
    testWirelessInvalidFramesTimeOut();
    testAuthority();
    testStatusUplink();
    testModeKeyWired();
    testSemiAutoFiresOnlyWhenReady();
    testFullAutoFiresByItself();
    testAutoWithoutCamera();
    testShotTableDrivesFlywheel();
    testSteadyShotDistance();
    testWirelessMode();
    testWirelessFlywheel();
    testElevationTargetCommands();
    testAutoNeedsDistance();
    testShotTableByHeight();
    testFineAdjustWired();
    testFineAdjustWireless();
    testTurretHomeCommands();
    testTestModeWired();
    testTestModeWireless();
    testWirelessLinkVersion();
    testModeCommandAndTargetReset();
    testAimUplink();
    testVisionUplink();
    printf("manual_control_test (pin table confirmed): all asserts passed\n");
#else
    testEnableRefusedUntilPinTableConfirmed();
    printf("manual_control_test (pin table unconfirmed gate): all asserts passed\n");
#endif
    return 0;
}
