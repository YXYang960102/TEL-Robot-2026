// Host test for GroundCommandState / RobotStatusTracker: PC line protocol
// -> RC channel state (table v2: arcade drive, nudge, fire / auto pulses,
// target hole, mode), the ground-side PC watchdog, input hardening
// (oversized lines, NaN), and robot status -> PC lines.
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "../src/GroundCommandState.h"

static std::vector<std::string> lines;
static void collect(void*, const char* line) { lines.push_back(line); }

// The latest "$WIRELESS" line (each report is "$WIRELESS" then "$AIM").
static std::string lastWireless() {
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        if (it->rfind("$WIRELESS,", 0) == 0) return *it;
    }
    return "";
}

static bool saw(const std::string& s) {
    for (const auto& l : lines) if (l == s) return true;
    return false;
}

static void send(GroundCommandState& st, const std::string& text, uint32_t now) {
    for (char c : text) st.rxByte(c, now, collect, nullptr);
    st.rxByte('\n', now, collect, nullptr);
}

static rcmap_command onAir(const GroundCommandState& st) {
    // What the robot decodes from this state.
    int ch[16];
    rcmap_encode(&st.command(), ch);
    rcmap_command d{};
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    return d;
}

static bool nearf(float a, float b) { return a - b < 0.01f && b - a < 0.01f; }

static void testDefaultsAndEnable() {
    GroundCommandState st;
    st.reset(0);
    rcmap_command d = onAir(st);
    assert(!d.armed && !d.fire && nearf(d.forward, 0) && nearf(d.turn, 0));
    assert(d.nudge == RCMAP_NUDGE_CENTER && d.auto_path == RCMAP_AUTO_NONE && d.target == 0);
    send(st, "T,1,1", 10);
    assert(saw("$GROUND_EVENT,REJECTED_NOT_ENABLED"));
    assert(nearf(onAir(st).forward, 0));

    send(st, "E", 20);
    assert(st.command().armed && saw("$GROUND_EVENT,ENABLE_REQUESTED"));
    // Tank from the Dashboard -> arcade on the radio.
    send(st, "T,1,1", 30);
    d = onAir(st);
    assert(nearf(d.forward, 1) && nearf(d.turn, 0));
    send(st, "T,-1,1", 31);           // D / stick right
    d = onAir(st);
    assert(nearf(d.forward, 0) && nearf(d.turn, 1));
    send(st, "T,0.3,0.9", 32);
    d = onAir(st);
    assert(nearf(d.forward, 0.6f) && nearf(d.turn, 0.3f));

    send(st, "X", 40);
    d = onAir(st);
    assert(!d.armed && nearf(d.forward, 0) && nearf(d.turn, 0));
    assert(saw("$GROUND_EVENT,ENABLE_RELEASED"));
}

static void testNudge() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    send(st, "ANG,1", 1);
    assert(onAir(st).nudge == RCMAP_NUDGE_UP);
    send(st, "ROT,-1", 2);           // both held: elevation wins
    assert(onAir(st).nudge == RCMAP_NUDGE_UP);
    send(st, "ANG,0", 3);
    assert(onAir(st).nudge == RCMAP_NUDGE_LEFT);
    send(st, "ROT,1", 4);
    assert(onAir(st).nudge == RCMAP_NUDGE_RIGHT);
    send(st, "ROT,0", 5);
    assert(onAir(st).nudge == RCMAP_NUDGE_CENTER);
    send(st, "ANG,-1", 6);
    assert(onAir(st).nudge == RCMAP_NUDGE_DOWN);
    send(st, "X", 7);                // release centers it
    assert(onAir(st).nudge == RCMAP_NUDGE_CENTER);
}

static void testFirePulse() {
    GroundCommandState st;
    st.reset(0);
    send(st, "F", 0);                // not enabled: nothing
    assert(!onAir(st).fire);
    send(st, "E", 10);
    send(st, "F", 20);
    assert(onAir(st).fire);
    st.tick(119, collect, nullptr);  // held for the 100 ms pulse
    assert(onAir(st).fire);
    send(st, "F", 110);              // a press during the pulse doesn't extend it
    st.tick(120, collect, nullptr);
    assert(!onAir(st).fire);
    send(st, "F", 130);              // next press: a new rising edge
    assert(onAir(st).fire);
    send(st, "X", 140);
    assert(!onAir(st).fire);         // release ends it at once
}

static void testTargetAndAutoPath() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    send(st, "TGT,5", 1);
    assert(onAir(st).target == 5);
    send(st, "TGT,9", 2);
    assert(onAir(st).target == 9);
    send(st, "TGT,0", 3);
    assert(onAir(st).target == 0);
    send(st, "TGT,7", 4);
    send(st, "X", 5);                // release keeps the selection...
    assert(onAir(st).target == 7);
    send(st, "E", 6);                // ...until the next Enable: back to the AI's pick
    assert(onAir(st).target == 0);
    send(st, "TGT,7", 7);

    send(st, "AUTO,2", 10);
    assert(onAir(st).auto_path == 2);
    send(st, "AUTO,3", 50);          // during the pulse: ignored
    assert(onAir(st).auto_path == 2);
    st.tick(10 + GroundCommandState::AUTO_PULSE_MS, collect, nullptr);
    assert(onAir(st).auto_path == RCMAP_AUTO_NONE);
    send(st, "AUTO,4", 300);
    assert(onAir(st).auto_path == 4);
    lines.clear();
    for (const char* bad : {"TGT,13", "TGT,-1", "TGT,a", "AUTO,0", "AUTO,5", "AUTO,"}) send(st, bad, 301);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && onAir(st).target == 7 && onAir(st).auto_path == 4);
    // The bonus holes (Q W E -> TGT,10..12, 2026-10-27) reach CH3.
    send(st, "TGT,12", 302);
    assert(onAir(st).target == 12);
    send(st, "TGT,10", 303);
    assert(onAir(st).target == 10);
}

static void testPcWatchdog() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    send(st, "T,1,1", 0);
    for (uint32_t t = 50; t <= 1000; t += 50) {
        send(st, "K", t);
        st.tick(t, collect, nullptr);
    }
    assert(st.command().armed);
    lines.clear();
    st.tick(1300, collect, nullptr); // 300 ms without a line: still fine
    assert(st.command().armed && nearf(onAir(st).forward, 1) && lines.empty());

    // Stage 1 (> PC_HOLD_MS): motion neutral, enable / mode / target kept.
    send(st, "TGT,5", 1000);
    send(st, "MODE,1", 1000);
    send(st, "MODE,2", 1000);
    send(st, "FLY,0.5", 1000);
    send(st, "ROT,1", 1000);
    send(st, "T,1,1", 1000);
    lines.clear();
    st.tick(1301, collect, nullptr);
    rcmap_command d = onAir(st);
    assert(st.command().armed && saw("$GROUND_EVENT,PC_HOLD"));
    assert(nearf(d.forward, 0) && nearf(d.turn, 0) && d.nudge == RCMAP_NUDGE_CENTER);
    assert(d.flywheel == 0.0f && d.target == 5 && d.mode_us == RCMAP_MODE_FULL_MANUAL_US);
    lines.clear();
    st.tick(1400, collect, nullptr); // printed once
    assert(lines.empty());
    // The next line resumes and reports the gap; the Dashboard's resends
    // bring the outputs back.
    send(st, "K", 1640);
    assert(saw("$GROUND_EVENT,PC_RESUMED,640"));
    send(st, "T,1,1", 1640);
    send(st, "FLY,0.5", 1640);
    st.tick(1640, collect, nullptr);
    assert(nearf(onAir(st).forward, 1) && nearf(onAir(st).flywheel, 0.5f));
    lines.clear();
    send(st, "K", 1700);              // no hold in between: nothing printed
    assert(lines.empty());

    // A rejected line doesn't count as the PC being alive.
    send(st, "Q", 1900);
    // Stage 2 (> PC_TIMEOUT_MS): released, like before.
    st.tick(1700 + GroundCommandState::PC_TIMEOUT_MS, collect, nullptr);
    assert(st.command().armed && saw("$GROUND_EVENT,PC_HOLD"));
    lines.clear();
    st.tick(1701 + GroundCommandState::PC_TIMEOUT_MS, collect, nullptr);
    assert(!st.command().armed && nearf(onAir(st).forward, 0));
    assert(saw("$GROUND_EVENT,PC_TIMEOUT") && !saw("$GROUND_EVENT,PC_HOLD"));
    // A fresh E after the timeout starts clean: no stale hold / resume.
    lines.clear();
    send(st, "E", 4000);
    send(st, "K", 4050);
    for (const auto& l : lines) assert(l.rfind("$GROUND_EVENT,PC_RESUMED", 0) != 0);
    assert(st.command().armed);
    // Not enabled: the watchdog stays quiet.
    send(st, "X", 4100);
    lines.clear();
    st.tick(9000, collect, nullptr);
    assert(lines.empty());
}

static void testInputHardening() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    send(st, "T,0.5,0.5", 0);
    lines.clear();
    for (const char* bad : {"T,nan,1", "T,inf,1", "T,1", "T,1,1,1", "T,a,1", "ROT,1e999", "Q", "R", "L"}) {
        send(st, bad, 1);
    }
    assert(nearf(onAir(st).forward, 0.5f));
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && saw("$GROUND_EVENT,UNKNOWN_COMMAND"));
    // ANGT has no radio channel: accepted (keeps the link alive), nothing
    // sent, no error.
    lines.clear();
    send(st, "ANGT,90", 2);
    assert(lines.empty());
    // Oversized line ending in a valid-looking command: dropped whole, and a
    // corrupt PC stream releases the enable.
    lines.clear();
    send(st, std::string(40, 'z') + "F", 2);
    assert(!st.command().armed && saw("$GROUND_EVENT,COMMAND_TOO_LONG"));
    assert(!onAir(st).fire);
    send(st, "E", 3); // next line parses normally
    assert(st.isEnabled());
    // (CH6 was high until 2 ms ago: it rises after ARM_LOW_MS, see testArmEdge.)
    st.tick(2 + GroundCommandState::ARM_LOW_MS, collect, nullptr);
    assert(st.command().armed);
}

static rcmap_status status(uint8_t flags, uint32_t seq, uint16_t code) {
    rcmap_status s{};
    s.flags = flags;
    s.event_seq = seq;
    s.event_code = code;
    return s;
}

static void testStatusTracker() {
    RobotStatusTracker tr;
    tr.reset(0);
    lines.clear();
    tr.tick(250, collect, nullptr);
    assert(lastWireless() == "$WIRELESS,0,0,0,0,0,0,0,0,-1"); // nothing heard yet
    // First status: baseline only, its old event is not replayed.
    tr.onStatus(status(RCMAP_STATUS_PINS_CONFIRMED | RCMAP_STATUS_RC_LINK_OK, 5,
                       RCMAP_EVENT_OUTPUTS_DISABLED), 300, collect, nullptr);
    assert(!saw("$EVENT,OUTPUTS_DISABLED"));
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS | RCMAP_STATUS_PINS_CONFIRMED |
                       RCMAP_STATUS_RC_LINK_OK | RCMAP_STATUS_ROTATE_MANUAL, 6,
                       RCMAP_EVENT_OUTPUTS_ENABLED), 400, collect, nullptr);
    assert(saw("$EVENT,OUTPUTS_ENABLED"));
    // Same seq repeated (every 100 ms): no duplicate event.
    lines.clear();
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 6, RCMAP_EVENT_OUTPUTS_ENABLED),
                450, collect, nullptr);
    assert(lines.empty());
    tr.tick(500, collect, nullptr);
    // A status code without the packed mode (older robot firmware) reports
    // mode -1, never "full auto".
    assert(lastWireless() == "$WIRELESS,1,1,2,0,0,0,0,50,-1");
    // Status goes stale: reported as not fresh with everything zeroed.
    tr.tick(1500, collect, nullptr);
    assert(tr.fresh(1500));                 // 1 s old: still fresh (STALE_MS 3 s)
    tr.tick(3500, collect, nullptr);
    assert(lastWireless().rfind("$WIRELESS,0,0,0,0,0,0,0,", 0) == 0);
    assert(!tr.fresh(3500));

    // Mode rides in the top bits of the code: the event name is still
    // decoded correctly and $WIRELESS reports the mode.
    lines.clear();
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 7,
                       rcmap_status_code(RCMAP_EVENT_MODE_SEMI_AUTO, RCMAP_MODE_SEMI_AUTO)),
                3600, collect, nullptr);
    assert(saw("$EVENT,MODE_SEMI_AUTO"));
    tr.tick(3760, collect, nullptr);        // reports every 250 ms (last at 3500)
    assert(lastWireless() == "$WIRELESS,1,1,2,0,0,0,0,160,1");
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 8,
                       rcmap_status_code(RCMAP_EVENT_FEED_TRIGGER_SENT, RCMAP_MODE_FULL_AUTO)),
                3800, collect, nullptr);
    assert(saw("$EVENT,FEED_TRIGGER_SENT"));
    tr.tick(4020, collect, nullptr);
    assert(lastWireless() == "$WIRELESS,1,1,2,0,0,0,0,220,0");
    // TEST (2026-10-26): full manual in the code + the TEST flag -> mode 3,
    // and the new event name is not dropped as corrupt.
    lines.clear();
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS | RCMAP_STATUS_TEST_MODE, 9,
                       rcmap_status_code(RCMAP_EVENT_MODE_TEST, RCMAP_MODE_TEST)),
                4100, collect, nullptr);
    assert(saw("$EVENT,MODE_TEST"));
    tr.tick(4300, collect, nullptr);
    assert(lastWireless() == "$WIRELESS,1,1,2,0,0,0,0,200,3");
}


static int ch4(const GroundCommandState& st) {
    int ch[16];
    rcmap_encode(&st.command(), ch);
    return ch[RCMAP_CH_MODE];
}

static void testAimFrames() {
    RobotStatusTracker tr;
    tr.reset(0);
    lines.clear();
    // A status frame first (baseline), then an aim frame: no $EVENT, the
    // status flags are untouched, and $AIM carries the aim fields.
    rcmap_status st{};
    st.flags = RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS;
    st.event_seq = 5;
    st.event_code = rcmap_status_code(RCMAP_EVENT_OUTPUTS_ENABLED, RCMAP_MODE_SEMI_AUTO);
    tr.onStatus(st, 10, collect, nullptr);
    rcmap_aim a{};
    a.distance_cm = 352;
    a.elevation_pulse_us = 1620;
    a.selected_id = 10;
    a.target_id = 10;
    a.valid = true;
    a.aim_offset_px = -15;
    rcmap_status af{};
    uint8_t flags;
    uint32_t seq;
    uint16_t code;
    rcmap_aim_to_status(&a, &flags, &seq, &code);
    af.flags = flags;
    af.event_seq = seq;
    af.event_code = code;
    tr.onStatus(af, 20, collect, nullptr);
    assert(lines.empty());                       // no $EVENT from an aim frame
    assert(tr.flags() == st.flags);
    tr.tick(300, collect, nullptr);
    assert(saw("$WIRELESS,1,1,2,0,0,0,0,290,1"));   // status as before (mode semi)
    assert(saw("$AIM,1,352,1620,10,10,1,-15"));
    // The next status frame's event still reports normally.
    st.event_seq = 6;
    st.event_code = rcmap_status_code(RCMAP_EVENT_FEED_TRIGGER_SENT, RCMAP_MODE_SEMI_AUTO);
    tr.onStatus(st, 310, collect, nullptr);
    assert(saw("$EVENT,FEED_TRIGGER_SENT"));
    // A status with an event code past the table is corrupt: dropped
    // whole (no $EVENT, flags untouched, freshness not renewed).
    lines.clear();
    rcmap_status bad{};
    bad.flags = 0;
    bad.event_seq = 99;
    bad.event_code = rcmap_status_code(3312 & 0x0FFF, RCMAP_MODE_FULL_MANUAL);
    tr.onStatus(bad, 320, collect, nullptr);
    assert(lines.empty() && tr.flags() == st.flags);
    // No aim frame for > STALE_MS: fresh 0 and zeros.
    lines.clear();
    tr.tick(20 + RobotStatusTracker::STALE_MS + 300, collect, nullptr);
    assert(saw("$AIM,0,0,0,0,0,0,0"));
    // Never any aim frame (older robot firmware): $AIM,0,...
    RobotStatusTracker none;
    none.reset(0);
    lines.clear();
    none.tick(300, collect, nullptr);
    assert(saw("$AIM,0,0,0,0,0,0,0"));
}

static void testModeKey() {
    GroundCommandState st;
    st.reset(0);
    lines.clear();
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "M", 1);                       // not enabled: refused, no change
    assert(saw("$GROUND_EVENT,REJECTED_NOT_ENABLED"));
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    send(st, "E", 2);
    send(st, "M", 4);
    assert(saw("$GROUND_EVENT,MODE_SEMI_AUTO"));
    assert(ch4(st) == RCMAP_MODE_SEMI_AUTO_US);
    send(st, "M", 5);
    assert(saw("$GROUND_EVENT,MODE_FULL_AUTO") && ch4(st) == RCMAP_MODE_FULL_AUTO_US);
    send(st, "M", 6);
    assert(saw("$GROUND_EVENT,MODE_FULL_MANUAL") && ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    // X, a PC timeout, and a fresh E all put CH4 back to manual, so the
    // robot (which refuses an Enable that isn't manual) always starts manual.
    send(st, "M", 7);
    send(st, "X", 8);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "E", 9);
    send(st, "M", 10);
    send(st, "M", 11);
    assert(ch4(st) == RCMAP_MODE_FULL_AUTO_US);
    st.tick(11 + GroundCommandState::PC_TIMEOUT_MS + 1, collect, nullptr);
    assert(saw("$GROUND_EVENT,PC_TIMEOUT") && ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "E", 400);
    send(st, "M", 401);
    send(st, "E", 402);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    // MODE,<n> (the Dashboard's mode menu): CH4 straight to that level.
    lines.clear();
    send(st, "MODE,0", 403);
    assert(saw("$GROUND_EVENT,MODE_FULL_AUTO") && ch4(st) == RCMAP_MODE_FULL_AUTO_US);
    send(st, "MODE,1", 404);
    assert(saw("$GROUND_EVENT,MODE_SEMI_AUTO") && ch4(st) == RCMAP_MODE_SEMI_AUTO_US);
    send(st, "MODE,2", 405);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "MODE,1", 406);
    lines.clear();
    for (const char* bad : {"MODE,4", "MODE,-1", "MODE,", "MODE,1x"}) send(st, bad, 407);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && ch4(st) == RCMAP_MODE_SEMI_AUTO_US);
    send(st, "X", 408);
    lines.clear();
    send(st, "MODE,0", 409);                // not enabled: refused
    assert(saw("$GROUND_EVENT,REJECTED_NOT_ENABLED") && ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
}

// MODE,3 = TEST: CH4 full manual with its Falcon, CH9 high.
static void testTestMode() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 1);
    lines.clear();
    send(st, "MODE,3", 2);
    assert(saw("$GROUND_EVENT,MODE_TEST"));
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    rcmap_command d = onAir(st);
    assert(d.test_mode && rcmap_command_mode(&d) == RCMAP_MODE_TEST);
    // The Falcon slider works as in full manual.
    send(st, "FLY,0.46", 3);
    assert(ch4(st) == 1000 + 3 * 46);
    d = onAir(st);
    assert(rcmap_command_mode(&d) == RCMAP_MODE_TEST && nearf(d.flywheel, 0.46f));
    // FLYO is the autos' offset: dropped.
    send(st, "FLYO,5", 4);
    assert(onAir(st).flywheel_offset_pct == 0 && ch4(st) == 1000 + 3 * 46);
    // MODE,3 again: no change, flywheel kept.
    send(st, "MODE,3", 5);
    assert(nearf(onAir(st).flywheel, 0.46f));
    // Any other mode clears CH9 and the flywheel.
    send(st, "MODE,1", 6);
    d = onAir(st);
    assert(!d.test_mode && rcmap_command_mode(&d) == RCMAP_MODE_SEMI_AUTO);
    send(st, "MODE,3", 7);
    send(st, "FLY,0.3", 8);
    send(st, "MODE,2", 9);
    d = onAir(st);
    assert(rcmap_command_mode(&d) == RCMAP_MODE_FULL_MANUAL && nearf(d.flywheel, 0.0f));
    // M leaves test for full manual (as from full auto).
    send(st, "MODE,3", 10);
    send(st, "M", 11);
    d = onAir(st);
    assert(rcmap_command_mode(&d) == RCMAP_MODE_FULL_MANUAL);
    // X (and every release) ends test mode: the next E must be manual.
    send(st, "MODE,3", 12);
    send(st, "X", 13);
    d = onAir(st);
    assert(!d.test_mode && rcmap_command_mode(&d) == RCMAP_MODE_FULL_MANUAL);
    // A PC timeout too.
    send(st, "E", 400);
    send(st, "MODE,3", 401);
    st.tick(401 + GroundCommandState::PC_TIMEOUT_MS + 1, collect, nullptr);
    assert(!onAir(st).test_mode);
}

static void testFlywheel() {
    GroundCommandState st;
    st.reset(0);
    lines.clear();
    send(st, "FLY,0.5", 1);                 // not enabled: refused
    assert(saw("$GROUND_EVENT,REJECTED_NOT_ENABLED") && ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    send(st, "E", 2);
    send(st, "FLY,0.42", 3);                // full manual: on CH4, 3 us per 1%
    assert(ch4(st) == 1000 + 3 * 42);
    assert(nearf(onAir(st).flywheel, 0.42f));
    send(st, "FLY,1", 4);
    assert(ch4(st) == 1300);
    send(st, "FLY,0", 5);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    lines.clear();
    for (const char* bad : {"FLY,1.5", "FLY,-0.1", "FLY,nan", "FLY,", "FLY,0.3x"}) send(st, bad, 6);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    // A mode change drops it (the robot does the same) and an auto mode
    // never carries it.
    send(st, "FLY,0.6", 7);
    send(st, "MODE,1", 8);
    assert(ch4(st) == RCMAP_MODE_SEMI_AUTO_US);
    send(st, "FLY,0.6", 9);
    assert(ch4(st) == RCMAP_MODE_SEMI_AUTO_US);
    send(st, "MODE,2", 10);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "FLY,0.6", 11);
    send(st, "MODE,2", 12);                 // same mode again: kept
    assert(ch4(st) == 1180);
    send(st, "M", 13);
    send(st, "M", 14);
    send(st, "M", 15);                      // back round to manual: off
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    // X, a PC timeout and E all turn it off.
    send(st, "FLY,0.6", 16);
    send(st, "X", 17);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "E", 18);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "FLY,0.6", 19);
    send(st, "E", 20);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "FLY,0.6", 21);
    st.tick(21 + GroundCommandState::PC_TIMEOUT_MS + 1, collect, nullptr);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);

    // FLYO (the autos' LT / RT offset): semi 1750 + 4 us per %, ignored in
    // manual, dropped on a mode change.
    send(st, "E", 3000);
    send(st, "FLYO,5", 3001);                // full manual: not kept
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US);
    send(st, "MODE,1", 3002);
    send(st, "FLYO,-3", 3003);
    assert(ch4(st) == RCMAP_MODE_SEMI_AUTO_US - 12);
    assert(onAir(st).flywheel_offset_pct == -3);
    lines.clear();
    for (const char* bad : {"FLYO,21", "FLYO,-21", "FLYO,", "FLYO,2.5"}) send(st, bad, 3004);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && ch4(st) == RCMAP_MODE_SEMI_AUTO_US - 12);
    send(st, "MODE,0", 3005);                // mode change: back to 0
    assert(ch4(st) == RCMAP_MODE_FULL_AUTO_US);
    send(st, "FLYO,20", 3006);
    assert(ch4(st) == RCMAP_MODE_FULL_AUTO_US + 80);
    send(st, "X", 3007);
    assert(ch4(st) == RCMAP_MODE_FULL_MANUAL_US && st.command().flywheel_offset_pct == 0);
}

static std::string lastLineStarting(const std::string& prefix) {
    for (auto it = lines.rbegin(); it != lines.rend(); ++it) {
        if (it->rfind(prefix, 0) == 0) return *it;
    }
    return "";
}

static void testVisionFrames() {
    RobotStatusTracker tr;
    tr.reset(0);
    lines.clear();
    tr.tick(250, collect, nullptr);
    assert(lastLineStarting("$VISION,") == "$VISION,0,0,0,0,0,0"); // none yet
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 3, RCMAP_EVENT_OUTPUTS_ENABLED),
                260, collect, nullptr);
    rcmap_vision v{};
    v.flywheel_pulse_us = 1290; v.tx_px = -62.5f; v.ty_px = 20.5f; v.valid = true; v.target_id = 10;
    rcmap_status vs{};
    rcmap_vision_to_status(&v, &vs.flags, &vs.event_seq, &vs.event_code);
    lines.clear();
    tr.onStatus(vs, 300, collect, nullptr);
    assert(lines.empty());                  // no bogus event from it
    tr.tick(500, collect, nullptr);
    assert(lastLineStarting("$VISION,") == "$VISION,1,1290,-125,41,1,10");
    // ...and the status is untouched by it.
    assert(lastWireless().rfind("$WIRELESS,1,1,2,", 0) == 0);
    // Stale after STALE_MS.
    lines.clear();
    tr.tick(300 + RobotStatusTracker::STALE_MS + 250, collect, nullptr);
    assert(lastLineStarting("$VISION,") == "$VISION,0,0,0,0,0,0");
}

static void testArmEdge() {
    GroundCommandState st;
    st.reset(1000);
    // CH6 has been low since boot: E raises it at once.
    send(st, "E", 1000);
    assert(st.isEnabled() && st.command().armed);
    const uint32_t e1 = st.enableEpoch();
    // E again while CH6 is high (the robot disarmed itself but the ground
    // still holds arm): CH6 must go low for ARM_LOW_MS before rising, or
    // the robot never sees the edge it needs.
    send(st, "E", 2000);
    assert(st.isEnabled() && !st.command().armed && st.enableEpoch() != e1);
    send(st, "T,1,1", 2050);              // commands are accepted meanwhile
    assert(nearf(onAir(st).forward, 1));
    st.tick(2000 + GroundCommandState::ARM_LOW_MS - 1, collect, nullptr);
    assert(!st.command().armed);
    send(st, "K", 2150);
    st.tick(2000 + GroundCommandState::ARM_LOW_MS, collect, nullptr);
    assert(st.command().armed);
    // X then a quick E: low only for the remainder of ARM_LOW_MS.
    const uint32_t e2 = st.enableEpoch();
    send(st, "X", 3000);
    assert(!st.isEnabled() && !st.command().armed && st.enableEpoch() != e2);
    send(st, "E", 3100);
    assert(!st.command().armed);
    st.tick(3000 + GroundCommandState::ARM_LOW_MS, collect, nullptr);
    assert(st.command().armed);
    // A release during the low wait: stays low, nothing rises later.
    send(st, "E", 4000);
    send(st, "X", 4050);
    st.tick(4500, collect, nullptr);
    assert(!st.command().armed && !st.isEnabled());
    // Epoch: unchanged by a release that had nothing enabled.
    const uint32_t e3 = st.enableEpoch();
    send(st, "X", 4600);
    assert(st.enableEpoch() == e3);
}

static void testStatusInvalidate() {
    RobotStatusTracker tr;
    tr.reset(0);
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 1, RCMAP_EVENT_OUTPUTS_ENABLED),
                100, collect, nullptr);
    assert(tr.fresh(200));
    // After an E / release the old status is not reported as current...
    tr.invalidate();
    assert(!tr.fresh(200));
    lines.clear();
    tr.tick(300, collect, nullptr);
    assert(lastWireless().rfind("$WIRELESS,0,", 0) == 0);
    // ...until the next status, whose new event still comes through.
    lines.clear();
    tr.onStatus(status(0, 2, RCMAP_EVENT_OUTPUTS_DISABLED), 400, collect, nullptr);
    assert(tr.fresh(400) && saw("$EVENT,OUTPUTS_DISABLED"));
    // An aim frame doesn't count as status.
    tr.invalidate();
    rcmap_aim a{};
    a.distance_cm = 300;
    rcmap_status as{};
    rcmap_aim_to_status(&a, &as.flags, &as.event_seq, &as.event_code);
    tr.onStatus(as, 500, collect, nullptr);
    assert(!tr.fresh(500));
}

int main() {
    testDefaultsAndEnable();
    testNudge();
    testFirePulse();
    testTargetAndAutoPath();
    testPcWatchdog();
    testInputHardening();
    testStatusTracker();
    testModeKey();
    testFlywheel();
    testTestMode();
    testArmEdge();
    testVisionFrames();
    testStatusInvalidate();
    testAimFrames();
    printf("ground control host tests: all asserts passed\n");
    return 0;
}
