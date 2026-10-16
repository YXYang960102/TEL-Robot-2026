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
    send(st, "X", 5);                // release keeps the selection
    assert(onAir(st).target == 7);
    send(st, "E", 6);

    send(st, "AUTO,2", 10);
    assert(onAir(st).auto_path == 2);
    send(st, "AUTO,3", 50);          // during the pulse: ignored
    assert(onAir(st).auto_path == 2);
    st.tick(10 + GroundCommandState::AUTO_PULSE_MS, collect, nullptr);
    assert(onAir(st).auto_path == RCMAP_AUTO_NONE);
    send(st, "AUTO,4", 300);
    assert(onAir(st).auto_path == 4);
    lines.clear();
    for (const char* bad : {"TGT,10", "TGT,-1", "TGT,a", "AUTO,0", "AUTO,5", "AUTO,"}) send(st, bad, 301);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && onAir(st).target == 7 && onAir(st).auto_path == 4);
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
    st.tick(1251, collect, nullptr); // 251 ms without a line
    assert(!st.command().armed && nearf(onAir(st).forward, 0));
    assert(saw("$GROUND_EVENT,PC_TIMEOUT"));
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
    // FLY / ANGT have no radio channel: accepted (keep the link alive),
    // nothing sent, no error.
    lines.clear();
    send(st, "FLY,0.4", 2);
    send(st, "ANGT,90", 2);
    assert(lines.empty());
    // Oversized line ending in a valid-looking command: dropped whole, and a
    // corrupt PC stream releases the enable.
    lines.clear();
    send(st, std::string(40, 'z') + "F", 2);
    assert(!st.command().armed && saw("$GROUND_EVENT,COMMAND_TOO_LONG"));
    assert(!onAir(st).fire);
    send(st, "E", 3); // next line parses normally
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
    assert(lines.back() == "$WIRELESS,0,0,0,0,0,0,0,0,-1"); // nothing heard yet
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
    assert(lines.back() == "$WIRELESS,1,1,2,0,0,0,0,50,-1");
    // Status goes stale: reported as not fresh with everything zeroed.
    tr.tick(1500, collect, nullptr);
    assert(lines.back().rfind("$WIRELESS,0,0,0,0,0,0,0,", 0) == 0);
    assert(!tr.fresh(1500));

    // Mode rides in the top bits of the code: the event name is still
    // decoded correctly and $WIRELESS reports the mode.
    lines.clear();
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 7,
                       rcmap_status_code(RCMAP_EVENT_MODE_SEMI_AUTO, RCMAP_MODE_SEMI_AUTO)),
                1600, collect, nullptr);
    assert(saw("$EVENT,MODE_SEMI_AUTO"));
    tr.tick(1760, collect, nullptr);        // reports every 250 ms (last at 1500)
    assert(lines.back() == "$WIRELESS,1,1,2,0,0,0,0,160,1");
    tr.onStatus(status(RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, 8,
                       rcmap_status_code(RCMAP_EVENT_FEED_TRIGGER_SENT, RCMAP_MODE_FULL_AUTO)),
                1800, collect, nullptr);
    assert(saw("$EVENT,FEED_TRIGGER_SENT"));
    tr.tick(2020, collect, nullptr);
    assert(lines.back() == "$WIRELESS,1,1,2,0,0,0,0,220,0");
}

static int ch4(const GroundCommandState& st) {
    int ch[16];
    rcmap_encode(&st.command(), ch);
    return ch[RCMAP_CH_MODE];
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
    printf("ground control host tests: all asserts passed\n");
    return 0;
}
