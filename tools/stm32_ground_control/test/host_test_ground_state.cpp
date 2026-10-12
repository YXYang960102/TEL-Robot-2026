// Host test for GroundCommandState / RobotStatusTracker: PC line protocol
// -> RC channel state, the ground-side PC watchdog, event counters, input
// hardening (oversized lines, NaN), and robot status -> PC lines.
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

static void encodeCheck(const rcmap_command& c, int expectArm, int expectLeft) {
    int ch[16];
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_ARM] == expectArm);
    assert(ch[RCMAP_CH_LEFT] == expectLeft);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_MANUAL_US);
}

static void testDefaultsAndEnable() {
    GroundCommandState st;
    st.reset(0);
    encodeCheck(st.command(), 1000, 1500); // idle: disarmed, neutral
    send(st, "T,1,1", 10);
    assert(saw("$GROUND_EVENT,REJECTED_NOT_ENABLED"));
    encodeCheck(st.command(), 1000, 1500);

    send(st, "E", 20);
    assert(st.command().armed && saw("$GROUND_EVENT,ENABLE_REQUESTED"));
    send(st, "T,1,-1", 30);
    send(st, "ROT,1", 30);
    send(st, "ANG,-1", 30);
    send(st, "FLY,0.4", 30);
    const rcmap_command& c = st.command();
    assert(c.left == 1.0f && c.right == -1.0f && c.rotate == 1 && c.angle == -1);
    assert(c.flywheel > 0.39f && c.flywheel < 0.41f);
    encodeCheck(c, 2000, 2000);

    send(st, "X", 40);
    assert(!st.command().armed && st.command().left == 0.0f && st.command().flywheel == 0.0f);
    assert(saw("$GROUND_EVENT,ENABLE_RELEASED"));
}

static void testCountersAdvanceOncePerPress() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    const uint8_t f0 = st.command().feed_counter;
    send(st, "F", 1);
    assert(st.command().feed_counter == (f0 + 1) % RCMAP_COUNTER_LEVELS);
    for (int i = 0; i < 6; i++) send(st, "F", 2);
    assert(st.command().feed_counter == (f0 + 7) % RCMAP_COUNTER_LEVELS);
    send(st, "L", 3);
    assert(st.command().lmode_counter == 1);
    // Releasing keeps the counter level, so the robot never sees a step.
    const uint8_t before = st.command().feed_counter;
    send(st, "X", 4);
    assert(st.command().feed_counter == before);
    // F while not enabled is not sent at all.
    send(st, "F", 5);
    assert(st.command().feed_counter == before);
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
    assert(!st.command().armed && st.command().left == 0.0f);
    assert(saw("$GROUND_EVENT,PC_TIMEOUT"));
}

static void testInputHardening() {
    GroundCommandState st;
    st.reset(0);
    send(st, "E", 0);
    send(st, "FLY,0.5", 0);
    lines.clear();
    for (const char* bad : {"FLY,nan", "FLY,inf", "T,1", "T,1,1,1", "T,a,1", "ROT,1e999", "Q"}) {
        send(st, bad, 1);
    }
    assert(st.command().flywheel > 0.49f && st.command().flywheel < 0.51f);
    assert(saw("$GROUND_EVENT,BAD_COMMAND") && saw("$GROUND_EVENT,UNKNOWN_COMMAND"));
    // Oversized line ending in a valid-looking command: dropped whole, and a
    // corrupt PC stream releases the enable.
    lines.clear();
    send(st, std::string(40, 'z') + "F", 2);
    assert(!st.command().armed && saw("$GROUND_EVENT,COMMAND_TOO_LONG"));
    assert(st.command().feed_counter == 0);
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
    assert(lines.back() == "$WIRELESS,0,0,0,0,0,0,0,0"); // nothing heard yet
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
    assert(lines.back() == "$WIRELESS,1,1,2,0,0,0,0,50");
    // Status goes stale: reported as not fresh with everything zeroed.
    tr.tick(1500, collect, nullptr);
    assert(lines.back().rfind("$WIRELESS,0,0,0,0,0,0,0,", 0) == 0);
    assert(!tr.fresh(1500));
}

static void testConveyor() {
    GroundCommandState st; st.reset(0);
    send(st, "R", 1); assert(!st.command().conveyor);
    send(st, "E", 2); send(st, "R", 3); assert(st.command().conveyor);
    int ch[16]; rcmap_command decoded{};
    rcmap_encode(&st.command(), ch);
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &decoded) && decoded.conveyor);
    send(st, "F", 4); assert(st.command().conveyor);
    send(st, "R", 5); assert(!st.command().conveyor);
    send(st, "R", 6); st.tick(300, collect, nullptr); assert(!st.command().conveyor);
    send(st, "E", 301); assert(!st.command().conveyor);
}

int main() {
    testConveyor();
    testDefaultsAndEnable();
    testCountersAdvanceOncePerPress();
    testPcWatchdog();
    testInputHardening();
    testStatusTracker();
    printf("ground control host tests: all asserts passed\n");
    return 0;
}
