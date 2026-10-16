/* Host test for src/IO/RcControlMap.c (channel table v2): encode/decode
 * round trip, decoding the values ELRS actually emits in Hybrid mode
 * (6-position AUX channels at 988/1193/1398/1602/1807/2012 us, 2-position
 * AUX1 at 988/2012), invalid frames, the operator mode, and the
 * status-in-battery-frame packing. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../src/IO/RcControlMap.h"

static int near(float a, float b) { return fabsf(a - b) < 0.01f; }

static rcmap_command base(void) {
    rcmap_command c;
    memset(&c, 0, sizeof c);
    return c;
}

static void test_round_trip(void) {
    rcmap_command c = base();
    c.forward = 1.0f; c.turn = -0.5f; c.target = 7; c.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    c.fire = true; c.armed = true; c.nudge = RCMAP_NUDGE_LEFT; c.auto_path = 3;
    int ch[16];
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_FORWARD] == 2000 && ch[RCMAP_CH_TURN] == 1250);
    assert(ch[RCMAP_CH_TARGET] == 1700 && ch[RCMAP_CH_MODE] == 1750);
    assert(ch[RCMAP_CH_FIRE] == 2000 && ch[RCMAP_CH_ARM] == 2000);
    assert(ch[RCMAP_CH_NUDGE] == 1600 && ch[RCMAP_CH_AUTO] == 1600);
    assert(ch[8] == 1500 && ch[9] == 1500 && ch[15] == 1500);
    rcmap_command d;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    assert(near(d.forward, 1.0f) && near(d.turn, -0.5f));
    assert(d.target == 7 && d.fire && d.armed && d.nudge == RCMAP_NUDGE_LEFT && d.auto_path == 3);
    assert(rcmap_mode_from_us(d.mode_us) == RCMAP_MODE_SEMI_AUTO);

    /* All-zero command = everything neutral / released / none. */
    rcmap_encode(&(rcmap_command){0}, ch);
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    assert(!d.armed && !d.fire && near(d.forward, 0) && near(d.turn, 0));
    assert(d.target == RCMAP_TARGET_NONE && d.nudge == RCMAP_NUDGE_CENTER && d.auto_path == RCMAP_AUTO_NONE);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_MANUAL_US);

    /* Out-of-range fields: axes clamp, NaN is neutral, bad levels send none. */
    c = base();
    c.forward = 5.0f; c.turn = NAN; c.target = 12; c.nudge = 9; c.auto_path = 7;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_FORWARD] == 2000 && ch[RCMAP_CH_TURN] == 1500);
    assert(ch[RCMAP_CH_TARGET] == 1000 && ch[RCMAP_CH_NUDGE] == 1000 && ch[RCMAP_CH_AUTO] == 1000);

    /* Every target hole round-trips. */
    for (uint8_t k = 0; k <= RCMAP_TARGET_MAX; k++) {
        c = base(); c.target = k;
        rcmap_encode(&c, ch);
        assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.target == k);
    }
}

static void test_elrs_hybrid_values(void) {
    /* What the robot actually receives: CH1-4 as sent (+-1 us), CH5
     * 988/2012, CH6+ snapped to the 6-position grid. */
    const int six[6] = {988, 1193, 1398, 1602, 1807, 2012};
    rcmap_command d;
    for (uint8_t k = 0; k < 5; k++) {
        int ch[16] = {1500, 1500, 1501, 2000, 988, 2012, six[k], six[4 - k], 1500, 1500};
        assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
        assert(d.nudge == k && d.auto_path == 4 - k && d.armed && !d.fire);
        assert(d.target == RCMAP_TARGET_NONE + 5); /* 1501 -> hole 5 */
    }
    int ch[16] = {1500, 1500, 1000, 2000, 2012, 988, 988, 988, 1500, 1500};
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.fire && !d.armed);
}

static void test_invalid_frames(void) {
    int ok[16] = {1500, 1500, 1000, 2000, 1000, 1000, 1000, 1000, 1500, 1500};
    rcmap_command d = base();
    assert(rcmap_decode(ok, RCMAP_CHANNEL_COUNT, &d));
    int ch[16];
    /* Any channel outside 900..2100 us rejects the whole frame. */
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        memcpy(ch, ok, sizeof ch); ch[i] = 2200;
        assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
        memcpy(ch, ok, sizeof ch); ch[i] = 800;
        assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    }
    /* Level channels between levels are corrupt, not rounded. */
    memcpy(ch, ok, sizeof ch); ch[RCMAP_CH_TARGET] = 1150;  /* between holes 1 and 2 */
    assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    memcpy(ch, ok, sizeof ch); ch[RCMAP_CH_NUDGE] = 1100;   /* between center and up */
    assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    memcpy(ch, ok, sizeof ch); ch[RCMAP_CH_AUTO] = 2000;    /* a 6th level: not used */
    assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    /* Too few channels. */
    assert(!rcmap_decode(ok, RCMAP_CHANNEL_COUNT - 1, &d));
    /* Fire/arm are fail-safe: only >= 1750 counts as on. */
    memcpy(ch, ok, sizeof ch); ch[RCMAP_CH_FIRE] = 1700; ch[RCMAP_CH_ARM] = 1749;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && !d.fire && !d.armed);
}

static void test_event_names_and_status(void) {
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_OUTPUTS_ENABLED), "OUTPUTS_ENABLED") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_FEED_BUSY), "FEED_BUSY") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_EVENT_SKIPPED), "EVENT_SKIPPED") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_AUTO_PATH_STARTED), "AUTO_PATH_STARTED") == 0);
    assert(strcmp(rcmap_event_name(999), "UNKNOWN_EVENT_CODE") == 0);

    rcmap_status s = {RCMAP_STATUS_ARMED | RCMAP_STATUS_OWNER_WIRELESS, RCMAP_EVENT_FEED_TRIGGER_SENT,
                      0x1234567u, 77};
    uint16_t v, cur;
    uint32_t cap;
    uint8_t rem;
    rcmap_status_to_battery(&s, &v, &cur, &cap, &rem);
    assert(v == 0xA503 && cur == RCMAP_EVENT_FEED_TRIGGER_SENT && cap == 0x234567u && rem == 77);
    rcmap_status back;
    assert(rcmap_status_from_battery(v, cur, cap, rem, &back));
    assert(back.flags == s.flags && back.event_code == s.event_code && back.event_seq == 0x234567u);
    /* A real 12.4 V battery frame (124 in 0.1 V) is not mistaken for status. */
    assert(!rcmap_status_from_battery(124, 0, 0, 100, &back));
}

static void test_operator_mode(void) {
    /* The three levels the ground sends decode to themselves. */
    assert(rcmap_mode_from_us(RCMAP_MODE_FULL_MANUAL_US) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_from_us(RCMAP_MODE_SEMI_AUTO_US) == RCMAP_MODE_SEMI_AUTO);
    assert(rcmap_mode_from_us(RCMAP_MODE_FULL_AUTO_US) == RCMAP_MODE_FULL_AUTO);
    /* Fail-safe: 0, garbage-low and the neutral idle of other channels are
     * not auto; boundaries sit halfway between the sent levels. */
    assert(rcmap_mode_from_us(0) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_from_us(1000) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_from_us(1249) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_from_us(1250) == RCMAP_MODE_FULL_AUTO);
    assert(rcmap_mode_from_us(1624) == RCMAP_MODE_FULL_AUTO);
    assert(rcmap_mode_from_us(1625) == RCMAP_MODE_SEMI_AUTO);
    assert(rcmap_mode_from_us(1874) == RCMAP_MODE_SEMI_AUTO);
    assert(rcmap_mode_from_us(1875) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_to_us(99) == RCMAP_MODE_FULL_MANUAL_US);

    /* Encode sends one of the three exact levels; a zero-initialized
     * command sends manual. */
    rcmap_command c;
    memset(&c, 0, sizeof c);
    int ch[16];
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_MANUAL_US);
    c.mode_us = 1530;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_AUTO_US);
    rcmap_command back;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &back));
    assert(rcmap_mode_from_us(back.mode_us) == RCMAP_MODE_FULL_AUTO);

    /* Status code: event in the low 12 bits, mode + 1 in the top 4. */
    for (uint8_t m = 0; m <= RCMAP_MODE_FULL_MANUAL; m++) {
        const uint16_t code = rcmap_status_code(RCMAP_EVENT_FIRE_NOT_READY, m);
        assert(rcmap_status_code_event(code) == RCMAP_EVENT_FIRE_NOT_READY);
        assert(rcmap_status_code_mode(code) == m);
    }
    /* A bare event code (older robot firmware) has no mode. */
    assert(rcmap_status_code_mode(RCMAP_EVENT_OUTPUTS_ENABLED) == RCMAP_MODE_UNKNOWN);
    assert(rcmap_status_code_event(RCMAP_EVENT_OUTPUTS_ENABLED) == RCMAP_EVENT_OUTPUTS_ENABLED);

    /* It survives the Battery-frame round trip. */
    rcmap_status st;
    memset(&st, 0, sizeof st);
    st.event_code = rcmap_status_code(RCMAP_EVENT_MODE_SEMI_AUTO, RCMAP_MODE_SEMI_AUTO);
    uint16_t v, cur; uint32_t cap; uint8_t rem;
    rcmap_status_to_battery(&st, &v, &cur, &cap, &rem);
    rcmap_status out;
    assert(rcmap_status_from_battery(v, cur, cap, rem, &out));
    assert(rcmap_status_code_mode(out.event_code) == RCMAP_MODE_SEMI_AUTO);

    /* Every event code has a name (the table and the enum stay in step). */
    for (uint16_t e = 0; e < RCMAP_EVENT_COUNT; e++) {
        assert(rcmap_event_name(e) != NULL && rcmap_event_name(e)[0] != '\0');
    }
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_MODE_FULL_AUTO), "MODE_FULL_AUTO") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL),
                  "ENABLE_REJECTED_NOT_MANUAL") == 0);
}

int main(void) {
    test_round_trip();
    test_elrs_hybrid_values();
    test_invalid_frames();
    test_event_names_and_status();
    test_operator_mode();
    printf("rc_control_map host tests: all asserts passed\n");
    return 0;
}
