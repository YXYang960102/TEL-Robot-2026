/* Host test for src/IO/RcControlMap.c: encode/decode round trip, decoding
 * the values ELRS actually emits in Hybrid mode (6-position AUX channels
 * spread evenly over 988..2012 us, 2-position AUX1 at 988/2012), invalid
 * frames, the event-counter rules (one action per +1, no repeat, no replay
 * after a reconnect, skipped jumps), and the status-in-battery-frame
 * packing. */
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
    c.left = 1.0f; c.right = -1.0f; c.flywheel = 0.4f; c.armed = true;
    c.rotate = -1; c.angle = 1; c.feed_counter = 3; c.lmode_counter = 5;
    int ch[16];
    rcmap_encode(&c, ch);
    assert(ch[0] == 2000 && ch[1] == 1000 && ch[2] == 1400 && ch[3] == 2000);
    assert(ch[4] == 2000 && ch[5] == 1000 && ch[6] == 2000 && ch[7] == 1600 && ch[8] == 2000);
    for (int i = RCMAP_CHANNEL_COUNT; i < 16; i++) assert(ch[i] == 1500);
    rcmap_command d;
    assert(rcmap_decode(ch, 16, &d));
    assert(near(d.left, 1.0f) && near(d.right, -1.0f) && near(d.flywheel, 0.4f));
    assert(d.armed && d.rotate == -1 && d.angle == 1);
    assert(d.feed_counter == 3 && d.lmode_counter == 5 && d.mode_us == 2000);

    /* Disarmed, neutral defaults. */
    c = base();
    rcmap_encode(&c, ch);
    assert(rcmap_decode(ch, 16, &d));
    assert(!d.armed && near(d.left, 0) && near(d.right, 0) && d.flywheel == 0.0f);
    assert(d.rotate == 0 && d.angle == 0 && d.feed_counter == 0);

    /* Out-of-range inputs are clamped, NaN goes neutral. */
    c = base();
    c.left = 5.0f; c.right = NAN; c.flywheel = -3.0f;
    rcmap_encode(&c, ch);
    assert(ch[0] == 2000 && ch[1] == 1500 && ch[2] == 1000);
}

static void test_elrs_hybrid_values(void) {
    /* What ELRS Hybrid mode emits for a 6-position AUX channel and the
       2-position AUX1, i.e. not our exact encoded values. */
    const int six[6] = {988, 1193, 1398, 1602, 1807, 2012};
    int ch[16];
    rcmap_command d;
    for (int k = 0; k < 6; k++) {
        for (int i = 0; i < 16; i++) ch[i] = 1500;
        ch[4] = 2012;
        ch[5] = 1398; /* 1500 quantized down */
        ch[6] = 1602; /* 1500 quantized up */
        ch[7] = six[k];
        ch[8] = six[(k + 2) % 6];
        assert(rcmap_decode(ch, 16, &d));
        assert(d.feed_counter == k && d.lmode_counter == (k + 2) % 6);
        assert(d.armed && d.rotate == 0 && d.angle == 0);
    }
    ch[4] = 988;
    ch[5] = 988;
    ch[6] = 2012;
    assert(rcmap_decode(ch, 16, &d));
    assert(!d.armed && d.rotate == -1 && d.angle == 1);
    ch[4] = 1500; /* anything not clearly high is NOT armed */
    assert(rcmap_decode(ch, 16, &d) && !d.armed);
}

static void test_invalid_frames(void) {
    int ch[16];
    rcmap_command c = base();
    rcmap_encode(&c, ch);
    rcmap_command d;
    memset(&d, 0x5A, sizeof d);
    const rcmap_command before = d;

    int bad[16];
    memcpy(bad, ch, sizeof ch);
    bad[0] = 850; /* below 900 */
    assert(!rcmap_decode(bad, 16, &d));
    memcpy(bad, ch, sizeof ch);
    bad[2] = 2200;
    assert(!rcmap_decode(bad, 16, &d));
    memcpy(bad, ch, sizeof ch);
    bad[7] = 1100; /* halfway between two counter levels */
    assert(!rcmap_decode(bad, 16, &d));
    assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT - 1, &d)); /* too few channels */
    assert(memcmp(&d, &before, sizeof d) == 0); /* untouched */
}

static void test_counter_rules(void) {
    rcmap_counter c;
    rcmap_counter_reset(&c);
    /* First level after (re)connect is only a baseline: no replay. */
    assert(rcmap_counter_update(&c, 4) == RCMAP_COUNTER_NONE);
    /* Same level repeated across many packets: nothing. */
    for (int i = 0; i < 20; i++) assert(rcmap_counter_update(&c, 4) == RCMAP_COUNTER_NONE);
    /* +1: exactly one event, then quiet again. */
    assert(rcmap_counter_update(&c, 5) == RCMAP_COUNTER_EVENT);
    assert(rcmap_counter_update(&c, 5) == RCMAP_COUNTER_NONE);
    /* Wrap 5 -> 0 is +1. */
    assert(rcmap_counter_update(&c, 0) == RCMAP_COUNTER_EVENT);
    /* Jump of +2 (two presses squeezed into one gap): ignored, rebaselined. */
    assert(rcmap_counter_update(&c, 2) == RCMAP_COUNTER_SKIPPED);
    assert(rcmap_counter_update(&c, 3) == RCMAP_COUNTER_EVENT);
    /* Going backwards (= +5 mod 6) is also a skip, never an action. */
    assert(rcmap_counter_update(&c, 2) == RCMAP_COUNTER_SKIPPED);
    /* Reconnect: reset, then the stale level from before is a baseline. */
    rcmap_counter_reset(&c);
    assert(rcmap_counter_update(&c, 3) == RCMAP_COUNTER_NONE);
}

static void test_event_names_and_status(void) {
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_OUTPUTS_ENABLED), "OUTPUTS_ENABLED") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_FEED_BUSY), "FEED_BUSY") == 0);
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_EVENT_SKIPPED), "EVENT_SKIPPED") == 0);
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
    test_counter_rules();
    test_event_names_and_status();
    test_operator_mode();
    printf("rc_control_map host tests: all asserts passed\n");
    return 0;
}
