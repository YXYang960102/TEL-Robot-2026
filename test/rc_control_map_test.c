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
    assert(ch[RCMAP_CH_TARGET] == 1560 && ch[RCMAP_CH_MODE] == 1750); /* 1000 + 80*7 */
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
    c.forward = 5.0f; c.turn = NAN; c.target = 13; c.nudge = 9; c.auto_path = 7;
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
        int ch[16] = {1500, 1500, 1401, 2000, 988, 2012, six[k], six[4 - k], 1500, 1500};
        assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
        assert(d.nudge == k && d.auto_path == 4 - k && d.armed && !d.fire);
        assert(d.target == RCMAP_TARGET_NONE + 5); /* 1401 -> hole 5 (1000 + 80*5) */
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
    memcpy(ch, ok, sizeof ch); ch[RCMAP_CH_TARGET] = 1120;  /* between holes 1 and 2 (1080 / 1160) */
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
    assert(rcmap_mode_from_us(1399) == RCMAP_MODE_FULL_MANUAL);
    assert(rcmap_mode_from_us(1400) == RCMAP_MODE_FULL_AUTO);
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

/* Aim frames: alternate with status in the same ST line / Battery frame. */
static void test_vision_frames(void) {
    rcmap_vision v;
    memset(&v, 0, sizeof v);
    v.flywheel_pulse_us = 1290; v.tx_px = -62.5f; v.ty_px = 20.4f; v.valid = true; v.target_id = 10;
    rcmap_status s;
    memset(&s, 0, sizeof s);
    rcmap_vision_to_status(&v, &s.flags, &s.event_seq, &s.event_code);
    /* Never mistaken for status or aim; the ST line field ranges hold. */
    assert(rcmap_status_is_vision(s.event_code) && !rcmap_status_is_aim(s.event_code));
    assert(s.event_seq <= 0xFFFFFFu);
    rcmap_aim a;
    assert(!rcmap_aim_from_status(&s, &a));
    rcmap_vision back;
    assert(rcmap_vision_from_status(&s, &back));
    assert(back.flywheel_pulse_us == 1290 && back.valid && back.target_id == 10);
    assert(back.tx_px == -62.5f && back.ty_px == 20.5f); /* 0.5 px steps */
    /* Clamps: pulse 16383 us, tx / ty +-1023.5 px. */
    v.flywheel_pulse_us = 40000; v.tx_px = 5000.0f; v.ty_px = -5000.0f; v.valid = false; v.target_id = 0;
    rcmap_vision_to_status(&v, &s.flags, &s.event_seq, &s.event_code);
    assert(rcmap_vision_from_status(&s, &back));
    assert(back.flywheel_pulse_us == 0x3FFF && back.tx_px == 1023.5f && back.ty_px == -1023.5f && !back.valid);
    /* Status codes (<= 0x3FFF) and aim frames are not vision frames. */
    for (uint8_t m = 0; m <= RCMAP_MODE_FULL_MANUAL; m++) {
        assert(!rcmap_status_is_vision(rcmap_status_code(RCMAP_EVENT_COUNT - 1, m)));
    }
    assert(!rcmap_status_is_vision(RCMAP_AIM_FRAME_BIT | 0x4000u));
}

static void test_flywheel_on_mode_channel(void) {
    rcmap_command c = base();
    c.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    int ch[16];
    rcmap_command d;
    /* Off = the plain manual level, so an older ground's values mean the
     * same thing. */
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == 2000);
    /* Every 1% step round-trips exactly, also with the link's +-1 us. */
    for (int p = 1; p <= 100; p++) {
        c.flywheel = (float)p / 100.0f;
        rcmap_encode(&c, ch);
        assert(ch[RCMAP_CH_MODE] == 1000 + 3 * p);
        for (int jitter = -1; jitter <= 1; jitter++) {
            int jittered[16];
            memcpy(jittered, ch, sizeof ch);
            jittered[RCMAP_CH_MODE] += jitter;
            assert(rcmap_decode(jittered, RCMAP_CHANNEL_COUNT, &d));
            assert(rcmap_mode_from_us(d.mode_us) == RCMAP_MODE_FULL_MANUAL);
            assert(d.mode_us == RCMAP_MODE_FULL_MANUAL_US);
            assert(near(d.flywheel, (float)p / 100.0f));
        }
    }
    /* Clamped, and NaN is off. */
    c.flywheel = 1.7f;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == 1300);
    c.flywheel = -0.3f;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == 2000);
    c.flywheel = NAN;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == 2000);
    /* Never sent outside full manual. */
    c.flywheel = 0.8f;
    c.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_SEMI_AUTO_US);
    c.mode_us = RCMAP_MODE_FULL_AUTO_US;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_AUTO_US);
    /* The old levels decode with the flywheel off. */
    const int levels[3] = {RCMAP_MODE_FULL_AUTO_US, RCMAP_MODE_SEMI_AUTO_US,
                           RCMAP_MODE_FULL_MANUAL_US};
    for (int i = 0; i < 3; i++) {
        ch[RCMAP_CH_MODE] = levels[i];
        assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
        assert(d.mode_us == levels[i] && d.flywheel == 0.0f);
    }
    /* ELRS end point below 1000 = off; the gap is rejected, not guessed. */
    ch[RCMAP_CH_MODE] = 988;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.flywheel == 0.0f);
    for (int us = 1304; us < RCMAP_MODE_FULL_AUTO_MIN_US; us++) {
        ch[RCMAP_CH_MODE] = us;
        assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    }
    ch[RCMAP_CH_MODE] = RCMAP_MODE_FULL_AUTO_MIN_US;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    assert(rcmap_mode_from_us(d.mode_us) == RCMAP_MODE_FULL_AUTO && d.flywheel == 0.0f);

    /* Auto flywheel offset (LT / RT): level + 4 us per 1%, +-20%, every
     * value round-trips (also with +-1 us) and stays in its mode's band;
     * never sent or decoded in full manual. */
    const int auto_modes[2] = {RCMAP_MODE_SEMI_AUTO_US, RCMAP_MODE_FULL_AUTO_US};
    for (int m = 0; m < 2; m++) {
        for (int off = -RCMAP_AUTO_OFFSET_MAX_PCT; off <= RCMAP_AUTO_OFFSET_MAX_PCT; off++) {
            rcmap_command a = base();
            a.mode_us = auto_modes[m];
            a.flywheel_offset_pct = (int8_t)off;
            rcmap_encode(&a, ch);
            assert(ch[RCMAP_CH_MODE] == auto_modes[m] + 4 * off);
            for (int jitter = -1; jitter <= 1; jitter++) {
                int j[16];
                memcpy(j, ch, sizeof ch);
                j[RCMAP_CH_MODE] += jitter;
                assert(rcmap_decode(j, RCMAP_CHANNEL_COUNT, &d));
                assert(d.mode_us == auto_modes[m] && d.flywheel_offset_pct == off);
            }
        }
    }
    c = base();
    c.mode_us = RCMAP_MODE_SEMI_AUTO_US;
    c.flywheel_offset_pct = 90;                       /* clamped */
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_SEMI_AUTO_US + 80);
    c.mode_us = RCMAP_MODE_FULL_MANUAL_US;            /* manual: ignored */
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_MODE] == RCMAP_MODE_FULL_MANUAL_US);
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.flywheel_offset_pct == 0);
}

static void test_aim_frames(void) {
    /* A status code can never look like an aim frame: every event x mode. */
    for (uint16_t e = 0; e < 0x1000u; e++) {
        for (uint8_t m = 0; m <= RCMAP_MODE_FULL_MANUAL; m++) {
            assert(!rcmap_status_is_aim(rcmap_status_code(e, m)));
        }
        assert(!rcmap_status_is_aim(e)); /* older firmware: no mode */
    }

    rcmap_aim a = {0};
    a.distance_cm = 352;            /* 3.52 m */
    a.elevation_pulse_us = 1620;
    a.selected_id = 10;             /* key 1 */
    a.target_id = 10;
    a.valid = true;
    a.aim_offset_px = -15;
    uint8_t flags;
    uint32_t seq;
    uint16_t code;
    rcmap_aim_to_status(&a, &flags, &seq, &code);
    /* Within the robot bridge's ST-line field ranges (flags <= 255 is the
     * type; seq <= 0xFFFFFF, code <= 65535). */
    assert(seq <= 0xFFFFFFu);
    assert(rcmap_status_is_aim(code));

    /* Mega -> bridge (Battery) -> ground, exactly like a status frame. */
    rcmap_status st = {0};
    st.flags = flags;
    st.event_seq = seq;
    st.event_code = code;
    st.rolling = 7;
    uint16_t v, cur;
    uint32_t cap;
    uint8_t rem;
    rcmap_status_to_battery(&st, &v, &cur, &cap, &rem);
    rcmap_status got;
    assert(rcmap_status_from_battery(v, cur, cap, rem, &got));
    rcmap_aim back;
    assert(rcmap_aim_from_status(&got, &back));
    assert(back.distance_cm == 352 && back.elevation_pulse_us == 1620);
    assert(back.selected_id == 10 && back.target_id == 10 && back.valid);
    assert(back.aim_offset_px == -15);

    /* Clamps and masks. */
    a.distance_cm = 60000;
    a.elevation_pulse_us = 9000;
    a.selected_id = 0;
    a.target_id = 0;
    a.valid = false;
    a.aim_offset_px = 127;
    rcmap_aim_to_status(&a, &flags, &seq, &code);
    st.flags = flags;
    st.event_seq = seq;
    st.event_code = code;
    assert(rcmap_aim_from_status(&st, &back));
    assert(back.distance_cm == 0x7FFFu && back.elevation_pulse_us == 0x0FFFu);
    assert(!back.valid && back.selected_id == 0 && back.target_id == 0 && back.aim_offset_px == 127);

    /* A status frame is not an aim frame. */
    st.event_code = rcmap_status_code(RCMAP_EVENT_OUTPUTS_ENABLED, RCMAP_MODE_FULL_MANUAL);
    assert(!rcmap_aim_from_status(&st, &back));
}

/* TEST mode (2026-10-26): CH9 high with CH4 in full manual (+ its Falcon). */
static void test_test_mode(void) {
    rcmap_command c;
    memset(&c, 0, sizeof c);
    c.mode_us = RCMAP_MODE_FULL_MANUAL_US;
    c.test_mode = true;
    c.flywheel = 0.46f;
    assert(rcmap_command_mode(&c) == RCMAP_MODE_TEST);
    int ch[16];
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_TEST] == 2000);
    assert(ch[RCMAP_CH_MODE] == RCMAP_FLYWHEEL_BASE_US + 138); /* 46% = 138 us */
    rcmap_command d;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    assert(d.test_mode && rcmap_command_mode(&d) == RCMAP_MODE_TEST && near(d.flywheel, 0.46f));
    /* ELRS 6-position high (1807 / 2012) still counts; flywheel 0 too. */
    ch[RCMAP_CH_TEST] = 1807;
    ch[RCMAP_CH_MODE] = RCMAP_MODE_FULL_MANUAL_US;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && rcmap_command_mode(&d) == RCMAP_MODE_TEST);
    assert(near(d.flywheel, 0.0f));
    /* An older ground's 1500, or anything below the switch threshold: no. */
    for (int us = 988; us <= 1602; us += 307) {
        ch[RCMAP_CH_TEST] = us;
        assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && !d.test_mode);
        assert(rcmap_command_mode(&d) == RCMAP_MODE_FULL_MANUAL);
    }
    /* CH4 at an auto level wins over a lagging CH9 high. */
    ch[RCMAP_CH_TEST] = 2012;
    ch[RCMAP_CH_MODE] = RCMAP_MODE_SEMI_AUTO_US;
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && !d.test_mode);
    assert(rcmap_command_mode(&d) == RCMAP_MODE_SEMI_AUTO);
    /* Encode never sends the flag with an auto level. */
    c.mode_us = RCMAP_MODE_FULL_AUTO_US;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_TEST] == 1500 && rcmap_command_mode(&c) == RCMAP_MODE_FULL_AUTO);
    /* Status: mode + 1 = 4 would be the vision marker, so TEST goes as
     * full manual in the code + the TEST flag. */
    const uint16_t code = rcmap_status_code(RCMAP_EVENT_MODE_TEST, RCMAP_MODE_TEST);
    assert(!rcmap_status_is_aim(code) && !rcmap_status_is_vision(code));
    assert(rcmap_status_code_mode(code) == RCMAP_MODE_FULL_MANUAL); /* an older ground's view */
    assert(rcmap_status_code_event(code) == RCMAP_EVENT_MODE_TEST);
    assert(rcmap_status_mode(code, RCMAP_STATUS_TEST_MODE | RCMAP_STATUS_ARMED) == RCMAP_MODE_TEST);
    assert(rcmap_status_mode(code, RCMAP_STATUS_ARMED) == RCMAP_MODE_FULL_MANUAL);
    /* The flag means nothing next to an auto mode, or with no mode. */
    assert(rcmap_status_mode(rcmap_status_code(RCMAP_EVENT_NONE, RCMAP_MODE_SEMI_AUTO),
                             RCMAP_STATUS_TEST_MODE) == RCMAP_MODE_SEMI_AUTO);
    assert(rcmap_status_mode(RCMAP_EVENT_NONE, RCMAP_STATUS_TEST_MODE) == RCMAP_MODE_UNKNOWN);
    for (uint8_t m = 0; m <= 15; m++) {
        assert(!rcmap_status_is_vision(rcmap_status_code(RCMAP_EVENT_COUNT - 1, m)));
        assert(!rcmap_status_is_aim(rcmap_status_code(RCMAP_EVENT_COUNT - 1, m)));
    }
    assert(strcmp(rcmap_event_name(RCMAP_EVENT_MODE_TEST), "MODE_TEST") == 0);
}

/* Mega <-> bridge line check (2026-10-27). */
static void test_line_check(void) {
    /* CRC-8/DVB-S2 check value ("123456789" -> 0xBC). */
    assert(rcmap_line_crc8("123456789", 9) == 0xBC);
    char buf[64];
    const char *text = "ST,75,123456,10";
    const uint16_t n = (uint16_t)strlen(text);
    memcpy(buf, text, n);
    const uint16_t len = rcmap_line_seal(buf, n, sizeof buf);
    assert(len == n + RCMAP_LINE_CHECK_CHARS && buf[n] == '*');
    assert(rcmap_line_open(buf, len) == (int)n);
    /* Every single-bit flip anywhere in the line is caught. */
    for (uint16_t i = 0; i < len; i++) {
        for (uint8_t b = 0; b < 8u; b++) {
            buf[i] ^= (char)(1u << b);
            assert(rcmap_line_open(buf, len) < 0);
            buf[i] ^= (char)(1u << b);
        }
    }
    /* No check, a short line, a bad hex digit: rejected. */
    assert(rcmap_line_open(text, n) < 0);
    assert(rcmap_line_open("*0", 2) < 0);
    buf[len - 1] = 'G';
    assert(rcmap_line_open(buf, len) < 0);
    /* No room: nothing written. */
    char small[4] = "abc";
    assert(rcmap_line_seal(small, 3, 4) == 0 && strcmp(small, "abc") == 0);
    /* The longest RC line (10 x 4 digits) and ST line fit their buffers. */
    assert(rcmap_line_seal(buf, 21, 24) == 24);
}

/* CH3 (2026-10-27): 13 levels 80 us apart, 10..12 = the bonus holes. */
static void test_target_levels(void) {
    rcmap_command c, d;
    int ch[16];
    memset(&c, 0, sizeof c);
    c.target = 12;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_TARGET] == 1960);
    assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.target == 12);
    /* +-35 us still that level, +-36 between levels: frame rejected. */
    for (uint8_t k = 0; k <= RCMAP_TARGET_MAX; k++) {
        const int level = 1000 + 80 * k;
        for (int e = -35; e <= 35; e += 35) {
            if (level + e < 900 || level + e > 2100) continue;
            ch[RCMAP_CH_TARGET] = level + e;
            assert(rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d) && d.target == k);
        }
        if (k < RCMAP_TARGET_MAX) {
            ch[RCMAP_CH_TARGET] = level + 40; /* halfway: neither */
            assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
        }
    }
    ch[RCMAP_CH_TARGET] = 2040; /* past hole 12 */
    assert(!rcmap_decode(ch, RCMAP_CHANNEL_COUNT, &d));
    /* Out of range targets are sent as none. */
    c.target = 13;
    rcmap_encode(&c, ch);
    assert(ch[RCMAP_CH_TARGET] == 1000);
}

int main(void) {
    test_round_trip();
    test_elrs_hybrid_values();
    test_invalid_frames();
    test_event_names_and_status();
    test_aim_frames();
    test_flywheel_on_mode_channel();
    test_vision_frames();
    test_operator_mode();
    test_test_mode();
    test_line_check();
    test_target_levels();
    printf("rc_control_map host tests: all asserts passed\n");
    return 0;
}
