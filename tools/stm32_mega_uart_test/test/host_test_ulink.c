/* Host test for src/Bench/UartLinkTest/ulink.c: unit checks plus a
 * millisecond-stepped simulation of the real Mega (M) and STM32 (S)
 * endpoints wired back-to-back through two byte channels with latency and
 * per-line fault injection (drop, corrupt, delay, duplicate, cut). */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ulink.h"

#define CHAN_CAP 200000
#define WIRE_LATENCY_MS 2u

typedef struct {
    char bytes[CHAN_CAP];
    uint32_t due[CHAN_CAP];
    int head, tail;
    /* current line being assembled by the sender, so faults act per line */
    char pending[400];
    int pending_len;
    int lines_sent;
    int a_lines, d_lines, c_lines, s_lines;
    int cut; /* wire disconnected: everything written is lost */
    /* fault hooks (by line type + per-type index), -1 = none */
    char fault_type;
    int fault_index;
    int fault_kind; /* 1 drop, 2 corrupt, 3 delay 80ms, 4 duplicate,
                       5 corrupt but still well-formed (only the CRC can tell) */
} channel;

static uint32_t now_ms;
static channel to_s, to_m; /* M->S, S->M */

static void chan_push(channel *ch, const char *data, int len, uint32_t extra_delay) {
    for (int i = 0; i < len; i++) {
        ch->bytes[ch->tail] = data[i];
        ch->due[ch->tail] = now_ms + WIRE_LATENCY_MS + extra_delay;
        ch->tail++;
        assert(ch->tail < CHAN_CAP);
    }
}

static void chan_write(void *ctx, const char *data, uint16_t len) {
    channel *ch = (channel *)ctx;
    for (uint16_t i = 0; i < len; i++) {
        ch->pending[ch->pending_len++] = data[i];
        assert(ch->pending_len < (int)sizeof(ch->pending));
        if (data[i] != '\n') continue;

        const char type = ch->pending[0];
        int index = -1;
        if (type == 'A') index = ch->a_lines++;
        if (type == 'D') index = ch->d_lines++;
        if (type == 'C') index = ch->c_lines++;
        if (type == 'S') index = ch->s_lines++;
        ch->lines_sent++;

        int kind = 0;
        if (type == ch->fault_type && index == ch->fault_index) kind = ch->fault_kind;
        if (ch->cut) kind = 1;

        if (kind == 1) {
            /* dropped */
        } else if (kind == 2) {
            ch->pending[2] ^= 0x01; /* flip a bit inside the body */
            chan_push(ch, ch->pending, ch->pending_len, 0);
        } else if (kind == 3) {
            chan_push(ch, ch->pending, ch->pending_len, 80);
        } else if (kind == 5) {
            /* last payload hex digit, just before "*CC\n": swap it for a
               different valid hex digit so every field still parses */
            char *c = &ch->pending[ch->pending_len - 5];
            *c = (*c == '0') ? '1' : '0';
            chan_push(ch, ch->pending, ch->pending_len, 0);
        } else if (kind == 4) {
            chan_push(ch, ch->pending, ch->pending_len, 0);
            chan_push(ch, ch->pending, ch->pending_len, 1);
        } else {
            chan_push(ch, ch->pending, ch->pending_len, 0);
        }
        ch->pending_len = 0;
    }
}

static ulink_endpoint mega, stm;

static void deliver(channel *ch, ulink_endpoint *to) {
    while (ch->head < ch->tail && ch->due[ch->head] <= now_ms) {
        ulink_rx_byte(to, ch->bytes[ch->head], now_ms);
        ch->head++;
    }
}

static void step(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        now_ms++;
        deliver(&to_m, &mega);
        deliver(&to_s, &stm);
        ulink_poll(&mega, now_ms);
        ulink_poll(&stm, now_ms);
    }
}

static void setup(void) {
    memset(&to_s, 0, sizeof(to_s));
    memset(&to_m, 0, sizeof(to_m));
    to_s.fault_index = to_m.fault_index = -1;
    now_ms = 5000;
    ulink_init(&mega, 'M', chan_write, &to_s, now_ms);
    ulink_init(&stm, 'S', chan_write, &to_m, now_ms);
}

static uint16_t next_run = 100;

static void start_run(void) {
    ulink_master_start(&mega, next_run++, now_ms);
    for (int i = 0; i < 2000 && mega.state != ULINK_RUNNING; i++) step(1);
    assert(mega.state == ULINK_RUNNING);
}

static void stop_and_drain(void) {
    ulink_master_stop(&mega, now_ms);
    for (int i = 0; i < 3000 && mega.state != ULINK_DONE; i++) step(1);
    assert(mega.state == ULINK_DONE);
}

/* deliver() is strictly FIFO: a line delayed by a fault also holds back
   the bytes queued behind it, like a stalled wire would. */

static void test_crc_and_payload(void) {
    assert(ulink_crc8("123456789", 9) == 0xF4); /* CRC-8/SMBUS check value */
    assert(ulink_payload('M', 7, 3) == ulink_payload('M', 7, 3));
    assert(ulink_payload('M', 7, 3) != ulink_payload('S', 7, 3));
    assert(ulink_payload('M', 7, 3) != ulink_payload('M', 8, 3));
    assert(ulink_payload('M', 7, 3) != ulink_payload('M', 7, 4));
}

static void expect_clean(const ulink_stats *s) {
    assert(s->timeout == 0 && s->ack_stale == 0 && s->ack_dup == 0);
    assert(s->ack_mismatch == 0 && s->bad == 0 && s->overflow == 0);
    assert(s->rx_gap == 0 && s->rx_dup == 0);
}

static void test_clean_60s_run(void) {
    setup();
    start_run();
    step(60000);
    stop_and_drain();
    assert(mega.drain_confirmed);
    const ulink_stats *m = &mega.st, *s = &mega.peer;
    printf("  clean 60s: M tx=%u ok=%u rx=%u | S tx=%u ok=%u rx=%u | rtt %u..%u ms\n",
           m->tx, m->ack_ok, m->rx_data, s->tx, s->ack_ok, s->rx_data,
           m->rtt_min_ms, m->rtt_max_ms);
    assert(m->tx >= 600 && m->tx <= 602);
    assert(s->tx >= 600 && s->tx <= 602);
    assert(m->ack_ok == m->tx && s->ack_ok == s->tx);
    assert(m->rx_data == s->tx && s->rx_data == m->tx); /* exact both ways */
    expect_clean(m);
    expect_clean(s);
    assert(m->rtt_min_ms >= 2 * WIRE_LATENCY_MS && m->rtt_max_ms <= 2 * WIRE_LATENCY_MS + 2);
    /* An ACK never triggers an ACK: the STM32 sent exactly one A per D it
       received, and nothing else besides its own D/S lines. */
    assert(to_m.a_lines == to_s.d_lines);
    assert(to_s.a_lines == to_m.d_lines);
    assert(ulink_take_events(&mega) == ULINK_EVENT_LINK_UP);
    assert(mega.link_up);
}

static void test_dropped_data_counts_timeout_and_gap(void) {
    setup();
    start_run();
    to_s.fault_type = 'D'; to_s.fault_index = 10; to_s.fault_kind = 1;
    to_m.fault_type = 'D'; to_m.fault_index = 20; to_m.fault_kind = 1;
    step(5000);
    stop_and_drain();
    assert(mega.st.timeout == 1 && mega.peer.rx_gap == 1);
    assert(mega.peer.timeout == 1 && mega.st.rx_gap == 1);
    assert(mega.st.ack_ok == mega.st.tx - 1 && mega.peer.ack_ok == mega.peer.tx - 1);
    assert(mega.st.ack_stale == 0 && mega.peer.ack_stale == 0);
}

static void test_dropped_ack_is_timeout_not_gap(void) {
    setup();
    start_run();
    to_m.fault_type = 'A'; to_m.fault_index = 15; to_m.fault_kind = 1;
    step(5000);
    stop_and_drain();
    assert(mega.st.timeout == 1 && mega.peer.rx_gap == 0); /* data arrived, ACK didn't */
    assert(mega.peer.rx_data == mega.st.tx);
}

static void test_corrupt_line_is_bad_and_times_out(void) {
    setup();
    start_run();
    to_s.fault_type = 'D'; to_s.fault_index = 7; to_s.fault_kind = 2;
    step(3000);
    stop_and_drain();
    assert(mega.peer.bad == 1);
    assert(mega.st.timeout == 1 && mega.peer.rx_gap == 1);
    assert(mega.st.ack_ok + 1 == mega.st.tx);
}

static void test_well_formed_corruption_caught_by_crc(void) {
    setup();
    start_run();
    to_m.fault_type = 'A'; to_m.fault_index = 11; to_m.fault_kind = 5;
    step(3000);
    stop_and_drain();
    assert(mega.st.bad == 1);          /* rejected by CRC... */
    assert(mega.st.ack_mismatch == 0); /* ...never reached ACK matching */
    assert(mega.st.timeout == 1 && mega.st.ack_ok + 1 == mega.st.tx);
}

static void test_late_ack_is_stale_not_success(void) {
    setup();
    start_run();
    to_m.fault_type = 'A'; to_m.fault_index = 12; to_m.fault_kind = 3; /* +80 ms */
    step(4000);
    stop_and_drain();
    /* The late ACK arrives after the 50 ms timeout. FIFO delivery also
       stalls the S->M bytes queued behind it, so a couple of S's own data
       timing may shift, but nothing late may be counted as a success. */
    assert(mega.st.timeout >= 1);
    assert(mega.st.ack_stale >= 1);
    assert(mega.st.ack_ok + mega.st.timeout == mega.st.tx);
    assert(mega.st.ack_mismatch == 0 && mega.st.bad == 0);
}

static void test_duplicate_ack_counted_once(void) {
    setup();
    start_run();
    to_m.fault_type = 'A'; to_m.fault_index = 9; to_m.fault_kind = 4;
    step(3000);
    stop_and_drain();
    assert(mega.st.ack_dup == 1);
    assert(mega.st.ack_ok == mega.st.tx && mega.st.timeout == 0);
}

static void test_disconnect_and_recover(void) {
    setup();
    start_run();
    step(2000);
    assert(mega.link_up);
    (void)ulink_take_events(&mega);

    to_s.cut = to_m.cut = 1; /* pull the wires */
    step(2000);
    assert(!mega.link_up);
    assert(ulink_take_events(&mega) == ULINK_EVENT_LINK_DOWN);
    const uint32_t timeouts_while_cut = mega.st.timeout;
    assert(timeouts_while_cut >= 19 && timeouts_while_cut <= 21);

    to_s.cut = to_m.cut = 0; /* plug back in */
    step(1000);
    assert(mega.link_up);
    assert(ulink_take_events(&mega) == ULINK_EVENT_LINK_UP);
    const uint32_t ok_before = mega.st.ack_ok;
    step(1000);
    assert(mega.st.ack_ok >= ok_before + 9);
    assert(mega.st.timeout == timeouts_while_cut); /* no new timeouts after recovery */
    stop_and_drain();
    assert(mega.drain_confirmed);
}

static void test_stop_while_disconnected_reports_unconfirmed(void) {
    setup();
    start_run();
    step(1000);
    to_s.cut = to_m.cut = 1;
    stop_and_drain();
    assert(!mega.drain_confirmed);
}

static void test_start_survives_lost_control_lines(void) {
    setup();
    to_s.fault_type = 'C'; to_s.fault_index = 0; to_s.fault_kind = 1;
    start_run();
    assert(to_s.c_lines >= 2);
    step(1000);
    stop_and_drain();
    assert(mega.drain_confirmed && mega.st.ack_ok == mega.st.tx);
}

static void test_oversized_line_resyncs(void) {
    setup();
    start_run();
    step(500);
    /* Garbage longer than the line buffer whose tail looks like a valid
       control/data line: must be dropped whole, and the next real line
       parses normally. */
    char junk[400];
    memset(junk, 'x', sizeof(junk));
    const char *tail = "C,1,0*00";
    memcpy(junk + sizeof(junk) - 1 - strlen(tail), tail, strlen(tail));
    junk[sizeof(junk) - 1] = '\n';
    chan_push(&to_s, junk, sizeof(junk), 0);
    step(2000);
    assert(stm.st.overflow == 1);
    assert(stm.sending); /* the "stop" in the tail was not obeyed */
    stop_and_drain();
    assert(mega.peer.overflow == 1 && mega.peer.bad == 0);
    assert(mega.st.ack_ok == mega.st.tx);
}

static void test_wrong_run_and_wrong_role_not_counted(void) {
    setup();
    start_run();
    step(500);
    /* An ACK for some other run, and a D claiming to come from the Mega
       itself: neither may count as anything good. */
    const uint32_t ok_before = mega.st.ack_ok;
    char line[64];
    int n = snprintf(line, sizeof(line), "A,S,%u,0,%08X", (unsigned)(mega.run + 1),
                     (unsigned)ulink_payload('M', (uint16_t)(mega.run + 1), 0));
    n += snprintf(line + n, sizeof(line) - n, "*%02X\n", ulink_crc8(line, (uint16_t)n));
    chan_push(&to_m, line, n, 0);
    n = snprintf(line, sizeof(line), "D,M,%u,999,%08X", (unsigned)mega.run,
                 (unsigned)ulink_payload('M', mega.run, 999));
    n += snprintf(line + n, sizeof(line) - n, "*%02X\n", ulink_crc8(line, (uint16_t)n));
    chan_push(&to_m, line, n, 0);
    step(1000);
    assert(mega.st.ack_stale == 1);
    assert(mega.st.bad == 1);
    assert(mega.st.ack_ok > ok_before);
    stop_and_drain();
}

int main(void) {
    test_crc_and_payload();
    test_clean_60s_run();
    test_dropped_data_counts_timeout_and_gap();
    test_dropped_ack_is_timeout_not_gap();
    test_corrupt_line_is_bad_and_times_out();
    test_well_formed_corruption_caught_by_crc();
    test_late_ack_is_stale_not_success();
    test_duplicate_ack_counted_once();
    test_disconnect_and_recover();
    test_stop_while_disconnected_reports_unconfirmed();
    test_start_survives_lost_control_lines();
    test_oversized_line_resyncs();
    test_wrong_run_and_wrong_role_not_counted();
    printf("ulink host tests: all asserts passed\n");
    return 0;
}
