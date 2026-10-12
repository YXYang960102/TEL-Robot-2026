#include "ulink.h"

/* No libc anywhere in here (the STM32 build is -nostdlib): formatting,
 * parsing and zeroing are all done by hand. See ulink.h for the protocol. */

#define ULINK_MAX_FIELDS 20u
#define ULINK_STATUS_FIELDS 18u /* "S" + 17 values */

typedef struct {
    char buf[ULINK_LINE_MAX];
    uint16_t len;
} line_builder;

static void lb_char(line_builder *lb, char c) {
    if (lb->len < ULINK_LINE_MAX) lb->buf[lb->len++] = c;
}

static void lb_u32(line_builder *lb, uint32_t v) {
    char tmp[10];
    uint8_t n = 0;
    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u && n < sizeof(tmp));
    while (n > 0u) lb_char(lb, tmp[--n]);
}

static char hex_digit(uint8_t v) {
    return (char)(v < 10u ? (uint8_t)('0' + v) : (uint8_t)('A' + (v - 10u)));
}

static void lb_hex8(line_builder *lb, uint32_t v) {
    for (int shift = 28; shift >= 0; shift -= 4) {
        lb_char(lb, hex_digit((uint8_t)((v >> shift) & 0xFu)));
    }
}

uint8_t ulink_crc8(const char *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (uint8_t b = 0; b < 8u; b++) {
            crc = (uint8_t)((crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1));
        }
    }
    return crc;
}

uint32_t ulink_payload(char from, uint16_t run, uint32_t seq) {
    uint32_t h = seq * 2654435761u;
    h ^= ((uint32_t)run << 16) | (uint32_t)run;
    h ^= (from == 'M') ? 0xA5C3E10Fu : 0x5A3C1EF0u;
    h ^= h >> 15;
    h *= 0x2C1B3C6Du;
    h ^= h >> 12;
    return h;
}

static void finish_and_send(ulink_endpoint *ep, line_builder *lb) {
    const uint8_t crc = ulink_crc8(lb->buf, lb->len);
    lb_char(lb, '*');
    lb_char(lb, hex_digit((uint8_t)(crc >> 4)));
    lb_char(lb, hex_digit((uint8_t)(crc & 0xFu)));
    lb_char(lb, '\n');
    if (ep->write) ep->write(ep->write_ctx, lb->buf, lb->len);
}

static void zero_stats(ulink_stats *s) {
    s->tx = s->ack_ok = s->timeout = s->ack_stale = s->ack_dup = s->ack_mismatch = 0;
    s->rx_data = s->rx_gap = s->rx_dup = s->bad = s->overflow = 0;
    s->rtt_min_ms = s->rtt_max_ms = s->rtt_sum_ms = 0;
}

static void reset_run_tracking(ulink_endpoint *ep) {
    ep->next_seq = 0;
    ep->outstanding = false;
    ep->have_last_acked = false;
    ep->have_peer_seq = false;
    ep->peer_expected_seq = 0;
    zero_stats(&ep->st);
}

void ulink_init(ulink_endpoint *ep, char role, ulink_write_fn write, void *ctx, uint32_t now_ms) {
    ep->role = role;
    ep->write = write;
    ep->write_ctx = ctx;
    ep->run = 0;
    ep->sending = false;
    ep->last_send_ms = now_ms;
    ep->out_seq = 0;
    ep->out_sent_ms = now_ms;
    ep->last_acked_seq = 0;
    ep->consecutive_timeouts = 0;
    ep->consecutive_ok = 0;
    ep->link_up = false;
    ep->events = 0;
    ep->len = 0;
    ep->discarding = false;
    ep->last_status_ms = now_ms;
    ep->state = ULINK_IDLE;
    ep->state_since_ms = now_ms;
    ep->last_control_ms = now_ms;
    ep->peer_status_seen = false;
    ep->peer_status_ms = now_ms;
    ep->peer_run = 0;
    ep->peer_running = false;
    ep->peer_outstanding = false;
    ep->drain_confirmed = false;
    zero_stats(&ep->peer);
    reset_run_tracking(ep);
}

uint8_t ulink_take_events(ulink_endpoint *ep) {
    const uint8_t e = ep->events;
    ep->events = 0;
    return e;
}

/* ---------------------------------------------------------------- send */

static void send_data_like(ulink_endpoint *ep, char type, char from, uint16_t run,
                           uint32_t seq, uint32_t payload) {
    line_builder lb;
    lb.len = 0;
    lb_char(&lb, type);
    lb_char(&lb, ',');
    lb_char(&lb, from);
    lb_char(&lb, ',');
    lb_u32(&lb, run);
    lb_char(&lb, ',');
    lb_u32(&lb, seq);
    lb_char(&lb, ',');
    lb_hex8(&lb, payload);
    finish_and_send(ep, &lb);
}

static void send_control(ulink_endpoint *ep, uint16_t run, bool start) {
    line_builder lb;
    lb.len = 0;
    lb_char(&lb, 'C');
    lb_char(&lb, ',');
    lb_u32(&lb, run);
    lb_char(&lb, ',');
    lb_char(&lb, start ? '1' : '0');
    finish_and_send(ep, &lb);
}

static void send_status(ulink_endpoint *ep) {
    const ulink_stats *s = &ep->st;
    const uint32_t values[17] = {
        ep->run, ep->sending ? 1u : 0u, s->tx, s->ack_ok, s->timeout, s->ack_stale,
        s->ack_dup, s->ack_mismatch, s->rx_data, s->rx_gap, s->rx_dup, s->bad,
        s->overflow, s->rtt_min_ms, s->rtt_max_ms, s->rtt_sum_ms, ep->outstanding ? 1u : 0u,
    };
    line_builder lb;
    lb.len = 0;
    lb_char(&lb, 'S');
    for (uint8_t i = 0; i < 17u; i++) {
        lb_char(&lb, ',');
        lb_u32(&lb, values[i]);
    }
    finish_and_send(ep, &lb);
}

/* --------------------------------------------------------------- parse */

typedef struct {
    const char *p;
    uint16_t n;
} field;

static bool parse_u32(field f, uint32_t *out) {
    if (f.n == 0u || f.n > 10u) return false;
    uint32_t v = 0;
    for (uint16_t i = 0; i < f.n; i++) {
        const char c = f.p[i];
        if (c < '0' || c > '9') return false;
        const uint32_t d = (uint32_t)(c - '0');
        if (v > (0xFFFFFFFFu - d) / 10u) return false;
        v = v * 10u + d;
    }
    *out = v;
    return true;
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex8(field f, uint32_t *out) {
    if (f.n != 8u) return false;
    uint32_t v = 0;
    for (uint16_t i = 0; i < 8u; i++) {
        const int h = hex_value(f.p[i]);
        if (h < 0) return false;
        v = (v << 4) | (uint32_t)h;
    }
    *out = v;
    return true;
}

static bool parse_role(field f, char *out) {
    if (f.n != 1u || (f.p[0] != 'M' && f.p[0] != 'S')) return false;
    *out = f.p[0];
    return true;
}

static bool parse_run(field f, uint16_t *out) {
    uint32_t v;
    if (!parse_u32(f, &v) || v > 0xFFFFu) return false;
    *out = (uint16_t)v;
    return true;
}

static void note_success(ulink_endpoint *ep) {
    ep->consecutive_timeouts = 0;
    if (ep->consecutive_ok < 255u) ep->consecutive_ok++;
    if (!ep->link_up && ep->consecutive_ok >= ULINK_LINK_UP_AFTER_SUCCESSES) {
        ep->link_up = true;
        ep->events |= ULINK_EVENT_LINK_UP;
    }
}

static void note_timeout(ulink_endpoint *ep) {
    ep->consecutive_ok = 0;
    if (ep->consecutive_timeouts < 255u) ep->consecutive_timeouts++;
    if (ep->link_up && ep->consecutive_timeouts >= ULINK_LINK_DOWN_AFTER_TIMEOUTS) {
        ep->link_up = false;
        ep->events |= ULINK_EVENT_LINK_DOWN;
    }
}

static bool counting_peer_data(const ulink_endpoint *ep, uint16_t run) {
    if (ep->run == 0u || run != ep->run) return false;
    if (ep->role == 'S') return true;
    return ep->state == ULINK_STARTING || ep->state == ULINK_RUNNING ||
           ep->state == ULINK_STOPPING;
}

static void handle_data(ulink_endpoint *ep, const field *f, uint8_t nf) {
    char from;
    uint16_t run;
    uint32_t seq, payload;
    if (nf != 5u || !parse_role(f[1], &from) || from == ep->role ||
        !parse_run(f[2], &run) || !parse_u32(f[3], &seq) || !parse_hex8(f[4], &payload) ||
        payload != ulink_payload(from, run, seq)) {
        ep->st.bad++;
        return;
    }

    /* Echo it back. The sender decides whether this ACK matches what it
     * has outstanding; ACKs themselves are never acknowledged. */
    send_data_like(ep, 'A', ep->role, run, seq, payload);

    if (!counting_peer_data(ep, run)) return;
    if (!ep->have_peer_seq) {
        ep->have_peer_seq = true;
        ep->st.rx_gap += seq; /* each run's first seq is 0 */
        ep->st.rx_data++;
        ep->peer_expected_seq = seq + 1u;
    } else if (seq == ep->peer_expected_seq) {
        ep->st.rx_data++;
        ep->peer_expected_seq++;
    } else if (seq > ep->peer_expected_seq) {
        ep->st.rx_gap += seq - ep->peer_expected_seq;
        ep->st.rx_data++;
        ep->peer_expected_seq = seq + 1u;
    } else {
        ep->st.rx_dup++;
    }
}

static void handle_ack(ulink_endpoint *ep, const field *f, uint8_t nf, uint32_t now_ms) {
    char from;
    uint16_t run;
    uint32_t seq, payload;
    if (nf != 5u || !parse_role(f[1], &from) || from == ep->role ||
        !parse_run(f[2], &run) || !parse_u32(f[3], &seq) || !parse_hex8(f[4], &payload)) {
        ep->st.bad++;
        return;
    }

    if (ep->outstanding && run == ep->run && seq == ep->out_seq) {
        if (payload != ulink_payload(ep->role, run, seq)) {
            ep->st.ack_mismatch++;
            return; /* still outstanding; will time out unless a good ACK arrives */
        }
        const uint32_t rtt = now_ms - ep->out_sent_ms;
        ep->outstanding = false;
        ep->have_last_acked = true;
        ep->last_acked_seq = seq;
        ep->st.ack_ok++;
        if (ep->st.ack_ok == 1u || rtt < ep->st.rtt_min_ms) ep->st.rtt_min_ms = rtt;
        if (rtt > ep->st.rtt_max_ms) ep->st.rtt_max_ms = rtt;
        ep->st.rtt_sum_ms += rtt;
        note_success(ep);
        return;
    }

    if (ep->have_last_acked && run == ep->run && seq == ep->last_acked_seq) {
        ep->st.ack_dup++;
    } else {
        ep->st.ack_stale++;
    }
}

static void handle_control(ulink_endpoint *ep, const field *f, uint8_t nf, uint32_t now_ms) {
    uint16_t run;
    if (ep->role != 'S' || nf != 3u || !parse_run(f[1], &run) || run == 0u ||
        f[2].n != 1u || (f[2].p[0] != '0' && f[2].p[0] != '1')) {
        ep->st.bad++;
        return;
    }
    if (f[2].p[0] == '1') {
        if (!(ep->sending && ep->run == run)) {
            ep->run = run;
            reset_run_tracking(ep);
            ep->sending = true;
            ep->last_send_ms = now_ms - ULINK_SEND_PERIOD_MS; /* first D right away */
        }
    } else {
        ep->sending = false; /* any outstanding D still resolves (ACK or timeout) */
    }
    send_status(ep);
    ep->last_status_ms = now_ms;
}

static void handle_status(ulink_endpoint *ep, const field *f, uint8_t nf, uint32_t now_ms) {
    uint32_t v[17];
    if (ep->role != 'M' || nf != ULINK_STATUS_FIELDS) {
        ep->st.bad++;
        return;
    }
    for (uint8_t i = 0; i < 17u; i++) {
        if (!parse_u32(f[i + 1u], &v[i])) {
            ep->st.bad++;
            return;
        }
    }
    if (v[0] > 0xFFFFu || v[1] > 1u || v[16] > 1u) {
        ep->st.bad++;
        return;
    }
    ep->peer_run = (uint16_t)v[0];
    ep->peer_running = v[1] == 1u;
    ep->peer.tx = v[2];
    ep->peer.ack_ok = v[3];
    ep->peer.timeout = v[4];
    ep->peer.ack_stale = v[5];
    ep->peer.ack_dup = v[6];
    ep->peer.ack_mismatch = v[7];
    ep->peer.rx_data = v[8];
    ep->peer.rx_gap = v[9];
    ep->peer.rx_dup = v[10];
    ep->peer.bad = v[11];
    ep->peer.overflow = v[12];
    ep->peer.rtt_min_ms = v[13];
    ep->peer.rtt_max_ms = v[14];
    ep->peer.rtt_sum_ms = v[15];
    ep->peer_outstanding = v[16] == 1u;
    ep->peer_status_seen = true;
    ep->peer_status_ms = now_ms;
}

static void handle_line(ulink_endpoint *ep, uint32_t now_ms) {
    if (ep->len == 0u) return; /* stray newline */

    /* "<body>*<CC>": the checksum separator must be exactly 3 from the end. */
    if (ep->len < 4u || ep->line[ep->len - 3u] != '*') {
        ep->st.bad++;
        return;
    }
    const int hi = hex_value(ep->line[ep->len - 2u]);
    const int lo = hex_value(ep->line[ep->len - 1u]);
    const uint16_t body_len = (uint16_t)(ep->len - 3u);
    if (hi < 0 || lo < 0 || (uint8_t)((hi << 4) | lo) != ulink_crc8(ep->line, body_len)) {
        ep->st.bad++;
        return;
    }

    field f[ULINK_MAX_FIELDS];
    uint8_t nf = 0;
    uint16_t start = 0;
    for (uint16_t i = 0; i <= body_len; i++) {
        if (i == body_len || ep->line[i] == ',') {
            if (nf == ULINK_MAX_FIELDS) {
                ep->st.bad++;
                return;
            }
            f[nf].p = &ep->line[start];
            f[nf].n = (uint16_t)(i - start);
            nf++;
            start = (uint16_t)(i + 1u);
        }
    }
    if (f[0].n != 1u) {
        ep->st.bad++;
        return;
    }
    switch (f[0].p[0]) {
        case 'D': handle_data(ep, f, nf); break;
        case 'A': handle_ack(ep, f, nf, now_ms); break;
        case 'C': handle_control(ep, f, nf, now_ms); break;
        case 'S': handle_status(ep, f, nf, now_ms); break;
        default: ep->st.bad++; break;
    }
}

void ulink_rx_byte(ulink_endpoint *ep, char c, uint32_t now_ms) {
    if (c == '\r') return;
    if (c == '\n') {
        if (ep->discarding) {
            ep->discarding = false; /* end of an oversized line: drop it whole */
        } else {
            handle_line(ep, now_ms);
        }
        ep->len = 0;
        return;
    }
    if (ep->discarding) return;
    if (ep->len >= ULINK_LINE_MAX) {
        ep->st.overflow++;
        ep->discarding = true;
        ep->len = 0;
        return;
    }
    ep->line[ep->len++] = c;
}

/* ---------------------------------------------------------------- poll */

void ulink_poll(ulink_endpoint *ep, uint32_t now_ms) {
    if (ep->outstanding && now_ms - ep->out_sent_ms >= ULINK_ACK_TIMEOUT_MS) {
        ep->outstanding = false;
        ep->st.timeout++;
        note_timeout(ep);
    }

    if (ep->sending && !ep->outstanding &&
        now_ms - ep->last_send_ms >= ULINK_SEND_PERIOD_MS) {
        const uint32_t seq = ep->next_seq++;
        send_data_like(ep, 'D', ep->role, ep->run, seq, ulink_payload(ep->role, ep->run, seq));
        ep->st.tx++;
        ep->outstanding = true;
        ep->out_seq = seq;
        ep->out_sent_ms = now_ms;
        ep->last_send_ms = now_ms;
    }

    if (ep->role == 'S') {
        if (now_ms - ep->last_status_ms >= ULINK_STATUS_PERIOD_MS) {
            ep->last_status_ms = now_ms;
            send_status(ep);
        }
        return;
    }

    if (ep->state == ULINK_STARTING) {
        if (ep->peer_status_seen && ep->peer_status_ms - ep->state_since_ms < 0x80000000u &&
            ep->peer_run == ep->run && ep->peer_running) {
            ep->state = ULINK_RUNNING;
            ep->state_since_ms = now_ms;
            ep->sending = true;
            ep->last_send_ms = now_ms - ULINK_SEND_PERIOD_MS;
        } else if (now_ms - ep->last_control_ms >= ULINK_CONTROL_RESEND_MS) {
            ep->last_control_ms = now_ms;
            send_control(ep, ep->run, true);
        }
    } else if (ep->state == ULINK_STOPPING) {
        const bool peer_idle = ep->peer_status_seen &&
            ep->peer_status_ms - ep->state_since_ms < 0x80000000u &&
            ep->peer_run == ep->run && !ep->peer_running && !ep->peer_outstanding;
        if (!ep->outstanding && peer_idle) {
            ep->state = ULINK_DONE;
            ep->state_since_ms = now_ms;
            ep->drain_confirmed = true;
        } else if (now_ms - ep->state_since_ms >= ULINK_DRAIN_TIMEOUT_MS) {
            ep->state = ULINK_DONE;
            ep->state_since_ms = now_ms;
            ep->drain_confirmed = false;
        } else if (now_ms - ep->last_control_ms >= ULINK_CONTROL_RESEND_MS) {
            ep->last_control_ms = now_ms;
            send_control(ep, ep->run, false);
        }
    }
}

void ulink_master_start(ulink_endpoint *ep, uint16_t run, uint32_t now_ms) {
    ep->run = run;
    reset_run_tracking(ep);
    ep->sending = false; /* data starts once the STM32 confirms the run */
    ep->state = ULINK_STARTING;
    ep->state_since_ms = now_ms;
    ep->last_control_ms = now_ms - ULINK_CONTROL_RESEND_MS;
    ep->drain_confirmed = false;
}

void ulink_master_stop(ulink_endpoint *ep, uint32_t now_ms) {
    if (ep->state != ULINK_STARTING && ep->state != ULINK_RUNNING) return;
    ep->sending = false;
    ep->state = ULINK_STOPPING;
    ep->state_since_ms = now_ms;
    ep->last_control_ms = now_ms - ULINK_CONTROL_RESEND_MS;
}
