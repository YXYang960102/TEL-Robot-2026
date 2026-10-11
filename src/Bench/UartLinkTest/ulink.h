#ifndef ULINK_H
#define ULINK_H

/* Standalone Mega <-> STM32 UART link test protocol (2026-10-11, requested
 * by Jeremy via Codex before any ELRS work). Pure C, no libc, no hardware:
 * compiled unchanged into the Mega bench sketch (MegaUartLinkTest.cpp),
 * the STM32 firmware (tools/stm32_mega_uart_test), and the host test.
 *
 * Wire format: one ASCII line per packet, "<body>*<CC>\n", CC = CRC-8
 * (poly 0x07, init 0) of <body> as two uppercase hex digits.
 *   D,<from>,<run>,<seq>,<payload>   data, sent every ULINK_SEND_PERIOD_MS
 *   A,<from>,<run>,<seq>,<payload>   ACK echoing a received D (from = acker)
 *   C,<run>,<1|0>                    Mega -> STM32: start/stop a run
 *   S,<run>,<running>,<counters...>  STM32 -> Mega status, every 250 ms
 * <from> is M (Mega) or S (STM32). <payload> is 8 hex digits derived from
 * (from, run, seq), so a receiver can verify content, not just framing.
 * An ACK never triggers an ACK. One data packet is outstanding at a time;
 * only an ACK matching the outstanding run+seq+payload counts as success.
 *
 * This 100 ms cadence is the link test's own schedule. It is NOT the ELRS
 * uplink/downlink rate design, which is still a separate, open item. */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ULINK_SEND_PERIOD_MS 100u
#define ULINK_ACK_TIMEOUT_MS 50u
#define ULINK_STATUS_PERIOD_MS 250u
#define ULINK_CONTROL_RESEND_MS 100u
#define ULINK_LINK_DOWN_AFTER_TIMEOUTS 5u
#define ULINK_LINK_UP_AFTER_SUCCESSES 3u
#define ULINK_LINE_MAX 200u

#define ULINK_EVENT_LINK_UP 0x01u
#define ULINK_EVENT_LINK_DOWN 0x02u

typedef void (*ulink_write_fn)(void *ctx, const char *data, uint16_t len);

typedef struct {
    uint32_t tx;           /* data packets this side sent */
    uint32_t ack_ok;       /* ...acked with matching run/seq/payload */
    uint32_t timeout;      /* ...not acked within ULINK_ACK_TIMEOUT_MS */
    uint32_t ack_stale;    /* ACK for a seq that isn't outstanding (late/old run) */
    uint32_t ack_dup;      /* ACK repeating the most recent already-acked seq */
    uint32_t ack_mismatch; /* ACK for the outstanding seq but wrong payload */
    uint32_t rx_data;      /* valid peer data packets in the current run */
    uint32_t rx_gap;       /* peer sequence numbers skipped (lost one way) */
    uint32_t rx_dup;       /* peer data repeated or out of order */
    uint32_t bad;          /* CRC/format/content errors */
    uint32_t overflow;     /* lines longer than ULINK_LINE_MAX, dropped */
    uint32_t rtt_min_ms;
    uint32_t rtt_max_ms;
    uint32_t rtt_sum_ms;
} ulink_stats;

typedef enum {
    ULINK_IDLE = 0,
    ULINK_STARTING, /* Mega: sending C start, waiting for STM32 to confirm */
    ULINK_RUNNING,
    ULINK_STOPPING, /* Mega: stopped sending, draining until both sides idle */
    ULINK_DONE      /* Mega: run finished, final numbers available */
} ulink_run_state;

typedef struct {
    char role; /* 'M' or 'S' */
    ulink_write_fn write;
    void *write_ctx;

    uint16_t run;
    bool sending;
    uint32_t next_seq;
    uint32_t last_send_ms;

    bool outstanding;
    uint32_t out_seq;
    uint32_t out_sent_ms;
    bool have_last_acked;
    uint32_t last_acked_seq;

    bool have_peer_seq;
    uint32_t peer_expected_seq;

    uint8_t consecutive_timeouts;
    uint8_t consecutive_ok;
    bool link_up;
    uint8_t events;

    ulink_stats st;

    char line[ULINK_LINE_MAX];
    uint16_t len;
    bool discarding;

    /* STM32 role */
    uint32_t last_status_ms;

    /* Mega role */
    ulink_run_state state;
    uint32_t state_since_ms;
    uint32_t last_control_ms;
    bool peer_status_seen;
    uint32_t peer_status_ms;
    uint16_t peer_run;
    bool peer_running;
    bool peer_outstanding;
    ulink_stats peer;
    bool drain_confirmed; /* STOPPING ended with the STM32's confirmed final status */
} ulink_endpoint;

void ulink_init(ulink_endpoint *ep, char role, ulink_write_fn write, void *ctx, uint32_t now_ms);
void ulink_rx_byte(ulink_endpoint *ep, char c, uint32_t now_ms);
void ulink_poll(ulink_endpoint *ep, uint32_t now_ms);

/* Mega role only. start: new run id (must differ from the last one). */
void ulink_master_start(ulink_endpoint *ep, uint16_t run, uint32_t now_ms);
void ulink_master_stop(ulink_endpoint *ep, uint32_t now_ms);

/* Returns and clears ULINK_EVENT_* flags raised since the last call. */
uint8_t ulink_take_events(ulink_endpoint *ep);

/* Exposed for tests. */
uint8_t ulink_crc8(const char *data, uint16_t len);
uint32_t ulink_payload(char from, uint16_t run, uint32_t seq);

/* How long STOPPING waits for the STM32 to confirm before giving up. */
#define ULINK_DRAIN_TIMEOUT_MS 2000u

#ifdef __cplusplus
}
#endif

#endif
