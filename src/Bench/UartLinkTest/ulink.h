#ifndef ULINK_H
#define ULINK_H

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
    uint32_t tx;
    uint32_t ack_ok;
    uint32_t timeout;
    uint32_t ack_stale;
    uint32_t ack_dup;
    uint32_t ack_mismatch;
    uint32_t rx_data;
    uint32_t rx_gap;
    uint32_t rx_dup;
    uint32_t bad;
    uint32_t overflow;
    uint32_t rtt_min_ms;
    uint32_t rtt_max_ms;
    uint32_t rtt_sum_ms;
} ulink_stats;

typedef enum {
    ULINK_IDLE = 0,
    ULINK_STARTING,
    ULINK_RUNNING,
    ULINK_STOPPING,
    ULINK_DONE
} ulink_run_state;

typedef struct {
    char role;
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

    uint32_t last_status_ms;

    ulink_run_state state;
    uint32_t state_since_ms;
    uint32_t last_control_ms;
    bool peer_status_seen;
    uint32_t peer_status_ms;
    uint16_t peer_run;
    bool peer_running;
    bool peer_outstanding;
    ulink_stats peer;
    bool drain_confirmed;
} ulink_endpoint;

void ulink_init(ulink_endpoint *ep, char role, ulink_write_fn write, void *ctx, uint32_t now_ms);
void ulink_rx_byte(ulink_endpoint *ep, char c, uint32_t now_ms);
void ulink_poll(ulink_endpoint *ep, uint32_t now_ms);

void ulink_master_start(ulink_endpoint *ep, uint16_t run, uint32_t now_ms);
void ulink_master_stop(ulink_endpoint *ep, uint32_t now_ms);

uint8_t ulink_take_events(ulink_endpoint *ep);

uint8_t ulink_crc8(const char *data, uint16_t len);
uint32_t ulink_payload(char from, uint16_t run, uint32_t seq);

#define ULINK_DRAIN_TIMEOUT_MS 2000u

#ifdef __cplusplus
}
#endif

#endif
