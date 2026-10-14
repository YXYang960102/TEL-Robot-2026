#ifndef RC_CONTROL_MAP_H
#define RC_CONTROL_MAP_H

/* Wireless control channel/event table, shared by:
 *   - the ground STM32 (tools/stm32_ground_control): PC command state ->
 *     CRSF RC channels (rcmap_encode),
 *   - the Mega (src/IO/ManualControl.cpp): RC channels -> the same command
 *     state the wired USB protocol produces (rcmap_decode + counters),
 *   - the status uplink (Mega -> robot bridge -> CRSF Battery frame ->
 *     ground), and
 *   - host tests.
 * Pure C, no libc beyond stdint/stdbool, so it builds for AVR, STM32 and
 * the host unchanged. See docs/codex-handoff.md 2026-10-11 "wireless
 * control" for the design.
 *
 * Channel table (CRSF channel numbers are 1-based, array index = N-1).
 * ELRS's default "Hybrid" switch mode sends CH1-4 at full resolution every
 * packet, CH5 (AUX1) as a 2-position switch every packet, and CH6+ as
 * 6-position values round-robin (~8 packets per refresh), so anything that
 * needs fine resolution lives in CH1-4 and everything in CH6+ only ever
 * uses 3 or 6 well-separated levels.
 *
 *   CH1  left drive      1000..2000 us  = -1..+1
 *   CH2  right drive     1000..2000 us  = -1..+1
 *   CH3  flywheel        1000..2000 us  = 0..1 open-loop ratio (not RPM)
 *   CH4  operator mode   2000 / 1750 / 1500 = FULL_MANUAL / SEMI_AUTO / FULL_AUTO
 *   CH5  arm             1000 = disarmed, 2000 = enable requested
 *   CH6  rotate jog      1000/1500/2000 = -1/0/+1
 *   CH7  angle jog       1000/1500/2000 = -1/0/+1
 *   CH8  F event counter 6 levels, 1000 + 200*k, k = 0..5
 *   CH9  L event counter 6 levels, 1000 + 200*k, k = 0..5
 *   CH10 horizontal conveyor 1000 = off, 2000 = on (state, not toggle)
 *
 * Event counters instead of "button held" levels: one press advances the
 * counter by exactly one level. A receiver fires one action only when it
 * sees +1; repeated packets carry the same level (no repeat), a lost packet
 * is covered by the next one, and after a reconnect the first level seen is
 * taken as the new baseline, so presses made during an outage are never
 * replayed. A jump of more than one level is ignored (EVENT_SKIPPED).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RCMAP_CH_LEFT 0u
#define RCMAP_CH_RIGHT 1u
#define RCMAP_CH_FLYWHEEL 2u
#define RCMAP_CH_MODE 3u
#define RCMAP_CH_ARM 4u
#define RCMAP_CH_ROTATE 5u
#define RCMAP_CH_ANGLE 6u
#define RCMAP_CH_FEED_COUNTER 7u
#define RCMAP_CH_LMODE_COUNTER 8u
#define RCMAP_CH_CONVEYOR 9u
#define RCMAP_CHANNEL_COUNT 10u

#define RCMAP_COUNTER_LEVELS 6u
/* Operator mode on CH4 (a full-resolution channel). Ids match the
 * firmware's OperatorMode enum order. The ground sends exactly the *_US
 * values; the robot decodes with thresholds halfway between them. */
#define RCMAP_MODE_FULL_AUTO 0u
#define RCMAP_MODE_SEMI_AUTO 1u
#define RCMAP_MODE_FULL_MANUAL 2u
#define RCMAP_MODE_FULL_AUTO_US 1500
#define RCMAP_MODE_SEMI_AUTO_US 1750
#define RCMAP_MODE_FULL_MANUAL_US 2000
#define RCMAP_MODE_FULL_AUTO_MIN_US 1250
#define RCMAP_MODE_SEMI_AUTO_MIN_US 1625
#define RCMAP_MODE_FULL_MANUAL_MIN_US 1875

/* Fail-safe: anything that isn't clearly an auto level (including 0 from a
 * zero-initialized command, or a value below 1250 us) reads as manual. */
static inline uint8_t rcmap_mode_from_us(int us) {
    if (us >= RCMAP_MODE_FULL_MANUAL_MIN_US) return RCMAP_MODE_FULL_MANUAL;
    if (us >= RCMAP_MODE_SEMI_AUTO_MIN_US) return RCMAP_MODE_SEMI_AUTO;
    if (us >= RCMAP_MODE_FULL_AUTO_MIN_US) return RCMAP_MODE_FULL_AUTO;
    return RCMAP_MODE_FULL_MANUAL;
}
static inline int rcmap_mode_to_us(uint8_t mode) {
    return mode == RCMAP_MODE_FULL_AUTO ? RCMAP_MODE_FULL_AUTO_US
         : mode == RCMAP_MODE_SEMI_AUTO ? RCMAP_MODE_SEMI_AUTO_US
         : RCMAP_MODE_FULL_MANUAL_US; /* anything unknown = manual */
}

typedef struct {
    float left;            /* -1..1 */
    float right;           /* -1..1 */
    float flywheel;        /* 0..1 */
    bool conveyor;         /* CH10: persistent on/off request */
    bool armed;            /* CH5 high = enable requested */
    int8_t rotate;         /* -1, 0, +1 */
    int8_t angle;          /* -1, 0, +1 */
    uint8_t feed_counter;  /* 0..5 */
    uint8_t lmode_counter; /* 0..5 */
    int mode_us;           /* CH4; encode sends rcmap_mode_to_us(rcmap_mode_from_us(mode_us)) */
} rcmap_command;

/* Fills ch_us[16]; channels beyond RCMAP_CHANNEL_COUNT are set to 1500. */
void rcmap_encode(const rcmap_command *c, int ch_us[16]);

/* Decodes the first RCMAP_CHANNEL_COUNT entries of ch_us. Returns false
 * (out untouched) if any used channel is outside 900..2100 us or a counter
 * channel isn't near one of its 6 levels -- the caller should treat that
 * frame as invalid, not guess. Arm is fail-safe: only >= 1750 us counts as
 * "enable requested". */
bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out);

typedef struct {
    bool have_baseline;
    uint8_t level;
} rcmap_counter;

#define RCMAP_COUNTER_NONE 0
#define RCMAP_COUNTER_EVENT 1
#define RCMAP_COUNTER_SKIPPED (-1)

void rcmap_counter_reset(rcmap_counter *c);
/* Feed one decoded level. Returns RCMAP_COUNTER_EVENT exactly once per +1
 * step, RCMAP_COUNTER_NONE for no change or a fresh baseline, and
 * RCMAP_COUNTER_SKIPPED (baseline moved, no action) for a jump > 1. */
int rcmap_counter_update(rcmap_counter *c, uint8_t level);

/* ---- Status uplink (Mega -> ground) ---------------------------------- */

/* Status flag bits. */
#define RCMAP_STATUS_ARMED 0x01u
#define RCMAP_STATUS_OWNER_WIRELESS 0x02u
#define RCMAP_STATUS_OWNER_WIRED 0x04u
#define RCMAP_STATUS_PINS_CONFIRMED 0x08u
#define RCMAP_STATUS_FEED_BUSY 0x10u
#define RCMAP_STATUS_ROTATE_MANUAL 0x20u
#define RCMAP_STATUS_RC_LINK_OK 0x40u

/* Event codes. Names are exactly the "$EVENT,<name>" strings the wired
 * USB protocol prints, so the ground can forward them to the same
 * Dashboard unchanged. Append only: the numbers travel over the air. */
typedef enum {
    RCMAP_EVENT_NONE = 0,
    RCMAP_EVENT_OUTPUTS_ENABLED,
    RCMAP_EVENT_OUTPUTS_DISABLED,
    RCMAP_EVENT_COMMAND_TIMEOUT,
    RCMAP_EVENT_ENABLE_REJECTED_PINS_UNCONFIRMED,
    RCMAP_EVENT_ENABLE_REJECTED_OTHER_SOURCE,
    RCMAP_EVENT_REJECTED_DISARMED,
    RCMAP_EVENT_REJECTED_NOT_OWNER,
    RCMAP_EVENT_DISABLED_RC_LINK_LOST,
    RCMAP_EVENT_DISABLED_CORRUPT_COMMAND,
    RCMAP_EVENT_FEED_TRIGGER_SENT,
    RCMAP_EVENT_FEED_BUSY,
    RCMAP_EVENT_ROTATE_MANUAL,
    RCMAP_EVENT_ROTATE_VISION_FOLLOW,
    RCMAP_EVENT_BAD_COMMAND,
    RCMAP_EVENT_UNKNOWN_COMMAND,
    RCMAP_EVENT_COMMAND_TOO_LONG,
    RCMAP_EVENT_EVENT_SKIPPED,
    RCMAP_EVENT_CONVEYOR_ON,
    RCMAP_EVENT_CONVEYOR_OFF,
    RCMAP_EVENT_MODE_FULL_MANUAL,
    RCMAP_EVENT_MODE_SEMI_AUTO,
    RCMAP_EVENT_MODE_FULL_AUTO,
    RCMAP_EVENT_FIRE_NOT_READY,
    RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL,
    RCMAP_EVENT_COUNT
} rcmap_event;

const char *rcmap_event_name(uint16_t code);

/* Status packed into the fields of a standard CRSF Battery frame -- a
 * telemetry type ELRS relays from a flight controller as-is. NOT yet
 * verified end to end on this link (the earlier bench may only have shown
 * LINK_STATISTICS frames; see docs/codex-handoff.md). voltage = 0xA5 marker in the high
 * byte + flags in the low byte; current = last event code; capacity =
 * 24-bit event sequence number; remaining = rolling counter from the
 * bridge so the ground can tell fresh status from a repeated stale one. */
#define RCMAP_STATUS_MARKER 0xA5u

/* The 16-bit "event code" field also carries the current operator mode in
 * its top 4 bits, stored as mode + 1 so that 0 means "not reported" (older
 * robot firmware). Event codes stay far below 0x1000. The robot bridge
 * forwards the 16 bits unchanged, so it needed no change for this. */
#define RCMAP_MODE_UNKNOWN 0xFFu
static inline uint16_t rcmap_status_code(uint16_t event, uint8_t mode) {
    return (uint16_t)((event & 0x0FFFu) | ((uint16_t)((mode + 1u) & 0x0Fu) << 12));
}
static inline uint16_t rcmap_status_code_event(uint16_t code) { return code & 0x0FFFu; }
static inline uint8_t rcmap_status_code_mode(uint16_t code) {
    const uint8_t stored = (uint8_t)(code >> 12);
    return stored == 0u ? RCMAP_MODE_UNKNOWN : (uint8_t)(stored - 1u);
}

typedef struct {
    uint8_t flags;
    uint16_t event_code;
    uint32_t event_seq; /* 24-bit */
    uint8_t rolling;
} rcmap_status;

void rcmap_status_to_battery(const rcmap_status *s, uint16_t *voltage, uint16_t *current,
                             uint32_t *capacity, uint8_t *remaining);
/* False if the marker doesn't match (a real battery frame, or garbage). */
bool rcmap_status_from_battery(uint16_t voltage, uint16_t current, uint32_t capacity,
                               uint8_t remaining, rcmap_status *out);

#ifdef __cplusplus
}
#endif

#endif
