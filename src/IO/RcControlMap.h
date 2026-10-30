#ifndef RC_CONTROL_MAP_H
#define RC_CONTROL_MAP_H

/* Wireless control channel/event table, shared by:
 *   - the ground STM32 (tools/stm32_ground_control): PC command state ->
 *     CRSF RC channels (rcmap_encode),
 *   - the Mega (src/IO/ManualControl.cpp): RC channels -> the same command
 *     state the wired USB protocol produces (rcmap_decode),
 *   - the status uplink (Mega -> robot bridge -> CRSF Battery frame ->
 *     ground), and
 *   - host tests.
 * Pure C, no libc beyond stdint/stdbool, so it builds for AVR, STM32 and
 * the host unchanged.
 *
 * Channel table v2 (2026-10-16, the team's competition control table; CRSF
 * channel numbers are 1-based, array index = N-1). ELRS's "Hybrid" switch
 * mode sends CH1-4 at full resolution every packet, CH5 (AUX1) as a
 * 2-position switch every packet, and CH6+ as 6-position values
 * round-robin, so anything with more than 6 levels lives in CH1-4 and the
 * fire button gets CH5 (every packet, so a 100 ms press is never missed).
 *
 *   CH1  forward/back    1000..2000 us = -1..+1 (continuous)
 *   CH2  turn            1000..2000 us = -1..+1 (continuous; +1 = right)
 *   CH3  target hole     1000 + 80*k, k = 0..12 (0 = none, 1..9 = the
 *                        3x3 holes: 1 left-bottom .. 9 right-top,
 *                        10..12 = the bonus holes left / middle / right,
 *                        keys Q W E; 2026-10-27, was 100*k for 0..9)
 *   CH4  operator mode + Falcon (2026-10-17):
 *          2000                FULL_MANUAL, flywheel 0
 *          1750                SEMI_AUTO
 *          1500                FULL_AUTO
 *          1000 + 300*f        FULL_MANUAL, flywheel f (0 < f <= 1; 3 us = 1%)
 *        In the autos the level carries the operator's flywheel offset
 *        (2026-10-18, LT / RT): level + 4 us per 1%, -20..+20%, so semi
 *        auto spans 1670..1830 and full auto 1420..1580, inside their
 *        bands. Exactly 1750 / 1500 = offset 0 (what an older ground sends).
 *        The flywheel shares CH4 because only CH1-4 are full resolution
 *        and it only matters in full manual (the autos run it from the
 *        shot table). 1300..1399 is a gap: a frame there is rejected. The
 *        three old levels are unchanged, so a ground STM32 still on the
 *        previous image can never make the robot spin the flywheel.
 *   CH5  fire            1000 = released, 2000 = pressed (the ground holds
 *                        it 100 ms per press; the robot acts on low->high)
 *   CH6  arm             1000 = disarmed, 2000 = enable requested
 *   CH7  nudge           1000 + 200*k, k = 0..4: center / up / down /
 *                        left / right (elevation up/down, turret left/right)
 *   CH8  auto path       1000 + 200*k, k = 0..4: none / 1 / 2 / 3 / 4
 *                        (keys Q W E R = left 1, left 2, right 1, right 2)
 *   CH9  test mode       1500 = off, 2000 = TEST (2026-10-26): the
 *                        operator mode is TEST only while CH9 is high AND
 *                        CH4 carries full manual (with its flywheel), so
 *                        test mode reuses full manual's CH4 Falcon
 *                        encoding. Any other CH4 level wins over CH9 (a
 *                        round-robin channel lags CH4 by tens of ms; a mode
 *                        change never waits for it). A ground still on the
 *                        older image sends 1500 here: never TEST.
 *   CH10 link version    1000 + 200*v (2026-10-27: v = RCMAP_LINK_VERSION,
 *                        1 -> 1200; ELRS's 6-position AUX sends 1193). The
 *                        robot rejects every frame whose CH10 isn't this
 *                        version's level: a ground on another channel
 *                        table (an older one sends 1500 here) can then
 *                        never enable or steer the robot, instead of
 *                        having its levels read as the wrong hole / mode.
 *                        Bump the version whenever a channel's meaning or
 *                        levels change.
 *
 * The level channels decode to the nearest level within a tolerance and a
 * frame with any channel between levels is rejected as corrupt, not
 * guessed. RCMAP_CHANNEL_COUNT stays 10 so the robot bridge's "RC,..."
 * line (and its firmware image) is unchanged.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RCMAP_CH_FORWARD 0u
#define RCMAP_CH_TURN 1u
#define RCMAP_CH_TARGET 2u
#define RCMAP_CH_MODE 3u
#define RCMAP_CH_FIRE 4u
#define RCMAP_CH_ARM 5u
#define RCMAP_CH_NUDGE 6u
#define RCMAP_CH_AUTO 7u
#define RCMAP_CH_TEST 8u
#define RCMAP_CH_VERSION 9u
/* 1 = table v2 + CH3 80 us steps (bonus holes) + CH9 test (2026-10-27).
 * 0..5 only (CH10 is a 6-position AUX channel). */
#define RCMAP_LINK_VERSION 1u
#define RCMAP_CHANNEL_COUNT 10u

#define RCMAP_TARGET_NONE 0u
#define RCMAP_TARGET_MAX 12u /* 10..12 = bonus holes (OperatorTarget.h) */

#define RCMAP_NUDGE_CENTER 0u
#define RCMAP_NUDGE_UP 1u
#define RCMAP_NUDGE_DOWN 2u
#define RCMAP_NUDGE_LEFT 3u
#define RCMAP_NUDGE_RIGHT 4u

#define RCMAP_AUTO_NONE 0u
#define RCMAP_AUTO_PATHS 4u /* 1..4 */

/* Operator mode on CH4 (a full-resolution channel). Ids match the
 * firmware's OperatorMode enum order. The ground sends exactly the *_US
 * values; the robot decodes with thresholds halfway between them. */
#define RCMAP_MODE_FULL_AUTO 0u
#define RCMAP_MODE_SEMI_AUTO 1u
#define RCMAP_MODE_FULL_MANUAL 2u
/* TEST (2026-10-26): the turret follows the camera, everything else
 * (elevation, flywheel, fire) is the operator's, as in full manual. On the
 * radio: CH4 full manual + CH9 high (no CH4 level of its own). */
#define RCMAP_MODE_TEST 3u
#define RCMAP_MODE_FULL_AUTO_US 1500
#define RCMAP_MODE_SEMI_AUTO_US 1750
#define RCMAP_MODE_FULL_MANUAL_US 2000
#define RCMAP_MODE_FULL_AUTO_MIN_US 1400
#define RCMAP_MODE_SEMI_AUTO_MIN_US 1625
#define RCMAP_MODE_FULL_MANUAL_MIN_US 1875
/* Full manual with the flywheel on: 1000 us = 0 .. 1300 us = 1.0. */
#define RCMAP_FLYWHEEL_BASE_US 1000
#define RCMAP_FLYWHEEL_SPAN_US 300
#define RCMAP_FLYWHEEL_MAX_US (RCMAP_FLYWHEEL_BASE_US + RCMAP_FLYWHEEL_SPAN_US)
/* Auto modes: the shot table's flywheel + this offset (percent). */
#define RCMAP_AUTO_OFFSET_US_PER_PCT 4
#define RCMAP_AUTO_OFFSET_MAX_PCT 20

/* Fail-safe: anything that isn't clearly an auto level (including 0 from a
 * zero-initialized command, the flywheel range, or the gap below 1400 us)
 * reads as manual. rcmap_decode is stricter: it rejects the gap. */
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
    float forward;     /* -1..1 */
    float turn;        /* -1..1, +1 = right */
    uint8_t target;    /* 0 = none, 1..9 */
    int mode_us;       /* CH4 mode; encode uses rcmap_mode_from_us(mode_us),
                          decode returns the mode's exact level */
    float flywheel;    /* CH4 too: 0..1, sent only in full manual; decode
                          gives 0 outside full manual, in 1% steps */
    int8_t flywheel_offset_pct; /* CH4 too: -20..20, sent only in the autos;
                                   decode gives 0 in full manual */
    bool test_mode;    /* CH9: with CH4 in full manual = TEST mode; encode
                          sends it only then, decode gives false otherwise */
    bool fire;         /* CH5 high = fire pressed */
    bool armed;        /* CH6 high = enable requested */
    uint8_t nudge;     /* RCMAP_NUDGE_* */
    uint8_t auto_path; /* 0 = none, 1..4 */
} rcmap_command;

/* The operator mode a command carries: CH4's level, or TEST when CH4 is
 * full manual and the test flag is set. */
static inline uint8_t rcmap_command_mode(const rcmap_command *c) {
    const uint8_t mode = rcmap_mode_from_us(c->mode_us);
    return mode == RCMAP_MODE_FULL_MANUAL && c->test_mode ? RCMAP_MODE_TEST : mode;
}

/* Fills ch_us[16]; channels beyond the table are set to 1500. Out-of-range
 * fields are clamped (axes) or sent as their neutral level. */
void rcmap_encode(const rcmap_command *c, int ch_us[16]);

/* Decodes the first RCMAP_CHANNEL_COUNT entries of ch_us. Returns false
 * (out untouched) if any channel is outside 900..2100 us, a level channel
 * isn't near one of its levels, or CH10 isn't RCMAP_LINK_VERSION -- the
 * caller should treat that frame as invalid, not guess. Fire and arm are
 * fail-safe: only >= 1750 us counts as pressed / enable requested. */
bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out);
/* CH10 carries this table's version (else the sender is on another one). */
bool rcmap_link_version_ok(const int *ch_us, uint8_t count);

/* ---- Mega <-> robot bridge text lines (2026-10-27) ------------------- */

/* The "RC,..." and "ST,..." lines on the Mega <-> robot bridge UART end in
 * "*HH": CRC-8/DVB-S2 (poly 0xD5, init 0, the one CRSF uses) of every byte
 * before the '*', as two upper-case hex digits. A plain digit can flip into
 * another digit on that wire (seen 2026-10-26: bogus armed/owner flags and
 * a jumping elevation readback) and still parse, so the receiver drops any
 * line whose check doesn't match instead. Both ends must run this. */
#define RCMAP_LINE_CHECK_CHARS 3u /* "*HH" */
uint8_t rcmap_line_crc8(const char *text, uint16_t len);
/* Appends "*HH" for text[0..len) at buf[len]; returns the new length, or 0
 * (buf untouched) if fewer than RCMAP_LINE_CHECK_CHARS bytes are free. */
uint16_t rcmap_line_seal(char *buf, uint16_t len, uint16_t buf_size);
/* A received line without its '\r' / '\n': the length of the text before
 * a matching "*HH", or -1 when the check is missing, malformed (lower-case
 * hex included) or wrong. */
int rcmap_line_open(const char *line, uint16_t len);

/* ---- Status uplink (Mega -> ground) ---------------------------------- */

/* Status flag bits. */
#define RCMAP_STATUS_ARMED 0x01u
#define RCMAP_STATUS_OWNER_WIRELESS 0x02u
#define RCMAP_STATUS_OWNER_WIRED 0x04u
#define RCMAP_STATUS_PINS_CONFIRMED 0x08u
#define RCMAP_STATUS_FEED_BUSY 0x10u
#define RCMAP_STATUS_ROTATE_MANUAL 0x20u
#define RCMAP_STATUS_RC_LINK_OK 0x40u
/* Operator mode is TEST (2026-10-26). The status code below has room for
 * modes 0-2 only (mode + 1 = 4 would set bit 14, the vision-frame marker),
 * so TEST travels as FULL_MANUAL in the code plus this flag: an older
 * ground just shows full manual. Read it back with rcmap_status_mode(). */
#define RCMAP_STATUS_TEST_MODE 0x80u

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
    RCMAP_EVENT_EVENT_SKIPPED,  /* retired with the v1 counters; number kept */
    RCMAP_EVENT_CONVEYOR_ON,    /* retired: the tower drives the conveyor */
    RCMAP_EVENT_CONVEYOR_OFF,   /* retired */
    RCMAP_EVENT_MODE_FULL_MANUAL,
    RCMAP_EVENT_MODE_SEMI_AUTO,
    RCMAP_EVENT_MODE_FULL_AUTO,
    RCMAP_EVENT_FIRE_NOT_READY,
    RCMAP_EVENT_ENABLE_REJECTED_NOT_MANUAL,
    RCMAP_EVENT_REJECTED_AUTO_MODE,
    RCMAP_EVENT_TARGET_SELECTED,
    RCMAP_EVENT_AUTO_PATH_STARTED,
    RCMAP_EVENT_AUTO_PATH_REJECTED,
    RCMAP_EVENT_MODE_TEST,      /* 2026-10-26: a ground older than this drops
                                   such a status as corrupt -- reflash both */
    RCMAP_EVENT_LINK_VERSION_MISMATCH, /* 2026-10-27: RC frames from a ground
                                          on another channel table (CH10) */
    RCMAP_EVENT_TURRET_ZERO_SET,       /* 2026-10-31: home point set (TZERO) */
    RCMAP_EVENT_TURRET_ZERO_REJECTED,  /* armed, or no healthy AS5600 reading */
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
    /* TEST is sent as FULL_MANUAL + RCMAP_STATUS_TEST_MODE (see there);
       anything else above FULL_MANUAL as FULL_MANUAL too, never into the
       aim / vision marker bits. */
    const uint8_t m = mode > RCMAP_MODE_FULL_MANUAL ? RCMAP_MODE_FULL_MANUAL : mode;
    return (uint16_t)((event & 0x0FFFu) | ((uint16_t)((m + 1u) & 0x03u) << 12));
}
static inline uint16_t rcmap_status_code_event(uint16_t code) { return code & 0x0FFFu; }
static inline uint8_t rcmap_status_code_mode(uint16_t code) {
    const uint8_t stored = (uint8_t)(code >> 12);
    return stored == 0u ? RCMAP_MODE_UNKNOWN : (uint8_t)(stored - 1u);
}
/* The full operator mode of a status frame: the code's, or TEST. */
static inline uint8_t rcmap_status_mode(uint16_t code, uint8_t flags) {
    const uint8_t mode = rcmap_status_code_mode(code);
    return mode == RCMAP_MODE_FULL_MANUAL && (flags & RCMAP_STATUS_TEST_MODE) ? RCMAP_MODE_TEST : mode;
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

/* Aim frames (2026-10-17): the Mega alternates its 100 ms status slot
 * between the status above and an aim frame, so the wireless Dashboard can
 * show the camera distance and the elevation pulse. Same "ST,<flags>,
 * <seq>,<code>" line and Battery frame -- the robot bridge forwards it
 * unchanged (its field ranges: flags <= 255, seq <= 0xFFFFFF, code <=
 * 65535), so the bridge firmware needed no change.
 *   code  bits 15-14 = 10 (aim frame; a status code is at most 0x3FFF),
 *         bits 0-13 = camera distance in cm (0 = none, max 16383; 15
 *         bits until 2026-10-27, when bits 15-14 = 11 became the seen
 *         frame below)
 *   seq   bits 0-11 elevation pulse us, 12-15 vision id sent to the Orin,
 *         16-19 vision id of the frame in hand, 20 valid, 21-23 turret
 *         guard state (2026-10-31, TurretGuard::State 0-7; 0 before)
 *   flags aim offset as int8 (+ = right), in px until 2026-10-30, in
 *         4 px units since (the offset reaches 300 px; ground unchanged) */
#define RCMAP_AIM_FRAME_BIT 0x8000u
typedef struct {
    uint16_t distance_cm;
    uint16_t elevation_pulse_us;
    uint8_t selected_id;
    uint8_t target_id;
    bool valid;
    int8_t aim_offset_px;
    uint8_t turret_state; /* 0-7 */
} rcmap_aim;

static inline bool rcmap_status_is_aim(uint16_t code) { return (code & 0xC000u) == RCMAP_AIM_FRAME_BIT; }
/* Out-of-range values are clamped (distance, pulse) or masked (ids). */
void rcmap_aim_to_status(const rcmap_aim *a, uint8_t *flags, uint32_t *seq, uint16_t *code);
/* False if s is not an aim frame. */
bool rcmap_aim_from_status(const rcmap_status *s, rcmap_aim *out);

/* Seen frames (2026-10-27, Orin protocol v3): which holes the camera
 * detects, for the Dashboard's board. The Mega's slots go status, aim,
 * status, vision, seen (each 100 ms).
 *   code  bits 15-14 = 11, bits 0-11 = the Orin's detected hole ids (bit
 *         id - 1, ids 1-12), bits 12-13 = vision state: 0 nothing, 1 the
 *         target in view, 2 a predicted (out-of-view) position
 *   seq   (2026-10-28; 0 before) bits 0-11 turret AS5600 degrees x 10,
 *         12-bit two's complement (+-204.7 deg), bits 12-17 its status
 *         bits (TurretAngle.h), bits 18-23 the depth ring's usable share:
 *         0 = none, else percent / 2 + 1 (2 % steps)
 *   flags (2026-10-28) the depth ring's sectors with depth (bit k =
 *         sector k: 0 right, clockwise on screen)
 * A ground older than this reads it as an aim frame: reflash both. */
#define RCMAP_SEEN_FRAME_BITS 0xC000u
#define RCMAP_VISION_STATE_NONE 0u
#define RCMAP_VISION_STATE_SEEN 1u
#define RCMAP_VISION_STATE_PREDICTED 2u
typedef struct {
    uint16_t seen_mask;  /* bit id - 1 */
    uint8_t state;       /* RCMAP_VISION_STATE_* */
    int16_t turret_deg10;   /* AS5600 turret degrees x 10, +-2047 */
    uint8_t turret_status;  /* TurretAngle status bits, 6 bits */
    int8_t ring_percent;    /* 0-100, -1 = none (even percents only) */
    uint8_t ring_sectors;   /* bit k = sector k has depth */
} rcmap_seen;

static inline bool rcmap_status_is_seen(uint16_t code) { return (code & 0xC000u) == RCMAP_SEEN_FRAME_BITS; }
void rcmap_seen_to_status(const rcmap_seen *v, uint8_t *flags, uint32_t *seq, uint16_t *code);
/* False if s is not a seen frame. */
bool rcmap_seen_from_status(const rcmap_status *s, rcmap_seen *out);

/* Vision frames (2026-10-17): tx / ty for the wireless Dashboard, in the
 * same ST line / Battery frame. The Mega's 100 ms slots go status, aim,
 * status, vision.
 *   code  bits 15-14 = 01 (a status code is at most 0x3FFF, an aim frame
 *         has bit 15 set), bits 0-13 = the Falcon's actual PWM pulse in us
 *         (2026-10-18; was the camera distance, which the aim frame
 *         already carries)
 *   seq   bits 0-11 tx * 2, bits 12-23 ty * 2, both 12-bit two's
 *         complement: 0.5 px steps, +-1023.5 px
 *   flags bit 0 valid, bits 1-4 vision id of the frame in hand
 * A ground STM32 older than this reads it as a status with a bogus event:
 * reflash the ground together with the Mega. */
#define RCMAP_VISION_FRAME_BIT 0x4000u
typedef struct {
    uint16_t flywheel_pulse_us; /* what D25 sends now, max 16383 */
    float tx_px;
    float ty_px;
    bool valid;
    uint8_t target_id;
} rcmap_vision;

static inline bool rcmap_status_is_vision(uint16_t code) {
    return (code & 0xC000u) == RCMAP_VISION_FRAME_BIT;
}
/* Out-of-range values are clamped (pulse, tx, ty) or masked (id). */
void rcmap_vision_to_status(const rcmap_vision *v, uint8_t *flags, uint32_t *seq, uint16_t *code);
/* False if s is not a vision frame. */
bool rcmap_vision_from_status(const rcmap_status *s, rcmap_vision *out);

#ifdef __cplusplus
}
#endif

#endif
