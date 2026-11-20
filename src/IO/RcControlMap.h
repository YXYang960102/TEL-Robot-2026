#ifndef RC_CONTROL_MAP_H
#define RC_CONTROL_MAP_H

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
#define RCMAP_LINK_VERSION 1u
#define RCMAP_CHANNEL_COUNT 10u

#define RCMAP_TARGET_NONE 0u
#define RCMAP_TARGET_MAX 12u
#define RCMAP_BALL_COLORS 6u
#define RCMAP_BALL_YELLOW 0u
#define RCMAP_BALL_PURPLE 1u
#define RCMAP_BALL_BLUE 2u
#define RCMAP_BALL_GREEN 3u
#define RCMAP_BALL_RED 4u
#define RCMAP_BALL_ORANGE 5u
#define RCMAP_HALF_KEEP 0u
#define RCMAP_HALF_FIRST 1u
#define RCMAP_HALF_SECOND 2u

#define RCMAP_NUDGE_CENTER 0u
#define RCMAP_NUDGE_UP 1u
#define RCMAP_NUDGE_DOWN 2u
#define RCMAP_NUDGE_LEFT 3u
#define RCMAP_NUDGE_RIGHT 4u

#define RCMAP_AUTO_NONE 0u
#define RCMAP_AUTO_PATHS 6u          /* radio paths 1..6 */
#define RCMAP_AUTO_CH8_PATHS 5u      /* 1..5 on CH8 levels 1..5 */
#define RCMAP_AUTO_PATH6_US 1350     /* path 6: CH4 pulse in the unused 1304..1399 gap = full auto + path 6 */
#define RCMAP_AUTO_PATH6_TOLERANCE_US 30

#define RCMAP_MODE_FULL_AUTO 0u
#define RCMAP_MODE_SEMI_AUTO 1u
#define RCMAP_MODE_FULL_MANUAL 2u
#define RCMAP_MODE_TEST 3u
#define RCMAP_MODE_FULL_AUTO_US 1500
#define RCMAP_MODE_SEMI_AUTO_US 1750
#define RCMAP_MODE_FULL_MANUAL_US 2000
#define RCMAP_MODE_FULL_AUTO_MIN_US 1400
#define RCMAP_MODE_SEMI_AUTO_MIN_US 1625
#define RCMAP_MODE_FULL_MANUAL_MIN_US 1875
#define RCMAP_FLYWHEEL_BASE_US 1000
#define RCMAP_FLYWHEEL_SPAN_US 300
#define RCMAP_FLYWHEEL_MAX_US (RCMAP_FLYWHEEL_BASE_US + RCMAP_FLYWHEEL_SPAN_US)
#define RCMAP_AUTO_OFFSET_US_PER_PCT 4
#define RCMAP_AUTO_OFFSET_MAX_PCT 20

static inline uint8_t rcmap_mode_from_us(int us) {
    if (us >= RCMAP_MODE_FULL_MANUAL_MIN_US) return RCMAP_MODE_FULL_MANUAL;
    if (us >= RCMAP_MODE_SEMI_AUTO_MIN_US) return RCMAP_MODE_SEMI_AUTO;
    if (us >= RCMAP_MODE_FULL_AUTO_MIN_US) return RCMAP_MODE_FULL_AUTO;
    return RCMAP_MODE_FULL_MANUAL;
}
static inline int rcmap_mode_to_us(uint8_t mode) {
    return mode == RCMAP_MODE_FULL_AUTO ? RCMAP_MODE_FULL_AUTO_US
         : mode == RCMAP_MODE_SEMI_AUTO ? RCMAP_MODE_SEMI_AUTO_US
         : RCMAP_MODE_FULL_MANUAL_US;
}

typedef struct {
    float forward;
    float turn;
    uint8_t target;
    uint8_t ball_color;
    int mode_us;
    float flywheel;
    int8_t flywheel_offset_pct;
    bool test_mode;
    bool fire;
    bool armed;
    uint8_t nudge;
    uint8_t auto_path;
    bool turret_home;
    bool capture_toggle;
    uint8_t half;
    bool match_start;
} rcmap_command;

static inline uint8_t rcmap_command_mode(const rcmap_command *c) {
    const uint8_t mode = rcmap_mode_from_us(c->mode_us);
    return mode == RCMAP_MODE_FULL_MANUAL && c->test_mode ? RCMAP_MODE_TEST : mode;
}

void rcmap_encode(const rcmap_command *c, int ch_us[16]);

bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out);
bool rcmap_link_version_ok(const int *ch_us, uint8_t count);

#define RCMAP_LINE_CHECK_CHARS 3u
uint8_t rcmap_line_crc8(const char *text, uint16_t len);
uint16_t rcmap_line_seal(char *buf, uint16_t len, uint16_t buf_size);
int rcmap_line_open(const char *line, uint16_t len);

#define RCMAP_STATUS_ARMED 0x01u
#define RCMAP_STATUS_OWNER_WIRELESS 0x02u
#define RCMAP_STATUS_OWNER_WIRED 0x04u
#define RCMAP_STATUS_PINS_CONFIRMED 0x08u
#define RCMAP_STATUS_FEED_BUSY 0x10u
#define RCMAP_STATUS_ROTATE_MANUAL 0x20u
#define RCMAP_STATUS_RC_LINK_OK 0x40u
#define RCMAP_STATUS_TEST_MODE 0x80u

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
    RCMAP_EVENT_REJECTED_AUTO_MODE,
    RCMAP_EVENT_TARGET_SELECTED,
    RCMAP_EVENT_AUTO_PATH_STARTED,
    RCMAP_EVENT_AUTO_PATH_REJECTED,
    RCMAP_EVENT_MODE_TEST,
    RCMAP_EVENT_LINK_VERSION_MISMATCH,
    RCMAP_EVENT_TURRET_ZERO_SET,
    RCMAP_EVENT_TURRET_ZERO_REJECTED,
    RCMAP_EVENT_CAPTURE_ON,
    RCMAP_EVENT_CAPTURE_OFF,
    RCMAP_EVENT_AUTO_PATH_DONE,
    RCMAP_EVENT_AUTO_PATH_TAKEOVER,
    RCMAP_EVENT_AUTO_PATH_ABORTED,
    RCMAP_EVENT_BALL_YELLOW,
    RCMAP_EVENT_BALL_PURPLE,
    RCMAP_EVENT_BALL_BLUE,
    RCMAP_EVENT_BALL_GREEN,
    RCMAP_EVENT_BALL_RED,
    RCMAP_EVENT_BALL_ORANGE,
    RCMAP_EVENT_HALF_FIRST,
    RCMAP_EVENT_HALF_SECOND,
    RCMAP_EVENT_MATCH_STARTED,
    RCMAP_EVENT_COUNT
} rcmap_event;

const char *rcmap_event_name(uint16_t code);

#define RCMAP_STATUS_MARKER 0xA5u

#define RCMAP_MODE_UNKNOWN 0xFFu
static inline uint16_t rcmap_status_code(uint16_t event, uint8_t mode) {
    const uint8_t m = mode > RCMAP_MODE_FULL_MANUAL ? RCMAP_MODE_FULL_MANUAL : mode;
    return (uint16_t)((event & 0x0FFFu) | ((uint16_t)((m + 1u) & 0x03u) << 12));
}
static inline uint16_t rcmap_status_code_event(uint16_t code) { return code & 0x0FFFu; }
static inline uint8_t rcmap_status_code_mode(uint16_t code) {
    const uint8_t stored = (uint8_t)(code >> 12);
    return stored == 0u ? RCMAP_MODE_UNKNOWN : (uint8_t)(stored - 1u);
}
static inline uint8_t rcmap_status_mode(uint16_t code, uint8_t flags) {
    const uint8_t mode = rcmap_status_code_mode(code);
    return mode == RCMAP_MODE_FULL_MANUAL && (flags & RCMAP_STATUS_TEST_MODE) ? RCMAP_MODE_TEST : mode;
}

typedef struct {
    uint8_t flags;
    uint16_t event_code;
    uint32_t event_seq;
    uint8_t rolling;
} rcmap_status;

void rcmap_status_to_battery(const rcmap_status *s, uint16_t *voltage, uint16_t *current,
                             uint32_t *capacity, uint8_t *remaining);
bool rcmap_status_from_battery(uint16_t voltage, uint16_t current, uint32_t capacity,
                               uint8_t remaining, rcmap_status *out);

#define RCMAP_AIM_FRAME_BIT 0x8000u
typedef struct {
    uint16_t distance_cm;
    uint16_t elevation_pulse_us;
    uint8_t selected_id;
    uint8_t target_id;
    bool valid;
    int8_t aim_offset_px;
    uint8_t turret_state;
} rcmap_aim;

static inline bool rcmap_status_is_aim(uint16_t code) { return (code & 0xC000u) == RCMAP_AIM_FRAME_BIT; }
void rcmap_aim_to_status(const rcmap_aim *a, uint8_t *flags, uint32_t *seq, uint16_t *code);
bool rcmap_aim_from_status(const rcmap_status *s, rcmap_aim *out);

#define RCMAP_SEEN_FRAME_BITS 0xC000u
#define RCMAP_VISION_STATE_NONE 0u
#define RCMAP_VISION_STATE_SEEN 1u
#define RCMAP_VISION_STATE_PREDICTED 2u
typedef struct {
    uint16_t seen_mask;
    uint8_t state;
    int16_t turret_deg10;
    uint8_t turret_status;
    int8_t ring_percent;
    uint8_t ring_sectors;
} rcmap_seen;

static inline bool rcmap_status_is_seen(uint16_t code) { return (code & 0xC000u) == RCMAP_SEEN_FRAME_BITS; }
void rcmap_seen_to_status(const rcmap_seen *v, uint8_t *flags, uint32_t *seq, uint16_t *code);
bool rcmap_seen_from_status(const rcmap_status *s, rcmap_seen *out);

#define RCMAP_VISION_FRAME_BIT 0x4000u
typedef struct {
    uint16_t flywheel_pulse_us;
    float tx_px;
    float ty_px;
    bool valid;
    uint8_t target_id;
} rcmap_vision;

static inline bool rcmap_status_is_vision(uint16_t code) {
    return (code & 0xE000u) == RCMAP_VISION_FRAME_BIT;
}
void rcmap_vision_to_status(const rcmap_vision *v, uint8_t *flags, uint32_t *seq, uint16_t *code);
bool rcmap_vision_from_status(const rcmap_status *s, rcmap_vision *out);

#define RCMAP_BALLS_FRAME_BITS 0x6000u
typedef struct {
    uint16_t loaded;
    uint16_t fired;
    bool ball_ready;
    bool required;
    bool tower_enabled;
    uint16_t blockers;
} rcmap_balls;

static inline bool rcmap_status_is_balls(uint16_t code) {
    return (code & 0xE000u) == RCMAP_BALLS_FRAME_BITS;
}
void rcmap_balls_to_status(const rcmap_balls *b, uint8_t *flags, uint32_t *seq, uint16_t *code);
bool rcmap_balls_from_status(const rcmap_status *s, rcmap_balls *out);

#ifdef __cplusplus
}
#endif

#endif
