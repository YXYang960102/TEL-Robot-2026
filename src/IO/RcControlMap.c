#include "RcControlMap.h"

#define RCMAP_MIN_VALID_US 900
#define RCMAP_MAX_VALID_US 2100
#define RCMAP_SWITCH_ON_US 1750
#define RCMAP_TARGET_BASE_US 1000
#define RCMAP_TARGET_STEP_US 80 /* 13 levels, 1000..1960 (CH3 is full resolution) */
#define RCMAP_TARGET_TOLERANCE_US 35
#define RCMAP_LEVEL_BASE_US 1000
#define RCMAP_LEVEL_STEP_US 200
#define RCMAP_LEVEL_TOLERANCE_US 90 /* ELRS 6-pos sends 988/1193/1398/1602/1807 */
#define RCMAP_FIVE_LEVELS 5u

static float clamp_float(float v, float lo, float hi) {
    /* NaN fails both comparisons and would pass through, so test it
       explicitly (v != v) and treat it as neutral. */
    if (v != v) return 0.0f < lo ? lo : (0.0f > hi ? hi : 0.0f);
    return v < lo ? lo : (v > hi ? hi : v);
}

static int round_float(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static int axis_to_us(float v) {
    return 1500 + round_float(clamp_float(v, -1.0f, 1.0f) * 500.0f);
}

static float us_to_axis(int us) {
    return clamp_float((float)(us - 1500) / 500.0f, -1.0f, 1.0f);
}

/* Nearest level k (0..levels-1) of base + step*k, within tolerance. */
static bool us_to_level(int us, int base, int step, int tolerance, uint8_t levels, uint8_t *k_out) {
    const int offset = us - base;
    if (offset < -tolerance) return false;
    const int k = (offset + step / 2) / step;
    if (k < 0 || k >= (int)levels) return false;
    const int nearest = base + k * step;
    const int error = us > nearest ? us - nearest : nearest - us;
    if (error > tolerance) return false;
    *k_out = (uint8_t)k;
    return true;
}

/* CH4: the mode level, or full manual's flywheel range when it is on. */
static int clamp_offset_pct(int pct) {
    return pct > RCMAP_AUTO_OFFSET_MAX_PCT ? RCMAP_AUTO_OFFSET_MAX_PCT
         : pct < -RCMAP_AUTO_OFFSET_MAX_PCT ? -RCMAP_AUTO_OFFSET_MAX_PCT : pct;
}

static int mode_channel_us(const rcmap_command *c) {
    const uint8_t mode = rcmap_mode_from_us(c->mode_us);
    if (mode != RCMAP_MODE_FULL_MANUAL) {
        return rcmap_mode_to_us(mode) +
               clamp_offset_pct(c->flywheel_offset_pct) * RCMAP_AUTO_OFFSET_US_PER_PCT;
    }
    const int percent = round_float(clamp_float(c->flywheel, 0.0f, 1.0f) * 100.0f);
    if (percent <= 0) return RCMAP_MODE_FULL_MANUAL_US;
    return RCMAP_FLYWHEEL_BASE_US + (percent * RCMAP_FLYWHEEL_SPAN_US + 50) / 100;
}

/* CH4 back to mode + flywheel; false for the gap between the flywheel
 * range and full auto. Flywheel is rounded to the nearest 1% (3 us), which
 * absorbs the link's +-1 us quantization. */
static bool decode_mode_channel(int us, int *mode_us, float *flywheel, int8_t *offset_pct) {
    *flywheel = 0.0f;
    *offset_pct = 0;
    if (us >= RCMAP_MODE_FULL_AUTO_MIN_US) {
        const uint8_t mode = rcmap_mode_from_us(us);
        *mode_us = rcmap_mode_to_us(mode);
        if (mode != RCMAP_MODE_FULL_MANUAL) {
            /* Nearest whole percent from the level (+-1 us link noise). */
            const int d = us - *mode_us;
            const int half = RCMAP_AUTO_OFFSET_US_PER_PCT / 2;
            const int pct = d >= 0 ? (d + half) / RCMAP_AUTO_OFFSET_US_PER_PCT
                                   : -((-d + half) / RCMAP_AUTO_OFFSET_US_PER_PCT);
            *offset_pct = (int8_t)clamp_offset_pct(pct);
        }
        return true;
    }
    if (us > RCMAP_FLYWHEEL_MAX_US + RCMAP_FLYWHEEL_SPAN_US / 100) return false;
    int percent = ((us - RCMAP_FLYWHEEL_BASE_US) * 100 + RCMAP_FLYWHEEL_SPAN_US / 2) /
                  RCMAP_FLYWHEEL_SPAN_US;
    if (us < RCMAP_FLYWHEEL_BASE_US) percent = 0;
    if (percent > 100) percent = 100;
    *mode_us = RCMAP_MODE_FULL_MANUAL_US;
    *flywheel = (float)percent / 100.0f;
    return true;
}

void rcmap_encode(const rcmap_command *c, int ch_us[16]) {
    for (uint8_t i = 0; i < 16u; i++) ch_us[i] = 1500;
    ch_us[RCMAP_CH_FORWARD] = axis_to_us(c->forward);
    ch_us[RCMAP_CH_TURN] = axis_to_us(c->turn);
    const uint8_t target = c->target <= RCMAP_TARGET_MAX ? c->target : RCMAP_TARGET_NONE;
    ch_us[RCMAP_CH_TARGET] = RCMAP_TARGET_BASE_US + (int)target * RCMAP_TARGET_STEP_US;
    ch_us[RCMAP_CH_MODE] = mode_channel_us(c);
    ch_us[RCMAP_CH_FIRE] = c->fire ? 2000 : 1000;
    ch_us[RCMAP_CH_ARM] = c->armed ? 2000 : 1000;
    const uint8_t nudge = c->nudge < RCMAP_FIVE_LEVELS ? c->nudge : RCMAP_NUDGE_CENTER;
    ch_us[RCMAP_CH_NUDGE] = RCMAP_LEVEL_BASE_US + (int)nudge * RCMAP_LEVEL_STEP_US;
    const uint8_t path = c->auto_path <= RCMAP_AUTO_PATHS ? c->auto_path : RCMAP_AUTO_NONE;
    ch_us[RCMAP_CH_AUTO] = RCMAP_LEVEL_BASE_US + (int)path * RCMAP_LEVEL_STEP_US;
    {
        const bool test = rcmap_command_mode(c) == RCMAP_MODE_TEST;
        const int level = test ? (c->turret_home ? 5 : 4) : (c->turret_home ? 1 : 2);
        ch_us[RCMAP_CH_TEST] = RCMAP_LEVEL_BASE_US + level * RCMAP_LEVEL_STEP_US;
    }
    ch_us[RCMAP_CH_VERSION] = RCMAP_LEVEL_BASE_US + (int)RCMAP_LINK_VERSION * RCMAP_LEVEL_STEP_US;
}

bool rcmap_link_version_ok(const int *ch_us, uint8_t count) {
    uint8_t version;
    return count >= RCMAP_CHANNEL_COUNT &&
           us_to_level(ch_us[RCMAP_CH_VERSION], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                       RCMAP_LEVEL_TOLERANCE_US, 6u, &version) &&
           version == RCMAP_LINK_VERSION;
}

bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out) {
    if (count < RCMAP_CHANNEL_COUNT) return false;
    if (!rcmap_link_version_ok(ch_us, count)) return false;
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        if (ch_us[i] < RCMAP_MIN_VALID_US || ch_us[i] > RCMAP_MAX_VALID_US) return false;
    }
    uint8_t target, nudge, path, test_level;
    int mode_us;
    float flywheel;
    int8_t offset_pct;
    if (!decode_mode_channel(ch_us[RCMAP_CH_MODE], &mode_us, &flywheel, &offset_pct) ||
        !us_to_level(ch_us[RCMAP_CH_TARGET], RCMAP_TARGET_BASE_US, RCMAP_TARGET_STEP_US,
                     RCMAP_TARGET_TOLERANCE_US, RCMAP_TARGET_MAX + 1u, &target) ||
        !us_to_level(ch_us[RCMAP_CH_NUDGE], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                     RCMAP_LEVEL_TOLERANCE_US, RCMAP_FIVE_LEVELS, &nudge) ||
        !us_to_level(ch_us[RCMAP_CH_AUTO], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                     RCMAP_LEVEL_TOLERANCE_US, RCMAP_FIVE_LEVELS, &path) ||
        !us_to_level(ch_us[RCMAP_CH_TEST], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                     RCMAP_LEVEL_TOLERANCE_US, 6u, &test_level)) {
        return false;
    }
    out->forward = us_to_axis(ch_us[RCMAP_CH_FORWARD]);
    out->turn = us_to_axis(ch_us[RCMAP_CH_TURN]);
    out->target = target;
    out->mode_us = mode_us;
    out->flywheel = flywheel;
    out->flywheel_offset_pct = offset_pct;
    out->fire = ch_us[RCMAP_CH_FIRE] >= RCMAP_SWITCH_ON_US;
    out->armed = ch_us[RCMAP_CH_ARM] >= RCMAP_SWITCH_ON_US;
    out->nudge = nudge;
    out->auto_path = path;
    /* Fail-safe like fire / arm: only a clear high counts, and only with
       CH4 in full manual (any other level is that mode, whatever CH9 says). */
    /* CH9 (a level, like CH7 / CH8: between levels = corrupt frame): TEST
       only with CH4 in full manual (any other level is that mode, whatever
       CH9 says); turret home on levels 1 and 5. */
    out->test_mode = (test_level == 4u || test_level == 5u) &&
                     rcmap_mode_from_us(mode_us) == RCMAP_MODE_FULL_MANUAL;
    out->turret_home = test_level == 1u || test_level == 5u;
    return true;
}

uint8_t rcmap_line_crc8(const char *text, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint8_t)text[i];
        for (uint8_t b = 0; b < 8u; b++) {
            const uint8_t shifted = (uint8_t)(crc << 1);
            crc = (crc & 0x80u) ? (uint8_t)(shifted ^ 0xD5u) : shifted;
        }
    }
    return crc;
}

static const char hex_digits[] = "0123456789ABCDEF";

uint16_t rcmap_line_seal(char *buf, uint16_t len, uint16_t buf_size) {
    if (buf_size < RCMAP_LINE_CHECK_CHARS || len > buf_size - RCMAP_LINE_CHECK_CHARS) return 0;
    const uint8_t crc = rcmap_line_crc8(buf, len);
    buf[len] = '*';
    buf[len + 1u] = hex_digits[crc >> 4];
    buf[len + 2u] = hex_digits[crc & 0x0Fu];
    return (uint16_t)(len + RCMAP_LINE_CHECK_CHARS);
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

int rcmap_line_open(const char *line, uint16_t len) {
    if (len < RCMAP_LINE_CHECK_CHARS) return -1;
    const uint16_t text = (uint16_t)(len - RCMAP_LINE_CHECK_CHARS);
    const int hi = hex_value(line[text + 1u]);
    const int lo = hex_value(line[text + 2u]);
    if (line[text] != '*' || hi < 0 || lo < 0) return -1;
    return rcmap_line_crc8(line, text) == (uint8_t)(hi << 4 | lo) ? (int)text : -1;
}

static const char *const event_names[RCMAP_EVENT_COUNT] = {
    "NONE",
    "OUTPUTS_ENABLED",
    "OUTPUTS_DISABLED",
    "COMMAND_TIMEOUT",
    "ENABLE_REJECTED_PINS_UNCONFIRMED",
    "ENABLE_REJECTED_OTHER_SOURCE",
    "REJECTED_DISARMED",
    "REJECTED_NOT_OWNER",
    "DISABLED_RC_LINK_LOST",
    "DISABLED_CORRUPT_COMMAND",
    "FEED_TRIGGER_SENT",
    "FEED_BUSY",
    "ROTATE_MANUAL",
    "ROTATE_VISION_FOLLOW",
    "BAD_COMMAND",
    "UNKNOWN_COMMAND",
    "COMMAND_TOO_LONG",
    "EVENT_SKIPPED",
    "CONVEYOR_ON",
    "CONVEYOR_OFF",
    "MODE_FULL_MANUAL",
    "MODE_SEMI_AUTO",
    "MODE_FULL_AUTO",
    "FIRE_NOT_READY",
    "ENABLE_REJECTED_NOT_MANUAL",
    "REJECTED_AUTO_MODE",
    "TARGET_SELECTED",
    "AUTO_PATH_STARTED",
    "AUTO_PATH_REJECTED",
    "MODE_TEST",
    "LINK_VERSION_MISMATCH",
    "TURRET_ZERO_SET",
    "TURRET_ZERO_REJECTED",
};

const char *rcmap_event_name(uint16_t code) {
    return code < RCMAP_EVENT_COUNT ? event_names[code] : "UNKNOWN_EVENT_CODE";
}

void rcmap_status_to_battery(const rcmap_status *s, uint16_t *voltage, uint16_t *current,
                             uint32_t *capacity, uint8_t *remaining) {
    *voltage = (uint16_t)(((uint16_t)RCMAP_STATUS_MARKER << 8) | s->flags);
    *current = s->event_code;
    *capacity = s->event_seq & 0xFFFFFFu;
    *remaining = s->rolling;
}

void rcmap_aim_to_status(const rcmap_aim *a, uint8_t *flags, uint32_t *seq, uint16_t *code) {
    const uint16_t cm = a->distance_cm > 0x3FFFu ? 0x3FFFu : a->distance_cm;
    const uint16_t pulse = a->elevation_pulse_us > 0x0FFFu ? 0x0FFFu : a->elevation_pulse_us;
    *code = (uint16_t)(RCMAP_AIM_FRAME_BIT | cm);
    *seq = (uint32_t)pulse | ((uint32_t)(a->selected_id & 0x0Fu) << 12) |
           ((uint32_t)(a->target_id & 0x0Fu) << 16) | ((uint32_t)(a->valid ? 1u : 0u) << 20) |
           ((uint32_t)(a->turret_state & 0x07u) << 21);
    *flags = (uint8_t)a->aim_offset_px;
}

bool rcmap_aim_from_status(const rcmap_status *s, rcmap_aim *out) {
    if (!rcmap_status_is_aim(s->event_code)) return false;
    out->distance_cm = (uint16_t)(s->event_code & 0x3FFFu);
    out->elevation_pulse_us = (uint16_t)(s->event_seq & 0x0FFFu);
    out->selected_id = (uint8_t)((s->event_seq >> 12) & 0x0Fu);
    out->target_id = (uint8_t)((s->event_seq >> 16) & 0x0Fu);
    out->valid = ((s->event_seq >> 20) & 1u) != 0u;
    out->turret_state = (uint8_t)((s->event_seq >> 21) & 0x07u);
    out->aim_offset_px = (int8_t)s->flags;
    return true;
}

void rcmap_seen_to_status(const rcmap_seen *v, uint8_t *flags, uint32_t *seq, uint16_t *code) {
    const uint8_t state = v->state > RCMAP_VISION_STATE_PREDICTED ? RCMAP_VISION_STATE_NONE : v->state;
    const int deg10 = v->turret_deg10 > 2047 ? 2047 : (v->turret_deg10 < -2047 ? -2047 : v->turret_deg10);
    const uint32_t ring = v->ring_percent < 0 ? 0u
                        : (uint32_t)((v->ring_percent > 100 ? 100 : v->ring_percent) + 1) / 2u + 1u;
    *code = (uint16_t)(RCMAP_SEEN_FRAME_BITS | ((uint16_t)state << 12) | (v->seen_mask & 0x0FFFu));
    *seq = ((uint32_t)deg10 & 0x0FFFu) | ((uint32_t)(v->turret_status & 0x3Fu) << 12) | (ring << 18);
    *flags = v->ring_percent < 0 ? 0u : v->ring_sectors;
}

bool rcmap_seen_from_status(const rcmap_status *s, rcmap_seen *out) {
    if (!rcmap_status_is_seen(s->event_code)) return false;
    out->seen_mask = (uint16_t)(s->event_code & 0x0FFFu);
    out->state = (uint8_t)((s->event_code >> 12) & 0x03u);
    const uint32_t deg = s->event_seq & 0x0FFFu;
    out->turret_deg10 = (int16_t)(deg >= 0x0800u ? (int32_t)deg - 0x1000 : (int32_t)deg);
    out->turret_status = (uint8_t)((s->event_seq >> 12) & 0x3Fu);
    const uint32_t ring = (s->event_seq >> 18) & 0x3Fu;
    out->ring_percent = (int8_t)(ring == 0u ? -1 : (ring > 51u ? 100 : (int)(ring - 1u) * 2));
    out->ring_sectors = ring == 0u ? 0u : s->flags;
    return true;
}

static uint32_t half_px_field(float px) {
    int v = round_float(clamp_float(px, -1023.5f, 1023.5f) * 2.0f);
    return (uint32_t)v & 0x0FFFu;
}

static float half_px_value(uint32_t field) {
    int v = (int)(field & 0x0FFFu);
    if (v >= 0x800) v -= 0x1000;
    return (float)v / 2.0f;
}

void rcmap_vision_to_status(const rcmap_vision *v, uint8_t *flags, uint32_t *seq, uint16_t *code) {
    const uint16_t us = v->flywheel_pulse_us > 0x3FFFu ? 0x3FFFu : v->flywheel_pulse_us;
    *code = (uint16_t)(RCMAP_VISION_FRAME_BIT | us);
    *seq = half_px_field(v->tx_px) | (half_px_field(v->ty_px) << 12);
    *flags = (uint8_t)((v->valid ? 1u : 0u) | ((v->target_id & 0x0Fu) << 1));
}

bool rcmap_vision_from_status(const rcmap_status *s, rcmap_vision *out) {
    if (!rcmap_status_is_vision(s->event_code)) return false;
    out->flywheel_pulse_us = (uint16_t)(s->event_code & 0x3FFFu);
    out->tx_px = half_px_value(s->event_seq);
    out->ty_px = half_px_value(s->event_seq >> 12);
    out->valid = (s->flags & 1u) != 0u;
    out->target_id = (uint8_t)((s->flags >> 1) & 0x0Fu);
    return true;
}

bool rcmap_status_from_battery(uint16_t voltage, uint16_t current, uint32_t capacity,
                               uint8_t remaining, rcmap_status *out) {
    if ((voltage >> 8) != RCMAP_STATUS_MARKER) return false;
    out->flags = (uint8_t)(voltage & 0xFFu);
    out->event_code = current;
    out->event_seq = capacity & 0xFFFFFFu;
    out->rolling = remaining;
    return true;
}
