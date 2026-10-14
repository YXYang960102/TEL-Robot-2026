#include "RcControlMap.h"

#define RCMAP_MIN_VALID_US 900
#define RCMAP_MAX_VALID_US 2100
#define RCMAP_ARM_ON_US 1750
#define RCMAP_DIR_LOW_US 1250
#define RCMAP_DIR_HIGH_US 1750
#define RCMAP_COUNTER_BASE_US 1000
#define RCMAP_COUNTER_STEP_US 200
#define RCMAP_COUNTER_TOLERANCE_US 90
#define RCMAP_FLYWHEEL_DEADBAND 0.02f

static float clamp_float(float v, float lo, float hi) {
    /* NaN fails both comparisons and would pass through, so test it
       explicitly (v != v) and treat it as the neutral low bound. */
    if (v != v) return lo < 0.0f ? 0.0f : lo;
    return v < lo ? lo : (v > hi ? hi : v);
}

static int round_float(float v) {
    return (int)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static int axis_to_us(float v) {
    return 1500 + round_float(clamp_float(v, -1.0f, 1.0f) * 500.0f);
}

static int dir_to_us(int8_t d) {
    return d > 0 ? 2000 : (d < 0 ? 1000 : 1500);
}

static int counter_to_us(uint8_t level) {
    return RCMAP_COUNTER_BASE_US + (int)(level % RCMAP_COUNTER_LEVELS) * RCMAP_COUNTER_STEP_US;
}

void rcmap_encode(const rcmap_command *c, int ch_us[16]) {
    for (uint8_t i = 0; i < 16u; i++) ch_us[i] = 1500;
    ch_us[RCMAP_CH_LEFT] = axis_to_us(c->left);
    ch_us[RCMAP_CH_RIGHT] = axis_to_us(c->right);
    ch_us[RCMAP_CH_FLYWHEEL] = 1000 + round_float(clamp_float(c->flywheel, 0.0f, 1.0f) * 1000.0f);
    ch_us[RCMAP_CH_MODE] = rcmap_mode_to_us(rcmap_mode_from_us(c->mode_us));
    ch_us[RCMAP_CH_CONVEYOR] = c->armed && c->conveyor ? 2000 : 1000;
    ch_us[RCMAP_CH_ARM] = c->armed ? 2000 : 1000;
    ch_us[RCMAP_CH_ROTATE] = dir_to_us(c->rotate);
    ch_us[RCMAP_CH_ANGLE] = dir_to_us(c->angle);
    ch_us[RCMAP_CH_FEED_COUNTER] = counter_to_us(c->feed_counter);
    ch_us[RCMAP_CH_LMODE_COUNTER] = counter_to_us(c->lmode_counter);
}

static float us_to_axis(int us) {
    return clamp_float((float)(us - 1500) / 500.0f, -1.0f, 1.0f);
}

static int8_t us_to_dir(int us) {
    if (us < RCMAP_DIR_LOW_US) return -1;
    if (us > RCMAP_DIR_HIGH_US) return 1;
    return 0;
}

static bool us_to_counter(int us, uint8_t *level) {
    const int offset = us - RCMAP_COUNTER_BASE_US;
    const int k = (offset + RCMAP_COUNTER_STEP_US / 2) / RCMAP_COUNTER_STEP_US;
    if (offset < -RCMAP_COUNTER_TOLERANCE_US || k < 0 || k >= (int)RCMAP_COUNTER_LEVELS) {
        return false;
    }
    const int nearest = RCMAP_COUNTER_BASE_US + k * RCMAP_COUNTER_STEP_US;
    const int error = us > nearest ? us - nearest : nearest - us;
    if (error > RCMAP_COUNTER_TOLERANCE_US) return false;
    *level = (uint8_t)k;
    return true;
}

bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out) {
    if (count < RCMAP_CHANNEL_COUNT) return false;
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        if (ch_us[i] < RCMAP_MIN_VALID_US || ch_us[i] > RCMAP_MAX_VALID_US) return false;
    }
    uint8_t feed, lmode;
    if (!us_to_counter(ch_us[RCMAP_CH_FEED_COUNTER], &feed) ||
        !us_to_counter(ch_us[RCMAP_CH_LMODE_COUNTER], &lmode)) {
        return false;
    }

    out->left = us_to_axis(ch_us[RCMAP_CH_LEFT]);
    out->right = us_to_axis(ch_us[RCMAP_CH_RIGHT]);
    float fly = clamp_float((float)(ch_us[RCMAP_CH_FLYWHEEL] - 1000) / 1000.0f, 0.0f, 1.0f);
    out->flywheel = fly < RCMAP_FLYWHEEL_DEADBAND ? 0.0f : fly;
    out->mode_us = ch_us[RCMAP_CH_MODE];
    out->conveyor = ch_us[RCMAP_CH_CONVEYOR] >= RCMAP_ARM_ON_US;
    out->armed = ch_us[RCMAP_CH_ARM] >= RCMAP_ARM_ON_US;
    out->rotate = us_to_dir(ch_us[RCMAP_CH_ROTATE]);
    out->angle = us_to_dir(ch_us[RCMAP_CH_ANGLE]);
    out->feed_counter = feed;
    out->lmode_counter = lmode;
    return true;
}

void rcmap_counter_reset(rcmap_counter *c) {
    c->have_baseline = false;
    c->level = 0;
}

int rcmap_counter_update(rcmap_counter *c, uint8_t level) {
    level = (uint8_t)(level % RCMAP_COUNTER_LEVELS);
    if (!c->have_baseline) {
        c->have_baseline = true;
        c->level = level;
        return RCMAP_COUNTER_NONE;
    }
    const uint8_t delta =
        (uint8_t)((level + RCMAP_COUNTER_LEVELS - c->level) % RCMAP_COUNTER_LEVELS);
    c->level = level;
    if (delta == 0u) return RCMAP_COUNTER_NONE;
    if (delta == 1u) return RCMAP_COUNTER_EVENT;
    return RCMAP_COUNTER_SKIPPED;
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

bool rcmap_status_from_battery(uint16_t voltage, uint16_t current, uint32_t capacity,
                               uint8_t remaining, rcmap_status *out) {
    if ((voltage >> 8) != RCMAP_STATUS_MARKER) return false;
    out->flags = (uint8_t)(voltage & 0xFFu);
    out->event_code = current;
    out->event_seq = capacity & 0xFFFFFFu;
    out->rolling = remaining;
    return true;
}
