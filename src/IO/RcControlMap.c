#include "RcControlMap.h"

#define RCMAP_MIN_VALID_US 900
#define RCMAP_MAX_VALID_US 2100
#define RCMAP_SWITCH_ON_US 1750
#define RCMAP_TARGET_BASE_US 1000
#define RCMAP_TARGET_STEP_US 100
#define RCMAP_TARGET_TOLERANCE_US 40
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

void rcmap_encode(const rcmap_command *c, int ch_us[16]) {
    for (uint8_t i = 0; i < 16u; i++) ch_us[i] = 1500;
    ch_us[RCMAP_CH_FORWARD] = axis_to_us(c->forward);
    ch_us[RCMAP_CH_TURN] = axis_to_us(c->turn);
    const uint8_t target = c->target <= RCMAP_TARGET_MAX ? c->target : RCMAP_TARGET_NONE;
    ch_us[RCMAP_CH_TARGET] = RCMAP_TARGET_BASE_US + (int)target * RCMAP_TARGET_STEP_US;
    ch_us[RCMAP_CH_MODE] = rcmap_mode_to_us(rcmap_mode_from_us(c->mode_us));
    ch_us[RCMAP_CH_FIRE] = c->fire ? 2000 : 1000;
    ch_us[RCMAP_CH_ARM] = c->armed ? 2000 : 1000;
    const uint8_t nudge = c->nudge < RCMAP_FIVE_LEVELS ? c->nudge : RCMAP_NUDGE_CENTER;
    ch_us[RCMAP_CH_NUDGE] = RCMAP_LEVEL_BASE_US + (int)nudge * RCMAP_LEVEL_STEP_US;
    const uint8_t path = c->auto_path <= RCMAP_AUTO_PATHS ? c->auto_path : RCMAP_AUTO_NONE;
    ch_us[RCMAP_CH_AUTO] = RCMAP_LEVEL_BASE_US + (int)path * RCMAP_LEVEL_STEP_US;
}

bool rcmap_decode(const int *ch_us, uint8_t count, rcmap_command *out) {
    if (count < RCMAP_CHANNEL_COUNT) return false;
    for (uint8_t i = 0; i < RCMAP_CHANNEL_COUNT; i++) {
        if (ch_us[i] < RCMAP_MIN_VALID_US || ch_us[i] > RCMAP_MAX_VALID_US) return false;
    }
    uint8_t target, nudge, path;
    if (!us_to_level(ch_us[RCMAP_CH_TARGET], RCMAP_TARGET_BASE_US, RCMAP_TARGET_STEP_US,
                     RCMAP_TARGET_TOLERANCE_US, RCMAP_TARGET_MAX + 1u, &target) ||
        !us_to_level(ch_us[RCMAP_CH_NUDGE], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                     RCMAP_LEVEL_TOLERANCE_US, RCMAP_FIVE_LEVELS, &nudge) ||
        !us_to_level(ch_us[RCMAP_CH_AUTO], RCMAP_LEVEL_BASE_US, RCMAP_LEVEL_STEP_US,
                     RCMAP_LEVEL_TOLERANCE_US, RCMAP_FIVE_LEVELS, &path)) {
        return false;
    }
    out->forward = us_to_axis(ch_us[RCMAP_CH_FORWARD]);
    out->turn = us_to_axis(ch_us[RCMAP_CH_TURN]);
    out->target = target;
    out->mode_us = ch_us[RCMAP_CH_MODE];
    out->fire = ch_us[RCMAP_CH_FIRE] >= RCMAP_SWITCH_ON_US;
    out->armed = ch_us[RCMAP_CH_ARM] >= RCMAP_SWITCH_ON_US;
    out->nudge = nudge;
    out->auto_path = path;
    return true;
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
