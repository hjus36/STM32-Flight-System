#include "lkm_fc100hz_core.h"
#include <math.h>
#include <string.h>
#include <limits.h>

static void incr_sat(uint32_t *v) { if (*v != UINT32_MAX) ++*v; }
static void add_sat(uint32_t *v, uint32_t n)
{
    if (UINT32_MAX - *v < n) *v = UINT32_MAX;
    else *v += n;
}
/* True iff now is at/after stamp and elapsed < 2^31 ticks. */
static bool valid_elapsed(uint32_t now, uint32_t stamp, uint32_t *age)
{
    const uint32_t difference = now - stamp;
    if (difference >= UINT32_C(0x80000000)) return false;
    if (age != NULL) *age = difference;
    return true;
}
static bool newer_counter(uint32_t value, uint32_t previous)
{
    uint32_t diff = value - previous;
    return value != 0u && diff != 0u && diff < UINT32_C(0x80000000);
}

lkm_fc100hz_result_t lkm_fc100hz_sched_init(
    lkm_fc100hz_scheduler_t *ctx, uint32_t start_ms,
    uint32_t allowed_lateness_ms, uint32_t allowed_work_ms)
{
    if (ctx == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    memset(ctx, 0, sizeof(*ctx));
    if (allowed_lateness_ms == 0u || allowed_lateness_ms >= LKM_FC100HZ_PERIOD_MS ||
        allowed_work_ms == 0u || allowed_work_ms >= LKM_FC100HZ_PERIOD_MS) {
        ctx->last_result = LKM_FC100HZ_BAD_CONFIG;
        return LKM_FC100HZ_BAD_CONFIG;
    }
    ctx->initialized = true;
    ctx->next_due_ms = start_ms + LKM_FC100HZ_PERIOD_MS;
    ctx->last_poll_ms = start_ms;
    ctx->max_allowed_lateness_ms = allowed_lateness_ms;
    ctx->max_allowed_work_ms = allowed_work_ms;
    ctx->last_result = LKM_FC100HZ_OK;
    return LKM_FC100HZ_OK;
}

lkm_fc100hz_result_t lkm_fc100hz_sched_poll(
    lkm_fc100hz_scheduler_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    if (!ctx->initialized || ctx->fault_latched || ctx->work_in_flight) {
        if (ctx->work_in_flight) ctx->fault_latched = true;
        ctx->last_result = LKM_FC100HZ_FAULT;
        return ctx->last_result;
    }
    if (!valid_elapsed(now_ms, ctx->last_poll_ms, NULL)) {
        ctx->fault_latched = true;
        ctx->last_result = LKM_FC100HZ_FAULT;
        return ctx->last_result;
    }
    ctx->last_poll_ms = now_ms;
    uint32_t late = 0u;
    if (!valid_elapsed(now_ms, ctx->next_due_ms, &late)) {
        ctx->last_result = LKM_FC100HZ_WAIT;
        return ctx->last_result;
    }
    const uint32_t slots_late = late / LKM_FC100HZ_PERIOD_MS;
    /* At each poll issue at most ONE slot, skip every already late slot. */
    const uint32_t slots_advanced = slots_late + 1u;
    ctx->last_deadline_ms = ctx->next_due_ms + slots_late * LKM_FC100HZ_PERIOD_MS;
    ctx->next_due_ms += slots_advanced * LKM_FC100HZ_PERIOD_MS;
    add_sat(&ctx->missed_slots, slots_late);
    const uint32_t current_lateness = late % LKM_FC100HZ_PERIOD_MS;
    if (current_lateness > ctx->max_lateness_ms) ctx->max_lateness_ms = current_lateness;
    if (slots_late != 0u || current_lateness > ctx->max_allowed_lateness_ms) {
        incr_sat(&ctx->late_events);
        incr_sat(&ctx->missed_slots); /* skipped current deadline */
        ctx->fault_latched = true;
        ctx->last_result = LKM_FC100HZ_LATE_DROP;
        return ctx->last_result;
    }
    ctx->work_in_flight = true;
    ctx->last_dispatch_ms = now_ms;
    incr_sat(&ctx->dispatch_count);
    ctx->last_result = LKM_FC100HZ_READY;
    return ctx->last_result;
}

lkm_fc100hz_result_t lkm_fc100hz_sched_finish(
    lkm_fc100hz_scheduler_t *ctx, uint32_t done_ms)
{
    if (ctx == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    if (!ctx->initialized || ctx->fault_latched || !ctx->work_in_flight) {
        ctx->last_result = LKM_FC100HZ_FAULT;
        return ctx->last_result;
    }
    ctx->work_in_flight = false;
    uint32_t work = 0u;
    if (!valid_elapsed(done_ms, ctx->last_dispatch_ms, &work) ||
        !valid_elapsed(done_ms, ctx->last_poll_ms, NULL)) {
        ctx->fault_latched = true;
        ctx->last_result = LKM_FC100HZ_FAULT;
        return ctx->last_result;
    }
    ctx->last_poll_ms = done_ms;
    if (work > ctx->max_work_ms_seen) ctx->max_work_ms_seen = work;
    if (work > ctx->max_allowed_work_ms ||
        !valid_elapsed(ctx->next_due_ms - 1u, done_ms, NULL)) {
        incr_sat(&ctx->overrun_events);
        ctx->fault_latched = true;
        ctx->last_result = LKM_FC100HZ_OVERRUN;
        return ctx->last_result;
    }
    ctx->last_result = LKM_FC100HZ_OK;
    return ctx->last_result;
}

void lkm_fc100hz_buffer_init(lkm_fc100hz_buffer_t *ctx)
{
    if (ctx != NULL) { memset(ctx, 0, sizeof(*ctx)); ctx->initialized = true; }
}

static lkm_fc100hz_result_t check_sample(
    const lkm_fc100hz_snapshot_t *sample, uint32_t now_ms)
{
    uint32_t age = 0u;
    if (sample == NULL || (!sample->bno_present && !sample->gyro_present))
        return LKM_FC100HZ_BAD_SAMPLE;
    if (sample->bno_present) {
        if (!sample->bno_counter || !isfinite(sample->roll_deg) ||
            !isfinite(sample->pitch_deg) || fabsf(sample->roll_deg) > 180.001f ||
            fabsf(sample->pitch_deg) > 90.001f) return LKM_FC100HZ_BAD_SAMPLE;
        if (!valid_elapsed(now_ms, sample->bno_host_received_ms, &age) ||
            age > LKM_FC100HZ_BNO_MAX_AGE_MS) return LKM_FC100HZ_STALE;
    }
    if (sample->gyro_present) {
        if (!sample->gyro_counter) return LKM_FC100HZ_BAD_SAMPLE;
        for (unsigned i = 0; i < 3u; i++) {
            if (!isfinite(sample->gyro_sensor_xyz_rad_s[i]) ||
                fabsf(sample->gyro_sensor_xyz_rad_s[i]) >
                LKM_FC100HZ_MAX_SENSOR_RATE_RAD_S) return LKM_FC100HZ_BAD_SAMPLE;
        }
        if (!valid_elapsed(now_ms, sample->gyro_host_received_ms, &age) ||
            age > LKM_FC100HZ_ICM_MAX_AGE_MS) return LKM_FC100HZ_STALE;
    }
    return LKM_FC100HZ_OK;
}

lkm_fc100hz_result_t lkm_fc100hz_buffer_push(
    lkm_fc100hz_buffer_t *ctx, const lkm_fc100hz_snapshot_t *sample, uint32_t now_ms)
{
    if (ctx == NULL || sample == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    if (!ctx->initialized) return LKM_FC100HZ_FAULT;
    lkm_fc100hz_result_t r = check_sample(sample, now_ms);
    if (r != LKM_FC100HZ_OK) {
        if (r == LKM_FC100HZ_STALE) incr_sat(&ctx->stale_count);
        else incr_sat(&ctx->bad_count);
        return r;
    }
    if ((sample->bno_present && ctx->bno_counter_seen &&
         !newer_counter(sample->bno_counter, ctx->last_bno_counter)) ||
        (sample->gyro_present && ctx->gyro_counter_seen &&
         !newer_counter(sample->gyro_counter, ctx->last_gyro_counter))) {
        incr_sat(&ctx->duplicate_count);
        return LKM_FC100HZ_DUPLICATE;
    }
    if (ctx->count >= LKM_FC100HZ_BUFFER_CAPACITY) {
        incr_sat(&ctx->overflow_count);
        return LKM_FC100HZ_BUFFER_FULL;
    }
    const uint8_t idx = (uint8_t)((ctx->head + ctx->count) % LKM_FC100HZ_BUFFER_CAPACITY);
    ctx->slots[idx] = *sample;
    /* Never carry data tagged as absent through the diagnostic buffer. */
    if (!sample->bno_present) {
        ctx->slots[idx].bno_host_received_ms = 0u;
        ctx->slots[idx].bno_counter = 0u;
        ctx->slots[idx].roll_deg = 0.0f;
        ctx->slots[idx].pitch_deg = 0.0f;
    }
    if (!sample->gyro_present) {
        ctx->slots[idx].gyro_host_received_ms = 0u;
        ctx->slots[idx].gyro_counter = 0u;
        memset(ctx->slots[idx].gyro_sensor_xyz_rad_s, 0,
               sizeof(ctx->slots[idx].gyro_sensor_xyz_rad_s));
    }
    ctx->slots[idx].stored_at_ms = now_ms;
    ctx->slots[idx].controller_ready = false;
    if (sample->bno_present) {
        ctx->last_bno_counter = sample->bno_counter;
        ctx->bno_counter_seen = true;
    }
    if (sample->gyro_present) {
        ctx->last_gyro_counter = sample->gyro_counter;
        ctx->gyro_counter_seen = true;
    }
    ++ctx->count;
    incr_sat(&ctx->accepted_count);
    return LKM_FC100HZ_OK;
}

lkm_fc100hz_result_t lkm_fc100hz_buffer_take_latest(
    lkm_fc100hz_buffer_t *ctx, uint32_t now_ms, lkm_fc100hz_snapshot_t *out)
{
    if (out == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    memset(out, 0, sizeof(*out));
    if (ctx == NULL) return LKM_FC100HZ_BAD_ARGUMENT;
    if (!ctx->initialized) return LKM_FC100HZ_FAULT;
    if (ctx->count == 0u) return LKM_FC100HZ_NO_SAMPLE;
    const uint8_t index = (uint8_t)((ctx->head + ctx->count - 1u) % LKM_FC100HZ_BUFFER_CAPACITY);
    lkm_fc100hz_snapshot_t sample = ctx->slots[index];
    if (ctx->count > 1u) add_sat(&ctx->dropped_older_on_latest_take,(uint32_t)ctx->count-1u);
    ctx->head = 0u;
    ctx->count = 0u;
    lkm_fc100hz_result_t r=check_sample(&sample,now_ms);
    if (r != LKM_FC100HZ_OK) {
        if (r == LKM_FC100HZ_STALE) incr_sat(&ctx->stale_count);
        else incr_sat(&ctx->bad_count);
        return r;
    }
    sample.controller_ready = false;
    *out = sample;
    return LKM_FC100HZ_OK;
}

const char *lkm_fc100hz_result_text(lkm_fc100hz_result_t r)
{
    switch(r) {
    case LKM_FC100HZ_OK: return "OK";
    case LKM_FC100HZ_WAIT: return "WAIT";
    case LKM_FC100HZ_READY: return "READY";
    case LKM_FC100HZ_LATE_DROP: return "LATE_DROP";
    case LKM_FC100HZ_OVERRUN: return "OVERRUN";
    case LKM_FC100HZ_FAULT: return "FAULT";
    case LKM_FC100HZ_BAD_ARGUMENT: return "BAD_ARGUMENT";
    case LKM_FC100HZ_BAD_CONFIG: return "BAD_CONFIG";
    case LKM_FC100HZ_NO_SAMPLE: return "NO_SAMPLE";
    case LKM_FC100HZ_DUPLICATE: return "DUPLICATE";
    case LKM_FC100HZ_STALE: return "STALE";
    case LKM_FC100HZ_BAD_SAMPLE: return "BAD_SAMPLE";
    case LKM_FC100HZ_BUFFER_FULL: return "BUFFER_FULL";
    default: return "UNKNOWN";
    }
}
