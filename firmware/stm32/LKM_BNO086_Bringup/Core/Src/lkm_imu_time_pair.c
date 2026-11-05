#include "lkm_imu_time_pair.h"
#include <math.h>
#include <string.h>

static bool allowed_report(uint8_t report_id)
{
    return report_id == LKM_ATTITUDE_ROTATION_VECTOR ||
           report_id == LKM_ATTITUDE_GAME_ROTATION_VECTOR;
}

lkm_timepair_result_t lkm_timepair_init(
    lkm_timepair_t *ctx, const lkm_timepair_config_t *cfg)
{
    if (ctx == NULL || cfg == NULL) return LKM_TIMEPAIR_BAD_ARGUMENT;
    memset(ctx, 0, sizeof(*ctx));
    if (!allowed_report(cfg->expected_bno_report_id) ||
        cfg->minimum_bno_accuracy > 3u ||
        cfg->max_bno_age_ms == 0u || cfg->max_bno_age_ms > 60000u ||
        cfg->max_gyro_age_ms == 0u || cfg->max_gyro_age_ms > 60000u ||
        cfg->max_host_skew_ms == 0u || cfg->max_host_skew_ms > 60000u ||
        !isfinite(cfg->max_abs_gyro_rad_s) ||
        cfg->max_abs_gyro_rad_s <= 0.0f ||
        cfg->max_abs_gyro_rad_s > 100.0f)
        return LKM_TIMEPAIR_BAD_CONFIG;
    ctx->cfg = *cfg;
    ctx->initialized = true;
    return LKM_TIMEPAIR_OBSERVATION_OK;
}

lkm_timepair_result_t lkm_timepair_observe(
    lkm_timepair_t *ctx,
    const lkm_bno086_timed_attitude_t *bno,
    const lkm_icm20602_sample_t *gyro,
    uint32_t now_ms,
    lkm_timepair_observation_t *out)
{
    if (out == NULL) return LKM_TIMEPAIR_BAD_ARGUMENT;
    memset(out, 0, sizeof(*out));
    if (ctx == NULL) return LKM_TIMEPAIR_BAD_ARGUMENT;
    if (!ctx->initialized) return LKM_TIMEPAIR_NOT_INITIALIZED;
    if (bno == NULL || bno->accepted_packet_counter == 0u)
        return LKM_TIMEPAIR_MISSING_BNO;
    if (gyro == NULL || !gyro->valid || gyro->sample_sequence == 0u)
        return LKM_TIMEPAIR_MISSING_GYRO;
    if (bno->attitude.report_id != ctx->cfg.expected_bno_report_id ||
        bno->attitude.accuracy_status > 3u ||
        bno->attitude.accuracy_status < ctx->cfg.minimum_bno_accuracy ||
        !isfinite(bno->attitude.roll_deg) ||
        !isfinite(bno->attitude.pitch_deg) ||
        !isfinite(bno->attitude.yaw_deg) ||
        fabsf(bno->attitude.roll_deg) > 180.001f ||
        fabsf(bno->attitude.pitch_deg) > 90.001f ||
        fabsf(bno->attitude.yaw_deg) > 180.001f)
        return LKM_TIMEPAIR_BAD_BNO;
    for (unsigned i = 0u; i < 3u; ++i) {
        if (!isfinite(gyro->gyro_sensor_xyz_rad_s[i]) ||
            fabsf(gyro->gyro_sensor_xyz_rad_s[i]) > ctx->cfg.max_abs_gyro_rad_s)
            return LKM_TIMEPAIR_BAD_GYRO;
    }
    /* Unsigned subtraction handles HAL tick rollover as long as TTL is <<
     * 2^31. A future/backwards sample yields a very large age and fails. */
    const uint32_t bno_age = (uint32_t)(now_ms - bno->host_received_ms);
    const uint32_t gyro_age = (uint32_t)(now_ms - gyro->received_ms);
    if (bno_age > ctx->cfg.max_bno_age_ms)
        return LKM_TIMEPAIR_BNO_STALE;
    if (gyro_age > ctx->cfg.max_gyro_age_ms)
        return LKM_TIMEPAIR_GYRO_STALE;
    const int32_t diff = (int32_t)gyro_age - (int32_t)bno_age;
    const uint32_t skew = (uint32_t)(diff < 0 ? -diff : diff);
    if (skew > ctx->cfg.max_host_skew_ms)
        return LKM_TIMEPAIR_TIME_SKEW;
    if (ctx->have_last_pair &&
        (ctx->last_bno_packet_counter == bno->accepted_packet_counter ||
         ctx->last_gyro_sample_sequence == gyro->sample_sequence))
        return LKM_TIMEPAIR_DUPLICATE;

    out->bno = *bno;
    out->gyro_sensor = *gyro;
    out->inspected_at_ms = now_ms;
    out->bno_age_ms = bno_age;
    out->gyro_age_ms = gyro_age;
    out->bno_minus_gyro_receipt_ms = diff;
    out->controller_feedback_ready = false; /* NEVER authorize controller */
    ctx->last_bno_packet_counter = bno->accepted_packet_counter;
    ctx->last_gyro_sample_sequence = gyro->sample_sequence;
    ctx->have_last_pair = true;
    ++ctx->accepted_observations;
    return LKM_TIMEPAIR_OBSERVATION_OK;
}

const char *lkm_timepair_result_text(lkm_timepair_result_t result)
{
    switch (result) {
    case LKM_TIMEPAIR_OBSERVATION_OK: return "OBSERVATION_ONLY";
    case LKM_TIMEPAIR_BAD_ARGUMENT: return "BAD_ARGUMENT";
    case LKM_TIMEPAIR_BAD_CONFIG: return "BAD_CONFIG";
    case LKM_TIMEPAIR_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case LKM_TIMEPAIR_MISSING_BNO: return "MISSING_BNO";
    case LKM_TIMEPAIR_MISSING_GYRO: return "MISSING_GYRO";
    case LKM_TIMEPAIR_BAD_BNO: return "BAD_BNO";
    case LKM_TIMEPAIR_BAD_GYRO: return "BAD_GYRO";
    case LKM_TIMEPAIR_BNO_STALE: return "BNO_STALE";
    case LKM_TIMEPAIR_GYRO_STALE: return "GYRO_STALE";
    case LKM_TIMEPAIR_TIME_SKEW: return "HOST_SKEW";
    case LKM_TIMEPAIR_DUPLICATE: return "DUPLICATE";
    default: return "UNKNOWN";
    }
}
