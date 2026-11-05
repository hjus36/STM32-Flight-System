#include "lkm_rp_feedback_gate.h"
#include <math.h>
#include <string.h>

#define LKM_RP_D2R (0.01745329251994329577f)
#define LKM_RP_MAX_SAFE_INTERVAL (0x7FFFFFFFu)

static bool is_sign(int8_t value) { return value == 1 || value == -1; }

static bool config_ok(const lkm_rp_gate_config_t *c)
{
    if (c == NULL || !is_sign(c->roll_sign) || !is_sign(c->pitch_sign) ||
        !is_sign(c->body_p_sign) || !is_sign(c->body_q_sign) ||
        (c->expected_report_id != LKM_ATTITUDE_ROTATION_VECTOR &&
         c->expected_report_id != LKM_ATTITUDE_GAME_ROTATION_VECTOR) ||
        c->minimum_accuracy > 3u || c->max_attitude_age_ms == 0u ||
        c->max_gyro_age_ms == 0u || c->max_sample_skew_ms > LKM_RP_MAX_SAFE_INTERVAL ||
        c->max_attitude_age_ms > LKM_RP_MAX_SAFE_INTERVAL ||
        c->max_gyro_age_ms > LKM_RP_MAX_SAFE_INTERVAL ||
        !isfinite(c->max_abs_roll_deg) || !isfinite(c->max_abs_pitch_deg) ||
        !isfinite(c->max_abs_body_rate_rad_s) ||
        c->max_abs_roll_deg <= 0.0f || c->max_abs_roll_deg > 180.0f ||
        c->max_abs_pitch_deg <= 0.0f || c->max_abs_pitch_deg > 90.0f ||
        c->max_abs_body_rate_rad_s <= 0.0f)
        return false;
    return true;
}

lkm_rp_gate_result_t LKM_RP_Gate_Init(
    lkm_rp_gate_t *gate, const lkm_rp_gate_config_t *config)
{
    if (gate == NULL || config == NULL) return LKM_RP_GATE_ERR_ARGUMENT;
    /* Explicit reset: a failed initialization cannot leave a previous gate
     * initialized with old calibration/verification state. */
    memset(gate, 0, sizeof(*gate));
    if (!config_ok(config)) return LKM_RP_GATE_ERR_CONFIG;
    gate->config = *config;
    gate->initialized = true;
    return LKM_RP_GATE_OK;
}

static bool age_ok(uint32_t now, uint32_t received, uint32_t limit)
{
    const uint32_t age = (uint32_t)(now - received);
    return age <= limit && age <= LKM_RP_MAX_SAFE_INTERVAL;
}

static bool skew_ok(uint32_t a, uint32_t b, uint32_t limit)
{
    const uint32_t d = (uint32_t)(a - b);
    if (d == UINT32_C(0x80000000)) return false;
    const uint32_t abs_d = (d <= LKM_RP_MAX_SAFE_INTERVAL) ? d : (uint32_t)(0u - d);
    return abs_d <= limit;
}

lkm_rp_gate_result_t LKM_RP_Gate_Evaluate(
    lkm_rp_gate_t *gate, const lkm_rp_gate_input_t *input,
    uint32_t now_ms, lkm_rp_feedback_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (gate == NULL || input == NULL || out == NULL) return LKM_RP_GATE_ERR_ARGUMENT;
    if (!gate->initialized || !config_ok(&gate->config))
        return LKM_RP_GATE_ERR_CONFIG;

    /* Sensor frame sign is NOT physically proven by off-line fixtures. */
    if (!gate->config.angle_axes_verified || !gate->config.gyro_axes_verified) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_FRAME_UNVERIFIED;
    }
    if (!input->attitude_ok || input->attitude == NULL) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_ATTITUDE_UNAVAILABLE;
    }
    if (!input->gyro_ok) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_RATE_UNAVAILABLE;
    }

    const lkm_bno086_attitude_t *a = input->attitude;
    if (a->report_id != gate->config.expected_report_id) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_REPORT;
    }
    if (a->accuracy_status > 3u ||
        a->accuracy_status < gate->config.minimum_accuracy) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_QUALITY;
    }

    if (!age_ok(now_ms, input->attitude_received_ms,
                gate->config.max_attitude_age_ms)) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_ATTITUDE_STALE;
    }
    if (!age_ok(now_ms, input->gyro_received_ms,
                gate->config.max_gyro_age_ms)) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_RATE_STALE;
    }
    if (!skew_ok(input->attitude_received_ms, input->gyro_received_ms,
                 gate->config.max_sample_skew_ms)) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_TIMESTAMP_SKEW;
    }
    if (!isfinite(a->roll_deg) || !isfinite(a->pitch_deg) ||
        fabsf(a->roll_deg) > gate->config.max_abs_roll_deg ||
        fabsf(a->pitch_deg) > gate->config.max_abs_pitch_deg) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_ANGLE;
    }
    if (!isfinite(input->gyro_body_p_rad_s) ||
        !isfinite(input->gyro_body_q_rad_s) ||
        fabsf(input->gyro_body_p_rad_s) > gate->config.max_abs_body_rate_rad_s ||
        fabsf(input->gyro_body_q_rad_s) > gate->config.max_abs_body_rate_rad_s) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_RATE;
    }
    if (gate->has_last_report && a->sequence == gate->last_report_sequence) {
        ++gate->rejected;
        return LKM_RP_GATE_ERR_REUSED_REPORT;
    }

    lkm_rp_feedback_t candidate = {0};
    candidate.roll_angle_deg = a->roll_deg * (float)gate->config.roll_sign;
    candidate.pitch_angle_deg = a->pitch_deg * (float)gate->config.pitch_sign;
    candidate.roll_angle_rad = candidate.roll_angle_deg * LKM_RP_D2R;
    candidate.pitch_angle_rad = candidate.pitch_angle_deg * LKM_RP_D2R;
    candidate.body_p_rad_s = input->gyro_body_p_rad_s * (float)gate->config.body_p_sign;
    candidate.body_q_rad_s = input->gyro_body_q_rad_s * (float)gate->config.body_q_sign;
    candidate.attitude_received_ms = input->attitude_received_ms;
    candidate.gyro_received_ms = input->gyro_received_ms;
    candidate.report_sequence = a->sequence;
    candidate.accuracy_status = a->accuracy_status;
    candidate.feedback_valid = true;

    *out = candidate;
    gate->last_report_sequence = a->sequence;
    gate->has_last_report = true;
    ++gate->accepted;
    return LKM_RP_GATE_OK;
}

const char *LKM_RP_Gate_ResultText(lkm_rp_gate_result_t result)
{
    switch (result) {
    case LKM_RP_GATE_OK: return "OK";
    case LKM_RP_GATE_ERR_ARGUMENT: return "ARGUMENT";
    case LKM_RP_GATE_ERR_CONFIG: return "CONFIG";
    case LKM_RP_GATE_ERR_FRAME_UNVERIFIED: return "FRAME_UNVERIFIED";
    case LKM_RP_GATE_ERR_ATTITUDE_UNAVAILABLE: return "ATTITUDE_UNAVAILABLE";
    case LKM_RP_GATE_ERR_RATE_UNAVAILABLE: return "RATE_UNAVAILABLE";
    case LKM_RP_GATE_ERR_REPORT: return "REPORT";
    case LKM_RP_GATE_ERR_QUALITY: return "QUALITY";
    case LKM_RP_GATE_ERR_ANGLE: return "ANGLE";
    case LKM_RP_GATE_ERR_RATE: return "RATE";
    case LKM_RP_GATE_ERR_ATTITUDE_STALE: return "ATTITUDE_STALE";
    case LKM_RP_GATE_ERR_RATE_STALE: return "RATE_STALE";
    case LKM_RP_GATE_ERR_TIMESTAMP_SKEW: return "TIMESTAMP_SKEW";
    case LKM_RP_GATE_ERR_REUSED_REPORT: return "REUSED_REPORT";
    default: return "UNKNOWN";
    }
}
