#include "lkm_aw_shadow.h"
#include <string.h>
#include <limits.h>

static lkm_aw_shadow_status_t s_status;
#if defined(LKM_AW_SHADOW_HOST_FIXTURE) && LKM_AW_SHADOW_HOST_FIXTURE
static lkm_fc100hz_scheduler_t s_sched;
static lkm_aw_preview_t s_preview;
static uint8_t s_fixture_seq;
static void incr_sat(uint32_t *n) { if (*n != UINT32_MAX) ++*n; }
static void fail_closed(void)
{
    s_status.virtual_test_enabled = false;
    s_status.last_preview_valid = false;
    memset(&s_status.last_preview, 0, sizeof(s_status.last_preview));
    LKM_AW_Preview_Invalidate(&s_preview);
}
#endif

bool LKM_AW_Shadow_Init(uint32_t logical_start_ms, bool request_virtual)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.initialized = true;
    s_status.controller_ready = false;
    s_status.esc_output_enabled = false;
    s_status.last_scheduler_result = LKM_FC100HZ_WAIT;
    s_status.last_preview_result = LKM_AW_PREVIEW_ERR_NOT_INITIALIZED;
#if defined(LKM_AW_SHADOW_HOST_FIXTURE) && LKM_AW_SHADOW_HOST_FIXTURE
    memset(&s_sched, 0, sizeof(s_sched));
    memset(&s_preview, 0, sizeof(s_preview));
    s_fixture_seq = 0u;
    if (!request_virtual) return true;
    if (lkm_fc100hz_sched_init(&s_sched, logical_start_ms, 2u, 5u)
        != LKM_FC100HZ_OK) return false;
    /* Fixture-only: synthetic, internally aligned axes. Never sensor approval. */
    const lkm_aw_preview_config_t cfg = { true, 10u, 0.5f, 8.0f };
    if (LKM_AW_Preview_Init(&s_preview, &cfg) != LKM_AW_PREVIEW_OK) return false;
    s_status.virtual_test_enabled = true;
    return true;
#else
    (void)logical_start_ms;
    /* Compile-time hard lock: even if caller supplies true, no synthetic
     * rates, model steps or generated motor numerics can be produced. */
    if (request_virtual) return false;
    return true;
#endif
}

void LKM_AW_Shadow_Service(uint32_t logical_now_ms)
{
#if defined(LKM_AW_SHADOW_HOST_FIXTURE) && LKM_AW_SHADOW_HOST_FIXTURE
    if (!s_status.initialized || !s_status.virtual_test_enabled) return;
    const lkm_fc100hz_result_t r = lkm_fc100hz_sched_poll(&s_sched, logical_now_ms);
    s_status.last_scheduler_result = r;
    if (r == LKM_FC100HZ_WAIT) { incr_sat(&s_status.no_step_poll_count); return; }
    if (r != LKM_FC100HZ_READY) {
        incr_sat(&s_status.scheduler_faults);
        fail_closed();
        return;
    }
    /* Every angle/rate below is SYNTHETIC and explicitly independent of
     * BNO086, ICM20602, receiver and motor driver. No actuator API exists. */
    lkm_rp_feedback_t fb = {0};
    lkm_aw_preview_command_t cmd = {0};
    fb.feedback_valid = true;
    fb.roll_angle_rad = 0.02f;
    fb.roll_angle_deg = 0.02f * 57.29577951308232f;
    fb.pitch_angle_rad = -0.01f;
    fb.pitch_angle_deg = -0.01f * 57.29577951308232f;
    fb.body_p_rad_s = 0.03f;
    fb.body_q_rad_s = -0.02f;
    fb.report_sequence = ++s_fixture_seq;
    cmd.command_valid = true;
    cmd.target_roll_angle_rad = 0.1f;
    cmd.target_pitch_angle_rad = -0.08f;
    cmd.collective_thrust_n = 4.0f;
    lkm_aw_preview_output_t output = {0};
    const lkm_aw_preview_result_t p = LKM_AW_Preview_Tick(
        &s_preview, &fb, &cmd, logical_now_ms, &output);
    s_status.last_preview_result = p;
    if (p != LKM_AW_PREVIEW_OK || !output.valid) {
        incr_sat(&s_status.preview_failures);
        fail_closed();
    } else {
        s_status.last_preview = output;
        s_status.last_preview_valid = true;
        incr_sat(&s_status.model_steps);
    }
    const lkm_fc100hz_result_t done = lkm_fc100hz_sched_finish(&s_sched,logical_now_ms);
    s_status.last_scheduler_result = done;
    if (done != LKM_FC100HZ_OK) {
        incr_sat(&s_status.scheduler_faults);
        fail_closed();
    }
    /* No test outcome may be interpreted as flight-control readiness. */
    s_status.controller_ready = false;
    s_status.esc_output_enabled = false;
#else
    (void)logical_now_ms;
#endif
}

bool LKM_AW_Shadow_GetStatus(lkm_aw_shadow_status_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!s_status.initialized) return false;
    *out = s_status;
    out->controller_ready = false;
    out->esc_output_enabled = false;
    return true;
}
