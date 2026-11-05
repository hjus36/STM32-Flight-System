#include "lkm_fc100hz_bridge.h"
#include "lkm_bno086_diag.h"
#include "lkm_bno086_timestamp.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <limits.h>

static lkm_fc100hz_bridge_status_t s_bridge;
static bool s_have_last_bno;
static uint32_t s_last_bno_counter;
static bool s_fault_counted;
static void incr_sat(uint32_t *n) { if (*n != UINT32_MAX) ++*n; }

void LKM_FC100Hz_Bridge_Init(uint32_t start_ms)
{
    memset(&s_bridge,0,sizeof(s_bridge));
    s_have_last_bno = false;
    s_last_bno_counter = 0u;
    s_fault_counted = false;
    lkm_fc100hz_buffer_init(&s_bridge.ring);
    /* Diagnostic-only cooperative timing. 1ms HAL tick resolution.
     * 2ms tolerated dispatch lateness, 5ms execution budget. */
    s_bridge.initialized = lkm_fc100hz_sched_init(&s_bridge.scheduler,
                                   start_ms,2u,5u)==LKM_FC100HZ_OK;
}

void LKM_FC100Hz_Bridge_Service(void)
{
    if (!s_bridge.initialized) return;
    const uint32_t now = HAL_GetTick();
    const lkm_fc100hz_result_t r = lkm_fc100hz_sched_poll(&s_bridge.scheduler, now);
    if (r != LKM_FC100HZ_READY) {
        if (!s_fault_counted && s_bridge.scheduler.fault_latched) {
            incr_sat(&s_bridge.scheduler_faults);
            s_fault_counted = true;
        }
        return;
    }
    /* READ-ONLY BNO getter. The ICM sensor remains disabled in main().
     * This bridge intentionally never fabricates gyro rates or body axes. */
    lkm_bno086_timed_attitude_t bno = {0};
    if (!LKM_BNO086_Diag_TryGetTimedAttitude(&bno)) {
        incr_sat(&s_bridge.no_bno_at_tick);
    } else if (s_have_last_bno && s_last_bno_counter == bno.accepted_packet_counter) {
        incr_sat(&s_bridge.bno_duplicates_skipped);
    } else {
        lkm_fc100hz_snapshot_t snap={0};
        snap.bno_present = true;
        snap.bno_host_received_ms = bno.host_received_ms;
        snap.bno_counter = bno.accepted_packet_counter;
        snap.roll_deg = bno.attitude.roll_deg;
        snap.pitch_deg = bno.attitude.pitch_deg;
        const lkm_fc100hz_result_t pushed =
            lkm_fc100hz_buffer_push(&s_bridge.ring,&snap,HAL_GetTick());
        if (pushed == LKM_FC100HZ_OK) {
            s_have_last_bno = true;
            s_last_bno_counter = snap.bno_counter;
            incr_sat(&s_bridge.bno_frames_copied);
        } else incr_sat(&s_bridge.ring_rejected);
    }
    /* Diagnostic consume-only. No data moves to Simulink or mixer. */
    lkm_fc100hz_snapshot_t discard={0};
    if (lkm_fc100hz_buffer_take_latest(&s_bridge.ring,HAL_GetTick(),&discard)
        == LKM_FC100HZ_OK) incr_sat(&s_bridge.drained_frames);
    if (lkm_fc100hz_sched_finish(&s_bridge.scheduler,HAL_GetTick()) != LKM_FC100HZ_OK &&
        !s_fault_counted) {
        incr_sat(&s_bridge.scheduler_faults);
        s_fault_counted = true;
    }
    s_bridge.controller_ready = false;
}

bool LKM_FC100Hz_Bridge_GetStatus(lkm_fc100hz_bridge_status_t *out)
{
    if (out == NULL) return false;
    memset(out,0,sizeof(*out));
    if (!s_bridge.initialized) return false;
    *out = s_bridge;
    out->controller_ready = false;
    return true;
}
