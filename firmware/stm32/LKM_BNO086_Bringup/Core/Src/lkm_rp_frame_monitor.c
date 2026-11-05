#include "lkm_rp_frame_monitor.h"
#include "lkm_dual_imu_diag.h"
#include "lkm_fc100hz_bridge.h"
#include "lkm_aw_shadow.h"
#include <limits.h>
#include <string.h>

static lkm_rp_frame_monitor_status_t s_state;
static lkm_rp_frame_contract_t s_contract;
static void incr(uint32_t *n){if(*n!=UINT32_MAX)++*n;}

void LKM_RP_FrameMonitor_Init(void)
{
    memset(&s_state,0,sizeof(s_state));
    memset(&s_contract,0,sizeof(s_contract));
    const lkm_rp_frame_config_t locked=LKM_RP_Frame_DefaultLocked();
    s_state.initialized=LKM_RP_Frame_Init(&s_contract,&locked)==LKM_FRAME_OK;
    s_state.safe_locked=s_state.initialized;
    s_state.last_frame_result=LKM_FRAME_ERR_UNVERIFIED;
}

void LKM_RP_FrameMonitor_Service(void)
{
    if(!s_state.initialized)return;
    incr(&s_state.service_calls);
    lkm_dual_imu_diag_status_t dual={0};
    lkm_fc100hz_bridge_status_t cycle={0};
    lkm_aw_shadow_status_t shadow={0};
    const bool d=LKM_DualIMU_Diag_GetStatus(&dual);
    const bool c=LKM_FC100Hz_Bridge_GetStatus(&cycle);
    const bool a=LKM_AW_Shadow_GetStatus(&shadow);
    s_state.sources_observed=d&&c&&a;
    if(!s_state.sources_observed)incr(&s_state.missing_status_calls);
    s_state.bno_reported_valid=d&&dual.bno_pose_valid;
    s_state.icm_spi_enabled=d&&dual.icm_enabled;

    /* Deliberately no data may be transformed to controller feedback in a
     * production build. Observe() returns UNVERIFIED before touching pair. */
    lkm_rp_frame_observation_t ignored={0};
    s_state.last_frame_result=LKM_RP_Frame_Observe(&s_contract,NULL,0u,&ignored);
    if(s_state.last_frame_result!=LKM_FRAME_ERR_UNVERIFIED ||
       ignored.diagnostic_valid || ignored.controller_ready ||
       ignored.esc_output_enabled ||
       (d&&(dual.icm_enabled||dual.controller_feedback_ready)) ||
       (c&&cycle.controller_ready) ||
       (a&&(shadow.controller_ready||shadow.esc_output_enabled||
            shadow.virtual_test_enabled))) {
        s_state.hazard_latched=true;
        incr(&s_state.hazard_events);
    }
    s_state.safe_locked=!s_state.hazard_latched &&
        s_state.last_frame_result==LKM_FRAME_ERR_UNVERIFIED;
    s_state.controller_ready=false;
    s_state.esc_output_enabled=false;
}

bool LKM_RP_FrameMonitor_GetStatus(lkm_rp_frame_monitor_status_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!s_state.initialized)return false;
    *out=s_state;
    out->controller_ready=false;
    out->esc_output_enabled=false;
    return true;
}
