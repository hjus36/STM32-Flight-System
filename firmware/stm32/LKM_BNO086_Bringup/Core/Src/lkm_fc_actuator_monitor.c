#include "lkm_fc_actuator_monitor.h"
#include "lkm_dual_imu_diag.h"
#include "lkm_fc100hz_bridge.h"
#include "lkm_aw_shadow.h"
#include "lkm_rp_frame_monitor.h"
#include <limits.h>
#include <string.h>

static lkm_act_guard_t s_guard;
static lkm_act_monitor_status_t s_status;
static void inc(uint32_t *v){if(*v!=UINT32_MAX)++*v;}

void LKM_FC_ActuatorMonitor_Init(void)
{
    memset(&s_guard,0,sizeof(s_guard));
    memset(&s_status,0,sizeof(s_status));
    const lkm_act_config_t cfg=LKM_FC_Actuator_DefaultConfig();
    const lkm_act_result_t result=LKM_FC_Actuator_Guard_Init(&s_guard,&cfg,false);
    s_status.initialized=s_guard.initialized;
    s_status.guard_state=s_guard.state;
    s_status.guard_result=result;
    s_status.production_locked=(result==LKM_ACT_ERR_PRODUCTION_LOCK &&
        s_guard.state==LKM_ACT_STATE_LOCKED);
}

void LKM_FC_ActuatorMonitor_Service(void)
{
    if(!s_status.initialized)return;
    inc(&s_status.service_calls);
    lkm_dual_imu_diag_status_t dual={0};
    lkm_fc100hz_bridge_status_t sched={0};
    lkm_aw_shadow_status_t shadow={0};
    lkm_rp_frame_monitor_status_t frame={0};
    const bool d=LKM_DualIMU_Diag_GetStatus(&dual);
    const bool t=LKM_FC100Hz_Bridge_GetStatus(&sched);
    const bool a=LKM_AW_Shadow_GetStatus(&shadow);
    const bool f=LKM_RP_FrameMonitor_GetStatus(&frame);
    s_status.source_status_available=d&&t&&a&&f;
    if(!s_status.source_status_available)inc(&s_status.missing_status_calls);
    /* Production must never convert upstream synthetic readiness or model N
       to an ESC/arming grant. We only record unexpected upstream hazards. */
    if((d && (dual.icm_enabled || dual.controller_feedback_ready)) ||
       (t && sched.controller_ready) ||
       (a && (shadow.controller_ready || shadow.esc_output_enabled ||
              shadow.virtual_test_enabled)) ||
       (f && (frame.controller_ready || frame.esc_output_enabled ||
              frame.hazard_latched)) ||
       s_guard.fixture_enabled || s_guard.state!=LKM_ACT_STATE_LOCKED) {
        if(!s_status.upstream_hazard_latched)inc(&s_status.hazard_events);
        s_status.upstream_hazard_latched=true;
    }
    s_status.guard_state=s_guard.state;
    s_status.guard_result=s_guard.last_result;
    s_status.production_locked=!s_status.upstream_hazard_latched &&
        s_guard.initialized && !s_guard.fixture_enabled &&
        s_guard.state==LKM_ACT_STATE_LOCKED;
    s_status.controller_ready=false;
    s_status.esc_output_enabled=false;
}

bool LKM_FC_ActuatorMonitor_GetStatus(lkm_act_monitor_status_t *out)
{
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(!s_status.initialized)return false;
    *out=s_status;
    out->controller_ready=false;
    out->esc_output_enabled=false;
    return true;
}
