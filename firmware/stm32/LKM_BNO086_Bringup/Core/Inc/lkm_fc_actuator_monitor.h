#ifndef LKM_FC_ACTUATOR_MONITOR_H
#define LKM_FC_ACTUATOR_MONITOR_H
/* Read-only production guard monitor. Does NOT sample receiver, ADC, IMU,
 * invoke model step, generate PWM/Oneshot/DShot, or authorize flight. */
#include "lkm_fc_actuator_guard.h"

typedef struct {
    bool initialized;
    bool source_status_available;
    bool production_locked;            /* should remain true */
    bool upstream_hazard_latched;       /* diagnostic only */
    bool controller_ready;             /* ALWAYS false */
    bool esc_output_enabled;           /* ALWAYS false */
    uint32_t service_calls;
    uint32_t missing_status_calls;
    uint32_t hazard_events;
    lkm_act_state_t guard_state;
    lkm_act_result_t guard_result;
} lkm_act_monitor_status_t;

void LKM_FC_ActuatorMonitor_Init(void);
void LKM_FC_ActuatorMonitor_Service(void);
bool LKM_FC_ActuatorMonitor_GetStatus(lkm_act_monitor_status_t *out);
#endif
