#ifndef LKM_RP_FRAME_MONITOR_H
#define LKM_RP_FRAME_MONITOR_H
/* Optional, read-only superloop monitor for LKM_FC_V1. NO SPI/UART/ESC/PWM.
 * It only observes existing diagnostic status, and deliberately never requests
 * frame approval or calls Simulink model step / feedback controller. */
#include <stdbool.h>
#include <stdint.h>
#include "lkm_rp_frame_contract.h"

typedef struct {
    bool initialized;
    bool sources_observed;
    bool safe_locked;            /* TRUE when no unexpected approval observed */
    bool hazard_latched;
    bool controller_ready;       /* ALWAYS false */
    bool esc_output_enabled;     /* ALWAYS false */
    bool bno_reported_valid;     /* NOT an axis approval */
    bool icm_spi_enabled;        /* MUST remain false in this firmware */
    uint32_t service_calls;
    uint32_t missing_status_calls;
    uint32_t hazard_events;
    lkm_rp_frame_result_t last_frame_result;
} lkm_rp_frame_monitor_status_t;

void LKM_RP_FrameMonitor_Init(void);
void LKM_RP_FrameMonitor_Service(void);
bool LKM_RP_FrameMonitor_GetStatus(lkm_rp_frame_monitor_status_t *out);
#endif
