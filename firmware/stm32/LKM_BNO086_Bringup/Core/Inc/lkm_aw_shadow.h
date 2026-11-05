#ifndef LKM_AW_SHADOW_H
#define LKM_AW_SHADOW_H

/* Synthetic Simulink PID shadow evaluator. Never drives PWM/ESC/GPIO/UART.
 * PRODUCTION BUILD: fixture macro is absent and all virtual stepping is locked.
 * HOST TEST: compile with -DLKM_AW_SHADOW_HOST_FIXTURE=1 to permit synthetic
 * numerical evaluation of original generated LKM_AW_Core_F405 model.
 * This does not certify physical sensor axes, live timing or motor mapping.
 */
#include <stdbool.h>
#include <stdint.h>
#include "lkm_fc100hz_core.h"
#include "lkm_aw_core_preview.h"

typedef struct {
    bool initialized;
    bool virtual_test_enabled;
    bool controller_ready;      /* ALWAYS false, including fixture mode */
    bool esc_output_enabled;    /* ALWAYS false */
    bool last_preview_valid;    /* Synthetic-only; not a motor authorization */
    uint32_t model_steps;
    uint32_t scheduler_faults;
    uint32_t preview_failures;
    uint32_t no_step_poll_count;
    lkm_fc100hz_result_t last_scheduler_result;
    lkm_aw_preview_result_t last_preview_result;
    lkm_aw_preview_output_t last_preview;
} lkm_aw_shadow_status_t;

/* `request_virtual` is DENIED in production (returns false, output disabled).
 * It may be enabled only with the dedicated HOST fixture compile flag. */
bool LKM_AW_Shadow_Init(uint32_t logical_start_ms, bool request_virtual);
void LKM_AW_Shadow_Service(uint32_t logical_now_ms);
bool LKM_AW_Shadow_GetStatus(lkm_aw_shadow_status_t *out);

#endif
