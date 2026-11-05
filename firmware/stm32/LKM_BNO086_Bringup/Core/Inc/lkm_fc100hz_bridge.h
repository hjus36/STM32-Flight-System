#ifndef LKM_FC100HZ_BRIDGE_H
#define LKM_FC100HZ_BRIDGE_H
/* Safe, optional STM32 application monitor. No timer IRQ, PWM, PID, ESC,
 * ICM SPI access, UART transmission, or hard real-time guarantee.
 * Called from the existing superloop only if installer --wire-main is used.
 */
#include "lkm_fc100hz_core.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool initialized;
    bool controller_ready; /* ALWAYS false */
    uint32_t bno_frames_copied;
    uint32_t bno_duplicates_skipped;
    uint32_t no_bno_at_tick;
    uint32_t drained_frames;
    uint32_t scheduler_faults;
    uint32_t ring_rejected;
    lkm_fc100hz_scheduler_t scheduler;
    lkm_fc100hz_buffer_t ring;
} lkm_fc100hz_bridge_status_t;
void LKM_FC100Hz_Bridge_Init(uint32_t start_ms);
void LKM_FC100Hz_Bridge_Service(void);
bool LKM_FC100Hz_Bridge_GetStatus(lkm_fc100hz_bridge_status_t *out);
#endif
