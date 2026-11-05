#ifndef LKM_BNO086_DIAG_H
#define LKM_BNO086_DIAG_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_bno086_attitude.h"
#include "lkm_bno086_timestamp.h"
#include "lkm_bno086_link.h"

/* Diagnostic-only session. A single static BNO086 device owns SPI2.
 * Not a real-time flight-loop interface. No ESC/PWM/arming functions. */
typedef enum {
    LKM_BNO086_DIAG_UNINITIALIZED = 0,
    LKM_BNO086_DIAG_PROBE_FAILED,
    LKM_BNO086_DIAG_NEED_SET_FEATURE,
    LKM_BNO086_DIAG_NEED_GET_FEATURE,
    LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM,
    LKM_BNO086_DIAG_STREAMING,
    LKM_BNO086_DIAG_FAULT
} lkm_bno086_diag_state_t;

/* New diagnostic API: reuses existing Product ID probe. Link init is performed
 * only after a real successful probe; no Set/Get is transmitted here. */
void LKM_BNO086_Diag_Init(void);
bool LKM_BNO086_Diag_ProductIdVerified(void);

/* Call from a diagnostic main while(1), never ISR or 1-kHz PID timer.
 * At most ONE Link SPI transaction per call. Blocking: up to 50 ms on
 * transmit-ready wait, HAL SPI timeout per chunk on transfer. */
void LKM_BNO086_Diag_Service(void);

/* Returns false on unconfirmed feature, low quality, stale data or fault.
 * On false, the caller-provided output is NOT modified. */
bool LKM_BNO086_Diag_TryGetAttitude(lkm_bno086_attitude_t *out);


/* Read-only diagnostic snapshot. False = *out zeroed, never stale data.
 * Does not perform SPI/UART/GPIO operations or authorize PID/motor outputs.
 * Call after LKM_BNO086_Diag_Service in the cooperative diagnostic loop.
 */
bool LKM_BNO086_Diag_TryGetTimedAttitude(lkm_bno086_timed_attitude_t *out);

lkm_bno086_diag_state_t LKM_BNO086_Diag_GetState(void);
lkm_link_result_t LKM_BNO086_Diag_LastLinkResult(void);
uint32_t LKM_BNO086_Diag_ServiceCalls(void);

#endif /* LKM_BNO086_DIAG_H */
