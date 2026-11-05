#include "lkm_bno086_uart_diag.h"
#include "lkm_bno086_diag.h"
#include "usart.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* A slow, explicitly diagnostic console. HAL_UART_Transmit() is blocking.
 * Lower rate and short timeout limit interference but do NOT establish
 * real-time or measured 100 Hz SPI performance. */
#define LKM_UART_STATUS_INTERVAL_MS 1000u
#define LKM_UART_POSE_INTERVAL_MS    500u
#define LKM_UART_TX_TIMEOUT_MS       25u
#define LKM_UART_BUFFER_LENGTH      160u

static bool s_ready;
static bool s_has_status;
static uint32_t s_status_at_ms;
static uint32_t s_pose_at_ms;
static uint32_t s_uart_errors;
static lkm_bno086_diag_state_t s_last_state;

static const char *state_text(lkm_bno086_diag_state_t state)
{
    switch (state) {
    case LKM_BNO086_DIAG_UNINITIALIZED:        return "UNINITIALIZED";
    case LKM_BNO086_DIAG_PROBE_FAILED:         return "PROBE_FAILED";
    case LKM_BNO086_DIAG_NEED_SET_FEATURE:     return "NEED_SET";
    case LKM_BNO086_DIAG_NEED_GET_FEATURE:     return "NEED_GET";
    case LKM_BNO086_DIAG_WAIT_FEATURE_CONFIRM:return "WAIT_CONFIRM";
    case LKM_BNO086_DIAG_STREAMING:            return "STREAMING";
    case LKM_BNO086_DIAG_FAULT:                return "FAULT";
    default:                                  return "UNKNOWN";
    }
}

static void transmit_line(const char *s)
{
    if (s == NULL) return;
    const size_t n = strlen(s);
    if (n == 0u || n >= LKM_UART_BUFFER_LENGTH || n > UINT16_MAX) {
        ++s_uart_errors;
        return;
    }
    if (HAL_UART_Transmit(&huart6, (uint8_t *)s,
                          (uint16_t)n, LKM_UART_TX_TIMEOUT_MS) != HAL_OK) {
        ++s_uart_errors;
    }
}

/* Centidegrees preserve the minus sign for angles between -1 and 0 degrees,
 * and avoid newlib-nano's optional printf float support. */
static bool degrees_to_cdeg(float deg, int32_t *cdeg)
{
    if (cdeg == NULL || !isfinite(deg) || deg < -360.0f || deg > 360.0f)
        return false;
    const float scaled = deg * 100.0f;
    *cdeg = (int32_t)(scaled + ((scaled < 0.0f) ? -0.5f : 0.5f));
    return true;
}

void LKM_BNO086_UartDiag_Init(void)
{
    const uint32_t now = HAL_GetTick();
    s_ready = true;
    s_has_status = false;
    s_status_at_ms = now;
    s_pose_at_ms = now;
    s_uart_errors = 0u;
    s_last_state = LKM_BNO086_DIAG_UNINITIALIZED;
}

void LKM_BNO086_UartDiag_Service(void)
{
    if (!s_ready) return;
    const uint32_t now = HAL_GetTick();
    const lkm_bno086_diag_state_t state = LKM_BNO086_Diag_GetState();
    const bool status_due = !s_has_status || state != s_last_state ||
        (uint32_t)(now - s_status_at_ms) >= LKM_UART_STATUS_INTERVAL_MS;
    char line[LKM_UART_BUFFER_LENGTH];

    if (status_due) {
        lkm_bno086_attitude_t current;
        const bool valid = state == LKM_BNO086_DIAG_STREAMING &&
            LKM_BNO086_Diag_TryGetAttitude(&current);
        const char *pose = (state == LKM_BNO086_DIAG_STREAMING) ?
            (valid ? "VALID" : "INVALID") : "N/A";
        const int n = snprintf(line, sizeof(line),
            "[LKM BNO086] STATE t=%lu s=%s link=%s id=%u pose=%s calls=%lu\r\n",
            (unsigned long)now, state_text(state),
            lkm_bno086_link_result_text(LKM_BNO086_Diag_LastLinkResult()),
            LKM_BNO086_Diag_ProductIdVerified() ? 1u : 0u, pose,
            (unsigned long)LKM_BNO086_Diag_ServiceCalls());
        if (n > 0 && (size_t)n < sizeof(line)) {
            transmit_line(line);
        } else {
            ++s_uart_errors;
        }
        s_status_at_ms = now;
        s_last_state = state;
        s_has_status = true;
    }

    /* Never print cached angles from PROBE_FAILED/FAULT or an invalid/stale
     * sample. A failure from TryGetAttitude leaves current unmodified, and
     * no UART pose line is constructed in that case. */
    if (state != LKM_BNO086_DIAG_STREAMING ||
        (uint32_t)(now - s_pose_at_ms) < LKM_UART_POSE_INTERVAL_MS)
        return;
    s_pose_at_ms = now;

    lkm_bno086_attitude_t current;
    if (!LKM_BNO086_Diag_TryGetAttitude(&current)) return;

    int32_t r, p, y;
    if (!degrees_to_cdeg(current.roll_deg, &r) ||
        !degrees_to_cdeg(current.pitch_deg, &p) ||
        !degrees_to_cdeg(current.yaw_deg, &y))
        return;
    const uint32_t ra = (uint32_t)((r < 0) ? -r : r);
    const uint32_t pa = (uint32_t)((p < 0) ? -p : p);
    const uint32_t ya = (uint32_t)((y < 0) ? -y : y);
    const int n = snprintf(line, sizeof(line),
        "[LKM BNO086] POSE t=%lu R=%c%lu.%02lu P=%c%lu.%02lu Y=%c%lu.%02lu q=%u seq=%u rpt=0x%02X\r\n",
        (unsigned long)now,
        (r < 0) ? '-' : '+', (unsigned long)(ra / 100u), (unsigned long)(ra % 100u),
        (p < 0) ? '-' : '+', (unsigned long)(pa / 100u), (unsigned long)(pa % 100u),
        (y < 0) ? '-' : '+', (unsigned long)(ya / 100u), (unsigned long)(ya % 100u),
        (unsigned)current.accuracy_status, (unsigned)current.sequence,
        (unsigned)current.report_id);
    if (n > 0 && (size_t)n < sizeof(line)) {
        transmit_line(line);
    } else {
        ++s_uart_errors;
    }
}

uint32_t LKM_BNO086_UartDiag_TxErrors(void)
{
    return s_uart_errors;
}
