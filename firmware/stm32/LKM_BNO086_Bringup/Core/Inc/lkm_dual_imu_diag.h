#ifndef LKM_DUAL_IMU_DIAG_H
#define LKM_DUAL_IMU_DIAG_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_icm20602_diag_core.h"
#include "lkm_imu_time_pair.h"

/* DIAGNOSTIC ONLY. The BNO086 app remains the sole SPI2 owner.
 * ICM is on dedicated SPI1. Never calls PID/PWM/motor/arming APIs.
 * Host-receipt proximity is OBSERVATION ONLY, not hardware synchronization.
 * No physical axis transform is approved and no controller is enabled.
 */
typedef struct {
    bool initialized;
    bool icm_enabled;
    lkm_icm_diag_state_t icm_state;
    lkm_icm20602_result_t last_icm_result;
    bool bno_pose_valid;
    bool gyro_sensor_sample_valid;
    bool controller_feedback_ready; /* ALWAYS false until separate verified integration */
    uint16_t calibration_seen;
    uint16_t calibration_target;
    uint32_t gyro_new_samples;
    uint32_t gyro_spi_transfers;
    uint32_t uart_failures;
    /* Read-only host receipt time telemetry; never an arming/feedback grant. */
    bool bno_timed_valid;
    bool timepair_near_this_service;
    lkm_timepair_result_t timepair_last_result;
    uint32_t timepair_accepted_total;
    int32_t timepair_last_accepted_skew_ms; /* historical, valid iff total>0 */
} lkm_dual_imu_diag_status_t;

/* false = SAFE default: NO SPI1 ICM operations, even when main is wired.
 * true = explicit schematic+CubeMX review approval only; does NOT approve
 * sensor mounting axes or authorize PID/motors. No hardware is exercised
 * by merely linking this code into an ELF.
 */
void LKM_DualIMU_Diag_Init(bool enable_icm_spi_after_review);
void LKM_DualIMU_Diag_Service(void);
bool LKM_DualIMU_Diag_GetStatus(lkm_dual_imu_diag_status_t *out);

#endif
