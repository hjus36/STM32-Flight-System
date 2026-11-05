#ifndef LKM_IMU_TIME_PAIR_H
#define LKM_IMU_TIME_PAIR_H

/* Offline, diagnostic-only host-receipt time proximity check.
 * BNO086 is SPI2; ICM-20602 is SPI1. Neither sensor is accessed here.
 * Time proximity is NOT sensor hardware synchronization, coordinate-frame
 * agreement, or permission to run PID/ESC/motors.
 */
#include <stdbool.h>
#include <stdint.h>
#include "lkm_bno086_timestamp.h"
#include "lkm_icm20602_gyro.h"

typedef enum {
    LKM_TIMEPAIR_OBSERVATION_OK = 0,
    LKM_TIMEPAIR_BAD_ARGUMENT,
    LKM_TIMEPAIR_BAD_CONFIG,
    LKM_TIMEPAIR_NOT_INITIALIZED,
    LKM_TIMEPAIR_MISSING_BNO,
    LKM_TIMEPAIR_MISSING_GYRO,
    LKM_TIMEPAIR_BAD_BNO,
    LKM_TIMEPAIR_BAD_GYRO,
    LKM_TIMEPAIR_BNO_STALE,
    LKM_TIMEPAIR_GYRO_STALE,
    LKM_TIMEPAIR_TIME_SKEW,
    LKM_TIMEPAIR_DUPLICATE
} lkm_timepair_result_t;

typedef struct {
    uint32_t max_bno_age_ms;       /* conservative diagnostic TTL e.g. 50 */
    uint32_t max_gyro_age_ms;      /* e.g. 20 */
    uint32_t max_host_skew_ms;     /* e.g. 15 */
    float max_abs_gyro_rad_s;      /* e.g. 40 */
    uint8_t expected_bno_report_id; /* 0x05 or 0x08 */
    uint8_t minimum_bno_accuracy;   /* 0..3 */
} lkm_timepair_config_t;

typedef struct {
    lkm_bno086_timed_attitude_t bno;
    lkm_icm20602_sample_t gyro_sensor;
    uint32_t inspected_at_ms;
    uint32_t bno_age_ms;
    uint32_t gyro_age_ms;
    int32_t bno_minus_gyro_receipt_ms;
    bool controller_feedback_ready; /* HARD FALSE; sensor axes unverified */
} lkm_timepair_observation_t;

typedef struct {
    bool initialized;
    bool have_last_pair;
    uint32_t last_bno_packet_counter;
    uint32_t last_gyro_sample_sequence;
    uint32_t accepted_observations;
    lkm_timepair_config_t cfg;
} lkm_timepair_t;

/* Pure C. Configuration and payload are independent of STM32 HAL. */
lkm_timepair_result_t lkm_timepair_init(
    lkm_timepair_t *ctx, const lkm_timepair_config_t *cfg);

/* Input BNO *must* be obtained with the timed diagnostic getter above and
 * ICM must be a calibrated, valid sample from its driver. For host tests,
 * synthetic samples may be supplied but are NOT hardware verification.
 * Both host stamps must originate from the same HAL_GetTick timebase.
 * On any non-OK outcome, *out is zeroed (no cached pose/rate reuse).
 * Distinct BNO and ICM sample counters are required for every new pair.
 */
lkm_timepair_result_t lkm_timepair_observe(
    lkm_timepair_t *ctx,
    const lkm_bno086_timed_attitude_t *bno,
    const lkm_icm20602_sample_t *gyro,
    uint32_t host_now_ms,
    lkm_timepair_observation_t *out);

const char *lkm_timepair_result_text(lkm_timepair_result_t result);

#endif /* LKM_IMU_TIME_PAIR_H */
