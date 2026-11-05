#include "lkm_dual_imu_diag.h"
#include "lkm_icm20602_board_spi1.h"
#include "lkm_bno086_diag.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Separate diagnostic owner: never access the BNO086 internal device/SPI2. */
static lkm_icm20602_hal_spi_t s_adapter;
static lkm_icm20602_bus_t s_bus;
static lkm_icm_diag_core_t s_gyro;
static lkm_timepair_t s_timepair;
static bool s_timepair_ready;
static bool s_timepair_near_since_last_print;
static lkm_dual_imu_diag_status_t s_status;
static uint32_t s_last_status_at;
static uint32_t s_last_rate_at;
static uint32_t s_last_printed_gyro_seq;
static bool s_status_sent;
static lkm_icm_diag_state_t s_last_state;

#define DUAL_UART_BUFFER 176u
#define DUAL_STATUS_MS 1000u
#define DUAL_GYRO_MS 500u
#define DUAL_UART_TIMEOUT_MS 25u

static bool uart_send(const char *message)
{
    if (message == NULL) return false;
    const size_t n = strlen(message);
    if (n == 0u || n >= DUAL_UART_BUFFER || n > UINT16_MAX ||
        HAL_UART_Transmit(&huart6, (uint8_t *)message,
                          (uint16_t)n, DUAL_UART_TIMEOUT_MS) != HAL_OK) {
        ++s_status.uart_failures;
        return false;
    }
    return true;
}

static bool signed_centidps(float rad_s, int32_t *out)
{
    if (out == NULL || !isfinite(rad_s) || fabsf(rad_s) > 40.0f) return false;
    const float scaled = rad_s * 5729.577951308232f; /* rad/s to 0.01 dps */
    if (!isfinite(scaled) || fabsf(scaled) > 229184.0f) return false;
    *out = (int32_t)(scaled + ((scaled < 0.0f) ? -0.5f : 0.5f));
    return true;
}

static unsigned long magnitude_cdeg(int32_t n)
{
    return (unsigned long)((n < 0) ? -(int64_t)n : (int64_t)n);
}

void LKM_DualIMU_Diag_Init(bool enable_icm_spi_after_review)
{
    memset(&s_adapter, 0, sizeof(s_adapter));
    memset(&s_bus, 0, sizeof(s_bus));
    memset(&s_gyro, 0, sizeof(s_gyro));
    memset(&s_status, 0, sizeof(s_status));
    memset(&s_timepair, 0, sizeof(s_timepair));
    s_timepair_near_since_last_print = false;
    /* Matches reviewed BNO stream: rotation vector, 100Hz requested,
     * 50ms freshness, accuracy >= 1. ICM driver default TTL is 20ms.
     * Receipt skew <= 15ms is ONLY a diagnostic observation. */
    const lkm_timepair_config_t tcfg = {
        50u, 20u, 15u, 40.0f, LKM_ATTITUDE_ROTATION_VECTOR, 1u
    };
    s_timepair_ready = (lkm_timepair_init(&s_timepair, &tcfg) ==
                        LKM_TIMEPAIR_OBSERVATION_OK);
    s_status.timepair_last_result = LKM_TIMEPAIR_NOT_INITIALIZED;
    s_status.initialized = true;
    s_last_status_at = HAL_GetTick();
    s_last_rate_at = s_last_status_at;
    s_last_printed_gyro_seq = 0u;
    s_status_sent = false;
    s_last_state = LKM_ICM_DIAG_OFF;
    s_status.icm_state = LKM_ICM_DIAG_OFF;
    s_status.last_icm_result = LKM_ICM_ERR_NOT_CONFIGURED;
    /* Explicit software review gate (false by default in main). */
    if (!enable_icm_spi_after_review) return;
    const lkm_icm20602_board_result_t review = LKM_ICM20602_BoardSPI1_Prepare(
        &s_adapter, &s_bus, true, 10u);
    if (review != LKM_ICM_BOARD_OK) {
        s_status.icm_state = LKM_ICM_DIAG_FAULT;
        s_status.last_icm_result = LKM_ICM_ERR_CONFIG;
        return;
    }
    const lkm_icm20602_config_t config = LKM_ICM20602_DefaultConfig();
    if (!LKM_ICM_DiagCore_Init(&s_gyro, &s_bus, &config, 10u)) {
        s_status.icm_state = LKM_ICM_DIAG_FAULT;
        s_status.last_icm_result = LKM_ICM_ERR_CONFIG;
        return;
    }
    s_status.icm_enabled = true;
    /* First sensor SPI transfers occur ONLY after explicit true gate. */
    (void)LKM_ICM_DiagCore_Start(&s_gyro);
    s_status.icm_state = s_gyro.state;
    s_status.last_icm_result = s_gyro.last_result;
}

void LKM_DualIMU_Diag_Service(void)
{
    if (!s_status.initialized) return;
    const uint32_t now = HAL_GetTick();
    /* IMPORTANT: only BNO's already-verified diagnostic getter; no SPI2. */
    lkm_bno086_attitude_t pose = {0};
    s_status.bno_pose_valid = LKM_BNO086_Diag_TryGetAttitude(&pose);
    /* Both public BNO getters are read-only. Never poll SPI2 from here. */
    lkm_bno086_timed_attitude_t timed_pose = {0};
    s_status.bno_timed_valid =
        LKM_BNO086_Diag_TryGetTimedAttitude(&timed_pose);
    s_status.gyro_sensor_sample_valid = false;
    lkm_icm20602_sample_t gyro = {0};
    if (s_status.icm_enabled) {
        LKM_ICM_DiagCore_Service(&s_gyro);
        s_status.icm_state = s_gyro.state;
        s_status.last_icm_result = s_gyro.last_result;
        s_status.calibration_seen = s_gyro.gyro.calibration_seen;
        s_status.calibration_target = s_gyro.gyro.config.calibration_samples;
        s_status.gyro_new_samples = s_gyro.valid_new_samples;
        s_status.gyro_sensor_sample_valid = LKM_ICM_DiagCore_TryGet(&s_gyro,&gyro);
    }
    s_status.gyro_spi_transfers = s_adapter.successful_transactions;
    /* Perform pure-C proximity check after all getters: same HAL tick base.
     * Never expose a valid time-pair as permission to run control/PWM. */
    s_status.timepair_near_this_service = false;
    lkm_timepair_observation_t observation = {0};
    if (s_timepair_ready) {
        s_status.timepair_last_result = lkm_timepair_observe(
            &s_timepair,
            s_status.bno_timed_valid ? &timed_pose : NULL,
            s_status.gyro_sensor_sample_valid ? &gyro : NULL,
            HAL_GetTick(), &observation);
        if (s_status.timepair_last_result == LKM_TIMEPAIR_OBSERVATION_OK) {
            s_status.timepair_near_this_service = true;
            s_timepair_near_since_last_print = true;
            s_status.timepair_last_accepted_skew_ms =
                observation.bno_minus_gyro_receipt_ms;
        }
        s_status.timepair_accepted_total = s_timepair.accepted_observations;
    } else {
        s_status.timepair_last_result = LKM_TIMEPAIR_NOT_INITIALIZED;
    }
    s_status.controller_feedback_ready = false;
    const bool changed = !s_status_sent || s_last_state != s_status.icm_state;
    char line[DUAL_UART_BUFFER];
    if (changed || (uint32_t)(now-s_last_status_at) >= DUAL_STATUS_MS) {
        const int n = snprintf(line, sizeof(line),
            "[LKM IMU] STATUS t=%lu icm=%s bno=%s gyro=%s cal=%u/%u axis=UNVERIFIED sync=UNVERIFIED pid=OFF\r\n",
            (unsigned long)now, LKM_ICM_DiagCore_StateText(s_status.icm_state),
            s_status.bno_pose_valid ? "VALID" : "INVALID",
            s_status.gyro_sensor_sample_valid ? "FRESH" : "INVALID",
            (unsigned)s_status.calibration_seen,
            (unsigned)s_status.calibration_target);
        if (n > 0 && (size_t)n < sizeof(line)) uart_send(line);
        else ++s_status.uart_failures;
        s_last_status_at = now;
        s_last_state = s_status.icm_state;
        s_status_sent = true;

        /* A second bounded line shows missing/duplicate/stale/near status.
         * During the default false gate, ICM_OFF is explicit. The historical
         * skew is labeled as such and never stands for the current sample. */
        const char *pair_text = s_status.icm_enabled ?
            lkm_timepair_result_text(s_status.timepair_last_result) :
            (s_status.icm_state == LKM_ICM_DIAG_FAULT ?
             "ICM_FAULT" : "ICM_DISABLED");
        char skew_text[24] = "NA";
        if (s_status.timepair_accepted_total > 0u) {
            (void)snprintf(skew_text, sizeof(skew_text), "%ld",
                           (long)s_status.timepair_last_accepted_skew_ms);
        }
        const int n_time = snprintf(line, sizeof(line),
            "[LKM IMU] TIME t=%lu pair=%s bno_timed=%s gyro=%s new_near=%s accepted=%lu last_skew_ms=%s axis=UNVERIFIED sync=UNVERIFIED pid=OFF\r\n",
            (unsigned long)now, pair_text,
            s_status.bno_timed_valid ? "YES" : "NO",
            s_status.gyro_sensor_sample_valid ? "YES" : "NO",
            s_timepair_near_since_last_print ? "YES" : "NO",
            (unsigned long)s_status.timepair_accepted_total, skew_text);
        if (n_time > 0 && (size_t)n_time < sizeof(line)) {
            if (uart_send(line)) s_timepair_near_since_last_print = false;
        } else ++s_status.uart_failures;
    }
    if (!s_status.gyro_sensor_sample_valid ||
        (uint32_t)(now-s_last_rate_at) < DUAL_GYRO_MS ||
        gyro.sample_sequence == s_last_printed_gyro_seq) return;
    s_last_rate_at = now;
    int32_t x,y,z;
    if (!signed_centidps(gyro.gyro_sensor_xyz_rad_s[0], &x) ||
        !signed_centidps(gyro.gyro_sensor_xyz_rad_s[1], &y) ||
        !signed_centidps(gyro.gyro_sensor_xyz_rad_s[2], &z)) return;
    const unsigned long ax=magnitude_cdeg(x), ay=magnitude_cdeg(y), az=magnitude_cdeg(z);
    const int n = snprintf(line, sizeof(line),
        "[LKM ICM20602] GYRO t=%lu sx=%c%lu.%02lu sy=%c%lu.%02lu sz=%c%lu.%02lu dps seq=%lu frame=SENSOR\r\n",
        (unsigned long)gyro.received_ms,
        x<0?'-':'+',ax/100u,ax%100u,
        y<0?'-':'+',ay/100u,ay%100u,
        z<0?'-':'+',az/100u,az%100u,
        (unsigned long)gyro.sample_sequence);
    if (n > 0 && (size_t)n < sizeof(line)) {
        uart_send(line);
        s_last_printed_gyro_seq=gyro.sample_sequence;
    } else ++s_status.uart_failures;
}

bool LKM_DualIMU_Diag_GetStatus(lkm_dual_imu_diag_status_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!s_status.initialized) return false;
    *out = s_status;
    return true;
}
