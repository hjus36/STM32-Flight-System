#ifndef LKM_RP_FRAME_CONTRACT_H
#define LKM_RP_FRAME_CONTRACT_H

/* LKM_FC_V1: offline/bench sensor-to-body frame contract.
 * BNO086 quaternion: ASSUMED Hamilton active sensor->world (q_WS) ONLY in a
 * synthetic fixture. ICM gyro: sensor angular velocity in rad/s.
 * With R_BS (sensor->body), q_WB=q_WS * inverse(q_BS), and w_B=R_BS*w_S.
 * The actual SH-2 reference frame, sensor mounting, physical signs and
 * calibration must be measured on hardware before any flight approval.
 * Host receipt proximity is NOT common sensor sample-time synchronization.
 * This library contains no SPI, timer, PWM, UART, ESC or motor operations.
 * Production STM32F405 builds CANNOT enable physical-frame approval.
 */

#include <stdbool.h>
#include <stdint.h>
#include "lkm_imu_time_pair.h"
#include "lkm_rp_feedback_gate.h"

typedef enum {
    LKM_FRAME_OK = 0,                       /* SYNTHETIC HOST FIXTURE ONLY */
    LKM_FRAME_ERR_ARGUMENT,
    LKM_FRAME_ERR_CONFIG,
    LKM_FRAME_ERR_APPROVAL_LOCKED,
    LKM_FRAME_ERR_NOT_INITIALIZED,
    LKM_FRAME_ERR_UNVERIFIED,
    LKM_FRAME_ERR_PAIR,
    LKM_FRAME_ERR_OLD_PAIR,
    LKM_FRAME_ERR_QUATERNION,
    LKM_FRAME_ERR_EULER_MISMATCH,
    LKM_FRAME_ERR_GYRO,
    LKM_FRAME_ERR_GATE
} lkm_rp_frame_result_t;

typedef struct {
    /* Signed-permutation, proper rotation (det=+1) matrix, ROW-major. Each
     * entry -1/0/1; one nonzero in each row and column. No arbitrary angles.
     * For a general mount angle, use a separately calibrated SO(3) transform
     * rather than pretending it is a sign change. */
    int8_t m[3][3];
} lkm_rp_mount90_t;

typedef struct {
    lkm_rp_mount90_t bno_sensor_to_body;
    lkm_rp_mount90_t icm_sensor_to_body;
    /* Synthetic ONLY. Never infer true flags from 2 close host timestamps. */
    bool bno_mount_physically_verified;
    bool icm_mount_physically_verified;
    bool bno_quaternion_convention_verified;
    bool gyro_stationary_calibration_verified;
    uint8_t expected_bno_report_id;
    uint8_t min_accuracy;
    uint32_t max_bno_age_ms;
    uint32_t max_gyro_age_ms;
    uint32_t max_host_skew_ms;
    float max_abs_roll_deg;
    float max_abs_pitch_deg;
    float max_abs_body_rate_rad_s;
} lkm_rp_frame_config_t;

typedef struct {
    bool initialized;
    bool have_last_pair;
    lkm_rp_frame_config_t config;
    lkm_rp_gate_t gate;
    uint32_t last_bno_packet_counter;
    uint32_t last_icm_sample_sequence;
    uint32_t accepted;
    uint32_t rejected;
    lkm_rp_gate_result_t last_gate_result;
} lkm_rp_frame_contract_t;

typedef struct {
    /* NEVER actuator-ready. "feedback" accepted is synthetic fixture only. */
    bool diagnostic_valid;
    bool controller_ready;                  /* ALWAYS false */
    bool esc_output_enabled;                /* ALWAYS false */
    float body_gyro_xyz_rad_s[3];
    float body_roll_deg;
    float body_pitch_deg;
    float body_yaw_deg;
    uint32_t bno_received_ms;
    uint32_t icm_received_ms;
    uint32_t bno_packet_counter;
    uint32_t icm_sample_sequence;
    lkm_rp_feedback_t feedback;             /* synthetic fixture only */
} lkm_rp_frame_observation_t;

lkm_rp_frame_config_t LKM_RP_Frame_DefaultLocked(void);
bool LKM_RP_Mount90_IsProper(const lkm_rp_mount90_t *m);
lkm_rp_frame_result_t LKM_RP_Frame_Init(
    lkm_rp_frame_contract_t *ctx, const lkm_rp_frame_config_t *config);
/* Input must come from a successful lkm_timepair_observe() result.
 * Further checking NEVER turns host receipt proximity into true sync.
 * On every failure: whole *out zeroed; stale feedback not reused.
 * In production, returns UNVERIFIED/APPROVAL_LOCKED and never exposes a
 * diagnostic_valid pose or a controller-ready output.
 */
lkm_rp_frame_result_t LKM_RP_Frame_Observe(
    lkm_rp_frame_contract_t *ctx, const lkm_timepair_observation_t *pair,
    uint32_t now_ms, lkm_rp_frame_observation_t *out);
const char *LKM_RP_Frame_ResultText(lkm_rp_frame_result_t result);

#endif
