#ifndef LKM_RP_FEEDBACK_GATE_H
#define LKM_RP_FEEDBACK_GATE_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_bno086_attitude.h"

/* Observation/feedback input contract only. No Simulink controller invocation,
 * timer, GPIO, PWM, arming or ESC path. All units are explicit.
 * Body angular rates (p,q) are NOT Euler angle time derivatives.
 * Do not apply sign-only frame conversion to rotated/misaligned IMU boards. */
typedef enum {
    LKM_RP_GATE_OK = 0,
    LKM_RP_GATE_ERR_ARGUMENT,
    LKM_RP_GATE_ERR_CONFIG,
    LKM_RP_GATE_ERR_FRAME_UNVERIFIED,
    LKM_RP_GATE_ERR_ATTITUDE_UNAVAILABLE,
    LKM_RP_GATE_ERR_RATE_UNAVAILABLE,
    LKM_RP_GATE_ERR_REPORT,
    LKM_RP_GATE_ERR_QUALITY,
    LKM_RP_GATE_ERR_ANGLE,
    LKM_RP_GATE_ERR_RATE,
    LKM_RP_GATE_ERR_ATTITUDE_STALE,
    LKM_RP_GATE_ERR_RATE_STALE,
    LKM_RP_GATE_ERR_TIMESTAMP_SKEW,
    LKM_RP_GATE_ERR_REUSED_REPORT
} lkm_rp_gate_result_t;

typedef struct {
    /* Confirm BOTH board sensor axes and gyro axes physically before setting
     * these flags true. Initial bench configuration MUST be false/false. */
    bool angle_axes_verified;
    bool gyro_axes_verified;
    /* Sign-only maps assume roll/pitch sensing axes are ALREADY aligned.
     * Arbitrary mounting requires a quaternion/rotation-matrix transform. */
    int8_t roll_sign;
    int8_t pitch_sign;
    int8_t body_p_sign;
    int8_t body_q_sign;
    uint8_t expected_report_id;        /* 0x05 or 0x08; diagnostic uses 0x05 */
    uint8_t minimum_accuracy;          /* SH-2 status 0..3 */
    uint32_t max_attitude_age_ms;      /* compare against real stream RX time */
    uint32_t max_gyro_age_ms;          /* compare against real gyro RX time */
    uint32_t max_sample_skew_ms;       /* max |attitude_rx - gyro_rx| */
    float max_abs_roll_deg;            /* < 90 recommended around hover */
    float max_abs_pitch_deg;
    float max_abs_body_rate_rad_s;
} lkm_rp_gate_config_t;

typedef struct {
    /* attitude_ok must come from LKM_BNO086_Diag_TryGetAttitude() or verified
     * stream getter; DO NOT turn it true just because a pointer is non-NULL. */
    bool attitude_ok;
    const lkm_bno086_attitude_t *attitude;
    /* ORIGINAL host receive timestamp, not time of this function call.
     * Existing diagnostic getter does NOT expose this field yet. */
    uint32_t attitude_received_ms;
    /* Independent, physically calibrated gyro (e.g. future ICM-20602).
     * These are body rates p and q, in rad/s, already frame-aligned.
     * Until that driver is available, gyro_ok MUST be false. */
    bool gyro_ok;
    float gyro_body_p_rad_s;
    float gyro_body_q_rad_s;
    uint32_t gyro_received_ms;
} lkm_rp_gate_input_t;

typedef struct {
    bool feedback_valid;           /* feedback gate only, NOT flight/arm ready */
    float roll_angle_deg;
    float pitch_angle_deg;
    float roll_angle_rad;
    float pitch_angle_rad;
    float body_p_rad_s;
    float body_q_rad_s;
    uint32_t attitude_received_ms;
    uint32_t gyro_received_ms;
    uint8_t report_sequence;
    uint8_t accuracy_status;
} lkm_rp_feedback_t;

typedef struct {
    lkm_rp_gate_config_t config;
    bool initialized;
    bool has_last_report;
    uint8_t last_report_sequence;
    uint32_t accepted;
    uint32_t rejected;
} lkm_rp_gate_t;

lkm_rp_gate_result_t LKM_RP_Gate_Init(
    lkm_rp_gate_t *gate, const lkm_rp_gate_config_t *config);

/* On ALL failures: output is zeroed, feedback_valid=false. No stale data.
 * uint32_t timer wrapping is supported for intervals < 2^31 ms.
 * Even OK means only that inputs met configured checks, not that the FC is safe
 * for motors. Controller input correspondence must be independently verified. */
lkm_rp_gate_result_t LKM_RP_Gate_Evaluate(
    lkm_rp_gate_t *gate, const lkm_rp_gate_input_t *input,
    uint32_t now_ms, lkm_rp_feedback_t *out);

const char *LKM_RP_Gate_ResultText(lkm_rp_gate_result_t result);

#endif /* LKM_RP_FEEDBACK_GATE_H */
