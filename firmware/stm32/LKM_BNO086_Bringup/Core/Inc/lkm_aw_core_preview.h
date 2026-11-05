#ifndef LKM_AW_CORE_PREVIEW_H
#define LKM_AW_CORE_PREVIEW_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_rp_feedback_gate.h"

/* BENCH/OFFLINE-ONLY. The returned thrust/torque figures are generated-model
 * diagnostics, NOT calibrated ESC commands, and NEVER an arming permission.
 * No GPIO, timer, UART, motor or ESC access occurs in this module.
 * 100 Hz model: exactly one LKM_AW_Core_F405_step per accepted 10 ms tick.
 * Caller must separately verify coordinate frames, motor mapping, scheduling,
 * sensor timestamps, receiver fail-safe, and actual thrust limits. */
typedef enum {
    LKM_AW_PREVIEW_OK = 0,
    LKM_AW_PREVIEW_ERR_ARGUMENT,
    LKM_AW_PREVIEW_ERR_NOT_INITIALIZED,
    LKM_AW_PREVIEW_ERR_CONFIG,
    LKM_AW_PREVIEW_ERR_AXES_UNVERIFIED,
    LKM_AW_PREVIEW_ERR_FEEDBACK,
    LKM_AW_PREVIEW_ERR_COMMAND,
    LKM_AW_PREVIEW_ERR_EARLY_TICK,
    LKM_AW_PREVIEW_ERR_REUSED_FEEDBACK,
    LKM_AW_PREVIEW_ERR_MISSED_TICK,
    LKM_AW_PREVIEW_ERR_MODEL
} lkm_aw_preview_result_t;

typedef struct {
    /* Only true for explicitly aligned synthetic test fixtures or after
     * independently verified physical sensor+controller coordinate frames. */
    bool coordinate_contract_verified;
    uint32_t step_period_ms;              /* For this model: 10 */
    float max_abs_target_angle_rad;       /* TEST-ONLY envelope, not flight limit */
    float max_preview_collective_n;       /* TEST-ONLY envelope, not motor limit */
} lkm_aw_preview_config_t;

typedef struct {
    bool command_valid;                   /* External command source must validate */
    float target_roll_angle_rad;
    float target_pitch_angle_rad;
    float collective_thrust_n;            /* N; never transformed to ESC drive */
} lkm_aw_preview_command_t;

typedef struct {
    bool valid;                           /* ONLY: offline model preview valid */
    uint32_t step_tick_ms;
    float actual_roll_angle_rad;
    float actual_roll_rate_rad_s;
    float actual_pitch_angle_rad;
    float actual_pitch_rate_rad_s;
    float target_roll_angle_rad;
    float target_pitch_angle_rad;
    float collective_thrust_n;
    /* These are the genuine generated model's raw outputs, never motor commands. */
    float motor_1_thrust_preview_n;
    float motor_2_thrust_preview_n;
    float motor_3_thrust_preview_n;
    float motor_4_thrust_preview_n;
    float requested_roll_torque_nm;
    float requested_pitch_torque_nm;
    float achieved_roll_torque_nm;
    float achieved_pitch_torque_nm;
    float target_roll_rate_rad_s;
    float target_pitch_rate_rad_s;
} lkm_aw_preview_output_t;

typedef struct {
    lkm_aw_preview_config_t config;
    bool initialized;
    bool require_model_reset;
    bool has_last_accepted_step;
    uint32_t last_accepted_step_ms;
    bool has_last_report;
    uint8_t last_report_sequence;
    uint32_t accepted_steps;
    uint32_t blocked_steps;
    uint32_t model_resets;
} lkm_aw_preview_t;

/* Init does not invoke model step; deliberate fresh initialization is required. */
lkm_aw_preview_result_t LKM_AW_Preview_Init(
    lkm_aw_preview_t *preview, const lkm_aw_preview_config_t *config);

/* No generated model step if feedback, command, timing or frame checks fail.
 * On failure, out is zeroed and cannot contain stale generated motor numbers.
 * After any true safety failure the model state resets on the next valid tick;
 * early duplicate ticks are simply ignored without resetting the controller. */
lkm_aw_preview_result_t LKM_AW_Preview_Tick(
    lkm_aw_preview_t *preview, const lkm_rp_feedback_t *feedback,
    const lkm_aw_preview_command_t *command, uint32_t tick_ms,
    lkm_aw_preview_output_t *out);

/* Explicitly invalidate and reset for a new simulated session. */
void LKM_AW_Preview_Invalidate(lkm_aw_preview_t *preview);

const char *LKM_AW_Preview_ResultText(lkm_aw_preview_result_t result);

#endif
