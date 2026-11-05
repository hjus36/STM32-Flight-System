#include "lkm_aw_core_preview.h"
#include "LKM_AW_Core_F405.h"
#include <math.h>
#include <string.h>

#define LKM_AW_PREVIEW_PERIOD_MS 10u
#define LKM_AW_PREVIEW_DEG_TO_RAD 0.01745329251994329577f
#define LKM_AW_PREVIEW_MAX_TICK_INTERVAL UINT32_C(0x7FFFFFFF)

static bool valid_config(const lkm_aw_preview_config_t *cfg)
{
    return cfg != NULL && cfg->step_period_ms == LKM_AW_PREVIEW_PERIOD_MS &&
           isfinite(cfg->max_abs_target_angle_rad) &&
           cfg->max_abs_target_angle_rad > 0.0f &&
           cfg->max_abs_target_angle_rad <= 1.5707963268f &&
           isfinite(cfg->max_preview_collective_n) &&
           cfg->max_preview_collective_n > 0.0f;
}

static void clear_model_ports(void)
{
    memset(&LKM_AW_Core_F405_U, 0, sizeof(LKM_AW_Core_F405_U));
    memset(&LKM_AW_Core_F405_Y, 0, sizeof(LKM_AW_Core_F405_Y));
}

void LKM_AW_Preview_Invalidate(lkm_aw_preview_t *preview)
{
    if (preview == NULL) return;
    preview->require_model_reset = true;
    preview->has_last_accepted_step = false;
    clear_model_ports();
}

lkm_aw_preview_result_t LKM_AW_Preview_Init(
    lkm_aw_preview_t *preview, const lkm_aw_preview_config_t *config)
{
    if (preview == NULL || config == NULL) return LKM_AW_PREVIEW_ERR_ARGUMENT;
    memset(preview, 0, sizeof(*preview));
    clear_model_ports();
    if (!valid_config(config)) return LKM_AW_PREVIEW_ERR_CONFIG;
    preview->config = *config;
    preview->initialized = true;
    preview->require_model_reset = true;
    return LKM_AW_PREVIEW_OK;
}

static bool valid_feedback(const lkm_rp_feedback_t *fb)
{
    if (fb == NULL || !fb->feedback_valid) return false;
    return isfinite(fb->roll_angle_rad) && isfinite(fb->pitch_angle_rad) &&
           isfinite(fb->roll_angle_deg) && isfinite(fb->pitch_angle_deg) &&
           isfinite(fb->body_p_rad_s) && isfinite(fb->body_q_rad_s) &&
           fabsf(fb->roll_angle_rad -
                 fb->roll_angle_deg * LKM_AW_PREVIEW_DEG_TO_RAD) < 0.001f &&
           fabsf(fb->pitch_angle_rad -
                 fb->pitch_angle_deg * LKM_AW_PREVIEW_DEG_TO_RAD) < 0.001f &&
           fabsf(fb->roll_angle_rad) <= 3.141592654f &&
           fabsf(fb->pitch_angle_rad) <= 1.570796327f;
}

static bool valid_command(const lkm_aw_preview_config_t *cfg,
                          const lkm_aw_preview_command_t *cmd)
{
    if (cmd == NULL || !cmd->command_valid) return false;
    return isfinite(cmd->target_roll_angle_rad) &&
           isfinite(cmd->target_pitch_angle_rad) &&
           isfinite(cmd->collective_thrust_n) &&
           fabsf(cmd->target_roll_angle_rad) <= cfg->max_abs_target_angle_rad &&
           fabsf(cmd->target_pitch_angle_rad) <= cfg->max_abs_target_angle_rad &&
           cmd->collective_thrust_n >= 0.0f &&
           cmd->collective_thrust_n <= cfg->max_preview_collective_n;
}

static bool finite_outputs(const ExtY_LKM_AW_Core_F405_T *y)
{
    return isfinite(y->Motor_1_Thrust) && isfinite(y->Motor_2_Thrust) &&
           isfinite(y->Motor_3_Thrust) && isfinite(y->Motor_4_Thrust) &&
           isfinite(y->Requested_Roll_Torque) && isfinite(y->Requested_Pitch_Torque) &&
           isfinite(y->Achieved_Roll_Torque) && isfinite(y->Achieved_Pitch_Torque) &&
           isfinite(y->Target_Roll_Rate) && isfinite(y->Target_Pitch_Rate);
}

static lkm_aw_preview_result_t reject(lkm_aw_preview_t *ctx,
                                      lkm_aw_preview_result_t cause)
{
    ++ctx->blocked_steps;
    LKM_AW_Preview_Invalidate(ctx);
    return cause;
}

lkm_aw_preview_result_t LKM_AW_Preview_Tick(
    lkm_aw_preview_t *ctx, const lkm_rp_feedback_t *fb,
    const lkm_aw_preview_command_t *cmd, uint32_t tick_ms,
    lkm_aw_preview_output_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (ctx == NULL || out == NULL) return LKM_AW_PREVIEW_ERR_ARGUMENT;
    if (!ctx->initialized || !valid_config(&ctx->config))
        return reject(ctx, LKM_AW_PREVIEW_ERR_NOT_INITIALIZED);
    if (!ctx->config.coordinate_contract_verified)
        return reject(ctx, LKM_AW_PREVIEW_ERR_AXES_UNVERIFIED);
    if (!valid_feedback(fb))
        return reject(ctx, LKM_AW_PREVIEW_ERR_FEEDBACK);
    if (!valid_command(&ctx->config, cmd))
        return reject(ctx, LKM_AW_PREVIEW_ERR_COMMAND);
    if (ctx->has_last_report && fb->report_sequence == ctx->last_report_sequence)
        return reject(ctx, LKM_AW_PREVIEW_ERR_REUSED_FEEDBACK);

    if (ctx->has_last_accepted_step) {
        const uint32_t delta = (uint32_t)(tick_ms - ctx->last_accepted_step_ms);
        if (delta < ctx->config.step_period_ms) {
            /* Duplicate/early invocation: no model step, preserve state for
             * the next correctly scheduled tick. Output remains cleared. */
            ++ctx->blocked_steps;
            clear_model_ports();
            return LKM_AW_PREVIEW_ERR_EARLY_TICK;
        }
        if (delta > ctx->config.step_period_ms ||
            delta > LKM_AW_PREVIEW_MAX_TICK_INTERVAL)
            return reject(ctx, LKM_AW_PREVIEW_ERR_MISSED_TICK);
    }

    if (ctx->require_model_reset) {
        /* The generated initialize only registers referenced model pointers;
         * fresh zero-initialized DW is needed after loss of feedback. */
        memset(&LKM_AW_Core_F405_DW, 0, sizeof(LKM_AW_Core_F405_DW));
        clear_model_ports();
        rtmSetErrorStatus(LKM_AW_Core_F405_M, NULL);
        LKM_AW_Core_F405_initialize();
        if (rtmGetErrorStatus(LKM_AW_Core_F405_M) != NULL)
            return reject(ctx, LKM_AW_PREVIEW_ERR_MODEL);
        ctx->require_model_reset = false;
        ++ctx->model_resets;
    }

    LKM_AW_Core_F405_U.Target_Roll_Angle = cmd->target_roll_angle_rad;
    LKM_AW_Core_F405_U.Actual_Roll_Rate = fb->body_p_rad_s;
    LKM_AW_Core_F405_U.Actual_Roll_Angle = fb->roll_angle_rad;
    LKM_AW_Core_F405_U.Target_Pitch_Angle = cmd->target_pitch_angle_rad;
    LKM_AW_Core_F405_U.Actual_Pitch_Rate = fb->body_q_rad_s;
    LKM_AW_Core_F405_U.Actual_Pitch_Angle = fb->pitch_angle_rad;
    LKM_AW_Core_F405_U.Collective_Thrust = cmd->collective_thrust_n;

    /* Generated MODEL preview only. No ESC, PWM, GPIO, or flight controller. */
    LKM_AW_Core_F405_step();
    if (rtmGetErrorStatus(LKM_AW_Core_F405_M) != NULL ||
        !finite_outputs(&LKM_AW_Core_F405_Y))
        return reject(ctx, LKM_AW_PREVIEW_ERR_MODEL);

    out->step_tick_ms = tick_ms;
    out->actual_roll_angle_rad = LKM_AW_Core_F405_U.Actual_Roll_Angle;
    out->actual_roll_rate_rad_s = LKM_AW_Core_F405_U.Actual_Roll_Rate;
    out->actual_pitch_angle_rad = LKM_AW_Core_F405_U.Actual_Pitch_Angle;
    out->actual_pitch_rate_rad_s = LKM_AW_Core_F405_U.Actual_Pitch_Rate;
    out->target_roll_angle_rad = LKM_AW_Core_F405_U.Target_Roll_Angle;
    out->target_pitch_angle_rad = LKM_AW_Core_F405_U.Target_Pitch_Angle;
    out->collective_thrust_n = LKM_AW_Core_F405_U.Collective_Thrust;
    out->motor_1_thrust_preview_n = LKM_AW_Core_F405_Y.Motor_1_Thrust;
    out->motor_2_thrust_preview_n = LKM_AW_Core_F405_Y.Motor_2_Thrust;
    out->motor_3_thrust_preview_n = LKM_AW_Core_F405_Y.Motor_3_Thrust;
    out->motor_4_thrust_preview_n = LKM_AW_Core_F405_Y.Motor_4_Thrust;
    out->requested_roll_torque_nm = LKM_AW_Core_F405_Y.Requested_Roll_Torque;
    out->requested_pitch_torque_nm = LKM_AW_Core_F405_Y.Requested_Pitch_Torque;
    out->achieved_roll_torque_nm = LKM_AW_Core_F405_Y.Achieved_Roll_Torque;
    out->achieved_pitch_torque_nm = LKM_AW_Core_F405_Y.Achieved_Pitch_Torque;
    out->target_roll_rate_rad_s = LKM_AW_Core_F405_Y.Target_Roll_Rate;
    out->target_pitch_rate_rad_s = LKM_AW_Core_F405_Y.Target_Pitch_Rate;
    out->valid = true;

    ctx->has_last_accepted_step = true;
    ctx->last_accepted_step_ms = tick_ms;
    ctx->has_last_report = true;
    ctx->last_report_sequence = fb->report_sequence;
    ++ctx->accepted_steps;
    return LKM_AW_PREVIEW_OK;
}

const char *LKM_AW_Preview_ResultText(lkm_aw_preview_result_t result)
{
    switch (result) {
    case LKM_AW_PREVIEW_OK: return "OK";
    case LKM_AW_PREVIEW_ERR_ARGUMENT: return "ARGUMENT";
    case LKM_AW_PREVIEW_ERR_NOT_INITIALIZED: return "NOT_INITIALIZED";
    case LKM_AW_PREVIEW_ERR_CONFIG: return "CONFIG";
    case LKM_AW_PREVIEW_ERR_AXES_UNVERIFIED: return "AXES_UNVERIFIED";
    case LKM_AW_PREVIEW_ERR_FEEDBACK: return "FEEDBACK";
    case LKM_AW_PREVIEW_ERR_COMMAND: return "COMMAND";
    case LKM_AW_PREVIEW_ERR_EARLY_TICK: return "EARLY_TICK";
    case LKM_AW_PREVIEW_ERR_REUSED_FEEDBACK: return "REUSED_FEEDBACK";
    case LKM_AW_PREVIEW_ERR_MISSED_TICK: return "MISSED_TICK";
    case LKM_AW_PREVIEW_ERR_MODEL: return "MODEL";
    default: return "UNKNOWN";
    }
}
