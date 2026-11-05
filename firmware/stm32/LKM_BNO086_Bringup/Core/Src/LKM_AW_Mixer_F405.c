/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Mixer_F405.c
 *
 * Code generated for Simulink model 'LKM_AW_Mixer_F405'.
 *
 * Model version                  : 1.12
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Thu Nov  5 16:35:45 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LKM_AW_Mixer_F405.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "LKM_AW_Mixer_F405_private.h"

/* Output and update for referenced model: 'LKM_AW_Mixer_F405' */
void LKM_AW_Mixer_F405(const float *rtu_Collective_Thrust, const float
  *rtu_Roll_Torque, const float *rtu_Pitch_Torque, const float *rtu_Yaw_Torque,
  float *rty_Motor_1_Thrust, float *rty_Motor_2_Thrust, float
  *rty_Motor_3_Thrust, float *rty_Motor_4_Thrust)
{
  float rtb_F_bounded[4];
  float rtu_Collective_Thrust_0[4];
  float base;
  float rtb_Quad_X_Mixing_Matrix_idx_0;
  float rtb_Quad_X_Mixing_Matrix_idx_1;
  float rtb_Quad_X_Mixing_Matrix_idx_2;
  float rtb_Quad_X_Mixing_Matrix_idx_3;
  int32_t i;
  bool b_x[4];

  /* SignalConversion generated from: '<Root>/Quad_X_Mixing_Matrix' */
  rtu_Collective_Thrust_0[0] = *rtu_Collective_Thrust;
  rtu_Collective_Thrust_0[1] = *rtu_Roll_Torque;
  rtu_Collective_Thrust_0[2] = *rtu_Pitch_Torque;
  rtu_Collective_Thrust_0[3] = *rtu_Yaw_Torque;

  /* Gain: '<Root>/Quad_X_Mixing_Matrix' */
  rtb_Quad_X_Mixing_Matrix_idx_0 = 0.0F;
  rtb_Quad_X_Mixing_Matrix_idx_1 = 0.0F;
  rtb_Quad_X_Mixing_Matrix_idx_2 = 0.0F;
  rtb_Quad_X_Mixing_Matrix_idx_3 = 0.0F;
  for (i = 0; i < 4; i++) {
    int32_t rtb_Quad_X_Mixing_Matrix_idx_0_;
    base = rtu_Collective_Thrust_0[i];
    rtb_Quad_X_Mixing_Matrix_idx_0_ = i << 2;
    rtb_Quad_X_Mixing_Matrix_idx_0 +=
      rtCP_Quad_X_Mixing_Matrix_Gain[rtb_Quad_X_Mixing_Matrix_idx_0_] * base;
    rtb_Quad_X_Mixing_Matrix_idx_1 +=
      rtCP_Quad_X_Mixing_Matrix_Gain[rtb_Quad_X_Mixing_Matrix_idx_0_ + 1] * base;
    rtb_Quad_X_Mixing_Matrix_idx_2 +=
      rtCP_Quad_X_Mixing_Matrix_Gain[rtb_Quad_X_Mixing_Matrix_idx_0_ + 2] * base;
    rtb_Quad_X_Mixing_Matrix_idx_3 +=
      rtCP_Quad_X_Mixing_Matrix_Gain[rtb_Quad_X_Mixing_Matrix_idx_0_ + 3] * base;

    /* MATLAB Function: '<Root>/Bounded_Thrust_Allocator' */
    rtb_F_bounded[i] = 0.0F;
  }

  /* End of Gain: '<Root>/Quad_X_Mixing_Matrix' */

  /* MATLAB Function: '<Root>/Bounded_Thrust_Allocator' incorporates:
   *  Constant: '<Root>/Max_Motor_Thrust'
   */
  if (!rtIsInfF(*rtu_Collective_Thrust) && !rtIsNaNF(*rtu_Collective_Thrust)) {
    bool y;
    b_x[0] = (rtIsInfF(rtb_Quad_X_Mixing_Matrix_idx_0) || rtIsNaNF
              (rtb_Quad_X_Mixing_Matrix_idx_0));
    b_x[1] = (rtIsInfF(rtb_Quad_X_Mixing_Matrix_idx_1) || rtIsNaNF
              (rtb_Quad_X_Mixing_Matrix_idx_1));
    b_x[2] = (rtIsInfF(rtb_Quad_X_Mixing_Matrix_idx_2) || rtIsNaNF
              (rtb_Quad_X_Mixing_Matrix_idx_2));
    b_x[3] = (rtIsInfF(rtb_Quad_X_Mixing_Matrix_idx_3) || rtIsNaNF
              (rtb_Quad_X_Mixing_Matrix_idx_3));
    y = false;
    i = 0;
    while (i < 4) {
      if (b_x[i]) {
        y = true;
        break;
      } else {
        i++;
      }
    }

    if (!y) {
      float alpha;
      float rtb_Quad_X_Mixing_Matrix_0;
      float z;
      base = fminf(fmaxf(*rtu_Collective_Thrust, 0.0F), 8.0F) / 4.0F;
      z = *rtu_Collective_Thrust / 4.0F;
      rtb_Quad_X_Mixing_Matrix_idx_0 -= z;
      rtb_Quad_X_Mixing_Matrix_idx_1 -= z;
      rtb_Quad_X_Mixing_Matrix_idx_2 -= z;
      rtb_Quad_X_Mixing_Matrix_idx_3 -= z;
      z = (((rtb_Quad_X_Mixing_Matrix_idx_0 + rtb_Quad_X_Mixing_Matrix_idx_1) +
            rtb_Quad_X_Mixing_Matrix_idx_2) + rtb_Quad_X_Mixing_Matrix_idx_3) /
        4.0F;
      alpha = 1.0F;
      rtb_Quad_X_Mixing_Matrix_0 = rtb_Quad_X_Mixing_Matrix_idx_0 - z;
      rtb_Quad_X_Mixing_Matrix_idx_0 = rtb_Quad_X_Mixing_Matrix_0;
      if (rtb_Quad_X_Mixing_Matrix_0 > 0.0F) {
        alpha = fminf(1.0F, (2.0F - base) / rtb_Quad_X_Mixing_Matrix_0);
      } else if (rtb_Quad_X_Mixing_Matrix_0 < 0.0F) {
        alpha = fminf(1.0F, base / -rtb_Quad_X_Mixing_Matrix_0);
      }

      rtb_Quad_X_Mixing_Matrix_0 = rtb_Quad_X_Mixing_Matrix_idx_1 - z;
      rtb_Quad_X_Mixing_Matrix_idx_1 = rtb_Quad_X_Mixing_Matrix_0;
      if (rtb_Quad_X_Mixing_Matrix_0 > 0.0F) {
        alpha = fminf(alpha, (2.0F - base) / rtb_Quad_X_Mixing_Matrix_0);
      } else if (rtb_Quad_X_Mixing_Matrix_0 < 0.0F) {
        alpha = fminf(alpha, base / -rtb_Quad_X_Mixing_Matrix_0);
      }

      rtb_Quad_X_Mixing_Matrix_0 = rtb_Quad_X_Mixing_Matrix_idx_2 - z;
      rtb_Quad_X_Mixing_Matrix_idx_2 = rtb_Quad_X_Mixing_Matrix_0;
      if (rtb_Quad_X_Mixing_Matrix_0 > 0.0F) {
        alpha = fminf(alpha, (2.0F - base) / rtb_Quad_X_Mixing_Matrix_0);
      } else if (rtb_Quad_X_Mixing_Matrix_0 < 0.0F) {
        alpha = fminf(alpha, base / -rtb_Quad_X_Mixing_Matrix_0);
      }

      rtb_Quad_X_Mixing_Matrix_0 = rtb_Quad_X_Mixing_Matrix_idx_3 - z;
      if (rtb_Quad_X_Mixing_Matrix_0 > 0.0F) {
        alpha = fminf(alpha, (2.0F - base) / rtb_Quad_X_Mixing_Matrix_0);
      } else if (rtb_Quad_X_Mixing_Matrix_0 < 0.0F) {
        alpha = fminf(alpha, base / -rtb_Quad_X_Mixing_Matrix_0);
      }

      rtb_F_bounded[0] = fminf(fmaxf(alpha * rtb_Quad_X_Mixing_Matrix_idx_0 +
        base, 0.0F), 2.0F);
      rtb_F_bounded[1] = fminf(fmaxf(alpha * rtb_Quad_X_Mixing_Matrix_idx_1 +
        base, 0.0F), 2.0F);
      rtb_F_bounded[2] = fminf(fmaxf(alpha * rtb_Quad_X_Mixing_Matrix_idx_2 +
        base, 0.0F), 2.0F);
      rtb_F_bounded[3] = fminf(fmaxf(alpha * rtb_Quad_X_Mixing_Matrix_0 + base,
        0.0F), 2.0F);
    }
  }

  /* SignalConversion generated from: '<Root>/Motor_1_Thrust' */
  *rty_Motor_1_Thrust = rtb_F_bounded[0];

  /* SignalConversion generated from: '<Root>/Motor_2_Thrust' */
  *rty_Motor_2_Thrust = rtb_F_bounded[1];

  /* SignalConversion generated from: '<Root>/Motor_3_Thrust' */
  *rty_Motor_3_Thrust = rtb_F_bounded[2];

  /* SignalConversion generated from: '<Root>/Motor_4_Thrust' */
  *rty_Motor_4_Thrust = rtb_F_bounded[3];
}

/* Model initialize function */
void LKM_AW_Mixer_F405_initialize(const char **rt_errorStatus,
  RT_MODEL_LKM_AW_Mixer_F405_T *const LKM_AW_Mixer_F405_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(LKM_AW_Mixer_F405_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
