/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Mixer_F405.h
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

#ifndef LKM_AW_Mixer_F405_h_
#define LKM_AW_Mixer_F405_h_
#ifndef LKM_AW_Mixer_F405_COMMON_INCLUDES_
#define LKM_AW_Mixer_F405_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* LKM_AW_Mixer_F405_COMMON_INCLUDES_ */

#include "LKM_AW_Mixer_F405_types.h"

/* Real-time Model Data Structure */
struct tag_RTM_LKM_AW_Mixer_F405_T {
  const char **errorStatus;
};

typedef struct {
  RT_MODEL_LKM_AW_Mixer_F405_T rtm;
} MdlrefDW_LKM_AW_Mixer_F405_T;

/* Model reference registration function */
extern void LKM_AW_Mixer_F405_initialize(const char **rt_errorStatus,
  RT_MODEL_LKM_AW_Mixer_F405_T *const LKM_AW_Mixer_F405_M);
extern void LKM_AW_Mixer_F405(const float *rtu_Collective_Thrust, const float
  *rtu_Roll_Torque, const float *rtu_Pitch_Torque, const float *rtu_Yaw_Torque,
  float *rty_Motor_1_Thrust, float *rty_Motor_2_Thrust, float
  *rty_Motor_3_Thrust, float *rty_Motor_4_Thrust);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<Root>/Display1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'LKM_AW_Mixer_F405'
 * '<S1>'   : 'LKM_AW_Mixer_F405/Bounded_Thrust_Allocator'
 */
#endif                                 /* LKM_AW_Mixer_F405_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
