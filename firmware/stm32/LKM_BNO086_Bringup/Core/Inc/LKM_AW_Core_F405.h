/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Core_F405.h
 *
 * Code generated for Simulink model 'LKM_AW_Core_F405'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Thu Nov  5 16:45:34 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef LKM_AW_Core_F405_h_
#define LKM_AW_Core_F405_h_
#ifndef LKM_AW_Core_F405_COMMON_INCLUDES_
#define LKM_AW_Core_F405_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "math.h"
#endif                                 /* LKM_AW_Core_F405_COMMON_INCLUDES_ */

#include "LKM_AW_Core_F405_types.h"
#include "LKM_AW_Ctrl_F405.h"
#include "LKM_AW_Mixer_F405.h"
#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  ((const char **)(&((rtm)->errorStatus)))
#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  MdlrefDW_LKM_AW_Ctrl_F405_T Roll_Pitch_Controller_InstanceD;/* '<Root>/Roll_Pitch_Controller' */
  MdlrefDW_LKM_AW_Mixer_F405_T Bounded_Motor_Mixer_InstanceDat;/* '<Root>/Bounded_Motor_Mixer' */
  float Roll_TR_Delay_DSTATE;          /* '<Root>/Roll_TR_Delay' */
  float Pitch_TR_Delay_DSTATE;         /* '<Root>/Pitch_TR_Delay' */
} DW_LKM_AW_Core_F405_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  float Target_Roll_Angle;             /* '<Root>/Target_Roll_Angle' */
  float Actual_Roll_Rate;              /* '<Root>/Actual_Roll_Rate' */
  float Actual_Roll_Angle;             /* '<Root>/Actual_Roll_Angle' */
  float Target_Pitch_Angle;            /* '<Root>/Target_Pitch_Angle' */
  float Actual_Pitch_Rate;             /* '<Root>/Actual_Pitch_Rate' */
  float Actual_Pitch_Angle;            /* '<Root>/Actual_Pitch_Angle' */
  float Collective_Thrust;             /* '<Root>/Collective_Thrust' */
} ExtU_LKM_AW_Core_F405_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  float Motor_1_Thrust;                /* '<Root>/Motor_1_Thrust' */
  float Motor_2_Thrust;                /* '<Root>/Motor_2_Thrust' */
  float Motor_3_Thrust;                /* '<Root>/Motor_3_Thrust' */
  float Motor_4_Thrust;                /* '<Root>/Motor_4_Thrust' */
  float Requested_Roll_Torque;         /* '<Root>/Requested_Roll_Torque' */
  float Requested_Pitch_Torque;        /* '<Root>/Requested_Pitch_Torque' */
  float Achieved_Roll_Torque;          /* '<Root>/Achieved_Roll_Torque' */
  float Achieved_Pitch_Torque;         /* '<Root>/Achieved_Pitch_Torque' */
  float Target_Roll_Rate;              /* '<Root>/Target_Roll_Rate' */
  float Target_Pitch_Rate;             /* '<Root>/Target_Pitch_Rate' */
} ExtY_LKM_AW_Core_F405_T;

/* Real-time Model Data Structure */
struct tag_RTM_LKM_AW_Core_F405_T {
  const char *errorStatus;
};

/* Block signals and states (default storage) */
extern DW_LKM_AW_Core_F405_T LKM_AW_Core_F405_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_LKM_AW_Core_F405_T LKM_AW_Core_F405_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_LKM_AW_Core_F405_T LKM_AW_Core_F405_Y;

/* Model entry point functions */
extern void LKM_AW_Core_F405_initialize(void);
extern void LKM_AW_Core_F405_step(void);
extern void LKM_AW_Core_F405_terminate(void);

/* Real-time Model object */
extern RT_MODEL_LKM_AW_Core_F405_T *const LKM_AW_Core_F405_M;
extern volatile bool stopRequested;
extern volatile bool runModel;

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
 * '<Root>' : 'LKM_AW_Core_F405'
 */
#endif                                 /* LKM_AW_Core_F405_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
