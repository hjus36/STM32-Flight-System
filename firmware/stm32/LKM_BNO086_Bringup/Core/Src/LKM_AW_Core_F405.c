/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Core_F405.c
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

#include "LKM_AW_Core_F405.h"
#include <stdint.h>
#include "LKM_AW_Core_F405_private.h"
#include "LKM_AW_Mixer_F405.h"
#include "LKM_AW_Ctrl_F405.h"

/* Block signals and states (default storage) */
DW_LKM_AW_Core_F405_T LKM_AW_Core_F405_DW;

/* External inputs (root inport signals with default storage) */
ExtU_LKM_AW_Core_F405_T LKM_AW_Core_F405_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_LKM_AW_Core_F405_T LKM_AW_Core_F405_Y;

/* Real-time model */
static RT_MODEL_LKM_AW_Core_F405_T LKM_AW_Core_F405_M_;
RT_MODEL_LKM_AW_Core_F405_T *const LKM_AW_Core_F405_M = &LKM_AW_Core_F405_M_;

/* Model step function */
void LKM_AW_Core_F405_step(void)
{
  float tmp[4];
  float tmp_0;
  int32_t i;

  /* ModelReference: '<Root>/Roll_Pitch_Controller' incorporates:
   *  Inport: '<Root>/Actual_Pitch_Angle'
   *  Inport: '<Root>/Actual_Pitch_Rate'
   *  Inport: '<Root>/Actual_Roll_Angle'
   *  Inport: '<Root>/Actual_Roll_Rate'
   *  Inport: '<Root>/Target_Pitch_Angle'
   *  Inport: '<Root>/Target_Roll_Angle'
   *  Outport: '<Root>/Requested_Pitch_Torque'
   *  Outport: '<Root>/Requested_Roll_Torque'
   *  Outport: '<Root>/Target_Pitch_Rate'
   *  Outport: '<Root>/Target_Roll_Rate'
   *  UnitDelay: '<Root>/Pitch_TR_Delay'
   *  UnitDelay: '<Root>/Roll_TR_Delay'
   */
  LKM_AW_Ctrl_F405(&LKM_AW_Core_F405_U.Target_Roll_Angle,
                   &LKM_AW_Core_F405_U.Actual_Roll_Rate,
                   &LKM_AW_Core_F405_U.Actual_Roll_Angle,
                   &LKM_AW_Core_F405_U.Target_Pitch_Angle,
                   &LKM_AW_Core_F405_U.Actual_Pitch_Rate,
                   &LKM_AW_Core_F405_U.Actual_Pitch_Angle,
                   &LKM_AW_Core_F405_DW.Roll_TR_Delay_DSTATE,
                   &LKM_AW_Core_F405_DW.Pitch_TR_Delay_DSTATE,
                   &LKM_AW_Core_F405_Y.Requested_Roll_Torque,
                   &LKM_AW_Core_F405_Y.Target_Roll_Rate,
                   &LKM_AW_Core_F405_Y.Requested_Pitch_Torque,
                   &LKM_AW_Core_F405_Y.Target_Pitch_Rate,
                   &(LKM_AW_Core_F405_DW.Roll_Pitch_Controller_InstanceD.rtdw));

  /* ModelReference: '<Root>/Bounded_Motor_Mixer' incorporates:
   *  Constant: '<Root>/Yaw_Torque_Zero'
   *  Inport: '<Root>/Collective_Thrust'
   *  Outport: '<Root>/Motor_1_Thrust'
   *  Outport: '<Root>/Motor_2_Thrust'
   *  Outport: '<Root>/Motor_3_Thrust'
   *  Outport: '<Root>/Motor_4_Thrust'
   *  Outport: '<Root>/Requested_Pitch_Torque'
   *  Outport: '<Root>/Requested_Roll_Torque'
   */
  LKM_AW_Mixer_F405(&LKM_AW_Core_F405_U.Collective_Thrust,
                    &LKM_AW_Core_F405_Y.Requested_Roll_Torque,
                    &LKM_AW_Core_F405_Y.Requested_Pitch_Torque,
                    &rtCP_Yaw_Torque_Zero_Value,
                    &LKM_AW_Core_F405_Y.Motor_1_Thrust,
                    &LKM_AW_Core_F405_Y.Motor_2_Thrust,
                    &LKM_AW_Core_F405_Y.Motor_3_Thrust,
                    &LKM_AW_Core_F405_Y.Motor_4_Thrust);

  /* SignalConversion generated from: '<Root>/Achieved_Torque_Matrix' incorporates:
   *  Outport: '<Root>/Motor_1_Thrust'
   *  Outport: '<Root>/Motor_2_Thrust'
   *  Outport: '<Root>/Motor_3_Thrust'
   *  Outport: '<Root>/Motor_4_Thrust'
   */
  tmp[0] = LKM_AW_Core_F405_Y.Motor_1_Thrust;
  tmp[1] = LKM_AW_Core_F405_Y.Motor_2_Thrust;
  tmp[2] = LKM_AW_Core_F405_Y.Motor_3_Thrust;
  tmp[3] = LKM_AW_Core_F405_Y.Motor_4_Thrust;

  /* Outport: '<Root>/Achieved_Roll_Torque' incorporates:
   *  Gain: '<Root>/Achieved_Torque_Matrix'
   */
  LKM_AW_Core_F405_Y.Achieved_Roll_Torque = 0.0F;

  /* Outport: '<Root>/Achieved_Pitch_Torque' incorporates:
   *  Gain: '<Root>/Achieved_Torque_Matrix'
   */
  LKM_AW_Core_F405_Y.Achieved_Pitch_Torque = 0.0F;

  /* Gain: '<Root>/Achieved_Torque_Matrix' */
  for (i = 0; i < 4; i++) {
    tmp_0 = tmp[i];

    /* Outport: '<Root>/Achieved_Roll_Torque' */
    LKM_AW_Core_F405_Y.Achieved_Roll_Torque += rtCP_Achieved_Torque_Matrix_Gai[3
      * i + 1] * tmp_0;

    /* Outport: '<Root>/Achieved_Pitch_Torque' */
    LKM_AW_Core_F405_Y.Achieved_Pitch_Torque += rtCP_Achieved_Torque_Matrix_Gai
      [3 * i + 2] * tmp_0;
  }

  /* Update for UnitDelay: '<Root>/Roll_TR_Delay' */
  LKM_AW_Core_F405_DW.Roll_TR_Delay_DSTATE =
    LKM_AW_Core_F405_Y.Achieved_Roll_Torque;

  /* Update for UnitDelay: '<Root>/Pitch_TR_Delay' */
  LKM_AW_Core_F405_DW.Pitch_TR_Delay_DSTATE =
    LKM_AW_Core_F405_Y.Achieved_Pitch_Torque;
}

/* Model initialize function */
void LKM_AW_Core_F405_initialize(void)
{
  /* Model Initialize function for ModelReference Block: '<Root>/Bounded_Motor_Mixer' */
  LKM_AW_Mixer_F405_initialize(rtmGetErrorStatusPointer(LKM_AW_Core_F405_M),
    &(LKM_AW_Core_F405_DW.Bounded_Motor_Mixer_InstanceDat.rtm));

  /* Model Initialize function for ModelReference Block: '<Root>/Roll_Pitch_Controller' */
  LKM_AW_Ctrl_F405_initialize(rtmGetErrorStatusPointer(LKM_AW_Core_F405_M),
    &(LKM_AW_Core_F405_DW.Roll_Pitch_Controller_InstanceD.rtm));
}

/* Model terminate function */
void LKM_AW_Core_F405_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
