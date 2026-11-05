/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Ctrl_F405.c
 *
 * Code generated for Simulink model 'LKM_AW_Ctrl_F405'.
 *
 * Model version                  : 1.26
 * Simulink Coder version         : 26.2 (R2026b) 22-May-2026
 * C/C++ source code generated on : Thu Nov  5 16:35:37 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "LKM_AW_Ctrl_F405.h"
#include "LKM_AW_Ctrl_F405_private.h"

/* Output and update for referenced model: 'LKM_AW_Ctrl_F405' */
void LKM_AW_Ctrl_F405(const float *rtu_Target_Roll_Angle, const float
                      *rtu_Actual_Roll_Rate, const float *rtu_Actual_Roll_Angle,
                      const float *rtu_Target_Pitch_Angle, const float
                      *rtu_Actual_Pitch_Rate, const float
                      *rtu_Actual_Pitch_Angle, const float
                      *rtu_Achieved_Roll_Torque, const float
                      *rtu_Achieved_Pitch_Torque, float *rty_Roll_Control_Torque,
                      float *rty_Target_Roll_Rate, float
                      *rty_Pitch_Control_Torque, float *rty_Target_Pitch_Rate,
                      DW_LKM_AW_Ctrl_F405_f_T *localDW)
{
  float rtb_Sum1;
  float rtb_Sum1_e;

  /* Gain: '<S2>/Gain' incorporates:
   *  Sum: '<S2>/Sum'
   */
  *rty_Target_Roll_Rate = (*rtu_Target_Roll_Angle - *rtu_Actual_Roll_Angle) *
    2.0F;

  /* Sum: '<S2>/Sum1' */
  rtb_Sum1 = *rty_Target_Roll_Rate - *rtu_Actual_Roll_Rate;

  /* Sum: '<S100>/Sum' incorporates:
   *  DiscreteIntegrator: '<S91>/Integrator'
   *  Gain: '<S96>/Proportional Gain'
   */
  *rty_Roll_Control_Torque = 0.05F * rtb_Sum1 + localDW->Integrator_DSTATE;

  /* Gain: '<S1>/Gain' incorporates:
   *  Sum: '<S1>/Sum'
   */
  *rty_Target_Pitch_Rate = (*rtu_Target_Pitch_Angle - *rtu_Actual_Pitch_Angle) *
    2.0F;

  /* Sum: '<S1>/Sum1' */
  rtb_Sum1_e = *rty_Target_Pitch_Rate - *rtu_Actual_Pitch_Rate;

  /* Sum: '<S47>/Sum' incorporates:
   *  DiscreteIntegrator: '<S38>/Integrator'
   *  Gain: '<S43>/Proportional Gain'
   */
  *rty_Pitch_Control_Torque = 0.05F * rtb_Sum1_e + localDW->Integrator_DSTATE_p;

  /* Update for DiscreteIntegrator: '<S91>/Integrator' incorporates:
   *  Gain: '<S102>/Kt'
   *  Gain: '<S88>/Integral Gain'
   *  Sum: '<S102>/SumI3'
   *  Sum: '<S103>/SumI1'
   */
  localDW->Integrator_DSTATE += ((*rtu_Achieved_Roll_Torque -
    *rty_Roll_Control_Torque) * 2.0F + 0.2F * rtb_Sum1) * 0.01F;

  /* Update for DiscreteIntegrator: '<S38>/Integrator' incorporates:
   *  Gain: '<S35>/Integral Gain'
   *  Gain: '<S49>/Kt'
   *  Sum: '<S49>/SumI3'
   *  Sum: '<S50>/SumI1'
   */
  localDW->Integrator_DSTATE_p += ((*rtu_Achieved_Pitch_Torque -
    *rty_Pitch_Control_Torque) * 2.0F + 0.2F * rtb_Sum1_e) * 0.01F;
}

/* Model initialize function */
void LKM_AW_Ctrl_F405_initialize(const char **rt_errorStatus,
  RT_MODEL_LKM_AW_Ctrl_F405_T *const LKM_AW_Ctrl_F405_M)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatusPointer(LKM_AW_Ctrl_F405_M, rt_errorStatus);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
