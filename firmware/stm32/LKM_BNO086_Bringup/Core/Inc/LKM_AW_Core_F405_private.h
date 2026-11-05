/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Core_F405_private.h
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

#ifndef LKM_AW_Core_F405_private_h_
#define LKM_AW_Core_F405_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "LKM_AW_Core_F405_types.h"

extern const float rtCP_pooled_HkVECsDp1mOJ;
extern const float rtCP_pooled_stbNgMQSavpU[12];

#define rtCP_Yaw_Torque_Zero_Value     rtCP_pooled_HkVECsDp1mOJ  /* Expression: single(0)
                                                                  * Referenced by: '<Root>/Yaw_Torque_Zero'
                                                                  */
#define rtCP_Achieved_Torque_Matrix_Gai rtCP_pooled_stbNgMQSavpU /* Expression: K_achieved_core
                                                                  * Referenced by: '<Root>/Achieved_Torque_Matrix'
                                                                  */
#endif                                 /* LKM_AW_Core_F405_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
