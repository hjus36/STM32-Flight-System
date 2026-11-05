/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Mixer_F405_private.h
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

#ifndef LKM_AW_Mixer_F405_private_h_
#define LKM_AW_Mixer_F405_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "LKM_AW_Mixer_F405_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         (*((rtm)->errorStatus))
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    (*((rtm)->errorStatus) = (val))
#endif

#ifndef rtmGetErrorStatusPointer
#define rtmGetErrorStatusPointer(rtm)  (rtm)->errorStatus
#endif

#ifndef rtmSetErrorStatusPointer
#define rtmSetErrorStatusPointer(rtm, val) ((rtm)->errorStatus = (val))
#endif

extern const float rtCP_pooled_SE6vMLHTpUXp[16];

#define rtCP_Quad_X_Mixing_Matrix_Gain rtCP_pooled_SE6vMLHTpUXp  /* Expression: K_mix
                                                                  * Referenced by: '<Root>/Quad_X_Mixing_Matrix'
                                                                  */
#endif                                 /* LKM_AW_Mixer_F405_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
