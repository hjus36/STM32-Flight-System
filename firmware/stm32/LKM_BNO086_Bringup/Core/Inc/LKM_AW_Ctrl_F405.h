/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LKM_AW_Ctrl_F405.h
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

#ifndef LKM_AW_Ctrl_F405_h_
#define LKM_AW_Ctrl_F405_h_
#ifndef LKM_AW_Ctrl_F405_COMMON_INCLUDES_
#define LKM_AW_Ctrl_F405_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "math.h"
#endif                                 /* LKM_AW_Ctrl_F405_COMMON_INCLUDES_ */

#include "LKM_AW_Ctrl_F405_types.h"

/* Block signals and states (default storage) for model 'LKM_AW_Ctrl_F405' */
typedef struct {
  float Integrator_DSTATE;             /* '<S91>/Integrator' */
  float Integrator_DSTATE_p;           /* '<S38>/Integrator' */
} DW_LKM_AW_Ctrl_F405_f_T;

/* Real-time Model Data Structure */
struct tag_RTM_LKM_AW_Ctrl_F405_T {
  const char **errorStatus;
};

typedef struct {
  DW_LKM_AW_Ctrl_F405_f_T rtdw;
  RT_MODEL_LKM_AW_Ctrl_F405_T rtm;
} MdlrefDW_LKM_AW_Ctrl_F405_T;

/* Model reference registration function */
extern void LKM_AW_Ctrl_F405_initialize(const char **rt_errorStatus,
  RT_MODEL_LKM_AW_Ctrl_F405_T *const LKM_AW_Ctrl_F405_M);
extern void LKM_AW_Ctrl_F405(const float *rtu_Target_Roll_Angle, const float
  *rtu_Actual_Roll_Rate, const float *rtu_Actual_Roll_Angle, const float
  *rtu_Target_Pitch_Angle, const float *rtu_Actual_Pitch_Rate, const float
  *rtu_Actual_Pitch_Angle, const float *rtu_Achieved_Roll_Torque, const float
  *rtu_Achieved_Pitch_Torque, float *rty_Roll_Control_Torque, float
  *rty_Target_Roll_Rate, float *rty_Pitch_Control_Torque, float
  *rty_Target_Pitch_Rate, DW_LKM_AW_Ctrl_F405_f_T *localDW);

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
 * '<Root>' : 'LKM_AW_Ctrl_F405'
 * '<S1>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller'
 * '<S2>'   : 'LKM_AW_Ctrl_F405/Roll_Controller'
 * '<S3>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller'
 * '<S4>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Anti-windup'
 * '<S5>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/D Gain'
 * '<S6>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/External Derivative'
 * '<S7>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Filter'
 * '<S8>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Filter ICs'
 * '<S9>'   : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/I Gain'
 * '<S10>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Ideal P Gain'
 * '<S11>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S12>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Integrator'
 * '<S13>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Integrator ICs'
 * '<S14>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/N Copy'
 * '<S15>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/N Gain'
 * '<S16>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/P Copy'
 * '<S17>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Parallel P Gain'
 * '<S18>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Reset Signal'
 * '<S19>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Saturation'
 * '<S20>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S21>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Sum'
 * '<S22>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Sum Fdbk'
 * '<S23>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tracking Mode'
 * '<S24>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S25>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S26>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S27>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/postSat Signal'
 * '<S28>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/preInt Signal'
 * '<S29>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/preSat Signal'
 * '<S30>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S31>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S32>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/External Derivative/Disabled'
 * '<S33>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Filter/Disabled'
 * '<S34>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S35>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S36>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S37>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S38>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S39>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S40>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S41>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S42>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S43>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S44>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S45>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S46>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S47>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S48>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S49>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tracking Mode/Enabled'
 * '<S50>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tracking Mode Sum/Tracking Mode'
 * '<S51>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S52>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S53>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S54>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/preInt Signal/Internal PreInt'
 * '<S55>'  : 'LKM_AW_Ctrl_F405/Pitch_Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S56>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller'
 * '<S57>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Anti-windup'
 * '<S58>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/D Gain'
 * '<S59>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/External Derivative'
 * '<S60>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Filter'
 * '<S61>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Filter ICs'
 * '<S62>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/I Gain'
 * '<S63>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Ideal P Gain'
 * '<S64>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S65>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Integrator'
 * '<S66>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Integrator ICs'
 * '<S67>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/N Copy'
 * '<S68>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/N Gain'
 * '<S69>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/P Copy'
 * '<S70>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Parallel P Gain'
 * '<S71>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Reset Signal'
 * '<S72>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Saturation'
 * '<S73>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S74>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Sum'
 * '<S75>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Sum Fdbk'
 * '<S76>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tracking Mode'
 * '<S77>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S78>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S79>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S80>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/postSat Signal'
 * '<S81>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/preInt Signal'
 * '<S82>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/preSat Signal'
 * '<S83>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S84>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S85>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/External Derivative/Disabled'
 * '<S86>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Filter/Disabled'
 * '<S87>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S88>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S89>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S90>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S91>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S92>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S93>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S94>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S95>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S96>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S97>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S98>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S99>'  : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S100>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S101>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S102>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tracking Mode/Enabled'
 * '<S103>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tracking Mode Sum/Tracking Mode'
 * '<S104>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S105>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S106>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S107>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/preInt Signal/Internal PreInt'
 * '<S108>' : 'LKM_AW_Ctrl_F405/Roll_Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* LKM_AW_Ctrl_F405_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
