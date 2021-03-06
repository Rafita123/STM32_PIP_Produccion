/*
 * File: control.h
 *
 * Code generated for Simulink model 'control'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Fri Jul 15 14:30:49 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_control_h_
#define RTW_HEADER_control_h_
#ifndef control_COMMON_INCLUDES_
#define control_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* control_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */

/* Model entry point functions */
extern void control_initialize(void);
extern void control_step(void);

/* Exported data declaration */

/* Data with Exported storage */
extern real_T rtEntrada_Control1;      /* '<Root>/Entrada_Control1' */
extern real_T rtEntrada_Control2;      /* '<Root>/Entrada_Control2' */
extern real_T rtEntrada_Control3;      /* '<Root>/Entrada_Control3' */
extern real_T rtEntrada_Control4;      /* '<Root>/Entrada_Control4' */
extern real_T rtIntegrator_DSTATE;     /* '<S38>/Integrator' */
extern real_T rtIntegrator_DSTATE_g;   /* '<S88>/Integrator' */
extern real_T rtIntegrator_DSTATE_gr;  /* '<S138>/Integrator' */
extern real_T rtIntegrator_DSTATE_grm; /* '<S188>/Integrator' */
extern real_T rtSalida_Control1;       /* '<Root>/Salida_Control1' */
extern real_T rtSalida_Control2;       /* '<Root>/Salida_Control2' */
extern real_T rtSalida_Control3;       /* '<Root>/Salida_Control3' */
extern real_T rtSalida_Control4;       /* '<Root>/Salida_Control4' */
extern real_T rtUD_DSTATE;             /* '<S31>/UD' */
extern real_T rtUD_DSTATE_m;           /* '<S81>/UD' */
extern real_T rtUD_DSTATE_mr;          /* '<S131>/UD' */
extern real_T rtUD_DSTATE_mrz;         /* '<S181>/UD' */

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S31>/DTDup' : Unused code path elimination
 * Block '<S81>/DTDup' : Unused code path elimination
 * Block '<S131>/DTDup' : Unused code path elimination
 * Block '<S181>/DTDup' : Unused code path elimination
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
 * '<Root>' : 'control'
 * '<S1>'   : 'control/PID Controller5'
 * '<S2>'   : 'control/PID Controller6'
 * '<S3>'   : 'control/PID Controller7'
 * '<S4>'   : 'control/PID Controller8'
 * '<S5>'   : 'control/PID Controller5/Anti-windup'
 * '<S6>'   : 'control/PID Controller5/D Gain'
 * '<S7>'   : 'control/PID Controller5/Filter'
 * '<S8>'   : 'control/PID Controller5/Filter ICs'
 * '<S9>'   : 'control/PID Controller5/I Gain'
 * '<S10>'  : 'control/PID Controller5/Ideal P Gain'
 * '<S11>'  : 'control/PID Controller5/Ideal P Gain Fdbk'
 * '<S12>'  : 'control/PID Controller5/Integrator'
 * '<S13>'  : 'control/PID Controller5/Integrator ICs'
 * '<S14>'  : 'control/PID Controller5/N Copy'
 * '<S15>'  : 'control/PID Controller5/N Gain'
 * '<S16>'  : 'control/PID Controller5/P Copy'
 * '<S17>'  : 'control/PID Controller5/Parallel P Gain'
 * '<S18>'  : 'control/PID Controller5/Reset Signal'
 * '<S19>'  : 'control/PID Controller5/Saturation'
 * '<S20>'  : 'control/PID Controller5/Saturation Fdbk'
 * '<S21>'  : 'control/PID Controller5/Sum'
 * '<S22>'  : 'control/PID Controller5/Sum Fdbk'
 * '<S23>'  : 'control/PID Controller5/Tracking Mode'
 * '<S24>'  : 'control/PID Controller5/Tracking Mode Sum'
 * '<S25>'  : 'control/PID Controller5/Tsamp - Integral'
 * '<S26>'  : 'control/PID Controller5/Tsamp - Ngain'
 * '<S27>'  : 'control/PID Controller5/postSat Signal'
 * '<S28>'  : 'control/PID Controller5/preSat Signal'
 * '<S29>'  : 'control/PID Controller5/Anti-windup/Passthrough'
 * '<S30>'  : 'control/PID Controller5/D Gain/Internal Parameters'
 * '<S31>'  : 'control/PID Controller5/Filter/Differentiator'
 * '<S32>'  : 'control/PID Controller5/Filter/Differentiator/Tsamp'
 * '<S33>'  : 'control/PID Controller5/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S34>'  : 'control/PID Controller5/Filter ICs/Internal IC - Differentiator'
 * '<S35>'  : 'control/PID Controller5/I Gain/Internal Parameters'
 * '<S36>'  : 'control/PID Controller5/Ideal P Gain/Passthrough'
 * '<S37>'  : 'control/PID Controller5/Ideal P Gain Fdbk/Disabled'
 * '<S38>'  : 'control/PID Controller5/Integrator/Discrete'
 * '<S39>'  : 'control/PID Controller5/Integrator ICs/Internal IC'
 * '<S40>'  : 'control/PID Controller5/N Copy/Disabled wSignal Specification'
 * '<S41>'  : 'control/PID Controller5/N Gain/Passthrough'
 * '<S42>'  : 'control/PID Controller5/P Copy/Disabled'
 * '<S43>'  : 'control/PID Controller5/Parallel P Gain/Internal Parameters'
 * '<S44>'  : 'control/PID Controller5/Reset Signal/Disabled'
 * '<S45>'  : 'control/PID Controller5/Saturation/Passthrough'
 * '<S46>'  : 'control/PID Controller5/Saturation Fdbk/Disabled'
 * '<S47>'  : 'control/PID Controller5/Sum/Sum_PID'
 * '<S48>'  : 'control/PID Controller5/Sum Fdbk/Disabled'
 * '<S49>'  : 'control/PID Controller5/Tracking Mode/Disabled'
 * '<S50>'  : 'control/PID Controller5/Tracking Mode Sum/Passthrough'
 * '<S51>'  : 'control/PID Controller5/Tsamp - Integral/Passthrough'
 * '<S52>'  : 'control/PID Controller5/Tsamp - Ngain/Passthrough'
 * '<S53>'  : 'control/PID Controller5/postSat Signal/Forward_Path'
 * '<S54>'  : 'control/PID Controller5/preSat Signal/Forward_Path'
 * '<S55>'  : 'control/PID Controller6/Anti-windup'
 * '<S56>'  : 'control/PID Controller6/D Gain'
 * '<S57>'  : 'control/PID Controller6/Filter'
 * '<S58>'  : 'control/PID Controller6/Filter ICs'
 * '<S59>'  : 'control/PID Controller6/I Gain'
 * '<S60>'  : 'control/PID Controller6/Ideal P Gain'
 * '<S61>'  : 'control/PID Controller6/Ideal P Gain Fdbk'
 * '<S62>'  : 'control/PID Controller6/Integrator'
 * '<S63>'  : 'control/PID Controller6/Integrator ICs'
 * '<S64>'  : 'control/PID Controller6/N Copy'
 * '<S65>'  : 'control/PID Controller6/N Gain'
 * '<S66>'  : 'control/PID Controller6/P Copy'
 * '<S67>'  : 'control/PID Controller6/Parallel P Gain'
 * '<S68>'  : 'control/PID Controller6/Reset Signal'
 * '<S69>'  : 'control/PID Controller6/Saturation'
 * '<S70>'  : 'control/PID Controller6/Saturation Fdbk'
 * '<S71>'  : 'control/PID Controller6/Sum'
 * '<S72>'  : 'control/PID Controller6/Sum Fdbk'
 * '<S73>'  : 'control/PID Controller6/Tracking Mode'
 * '<S74>'  : 'control/PID Controller6/Tracking Mode Sum'
 * '<S75>'  : 'control/PID Controller6/Tsamp - Integral'
 * '<S76>'  : 'control/PID Controller6/Tsamp - Ngain'
 * '<S77>'  : 'control/PID Controller6/postSat Signal'
 * '<S78>'  : 'control/PID Controller6/preSat Signal'
 * '<S79>'  : 'control/PID Controller6/Anti-windup/Passthrough'
 * '<S80>'  : 'control/PID Controller6/D Gain/Internal Parameters'
 * '<S81>'  : 'control/PID Controller6/Filter/Differentiator'
 * '<S82>'  : 'control/PID Controller6/Filter/Differentiator/Tsamp'
 * '<S83>'  : 'control/PID Controller6/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S84>'  : 'control/PID Controller6/Filter ICs/Internal IC - Differentiator'
 * '<S85>'  : 'control/PID Controller6/I Gain/Internal Parameters'
 * '<S86>'  : 'control/PID Controller6/Ideal P Gain/Passthrough'
 * '<S87>'  : 'control/PID Controller6/Ideal P Gain Fdbk/Disabled'
 * '<S88>'  : 'control/PID Controller6/Integrator/Discrete'
 * '<S89>'  : 'control/PID Controller6/Integrator ICs/Internal IC'
 * '<S90>'  : 'control/PID Controller6/N Copy/Disabled wSignal Specification'
 * '<S91>'  : 'control/PID Controller6/N Gain/Passthrough'
 * '<S92>'  : 'control/PID Controller6/P Copy/Disabled'
 * '<S93>'  : 'control/PID Controller6/Parallel P Gain/Internal Parameters'
 * '<S94>'  : 'control/PID Controller6/Reset Signal/Disabled'
 * '<S95>'  : 'control/PID Controller6/Saturation/Passthrough'
 * '<S96>'  : 'control/PID Controller6/Saturation Fdbk/Disabled'
 * '<S97>'  : 'control/PID Controller6/Sum/Sum_PID'
 * '<S98>'  : 'control/PID Controller6/Sum Fdbk/Disabled'
 * '<S99>'  : 'control/PID Controller6/Tracking Mode/Disabled'
 * '<S100>' : 'control/PID Controller6/Tracking Mode Sum/Passthrough'
 * '<S101>' : 'control/PID Controller6/Tsamp - Integral/Passthrough'
 * '<S102>' : 'control/PID Controller6/Tsamp - Ngain/Passthrough'
 * '<S103>' : 'control/PID Controller6/postSat Signal/Forward_Path'
 * '<S104>' : 'control/PID Controller6/preSat Signal/Forward_Path'
 * '<S105>' : 'control/PID Controller7/Anti-windup'
 * '<S106>' : 'control/PID Controller7/D Gain'
 * '<S107>' : 'control/PID Controller7/Filter'
 * '<S108>' : 'control/PID Controller7/Filter ICs'
 * '<S109>' : 'control/PID Controller7/I Gain'
 * '<S110>' : 'control/PID Controller7/Ideal P Gain'
 * '<S111>' : 'control/PID Controller7/Ideal P Gain Fdbk'
 * '<S112>' : 'control/PID Controller7/Integrator'
 * '<S113>' : 'control/PID Controller7/Integrator ICs'
 * '<S114>' : 'control/PID Controller7/N Copy'
 * '<S115>' : 'control/PID Controller7/N Gain'
 * '<S116>' : 'control/PID Controller7/P Copy'
 * '<S117>' : 'control/PID Controller7/Parallel P Gain'
 * '<S118>' : 'control/PID Controller7/Reset Signal'
 * '<S119>' : 'control/PID Controller7/Saturation'
 * '<S120>' : 'control/PID Controller7/Saturation Fdbk'
 * '<S121>' : 'control/PID Controller7/Sum'
 * '<S122>' : 'control/PID Controller7/Sum Fdbk'
 * '<S123>' : 'control/PID Controller7/Tracking Mode'
 * '<S124>' : 'control/PID Controller7/Tracking Mode Sum'
 * '<S125>' : 'control/PID Controller7/Tsamp - Integral'
 * '<S126>' : 'control/PID Controller7/Tsamp - Ngain'
 * '<S127>' : 'control/PID Controller7/postSat Signal'
 * '<S128>' : 'control/PID Controller7/preSat Signal'
 * '<S129>' : 'control/PID Controller7/Anti-windup/Passthrough'
 * '<S130>' : 'control/PID Controller7/D Gain/Internal Parameters'
 * '<S131>' : 'control/PID Controller7/Filter/Differentiator'
 * '<S132>' : 'control/PID Controller7/Filter/Differentiator/Tsamp'
 * '<S133>' : 'control/PID Controller7/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S134>' : 'control/PID Controller7/Filter ICs/Internal IC - Differentiator'
 * '<S135>' : 'control/PID Controller7/I Gain/Internal Parameters'
 * '<S136>' : 'control/PID Controller7/Ideal P Gain/Passthrough'
 * '<S137>' : 'control/PID Controller7/Ideal P Gain Fdbk/Disabled'
 * '<S138>' : 'control/PID Controller7/Integrator/Discrete'
 * '<S139>' : 'control/PID Controller7/Integrator ICs/Internal IC'
 * '<S140>' : 'control/PID Controller7/N Copy/Disabled wSignal Specification'
 * '<S141>' : 'control/PID Controller7/N Gain/Passthrough'
 * '<S142>' : 'control/PID Controller7/P Copy/Disabled'
 * '<S143>' : 'control/PID Controller7/Parallel P Gain/Internal Parameters'
 * '<S144>' : 'control/PID Controller7/Reset Signal/Disabled'
 * '<S145>' : 'control/PID Controller7/Saturation/Passthrough'
 * '<S146>' : 'control/PID Controller7/Saturation Fdbk/Disabled'
 * '<S147>' : 'control/PID Controller7/Sum/Sum_PID'
 * '<S148>' : 'control/PID Controller7/Sum Fdbk/Disabled'
 * '<S149>' : 'control/PID Controller7/Tracking Mode/Disabled'
 * '<S150>' : 'control/PID Controller7/Tracking Mode Sum/Passthrough'
 * '<S151>' : 'control/PID Controller7/Tsamp - Integral/Passthrough'
 * '<S152>' : 'control/PID Controller7/Tsamp - Ngain/Passthrough'
 * '<S153>' : 'control/PID Controller7/postSat Signal/Forward_Path'
 * '<S154>' : 'control/PID Controller7/preSat Signal/Forward_Path'
 * '<S155>' : 'control/PID Controller8/Anti-windup'
 * '<S156>' : 'control/PID Controller8/D Gain'
 * '<S157>' : 'control/PID Controller8/Filter'
 * '<S158>' : 'control/PID Controller8/Filter ICs'
 * '<S159>' : 'control/PID Controller8/I Gain'
 * '<S160>' : 'control/PID Controller8/Ideal P Gain'
 * '<S161>' : 'control/PID Controller8/Ideal P Gain Fdbk'
 * '<S162>' : 'control/PID Controller8/Integrator'
 * '<S163>' : 'control/PID Controller8/Integrator ICs'
 * '<S164>' : 'control/PID Controller8/N Copy'
 * '<S165>' : 'control/PID Controller8/N Gain'
 * '<S166>' : 'control/PID Controller8/P Copy'
 * '<S167>' : 'control/PID Controller8/Parallel P Gain'
 * '<S168>' : 'control/PID Controller8/Reset Signal'
 * '<S169>' : 'control/PID Controller8/Saturation'
 * '<S170>' : 'control/PID Controller8/Saturation Fdbk'
 * '<S171>' : 'control/PID Controller8/Sum'
 * '<S172>' : 'control/PID Controller8/Sum Fdbk'
 * '<S173>' : 'control/PID Controller8/Tracking Mode'
 * '<S174>' : 'control/PID Controller8/Tracking Mode Sum'
 * '<S175>' : 'control/PID Controller8/Tsamp - Integral'
 * '<S176>' : 'control/PID Controller8/Tsamp - Ngain'
 * '<S177>' : 'control/PID Controller8/postSat Signal'
 * '<S178>' : 'control/PID Controller8/preSat Signal'
 * '<S179>' : 'control/PID Controller8/Anti-windup/Passthrough'
 * '<S180>' : 'control/PID Controller8/D Gain/Internal Parameters'
 * '<S181>' : 'control/PID Controller8/Filter/Differentiator'
 * '<S182>' : 'control/PID Controller8/Filter/Differentiator/Tsamp'
 * '<S183>' : 'control/PID Controller8/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S184>' : 'control/PID Controller8/Filter ICs/Internal IC - Differentiator'
 * '<S185>' : 'control/PID Controller8/I Gain/Internal Parameters'
 * '<S186>' : 'control/PID Controller8/Ideal P Gain/Passthrough'
 * '<S187>' : 'control/PID Controller8/Ideal P Gain Fdbk/Disabled'
 * '<S188>' : 'control/PID Controller8/Integrator/Discrete'
 * '<S189>' : 'control/PID Controller8/Integrator ICs/Internal IC'
 * '<S190>' : 'control/PID Controller8/N Copy/Disabled wSignal Specification'
 * '<S191>' : 'control/PID Controller8/N Gain/Passthrough'
 * '<S192>' : 'control/PID Controller8/P Copy/Disabled'
 * '<S193>' : 'control/PID Controller8/Parallel P Gain/Internal Parameters'
 * '<S194>' : 'control/PID Controller8/Reset Signal/Disabled'
 * '<S195>' : 'control/PID Controller8/Saturation/Passthrough'
 * '<S196>' : 'control/PID Controller8/Saturation Fdbk/Disabled'
 * '<S197>' : 'control/PID Controller8/Sum/Sum_PID'
 * '<S198>' : 'control/PID Controller8/Sum Fdbk/Disabled'
 * '<S199>' : 'control/PID Controller8/Tracking Mode/Disabled'
 * '<S200>' : 'control/PID Controller8/Tracking Mode Sum/Passthrough'
 * '<S201>' : 'control/PID Controller8/Tsamp - Integral/Passthrough'
 * '<S202>' : 'control/PID Controller8/Tsamp - Ngain/Passthrough'
 * '<S203>' : 'control/PID Controller8/postSat Signal/Forward_Path'
 * '<S204>' : 'control/PID Controller8/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
