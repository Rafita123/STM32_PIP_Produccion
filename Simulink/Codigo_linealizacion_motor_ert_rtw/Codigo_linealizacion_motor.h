/*
 * File: Codigo_linealizacion_motor.h
 *
 * Code generated for Simulink model 'Codigo_linealizacion_motor'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sun Apr 17 20:07:35 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Codigo_linealizacion_motor_h_
#define RTW_HEADER_Codigo_linealizacion_motor_h_
#ifndef Codigo_linealizacion_motor_COMMON_INCLUDES_
#define Codigo_linealizacion_motor_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                         /* Codigo_linealizacion_motor_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Model entry point functions */
extern void Codigo_linealizacion_motor_initialize(void);
extern void Codigo_linealizacion_motor_step(void);

/* Exported data declaration */

/* Data with Exported storage */
extern real_T rtEntrada_Linealizacion1;/* '<Root>/Entrada_Linealizacion1' */
extern uint32_T rtSalida_Linealizacion1;/* '<Root>/Salida_Linealizacion1' */
extern real_T rtp_SignalC1[2];         /* '<Root>/Data Store Memory3' */
extern real_T rtp_SignalC2[2];         /* '<Root>/Data Store Memory1' */
extern real_T rtp_SignalC3[2];         /* '<Root>/Data Store Memory' */
extern real_T rtp_SignalC4[2];         /* '<Root>/Data Store Memory2' */

/* Real-time Model object */
extern RT_MODEL *const rtM;

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
 * '<Root>' : 'Codigo_linealizacion_motor'
 * '<S1>'   : 'Codigo_linealizacion_motor/Condición 1'
 * '<S2>'   : 'Codigo_linealizacion_motor/Condición 2'
 * '<S3>'   : 'Codigo_linealizacion_motor/Condición 3'
 * '<S4>'   : 'Codigo_linealizacion_motor/Condición 6'
 * '<S5>'   : 'Codigo_linealizacion_motor/Conición 4'
 * '<S6>'   : 'Codigo_linealizacion_motor/Conición 5'
 * '<S7>'   : 'Codigo_linealizacion_motor/Condición 1/MATLAB Function'
 * '<S8>'   : 'Codigo_linealizacion_motor/Condición 2/MATLAB Function'
 * '<S9>'   : 'Codigo_linealizacion_motor/Condición 3/MATLAB Function'
 * '<S10>'  : 'Codigo_linealizacion_motor/Condición 6/MATLAB Function'
 * '<S11>'  : 'Codigo_linealizacion_motor/Conición 4/MATLAB Function'
 * '<S12>'  : 'Codigo_linealizacion_motor/Conición 5/MATLAB Function'
 */
#endif                            /* RTW_HEADER_Codigo_linealizacion_motor_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
