/*
 * File: Linealizacion.h
 *
 * Code generated for Simulink model 'Linealizacion'.
 *
 * Model version                  : 1.20
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sat Aug 20 11:10:34 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Linealizacion_h_
#define RTW_HEADER_Linealizacion_h_
#ifndef Linealizacion_COMMON_INCLUDES_
#define Linealizacion_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Linealizacion_COMMON_INCLUDES_ */

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
extern void Linealizacion_initialize(void);
extern void Linealizacion_step(void);

/* Exported data declaration */

/* Data with Exported storage */
extern real_T rtEntrada_Linealizacion1;/* '<Root>/Entrada_Linealizacion1' */
extern real_T rtEntrada_Linealizacion2;/* '<Root>/Entrada_Linealizacion2' */
extern real_T rtEntrada_Linealizacion3;/* '<Root>/Entrada_Linealizacion3' */
extern real_T rtEntrada_Linealizacion4;/* '<Root>/Entrada_Linealizacion4' */
extern uint32_T rtSalida_Linealizacion1;/* '<Root>/Salida_Linealizacion1' */
extern uint32_T rtSalida_Linealizacion2;/* '<Root>/Salida_Linealizacion2' */
extern uint32_T rtSalida_Linealizacion3;/* '<Root>/Salida_Linealizacion3' */
extern uint32_T rtSalida_Linealizacion4;/* '<Root>/Salida_Linealizacion4' */
extern real_T rtp_SignalC1[2];         /* '<Root>/Data Store Memory3' */
extern real_T rtp_SignalC2[2];         /* '<Root>/Data Store Memory1' */
extern real_T rtp_SignalC3[2];         /* '<Root>/Data Store Memory' */
extern real_T rtp_SignalC4[2];         /* '<Root>/Data Store Memory2' */
extern real_T rtp_SignalC5[2];         /* '<Root>/Data Store Memory5' */

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
 * '<Root>' : 'Linealizacion'
 * '<S1>'   : 'Linealizacion/Condición 1'
 * '<S2>'   : 'Linealizacion/Condición 1012'
 * '<S3>'   : 'Linealizacion/Condición 1113'
 * '<S4>'   : 'Linealizacion/Condición 12123123'
 * '<S5>'   : 'Linealizacion/Condición 12125'
 * '<S6>'   : 'Linealizacion/Condición 12127'
 * '<S7>'   : 'Linealizacion/Condición 12128'
 * '<S8>'   : 'Linealizacion/Condición 1213'
 * '<S9>'   : 'Linealizacion/Condición 126'
 * '<S10>'  : 'Linealizacion/Condición 143'
 * '<S11>'  : 'Linealizacion/Condición 144'
 * '<S12>'  : 'Linealizacion/Condición 1545'
 * '<S13>'  : 'Linealizacion/Condición 2'
 * '<S14>'  : 'Linealizacion/Condición 3'
 * '<S15>'  : 'Linealizacion/Condición 6'
 * '<S16>'  : 'Linealizacion/Condición 9123'
 * '<S17>'  : 'Linealizacion/Conición 1'
 * '<S18>'  : 'Linealizacion/Conición 12122'
 * '<S19>'  : 'Linealizacion/Conición 12123'
 * '<S20>'  : 'Linealizacion/Conición 237'
 * '<S21>'  : 'Linealizacion/Conición 4'
 * '<S22>'  : 'Linealizacion/Conición 428'
 * '<S23>'  : 'Linealizacion/Conición 454'
 * '<S24>'  : 'Linealizacion/Conición 5'
 * '<S25>'  : 'Linealizacion/Conición 516'
 * '<S26>'  : 'Linealizacion/Condición 1/MATLAB Function'
 * '<S27>'  : 'Linealizacion/Condición 1012/MATLAB Function'
 * '<S28>'  : 'Linealizacion/Condición 1113/MATLAB Function'
 * '<S29>'  : 'Linealizacion/Condición 12123123/MATLAB Function'
 * '<S30>'  : 'Linealizacion/Condición 12125/MATLAB Function'
 * '<S31>'  : 'Linealizacion/Condición 12127/MATLAB Function'
 * '<S32>'  : 'Linealizacion/Condición 12128/MATLAB Function'
 * '<S33>'  : 'Linealizacion/Condición 1213/MATLAB Function'
 * '<S34>'  : 'Linealizacion/Condición 126/MATLAB Function'
 * '<S35>'  : 'Linealizacion/Condición 143/MATLAB Function'
 * '<S36>'  : 'Linealizacion/Condición 144/MATLAB Function'
 * '<S37>'  : 'Linealizacion/Condición 1545/MATLAB Function'
 * '<S38>'  : 'Linealizacion/Condición 2/MATLAB Function'
 * '<S39>'  : 'Linealizacion/Condición 3/MATLAB Function'
 * '<S40>'  : 'Linealizacion/Condición 6/MATLAB Function'
 * '<S41>'  : 'Linealizacion/Condición 9123/MATLAB Function'
 * '<S42>'  : 'Linealizacion/Conición 1/MATLAB Function'
 * '<S43>'  : 'Linealizacion/Conición 12122/MATLAB Function'
 * '<S44>'  : 'Linealizacion/Conición 12123/MATLAB Function'
 * '<S45>'  : 'Linealizacion/Conición 237/MATLAB Function'
 * '<S46>'  : 'Linealizacion/Conición 4/MATLAB Function'
 * '<S47>'  : 'Linealizacion/Conición 428/MATLAB Function'
 * '<S48>'  : 'Linealizacion/Conición 454/MATLAB Function'
 * '<S49>'  : 'Linealizacion/Conición 5/MATLAB Function'
 * '<S50>'  : 'Linealizacion/Conición 516/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_Linealizacion_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
