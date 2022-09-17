/*
 * File: Linealizacion.h
 *
 * Code generated for Simulink model 'Linealizacion'.
 *
 * Model version                  : 1.23
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sat Sep  3 17:44:03 2022
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
 * '<S1>'   : 'Linealizacion/Condici�n 1'
 * '<S2>'   : 'Linealizacion/Condici�n 10'
 * '<S3>'   : 'Linealizacion/Condici�n 11'
 * '<S4>'   : 'Linealizacion/Condici�n 12'
 * '<S5>'   : 'Linealizacion/Condici�n 13'
 * '<S6>'   : 'Linealizacion/Condici�n 14'
 * '<S7>'   : 'Linealizacion/Condici�n 15'
 * '<S8>'   : 'Linealizacion/Condici�n 16'
 * '<S9>'   : 'Linealizacion/Condici�n 2'
 * '<S10>'  : 'Linealizacion/Condici�n 3'
 * '<S11>'  : 'Linealizacion/Condici�n 4'
 * '<S12>'  : 'Linealizacion/Condici�n 5'
 * '<S13>'  : 'Linealizacion/Condici�n 6'
 * '<S14>'  : 'Linealizacion/Condici�n 7'
 * '<S15>'  : 'Linealizacion/Condici�n 8'
 * '<S16>'  : 'Linealizacion/Condici�n 9'
 * '<S17>'  : 'Linealizacion/Conici�n 1'
 * '<S18>'  : 'Linealizacion/Conici�n 10'
 * '<S19>'  : 'Linealizacion/Conici�n 11'
 * '<S20>'  : 'Linealizacion/Conici�n 12'
 * '<S21>'  : 'Linealizacion/Conici�n 2'
 * '<S22>'  : 'Linealizacion/Conici�n 3'
 * '<S23>'  : 'Linealizacion/Conici�n 4'
 * '<S24>'  : 'Linealizacion/Conici�n 5'
 * '<S25>'  : 'Linealizacion/Conici�n 6'
 * '<S26>'  : 'Linealizacion/Conici�n 7'
 * '<S27>'  : 'Linealizacion/Conici�n 8'
 * '<S28>'  : 'Linealizacion/Conici�n 9'
 * '<S29>'  : 'Linealizacion/Condici�n 1/MATLAB Function'
 * '<S30>'  : 'Linealizacion/Condici�n 10/MATLAB Function'
 * '<S31>'  : 'Linealizacion/Condici�n 11/MATLAB Function'
 * '<S32>'  : 'Linealizacion/Condici�n 12/MATLAB Function'
 * '<S33>'  : 'Linealizacion/Condici�n 13/MATLAB Function'
 * '<S34>'  : 'Linealizacion/Condici�n 14/MATLAB Function'
 * '<S35>'  : 'Linealizacion/Condici�n 15/MATLAB Function'
 * '<S36>'  : 'Linealizacion/Condici�n 16/MATLAB Function'
 * '<S37>'  : 'Linealizacion/Condici�n 2/MATLAB Function'
 * '<S38>'  : 'Linealizacion/Condici�n 3/MATLAB Function'
 * '<S39>'  : 'Linealizacion/Condici�n 4/MATLAB Function'
 * '<S40>'  : 'Linealizacion/Condici�n 5/MATLAB Function'
 * '<S41>'  : 'Linealizacion/Condici�n 6/MATLAB Function'
 * '<S42>'  : 'Linealizacion/Condici�n 7/MATLAB Function'
 * '<S43>'  : 'Linealizacion/Condici�n 8/MATLAB Function'
 * '<S44>'  : 'Linealizacion/Condici�n 9/MATLAB Function'
 * '<S45>'  : 'Linealizacion/Conici�n 1/MATLAB Function'
 * '<S46>'  : 'Linealizacion/Conici�n 10/MATLAB Function'
 * '<S47>'  : 'Linealizacion/Conici�n 11/MATLAB Function'
 * '<S48>'  : 'Linealizacion/Conici�n 12/MATLAB Function'
 * '<S49>'  : 'Linealizacion/Conici�n 2/MATLAB Function'
 * '<S50>'  : 'Linealizacion/Conici�n 3/MATLAB Function'
 * '<S51>'  : 'Linealizacion/Conici�n 4/MATLAB Function'
 * '<S52>'  : 'Linealizacion/Conici�n 5/MATLAB Function'
 * '<S53>'  : 'Linealizacion/Conici�n 6/MATLAB Function'
 * '<S54>'  : 'Linealizacion/Conici�n 7/MATLAB Function'
 * '<S55>'  : 'Linealizacion/Conici�n 8/MATLAB Function'
 * '<S56>'  : 'Linealizacion/Conici�n 9/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_Linealizacion_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
