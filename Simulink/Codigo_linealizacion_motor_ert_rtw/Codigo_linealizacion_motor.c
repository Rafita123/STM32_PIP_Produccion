/*
 * File: Codigo_linealizacion_motor.c
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

#include "Codigo_linealizacion_motor.h"

/* Exported data definition */

/* Data with Exported storage */
real_T rtEntrada_Linealizacion1;       /* '<Root>/Entrada_Linealizacion1' */
uint32_T rtSalida_Linealizacion1;      /* '<Root>/Salida_Linealizacion1' */
real_T rtp_SignalC1[2];                /* '<Root>/Data Store Memory3' */
real_T rtp_SignalC2[2];                /* '<Root>/Data Store Memory1' */
real_T rtp_SignalC3[2];                /* '<Root>/Data Store Memory' */
real_T rtp_SignalC4[2];                /* '<Root>/Data Store Memory2' */

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void Codigo_linealizacion_motor_step(void)
{
  real_T rtu1_p;

  /* Gain: '<Root>/Gain1' incorporates:
   *  DataStoreWrite: '<Root>/Data Store Write4'
   *  Inport: '<Root>/Entrada_Linealizacion1'
   */
  rtu1_p = 0.0016047071409467772 * rtEntrada_Linealizacion1;

  /* If: '<Root>/If' incorporates:
   *  DataStoreRead: '<Root>/Data Store Read4'
   */
  if (!(rtu1_p == 0.0)) {
    if (0.0141833 > rtu1_p) {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 2' incorporates:
       *  ActionPort: '<S2>/Action Port'
       */
      /* SignalConversion generated from: '<S2>/y' incorporates:
       *  DataStoreRead: '<S2>/Data Store Read'
       *  DataStoreRead: '<S2>/Data Store Read4'
       *  MATLAB Function: '<S2>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC1[0] + rtp_SignalC1[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 2' */
    } else if ((0.8 >= rtu1_p) >= 0.0141833) {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 3' incorporates:
       *  ActionPort: '<S3>/Action Port'
       */
      /* SignalConversion generated from: '<S3>/Out1' incorporates:
       *  DataStoreRead: '<S3>/Data Store Read1'
       *  DataStoreRead: '<S3>/Data Store Read4'
       *  MATLAB Function: '<S3>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC2[0] + rtp_SignalC2[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 3' */
    } else if ((1.05 > rtu1_p) >= 0.8) {
      /* Outputs for IfAction SubSystem: '<Root>/Conición 4' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      /* SignalConversion generated from: '<S5>/Out1' incorporates:
       *  DataStoreRead: '<S5>/Data Store Read1'
       *  DataStoreRead: '<S5>/Data Store Read4'
       *  MATLAB Function: '<S5>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC3[0] + rtp_SignalC3[1];

      /* End of Outputs for SubSystem: '<Root>/Conición 4' */
    } else {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 6' incorporates:
       *  ActionPort: '<S4>/Action Port'
       */
      /* SignalConversion generated from: '<S4>/Out1' incorporates:
       *  Constant: '<S4>/Constant'
       *  DataStoreRead: '<S4>/Data Store Read2'
       *  MATLAB Function: '<S4>/MATLAB Function'
       */
      rtu1_p = 1.2 * rtp_SignalC4[0] + rtp_SignalC4[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 6' */
    }
  }

  /* End of If: '<Root>/If' */

  /* Outport: '<Root>/Salida_Linealizacion1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   */
  rtSalida_Linealizacion1 = (uint32_T)rtu1_p;
}

/* Model initialize function */
void Codigo_linealizacion_motor_initialize(void)
{
  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  rtp_SignalC3[0] = 3113.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
  rtp_SignalC2[0] = 1439.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
  rtp_SignalC4[0] = 6830.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
  rtp_SignalC1[0] = 42990.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  rtp_SignalC3[1] = -735.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
  rtp_SignalC2[1] = 606.7;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
  rtp_SignalC4[1] = -4663.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
  rtp_SignalC1[1] = 0.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
