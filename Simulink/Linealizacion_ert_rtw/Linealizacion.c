/*
 * File: Linealizacion.c
 *
 * Code generated for Simulink model 'Linealizacion'.
 *
 * Model version                  : 1.19
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Wed Mar 30 20:01:09 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Linealizacion.h"

/* Exported data definition */

/* Data with Exported storage */
real_T rtEntrada_Linealizacion1;       /* '<Root>/Entrada_Linealizacion1' */
real_T rtEntrada_Linealizacion2;       /* '<Root>/Entrada_Linealizacion2' */
real_T rtEntrada_Linealizacion3;       /* '<Root>/Entrada_Linealizacion3' */
real_T rtEntrada_Linealizacion4;       /* '<Root>/Entrada_Linealizacion4' */
uint32_T rtSalida_Linealizacion1;      /* '<Root>/Salida_Linealizacion1' */
uint32_T rtSalida_Linealizacion2;      /* '<Root>/Salida_Linealizacion2' */
uint32_T rtSalida_Linealizacion3;      /* '<Root>/Salida_Linealizacion3' */
uint32_T rtSalida_Linealizacion4;      /* '<Root>/Salida_Linealizacion4' */
real_T rtp_SignalC1[2];                /* '<Root>/Data Store Memory3' */
real_T rtp_SignalC2[2];                /* '<Root>/Data Store Memory1' */
real_T rtp_SignalC3[2];                /* '<Root>/Data Store Memory' */
real_T rtp_SignalC4[2];                /* '<Root>/Data Store Memory2' */

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void Linealizacion_step(void)
{
  real_T rtu1_p;
  real_T rtu2_p;
  real_T rtu3_p;
  real_T rtu4_p;

  /* Gain: '<Root>/Gain1' incorporates:
   *  DataStoreWrite: '<Root>/Data Store Write4'
   *  Inport: '<Root>/Entrada_Linealizacion1'
   */
  rtu1_p = 0.0016047071409467772 * rtEntrada_Linealizacion1;

  /* Gain: '<Root>/Gain2' incorporates:
   *  DataStoreWrite: '<Root>/Data Store Write'
   *  Inport: '<Root>/Entrada_Linealizacion2'
   */
  rtu2_p = 0.0016047071409467772 * rtEntrada_Linealizacion2;

  /* Gain: '<Root>/Gain3' incorporates:
   *  DataStoreWrite: '<Root>/Data Store Write1'
   *  Inport: '<Root>/Entrada_Linealizacion3'
   */
  rtu3_p = 0.0016047071409467772 * rtEntrada_Linealizacion3;

  /* Gain: '<Root>/Gain4' incorporates:
   *  DataStoreWrite: '<Root>/Data Store Write2'
   *  Inport: '<Root>/Entrada_Linealizacion4'
   */
  rtu4_p = 0.0016047071409467772 * rtEntrada_Linealizacion4;

  /* If: '<Root>/If' incorporates:
   *  DataStoreRead: '<Root>/Data Store Read4'
   */
  if (!(rtu1_p == 0.0)) {
    if (0.0141833 > rtu1_p) {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 1' incorporates:
       *  ActionPort: '<S1>/Action Port'
       */
      /* SignalConversion generated from: '<S1>/y' incorporates:
       *  DataStoreRead: '<S1>/Data Store Read'
       *  DataStoreRead: '<S1>/Data Store Read4'
       *  MATLAB Function: '<S1>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC1[0] + rtp_SignalC1[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 1' */
    } else if ((0.8 >= rtu1_p) >= 0.0141833) {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 2' incorporates:
       *  ActionPort: '<S9>/Action Port'
       */
      /* SignalConversion generated from: '<S9>/Out1' incorporates:
       *  DataStoreRead: '<S9>/Data Store Read1'
       *  DataStoreRead: '<S9>/Data Store Read4'
       *  MATLAB Function: '<S9>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC2[0] + rtp_SignalC2[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 2' */
    } else if ((1.05 > rtu1_p) >= 0.8) {
      /* Outputs for IfAction SubSystem: '<Root>/Conición 5' incorporates:
       *  ActionPort: '<S21>/Action Port'
       */
      /* SignalConversion generated from: '<S21>/Out1' incorporates:
       *  DataStoreRead: '<S21>/Data Store Read1'
       *  DataStoreRead: '<S21>/Data Store Read4'
       *  MATLAB Function: '<S21>/MATLAB Function'
       */
      rtu1_p = rtu1_p * rtp_SignalC3[0] + rtp_SignalC3[1];

      /* End of Outputs for SubSystem: '<Root>/Conición 5' */
    } else {
      /* Outputs for IfAction SubSystem: '<Root>/Condición 3' incorporates:
       *  ActionPort: '<S10>/Action Port'
       */
      /* SignalConversion generated from: '<S10>/Out1' incorporates:
       *  Constant: '<S10>/Constant'
       *  DataStoreRead: '<S10>/Data Store Read2'
       *  MATLAB Function: '<S10>/MATLAB Function'
       */
      rtu1_p = 1.2 * rtp_SignalC4[0] + rtp_SignalC4[1];

      /* End of Outputs for SubSystem: '<Root>/Condición 3' */
    }
  }

  /* End of If: '<Root>/If' */

  /* Outport: '<Root>/Salida_Linealizacion1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   */
  rtSalida_Linealizacion1 = (uint32_T)rtu1_p;

  /* If: '<Root>/If1' incorporates:
   *  DataStoreRead: '<Root>/Data Store Read'
   */
  if (rtu2_p == 0.0) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 8' incorporates:
     *  ActionPort: '<S15>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion2' incorporates:
     *  DataStoreRead: '<S15>/Data Store Read'
     *  DataTypeConversion: '<Root>/Data Type Conversion1'
     *  MATLAB Function: '<S15>/MATLAB Function'
     */
    rtSalida_Linealizacion2 = (uint32_T)rtu2_p;

    /* End of Outputs for SubSystem: '<Root>/Condición 8' */
  } else if (0.0141833 > rtu2_p) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 5' incorporates:
     *  ActionPort: '<S12>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion2' incorporates:
     *  DataStoreRead: '<S12>/Data Store Read'
     *  DataStoreRead: '<S12>/Data Store Read1'
     *  DataTypeConversion: '<Root>/Data Type Conversion1'
     *  MATLAB Function: '<S12>/MATLAB Function'
     */
    rtSalida_Linealizacion2 = (uint32_T)(rtu2_p * rtp_SignalC1[0] +
      rtp_SignalC1[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 5' */
  } else if ((0.8 >= rtu2_p) >= 0.0141833) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 6' incorporates:
     *  ActionPort: '<S13>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion2' incorporates:
     *  DataStoreRead: '<S13>/Data Store Read'
     *  DataStoreRead: '<S13>/Data Store Read1'
     *  DataTypeConversion: '<Root>/Data Type Conversion1'
     *  MATLAB Function: '<S13>/MATLAB Function'
     */
    rtSalida_Linealizacion2 = (uint32_T)(rtu2_p * rtp_SignalC2[0] +
      rtp_SignalC2[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 6' */
  } else if ((1.05 > rtu2_p) >= 0.8) {
    /* Outputs for IfAction SubSystem: '<Root>/Conición 3' incorporates:
     *  ActionPort: '<S19>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion2' incorporates:
     *  DataStoreRead: '<S19>/Data Store Read'
     *  DataStoreRead: '<S19>/Data Store Read1'
     *  DataTypeConversion: '<Root>/Data Type Conversion1'
     *  MATLAB Function: '<S19>/MATLAB Function'
     */
    rtSalida_Linealizacion2 = (uint32_T)(rtu2_p * rtp_SignalC3[0] +
      rtp_SignalC3[1]);

    /* End of Outputs for SubSystem: '<Root>/Conición 3' */
  } else {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 7' incorporates:
     *  ActionPort: '<S14>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion2' incorporates:
     *  Constant: '<S14>/Constant'
     *  DataStoreRead: '<S14>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion1'
     *  MATLAB Function: '<S14>/MATLAB Function'
     */
    rtSalida_Linealizacion2 = (uint32_T)(1.2 * rtp_SignalC4[0] + rtp_SignalC4[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 7' */
  }

  /* End of If: '<Root>/If1' */

  /* If: '<Root>/If2' incorporates:
   *  DataStoreRead: '<Root>/Data Store Read1'
   */
  if (rtu3_p == 0.0) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 12' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion3' incorporates:
     *  DataStoreRead: '<S4>/Data Store Read1'
     *  DataTypeConversion: '<Root>/Data Type Conversion2'
     *  MATLAB Function: '<S4>/MATLAB Function'
     */
    rtSalida_Linealizacion3 = (uint32_T)rtu3_p;

    /* End of Outputs for SubSystem: '<Root>/Condición 12' */
  } else if (0.0141833 > rtu3_p) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 9' incorporates:
     *  ActionPort: '<S16>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion3' incorporates:
     *  DataStoreRead: '<S16>/Data Store Read'
     *  DataStoreRead: '<S16>/Data Store Read1'
     *  DataTypeConversion: '<Root>/Data Type Conversion2'
     *  MATLAB Function: '<S16>/MATLAB Function'
     */
    rtSalida_Linealizacion3 = (uint32_T)(rtu3_p * rtp_SignalC1[0] +
      rtp_SignalC1[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 9' */
  } else if ((0.8 >= rtu3_p) >= 0.0141833) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 10' incorporates:
     *  ActionPort: '<S2>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion3' incorporates:
     *  DataStoreRead: '<S2>/Data Store Read1'
     *  DataStoreRead: '<S2>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion2'
     *  MATLAB Function: '<S2>/MATLAB Function'
     */
    rtSalida_Linealizacion3 = (uint32_T)(rtu3_p * rtp_SignalC2[0] +
      rtp_SignalC2[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 10' */
  } else if ((1.05 > rtu3_p) >= 0.8) {
    /* Outputs for IfAction SubSystem: '<Root>/Conición 6' incorporates:
     *  ActionPort: '<S22>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion3' incorporates:
     *  DataStoreRead: '<S22>/Data Store Read1'
     *  DataStoreRead: '<S22>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion2'
     *  MATLAB Function: '<S22>/MATLAB Function'
     */
    rtSalida_Linealizacion3 = (uint32_T)(rtu3_p * rtp_SignalC3[0] +
      rtp_SignalC3[1]);

    /* End of Outputs for SubSystem: '<Root>/Conición 6' */
  } else {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 11' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion3' incorporates:
     *  Constant: '<S3>/Constant'
     *  DataStoreRead: '<S3>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion2'
     *  MATLAB Function: '<S3>/MATLAB Function'
     */
    rtSalida_Linealizacion3 = (uint32_T)(1.2 * rtp_SignalC4[0] + rtp_SignalC4[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 11' */
  }

  /* End of If: '<Root>/If2' */

  /* If: '<Root>/If3' incorporates:
   *  DataStoreRead: '<Root>/Data Store Read2'
   */
  if (rtu4_p == 0.0) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 16' incorporates:
     *  ActionPort: '<S8>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion4' incorporates:
     *  DataStoreRead: '<S8>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion3'
     *  MATLAB Function: '<S8>/MATLAB Function'
     */
    rtSalida_Linealizacion4 = (uint32_T)rtu4_p;

    /* End of Outputs for SubSystem: '<Root>/Condición 16' */
  } else if (0.0141833 > rtu4_p) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 13' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion4' incorporates:
     *  DataStoreRead: '<S5>/Data Store Read'
     *  DataStoreRead: '<S5>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion3'
     *  MATLAB Function: '<S5>/MATLAB Function'
     */
    rtSalida_Linealizacion4 = (uint32_T)(rtu4_p * rtp_SignalC1[0] +
      rtp_SignalC1[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 13' */
  } else if ((0.8 >= rtu4_p) >= 0.0141833) {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 14' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion4' incorporates:
     *  DataStoreRead: '<S6>/Data Store Read1'
     *  DataStoreRead: '<S6>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion3'
     *  MATLAB Function: '<S6>/MATLAB Function'
     */
    rtSalida_Linealizacion4 = (uint32_T)(rtu4_p * rtp_SignalC2[0] +
      rtp_SignalC2[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 14' */
  } else if ((1.05 > rtu4_p) >= 0.8) {
    /* Outputs for IfAction SubSystem: '<Root>/Conición 8' incorporates:
     *  ActionPort: '<S24>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion4' incorporates:
     *  DataStoreRead: '<S24>/Data Store Read1'
     *  DataStoreRead: '<S24>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion3'
     *  MATLAB Function: '<S24>/MATLAB Function'
     */
    rtSalida_Linealizacion4 = (uint32_T)(rtu4_p * rtp_SignalC3[0] +
      rtp_SignalC3[1]);

    /* End of Outputs for SubSystem: '<Root>/Conición 8' */
  } else {
    /* Outputs for IfAction SubSystem: '<Root>/Condición 15' incorporates:
     *  ActionPort: '<S7>/Action Port'
     */
    /* Outport: '<Root>/Salida_Linealizacion4' incorporates:
     *  Constant: '<S7>/Constant'
     *  DataStoreRead: '<S7>/Data Store Read2'
     *  DataTypeConversion: '<Root>/Data Type Conversion3'
     *  MATLAB Function: '<S7>/MATLAB Function'
     */
    rtSalida_Linealizacion4 = (uint32_T)(1.2 * rtp_SignalC4[0] + rtp_SignalC4[1]);

    /* End of Outputs for SubSystem: '<Root>/Condición 15' */
  }

  /* End of If: '<Root>/If3' */
}

/* Model initialize function */
void Linealizacion_initialize(void)
{
  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  rtp_SignalC3[0] = 3113.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory1' */
  rtp_SignalC2[0] = 1439.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory2' */
  rtp_SignalC4[0] = 6778.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory3' */
  rtp_SignalC1[0] = 42990.0;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  rtp_SignalC3[1] = -691.0;

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
