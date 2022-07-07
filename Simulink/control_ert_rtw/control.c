/*
 * File: control.c
 *
 * Code generated for Simulink model 'control'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sat Apr  2 12:37:04 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "control.h"

/* Exported data definition */

/* Data with Exported storage */
real_T rtEntrada_Control1;             /* '<Root>/Entrada_Control1' */
real_T rtEntrada_Control2;             /* '<Root>/Entrada_Control2' */
real_T rtEntrada_Control3;             /* '<Root>/Entrada_Control3' */
real_T rtEntrada_Control4;             /* '<Root>/Entrada_Control4' */
real_T rtIntegrator_DSTATE;            /* '<S38>/Integrator' */
real_T rtIntegrator_DSTATE_g;          /* '<S88>/Integrator' */
real_T rtIntegrator_DSTATE_gr;         /* '<S138>/Integrator' */
real_T rtIntegrator_DSTATE_grm;        /* '<S188>/Integrator' */
real_T rtSalida_Control1;              /* '<Root>/Salida_Control1' */
real_T rtSalida_Control2;              /* '<Root>/Salida_Control2' */
real_T rtSalida_Control3;              /* '<Root>/Salida_Control3' */
real_T rtSalida_Control4;              /* '<Root>/Salida_Control4' */
real_T rtUD_DSTATE;                    /* '<S31>/UD' */
real_T rtUD_DSTATE_m;                  /* '<S81>/UD' */
real_T rtUD_DSTATE_mr;                 /* '<S131>/UD' */
real_T rtUD_DSTATE_mrz;                /* '<S181>/UD' */

/* Model step function */
void control_step(void)
{
  real_T rtb_Tsamp;
  real_T rtb_Tsamp_e;
  real_T rtb_Tsamp_fo;
  real_T rtb_Tsamp_pd;

  /* SampleTimeMath: '<S33>/Tsamp' incorporates:
   *  Gain: '<S30>/Derivative Gain'
   *  Inport: '<Root>/Entrada_Control1'
   *
   * About '<S33>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp = 0.00285610125568279 * rtEntrada_Control1 * 200.0;

  /* Outport: '<Root>/Salida_Control1' incorporates:
   *  Delay: '<S31>/UD'
   *  DiscreteIntegrator: '<S38>/Integrator'
   *  Gain: '<S43>/Proportional Gain'
   *  Inport: '<Root>/Entrada_Control1'
   *  Sum: '<S31>/Diff'
   *  Sum: '<S47>/Sum'
   */
  rtSalida_Control1 = (2.28488100454623 * rtEntrada_Control1 +
                       rtIntegrator_DSTATE) + (rtb_Tsamp - rtUD_DSTATE);

  /* SampleTimeMath: '<S83>/Tsamp' incorporates:
   *  Gain: '<S80>/Derivative Gain'
   *  Inport: '<Root>/Entrada_Control2'
   *
   * About '<S83>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp_e = 0.00285610125568279 * rtEntrada_Control2 * 200.0;

  /* Outport: '<Root>/Salida_Control2' incorporates:
   *  Delay: '<S81>/UD'
   *  DiscreteIntegrator: '<S88>/Integrator'
   *  Gain: '<S93>/Proportional Gain'
   *  Inport: '<Root>/Entrada_Control2'
   *  Sum: '<S81>/Diff'
   *  Sum: '<S97>/Sum'
   */
  rtSalida_Control2 = (2.28488100454623 * rtEntrada_Control2 + rtIntegrator_DSTATE_g) +
    (rtb_Tsamp_e - rtUD_DSTATE_m);

  /* SampleTimeMath: '<S133>/Tsamp' incorporates:
   *  Gain: '<S130>/Derivative Gain'
   *  Inport: '<Root>/Entrada_Control3'
   *
   * About '<S133>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp_fo = 0.00285610125568279 * rtEntrada_Control3 * 200.0;

  /* Outport: '<Root>/Salida_Control3' incorporates:
   *  Delay: '<S131>/UD'
   *  DiscreteIntegrator: '<S138>/Integrator'
   *  Gain: '<S143>/Proportional Gain'
   *  Inport: '<Root>/Entrada_Control3'
   *  Sum: '<S131>/Diff'
   *  Sum: '<S147>/Sum'
   */
  rtSalida_Control3 = (2.28488100454623 * rtEntrada_Control3 +
                       rtIntegrator_DSTATE_gr) + (rtb_Tsamp_fo - rtUD_DSTATE_mr);

  /* SampleTimeMath: '<S183>/Tsamp' incorporates:
   *  Gain: '<S180>/Derivative Gain'
   *  Inport: '<Root>/Entrada_Control4'
   *
   * About '<S183>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp_pd = 0.00285610125568279 * rtEntrada_Control4 * 200.0;

  /* Outport: '<Root>/Salida_Control4' incorporates:
   *  Delay: '<S181>/UD'
   *  DiscreteIntegrator: '<S188>/Integrator'
   *  Gain: '<S193>/Proportional Gain'
   *  Inport: '<Root>/Entrada_Control4'
   *  Sum: '<S181>/Diff'
   *  Sum: '<S197>/Sum'
   */
  rtSalida_Control4 = (2.28488100454623 * rtEntrada_Control4 +
                       rtIntegrator_DSTATE_grm) + (rtb_Tsamp_pd -
    rtUD_DSTATE_mrz);

  /* Update for DiscreteIntegrator: '<S38>/Integrator' incorporates:
   *  Gain: '<S35>/Integral Gain'
   *  Inport: '<Root>/Entrada_Control1'
   */
  rtIntegrator_DSTATE = 456.976200909247 * rtEntrada_Control1 * 0.005 +
    rtIntegrator_DSTATE;

  /* Update for Delay: '<S31>/UD' */
  rtUD_DSTATE = rtb_Tsamp;

  /* Update for DiscreteIntegrator: '<S88>/Integrator' incorporates:
   *  Gain: '<S85>/Integral Gain'
   *  Inport: '<Root>/Entrada_Control2'
   */
  rtIntegrator_DSTATE_g = 456.976200909247 * rtEntrada_Control2 * 0.005 +
    rtIntegrator_DSTATE_g;

  /* Update for Delay: '<S81>/UD' */
  rtUD_DSTATE_m = rtb_Tsamp_e;

  /* Update for DiscreteIntegrator: '<S138>/Integrator' incorporates:
   *  Gain: '<S135>/Integral Gain'
   *  Inport: '<Root>/Entrada_Control3'
   */
  rtIntegrator_DSTATE_gr = 456.976200909247 * rtEntrada_Control3 * 0.005 +
    rtIntegrator_DSTATE_gr;

  /* Update for Delay: '<S131>/UD' */
  rtUD_DSTATE_mr = rtb_Tsamp_fo;

  /* Update for DiscreteIntegrator: '<S188>/Integrator' incorporates:
   *  Gain: '<S185>/Integral Gain'
   *  Inport: '<Root>/Entrada_Control4'
   */
  rtIntegrator_DSTATE_grm = 456.976200909247 * rtEntrada_Control4 * 0.005 +
    rtIntegrator_DSTATE_grm;

  /* Update for Delay: '<S181>/UD' */
  rtUD_DSTATE_mrz = rtb_Tsamp_pd;
}

/* Model initialize function */
void control_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
