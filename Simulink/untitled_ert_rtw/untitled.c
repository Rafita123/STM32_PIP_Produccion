/*
 * File: untitled.c
 *
 * Code generated for Simulink model 'untitled'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Sat May 14 16:52:56 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "untitled.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void untitled_step(void)
{
  real_T rtb_Sum;

  /* Sum: '<S2>/Sum' incorporates:
   *  Gain: '<S2>/Gain'
   *  Inport: '<Root>/In1'
   *  Sum: '<S2>/Diff'
   *  UnitDelay: '<S2>/UD'
   *
   * Block description for '<S2>/Sum':
   *
   *  Add in CPU
   *
   * Block description for '<S2>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  rtb_Sum = (rtDW.UD_DSTATE - rtU.In1) * 0.95 + rtU.In1;

  /* Outport: '<Root>/Out1' */
  rtY.Out1 = rtb_Sum;

  /* Update for UnitDelay: '<S2>/UD'
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE = rtb_Sum;
}

/* Model initialize function */
void untitled_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
