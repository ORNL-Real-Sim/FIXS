/*
 * tmp.c
 *
 * Code generation for model "tmp".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jul 10 17:36:05 2023
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "tmp_dsrtvdf.h"
#include "tmp.h"
#include "tmp_private.h"

/* Block signals (default storage) */
B_tmp_T tmp_B;

/* Real-time model */
RT_MODEL_tmp_T tmp_M_;
RT_MODEL_tmp_T *const tmp_M = &tmp_M_;

/* Model output function */
void tmp_output(void)
{
  /* Sin: '<Root>/Sine Wave' */
  tmp_B.sinewave = sin(tmp_P.SineWave_Freq * tmp_M->Timing.t[0] +
                       tmp_P.SineWave_Phase) * tmp_P.SineWave_Amp +
    tmp_P.SineWave_Bias;
}

/* Model update function */
void tmp_update(void)
{
  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++tmp_M->Timing.clockTick0)) {
    ++tmp_M->Timing.clockTickH0;
  }

  tmp_M->Timing.t[0] = tmp_M->Timing.clockTick0 * tmp_M->Timing.stepSize0 +
    tmp_M->Timing.clockTickH0 * tmp_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    tmp_M->Timing.clockTick1++;
    if (!tmp_M->Timing.clockTick1) {
      tmp_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void tmp_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)tmp_M, 0,
                sizeof(RT_MODEL_tmp_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&tmp_M->solverInfo, &tmp_M->Timing.simTimeStep);
    rtsiSetTPtr(&tmp_M->solverInfo, &rtmGetTPtr(tmp_M));
    rtsiSetStepSizePtr(&tmp_M->solverInfo, &tmp_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&tmp_M->solverInfo, (&rtmGetErrorStatus(tmp_M)));
    rtsiSetRTModelPtr(&tmp_M->solverInfo, tmp_M);
  }

  rtsiSetSimTimeStep(&tmp_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&tmp_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(tmp_M, &tmp_M->Timing.tArray[0]);
  tmp_M->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) &tmp_B), 0,
                sizeof(B_tmp_T));

  {
    /* user code (registration function declaration) */
    /*Initialize global TRC pointers. */
    tmp_rti_init_trc_pointers();
  }
}

/* Model terminate function */
void tmp_terminate(void)
{
  /* (no terminate code required) */
}
