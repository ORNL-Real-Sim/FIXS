/*
 * CAVE_MachE_sil_test.c
 *
 * Code generation for model "CAVE_MachE_sil_test".
 *
 * Model version              : 1.49
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Tue Aug 22 23:31:18 2023
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CAVE_MachE_sil_test_dsrtvdf.h"
#include "CAVE_MachE_sil_test.h"
#include "CAVE_MachE_sil_test_private.h"

/* Named constants for Chart: '<S346>/LockUp' */
#define CAVE_MachE_sil_test_IN_Locked  ((uint8_T)1U)
#define CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define CAVE_MachE_sil_test_IN_Slipping ((uint8_T)2U)

/* Named constants for Chart: '<S102>/Band-Aid' */
#define CAVE_MachE_sil_test_IN_Blend   ((uint8_T)1U)
#define CAVE_MachE_sil_test_IN_Driving ((uint8_T)2U)
#define CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k ((uint8_T)0U)
#define CAVE_MachE_sil_test_IN_Running ((uint8_T)1U)
#define CAVE_MachE_sil_test_IN_SpeedMode ((uint8_T)1U)
#define CAVE_MachE_sil_test_IN_TorqueMode ((uint8_T)2U)
#define CAVE_MachE_sil_test_IN_dStop   ((uint8_T)2U)

/* Block signals (default storage) */
B_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_B;

/* Continuous states */
X_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_X;

/* Periodic continuous states */
PeriodicIndX_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_PeriodicIndX;
PeriodicRngX_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_PeriodicRngX;

/* Block states (default storage) */
DW_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_DW;

/* Real-time model */
RT_MODEL_CAVE_MachE_sil_test_T CAVE_MachE_sil_test_M_;
RT_MODEL_CAVE_MachE_sil_test_T *const CAVE_MachE_sil_test_M =
  &CAVE_MachE_sil_test_M_;

/* Forward declaration for local functions */
static void CAVE_MachE_sil_test_power(const real_T a_data[], const int32_T
  a_size[2], real_T y_data[], int32_T y_size[2]);
static void CAVE_MachE_sil_test_div0protect(real_T u, real_T tol, real_T *y,
  real_T *yabs);
static boolean_T CAVE_MachE_sil_test_any(boolean_T x);
static real_T CAVE_MachE_sil_test_div0protect_p(real_T u);
static void CAVE_MachE_sil_test_sin(real_T x_data[], const int32_T x_size[2]);
static void CAVE_MachE_sil_test_abs(const real_T x_data[], const int32_T x_size
  [2], real_T y_data[], int32_T y_size[2]);
static void CAVE_MachE_sil_test_atan(real_T x_data[], const int32_T x_size[2]);
static void CAVE_MachE_sil_test_cos(real_T x_data[], const int32_T x_size[2]);
static void CAVE_MachE_sil_test_sqrt(real_T x_data[], const int32_T x_size[2]);
static void CAVE_MachE_sil_test_div0protect_pe(real_T u, real_T *y, real_T *yabs);
static void CAVE_MachE_sil_test_acos(real_T x_data[], const int32_T x_size[2]);
static void CAVE_MachE_sil_test_tanh(real_T x_data[], const int32_T x_size[2]);
static real_T CAVE_MachE_sil_test_rollingMoment(real_T Fx, real_T Vcx, real_T Fz,
  real_T press, real_T b_gamma, real_T Vo, real_T Ro, real_T Fzo, real_T pio,
  real_T b_QSY1, real_T b_QSY2, real_T b_QSY3, real_T b_QSY4, real_T b_QSY5,
  real_T b_QSY6, real_T b_QSY7, real_T b_QSY8, real_T lam_My);

/* Forward declaration for local functions */
static boolean_T CAVE_MachE_sil_test_detectLockup(real_T Tout, real_T Tfmaxs,
  B_LockUp_CAVE_MachE_sil_test_T *localB, DW_LockUp_CAVE_MachE_sil_test_T
  *localDW, P_LockUp_CAVE_MachE_sil_test_T *localP, real_T rtp_br);
static boolean_T CAVE_MachE_sil_test_detectSlip(real_T Tout, real_T Tfmaxs,
  B_LockUp_CAVE_MachE_sil_test_T *localB);
real_T look1_binlcpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T look1_binlcapw(real_T u0, const real_T bp0[], const real_T table[],
                      uint32_T maxIndex)
{
  real_T y;
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex;
    frac = 0.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  if (iLeft == maxIndex) {
    y = table[iLeft];
  } else {
    y = (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
  }

  return y;
}

real_T look2_binlcapw(real_T u0, real_T u1, const real_T bp0[], const real_T
                      bp1[], const real_T table[], const uint32_T maxIndex[],
                      uint32_T stride)
{
  real_T y;
  real_T frac;
  uint32_T bpIndices[2];
  real_T fractions[2];
  real_T yR_1d;
  uint32_T iRght;
  uint32_T bpIdx;
  uint32_T iLeft;

  /* Column-major Lookup 2-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex[0U]]) {
    /* Binary Search */
    bpIdx = maxIndex[0U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[0U];
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex[0U];
    frac = 0.0;
  }

  fractions[0U] = frac;
  bpIndices[0U] = iLeft;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u1 <= bp1[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u1 < bp1[maxIndex[1U]]) {
    /* Binary Search */
    bpIdx = maxIndex[1U] >> 1U;
    iLeft = 0U;
    iRght = maxIndex[1U];
    while (iRght - iLeft > 1U) {
      if (u1 < bp1[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u1 - bp1[iLeft]) / (bp1[iLeft + 1U] - bp1[iLeft]);
  } else {
    iLeft = maxIndex[1U];
    frac = 0.0;
  }

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  bpIdx = iLeft * stride + bpIndices[0U];
  if (bpIndices[0U] == maxIndex[0U]) {
    y = table[bpIdx];
  } else {
    y = (table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx];
  }

  if (iLeft == maxIndex[1U]) {
  } else {
    bpIdx += stride;
    if (bpIndices[0U] == maxIndex[0U]) {
      yR_1d = table[bpIdx];
    } else {
      yR_1d = (table[bpIdx + 1U] - table[bpIdx]) * fractions[0U] + table[bpIdx];
    }

    y += (yR_1d - y) * frac;
  }

  return y;
}

/* State reduction function */
void local_stateReduction(real_T* x, int_T* p, int_T n, real_T* r)
{
  int_T i, j;
  for (i = 0, j = 0; i < n; ++i, ++j) {
    int_T k = p[i];
    real_T lb = r[j++];
    real_T xk = x[k]-lb;
    real_T rk = r[j]-lb;
    int_T q = (int_T) floor(xk/rk);
    if (q) {
      x[k] = xk-q*rk+lb;
    }
  }
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 93;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  CAVE_MachE_sil_test_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  CAVE_MachE_sil_test_output();
  CAVE_MachE_sil_test_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  CAVE_MachE_sil_test_output();
  CAVE_MachE_sil_test_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  CAVE_MachE_sil_test_output();
  CAVE_MachE_sil_test_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  local_stateReduction(x, rtsiGetPeriodicContStateIndices(si), 3,
                       rtsiGetPeriodicContStateRanges(si));
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * Output and update for atomic system:
 *    '<S80>/MATLAB Function'
 *    '<S81>/MATLAB Function'
 *    '<S82>/MATLAB Function'
 *    '<S83>/MATLAB Function'
 */
void CAVE_MachE_sil_test_MATLABFunction(real_T rtu_axleTorq, real_T rtu_wheelSpd,
  B_MATLABFunction_CAVE_MachE_sil_test_T *localB)
{
  /* MATLAB Function 'Vehicle Under Test/Vehicle/Rototest and ORNL Interface/Get Wheel Spd Cmd Based on Wheel Dynamics/Whl Dynamics  & Format CM Powertrain Signal/Subsystem/MATLAB Function': '<S84>:1' */
  if ((fabs(rtu_axleTorq) < 1.0E-5) && (fabs(rtu_wheelSpd) < 1.0E-5)) {
    /* '<S84>:1:4' */
    /* '<S84>:1:5' */
    localB->switchFlag = 1.0;
  } else {
    /* '<S84>:1:7' */
    localB->switchFlag = 0.0;
  }
}

/*
 * Output and update for atomic system:
 *    '<S107>/Defloater'
 *    '<S108>/Defloater'
 *    '<S109>/Defloater'
 *    '<S110>/Defloater'
 */
void CAVE_MachE_sil_test_Defloater(real_T rtu_defloatMe,
  B_Defloater_CAVE_MachE_sil_test_T *localB)
{
  uint8_T float32[4];
  real32_T x;

  /* MATLAB Function 'Defloater/Defloater': '<S114>:1' */
  /* '<S114>:1:8' */
  x = (real32_T)rtu_defloatMe;
  memcpy((void *)&float32[0], (void *)&x, (uint32_T)((size_t)4 * sizeof(uint8_T)));

  /* '<S114>:1:10' */
  /* '<S114>:1:11' */
  /* '<S114>:1:12' */
  /* '<S114>:1:13' */
  localB->byte1 = float32[0];
  localB->byte2 = float32[1];
  localB->byte3 = float32[2];
  localB->byte4 = float32[3];
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  real_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/*
 * Start for atomic system:
 *    '<S172>/Min stop reached'
 *    '<S206>/Min stop reached'
 */
void CAVE_MachE_sil_test_Minstopreached_Start
  (B_Minstopreached_CAVE_MachE_sil_test_T *localB)
{
  localB->Sum1 = 0.0;
  localB->Gain5 = 0.0;
  localB->Gain4 = 0.0;
  localB->Product3 = 0.0;
  localB->Abs1 = 0.0;
  localB->Saturation = 0.0;
  localB->TrigonometricFunction = 0.0;
  localB->Gain = 0.0;
  localB->Sum2 = 0.0;
  localB->LowerHardStopBlendMult = 0.0;
  localB->MathFunction = 0.0;
  localB->Product2 = 0.0;
  localB->Product = 0.0;
  localB->Product1 = 0.0;
  localB->Sum = 0.0;
  localB->Product4 = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S172>/Min stop reached'
 *    '<S206>/Min stop reached'
 */
void CAVE_MachE_sil_test_Minstopreached(RT_MODEL_CAVE_MachE_sil_test_T * const
  CAVE_MachE_sil_test_M, real_T rtu_x, real_T rtu_xdot, real_T rtu_k, real_T
  rtu_c, real_T rtu_xmax, B_Minstopreached_CAVE_MachE_sil_test_T *localB,
  P_Minstopreached_CAVE_MachE_sil_test_T *localP, real_T rtp_Hmax)
{
  real_T u0;
  real_T u1;
  real_T u2;

  /* Sum: '<S178>/Sum1' */
  localB->Sum1 = rtu_x + rtu_xmax;

  /* Gain: '<S178>/Gain5' */
  localB->Gain5 = localP->Gain5_Gain * localB->Sum1;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S178>/Gain4' */
    localB->Gain4 = localP->Gain4_Gain * rtu_xmax;
  }

  /* Product: '<S178>/Product3' */
  localB->Product3 = localB->Gain5 / localB->Gain4;

  /* Abs: '<S178>/Abs1' */
  localB->Abs1 = fabs(localB->Product3);

  /* Saturate: '<S178>/Saturation' */
  u0 = localB->Abs1;
  u1 = localP->Saturation_LowerSat;
  u2 = localP->Saturation_UpperSat;
  if (u0 > u2) {
    localB->Saturation = u2;
  } else if (u0 < u1) {
    localB->Saturation = u1;
  } else {
    localB->Saturation = u0;
  }

  /* End of Saturate: '<S178>/Saturation' */

  /* Trigonometry: '<S178>/Trigonometric Function' */
  localB->TrigonometricFunction = tanh(localB->Saturation);

  /* Gain: '<S178>/Gain' */
  localB->Gain = localP->Gain_Gain * localB->TrigonometricFunction;

  /* Sum: '<S178>/Sum2' incorporates:
   *  Constant: '<S178>/Constant1'
   */
  localB->Sum2 = rtu_x + rtp_Hmax;

  /* Lookup_n-D: '<S178>/Lower Hard Stop Blend Mult' */
  localB->LowerHardStopBlendMult = look1_binlcpw(localB->Sum2,
    localP->LowerHardStopBlendMult_bp01Data,
    localP->LowerHardStopBlendMult_tableData, 2U);

  /* Math: '<S178>/Math Function' incorporates:
   *  Constant: '<S178>/Constant'
   */
  u0 = localB->Saturation;
  u1 = localP->Constant_Value;
  if ((u0 < 0.0) && (u1 > floor(u1))) {
    localB->MathFunction = -rt_powd_snf(-u0, u1);
  } else {
    localB->MathFunction = rt_powd_snf(u0, u1);
  }

  /* End of Math: '<S178>/Math Function' */

  /* Product: '<S178>/Product2' */
  localB->Product2 = localB->TrigonometricFunction * localB->MathFunction;

  /* Product: '<S178>/Product' */
  localB->Product = localB->Sum1 * rtu_k * localB->Product2;

  /* Product: '<S178>/Product1' */
  localB->Product1 = localB->Gain * rtu_c * rtu_xdot;

  /* Sum: '<S178>/Sum' */
  localB->Sum = localB->Product + localB->Product1;

  /* Product: '<S178>/Product4' */
  localB->Product4 = localB->Sum * localB->LowerHardStopBlendMult;
}

/*
 * Start for atomic system:
 *    '<S172>/Max stop reached'
 *    '<S206>/Max stop reached'
 */
void CAVE_MachE_sil_test_Maxstopreached_Start
  (B_Maxstopreached_CAVE_MachE_sil_test_T *localB)
{
  localB->Sum1 = 0.0;
  localB->Gain5 = 0.0;
  localB->Gain4 = 0.0;
  localB->Product3 = 0.0;
  localB->Abs1 = 0.0;
  localB->Saturation = 0.0;
  localB->TrigonometricFunction = 0.0;
  localB->Gain = 0.0;
  localB->MathFunction = 0.0;
  localB->Product2 = 0.0;
  localB->Product = 0.0;
  localB->Product1 = 0.0;
  localB->Sum = 0.0;
  localB->Sum2 = 0.0;
  localB->UpperHardStopBlendMult = 0.0;
  localB->Product4 = 0.0;
}

/*
 * Output and update for atomic system:
 *    '<S172>/Max stop reached'
 *    '<S206>/Max stop reached'
 */
void CAVE_MachE_sil_test_Maxstopreached(RT_MODEL_CAVE_MachE_sil_test_T * const
  CAVE_MachE_sil_test_M, real_T rtu_x, real_T rtu_xdot, real_T rtu_k, real_T
  rtu_c, real_T rtu_xmax, B_Maxstopreached_CAVE_MachE_sil_test_T *localB,
  P_Maxstopreached_CAVE_MachE_sil_test_T *localP, real_T rtp_Hmax)
{
  real_T u0;
  real_T u1;
  real_T u2;

  /* Sum: '<S177>/Sum1' */
  localB->Sum1 = rtu_x - rtu_xmax;

  /* Gain: '<S177>/Gain5' */
  localB->Gain5 = localP->Gain5_Gain * localB->Sum1;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S177>/Gain4' */
    localB->Gain4 = localP->Gain4_Gain * rtu_xmax;
  }

  /* Product: '<S177>/Product3' */
  localB->Product3 = localB->Gain5 / localB->Gain4;

  /* Abs: '<S177>/Abs1' */
  localB->Abs1 = fabs(localB->Product3);

  /* Saturate: '<S177>/Saturation' */
  u0 = localB->Abs1;
  u1 = localP->Saturation_LowerSat;
  u2 = localP->Saturation_UpperSat;
  if (u0 > u2) {
    localB->Saturation = u2;
  } else if (u0 < u1) {
    localB->Saturation = u1;
  } else {
    localB->Saturation = u0;
  }

  /* End of Saturate: '<S177>/Saturation' */

  /* Trigonometry: '<S177>/Trigonometric Function' */
  localB->TrigonometricFunction = tanh(localB->Saturation);

  /* Gain: '<S177>/Gain' */
  localB->Gain = localP->Gain_Gain * localB->TrigonometricFunction;

  /* Math: '<S177>/Math Function' incorporates:
   *  Constant: '<S177>/Constant'
   */
  u0 = localB->Saturation;
  u1 = localP->Constant_Value;
  if ((u0 < 0.0) && (u1 > floor(u1))) {
    localB->MathFunction = -rt_powd_snf(-u0, u1);
  } else {
    localB->MathFunction = rt_powd_snf(u0, u1);
  }

  /* End of Math: '<S177>/Math Function' */

  /* Product: '<S177>/Product2' */
  localB->Product2 = localB->TrigonometricFunction * localB->MathFunction;

  /* Product: '<S177>/Product' */
  localB->Product = localB->Sum1 * rtu_k * localB->Product2;

  /* Product: '<S177>/Product1' */
  localB->Product1 = localB->Gain * rtu_c * rtu_xdot;

  /* Sum: '<S177>/Sum' */
  localB->Sum = localB->Product + localB->Product1;

  /* Sum: '<S177>/Sum2' incorporates:
   *  Constant: '<S177>/Constant1'
   */
  localB->Sum2 = rtu_x - rtp_Hmax;

  /* Lookup_n-D: '<S177>/Upper Hard Stop Blend Mult' */
  localB->UpperHardStopBlendMult = look1_binlcpw(localB->Sum2,
    localP->UpperHardStopBlendMult_bp01Data,
    localP->UpperHardStopBlendMult_tableData, 2U);

  /* Product: '<S177>/Product4' */
  localB->Product4 = localB->Sum * localB->UpperHardStopBlendMult;
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_power(const real_T a_data[], const int32_T
  a_size[2], real_T y_data[], int32_T y_size[2])
{
  real_T z1_data;
  int32_T loop_ub;
  int32_T z1_size_idx_1;
  y_size[1] = (int8_T)a_size[1];
  z1_size_idx_1 = y_size[1];
  loop_ub = y_size[1] - 1;
  if (0 <= loop_ub) {
    memcpy(&z1_data, &y_data[0], (loop_ub + 1) * sizeof(real_T));
  }

  if (0 <= y_size[1] - 1) {
    z1_data = a_data[0] * a_data[0];
  }

  y_size[0] = 1;
  y_size[1] = z1_size_idx_1;
  loop_ub = z1_size_idx_1 - 1;
  if (0 <= loop_ub) {
    memcpy(&y_data[0], &z1_data, (loop_ub + 1) * sizeof(real_T));
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_div0protect(real_T u, real_T tol, real_T *y,
  real_T *yabs)
{
  int32_T trueCount;
  real_T d_data;
  real_T yabs_data;
  real_T tmp_data;
  int32_T yabs_size[2];
  int32_T tmp_size[2];
  *yabs = fabs(u);
  trueCount = 0;
  if (*yabs < tol) {
    trueCount = 1;
  }

  yabs_size[0] = 1;
  yabs_size[1] = trueCount;
  if (0 <= trueCount - 1) {
    yabs_data = *yabs / tol;
  }

  CAVE_MachE_sil_test_power(&yabs_data, yabs_size, &tmp_data, tmp_size);
  if (0 <= trueCount - 1) {
    d_data = 2.0 * tol / (3.0 - tmp_data);
  }

  if (*yabs < tol) {
    *yabs = d_data;
  }

  trueCount = 0;
  if (u < 0.0) {
    trueCount = 1;
  }

  trueCount--;
  if (0 <= trueCount) {
    d_data = -*yabs;
  }

  *y = *yabs;
  if (u < 0.0) {
    *y = d_data;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static boolean_T CAVE_MachE_sil_test_any(boolean_T x)
{
  boolean_T b;
  b = !x;
  return !b;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp_0, tmp);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static real_T CAVE_MachE_sil_test_div0protect_p(real_T u)
{
  real_T y;
  int32_T trueCount;
  real_T d_data;
  real_T yabs_data;
  real_T tmp_data;
  int32_T yabs_size[2];
  int32_T tmp_size[2];
  y = fabs(u);
  trueCount = 0;
  if (y < 0.0001) {
    trueCount = 1;
  }

  yabs_size[0] = 1;
  yabs_size[1] = trueCount;
  if (0 <= trueCount - 1) {
    yabs_data = y / 0.0001;
  }

  CAVE_MachE_sil_test_power(&yabs_data, yabs_size, &tmp_data, tmp_size);
  if (0 <= trueCount - 1) {
    d_data = 0.0002 / (3.0 - tmp_data);
  }

  if (y < 0.0001) {
    y = d_data;
  }

  trueCount = 0;
  if (u < 0.0) {
    trueCount = 1;
  }

  trueCount--;
  if (0 <= trueCount) {
    d_data = -y;
  }

  if (u < 0.0) {
    y = d_data;
  }

  return y;
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_sin(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = sin(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_abs(const real_T x_data[], const int32_T x_size
  [2], real_T y_data[], int32_T y_size[2])
{
  y_size[0] = 1;
  y_size[1] = (int8_T)x_size[1];
  if (0 <= x_size[1] - 1) {
    y_data[0] = fabs(x_data[0]);
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_atan(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = atan(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_cos(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = cos(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_sqrt(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = sqrt(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_div0protect_pe(real_T u, real_T *y, real_T *yabs)
{
  int32_T trueCount;
  real_T d_data;
  real_T yabs_data;
  real_T tmp_data;
  int32_T yabs_size[2];
  int32_T tmp_size[2];
  *yabs = fabs(u);
  trueCount = 0;
  if (*yabs < 0.0001) {
    trueCount = 1;
  }

  yabs_size[0] = 1;
  yabs_size[1] = trueCount;
  if (0 <= trueCount - 1) {
    yabs_data = *yabs / 0.0001;
  }

  CAVE_MachE_sil_test_power(&yabs_data, yabs_size, &tmp_data, tmp_size);
  if (0 <= trueCount - 1) {
    d_data = 0.0002 / (3.0 - tmp_data);
  }

  if (*yabs < 0.0001) {
    *yabs = d_data;
  }

  trueCount = 0;
  if (u < 0.0) {
    trueCount = 1;
  }

  trueCount--;
  if (0 <= trueCount) {
    d_data = -*yabs;
  }

  *y = *yabs;
  if (u < 0.0) {
    *y = d_data;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_acos(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = acos(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static void CAVE_MachE_sil_test_tanh(real_T x_data[], const int32_T x_size[2])
{
  int32_T nx;
  int32_T k;
  nx = x_size[1];
  k = 0;
  while (k <= nx - 1) {
    x_data[0] = tanh(x_data[0]);
    k = 1;
  }
}

/* Function for MATLAB Function: '<S344>/Magic Tire Const Input' */
static real_T CAVE_MachE_sil_test_rollingMoment(real_T Fx, real_T Vcx, real_T Fz,
  real_T press, real_T b_gamma, real_T Vo, real_T Ro, real_T Fzo, real_T pio,
  real_T b_QSY1, real_T b_QSY2, real_T b_QSY3, real_T b_QSY4, real_T b_QSY5,
  real_T b_QSY6, real_T b_QSY7, real_T b_QSY8, real_T lam_My)
{
  real_T My;
  My = ((((Fx / Fzo * b_QSY2 + b_QSY1) + fabs(Vcx / Vo) * b_QSY3) + rt_powd_snf
         (Vcx / Vo, 4.0) * b_QSY4) + (Fz / Fzo * b_QSY6 + b_QSY5) * (b_gamma *
         b_gamma)) * (-Ro * Fzo * lam_My) * (rt_powd_snf(Fz / Fzo, b_QSY7) *
    rt_powd_snf(press / pio, b_QSY8));
  return My;
}

/*
 * Output and update for atomic system:
 *    '<S344>/Magic Tire Const Input'
 *    '<S369>/Magic Tire Const Input'
 *    '<S394>/Magic Tire Const Input'
 *    '<S419>/Magic Tire Const Input'
 */
void CAVE_MachE_sil_test_MagicTireConstInput(real_T rtu_Omega, real_T rtu_Vx,
  real_T rtu_Vy, real_T rtu_psidot, real_T rtu_Gamma, real_T rtu_TirePrs, const
  real_T rtu_ScaleFactors[27], real_T rtu_rhoz, real_T rtu_Fx_ext, real_T
  rtu_Fy_ext, real_T *rty_Mx, real_T *rty_Mz, real_T *rty_Kappa, real_T
  *rty_Alpha, B_MagicTireConstInput_CAVE_MachE_sil_test_T *localB, real_T
  rtp_ALPMAX, real_T rtp_ALPMIN, real_T rtp_BREFF, real_T rtp_CAMMAX, real_T
  rtp_CAMMIN, real_T rtp_DREFF, real_T rtp_FNOMIN, real_T rtp_FREFF, real_T
  rtp_FZMAX, real_T rtp_FZMIN, real_T rtp_KPUMAX, real_T rtp_KPUMIN, real_T
  rtp_LATERAL_STIFFNESS, real_T rtp_LONGITUDINAL_STIFFNESS, real_T rtp_LONGVL,
  real_T rtp_NOMPRES, real_T rtp_PCFX1, real_T rtp_PCFX2, real_T rtp_PCFX3,
  real_T rtp_PCFY1, real_T rtp_PCFY2, real_T rtp_PCFY3, real_T rtp_PCX1, real_T
  rtp_PCY1, real_T rtp_PDX1, real_T rtp_PDX2, real_T rtp_PDX3, real_T rtp_PDXP1,
  real_T rtp_PDXP2, real_T rtp_PDXP3, real_T rtp_PDY1, real_T rtp_PDY2, real_T
  rtp_PDY3, real_T rtp_PDYP1, real_T rtp_PDYP2, real_T rtp_PDYP3, real_T
  rtp_PDYP4, real_T rtp_PECP1, real_T rtp_PECP2, real_T rtp_PEX1, real_T
  rtp_PEX2, real_T rtp_PEX3, real_T rtp_PEX4, real_T rtp_PEY1, real_T rtp_PEY2,
  real_T rtp_PEY4, real_T rtp_PEY5, real_T rtp_PFZ1, real_T rtp_PHX1, real_T
  rtp_PHX2, real_T rtp_PHYP1, real_T rtp_PHYP2, real_T rtp_PHYP3, real_T
  rtp_PHYP4, real_T rtp_PKX1, real_T rtp_PKX2, real_T rtp_PKX3, real_T rtp_PKY1,
  real_T rtp_PKY2, real_T rtp_PKY3, real_T rtp_PKY4, real_T rtp_PKY5, real_T
  rtp_PKY6, real_T rtp_PKY7, real_T rtp_PKYP1, real_T rtp_PPMX1, real_T rtp_PPX1,
  real_T rtp_PPX2, real_T rtp_PPX3, real_T rtp_PPX4, real_T rtp_PPY1, real_T
  rtp_PPY2, real_T rtp_PPY3, real_T rtp_PPY4, real_T rtp_PPY5, real_T rtp_PPZ1,
  real_T rtp_PPZ2, real_T rtp_PRESMAX, real_T rtp_PRESMIN, real_T rtp_PVX1,
  real_T rtp_PVX2, real_T rtp_PVY3, real_T rtp_PVY4, real_T rtp_QBRP1, real_T
  rtp_QBZ1, real_T rtp_QBZ10, real_T rtp_QBZ2, real_T rtp_QBZ3, real_T rtp_QBZ5,
  real_T rtp_QBZ6, real_T rtp_QBZ9, real_T rtp_QCRP1, real_T rtp_QCRP2, real_T
  rtp_QCZ1, real_T rtp_QDRP1, real_T rtp_QDRP2, real_T rtp_QDTP1, real_T
  rtp_QDZ1, real_T rtp_QDZ10, real_T rtp_QDZ11, real_T rtp_QDZ2, real_T rtp_QDZ4,
  real_T rtp_QDZ8, real_T rtp_QDZ9, real_T rtp_QEZ1, real_T rtp_QEZ2, real_T
  rtp_QEZ3, real_T rtp_QEZ5, real_T rtp_QHZ3, real_T rtp_QHZ4, real_T rtp_QSX10,
  real_T rtp_QSX11, real_T rtp_QSX12, real_T rtp_QSX13, real_T rtp_QSX14, real_T
  rtp_QSX2, real_T rtp_QSX3, real_T rtp_QSX4, real_T rtp_QSX5, real_T rtp_QSX6,
  real_T rtp_QSX7, real_T rtp_QSX8, real_T rtp_QSX9, real_T rtp_QSY1, real_T
  rtp_QSY2, real_T rtp_QSY3, real_T rtp_QSY4, real_T rtp_QSY5, real_T rtp_QSY6,
  real_T rtp_QSY7, real_T rtp_QSY8, real_T rtp_Q_FCX, real_T rtp_Q_FCY, real_T
  rtp_Q_FCY2, real_T rtp_Q_FZ1, real_T rtp_Q_FZ2, real_T rtp_Q_FZ3, real_T
  rtp_Q_RA1, real_T rtp_Q_RA2, real_T rtp_Q_RB1, real_T rtp_Q_RB2, real_T
  rtp_Q_RE0, real_T rtp_Q_V1, real_T rtp_Q_V2, real_T rtp_RBX1, real_T rtp_RBX2,
  real_T rtp_RBX3, real_T rtp_RBY1, real_T rtp_RBY2, real_T rtp_RBY4, real_T
  rtp_RCX1, real_T rtp_RCY1, real_T rtp_REX1, real_T rtp_REX2, real_T rtp_REY1,
  real_T rtp_REY2, real_T rtp_RHY1, real_T rtp_RHY2, real_T rtp_RVY3, real_T
  rtp_RVY4, real_T rtp_RVY5, real_T rtp_RVY6, real_T rtp_SSZ2, real_T rtp_SSZ3,
  real_T rtp_SSZ4, real_T rtp_UNLOADED_RADIUS, real_T rtp_VERTICAL_STIFFNESS,
  real_T rtp_VXLOW, real_T rtp_WIDTH)
{
  real_T scaleFactors[27];
  real_T b_PRESMIN;
  real_T b_FZMIN;
  real_T b_VXLOW;
  real_T b_CAMMIN;
  real_T b_CAMMAX;
  real_T b_BREFF;
  real_T b_FREFF;
  real_T b_Q_V2;
  real_T b_Q_FZ1;
  real_T b_Q_FCX;
  real_T b_Q_FCY;
  real_T b_PKX3;
  real_T b_PHX2;
  real_T b_PVX1;
  real_T b_PKY2;
  real_T b_PKY3;
  real_T b_PKY7;
  real_T b_RVY4;
  real_T b_QEZ1;
  real_T b_QEZ2;
  real_T b_PKYP1;
  real_T b_PDYP4;
  real_T b_PHYP1;
  real_T b_PECP2;
  real_T b_QDRP2;
  real_T lam_mux;
  real_T lam_muy;
  real_T lam_Kyalpha;
  real_T lam_Hy;
  real_T lam_Kzgamma;
  real_T lam_Mx;
  real_T lam_VMx;
  real_T lam_My;
  real_T dpi;
  real_T Cz;
  real_T dfz;
  real_T epsilon_gamma;
  real_T kappa_x;
  real_T gamma_star;
  real_T Vsx;
  real_T Vsy;
  real_T Vs;
  real_T Vc;
  real_T Vc_prime;
  boolean_T isLowSpeed;
  real_T reduction_factor_data;
  real_T mu_y;
  real_T phi_t;
  boolean_T turnslipinds;
  real_T SHykappa;
  real_T Eykappa;
  real_T Bykappa;
  real_T zeta[9];
  real_T Bxphi;
  real_T Byphi;
  real_T Kygammao;
  real_T Kyalpha;
  real_T DHyphi;
  real_T BHyphi;
  real_T SVygamma;
  real_T Mzphiinf;
  real_T Gykappa;
  real_T Mzphi90;
  real_T DDrphi;
  real_T tmpDrphiVar_data;
  real_T Cx;
  real_T Dx;
  real_T Ex;
  real_T Kxkappa;
  real_T Bx;
  real_T Exalpha;
  real_T Bxalpha;
  real_T Gxalpha;
  real_T Cy;
  real_T Dy;
  real_T Ey;
  real_T SVykappa;
  real_T cosprimealpha;
  real_T alpha_t;
  real_T alpha_r;
  real_T Bt;
  real_T Et;
  real_T Br;
  real_T alpha_req;
  real_T Mzr;
  real_T t;
  real_T unusedU0;
  int32_T b_i;
  int32_T o_trueCount;
  real_T fc_data;
  int32_T i;
  real_T tmp_data;
  real_T rtu_Vx_data;
  real_T reduction_factor_data_0;
  int32_T loop_ub;
  int32_T reduction_factor_size[2];
  int32_T tmpDrphiVar_size[2];
  int32_T fc_size[2];
  int32_T lam_Kzgamma_size[2];
  int32_T b_UNLOADED_RADIUS_size[2];
  int32_T phi_t_size[2];
  int32_T phi_t_size_0[2];
  int32_T phi_t_size_1[2];
  int32_T rtu_Vx_size[2];
  int32_T tmp_size[2];

  /* MATLAB Function 'Magic Tire Const Input/Magic Tire Const Input': '<S349>:1' */
  /* '<S349>:1:193' */
  memcpy(&scaleFactors[0], &rtu_ScaleFactors[0], 27U * sizeof(real_T));
  dpi = rtp_PRESMAX;
  b_PRESMIN = rtp_PRESMIN;
  dfz = rtp_FZMAX;
  b_FZMIN = rtp_FZMIN;
  b_VXLOW = rtp_VXLOW;
  kappa_x = rtp_KPUMAX;
  Vs = rtp_KPUMIN;
  gamma_star = rtp_ALPMAX;
  Vsx = rtp_ALPMIN;
  b_CAMMIN = rtp_CAMMIN;
  b_CAMMAX = rtp_CAMMAX;
  Cz = rtp_VERTICAL_STIFFNESS;
  Vc_prime = rtp_DREFF;
  b_BREFF = rtp_BREFF;
  b_FREFF = rtp_FREFF;
  b_Q_V2 = rtp_Q_V2;
  b_Q_FZ1 = rtp_Q_FZ1;
  b_Q_FCX = rtp_Q_FCX;
  b_Q_FCY = rtp_Q_FCY;
  Cx = rtp_PCX1;
  Dx = rtp_PDX1;
  Ex = rtp_PEX1;
  Kxkappa = rtp_PKX1;
  Bx = rtp_PKX2;
  b_PKX3 = rtp_PKX3;
  Vc = rtp_PHX1;
  b_PHX2 = rtp_PHX2;
  b_PVX1 = rtp_PVX1;
  Dy = rtp_PVX2;
  Bxalpha = rtp_RBX1;
  Gxalpha = rtp_RCX1;
  Exalpha = rtp_REX1;
  cosprimealpha = rtp_QSX6;
  Cy = rtp_PCY1;
  mu_y = rtp_PDY1;
  phi_t = rtp_PDY2;
  Ey = rtp_PEY1;
  DHyphi = rtp_PKY1;
  b_PKY2 = rtp_PKY2;
  b_PKY3 = rtp_PKY3;
  Kyalpha = rtp_PKY6;
  b_PKY7 = rtp_PKY7;
  SVygamma = rtp_PVY3;
  Kygammao = rtp_PPY5;
  Bykappa = rtp_RBY1;
  Gykappa = rtp_RCY1;
  Eykappa = rtp_REY1;
  SHykappa = rtp_RHY1;
  SVykappa = rtp_RVY3;
  b_RVY4 = rtp_RVY4;
  Bt = rtp_QBZ1;
  Br = rtp_QBZ9;
  alpha_req = rtp_QBZ10;
  Et = rtp_QCZ1;
  t = rtp_QDZ1;
  Mzr = rtp_QDZ8;
  b_QEZ1 = rtp_QEZ1;
  b_QEZ2 = rtp_QEZ2;
  alpha_t = rtp_QHZ3;
  alpha_r = rtp_QHZ4;
  Bxphi = rtp_PDXP1;
  b_PKYP1 = rtp_PKYP1;
  Byphi = rtp_PDYP1;
  b_PDYP4 = rtp_PDYP4;
  b_PHYP1 = rtp_PHYP1;
  BHyphi = rtp_PHYP4;
  epsilon_gamma = rtp_PECP1;
  b_PECP2 = rtp_PECP2;
  Mzphiinf = rtp_QDTP1;
  Mzphi90 = rtp_QCRP2;
  DDrphi = rtp_QDRP1;
  b_QDRP2 = rtp_QDRP2;
  lam_mux = rtu_ScaleFactors[1];
  lam_muy = rtu_ScaleFactors[2];
  lam_Kyalpha = rtu_ScaleFactors[5];
  lam_Hy = rtu_ScaleFactors[11];
  lam_Kzgamma = rtu_ScaleFactors[15];
  lam_Mx = rtu_ScaleFactors[23];
  lam_VMx = rtu_ScaleFactors[24];
  lam_My = rtu_ScaleFactors[25];
  CAVE_MachE_sil_test_div0protect(rtu_Vx, rtp_VXLOW, &unusedU0, &Vsy);
  unusedU0 = rtu_Gamma;
  if (rtu_Gamma < b_CAMMIN) {
    unusedU0 = b_CAMMIN;
  }

  if (unusedU0 > b_CAMMAX) {
    unusedU0 = b_CAMMAX;
  }

  b_CAMMIN = rtu_TirePrs;
  if (rtu_TirePrs < b_PRESMIN) {
    b_CAMMIN = b_PRESMIN;
  }

  if (b_CAMMIN > dpi) {
    b_CAMMIN = dpi;
  }

  dpi = (b_CAMMIN - rtp_NOMPRES) / rtp_NOMPRES;
  b_PRESMIN = scaleFactors[1];
  if (lam_mux <= 0.0) {
    b_PRESMIN = 2.2204460492503131E-16;
  }

  b_CAMMAX = scaleFactors[2];
  if (lam_muy <= 0.0) {
    b_CAMMAX = 2.2204460492503131E-16;
  }

  if (CAVE_MachE_sil_test_any(b_Q_FZ1 == 0.0)) {
    lam_mux = Cz * rtp_UNLOADED_RADIUS / rtp_FNOMIN;
    b_Q_FZ1 = sqrt(lam_mux * lam_mux - 4.0 * rtp_Q_FZ2);
  }

  lam_mux = scaleFactors[0] * rtp_FNOMIN;
  b_Q_FCX = b_Q_FCX * rtu_Fx_ext / rtp_FNOMIN;
  b_Q_FCY = b_Q_FCY * rtu_Fy_ext / rtp_FNOMIN;
  lam_muy = rtu_rhoz / rtp_UNLOADED_RADIUS;
  b_Q_V2 = (((b_Q_V2 * fabs(rtu_Omega) * rtp_UNLOADED_RADIUS / rtp_LONGVL + 1.0)
             - b_Q_FCX * b_Q_FCX) - b_Q_FCY * b_Q_FCY) * ((unusedU0 * unusedU0 *
    rtp_Q_FZ3 + b_Q_FZ1) * rtu_rhoz / rtp_UNLOADED_RADIUS + lam_muy * lam_muy *
    rtp_Q_FZ2) * (rtp_PFZ1 * dpi + 1.0) * lam_mux;
  if (rtu_rhoz == 0.0) {
    Cz *= scaleFactors[22];
  } else {
    Cz = (rtp_Q_FCY2 * rtu_Fy_ext / rtp_FNOMIN + b_Q_V2 / fabs(rtu_rhoz)) *
      scaleFactors[22];
  }

  b_Q_FZ1 = b_Q_V2;
  if (b_Q_V2 < b_FZMIN) {
    b_Q_FZ1 = b_FZMIN;
  }

  if (b_Q_FZ1 > dfz) {
    b_Q_FZ1 = dfz;
  }

  dfz = (b_Q_FZ1 - lam_mux) / lam_mux;
  epsilon_gamma *= b_PECP2 * dfz + 1.0;
  b_PECP2 = rtu_Omega * rtp_UNLOADED_RADIUS / rtp_LONGVL;
  Vc_prime = (b_PECP2 * b_PECP2 * rtp_Q_V1 + rtp_Q_RE0) * rtp_UNLOADED_RADIUS -
    (atan(b_Q_V2 / rtp_FNOMIN * b_BREFF) * Vc_prime + b_Q_V2 / rtp_FNOMIN *
     b_FREFF) * (rtp_FNOMIN / Cz);
  b_BREFF = Vc_prime;
  if (Vc_prime < 0.001) {
    b_BREFF = 0.001;
  }

  Vc_prime = (b_BREFF * rtu_Omega - rtu_Vx) / Vsy;
  if (Vc_prime < Vs) {
    Vc_prime = Vs;
  }

  b_FREFF = Vc_prime;
  if (Vc_prime > kappa_x) {
    b_FREFF = kappa_x;
  }

  kappa_x = (b_PHX2 * dfz + Vc) * scaleFactors[10] + b_FREFF;
  Vs = rt_atan2d_snf(rtu_Vy, Vsy);
  if (Vs < Vsx) {
    Vs = Vsx;
  }

  b_FZMIN = Vs;
  if (Vs > gamma_star) {
    b_FZMIN = gamma_star;
  }

  gamma_star = sin(unusedU0);
  Vsx = -Vsy * b_FREFF;
  Vsy = -Vsy * tan(b_FZMIN);
  Vs = sqrt(Vsx * Vsx + Vsy * Vsy);
  CAVE_MachE_sil_test_div0protect(rtu_Vx, b_VXLOW, &b_PHX2, &Vsx);
  Vc = sqrt(rtu_Vx * rtu_Vx + Vsy * Vsy);
  CAVE_MachE_sil_test_div0protect(Vc, b_VXLOW, &Vsx, &Vc_prime);
  Vc_prime = CAVE_MachE_sil_test_div0protect_p(Vc);
  Vc = b_BREFF * rtu_Omega - rtu_Vx;
  isLowSpeed = (sqrt(Vc * Vc + Vsy * Vsy) < b_VXLOW);
  i = 0;
  if (isLowSpeed) {
    i = 1;
  }

  reduction_factor_size[0] = 1;
  reduction_factor_size[1] = i;
  loop_ub = i - 1;
  if (0 <= loop_ub) {
    reduction_factor_data = Vc / b_VXLOW * 3.1415926535897931 / 2.0;
  }

  CAVE_MachE_sil_test_sin(&reduction_factor_data, reduction_factor_size);
  i = -1;
  if (isLowSpeed) {
    i = 0;
  }

  loop_ub = 0;
  b_i = 0;
  while (b_i <= i) {
    if (Vc < 0.0) {
      loop_ub++;
    }

    b_i = 1;
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    reduction_factor_data_0 = -reduction_factor_data;
  }

  for (b_i = 0; b_i < loop_ub; b_i++) {
    reduction_factor_data = reduction_factor_data_0;
  }

  Vc = b_PRESMIN / (scaleFactors[3] * Vs / rtp_LONGVL + 1.0);
  Vsy = b_CAMMAX / (scaleFactors[3] * Vs / rtp_LONGVL + 1.0);
  Vs = Vsy / (0.0 * Vsy + 1.0);
  mu_y = ((rtp_PPY3 * dpi + 1.0) + dpi * dpi * rtp_PPY4) * (phi_t * dfz + mu_y) *
    (1.0 - gamma_star * gamma_star * rtp_PDY3) * Vsy;
  phi_t = -rtu_psidot / b_PHX2 * cos(b_FZMIN);
  i = 0;
  loop_ub = 0;
  if (isLowSpeed) {
    i = 1;
    loop_ub = 1;
  }

  rtu_Vx_size[0] = 1;
  rtu_Vx_size[1] = loop_ub;
  if (0 <= loop_ub - 1) {
    rtu_Vx_data = rtu_Vx;
  }

  CAVE_MachE_sil_test_abs(&rtu_Vx_data, rtu_Vx_size, &reduction_factor_data_0,
    tmp_size);
  if (0 <= i - 1) {
    tmpDrphiVar_data = reduction_factor_data_0 / b_VXLOW * phi_t;
  }

  b_PHX2 = phi_t;
  if (isLowSpeed) {
    b_PHX2 = tmpDrphiVar_data;
  }

  phi_t = (rtu_psidot - (1.0 - epsilon_gamma) * fabs(rtu_Omega) * sin(unusedU0))
    * -(1.0 / Vc_prime);
  turnslipinds = (fabs(phi_t) > 0.01);
  SHykappa += rtp_RHY2 * dfz;
  Eykappa += rtp_REY2 * dfz;
  if (Eykappa > 1.0) {
    Eykappa = 1.0;
  }

  Bykappa = (gamma_star * gamma_star * rtp_RBY4 + Bykappa) * cos(atan(rtp_RBY2 *
    b_FZMIN)) * scaleFactors[19];
  Vc_prime = Bykappa;
  if (Bykappa < 0.0) {
    Vc_prime = 0.0;
  }

  Bykappa = b_FREFF + SHykappa;
  if (CAVE_MachE_sil_test_any(isLowSpeed)) {
    loop_ub = reduction_factor_size[0] * reduction_factor_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      tmpDrphiVar_data = Bykappa * reduction_factor_data;
    }

    if (isLowSpeed) {
      Bykappa = tmpDrphiVar_data;
    }
  }

  for (i = 0; i < 9; i++) {
    zeta[i] = 1.0;
  }

  i = 0;
  if (turnslipinds) {
    i = 1;
  }

  if (0 <= i - 1) {
    zeta[0] = 0.0;
  }

  Bxphi = (rtp_PDXP2 * dfz + 1.0) * Bxphi * cos(atan(rtp_PDXP3 * b_FREFF));
  Byphi = (rtp_PDYP2 * dfz + 1.0) * Byphi * cos(atan(rtp_PDYP3 * tan(b_FZMIN)));
  i = 0;
  loop_ub = 0;
  if (turnslipinds) {
    i = 1;
    loop_ub = 1;
  }

  fc_size[0] = 1;
  fc_size[1] = i;
  loop_ub--;
  if (0 <= loop_ub) {
    fc_data = Bxphi * rtp_UNLOADED_RADIUS * phi_t;
  }

  CAVE_MachE_sil_test_atan(&fc_data, fc_size);
  CAVE_MachE_sil_test_cos(&fc_data, fc_size);
  loop_ub = fc_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[1] = fc_data;
  }

  loop_ub = 0;
  b_i = 0;
  if (turnslipinds) {
    loop_ub = 1;
  }

  o_trueCount = 0;
  if (turnslipinds) {
    b_i = 1;
  }

  i = 0;
  if (turnslipinds) {
    o_trueCount = 1;
    i = 1;
  }

  phi_t_size_1[0] = 1;
  phi_t_size_1[1] = loop_ub;
  if (0 <= loop_ub - 1) {
    rtu_Vx_data = phi_t;
  }

  CAVE_MachE_sil_test_abs(&rtu_Vx_data, phi_t_size_1, &fc_data, fc_size);
  phi_t_size_0[0] = 1;
  phi_t_size_0[1] = o_trueCount;
  if (0 <= o_trueCount - 1) {
    rtu_Vx_data = phi_t;
  }

  CAVE_MachE_sil_test_abs(&rtu_Vx_data, phi_t_size_0, &reduction_factor_data_0,
    tmp_size);
  tmpDrphiVar_size[0] = 1;
  tmpDrphiVar_size[1] = b_i;
  loop_ub = tmp_size[0] * tmp_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    tmpDrphiVar_data = rtp_UNLOADED_RADIUS * reduction_factor_data_0;
  }

  CAVE_MachE_sil_test_sqrt(&tmpDrphiVar_data, tmpDrphiVar_size);
  loop_ub = i;
  fc_size[0] = 1;
  fc_size[1] = i;
  loop_ub--;
  for (b_i = 0; b_i <= loop_ub; b_i++) {
    Bxphi = fc_data;
    Bxphi = (rtp_UNLOADED_RADIUS * Bxphi + b_PDYP4 * tmpDrphiVar_data) * Byphi;
    fc_data = Bxphi;
  }

  CAVE_MachE_sil_test_atan(&fc_data, fc_size);
  CAVE_MachE_sil_test_cos(&fc_data, fc_size);
  loop_ub = fc_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[2] = fc_data;
  }

  i = 0;
  loop_ub = 0;
  if (turnslipinds) {
    i = 1;
  }

  b_i = 0;
  if (turnslipinds) {
    loop_ub = 1;
    b_i = 1;
  }

  b_UNLOADED_RADIUS_size[0] = 1;
  b_UNLOADED_RADIUS_size[1] = loop_ub;
  if (0 <= loop_ub - 1) {
    rtu_Vx_data = rtp_UNLOADED_RADIUS;
  }

  CAVE_MachE_sil_test_power(&rtu_Vx_data, b_UNLOADED_RADIUS_size,
    &reduction_factor_data_0, tmp_size);
  phi_t_size[0] = 1;
  phi_t_size[1] = b_i;
  if (0 <= b_i - 1) {
    rtu_Vx_data = phi_t;
  }

  CAVE_MachE_sil_test_power(&rtu_Vx_data, phi_t_size, &tmp_data, rtu_Vx_size);
  fc_size[0] = 1;
  fc_size[1] = i;
  loop_ub = tmp_size[0] * tmp_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    fc_data = b_PKYP1 * reduction_factor_data_0 * tmp_data;
  }

  CAVE_MachE_sil_test_atan(&fc_data, fc_size);
  CAVE_MachE_sil_test_cos(&fc_data, fc_size);
  loop_ub = fc_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[3] = fc_data;
  }

  Kygammao = (b_PKY7 * dfz + Kyalpha) * b_Q_FZ1 * (Kygammao * dpi + 1.0) *
    scaleFactors[14];
  Kyalpha = sin(atan(b_Q_FZ1 / lam_mux / (gamma_star * gamma_star * rtp_PKY5 +
    b_PKY2) / (rtp_PPY2 * dpi + 1.0)) * rtp_PKY4) * ((rtp_PPY1 * dpi + 1.0) *
    (DHyphi * lam_mux) * (1.0 - b_PKY3 * fabs(gamma_star))) * zeta[3] *
    scaleFactors[5];
  CAVE_MachE_sil_test_div0protect_pe(Kyalpha, &b_PKY7, &b_PKYP1);
  CAVE_MachE_sil_test_div0protect_pe(DHyphi * lam_mux * (rtp_PPY1 * dpi + 1.0) *
    (1.0 - b_PKY3 * 0.0) * sin(rtp_PKY4 * atan(b_Q_FZ1 / lam_mux / (b_PKY2 +
    rtp_PKY5 * (gamma_star * gamma_star)) / (rtp_PPY2 * dpi + 1.0))) * zeta[3] *
    lam_Kyalpha, &b_PKYP1, &b_PDYP4);
  if (b_PHYP1 < 0.0) {
    b_PHYP1 = 0.0;
  }

  DHyphi = (rtp_PHYP3 * dfz + rtp_PHYP2) * tanh(rtu_Vx);
  b_PKY3 = BHyphi;
  if (BHyphi > 1.0) {
    b_PKY3 = 1.0;
  }

  BHyphi = Kygammao / (1.0 - epsilon_gamma) / (b_PHYP1 * DHyphi * b_PKYP1);
  b_PKY2 = rtp_UNLOADED_RADIUS * phi_t;
  BHyphi = sin(atan(BHyphi * b_PKY2 - (BHyphi * b_PKY2 - atan(BHyphi * b_PKY2)) *
                    b_PKY3) * b_PHYP1) * DHyphi * tanh(rtu_Vx);
  SVygamma = (rtp_PVY4 * dfz + SVygamma) * b_Q_FZ1 * gamma_star * zeta[2] *
    scaleFactors[14] * Vs;
  i = 0;
  if (turnslipinds) {
    i = 1;
  }

  if (0 <= i - 1) {
    zeta[4] = (BHyphi + 1.0) - SVygamma / b_PKY7;
  }

  i = 0;
  loop_ub = 0;
  if (turnslipinds) {
    i = 1;
    loop_ub = 1;
  }

  fc_size[0] = 1;
  fc_size[1] = i;
  loop_ub--;
  if (0 <= loop_ub) {
    fc_data = Mzphiinf * rtp_UNLOADED_RADIUS * phi_t;
  }

  CAVE_MachE_sil_test_atan(&fc_data, fc_size);
  CAVE_MachE_sil_test_cos(&fc_data, fc_size);
  loop_ub = fc_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[5] = fc_data;
  }

  i = 0;
  loop_ub = 0;
  if (turnslipinds) {
    i = 1;
    loop_ub = 1;
  }

  fc_size[0] = 1;
  fc_size[1] = i;
  loop_ub--;
  if (0 <= loop_ub) {
    fc_data = rtp_QBRP1 * rtp_UNLOADED_RADIUS * phi_t;
  }

  CAVE_MachE_sil_test_atan(&fc_data, fc_size);
  CAVE_MachE_sil_test_cos(&fc_data, fc_size);
  loop_ub = fc_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[6] = fc_data;
  }

  Mzphiinf = rtp_QCRP1 * mu_y * rtp_UNLOADED_RADIUS * b_Q_FZ1 * sqrt(b_Q_FZ1 /
    lam_mux) * scaleFactors[26];
  if (Mzphiinf < 0.0) {
    Mzphiinf = 1.0E-6;
  }

  Gykappa = cos(atan(Vc_prime * Bykappa - (Vc_prime * Bykappa - atan(Vc_prime *
    Bykappa)) * Eykappa) * Gykappa) / cos(atan(Vc_prime * SHykappa - (Vc_prime *
    SHykappa - atan(Vc_prime * SHykappa)) * Eykappa) * Gykappa);
  if (Gykappa < 0.0) {
    Gykappa = 0.0;
  }

  Mzphi90 = Mzphiinf * 2.0 / 3.1415926535897931 * atan(Mzphi90 *
    rtp_UNLOADED_RADIUS * fabs(b_PHX2)) * Gykappa;
  DHyphi = DDrphi;
  if (DDrphi < 0.0) {
    DHyphi = 0.0;
  }

  b_PKY2 = b_QDRP2;
  if (b_QDRP2 < 0.0) {
    b_PKY2 = 0.0;
  }

  DDrphi = Mzphiinf / sin(1.5707963267948966 * DHyphi);
  Mzphiinf = 1.0 - epsilon_gamma;
  if (Mzphiinf < 0.0) {
    Mzphiinf = -1.0;
  } else if (Mzphiinf > 0.0) {
    Mzphiinf = 1.0;
  } else if (Mzphiinf == 0.0) {
    Mzphiinf = 0.0;
  } else {
    Mzphiinf = (rtNaN);
  }

  lam_Kzgamma = ((rtp_QDZ11 * dfz + rtp_QDZ10) * fabs(unusedU0) + (rtp_QDZ9 *
    dfz + Mzr)) * (b_Q_FZ1 * rtp_UNLOADED_RADIUS) * lam_Kzgamma / (DHyphi *
    DDrphi * (1.0 - epsilon_gamma) + 0.0001 * Mzphiinf);
  b_QDRP2 = -rtp_UNLOADED_RADIUS * phi_t;
  lam_Kzgamma = sin(atan(lam_Kzgamma * b_QDRP2 - (lam_Kzgamma * b_QDRP2 - atan
    (lam_Kzgamma * b_QDRP2)) * b_PKY2) * DHyphi) * DDrphi;
  if (lam_Kzgamma < 0.0) {
    DDrphi = -1.0;
  } else if (lam_Kzgamma > 0.0) {
    DDrphi = 1.0;
  } else if (lam_Kzgamma == 0.0) {
    DDrphi = 0.0;
  } else {
    DDrphi = (rtNaN);
  }

  if (DDrphi == 0.0) {
    DDrphi = 1.0;
  }

  i = 0;
  if (turnslipinds) {
    i = 1;
  }

  lam_Kzgamma_size[0] = 1;
  lam_Kzgamma_size[1] = i;
  if (0 <= i - 1) {
    rtu_Vx_data = lam_Kzgamma;
  }

  CAVE_MachE_sil_test_abs(&rtu_Vx_data, lam_Kzgamma_size,
    &reduction_factor_data_0, tmp_size);
  tmpDrphiVar_size[0] = 1;
  tmpDrphiVar_size[1] = i;
  phi_t = 0.0001 * DDrphi;
  if (0 <= i - 1) {
    tmpDrphiVar_data = Mzphi90 / (-reduction_factor_data_0 + phi_t);
  }

  i = tmpDrphiVar_size[1];
  loop_ub = 0;
  while (loop_ub <= i - 1) {
    if (tmpDrphiVar_data > 1.0) {
      tmpDrphiVar_data = 1.0;
    }

    loop_ub = 1;
  }

  i = tmpDrphiVar_size[1];
  loop_ub = 0;
  while (loop_ub <= i - 1) {
    if (tmpDrphiVar_data < -1.0) {
      tmpDrphiVar_data = -1.0;
    }

    loop_ub = 1;
  }

  CAVE_MachE_sil_test_acos(&tmpDrphiVar_data, tmpDrphiVar_size);
  loop_ub = tmpDrphiVar_size[1];
  for (b_i = 0; b_i < loop_ub; b_i++) {
    zeta[7] = 0.63661977236758138 * tmpDrphiVar_data;
  }

  i = 0;
  if (turnslipinds) {
    i = 1;
  }

  if (0 <= i - 1) {
    zeta[8] = lam_Kzgamma + 1.0;
  }

  b_i = 0;
  if (turnslipinds) {
    b_i = 1;
  }

  loop_ub = b_i - 1;
  if (0 <= loop_ub) {
    tmpDrphiVar_data = 0.0 * dfz * lam_Hy;
  }

  i = 0;
  if (turnslipinds) {
    i = 1;
  }

  loop_ub = i - 1;
  if (0 <= loop_ub) {
    fc_data = SVygamma / b_PKY7;
  }

  lam_Kzgamma = 0.0;
  if (turnslipinds) {
    lam_Kzgamma = (tmpDrphiVar_data + BHyphi) - fc_data;
  }

  Cx *= scaleFactors[6];
  if (Cx < 0.0) {
    Cx = 0.0;
  }

  Dx = ((rtp_PPX3 * dpi + 1.0) + dpi * dpi * rtp_PPX4) * (rtp_PDX2 * dfz + Dx) *
    (1.0 - unusedU0 * unusedU0 * rtp_PDX3) * Vc * b_Q_FZ1 * zeta[1];
  if (Dx < 0.0) {
    Dx = 0.0;
  }

  Ex = ((rtp_PEX2 * dfz + Ex) + dfz * dfz * rtp_PEX3) * (1.0 - tanh(10.0 *
    kappa_x) * rtp_PEX4) * scaleFactors[8];
  if (Ex > 1.0) {
    Ex = 1.0;
  }

  Kxkappa = (Bx * dfz + Kxkappa) * b_Q_FZ1 * exp(b_PKX3 * dfz) * ((rtp_PPX1 *
    dpi + 1.0) + dpi * dpi * rtp_PPX2) * scaleFactors[4];
  CAVE_MachE_sil_test_div0protect_pe(Cx * Dx, &Bx, &b_PKX3);
  Bx = Kxkappa / Bx;
  Exalpha += rtp_REX2 * dfz;
  if (Exalpha > 1.0) {
    Exalpha = 1.0;
  }

  Bxalpha = (gamma_star * gamma_star * rtp_RBX3 + Bxalpha) * cos(atan(rtp_RBX2 *
    b_FREFF)) * scaleFactors[18];
  if (Bxalpha < 0.0) {
    Bxalpha = 0.0;
  }

  Gxalpha = cos(atan(Bxalpha * b_FZMIN - (Bxalpha * b_FZMIN - atan(Bxalpha *
    b_FZMIN)) * Exalpha) * Gxalpha) / cos(atan(Bxalpha * 0.0 - (Bxalpha * 0.0 -
    atan(Bxalpha * 0.0)) * Exalpha) * Gxalpha);
  if (Gxalpha < 0.0) {
    Gxalpha = 0.0;
  }

  b_PVX1 = (sin(atan(Bx * kappa_x - (Bx * kappa_x - atan(Bx * kappa_x)) * Ex) *
                Cx) * Dx + (Dy * dfz + b_PVX1) * b_Q_FZ1 * (Vc * 10.0 / (0.0 *
              Vc + 1.0)) * scaleFactors[12] * zeta[1]) * Gxalpha;
  if (CAVE_MachE_sil_test_any(isLowSpeed)) {
    loop_ub = reduction_factor_size[0] * reduction_factor_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      tmpDrphiVar_data = b_PVX1 * reduction_factor_data;
    }

    if (isLowSpeed) {
      b_PVX1 = tmpDrphiVar_data;
    }
  }

  Cy *= scaleFactors[7];
  kappa_x = Cy;
  if (Cy < 0.0) {
    kappa_x = 0.0;
  }

  Dy = mu_y * b_Q_FZ1 * zeta[2];
  CAVE_MachE_sil_test_div0protect_pe(kappa_x * Dy, &Cy, &Gxalpha);
  Cy = Kyalpha / Cy;
  if (CAVE_MachE_sil_test_any(!turnslipinds)) {
    b_i = 0;
    o_trueCount = 0;
    if (!turnslipinds) {
      b_i = 1;
      o_trueCount = 1;
    }

    loop_ub = o_trueCount - 1;
    if (0 <= loop_ub) {
      tmpDrphiVar_data = (Kygammao * gamma_star - SVygamma) / b_PKY7;
    }

    loop_ub = b_i - 1;
    if (0 <= loop_ub) {
      fc_data = 0.0 * dfz * lam_Hy;
    }

    i = 0;
    if (!turnslipinds) {
      i = 1;
    }

    if (0 <= i - 1) {
      lam_Kzgamma = ((tmpDrphiVar_data * zeta[0] + fc_data) + zeta[4]) - 1.0;
    }
  }

  lam_Hy = 0.0 * dfz * b_Q_FZ1 * scaleFactors[13] * Vs * zeta[2] + SVygamma;
  Gxalpha = b_FZMIN + lam_Kzgamma;
  if (Gxalpha < 0.0) {
    Bxalpha = -1.0;
  } else if (Gxalpha > 0.0) {
    Bxalpha = 1.0;
  } else if (Gxalpha == 0.0) {
    Bxalpha = 0.0;
  } else {
    Bxalpha = (rtNaN);
  }

  if (Bxalpha == 0.0) {
    Bxalpha = 1.0;
  }

  Ey = ((gamma_star * gamma_star * rtp_PEY5 + 1.0) - rtp_PEY4 * gamma_star *
        Bxalpha) * (rtp_PEY2 * dfz + Ey) * scaleFactors[9];
  if (Ey > 1.0) {
    Ey = 1.0;
  }

  SVykappa = (0.0 * dfz + SVykappa * gamma_star) * (mu_y * b_Q_FZ1) * cos(atan
    (b_RVY4 * b_FZMIN)) * zeta[2] * sin(atan(rtp_RVY6 * b_FREFF) * rtp_RVY5) *
    scaleFactors[20];
  b_RVY4 = (sin(atan(Cy * Gxalpha - (Cy * Gxalpha - atan(Cy * Gxalpha)) * Ey) *
                kappa_x) * Dy + lam_Hy) * Gykappa + SVykappa;
  if (CAVE_MachE_sil_test_any(fabs(rtu_Vx) < b_VXLOW)) {
    loop_ub = 0;
    if (isLowSpeed) {
      loop_ub = 1;
    }

    tmpDrphiVar_size[0] = 1;
    tmpDrphiVar_size[1] = loop_ub;
    loop_ub--;
    if (0 <= loop_ub) {
      tmpDrphiVar_data = 4.0 * rtu_Vy;
    }

    CAVE_MachE_sil_test_tanh(&tmpDrphiVar_data, tmpDrphiVar_size);
    i = tmpDrphiVar_size[0] * tmpDrphiVar_size[1];
    loop_ub = i - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      b_VXLOW = tmpDrphiVar_data;
      b_VXLOW *= b_RVY4;
      tmpDrphiVar_data = b_VXLOW;
    }

    if (isLowSpeed) {
      b_RVY4 = tmpDrphiVar_data;
    }
  }

  b_VXLOW = cosprimealpha * b_Q_FZ1 / rtp_FNOMIN;
  cosprimealpha = rtu_Vx / Vsx;
  alpha_t = ((alpha_r * dfz + alpha_t) * gamma_star + 0.0 * dfz) + b_FZMIN;
  alpha_r = (lam_Hy / b_PKY7 + lam_Kzgamma) + b_FZMIN;
  Bt = (((0.0 * unusedU0 + 1.0) + rtp_QBZ5 * fabs(unusedU0)) + gamma_star *
        gamma_star * rtp_QBZ6) * ((rtp_QBZ2 * dfz + Bt) + dfz * dfz * rtp_QBZ3) *
    scaleFactors[5] / Vsy;
  if (Bt < 0.0) {
    Bt = 0.0;
  }

  Vsx = Et;
  if (Et < 0.0) {
    Vsx = 0.0;
  }

  Et = (rtp_QEZ5 * gamma_star * 2.0 / 3.1415926535897931 * atan(Bt * Vsx *
         alpha_t) + 1.0) * ((b_QEZ2 * dfz + b_QEZ1) + dfz * dfz * rtp_QEZ3);
  b_QEZ2 = Et;
  if (Et > 1.0) {
    b_QEZ2 = 1.0;
  }

  Br = (Br * scaleFactors[5] / Vsy + alpha_req * Cy * kappa_x) * zeta[6];
  alpha_req = tan(alpha_r);
  Et = Kxkappa / b_PKY7;
  alpha_req = atan(sqrt(Et * Et * (b_FREFF * b_FREFF) + alpha_req * alpha_req)) *
    tanh(10.0 * alpha_r);
  Et = tan(alpha_t);
  b_QEZ1 = Kxkappa / b_PKY7;
  Et = atan(sqrt(b_QEZ1 * b_QEZ1 * (b_FREFF * b_FREFF) + Et * Et)) * tanh(10.0 *
    alpha_t);
  Mzr = (((((rtp_QDZ9 * dfz + Mzr) * (rtp_PPZ2 * dpi + 1.0) + (rtp_QDZ11 * dfz +
             rtp_QDZ10) * fabs(gamma_star)) * gamma_star * scaleFactors[15] *
           zeta[0] + 0.0 * dfz * scaleFactors[17] * zeta[2]) * (b_Q_FZ1 *
           rtp_UNLOADED_RADIUS) * Vsy * tanh(10.0 * rtu_Vx) * cosprimealpha +
          zeta[8]) - 1.0) * cos(atan(Br * alpha_req - (Br * alpha_req - atan(Br *
    alpha_req)) * 0.0) * zeta[7]);
  t = (rtp_QDZ2 * dfz + t) * (1.0 - rtp_PPZ1 * dpi) * ((0.0 * unusedU0 + 1.0) +
    unusedU0 * unusedU0 * rtp_QDZ4) * b_Q_FZ1 * (rtp_UNLOADED_RADIUS / lam_mux) *
    scaleFactors[16] * zeta[5] * cos(atan(Bt * Et - (Bt * Et - atan(Bt * Et)) *
    b_QEZ2) * Vsx) * cosprimealpha * scaleFactors[0];
  if (CAVE_MachE_sil_test_any(isLowSpeed)) {
    loop_ub = reduction_factor_size[0] * reduction_factor_size[1] - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      tmpDrphiVar_data = t * reduction_factor_data;
    }

    if (isLowSpeed) {
      t = tmpDrphiVar_data;
    }

    i = reduction_factor_size[0] * reduction_factor_size[1];
    loop_ub = i - 1;
    for (b_i = 0; b_i <= loop_ub; b_i++) {
      cosprimealpha = reduction_factor_data;
      cosprimealpha *= Mzr;
      reduction_factor_data = cosprimealpha;
    }

    if (isLowSpeed) {
      Mzr = reduction_factor_data;
    }
  }

  /* '<S349>:1:193' */
  localB->Fx = b_PVX1;

  /* '<S349>:1:193' */
  localB->Fy = b_RVY4;

  /* '<S349>:1:193' */
  localB->FzTire = b_Q_FZ1;

  /* '<S349>:1:193' */
  *rty_Mx = (((((0.0 * lam_VMx - (rtp_PPMX1 * dpi + 1.0) * (rtp_QSX2 * unusedU0))
                - rtp_QSX12 * unusedU0 * fabs(unusedU0)) + rtp_QSX3 * b_RVY4 /
               rtp_FNOMIN) + sin(atan(rtp_QSX9 * b_RVY4 / rtp_FNOMIN) * rtp_QSX8
    + rtp_QSX7 * unusedU0) * (cos(atan(b_VXLOW * b_VXLOW) * rtp_QSX5) * rtp_QSX4))
             + atan(rtp_QSX11 * b_Q_FZ1 / rtp_FNOMIN) * rtp_QSX10 * unusedU0) *
    (rtp_UNLOADED_RADIUS * b_Q_FZ1 * lam_Mx) + rtp_UNLOADED_RADIUS * b_RVY4 *
    lam_Mx * (rtp_QSX14 * fabs(unusedU0) + rtp_QSX13);

  /* '<S349>:1:193' */
  localB->My = tanh(10.0 * rtu_Omega) * CAVE_MachE_sil_test_rollingMoment(b_PVX1,
    rtu_Vx, b_Q_V2, b_CAMMIN, unusedU0, rtp_LONGVL, rtp_UNLOADED_RADIUS,
    rtp_FNOMIN, rtp_NOMPRES, rtp_QSY1, rtp_QSY2, rtp_QSY3, rtp_QSY4, rtp_QSY5,
    rtp_QSY6, rtp_QSY7, rtp_QSY8, lam_My);

  /* '<S349>:1:193' */
  *rty_Mz = ((rtp_SSZ4 * dfz + rtp_SSZ3) * gamma_star + rtp_SSZ2 * b_RVY4 /
             rtp_FNOMIN) * rtp_UNLOADED_RADIUS * scaleFactors[21] * b_PVX1 +
    ((b_RVY4 - SVykappa) * -t + Mzr);

  /* '<S349>:1:193' */
  localB->Re = b_BREFF;

  /* '<S349>:1:193' */
  *rty_Kappa = b_FREFF;

  /* '<S349>:1:193' */
  *rty_Alpha = b_FZMIN;

  /* '<S349>:1:193' */
  localB->sig_x = fabs(Kxkappa / (((rtp_PCFX1 * dfz + 1.0) + dfz * dfz *
    rtp_PCFX2) * rtp_LONGITUDINAL_STIFFNESS * (rtp_PCFX3 * dpi + 1.0)));

  /* '<S349>:1:193' */
  localB->sig_y = fabs(Kyalpha / (((rtp_PCFY1 * dfz + 1.0) + dfz * dfz *
    rtp_PCFY2) * rtp_LATERAL_STIFFNESS * (rtp_PCFY3 * dpi + 1.0)));

  /* '<S349>:1:193' */
  localB->a = (b_Q_V2 / (Cz * rtp_UNLOADED_RADIUS) * rtp_Q_RA2 + sqrt(b_Q_V2 /
    (Cz * rtp_UNLOADED_RADIUS)) * rtp_Q_RA1) * rtp_UNLOADED_RADIUS;

  /* '<S349>:1:193' */
  localB->b = (b_Q_V2 / (Cz * rtp_UNLOADED_RADIUS) * rtp_Q_RB2 + rt_powd_snf
               (b_Q_V2 / (Cz * rtp_UNLOADED_RADIUS), 0.33333333333333331) *
               rtp_Q_RB1) * rtp_WIDTH;
}

/* Function for Chart: '<S346>/LockUp' */
static boolean_T CAVE_MachE_sil_test_detectLockup(real_T Tout, real_T Tfmaxs,
  B_LockUp_CAVE_MachE_sil_test_T *localB, DW_LockUp_CAVE_MachE_sil_test_T
  *localDW, P_LockUp_CAVE_MachE_sil_test_T *localP, real_T rtp_br)
{
  int32_T rowIdx;
  boolean_T tmp;

  /* Simulink Function 'detectLockup': '<S353>:63' */
  localB->Tout_l = Tout;
  localB->Tfmaxs_b = Tfmaxs;

  /* Outputs for Function Call SubSystem: '<S353>/detectLockup' */
  /* Gain: '<S364>/Output Damping' incorporates:
   *  Constant: '<S358>/Constant'
   */
  localB->OutputDamping_j = rtp_br * localP->Constant_Value;

  /* Sum: '<S364>/Sum2' */
  localB->Sum2 = (0.0 - localB->Tout_l) - localB->OutputDamping_j;

  /* Sum: '<S364>/Sum1' */
  localB->Sum1 = localB->Sum2 + localB->OutputDamping_j;

  /* Abs: '<S361>/Abs' */
  localB->Abs = fabs(localB->Sum1);

  /* RelationalOperator: '<S361>/Relational Operator' */
  localB->RelationalOperator = (localB->Abs >= localB->Tfmaxs_b);

  /* UnaryMinus: '<S365>/Unary Minus' */
  localB->UnaryMinus = -localB->Tout_l;

  /* Abs: '<S366>/Abs' */
  localB->Abs_e = fabs(localB->UnaryMinus);

  /* RelationalOperator: '<S366>/Relational Operator' */
  localB->RelationalOperator_c = (localB->Abs_e <= localB->Tfmaxs_b);

  /* UnitDelay: '<S363>/Unit Delay' */
  localB->UnitDelay = localDW->UnitDelay_DSTATE;

  /* CombinatorialLogic: '<S363>/Combinatorial  Logic' */
  tmp = localB->RelationalOperator_c;
  rowIdx = tmp;
  tmp = localB->RelationalOperator;
  rowIdx = (int32_T)(((uint32_T)rowIdx << 1) + tmp);
  tmp = localB->UnitDelay;
  rowIdx = (int32_T)(((uint32_T)rowIdx << 1) + tmp);
  localB->CombinatorialLogic = localP->CombinatorialLogic_table[(uint32_T)rowIdx];

  /* Update for UnitDelay: '<S363>/Unit Delay' */
  localDW->UnitDelay_DSTATE = localB->CombinatorialLogic;

  /* End of Outputs for SubSystem: '<S353>/detectLockup' */
  return localB->CombinatorialLogic;
}

/* Function for Chart: '<S346>/LockUp' */
static boolean_T CAVE_MachE_sil_test_detectSlip(real_T Tout, real_T Tfmaxs,
  B_LockUp_CAVE_MachE_sil_test_T *localB)
{
  /* Simulink Function 'detectSlip': '<S353>:65' */
  localB->Tout = Tout;
  localB->Tfmaxs = Tfmaxs;

  /* Outputs for Function Call SubSystem: '<S353>/detectSlip' */
  /* Abs: '<S367>/Abs' */
  localB->Abs_l = fabs(localB->Tout);

  /* RelationalOperator: '<S367>/Relational Operator' */
  localB->RelationalOperator_d = (localB->Abs_l >= localB->Tfmaxs);

  /* End of Outputs for SubSystem: '<S353>/detectSlip' */
  return localB->RelationalOperator_d;
}

/*
 * System initialize for atomic system:
 *    '<S346>/LockUp'
 *    '<S371>/LockUp'
 *    '<S396>/LockUp'
 *    '<S421>/LockUp'
 */
void CAVE_MachE_sil_test_LockUp_Init(B_LockUp_CAVE_MachE_sil_test_T *localB,
  DW_LockUp_CAVE_MachE_sil_test_T *localDW, P_LockUp_CAVE_MachE_sil_test_T
  *localP, X_LockUp_CAVE_MachE_sil_test_T *localX, real_T rtp_omegao)
{
  localDW->is_active_c6_autolibshared = 0U;
  localDW->is_c6_autolibshared = CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD;
  localB->Omega = 0.0;
  localB->Omegadot = 0.0;
  localB->Myb = 0.0;
  localB->ReactionTorque = 0.0;

  /* SystemInitialize for Function Call SubSystem: '<S353>/detectSlip' */
  /* SystemInitialize for Outport: '<S359>/yn' */
  localB->RelationalOperator_d = localP->yn_Y0;

  /* End of SystemInitialize for SubSystem: '<S353>/detectSlip' */

  /* SystemInitialize for Function Call SubSystem: '<S353>/detectLockup' */
  /* InitializeConditions for UnitDelay: '<S363>/Unit Delay' */
  localDW->UnitDelay_DSTATE = localP->UnitDelay_InitialCondition;

  /* SystemInitialize for Outport: '<S358>/yn' */
  localB->CombinatorialLogic = localP->yn_Y0_m;

  /* End of SystemInitialize for SubSystem: '<S353>/detectLockup' */

  /* SystemInitialize for IfAction SubSystem: '<S353>/Slipping' */
  /* InitializeConditions for Integrator: '<S357>/omega wheel' */
  localX->omegaWheel_l = rtp_omegao;

  /* End of SystemInitialize for SubSystem: '<S353>/Slipping' */
}

/*
 * Outputs for atomic system:
 *    '<S346>/LockUp'
 *    '<S371>/LockUp'
 *    '<S396>/LockUp'
 *    '<S421>/LockUp'
 */
void CAVE_MachE_sil_test_LockUp(RT_MODEL_CAVE_MachE_sil_test_T * const
  CAVE_MachE_sil_test_M, real_T rtu_Tout, real_T rtu_Tfmaxs, real_T rtu_Tfmaxk,
  B_LockUp_CAVE_MachE_sil_test_T *localB, DW_LockUp_CAVE_MachE_sil_test_T
  *localDW, P_LockUp_CAVE_MachE_sil_test_T *localP,
  X_LockUp_CAVE_MachE_sil_test_T *localX, real_T rtp_omegao, real_T rtp_br,
  real_T rtp_Iyy, real_T rtp_OmegaTol)
{
  real_T tmp;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    localDW->lastMajorTime = CAVE_MachE_sil_test_M->Timing.t[0];

    /* Chart: '<S346>/LockUp' */
    /* Gateway: Longitudinal Wheel/Wheel Module/Clutch */
    /* During: Longitudinal Wheel/Wheel Module/Clutch */
    if (localDW->is_active_c6_autolibshared == 0U) {
      /* Entry: Longitudinal Wheel/Wheel Module/Clutch */
      localDW->is_active_c6_autolibshared = 1U;

      /* Entry Internal: Longitudinal Wheel/Wheel Module/Clutch */
      /* Transition: '<S353>:18' */
      localX->omegaWheel_l = rtp_omegao;
      localDW->is_c6_autolibshared = CAVE_MachE_sil_test_IN_Slipping;

      /* Integrator: '<S357>/omega wheel' */
      localB->omegawheel = localX->omegaWheel_l;

      /* Gain: '<S357>/-4' */
      localB->u = localP->u_Gain * localB->omegawheel;

      /* Trigonometry: '<S357>/Trigonometric Function' */
      localB->TrigonometricFunction = tanh(localB->u);

      /* Product: '<S357>/Max Dynamic Friction Torque1' */
      localB->MaxDynamicFrictionTorque1 = rtu_Tfmaxk *
        localB->TrigonometricFunction;

      /* Gain: '<S357>/Output Damping' */
      localB->OutputDamping = rtp_br * localB->omegawheel;

      /* SignalConversion generated from: '<S357>/Myb' */
      localB->Myb = localB->OutputDamping;

      /* SignalConversion generated from: '<S357>/Omega' */
      localB->Omega = localB->omegawheel;

      /* Sum: '<S357>/Output Sum' */
      localB->OutputSum = (localB->MaxDynamicFrictionTorque1 - rtu_Tout) -
        localB->OutputDamping;

      /* Gain: '<S357>/Output Inertia' */
      tmp = 1.0 / rtp_Iyy;
      localB->OutputInertia = tmp * localB->OutputSum;

      /* SignalConversion generated from: '<S357>/Omegadot' */
      localB->Omegadot = localB->OutputInertia;

      /* SignalConversion generated from: '<S357>/ReactionTorque' */
      localB->ReactionTorque = localB->OutputSum;
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&CAVE_MachE_sil_test_M->solverInfo, true);
    } else if (localDW->is_c6_autolibshared == CAVE_MachE_sil_test_IN_Locked) {
      /* During 'Locked': '<S353>:2' */
      if (CAVE_MachE_sil_test_detectSlip(rtu_Tout, rtu_Tfmaxs, localB)) {
        /* Transition: '<S353>:17' */
        localX->omegaWheel_l = 0.0;
        localDW->is_c6_autolibshared = CAVE_MachE_sil_test_IN_Slipping;

        /* Integrator: '<S357>/omega wheel' */
        localB->omegawheel = localX->omegaWheel_l;

        /* Gain: '<S357>/-4' */
        localB->u = localP->u_Gain * localB->omegawheel;

        /* Trigonometry: '<S357>/Trigonometric Function' */
        localB->TrigonometricFunction = tanh(localB->u);

        /* Product: '<S357>/Max Dynamic Friction Torque1' */
        localB->MaxDynamicFrictionTorque1 = rtu_Tfmaxk *
          localB->TrigonometricFunction;

        /* Gain: '<S357>/Output Damping' */
        localB->OutputDamping = rtp_br * localB->omegawheel;

        /* SignalConversion generated from: '<S357>/Myb' */
        localB->Myb = localB->OutputDamping;

        /* SignalConversion generated from: '<S357>/Omega' */
        localB->Omega = localB->omegawheel;

        /* Sum: '<S357>/Output Sum' */
        localB->OutputSum = (localB->MaxDynamicFrictionTorque1 - rtu_Tout) -
          localB->OutputDamping;

        /* Gain: '<S357>/Output Inertia' */
        tmp = 1.0 / rtp_Iyy;
        localB->OutputInertia = tmp * localB->OutputSum;

        /* SignalConversion generated from: '<S357>/Omegadot' */
        localB->Omegadot = localB->OutputInertia;

        /* SignalConversion generated from: '<S357>/ReactionTorque' */
        localB->ReactionTorque = localB->OutputSum;
        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&CAVE_MachE_sil_test_M->solverInfo, true);
      }
    } else {
      /* During 'Slipping': '<S353>:62' */
      if (CAVE_MachE_sil_test_detectLockup(rtu_Tout, rtu_Tfmaxs, localB, localDW,
           localP, rtp_br) && (fabs(localB->Omega) <= rtp_OmegaTol)) {
        /* Transition: '<S353>:16' */
        localDW->is_c6_autolibshared = CAVE_MachE_sil_test_IN_Locked;

        /* UnaryMinus: '<S356>/Unary Minus' */
        localB->ReactionTorque = -rtu_Tout;
        if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
          /* SignalConversion generated from: '<S356>/Omega' incorporates:
           *  Constant: '<S356>/locked'
           */
          localB->Omega = localP->locked_Value;

          /* SignalConversion generated from: '<S356>/Omegadot' incorporates:
           *  Constant: '<S356>/locked1'
           */
          localB->Omegadot = localP->locked1_Value;

          /* SignalConversion generated from: '<S356>/Myb' incorporates:
           *  Constant: '<S356>/locked2'
           */
          localB->Myb = localP->locked2_Value;
        }

        rtsiSetBlockStateForSolverChangedAtMajorStep
          (&CAVE_MachE_sil_test_M->solverInfo, true);
      }
    }

    /* End of Chart: '<S346>/LockUp' */
  }

  if (localDW->is_c6_autolibshared == CAVE_MachE_sil_test_IN_Locked) {
    /* UnaryMinus: '<S356>/Unary Minus' */
    /* During 'Locked': '<S353>:2' */
    localB->ReactionTorque = -rtu_Tout;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* SignalConversion generated from: '<S356>/Omega' incorporates:
       *  Constant: '<S356>/locked'
       */
      localB->Omega = localP->locked_Value;

      /* SignalConversion generated from: '<S356>/Omegadot' incorporates:
       *  Constant: '<S356>/locked1'
       */
      localB->Omegadot = localP->locked1_Value;

      /* SignalConversion generated from: '<S356>/Myb' incorporates:
       *  Constant: '<S356>/locked2'
       */
      localB->Myb = localP->locked2_Value;
    }
  } else {
    /* Integrator: '<S357>/omega wheel' */
    /* During 'Slipping': '<S353>:62' */
    localB->omegawheel = localX->omegaWheel_l;

    /* Gain: '<S357>/-4' */
    localB->u = localP->u_Gain * localB->omegawheel;

    /* Trigonometry: '<S357>/Trigonometric Function' */
    localB->TrigonometricFunction = tanh(localB->u);

    /* Product: '<S357>/Max Dynamic Friction Torque1' */
    localB->MaxDynamicFrictionTorque1 = rtu_Tfmaxk *
      localB->TrigonometricFunction;

    /* Gain: '<S357>/Output Damping' */
    localB->OutputDamping = rtp_br * localB->omegawheel;

    /* SignalConversion generated from: '<S357>/Myb' */
    localB->Myb = localB->OutputDamping;

    /* SignalConversion generated from: '<S357>/Omega' */
    localB->Omega = localB->omegawheel;

    /* Sum: '<S357>/Output Sum' */
    localB->OutputSum = (localB->MaxDynamicFrictionTorque1 - rtu_Tout) -
      localB->OutputDamping;

    /* Gain: '<S357>/Output Inertia' */
    tmp = 1.0 / rtp_Iyy;
    localB->OutputInertia = tmp * localB->OutputSum;

    /* SignalConversion generated from: '<S357>/Omegadot' */
    localB->Omegadot = localB->OutputInertia;

    /* SignalConversion generated from: '<S357>/ReactionTorque' */
    localB->ReactionTorque = localB->OutputSum;
  }
}

/*
 * Derivatives for atomic system:
 *    '<S346>/LockUp'
 *    '<S371>/LockUp'
 *    '<S396>/LockUp'
 *    '<S421>/LockUp'
 */
void CAVE_MachE_sil_test_LockUp_Deriv(B_LockUp_CAVE_MachE_sil_test_T *localB,
  DW_LockUp_CAVE_MachE_sil_test_T *localDW, XDot_LockUp_CAVE_MachE_sil_test_T
  *localXdot)
{
  localXdot->omegaWheel_l = 0.0;
  if (localDW->is_c6_autolibshared == CAVE_MachE_sil_test_IN_Slipping) {
    /* Derivatives for Integrator: '<S357>/omega wheel' */
    localXdot->omegaWheel_l = localB->OutputInertia;
  }
}

void rt_mrdivide_U1d1x3_U2d3x3_Yd1x3_snf(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T u1_0[9];
  int32_T ONE;
  int32_T THREE;
  int32_T r1;
  int32_T r2;
  real_T maxval;
  real_T a21;
  real_T x;
  real_T u0_idx_0;
  real_T u0_idx_1;
  real_T u0_idx_2;
  u0_idx_0 = u0[0];
  u0_idx_1 = u0[1];
  u0_idx_2 = u0[2];
  memcpy(&u1_0[0], &u1[0], 9U * sizeof(real_T));
  THREE = 2;
  r1 = 0;
  r2 = 1;
  x = u1_0[0];
  x = fabs(x);
  maxval = x;
  x = u1_0[1];
  x = fabs(x);
  a21 = x;
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  x = u1_0[2];
  x = fabs(x);
  a21 = x;
  if (a21 > maxval) {
    r1 = 2;
    r2 = 1;
    THREE = 0;
  }

  u1_0[r2] /= u1_0[r1];
  u1_0[THREE] /= u1_0[r1];
  u1_0[r2 + 3] -= u1_0[r1 + 3] * u1_0[r2];
  u1_0[THREE + 3] -= u1_0[r1 + 3] * u1_0[THREE];
  u1_0[r2 + 6] -= u1_0[r1 + 6] * u1_0[r2];
  u1_0[THREE + 6] -= u1_0[r1 + 6] * u1_0[THREE];
  x = u1_0[THREE + 3];
  x = fabs(x);
  a21 = x;
  x = u1_0[r2 + 3];
  x = fabs(x);
  maxval = x;
  if (a21 > maxval) {
    ONE = r2 + 1;
    r2 = THREE;
    THREE = ONE - 1;
  }

  u1_0[THREE + 3] /= u1_0[r2 + 3];
  u1_0[THREE + 6] -= u1_0[THREE + 3] * u1_0[r2 + 6];
  y[r1] = u0_idx_0 / u1_0[r1];
  y[r2] = u0_idx_1 - u1_0[r1 + 3] * y[r1];
  y[THREE] = u0_idx_2 - u1_0[r1 + 6] * y[r1];
  y[r2] /= u1_0[r2 + 3];
  y[THREE] -= u1_0[r2 + 6] * y[r2];
  y[THREE] /= u1_0[THREE + 6];
  y[r2] -= u1_0[THREE + 3] * y[THREE];
  y[r1] -= y[THREE] * u1_0[THREE];
  y[r1] -= y[r2] * u1_0[r2];
}

/* Model output function */
void CAVE_MachE_sil_test_output(void)
{
  /* local scratch DWork variables */
  int32_T ForEach_itr;
  int32_T ForEach_itr_j;
  int32_T ForEach_itr_k;
  int32_T ForEach_itr_h;
  int32_T ForEach_itr_hh;
  int32_T ForEach_itr_c;
  int32_T ForEach_itr_o;
  int32_T ForEach_itr_f;
  boolean_T sf_internal_predicateOutput;
  int32_T y;
  real_T R[24];
  real_T M[8];
  real_T Imat[72];
  real_T b_b[21];
  real_T b_a[3];
  int32_T ibmat;
  int8_T c_b[21];
  real_T d_b[24];
  int32_T ibtile;
  real_T Itemp[72];
  real_T tmp[3];
  int8_T rtAction;
  real_T Bias;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_e4;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0;
  real_T rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l;
  real_T rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0_a;
  real_T M_0;
  real_T u[9];
  real_T rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
  real_T rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  static const int8_T e[3] = { -1, 1, -1 };

  static const int8_T f_b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T f[9] = { 1, -1, -1, -1, 1, -1, -1, -1, 1 };

  boolean_T exitg1;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* set solver stop time */
    if (!(CAVE_MachE_sil_test_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&CAVE_MachE_sil_test_M->solverInfo,
                            ((CAVE_MachE_sil_test_M->Timing.clockTickH0 + 1) *
        CAVE_MachE_sil_test_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&CAVE_MachE_sil_test_M->solverInfo,
                            ((CAVE_MachE_sil_test_M->Timing.clockTick0 + 1) *
        CAVE_MachE_sil_test_M->Timing.stepSize0 +
        CAVE_MachE_sil_test_M->Timing.clockTickH0 *
        CAVE_MachE_sil_test_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(CAVE_MachE_sil_test_M)) {
    CAVE_MachE_sil_test_M->Timing.t[0] = rtsiGetT
      (&CAVE_MachE_sil_test_M->solverInfo);
  }

  /* Integrator: '<S218>/xe,ye,ze' */
  CAVE_MachE_sil_test_B.xeyeze[0] = CAVE_MachE_sil_test_X.xeyeze_CSTATE[0];
  CAVE_MachE_sil_test_B.xeyeze[1] = CAVE_MachE_sil_test_X.xeyeze_CSTATE[1];
  CAVE_MachE_sil_test_B.xeyeze[2] = CAVE_MachE_sil_test_X.xeyeze_CSTATE[2];

  /* SignalConversion generated from: '<S1>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate[0] = CAVE_MachE_sil_test_B.xeyeze[0];

  /* SignalConversion generated from: '<S1>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate[1] = CAVE_MachE_sil_test_B.xeyeze[1];

  /* Integrator: '<S230>/phi theta psi' */
  CAVE_MachE_sil_test_B.phithetapsi[0] =
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[0];
  CAVE_MachE_sil_test_B.phithetapsi[1] =
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[1];
  CAVE_MachE_sil_test_B.phithetapsi[2] =
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[2];

  /* SignalConversion generated from: '<S1>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate[2] =
    CAVE_MachE_sil_test_B.phithetapsi[2];

  /* Clock: '<S1>/Clock' */
  CAVE_MachE_sil_test_B.VectorConcatenate[3] = CAVE_MachE_sil_test_M->Timing.t[0];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
  }

  /* Integrator: '<S218>/ub,vb,wb' */
  CAVE_MachE_sil_test_B.ubvbwb[0] = CAVE_MachE_sil_test_X.ubvbwb_CSTATE[0];
  CAVE_MachE_sil_test_B.ubvbwb[1] = CAVE_MachE_sil_test_X.ubvbwb_CSTATE[1];
  CAVE_MachE_sil_test_B.ubvbwb[2] = CAVE_MachE_sil_test_X.ubvbwb_CSTATE[2];

  /* UnitConversion: '<S234>/Unit Conversion' */
  /* Unit Conversion - from: m/s to: m/s
     Expression: output = (1*input) + (0) */
  CAVE_MachE_sil_test_B.UnitConversion[0] = CAVE_MachE_sil_test_B.ubvbwb[0];
  CAVE_MachE_sil_test_B.UnitConversion[1] = CAVE_MachE_sil_test_B.ubvbwb[1];
  CAVE_MachE_sil_test_B.UnitConversion[2] = CAVE_MachE_sil_test_B.ubvbwb[2];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Constant: '<S1>/Constant' */
    CAVE_MachE_sil_test_B.Constant = CAVE_MachE_sil_test_P.Constant_Value_b;

    /* Constant: '<S1>/Constant2' */
    CAVE_MachE_sil_test_B.GearCmd = CAVE_MachE_sil_test_P.Constant2_Value;

    /* Constant: '<S1>/Constant3' */
    CAVE_MachE_sil_test_B.CltchCmd = CAVE_MachE_sil_test_P.Constant3_Value;

    /* Constant: '<S1>/Constant4' */
    CAVE_MachE_sil_test_B.lgSw = CAVE_MachE_sil_test_P.Constant4_Value;

    /* Switch: '<S1>/Switch2' incorporates:
     *  Constant: '<Root>/DriveOption'
     *  Constant: '<Root>/Steer'
     *  Constant: '<S1>/Constant1'
     */
    if (CAVE_MachE_sil_test_P.DriveOption_Value >
        CAVE_MachE_sil_test_P.Switch2_Threshold) {
      CAVE_MachE_sil_test_B.Switch2 = CAVE_MachE_sil_test_P.Steer_Value;
    } else {
      CAVE_MachE_sil_test_B.Switch2 = CAVE_MachE_sil_test_P.Constant1_Value;
    }

    /* End of Switch: '<S1>/Switch2' */

    /* Lookup_n-D: '<S1>/EV Bolt Steer' */
    CAVE_MachE_sil_test_B.SteeringCmd = look1_binlxpw
      (CAVE_MachE_sil_test_B.Switch2, CAVE_MachE_sil_test_P.EVBoltSteer_bp01Data,
       CAVE_MachE_sil_test_P.EVBoltSteer_tableData, 182U);

    /* Constant: '<S6>/Constant' */
    CAVE_MachE_sil_test_B.Constant_i = CAVE_MachE_sil_test_P.EnvPrs;

    /* Constant: '<S6>/Constant1' */
    CAVE_MachE_sil_test_B.Constant1 = CAVE_MachE_sil_test_P.EnvTemp;

    /* Constant: '<S6>/Constant2' */
    CAVE_MachE_sil_test_B.VectorConcatenate_j[0] =
      CAVE_MachE_sil_test_P.Constant2_Value_m;

    /* Constant: '<S6>/Constant3' */
    CAVE_MachE_sil_test_B.VectorConcatenate_j[1] =
      CAVE_MachE_sil_test_P.Constant3_Value_p;

    /* Constant: '<S6>/Constant4' */
    CAVE_MachE_sil_test_B.VectorConcatenate_j[2] =
      CAVE_MachE_sil_test_P.Constant4_Value_e;
  }

  /* Integrator: '<S9>/Integrator' */
  CAVE_MachE_sil_test_B.EnrgyTrans = CAVE_MachE_sil_test_X.Integrator_CSTATE;

  /* UnitConversion: '<S9>/Unit Conversion' */
  /* Unit Conversion - from: J to: kW*h
     Expression: output = (2.77778e-07*input) + (0) */
  CAVE_MachE_sil_test_B.UnitConversion_c = 2.7777777777777776E-7 *
    CAVE_MachE_sil_test_B.EnrgyTrans;

  /* Step: '<S1>/Step' */
  Bias = CAVE_MachE_sil_test_M->Timing.t[0];
  if (Bias < CAVE_MachE_sil_test_P.Step_Time) {
    CAVE_MachE_sil_test_B.AccelCmd = CAVE_MachE_sil_test_P.Step_Y0;
  } else {
    CAVE_MachE_sil_test_B.AccelCmd = CAVE_MachE_sil_test_P.Step_YFinal;
  }

  /* End of Step: '<S1>/Step' */

  /* Switch: '<S1>/Switch' incorporates:
   *  Constant: '<Root>/DriveOption'
   *  Constant: '<Root>/Throttle'
   */
  if (CAVE_MachE_sil_test_P.DriveOption_Value >
      CAVE_MachE_sil_test_P.Switch_Threshold) {
    CAVE_MachE_sil_test_B.AccelCmd_b = CAVE_MachE_sil_test_P.Throttle_Value;
  } else {
    CAVE_MachE_sil_test_B.AccelCmd_b = CAVE_MachE_sil_test_B.AccelCmd;
  }

  /* End of Switch: '<S1>/Switch' */

  /* TransferFcn: '<S120>/Transfer Fcn3' */
  CAVE_MachE_sil_test_B.CarTrq_T2WFL = 0.0;
  CAVE_MachE_sil_test_B.CarTrq_T2WFL += CAVE_MachE_sil_test_P.TransferFcn3_C *
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn1' */
  CAVE_MachE_sil_test_B.CarTrq_T2WFR = 0.0;
  CAVE_MachE_sil_test_B.CarTrq_T2WFR += CAVE_MachE_sil_test_P.TransferFcn1_C *
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn2' */
  CAVE_MachE_sil_test_B.CarTrq_T2WRL = 0.0;
  CAVE_MachE_sil_test_B.CarTrq_T2WRL += CAVE_MachE_sil_test_P.TransferFcn2_C *
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn4' */
  CAVE_MachE_sil_test_B.CarTrq_T2WRR = 0.0;
  CAVE_MachE_sil_test_B.CarTrq_T2WRR += CAVE_MachE_sil_test_P.TransferFcn4_C *
    CAVE_MachE_sil_test_X.TransferFcn4_CSTATE;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S1>/Switch1' incorporates:
     *  Constant: '<Root>/Brake'
     *  Constant: '<Root>/DriveOption'
     *  Constant: '<S1>/Constant6'
     */
    if (CAVE_MachE_sil_test_P.DriveOption_Value >
        CAVE_MachE_sil_test_P.Switch1_Threshold) {
      CAVE_MachE_sil_test_B.DecelCmd = CAVE_MachE_sil_test_P.Brake_Value;
    } else {
      CAVE_MachE_sil_test_B.DecelCmd = CAVE_MachE_sil_test_P.Constant6_Value;
    }

    /* End of Switch: '<S1>/Switch1' */
  }

  /* TransferFcn: '<S120>/Transfer Fcn' */
  CAVE_MachE_sil_test_B.TransferFcn = 0.0;
  CAVE_MachE_sil_test_B.TransferFcn += CAVE_MachE_sil_test_P.TransferFcn_C *
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn9' */
  CAVE_MachE_sil_test_B.TransferFcn9 = 0.0;
  CAVE_MachE_sil_test_B.TransferFcn9 += CAVE_MachE_sil_test_P.TransferFcn9_C *
    CAVE_MachE_sil_test_X.TransferFcn9_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn10' */
  CAVE_MachE_sil_test_B.TransferFcn10 = 0.0;
  CAVE_MachE_sil_test_B.TransferFcn10 += CAVE_MachE_sil_test_P.TransferFcn10_C *
    CAVE_MachE_sil_test_X.TransferFcn10_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn11' */
  CAVE_MachE_sil_test_B.TransferFcn11 = 0.0;
  CAVE_MachE_sil_test_B.TransferFcn11 += CAVE_MachE_sil_test_P.TransferFcn11_C *
    CAVE_MachE_sil_test_X.TransferFcn11_CSTATE;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S30>/Memory' */
    CAVE_MachE_sil_test_B.Memory = CAVE_MachE_sil_test_DW.Memory_PreviousInput;

    /* Gain: '<S30>/Gain3' */
    CAVE_MachE_sil_test_B.Gain3 = CAVE_MachE_sil_test_P.DiffRatio *
      CAVE_MachE_sil_test_B.Memory;

    /* Gain: '<S30>/Gain4' */
    CAVE_MachE_sil_test_B.T_FL = CAVE_MachE_sil_test_P.Gain4_Gain *
      CAVE_MachE_sil_test_B.Gain3;

    /* SignalConversion generated from: '<S30>/Vector Concatenate' */
    CAVE_MachE_sil_test_B.VectorConcatenate_l[0] = CAVE_MachE_sil_test_B.T_FL;

    /* SignalConversion generated from: '<S30>/Vector Concatenate' */
    CAVE_MachE_sil_test_B.VectorConcatenate_l[1] = CAVE_MachE_sil_test_B.T_FL;

    /* SignalConversion generated from: '<S30>/Vector Concatenate' incorporates:
     *  Constant: '<S30>/Constant'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_l[2] =
      CAVE_MachE_sil_test_P.Constant_Value_e;

    /* SignalConversion generated from: '<S30>/Vector Concatenate' incorporates:
     *  Constant: '<S30>/Constant'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_l[3] =
      CAVE_MachE_sil_test_P.Constant_Value_e;

    /* Memory: '<S30>/Memory1' */
    CAVE_MachE_sil_test_B.Memory1 = CAVE_MachE_sil_test_DW.Memory1_PreviousInput;

    /* Abs: '<S39>/Abs' */
    CAVE_MachE_sil_test_B.Abs = fabs(CAVE_MachE_sil_test_B.Memory1);

    /* Product: '<S39>/Divide' incorporates:
     *  Constant: '<S39>/Constant'
     *  Constant: '<S39>/Constant1'
     */
    CAVE_MachE_sil_test_B.Divide = CAVE_MachE_sil_test_P.MtrPwrMax /
      CAVE_MachE_sil_test_P.MtrTrqMax;
  }

  /* Integrator: '<S39>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator = CAVE_MachE_sil_test_X.Integrator_CSTATE_k;

  /* Gain: '<S39>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.MtrTrqTimeCnst;
  CAVE_MachE_sil_test_B.Gain1 = Bias * CAVE_MachE_sil_test_B.Integrator;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* If: '<S39>/If' */
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      rtAction = (int8_T)!(CAVE_MachE_sil_test_B.Abs <=
                           CAVE_MachE_sil_test_B.Divide);
      CAVE_MachE_sil_test_DW.If_ActiveSubsystem = rtAction;
    } else {
      rtAction = CAVE_MachE_sil_test_DW.If_ActiveSubsystem;
    }

    switch (rtAction) {
     case 0:
      /* Outputs for IfAction SubSystem: '<S39>/If Action Subsystem' incorporates:
       *  ActionPort: '<S42>/Action Port'
       */
      /* Saturate: '<S42>/Saturation' */
      Bias = -CAVE_MachE_sil_test_P.MtrTrqMax;
      if (CAVE_MachE_sil_test_B.Gain1 > CAVE_MachE_sil_test_P.MtrTrqMax) {
        CAVE_MachE_sil_test_B.Merge = CAVE_MachE_sil_test_P.MtrTrqMax;
      } else if (CAVE_MachE_sil_test_B.Gain1 < Bias) {
        CAVE_MachE_sil_test_B.Merge = Bias;
      } else {
        CAVE_MachE_sil_test_B.Merge = CAVE_MachE_sil_test_B.Gain1;
      }

      /* End of Saturate: '<S42>/Saturation' */
      /* End of Outputs for SubSystem: '<S39>/If Action Subsystem' */
      break;

     case 1:
      /* Outputs for IfAction SubSystem: '<S39>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S43>/Action Port'
       */
      /* Product: '<S43>/Divide' incorporates:
       *  Constant: '<S43>/Constant'
       */
      CAVE_MachE_sil_test_B.Divide_l4 = CAVE_MachE_sil_test_P.MtrPwrMax /
        CAVE_MachE_sil_test_B.Abs;

      /* Product: '<S43>/Divide1' */
      CAVE_MachE_sil_test_B.Divide1_f = CAVE_MachE_sil_test_B.Gain1 /
        CAVE_MachE_sil_test_B.Divide_l4;

      /* Lookup_n-D: '<S43>/Interpolated zero-crossing' */
      CAVE_MachE_sil_test_B.Interpolatedzerocrossing = look1_binlcpw
        (CAVE_MachE_sil_test_B.Divide1_f,
         CAVE_MachE_sil_test_P.Interpolatedzerocrossing_bp01Data,
         CAVE_MachE_sil_test_P.Interpolatedzerocrossing_tableData, 1U);

      /* Product: '<S43>/Divide2' */
      CAVE_MachE_sil_test_B.Divide2 =
        CAVE_MachE_sil_test_B.Interpolatedzerocrossing *
        CAVE_MachE_sil_test_B.Divide_l4;

      /* SignalConversion: '<S43>/Signal Conversion' */
      CAVE_MachE_sil_test_B.Merge = CAVE_MachE_sil_test_B.Divide2;

      /* End of Outputs for SubSystem: '<S39>/If Action Subsystem1' */
      break;
    }

    /* End of If: '<S39>/If' */
  }

  /* TransferFcn: '<S4>/Transfer Fcn' */
  CAVE_MachE_sil_test_B.VectorConcatenate_p[0] = 0.0;
  CAVE_MachE_sil_test_B.VectorConcatenate_p[0] +=
    CAVE_MachE_sil_test_P.TransferFcn_C_a *
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE_i;

  /* TransferFcn: '<S4>/Transfer Fcn1' */
  CAVE_MachE_sil_test_B.VectorConcatenate_p[1] = 0.0;
  CAVE_MachE_sil_test_B.VectorConcatenate_p[1] +=
    CAVE_MachE_sil_test_P.TransferFcn1_C_a *
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE_i;

  /* TransferFcn: '<S4>/Transfer Fcn2' */
  CAVE_MachE_sil_test_B.VectorConcatenate_p[2] = 0.0;
  CAVE_MachE_sil_test_B.VectorConcatenate_p[2] +=
    CAVE_MachE_sil_test_P.TransferFcn2_C_h *
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE_h;

  /* TransferFcn: '<S4>/Transfer Fcn3' */
  CAVE_MachE_sil_test_B.VectorConcatenate_p[3] = 0.0;
  CAVE_MachE_sil_test_B.VectorConcatenate_p[3] +=
    CAVE_MachE_sil_test_P.TransferFcn3_C_m *
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE_h;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S12>/Memory' */
    CAVE_MachE_sil_test_B.Curr = CAVE_MachE_sil_test_DW.Memory_PreviousInput_f;

    /* Gain: '<S23>/Gain1' */
    Bias = -1.0 / (CAVE_MachE_sil_test_P.BattNumCellsParallel * 3600.0);
    CAVE_MachE_sil_test_B.Gain1_i = Bias * CAVE_MachE_sil_test_B.Curr;

    /* Switch: '<S23>/Switch' incorporates:
     *  Constant: '<S19>/Constant1'
     *  Constant: '<S23>/Constant1'
     */
    Bias = CAVE_MachE_sil_test_P.InitStates[(int32_T)
      CAVE_MachE_sil_test_P.TestScnrioCycleNum - 1].BattSoc *
      CAVE_MachE_sil_test_P.BattChrgCapcty;
    if (Bias > CAVE_MachE_sil_test_P.BattChrgCapcty) {
      CAVE_MachE_sil_test_B.Switch = CAVE_MachE_sil_test_P.BattChrgCapcty;
    } else {
      CAVE_MachE_sil_test_B.Switch = CAVE_MachE_sil_test_P.InitStates[(int32_T)
        CAVE_MachE_sil_test_P.TestScnrioCycleNum - 1].BattSoc *
        CAVE_MachE_sil_test_P.BattChrgCapcty;
    }

    /* End of Switch: '<S23>/Switch' */
  }

  /* Integrator: '<S23>/Integrator Limited' */
  /* Limited  Integrator  */
  if (CAVE_MachE_sil_test_DW.IntegratorLimited_IWORK != 0) {
    CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE =
      CAVE_MachE_sil_test_B.Switch;
  }

  if (CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE >=
      CAVE_MachE_sil_test_P.BattChrgCapcty) {
    CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE =
      CAVE_MachE_sil_test_P.BattChrgCapcty;
  } else {
    if (CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE <=
        CAVE_MachE_sil_test_P.IntegratorLimited_LowerSat) {
      CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE =
        CAVE_MachE_sil_test_P.IntegratorLimited_LowerSat;
    }
  }

  CAVE_MachE_sil_test_B.IntegratorLimited =
    CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE;

  /* End of Integrator: '<S23>/Integrator Limited' */

  /* Product: '<S24>/Divide' incorporates:
   *  Constant: '<S24>/Constant1'
   */
  CAVE_MachE_sil_test_B.Divide_a = CAVE_MachE_sil_test_B.IntegratorLimited /
    CAVE_MachE_sil_test_P.BattChrgCapcty;

  /* Lookup_n-D: '<S25>/Em' */
  CAVE_MachE_sil_test_B.Em = look1_binlcapw(CAVE_MachE_sil_test_B.Divide_a,
    CAVE_MachE_sil_test_P.BattOpenVoltCapBpts,
    CAVE_MachE_sil_test_P.BattOpenVoltTbl, 10U);

  /* Lookup_n-D: '<S25>/R' incorporates:
   *  Constant: '<S16>/Constant'
   */
  CAVE_MachE_sil_test_B.R = look2_binlcapw(CAVE_MachE_sil_test_P.EnvTemp,
    CAVE_MachE_sil_test_B.Divide_a, CAVE_MachE_sil_test_P.BattResistTempBpts,
    CAVE_MachE_sil_test_P.BattResistSocBpts, CAVE_MachE_sil_test_P.BattResistTbl,
    CAVE_MachE_sil_test_P.R_maxIndex, 7U);
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S25>/Gain2' */
    Bias = 1.0 / CAVE_MachE_sil_test_P.BattNumCellsParallel;
    CAVE_MachE_sil_test_B.Gain2 = Bias * CAVE_MachE_sil_test_B.Curr;

    /* Sum: '<S30>/Add' */
    CAVE_MachE_sil_test_B.AxlTrqLump = CAVE_MachE_sil_test_B.T_FL +
      CAVE_MachE_sil_test_B.T_FL;

    /* Reshape: '<S315>/Reshape4' */
    CAVE_MachE_sil_test_B.Reshape4[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_l[0];
    CAVE_MachE_sil_test_B.Reshape4[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_l[1];
    CAVE_MachE_sil_test_B.Reshape4[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_l[2];
    CAVE_MachE_sil_test_B.Reshape4[3] =
      CAVE_MachE_sil_test_B.VectorConcatenate_l[3];

    /* Gain: '<S315>/Gain4' */
    CAVE_MachE_sil_test_B.Gain4_a[0] = CAVE_MachE_sil_test_P.Gain4_Gain_e[0] *
      CAVE_MachE_sil_test_B.Reshape4[0];
    CAVE_MachE_sil_test_B.Gain4_a[1] = CAVE_MachE_sil_test_P.Gain4_Gain_e[1] *
      CAVE_MachE_sil_test_B.Reshape4[1];
    CAVE_MachE_sil_test_B.Gain4_a[2] = CAVE_MachE_sil_test_P.Gain4_Gain_e[2] *
      CAVE_MachE_sil_test_B.Reshape4[2];
    CAVE_MachE_sil_test_B.Gain4_a[3] = CAVE_MachE_sil_test_P.Gain4_Gain_e[3] *
      CAVE_MachE_sil_test_B.Reshape4[3];
  }

  /* Product: '<S25>/Product' */
  CAVE_MachE_sil_test_B.Product = CAVE_MachE_sil_test_B.R *
    CAVE_MachE_sil_test_B.Gain2;

  /* Sum: '<S25>/Subtract' */
  CAVE_MachE_sil_test_B.Subtract = CAVE_MachE_sil_test_B.Em -
    CAVE_MachE_sil_test_B.Product;

  /* Gain: '<S25>/Gain1' */
  CAVE_MachE_sil_test_B.Gain1_c = CAVE_MachE_sil_test_P.BattNumCellsSeries *
    CAVE_MachE_sil_test_B.Subtract;

  /* Product: '<S25>/Product1' */
  CAVE_MachE_sil_test_B.Product1 = CAVE_MachE_sil_test_B.Product *
    CAVE_MachE_sil_test_B.Gain2;

  /* Gain: '<S25>/Gain3' */
  CAVE_MachE_sil_test_B.Gain3_o = CAVE_MachE_sil_test_P.BattNumCellsSeries *
    CAVE_MachE_sil_test_B.Product1;

  /* Gain: '<S25>/Gain4' */
  CAVE_MachE_sil_test_B.Gain4 = CAVE_MachE_sil_test_P.BattNumCellsParallel *
    CAVE_MachE_sil_test_B.Gain3_o;

  /* Product: '<S20>/Product' */
  CAVE_MachE_sil_test_B.Product_b = CAVE_MachE_sil_test_B.Curr *
    CAVE_MachE_sil_test_B.Gain1_c;

  /* Gain: '<S20>/Gain' */
  CAVE_MachE_sil_test_B.Gain = CAVE_MachE_sil_test_P.Gain_Gain *
    CAVE_MachE_sil_test_B.Product_b;

  /* Gain: '<S20>/Gain1' */
  CAVE_MachE_sil_test_B.Gain1_e = CAVE_MachE_sil_test_P.Gain1_Gain *
    CAVE_MachE_sil_test_B.Gain4;

  /* Sum: '<S20>/Add' */
  CAVE_MachE_sil_test_B.Add = CAVE_MachE_sil_test_B.Gain +
    CAVE_MachE_sil_test_B.Gain1_e;

  /* Integrator: '<S350>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_k = CAVE_MachE_sil_test_X.Integrator_CSTATE_p;

  /* Integrator: '<S347>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_a = CAVE_MachE_sil_test_X.Integrator_CSTATE_o;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* SignalConversion generated from: '<S120>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Ground Z Level'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_g[0] =
      CAVE_MachE_sil_test_P.GroundZLevel_Value[0];

    /* SignalConversion generated from: '<S120>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Ground Z Level'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_g[1] =
      CAVE_MachE_sil_test_P.GroundZLevel_Value[1];

    /* SignalConversion generated from: '<S120>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Ground Z Level'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_g[2] =
      CAVE_MachE_sil_test_P.GroundZLevel_Value[2];

    /* SignalConversion generated from: '<S120>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Ground Z Level'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate_g[3] =
      CAVE_MachE_sil_test_P.GroundZLevel_Value[3];
  }

  /* SecondOrderIntegrator: '<S345>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1 =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2 =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE[1];

  /* Sum: '<S345>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6 = CAVE_MachE_sil_test_B.VectorConcatenate_g[0] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1;

  /* Saturate: '<S345>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6 > CAVE_MachE_sil_test_P.Saturation_UpperSat) {
    CAVE_MachE_sil_test_B.Saturation = CAVE_MachE_sil_test_P.Saturation_UpperSat;
  } else if (CAVE_MachE_sil_test_B.Sum6 <
             CAVE_MachE_sil_test_P.Saturation_LowerSat) {
    CAVE_MachE_sil_test_B.Saturation = CAVE_MachE_sil_test_P.Saturation_LowerSat;
  } else {
    CAVE_MachE_sil_test_B.Saturation = CAVE_MachE_sil_test_B.Sum6;
  }

  /* End of Saturate: '<S345>/Saturation' */

  /* Sum: '<S346>/Add2' incorporates:
   *  Constant: '<S346>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2 =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_UNLOADED_RADIUS -
    CAVE_MachE_sil_test_B.Saturation;

  /* Product: '<S346>/Product3' */
  CAVE_MachE_sil_test_B.Product3 = CAVE_MachE_sil_test_B.Integrator_a *
    CAVE_MachE_sil_test_B.Add2;

  /* Sum: '<S346>/Add1' */
  CAVE_MachE_sil_test_B.Add1 = (CAVE_MachE_sil_test_B.Product3 -
    CAVE_MachE_sil_test_B.Gain4_a[0]) - CAVE_MachE_sil_test_B.Integrator_k;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Saturate: '<S54>/Saturation' */
    if (CAVE_MachE_sil_test_B.DecelCmd >
        CAVE_MachE_sil_test_P.Saturation_UpperSat_a) {
      CAVE_MachE_sil_test_B.Saturation_f =
        CAVE_MachE_sil_test_P.Saturation_UpperSat_a;
    } else if (CAVE_MachE_sil_test_B.DecelCmd <
               CAVE_MachE_sil_test_P.Saturation_LowerSat_h) {
      CAVE_MachE_sil_test_B.Saturation_f =
        CAVE_MachE_sil_test_P.Saturation_LowerSat_h;
    } else {
      CAVE_MachE_sil_test_B.Saturation_f = CAVE_MachE_sil_test_B.DecelCmd;
    }

    /* End of Saturate: '<S54>/Saturation' */

    /* Gain: '<S54>/Gain1' */
    CAVE_MachE_sil_test_B.BrkTrqReqTotal = CAVE_MachE_sil_test_P.BrakeMaxTrq *
      CAVE_MachE_sil_test_B.Saturation_f;

    /* Saturate: '<S70>/Saturation1' */
    if (CAVE_MachE_sil_test_B.Memory1 >
        CAVE_MachE_sil_test_P.Saturation1_UpperSat) {
      CAVE_MachE_sil_test_B.Saturation1 =
        CAVE_MachE_sil_test_P.Saturation1_UpperSat;
    } else if (CAVE_MachE_sil_test_B.Memory1 <
               CAVE_MachE_sil_test_P.Saturation1_LowerSat) {
      CAVE_MachE_sil_test_B.Saturation1 =
        CAVE_MachE_sil_test_P.Saturation1_LowerSat;
    } else {
      CAVE_MachE_sil_test_B.Saturation1 = CAVE_MachE_sil_test_B.Memory1;
    }

    /* End of Saturate: '<S70>/Saturation1' */

    /* Product: '<S70>/Divide' incorporates:
     *  Constant: '<S70>/Constant'
     */
    CAVE_MachE_sil_test_B.Divide_m = CAVE_MachE_sil_test_P.MtrPwrMax /
      CAVE_MachE_sil_test_B.Saturation1;

    /* Saturate: '<S70>/Saturation' */
    if (CAVE_MachE_sil_test_B.Divide_m > CAVE_MachE_sil_test_P.MtrTrqMax) {
      CAVE_MachE_sil_test_B.Saturation_n = CAVE_MachE_sil_test_P.MtrTrqMax;
    } else if (CAVE_MachE_sil_test_B.Divide_m <
               CAVE_MachE_sil_test_P.Saturation_LowerSat_hi) {
      CAVE_MachE_sil_test_B.Saturation_n =
        CAVE_MachE_sil_test_P.Saturation_LowerSat_hi;
    } else {
      CAVE_MachE_sil_test_B.Saturation_n = CAVE_MachE_sil_test_B.Divide_m;
    }

    /* End of Saturate: '<S70>/Saturation' */

    /* Gain: '<S54>/MotTrqReflectedToWheels' */
    CAVE_MachE_sil_test_B.MotTrqMaxWhls = CAVE_MachE_sil_test_P.DiffRatio *
      CAVE_MachE_sil_test_B.Saturation_n;

    /* MinMax: '<S54>/MinMax' */
    Bias = fmin(CAVE_MachE_sil_test_B.BrkTrqReqTotal,
                CAVE_MachE_sil_test_B.MotTrqMaxWhls);
    CAVE_MachE_sil_test_B.min = Bias;
  }

  /* Lookup_n-D: '<S54>/RegenBrakingCutoff' */
  CAVE_MachE_sil_test_B.RegenBrakingCutoff = look1_binlcapw
    (CAVE_MachE_sil_test_B.UnitConversion[0],
     CAVE_MachE_sil_test_P.SupvsryCtrlRegenSpdBpts,
     CAVE_MachE_sil_test_P.SupvsryCtrlRegenBrkCutOffTbl, 1U);

  /* Lookup_n-D: '<S54>/ChrgLmt' */
  CAVE_MachE_sil_test_B.ChrgLmt = look1_binlcapw(CAVE_MachE_sil_test_B.Divide_a,
    CAVE_MachE_sil_test_P.BattChargeLimitSocBpts,
    CAVE_MachE_sil_test_P.BattChargeLimitTbl, 11U);

  /* Product: '<S54>/Product1' */
  CAVE_MachE_sil_test_B.RegenFactor = CAVE_MachE_sil_test_B.RegenBrakingCutoff *
    CAVE_MachE_sil_test_B.ChrgLmt;

  /* Product: '<S54>/Product3' */
  CAVE_MachE_sil_test_B.MotTrqRegenWhl = CAVE_MachE_sil_test_B.min *
    CAVE_MachE_sil_test_B.RegenFactor;

  /* Sum: '<S54>/Subtract' */
  CAVE_MachE_sil_test_B.Subtract_k = CAVE_MachE_sil_test_B.BrkTrqReqTotal -
    CAVE_MachE_sil_test_B.MotTrqRegenWhl;

  /* Gain: '<S54>/Gain2' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.BrakeMaxTrq;
  CAVE_MachE_sil_test_B.BrkTrqReqTotal_o = Bias *
    CAVE_MachE_sil_test_B.Subtract_k;

  /* Saturate: '<S54>/Saturation1' */
  if (CAVE_MachE_sil_test_B.BrkTrqReqTotal_o >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_a) {
    CAVE_MachE_sil_test_B.Saturation1_g =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_a;
  } else if (CAVE_MachE_sil_test_B.BrkTrqReqTotal_o <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_l) {
    CAVE_MachE_sil_test_B.Saturation1_g =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_l;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_g = CAVE_MachE_sil_test_B.BrkTrqReqTotal_o;
  }

  /* End of Saturate: '<S54>/Saturation1' */

  /* Gain: '<S121>/Gain' */
  CAVE_MachE_sil_test_B.Gain_j = CAVE_MachE_sil_test_P.BrakeMaxPrs *
    CAVE_MachE_sil_test_B.Saturation1_g;

  /* Gain: '<S121>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_p = CAVE_MachE_sil_test_P.Gain2_Gain *
    CAVE_MachE_sil_test_B.Gain_j;

  /* Gain: '<S121>/Gain3' */
  CAVE_MachE_sil_test_B.Gain3_n = CAVE_MachE_sil_test_P.Gain3_Gain *
    CAVE_MachE_sil_test_B.Gain2_p;

  /* Saturate: '<S121>/Saturation' */
  if (CAVE_MachE_sil_test_B.Gain3_n > CAVE_MachE_sil_test_P.BrakeMaxPrs) {
    Bias = CAVE_MachE_sil_test_P.BrakeMaxPrs;
  } else if (CAVE_MachE_sil_test_B.Gain3_n <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_j) {
    Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_j;
  } else {
    Bias = CAVE_MachE_sil_test_B.Gain3_n;
  }

  CAVE_MachE_sil_test_B.VectorConcatenate2[0] = Bias;

  /* End of Saturate: '<S121>/Saturation' */

  /* Saturate: '<S121>/Saturation1' */
  if (CAVE_MachE_sil_test_B.Gain3_n > CAVE_MachE_sil_test_P.BrakeMaxPrs) {
    Bias = CAVE_MachE_sil_test_P.BrakeMaxPrs;
  } else if (CAVE_MachE_sil_test_B.Gain3_n <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_i) {
    Bias = CAVE_MachE_sil_test_P.Saturation1_LowerSat_i;
  } else {
    Bias = CAVE_MachE_sil_test_B.Gain3_n;
  }

  CAVE_MachE_sil_test_B.VectorConcatenate2[1] = Bias;

  /* End of Saturate: '<S121>/Saturation1' */

  /* Gain: '<S121>/Gain1' */
  CAVE_MachE_sil_test_B.Gain1_n = CAVE_MachE_sil_test_P.Gain1_Gain_e *
    CAVE_MachE_sil_test_B.Gain_j;

  /* Gain: '<S121>/Gain4' */
  CAVE_MachE_sil_test_B.Gain4_f = CAVE_MachE_sil_test_P.Gain4_Gain_h *
    CAVE_MachE_sil_test_B.Gain1_n;

  /* Saturate: '<S121>/Saturation2' */
  if (CAVE_MachE_sil_test_B.Gain4_f > CAVE_MachE_sil_test_P.BrakeMaxPrs) {
    Bias = CAVE_MachE_sil_test_P.BrakeMaxPrs;
  } else if (CAVE_MachE_sil_test_B.Gain4_f <
             CAVE_MachE_sil_test_P.Saturation2_LowerSat) {
    Bias = CAVE_MachE_sil_test_P.Saturation2_LowerSat;
  } else {
    Bias = CAVE_MachE_sil_test_B.Gain4_f;
  }

  CAVE_MachE_sil_test_B.VectorConcatenate2[2] = Bias;

  /* End of Saturate: '<S121>/Saturation2' */

  /* Saturate: '<S121>/Saturation3' */
  if (CAVE_MachE_sil_test_B.Gain4_f > CAVE_MachE_sil_test_P.BrakeMaxPrs) {
    Bias = CAVE_MachE_sil_test_P.BrakeMaxPrs;
  } else if (CAVE_MachE_sil_test_B.Gain4_f <
             CAVE_MachE_sil_test_P.Saturation3_LowerSat) {
    Bias = CAVE_MachE_sil_test_P.Saturation3_LowerSat;
  } else {
    Bias = CAVE_MachE_sil_test_B.Gain4_f;
  }

  CAVE_MachE_sil_test_B.VectorConcatenate2[3] = Bias;

  /* End of Saturate: '<S121>/Saturation3' */

  /* Reshape: '<S315>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3[0] = CAVE_MachE_sil_test_B.VectorConcatenate2[0];
  CAVE_MachE_sil_test_B.Reshape3[1] = CAVE_MachE_sil_test_B.VectorConcatenate2[1];
  CAVE_MachE_sil_test_B.Reshape3[2] = CAVE_MachE_sil_test_B.VectorConcatenate2[2];
  CAVE_MachE_sil_test_B.Reshape3[3] = CAVE_MachE_sil_test_B.VectorConcatenate2[3];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S355>/Torque Conversion1' incorporates:
     *  Constant: '<S355>/Disk brake actuator bore'
     */
    CAVE_MachE_sil_test_B.TorqueConversion1 =
      CAVE_MachE_sil_test_P.TorqueConversion1_Gain *
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_disk_abore;
  }

  /* Product: '<S355>/product' incorporates:
   *  Constant: '<S355>/Disk brake actuator bore'
   *  Constant: '<S355>/Number of brake pads'
   */
  CAVE_MachE_sil_test_B.product = CAVE_MachE_sil_test_B.Reshape3[0] *
    CAVE_MachE_sil_test_B.TorqueConversion1 *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_disk_abore *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_num_pads;

  /* Saturate: '<S355>/Disallow Negative Brake Torque' */
  if (CAVE_MachE_sil_test_B.product >
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat;
  } else if (CAVE_MachE_sil_test_B.product <
             CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat;
  } else {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque =
      CAVE_MachE_sil_test_B.product;
  }

  /* End of Saturate: '<S355>/Disallow Negative Brake Torque' */

  /* Gain: '<S355>/Torque Conversion' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Rm *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_mu_kinetic;
  CAVE_MachE_sil_test_B.TorqueConversion = Bias *
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque;

  /* Gain: '<S352>/Ratio of static to kinetic' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_mu_static /
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_mu_kinetic;
  CAVE_MachE_sil_test_B.Ratioofstatictokinetic = Bias *
    CAVE_MachE_sil_test_B.TorqueConversion;

  /* Chart: '<S346>/LockUp' */
  CAVE_MachE_sil_test_LockUp(CAVE_MachE_sil_test_M, CAVE_MachE_sil_test_B.Add1,
    CAVE_MachE_sil_test_B.Ratioofstatictokinetic,
    CAVE_MachE_sil_test_B.TorqueConversion, &CAVE_MachE_sil_test_B.sf_LockUp,
    &CAVE_MachE_sil_test_DW.sf_LockUp, &CAVE_MachE_sil_test_P.sf_LockUp,
    &CAVE_MachE_sil_test_X.sf_LockUp,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_omegao,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_br,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_IYY,
    CAVE_MachE_sil_test_P.LockUp_OmegaTol);

  /* SignalConversion generated from: '<S340>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_h[0] =
    CAVE_MachE_sil_test_B.sf_LockUp.Omega;

  /* Integrator: '<S375>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_f = CAVE_MachE_sil_test_X.Integrator_CSTATE_c;

  /* Integrator: '<S372>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_l =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_ch;

  /* SecondOrderIntegrator: '<S370>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_a =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_l[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_n =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_l[1];

  /* Sum: '<S370>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_p = CAVE_MachE_sil_test_B.VectorConcatenate_g[1] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_a;

  /* Saturate: '<S370>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_p > CAVE_MachE_sil_test_P.Saturation_UpperSat_e)
  {
    CAVE_MachE_sil_test_B.Saturation_d =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_e;
  } else if (CAVE_MachE_sil_test_B.Sum6_p <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_p) {
    CAVE_MachE_sil_test_B.Saturation_d =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_p;
  } else {
    CAVE_MachE_sil_test_B.Saturation_d = CAVE_MachE_sil_test_B.Sum6_p;
  }

  /* End of Saturate: '<S370>/Saturation' */

  /* Sum: '<S371>/Add2' incorporates:
   *  Constant: '<S371>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_l =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_UNLOADED_RADIUS -
    CAVE_MachE_sil_test_B.Saturation_d;

  /* Product: '<S371>/Product3' */
  CAVE_MachE_sil_test_B.Product3_h = CAVE_MachE_sil_test_B.Integrator_l *
    CAVE_MachE_sil_test_B.Add2_l;

  /* Sum: '<S371>/Add1' */
  CAVE_MachE_sil_test_B.Add1_g = (CAVE_MachE_sil_test_B.Product3_h -
    CAVE_MachE_sil_test_B.Gain4_a[1]) - CAVE_MachE_sil_test_B.Integrator_f;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S380>/Torque Conversion1' incorporates:
     *  Constant: '<S380>/Disk brake actuator bore'
     */
    CAVE_MachE_sil_test_B.TorqueConversion1_e =
      CAVE_MachE_sil_test_P.TorqueConversion1_Gain_p *
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_disk_abore;
  }

  /* Product: '<S380>/product' incorporates:
   *  Constant: '<S380>/Disk brake actuator bore'
   *  Constant: '<S380>/Number of brake pads'
   */
  CAVE_MachE_sil_test_B.product_l = CAVE_MachE_sil_test_B.Reshape3[1] *
    CAVE_MachE_sil_test_B.TorqueConversion1_e *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_disk_abore *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_num_pads;

  /* Saturate: '<S380>/Disallow Negative Brake Torque' */
  if (CAVE_MachE_sil_test_B.product_l >
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_k) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_c =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_k;
  } else if (CAVE_MachE_sil_test_B.product_l <
             CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_f) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_c =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_f;
  } else {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_c =
      CAVE_MachE_sil_test_B.product_l;
  }

  /* End of Saturate: '<S380>/Disallow Negative Brake Torque' */

  /* Gain: '<S380>/Torque Conversion' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Rm *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_mu_kinetic;
  CAVE_MachE_sil_test_B.TorqueConversion_p = Bias *
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_c;

  /* Gain: '<S377>/Ratio of static to kinetic' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_mu_static /
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_mu_kinetic;
  CAVE_MachE_sil_test_B.Ratioofstatictokinetic_n = Bias *
    CAVE_MachE_sil_test_B.TorqueConversion_p;

  /* Chart: '<S371>/LockUp' */
  CAVE_MachE_sil_test_LockUp(CAVE_MachE_sil_test_M, CAVE_MachE_sil_test_B.Add1_g,
    CAVE_MachE_sil_test_B.Ratioofstatictokinetic_n,
    CAVE_MachE_sil_test_B.TorqueConversion_p, &CAVE_MachE_sil_test_B.sf_LockUp_n,
    &CAVE_MachE_sil_test_DW.sf_LockUp_n, &CAVE_MachE_sil_test_P.sf_LockUp_n,
    &CAVE_MachE_sil_test_X.sf_LockUp_n,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_omegao,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_br,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_IYY,
    CAVE_MachE_sil_test_P.LockUp_OmegaTol_p);

  /* SignalConversion generated from: '<S340>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_h[1] =
    CAVE_MachE_sil_test_B.sf_LockUp_n.Omega;

  /* Integrator: '<S400>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_ah =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_d;

  /* Integrator: '<S397>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_fm =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_b;

  /* SecondOrderIntegrator: '<S395>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_o =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_ln[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_p =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_ln[1];

  /* Sum: '<S395>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_f = CAVE_MachE_sil_test_B.VectorConcatenate_g[2] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_o;

  /* Saturate: '<S395>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_f > CAVE_MachE_sil_test_P.Saturation_UpperSat_b)
  {
    CAVE_MachE_sil_test_B.Saturation_j =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_b;
  } else if (CAVE_MachE_sil_test_B.Sum6_f <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_e) {
    CAVE_MachE_sil_test_B.Saturation_j =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_e;
  } else {
    CAVE_MachE_sil_test_B.Saturation_j = CAVE_MachE_sil_test_B.Sum6_f;
  }

  /* End of Saturate: '<S395>/Saturation' */

  /* Sum: '<S396>/Add2' incorporates:
   *  Constant: '<S396>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_e =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_UNLOADED_RADIUS -
    CAVE_MachE_sil_test_B.Saturation_j;

  /* Product: '<S396>/Product3' */
  CAVE_MachE_sil_test_B.Product3_l = CAVE_MachE_sil_test_B.Integrator_fm *
    CAVE_MachE_sil_test_B.Add2_e;

  /* Sum: '<S396>/Add1' */
  CAVE_MachE_sil_test_B.Add1_k = (CAVE_MachE_sil_test_B.Product3_l -
    CAVE_MachE_sil_test_B.Gain4_a[2]) - CAVE_MachE_sil_test_B.Integrator_ah;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S405>/Torque Conversion1' incorporates:
     *  Constant: '<S405>/Disk brake actuator bore'
     */
    CAVE_MachE_sil_test_B.TorqueConversion1_b =
      CAVE_MachE_sil_test_P.TorqueConversion1_Gain_b *
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_disk_abore;
  }

  /* Product: '<S405>/product' incorporates:
   *  Constant: '<S405>/Disk brake actuator bore'
   *  Constant: '<S405>/Number of brake pads'
   */
  CAVE_MachE_sil_test_B.product_c = CAVE_MachE_sil_test_B.Reshape3[2] *
    CAVE_MachE_sil_test_B.TorqueConversion1_b *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_disk_abore *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_num_pads;

  /* Saturate: '<S405>/Disallow Negative Brake Torque' */
  if (CAVE_MachE_sil_test_B.product_c >
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_i) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_d =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_i;
  } else if (CAVE_MachE_sil_test_B.product_c <
             CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_i) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_d =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_i;
  } else {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_d =
      CAVE_MachE_sil_test_B.product_c;
  }

  /* End of Saturate: '<S405>/Disallow Negative Brake Torque' */

  /* Gain: '<S405>/Torque Conversion' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Rm *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_mu_kinetic;
  CAVE_MachE_sil_test_B.TorqueConversion_e = Bias *
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_d;

  /* Gain: '<S402>/Ratio of static to kinetic' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_mu_static /
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_mu_kinetic;
  CAVE_MachE_sil_test_B.Ratioofstatictokinetic_c = Bias *
    CAVE_MachE_sil_test_B.TorqueConversion_e;

  /* Chart: '<S396>/LockUp' */
  CAVE_MachE_sil_test_LockUp(CAVE_MachE_sil_test_M, CAVE_MachE_sil_test_B.Add1_k,
    CAVE_MachE_sil_test_B.Ratioofstatictokinetic_c,
    CAVE_MachE_sil_test_B.TorqueConversion_e, &CAVE_MachE_sil_test_B.sf_LockUp_h,
    &CAVE_MachE_sil_test_DW.sf_LockUp_h, &CAVE_MachE_sil_test_P.sf_LockUp_h,
    &CAVE_MachE_sil_test_X.sf_LockUp_h,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_omegao,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_br,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_IYY,
    CAVE_MachE_sil_test_P.LockUp_OmegaTol_l);

  /* SignalConversion generated from: '<S340>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_h[2] =
    CAVE_MachE_sil_test_B.sf_LockUp_h.Omega;

  /* Integrator: '<S425>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_p = CAVE_MachE_sil_test_X.Integrator_CSTATE_a;

  /* Integrator: '<S422>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_b = CAVE_MachE_sil_test_X.Integrator_CSTATE_g;

  /* SecondOrderIntegrator: '<S420>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_h =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_j[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_h =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_j[1];

  /* Sum: '<S420>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_o = CAVE_MachE_sil_test_B.VectorConcatenate_g[3] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_h;

  /* Saturate: '<S420>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_o > CAVE_MachE_sil_test_P.Saturation_UpperSat_d)
  {
    CAVE_MachE_sil_test_B.Saturation_nz =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_d;
  } else if (CAVE_MachE_sil_test_B.Sum6_o <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_o) {
    CAVE_MachE_sil_test_B.Saturation_nz =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_o;
  } else {
    CAVE_MachE_sil_test_B.Saturation_nz = CAVE_MachE_sil_test_B.Sum6_o;
  }

  /* End of Saturate: '<S420>/Saturation' */

  /* Sum: '<S421>/Add2' incorporates:
   *  Constant: '<S421>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_d =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_UNLOADED_RADIUS -
    CAVE_MachE_sil_test_B.Saturation_nz;

  /* Product: '<S421>/Product3' */
  CAVE_MachE_sil_test_B.Product3_i = CAVE_MachE_sil_test_B.Integrator_b *
    CAVE_MachE_sil_test_B.Add2_d;

  /* Sum: '<S421>/Add1' */
  CAVE_MachE_sil_test_B.Add1_ge = (CAVE_MachE_sil_test_B.Product3_i -
    CAVE_MachE_sil_test_B.Gain4_a[3]) - CAVE_MachE_sil_test_B.Integrator_p;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S430>/Torque Conversion1' incorporates:
     *  Constant: '<S430>/Disk brake actuator bore'
     */
    CAVE_MachE_sil_test_B.TorqueConversion1_g =
      CAVE_MachE_sil_test_P.TorqueConversion1_Gain_n *
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_disk_abore;
  }

  /* Product: '<S430>/product' incorporates:
   *  Constant: '<S430>/Disk brake actuator bore'
   *  Constant: '<S430>/Number of brake pads'
   */
  CAVE_MachE_sil_test_B.product_m = CAVE_MachE_sil_test_B.Reshape3[3] *
    CAVE_MachE_sil_test_B.TorqueConversion1_g *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_disk_abore *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_num_pads;

  /* Saturate: '<S430>/Disallow Negative Brake Torque' */
  if (CAVE_MachE_sil_test_B.product_m >
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_c) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_a =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_c;
  } else if (CAVE_MachE_sil_test_B.product_m <
             CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_k) {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_a =
      CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_LowerSat_k;
  } else {
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_a =
      CAVE_MachE_sil_test_B.product_m;
  }

  /* End of Saturate: '<S430>/Disallow Negative Brake Torque' */

  /* Gain: '<S430>/Torque Conversion' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Rm *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_mu_kinetic;
  CAVE_MachE_sil_test_B.TorqueConversion_n = Bias *
    CAVE_MachE_sil_test_B.DisallowNegativeBrakeTorque_a;

  /* Gain: '<S427>/Ratio of static to kinetic' */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_mu_static /
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_mu_kinetic;
  CAVE_MachE_sil_test_B.Ratioofstatictokinetic_b = Bias *
    CAVE_MachE_sil_test_B.TorqueConversion_n;

  /* Chart: '<S421>/LockUp' */
  CAVE_MachE_sil_test_LockUp(CAVE_MachE_sil_test_M,
    CAVE_MachE_sil_test_B.Add1_ge,
    CAVE_MachE_sil_test_B.Ratioofstatictokinetic_b,
    CAVE_MachE_sil_test_B.TorqueConversion_n, &CAVE_MachE_sil_test_B.sf_LockUp_c,
    &CAVE_MachE_sil_test_DW.sf_LockUp_c, &CAVE_MachE_sil_test_P.sf_LockUp_c,
    &CAVE_MachE_sil_test_X.sf_LockUp_c,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_omegao,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_br,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_IYY,
    CAVE_MachE_sil_test_P.LockUp_OmegaTol_d);

  /* SignalConversion generated from: '<S340>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_h[3] =
    CAVE_MachE_sil_test_B.sf_LockUp_c.Omega;

  /* MultiPortSwitch: '<S4>/Multiport Switch' incorporates:
   *  Constant: '<S4>/VehicleSimulationType'
   */
  if ((int32_T)CAVE_MachE_sil_test_P.VehicleSimulationType_Value == 0) {
    CAVE_MachE_sil_test_B.MultiportSwitch[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_h[0];
    CAVE_MachE_sil_test_B.MultiportSwitch[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_h[1];
    CAVE_MachE_sil_test_B.MultiportSwitch[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_h[2];
    CAVE_MachE_sil_test_B.MultiportSwitch[3] =
      CAVE_MachE_sil_test_B.VectorConcatenate_h[3];
  } else {
    CAVE_MachE_sil_test_B.MultiportSwitch[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_p[0];
    CAVE_MachE_sil_test_B.MultiportSwitch[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_p[1];
    CAVE_MachE_sil_test_B.MultiportSwitch[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_p[2];
    CAVE_MachE_sil_test_B.MultiportSwitch[3] =
      CAVE_MachE_sil_test_B.VectorConcatenate_p[3];
  }

  /* End of MultiPortSwitch: '<S4>/Multiport Switch' */

  /* Sum: '<S30>/Add1' */
  CAVE_MachE_sil_test_B.Add1_j = CAVE_MachE_sil_test_B.MultiportSwitch[0] +
    CAVE_MachE_sil_test_B.MultiportSwitch[1];

  /* Gain: '<S30>/Gain1' */
  CAVE_MachE_sil_test_B.Gain1_k = CAVE_MachE_sil_test_P.Gain1_Gain_p *
    CAVE_MachE_sil_test_B.Add1_j;

  /* Gain: '<S30>/Gain2' */
  CAVE_MachE_sil_test_B.Spd = CAVE_MachE_sil_test_P.DiffRatio *
    CAVE_MachE_sil_test_B.Gain1_k;

  /* Gain: '<S4>/Gain1' */
  CAVE_MachE_sil_test_B.VehSpdKph = CAVE_MachE_sil_test_P.Gain1_Gain_f *
    CAVE_MachE_sil_test_B.UnitConversion[0];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Product: '<S31>/Transferred Power' */
    CAVE_MachE_sil_test_B.TransferredPower = CAVE_MachE_sil_test_B.Merge *
      CAVE_MachE_sil_test_B.Memory1;

    /* SignalConversion generated from: '<S31>/Vector Concatenate' */
    CAVE_MachE_sil_test_B.VectorConcatenate_e[0] =
      CAVE_MachE_sil_test_B.TransferredPower;

    /* SignalConversion generated from: '<S31>/Vector Concatenate' */
    CAVE_MachE_sil_test_B.VectorConcatenate_e[1] =
      CAVE_MachE_sil_test_B.TransferredPower;

    /* SignalConversion generated from: '<S31>/Vector Concatenate' */
    CAVE_MachE_sil_test_B.VectorConcatenate_e[2] =
      CAVE_MachE_sil_test_B.TransferredPower;

    /* Constant: '<S31>/Constant' */
    CAVE_MachE_sil_test_B.PwrMechLoss = CAVE_MachE_sil_test_P.Constant_Value_dv;

    /* Constant: '<S31>/Constant1' */
    CAVE_MachE_sil_test_B.PwrDampLoss = CAVE_MachE_sil_test_P.Constant1_Value_i;

    /* Constant: '<S31>/Constant2' */
    CAVE_MachE_sil_test_B.PwrStoredShft =
      CAVE_MachE_sil_test_P.Constant2_Value_o;

    /* Product: '<S41>/Product' */
    CAVE_MachE_sil_test_B.Product_o = CAVE_MachE_sil_test_B.Memory1 *
      CAVE_MachE_sil_test_B.Merge;

    /* Lookup_n-D: '<S41>/2-D Lookup Table' */
    CAVE_MachE_sil_test_B.uDLookupTable = look2_binlcapw
      (CAVE_MachE_sil_test_B.Memory1, CAVE_MachE_sil_test_B.Merge,
       CAVE_MachE_sil_test_P.uDLookupTable_bp01Data,
       CAVE_MachE_sil_test_P.uDLookupTable_bp02Data,
       CAVE_MachE_sil_test_P.uDLookupTable_tableData,
       CAVE_MachE_sil_test_P.uDLookupTable_maxIndex, 23U);

    /* Sum: '<S38>/Add' */
    CAVE_MachE_sil_test_B.Add_d = CAVE_MachE_sil_test_B.Product_o +
      CAVE_MachE_sil_test_B.uDLookupTable;
  }

  /* Saturate: '<S38>/Saturation' */
  if (CAVE_MachE_sil_test_B.Gain1_c >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_h) {
    CAVE_MachE_sil_test_B.Saturation_g =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_h;
  } else if (CAVE_MachE_sil_test_B.Gain1_c <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_oa) {
    CAVE_MachE_sil_test_B.Saturation_g =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_oa;
  } else {
    CAVE_MachE_sil_test_B.Saturation_g = CAVE_MachE_sil_test_B.Gain1_c;
  }

  /* End of Saturate: '<S38>/Saturation' */

  /* Product: '<S38>/Divide' */
  CAVE_MachE_sil_test_B.Divide_f = CAVE_MachE_sil_test_B.Add_d /
    CAVE_MachE_sil_test_B.Saturation_g;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Saturate: '<S66>/Saturation1' */
    if (CAVE_MachE_sil_test_B.Memory1 >
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_l) {
      CAVE_MachE_sil_test_B.Saturation1_d =
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_l;
    } else if (CAVE_MachE_sil_test_B.Memory1 <
               CAVE_MachE_sil_test_P.Saturation1_LowerSat_j) {
      CAVE_MachE_sil_test_B.Saturation1_d =
        CAVE_MachE_sil_test_P.Saturation1_LowerSat_j;
    } else {
      CAVE_MachE_sil_test_B.Saturation1_d = CAVE_MachE_sil_test_B.Memory1;
    }

    /* End of Saturate: '<S66>/Saturation1' */

    /* Product: '<S66>/Divide' incorporates:
     *  Constant: '<S66>/Constant'
     */
    CAVE_MachE_sil_test_B.Divide_o = CAVE_MachE_sil_test_P.MtrPwrMax /
      CAVE_MachE_sil_test_B.Saturation1_d;

    /* Saturate: '<S66>/Saturation' */
    if (CAVE_MachE_sil_test_B.Divide_o > CAVE_MachE_sil_test_P.MtrTrqMax) {
      CAVE_MachE_sil_test_B.Saturation_l = CAVE_MachE_sil_test_P.MtrTrqMax;
    } else if (CAVE_MachE_sil_test_B.Divide_o <
               CAVE_MachE_sil_test_P.Saturation_LowerSat_ej) {
      CAVE_MachE_sil_test_B.Saturation_l =
        CAVE_MachE_sil_test_P.Saturation_LowerSat_ej;
    } else {
      CAVE_MachE_sil_test_B.Saturation_l = CAVE_MachE_sil_test_B.Divide_o;
    }

    /* End of Saturate: '<S66>/Saturation' */
  }

  /* Saturate: '<S55>/Saturation1' */
  Bias = CAVE_MachE_sil_test_B.UnitConversion[0];
  if (Bias > CAVE_MachE_sil_test_P.Saturation1_UpperSat_ak) {
    CAVE_MachE_sil_test_B.Saturation1_a =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_ak;
  } else if (Bias < CAVE_MachE_sil_test_P.Saturation1_LowerSat_c) {
    CAVE_MachE_sil_test_B.Saturation1_a =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_c;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_a = Bias;
  }

  /* End of Saturate: '<S55>/Saturation1' */

  /* Product: '<S55>/Divide' incorporates:
   *  Constant: '<S55>/Constant'
   */
  CAVE_MachE_sil_test_B.Divide_l = CAVE_MachE_sil_test_P.MtrPwrMax /
    CAVE_MachE_sil_test_B.Saturation1_a;

  /* Saturate: '<S55>/Saturation' */
  if (CAVE_MachE_sil_test_B.Divide_l > CAVE_MachE_sil_test_P.MtrTrqMax) {
    CAVE_MachE_sil_test_B.Saturation_h = CAVE_MachE_sil_test_P.MtrTrqMax;
  } else if (CAVE_MachE_sil_test_B.Divide_l <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_p0) {
    CAVE_MachE_sil_test_B.Saturation_h =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_p0;
  } else {
    CAVE_MachE_sil_test_B.Saturation_h = CAVE_MachE_sil_test_B.Divide_l;
  }

  /* End of Saturate: '<S55>/Saturation' */

  /* Product: '<S51>/Product1' */
  CAVE_MachE_sil_test_B.Product1_g = CAVE_MachE_sil_test_B.AccelCmd_b *
    CAVE_MachE_sil_test_B.Saturation_h;

  /* RelationalOperator: '<S56>/Compare' incorporates:
   *  Constant: '<S56>/Constant'
   */
  CAVE_MachE_sil_test_B.Compare = (CAVE_MachE_sil_test_B.AccelCmd_b >
    CAVE_MachE_sil_test_P.CompareToConstant_const);

  /* Gain: '<S54>/WhlTrqReflectedToMot' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.DiffRatio;
  CAVE_MachE_sil_test_B.WhlTrqReflectedToMot = Bias *
    CAVE_MachE_sil_test_B.MotTrqRegenWhl;

  /* Gain: '<S54>/Gain' */
  CAVE_MachE_sil_test_B.MotTrqCmdRegen = CAVE_MachE_sil_test_P.Gain_Gain_e *
    CAVE_MachE_sil_test_B.WhlTrqReflectedToMot;

  /* Switch: '<S53>/Accel Decel Switch' */
  if (CAVE_MachE_sil_test_B.Compare) {
    CAVE_MachE_sil_test_B.AccelDecelSwitch = CAVE_MachE_sil_test_B.Product1_g;
  } else {
    CAVE_MachE_sil_test_B.AccelDecelSwitch =
      CAVE_MachE_sil_test_B.MotTrqCmdRegen;
  }

  /* End of Switch: '<S53>/Accel Decel Switch' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Abs: '<S57>/Abs' */
    CAVE_MachE_sil_test_B.Abs_l = fabs(CAVE_MachE_sil_test_B.Memory1);

    /* RelationalOperator: '<S59>/Compare' incorporates:
     *  Constant: '<S59>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_e = (CAVE_MachE_sil_test_B.Abs_l <
      CAVE_MachE_sil_test_P.CompareToConstant_const_l);
  }

  /* Lookup_n-D: '<S52>/DischrgLmt' */
  CAVE_MachE_sil_test_B.DischrgLmt = look1_binlcapw
    (CAVE_MachE_sil_test_B.Divide_a,
     CAVE_MachE_sil_test_P.BattDischargeLimitSocBpts,
     CAVE_MachE_sil_test_P.BattDischargeLimitTbl, 10U);

  /* Product: '<S52>/Product' incorporates:
   *  Constant: '<S52>/MaxDischrg'
   */
  CAVE_MachE_sil_test_B.Product_f = CAVE_MachE_sil_test_B.DischrgLmt *
    CAVE_MachE_sil_test_P.BattDischargeMaxPwr;

  /* Product: '<S60>/Product3' */
  CAVE_MachE_sil_test_B.Product3_f = CAVE_MachE_sil_test_B.Memory1 *
    CAVE_MachE_sil_test_B.AccelDecelSwitch;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S60>/rads_to_rpm' */
    CAVE_MachE_sil_test_B.rads_to_rpm = CAVE_MachE_sil_test_P.rads_to_rpm_Gain *
      CAVE_MachE_sil_test_B.Memory1;

    /* Abs: '<S60>/Abs' */
    CAVE_MachE_sil_test_B.Abs_f = fabs(CAVE_MachE_sil_test_B.rads_to_rpm);
  }

  /* Abs: '<S60>/Abs1' */
  CAVE_MachE_sil_test_B.Abs1 = fabs(CAVE_MachE_sil_test_B.AccelDecelSwitch);

  /* Lookup_n-D: '<S60>/Eff Map' */
  CAVE_MachE_sil_test_B.EffMap = look2_binlcapw(CAVE_MachE_sil_test_B.Abs_f,
    CAVE_MachE_sil_test_B.Abs1, CAVE_MachE_sil_test_P.MtrEffSpdBpts,
    CAVE_MachE_sil_test_P.MtrEffTrqBpts, CAVE_MachE_sil_test_P.MtrEffTbl,
    CAVE_MachE_sil_test_P.EffMap_maxIndex, 12U);

  /* Gain: '<S60>/Gain1' */
  CAVE_MachE_sil_test_B.Gain1_h = CAVE_MachE_sil_test_P.Gain1_Gain_o *
    CAVE_MachE_sil_test_B.EffMap;

  /* Product: '<S60>/Product' */
  CAVE_MachE_sil_test_B.Product_o4 = CAVE_MachE_sil_test_B.Memory1 *
    CAVE_MachE_sil_test_B.AccelDecelSwitch;

  /* Switch: '<S60>/Switch2' incorporates:
   *  Constant: '<S60>/Constant1'
   *  Constant: '<S60>/Constant2'
   */
  if (CAVE_MachE_sil_test_B.Product_o4 >=
      CAVE_MachE_sil_test_P.Switch2_Threshold_o) {
    CAVE_MachE_sil_test_B.Switch2_d = CAVE_MachE_sil_test_P.Constant1_Value_g;
  } else {
    CAVE_MachE_sil_test_B.Switch2_d = CAVE_MachE_sil_test_P.Constant2_Value_k;
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Math: '<S60>/Math Function' */
  Bias = CAVE_MachE_sil_test_B.Gain1_h;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.Switch2_d;
  if ((Bias < 0.0) && (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 >
                       floor
                       (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1)))
  {
    CAVE_MachE_sil_test_B.MathFunction = -rt_powd_snf(-Bias,
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  } else {
    CAVE_MachE_sil_test_B.MathFunction = rt_powd_snf(Bias,
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  }

  /* End of Math: '<S60>/Math Function' */

  /* Product: '<S60>/Product4' */
  CAVE_MachE_sil_test_B.Product4 = CAVE_MachE_sil_test_B.Product3_f *
    CAVE_MachE_sil_test_B.MathFunction;

  /* Sum: '<S58>/Subtract' */
  CAVE_MachE_sil_test_B.Subtract_e = CAVE_MachE_sil_test_B.Product_f -
    CAVE_MachE_sil_test_B.Product4;

  /* RelationalOperator: '<S63>/Compare' incorporates:
   *  Constant: '<S63>/Constant'
   */
  CAVE_MachE_sil_test_B.Compare_l = (CAVE_MachE_sil_test_B.Subtract_e >=
    CAVE_MachE_sil_test_P.Constant_Value);

  /* Lookup_n-D: '<S52>/ChrgLmt' */
  CAVE_MachE_sil_test_B.ChrgLmt_c = look1_binlcapw
    (CAVE_MachE_sil_test_B.Divide_a,
     CAVE_MachE_sil_test_P.BattChargeLimitSocBpts,
     CAVE_MachE_sil_test_P.BattChargeLimitTbl, 11U);

  /* Product: '<S52>/Product1' incorporates:
   *  Constant: '<S52>/MaxChrg'
   */
  CAVE_MachE_sil_test_B.Product1_h = CAVE_MachE_sil_test_B.ChrgLmt_c *
    CAVE_MachE_sil_test_P.BattChargeLimitMaxPwr;

  /* Sum: '<S58>/Subtract1' */
  CAVE_MachE_sil_test_B.Subtract1 = CAVE_MachE_sil_test_B.Product1_h -
    CAVE_MachE_sil_test_B.Product4;

  /* RelationalOperator: '<S64>/Compare' incorporates:
   *  Constant: '<S64>/Constant'
   */
  CAVE_MachE_sil_test_B.Compare_a = (CAVE_MachE_sil_test_B.Subtract1 <=
    CAVE_MachE_sil_test_P.Constant_Value_d);

  /* Logic: '<S58>/Logical Operator' */
  CAVE_MachE_sil_test_B.LogicalOperator = (CAVE_MachE_sil_test_B.Compare_l &&
    CAVE_MachE_sil_test_B.Compare_a);

  /* RelationalOperator: '<S65>/LowerRelop1' */
  CAVE_MachE_sil_test_B.LowerRelop1 = (CAVE_MachE_sil_test_B.Product4 >
    CAVE_MachE_sil_test_B.Product_f);

  /* RelationalOperator: '<S65>/UpperRelop' */
  CAVE_MachE_sil_test_B.UpperRelop = (CAVE_MachE_sil_test_B.Product4 <
    CAVE_MachE_sil_test_B.Product1_h);

  /* Switch: '<S65>/Switch' */
  if (CAVE_MachE_sil_test_B.UpperRelop) {
    CAVE_MachE_sil_test_B.Switch_b = CAVE_MachE_sil_test_B.Product1_h;
  } else {
    CAVE_MachE_sil_test_B.Switch_b = CAVE_MachE_sil_test_B.Product4;
  }

  /* End of Switch: '<S65>/Switch' */

  /* Switch: '<S65>/Switch2' */
  if (CAVE_MachE_sil_test_B.LowerRelop1) {
    CAVE_MachE_sil_test_B.Switch2_h = CAVE_MachE_sil_test_B.Product_f;
  } else {
    CAVE_MachE_sil_test_B.Switch2_h = CAVE_MachE_sil_test_B.Switch_b;
  }

  /* End of Switch: '<S65>/Switch2' */

  /* Product: '<S57>/ElecToMechPwr' */
  CAVE_MachE_sil_test_B.ElecToMechPwr = CAVE_MachE_sil_test_B.Switch2_h /
    CAVE_MachE_sil_test_B.MathFunction;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* UnaryMinus: '<S62>/Unary Minus' incorporates:
     *  Constant: '<S62>/Constant'
     */
    CAVE_MachE_sil_test_B.UnaryMinus = -CAVE_MachE_sil_test_P.Constant_Value_j;

    /* Switch: '<S62>/Switch1' incorporates:
     *  Constant: '<S62>/Constant'
     */
    if (CAVE_MachE_sil_test_B.Memory1 >=
        CAVE_MachE_sil_test_P.Switch1_Threshold_k) {
      CAVE_MachE_sil_test_B.Switch1 = CAVE_MachE_sil_test_P.Constant_Value_j;
    } else {
      CAVE_MachE_sil_test_B.Switch1 = CAVE_MachE_sil_test_B.UnaryMinus;
    }

    /* End of Switch: '<S62>/Switch1' */

    /* Fcn: '<S62>/Fcn' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.Memory1 / 2.0;
    Bias = rt_powd_snf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1,
                       2.0);
    CAVE_MachE_sil_test_B.Fcn = 4.0 / (3.0 - Bias);

    /* Product: '<S62>/Product' */
    CAVE_MachE_sil_test_B.Product_d = CAVE_MachE_sil_test_B.Switch1 *
      CAVE_MachE_sil_test_B.Fcn;

    /* RelationalOperator: '<S68>/Compare' incorporates:
     *  Constant: '<S68>/Constant'
     */
    Bias = -CAVE_MachE_sil_test_P.div0protectpoly_thresh;
    CAVE_MachE_sil_test_B.Compare_m = (CAVE_MachE_sil_test_B.Memory1 >= Bias);

    /* RelationalOperator: '<S69>/Compare' incorporates:
     *  Constant: '<S69>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_j = (CAVE_MachE_sil_test_B.Memory1 <=
      CAVE_MachE_sil_test_P.div0protectpoly_thresh);

    /* Logic: '<S62>/Logical Operator' */
    CAVE_MachE_sil_test_B.LogicalOperator_a = (CAVE_MachE_sil_test_B.Compare_m &&
      CAVE_MachE_sil_test_B.Compare_j);

    /* Switch: '<S62>/Switch' */
    if (CAVE_MachE_sil_test_B.LogicalOperator_a) {
      CAVE_MachE_sil_test_B.Switch_bq = CAVE_MachE_sil_test_B.Product_d;
    } else {
      CAVE_MachE_sil_test_B.Switch_bq = CAVE_MachE_sil_test_B.Memory1;
    }

    /* End of Switch: '<S62>/Switch' */
  }

  /* Product: '<S57>/MechPwrToTrq' */
  CAVE_MachE_sil_test_B.MechPwrToTrq = CAVE_MachE_sil_test_B.ElecToMechPwr /
    CAVE_MachE_sil_test_B.Switch_bq;

  /* Switch: '<S57>/Switch' */
  if (CAVE_MachE_sil_test_B.LogicalOperator) {
    CAVE_MachE_sil_test_B.Switch_n = CAVE_MachE_sil_test_B.AccelDecelSwitch;
  } else {
    CAVE_MachE_sil_test_B.Switch_n = CAVE_MachE_sil_test_B.MechPwrToTrq;
  }

  /* End of Switch: '<S57>/Switch' */

  /* Switch: '<S57>/Switch1' */
  if (CAVE_MachE_sil_test_B.Compare_e) {
    CAVE_MachE_sil_test_B.Switch1_n = CAVE_MachE_sil_test_B.AccelDecelSwitch;
  } else {
    CAVE_MachE_sil_test_B.Switch1_n = CAVE_MachE_sil_test_B.Switch_n;
  }

  /* End of Switch: '<S57>/Switch1' */

  /* RelationalOperator: '<S67>/LowerRelop1' */
  CAVE_MachE_sil_test_B.LowerRelop1_a = (CAVE_MachE_sil_test_B.Switch1_n >
    CAVE_MachE_sil_test_B.Saturation_l);
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Gain: '<S61>/Gain' */
    CAVE_MachE_sil_test_B.Gain_j4 = CAVE_MachE_sil_test_P.Gain_Gain_k *
      CAVE_MachE_sil_test_B.Saturation_l;
  }

  /* RelationalOperator: '<S67>/UpperRelop' */
  CAVE_MachE_sil_test_B.UpperRelop_p = (CAVE_MachE_sil_test_B.Switch1_n <
    CAVE_MachE_sil_test_B.Gain_j4);

  /* Switch: '<S67>/Switch' */
  if (CAVE_MachE_sil_test_B.UpperRelop_p) {
    CAVE_MachE_sil_test_B.Switch_bu = CAVE_MachE_sil_test_B.Gain_j4;
  } else {
    CAVE_MachE_sil_test_B.Switch_bu = CAVE_MachE_sil_test_B.Switch1_n;
  }

  /* End of Switch: '<S67>/Switch' */

  /* Switch: '<S67>/Switch2' */
  if (CAVE_MachE_sil_test_B.LowerRelop1_a) {
    CAVE_MachE_sil_test_B.Switch2_k = CAVE_MachE_sil_test_B.Saturation_l;
  } else {
    CAVE_MachE_sil_test_B.Switch2_k = CAVE_MachE_sil_test_B.Switch_bu;
  }

  /* End of Switch: '<S67>/Switch2' */

  /* Sum: '<S39>/Sum' */
  CAVE_MachE_sil_test_B.Sum = CAVE_MachE_sil_test_B.Switch2_k -
    CAVE_MachE_sil_test_B.Gain1;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Sum: '<S40>/Add' */
    CAVE_MachE_sil_test_B.Add_p = CAVE_MachE_sil_test_B.Product_o +
      CAVE_MachE_sil_test_B.uDLookupTable;

    /* Gain: '<S40>/Gain' */
    CAVE_MachE_sil_test_B.Gain_k = CAVE_MachE_sil_test_P.Gain_Gain_o *
      CAVE_MachE_sil_test_B.Product_o;

    /* Gain: '<S40>/Gain1' incorporates:
     *  Constant: '<S37>/Constant'
     */
    CAVE_MachE_sil_test_B.Gain1_l = CAVE_MachE_sil_test_P.Gain1_Gain_g *
      CAVE_MachE_sil_test_P.Constant_Value_l;

    /* Sum: '<S40>/Subtract' */
    CAVE_MachE_sil_test_B.Subtract_i = (CAVE_MachE_sil_test_B.Gain1_l -
      CAVE_MachE_sil_test_B.Gain_k) - CAVE_MachE_sil_test_B.Add_p;

    /* Gain: '<S14>/mph2m//s' incorporates:
     *  Constant: '<S14>/CmdSpeed (mph)'
     */
    CAVE_MachE_sil_test_B.mph2ms = CAVE_MachE_sil_test_P.mph2ms_Gain *
      CAVE_MachE_sil_test_P.CmdSpeedmph_Value;

    /* Gain: '<S14>/1//tireRadius' */
    CAVE_MachE_sil_test_B.utireRadius = CAVE_MachE_sil_test_P.utireRadius_Gain *
      CAVE_MachE_sil_test_B.mph2ms;

    /* Constant: '<S14>/Constant' */
    CAVE_MachE_sil_test_B.Constant_c = CAVE_MachE_sil_test_P.Constant_Value_i;

    /* Lookup_n-D: '<S71>/APPToTorque' incorporates:
     *  Constant: '<S71>/APP_pct'
     */
    CAVE_MachE_sil_test_B.APPTorque = look1_binlxpw
      (CAVE_MachE_sil_test_P.APP_pct_Value,
       CAVE_MachE_sil_test_P.APPToTorque_bp01Data,
       CAVE_MachE_sil_test_P.APPToTorque_tableData, 1U);

    /* Lookup_n-D: '<S71>/APPToTorque1' incorporates:
     *  Constant: '<S71>/BPP_pct'
     */
    CAVE_MachE_sil_test_B.APPToTorque1 = look1_binlxpw
      (CAVE_MachE_sil_test_P.BPP_pct_Value,
       CAVE_MachE_sil_test_P.APPToTorque1_bp01Data,
       CAVE_MachE_sil_test_P.APPToTorque1_tableData, 1U);

    /* Gain: '<S71>/Flip' */
    CAVE_MachE_sil_test_B.BPPTorque = CAVE_MachE_sil_test_P.Flip_Gain *
      CAVE_MachE_sil_test_B.APPToTorque1;

    /* Sum: '<S71>/Sum' */
    CAVE_MachE_sil_test_B.TorqueTotal = CAVE_MachE_sil_test_B.APPTorque +
      CAVE_MachE_sil_test_B.BPPTorque;
  }

  /* Integrator: '<S80>/omega wheel' */
  CAVE_MachE_sil_test_B.omegawheel = CAVE_MachE_sil_test_X.omegaWheel;

  /* Integrator: '<S81>/omega wheel' */
  CAVE_MachE_sil_test_B.omegawheel_d = CAVE_MachE_sil_test_X.omegaWheel_j;

  /* Integrator: '<S82>/omega wheel' */
  CAVE_MachE_sil_test_B.omegawheel_g = CAVE_MachE_sil_test_X.omegaWheel_d;

  /* Integrator: '<S83>/omega wheel' */
  CAVE_MachE_sil_test_B.omegawheel_k = CAVE_MachE_sil_test_X.omegaWheel_jl;

  /* Saturate: '<S73>/Saturation' */
  if (CAVE_MachE_sil_test_B.omegawheel >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_j) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_j;
  } else if (CAVE_MachE_sil_test_B.omegawheel <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_j4) {
    Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_j4;
  } else {
    Bias = CAVE_MachE_sil_test_B.omegawheel;
  }

  CAVE_MachE_sil_test_B.Saturation_fw[0] = Bias;
  if (CAVE_MachE_sil_test_B.omegawheel_d >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_j) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_j;
  } else if (CAVE_MachE_sil_test_B.omegawheel_d <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_j4) {
    Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_j4;
  } else {
    Bias = CAVE_MachE_sil_test_B.omegawheel_d;
  }

  CAVE_MachE_sil_test_B.Saturation_fw[1] = Bias;
  if (CAVE_MachE_sil_test_B.omegawheel_g >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_j) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_j;
  } else if (CAVE_MachE_sil_test_B.omegawheel_g <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_j4) {
    Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_j4;
  } else {
    Bias = CAVE_MachE_sil_test_B.omegawheel_g;
  }

  CAVE_MachE_sil_test_B.Saturation_fw[2] = Bias;
  if (CAVE_MachE_sil_test_B.omegawheel_k >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_j) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_j;
  } else if (CAVE_MachE_sil_test_B.omegawheel_k <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_j4) {
    Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_j4;
  } else {
    Bias = CAVE_MachE_sil_test_B.omegawheel_k;
  }

  CAVE_MachE_sil_test_B.Saturation_fw[3] = Bias;

  /* End of Saturate: '<S73>/Saturation' */

  /* Gain: '<S71>/WheelRadius' */
  CAVE_MachE_sil_test_B.Speed_mps[0] = CAVE_MachE_sil_test_P.WheelRadius_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[0];

  /* Gain: '<S71>/mps2kph' */
  CAVE_MachE_sil_test_B.Speed_kph[0] = CAVE_MachE_sil_test_P.mps2kph_Gain *
    CAVE_MachE_sil_test_B.Speed_mps[0];

  /* Gain: '<S71>/kph2mph' */
  CAVE_MachE_sil_test_B.Speed_mph[0] = CAVE_MachE_sil_test_P.kph2mph_Gain *
    CAVE_MachE_sil_test_B.Speed_kph[0];

  /* Lookup_n-D: '<S71>/NissanTrans' */
  Bias = CAVE_MachE_sil_test_B.Speed_mph[0];
  CAVE_MachE_sil_test_B.Gear[0] = look1_binlcpw(Bias,
    CAVE_MachE_sil_test_P.NissanTrans_bp01Data,
    CAVE_MachE_sil_test_P.NissanTrans_tableData, 7U);

  /* Product: '<S71>/Gear' */
  CAVE_MachE_sil_test_B.TransTorqueOut[0] = CAVE_MachE_sil_test_B.TorqueTotal *
    CAVE_MachE_sil_test_B.Gear[0];

  /* Gain: '<S71>/WheelRadius' */
  CAVE_MachE_sil_test_B.Speed_mps[1] = CAVE_MachE_sil_test_P.WheelRadius_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[1];

  /* Gain: '<S71>/mps2kph' */
  CAVE_MachE_sil_test_B.Speed_kph[1] = CAVE_MachE_sil_test_P.mps2kph_Gain *
    CAVE_MachE_sil_test_B.Speed_mps[1];

  /* Gain: '<S71>/kph2mph' */
  CAVE_MachE_sil_test_B.Speed_mph[1] = CAVE_MachE_sil_test_P.kph2mph_Gain *
    CAVE_MachE_sil_test_B.Speed_kph[1];

  /* Lookup_n-D: '<S71>/NissanTrans' */
  Bias = CAVE_MachE_sil_test_B.Speed_mph[1];
  CAVE_MachE_sil_test_B.Gear[1] = look1_binlcpw(Bias,
    CAVE_MachE_sil_test_P.NissanTrans_bp01Data,
    CAVE_MachE_sil_test_P.NissanTrans_tableData, 7U);

  /* Product: '<S71>/Gear' */
  CAVE_MachE_sil_test_B.TransTorqueOut[1] = CAVE_MachE_sil_test_B.TorqueTotal *
    CAVE_MachE_sil_test_B.Gear[1];

  /* Gain: '<S71>/WheelRadius' */
  CAVE_MachE_sil_test_B.Speed_mps[2] = CAVE_MachE_sil_test_P.WheelRadius_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[2];

  /* Gain: '<S71>/mps2kph' */
  CAVE_MachE_sil_test_B.Speed_kph[2] = CAVE_MachE_sil_test_P.mps2kph_Gain *
    CAVE_MachE_sil_test_B.Speed_mps[2];

  /* Gain: '<S71>/kph2mph' */
  CAVE_MachE_sil_test_B.Speed_mph[2] = CAVE_MachE_sil_test_P.kph2mph_Gain *
    CAVE_MachE_sil_test_B.Speed_kph[2];

  /* Lookup_n-D: '<S71>/NissanTrans' */
  Bias = CAVE_MachE_sil_test_B.Speed_mph[2];
  CAVE_MachE_sil_test_B.Gear[2] = look1_binlcpw(Bias,
    CAVE_MachE_sil_test_P.NissanTrans_bp01Data,
    CAVE_MachE_sil_test_P.NissanTrans_tableData, 7U);

  /* Product: '<S71>/Gear' */
  CAVE_MachE_sil_test_B.TransTorqueOut[2] = CAVE_MachE_sil_test_B.TorqueTotal *
    CAVE_MachE_sil_test_B.Gear[2];

  /* Gain: '<S71>/WheelRadius' */
  CAVE_MachE_sil_test_B.Speed_mps[3] = CAVE_MachE_sil_test_P.WheelRadius_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[3];

  /* Gain: '<S71>/mps2kph' */
  CAVE_MachE_sil_test_B.Speed_kph[3] = CAVE_MachE_sil_test_P.mps2kph_Gain *
    CAVE_MachE_sil_test_B.Speed_mps[3];

  /* Gain: '<S71>/kph2mph' */
  CAVE_MachE_sil_test_B.Speed_mph[3] = CAVE_MachE_sil_test_P.kph2mph_Gain *
    CAVE_MachE_sil_test_B.Speed_kph[3];

  /* Lookup_n-D: '<S71>/NissanTrans' */
  Bias = CAVE_MachE_sil_test_B.Speed_mph[3];
  CAVE_MachE_sil_test_B.Gear[3] = look1_binlcpw(Bias,
    CAVE_MachE_sil_test_P.NissanTrans_bp01Data,
    CAVE_MachE_sil_test_P.NissanTrans_tableData, 7U);

  /* Product: '<S71>/Gear' */
  CAVE_MachE_sil_test_B.TransTorqueOut[3] = CAVE_MachE_sil_test_B.TorqueTotal *
    CAVE_MachE_sil_test_B.Gear[3];

  /* Gain: '<S71>/FDR' */
  CAVE_MachE_sil_test_B.WheelTorqueOut[0] = CAVE_MachE_sil_test_P.FDR_Gain *
    CAVE_MachE_sil_test_B.TransTorqueOut[0];
  CAVE_MachE_sil_test_B.WheelTorqueOut[1] = CAVE_MachE_sil_test_P.FDR_Gain *
    CAVE_MachE_sil_test_B.TransTorqueOut[1];
  CAVE_MachE_sil_test_B.WheelTorqueOut[2] = CAVE_MachE_sil_test_P.FDR_Gain *
    CAVE_MachE_sil_test_B.TransTorqueOut[2];
  CAVE_MachE_sil_test_B.WheelTorqueOut[3] = CAVE_MachE_sil_test_P.FDR_Gain *
    CAVE_MachE_sil_test_B.TransTorqueOut[3];

  /* MultiPortSwitch: '<S14>/Multiport Switch1' incorporates:
   *  Constant: '<S14>/ChargingMode'
   */
  if ((int32_T)CAVE_MachE_sil_test_P.ChargingMode_Value == 0) {
    CAVE_MachE_sil_test_B.MultiportSwitch1[0] =
      CAVE_MachE_sil_test_B.Saturation_fw[0];
    CAVE_MachE_sil_test_B.MultiportSwitch1[1] =
      CAVE_MachE_sil_test_B.Saturation_fw[1];
    CAVE_MachE_sil_test_B.MultiportSwitch1[2] =
      CAVE_MachE_sil_test_B.Saturation_fw[2];
    CAVE_MachE_sil_test_B.MultiportSwitch1[3] =
      CAVE_MachE_sil_test_B.Saturation_fw[3];
  } else {
    CAVE_MachE_sil_test_B.MultiportSwitch1[0] =
      CAVE_MachE_sil_test_B.utireRadius;
    CAVE_MachE_sil_test_B.MultiportSwitch1[1] =
      CAVE_MachE_sil_test_B.utireRadius;
    CAVE_MachE_sil_test_B.MultiportSwitch1[2] =
      CAVE_MachE_sil_test_B.utireRadius;
    CAVE_MachE_sil_test_B.MultiportSwitch1[3] =
      CAVE_MachE_sil_test_B.utireRadius;
  }

  /* End of MultiPortSwitch: '<S14>/Multiport Switch1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_B.Memory_j[0] =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[0];
    CAVE_MachE_sil_test_B.Memory_j[1] =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[1];
    CAVE_MachE_sil_test_B.Memory_j[2] =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[2];
    CAVE_MachE_sil_test_B.Memory_j[3] =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[3];

    /* DiscreteIntegrator: '<S72>/Discrete-Time Integrator4' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE != 0) {
      CAVE_MachE_sil_test_B.PTWFLrot =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE;
    } else {
      CAVE_MachE_sil_test_B.PTWFLrot =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator4_gainval *
        CAVE_MachE_sil_test_B.MultiportSwitch1[0] +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S72>/Discrete-Time Integrator4' */

    /* DiscreteIntegrator: '<S72>/Discrete-Time Integrator5' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE != 0) {
      CAVE_MachE_sil_test_B.PTWFRrot =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE;
    } else {
      CAVE_MachE_sil_test_B.PTWFRrot =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator5_gainval *
        CAVE_MachE_sil_test_B.MultiportSwitch1[1] +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S72>/Discrete-Time Integrator5' */

    /* DiscreteIntegrator: '<S72>/Discrete-Time Integrator6' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE != 0) {
      CAVE_MachE_sil_test_B.PTWRLrot =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE;
    } else {
      CAVE_MachE_sil_test_B.PTWRLrot =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator6_gainval *
        CAVE_MachE_sil_test_B.MultiportSwitch1[2] +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S72>/Discrete-Time Integrator6' */

    /* DiscreteIntegrator: '<S72>/Discrete-Time Integrator7' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE != 0) {
      CAVE_MachE_sil_test_B.PtTWRRrot =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE;
    } else {
      CAVE_MachE_sil_test_B.PtTWRRrot =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator7_gainval *
        CAVE_MachE_sil_test_B.MultiportSwitch1[3] +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S72>/Discrete-Time Integrator7' */

    /* Memory: '<S72>/Memory3' */
    CAVE_MachE_sil_test_B.Memory3 = CAVE_MachE_sil_test_DW.Memory3_PreviousInput;

    /* Saturate: '<S72>/Saturation3' */
    if (CAVE_MachE_sil_test_B.Memory3 >
        CAVE_MachE_sil_test_P.Saturation3_UpperSat) {
      CAVE_MachE_sil_test_B.Saturation3 =
        CAVE_MachE_sil_test_P.Saturation3_UpperSat;
    } else if (CAVE_MachE_sil_test_B.Memory3 <
               CAVE_MachE_sil_test_P.Saturation3_LowerSat_c) {
      CAVE_MachE_sil_test_B.Saturation3 =
        CAVE_MachE_sil_test_P.Saturation3_LowerSat_c;
    } else {
      CAVE_MachE_sil_test_B.Saturation3 = CAVE_MachE_sil_test_B.Memory3;
    }

    /* End of Saturate: '<S72>/Saturation3' */

    /* Product: '<S72>/Product3' incorporates:
     *  Constant: '<S75>/Constant1'
     */
    CAVE_MachE_sil_test_B.Product3_j = CAVE_MachE_sil_test_P.Constant1_Value_m *
      CAVE_MachE_sil_test_B.Saturation3;

    /* Gain: '<S72>/Gain5' */
    CAVE_MachE_sil_test_B.Gain5 = CAVE_MachE_sil_test_P.Gain5_Gain *
      CAVE_MachE_sil_test_B.Product3_j;

    /* Memory: '<S72>/Memory1' */
    CAVE_MachE_sil_test_B.Memory1_m =
      CAVE_MachE_sil_test_DW.Memory1_PreviousInput_f;

    /* Saturate: '<S72>/Saturation1' */
    if (CAVE_MachE_sil_test_B.Memory1_m >
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_f) {
      CAVE_MachE_sil_test_B.Saturation1_f =
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_f;
    } else if (CAVE_MachE_sil_test_B.Memory1_m <
               CAVE_MachE_sil_test_P.Saturation1_LowerSat_jv) {
      CAVE_MachE_sil_test_B.Saturation1_f =
        CAVE_MachE_sil_test_P.Saturation1_LowerSat_jv;
    } else {
      CAVE_MachE_sil_test_B.Saturation1_f = CAVE_MachE_sil_test_B.Memory1_m;
    }

    /* End of Saturate: '<S72>/Saturation1' */

    /* Product: '<S72>/Product1' incorporates:
     *  Constant: '<S75>/Constant1'
     */
    CAVE_MachE_sil_test_B.Product1_p = CAVE_MachE_sil_test_P.Constant1_Value_m *
      CAVE_MachE_sil_test_B.Saturation1_f;

    /* Gain: '<S72>/Gain6' */
    CAVE_MachE_sil_test_B.Gain6 = CAVE_MachE_sil_test_P.Gain6_Gain *
      CAVE_MachE_sil_test_B.Product1_p;

    /* Memory: '<S72>/Memory2' */
    CAVE_MachE_sil_test_B.Memory2 = CAVE_MachE_sil_test_DW.Memory2_PreviousInput;

    /* Saturate: '<S72>/Saturation2' */
    if (CAVE_MachE_sil_test_B.Memory2 >
        CAVE_MachE_sil_test_P.Saturation2_UpperSat) {
      CAVE_MachE_sil_test_B.Saturation2 =
        CAVE_MachE_sil_test_P.Saturation2_UpperSat;
    } else if (CAVE_MachE_sil_test_B.Memory2 <
               CAVE_MachE_sil_test_P.Saturation2_LowerSat_h) {
      CAVE_MachE_sil_test_B.Saturation2 =
        CAVE_MachE_sil_test_P.Saturation2_LowerSat_h;
    } else {
      CAVE_MachE_sil_test_B.Saturation2 = CAVE_MachE_sil_test_B.Memory2;
    }

    /* End of Saturate: '<S72>/Saturation2' */

    /* Product: '<S72>/Product2' incorporates:
     *  Constant: '<S75>/Constant'
     */
    CAVE_MachE_sil_test_B.Product2 = CAVE_MachE_sil_test_P.Constant_Value_m *
      CAVE_MachE_sil_test_B.Saturation2;

    /* Gain: '<S72>/Gain7' */
    CAVE_MachE_sil_test_B.Gain7 = CAVE_MachE_sil_test_P.Gain7_Gain *
      CAVE_MachE_sil_test_B.Product2;

    /* Memory: '<S72>/Memory4' */
    CAVE_MachE_sil_test_B.Memory4 = CAVE_MachE_sil_test_DW.Memory4_PreviousInput;

    /* Saturate: '<S72>/Saturation4' */
    if (CAVE_MachE_sil_test_B.Memory4 >
        CAVE_MachE_sil_test_P.Saturation4_UpperSat) {
      CAVE_MachE_sil_test_B.Saturation4 =
        CAVE_MachE_sil_test_P.Saturation4_UpperSat;
    } else if (CAVE_MachE_sil_test_B.Memory4 <
               CAVE_MachE_sil_test_P.Saturation4_LowerSat) {
      CAVE_MachE_sil_test_B.Saturation4 =
        CAVE_MachE_sil_test_P.Saturation4_LowerSat;
    } else {
      CAVE_MachE_sil_test_B.Saturation4 = CAVE_MachE_sil_test_B.Memory4;
    }

    /* End of Saturate: '<S72>/Saturation4' */

    /* Product: '<S72>/Product4' incorporates:
     *  Constant: '<S75>/Constant'
     */
    CAVE_MachE_sil_test_B.Product4_g = CAVE_MachE_sil_test_P.Constant_Value_m *
      CAVE_MachE_sil_test_B.Saturation4;

    /* Gain: '<S72>/Gain8' */
    CAVE_MachE_sil_test_B.Gain8 = CAVE_MachE_sil_test_P.Gain8_Gain *
      CAVE_MachE_sil_test_B.Product4_g;

    /* Constant: '<S72>/Ignition' */
    CAVE_MachE_sil_test_B.PTIgnition = CAVE_MachE_sil_test_P.Ignition_Value;

    /* Constant: '<S72>/Operation Error' */
    CAVE_MachE_sil_test_B.OperationError =
      CAVE_MachE_sil_test_P.OperationError_Value;

    /* Constant: '<S72>/Operation State Driving' */
    CAVE_MachE_sil_test_B.OperationStateDriving =
      CAVE_MachE_sil_test_P.OperationStateDriving_Value;

    /* Constant: '<S72>/Zero1' */
    CAVE_MachE_sil_test_B.Zero1 = CAVE_MachE_sil_test_P.Zero1_Value;

    /* Constant: '<S72>/Zero2' */
    CAVE_MachE_sil_test_B.Zero2 = CAVE_MachE_sil_test_P.Zero2_Value;

    /* Constant: '<S72>/Zero3' */
    CAVE_MachE_sil_test_B.Zero3 = CAVE_MachE_sil_test_P.Zero3_Value;

    /* Constant: '<S72>/Zero4' */
    CAVE_MachE_sil_test_B.Zero4 = CAVE_MachE_sil_test_P.Zero4_Value;

    /* Constant: '<S72>/Zero5' */
    CAVE_MachE_sil_test_B.Zero5 = CAVE_MachE_sil_test_P.Zero5_Value;

    /* DiscreteIntegrator: '<S78>/Discrete-Time Integrator4' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE_b != 0) {
      CAVE_MachE_sil_test_B.PTWFLrot_a =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE_k;
    } else {
      CAVE_MachE_sil_test_B.PTWFLrot_a =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator4_gainval_a *
        CAVE_MachE_sil_test_B.omegawheel +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE_k;
    }

    /* End of DiscreteIntegrator: '<S78>/Discrete-Time Integrator4' */

    /* DiscreteIntegrator: '<S78>/Discrete-Time Integrator5' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE_g != 0) {
      CAVE_MachE_sil_test_B.PTWFRrot_n =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE_a;
    } else {
      CAVE_MachE_sil_test_B.PTWFRrot_n =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator5_gainval_e *
        CAVE_MachE_sil_test_B.omegawheel_d +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE_a;
    }

    /* End of DiscreteIntegrator: '<S78>/Discrete-Time Integrator5' */

    /* DiscreteIntegrator: '<S78>/Discrete-Time Integrator6' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE_p != 0) {
      CAVE_MachE_sil_test_B.PTWRLrot_n =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE_j;
    } else {
      CAVE_MachE_sil_test_B.PTWRLrot_n =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator6_gainval_k *
        CAVE_MachE_sil_test_B.omegawheel_g +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE_j;
    }

    /* End of DiscreteIntegrator: '<S78>/Discrete-Time Integrator6' */

    /* DiscreteIntegrator: '<S78>/Discrete-Time Integrator7' */
    if (CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE_g != 0) {
      CAVE_MachE_sil_test_B.PtTWRRrot_c =
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE_i;
    } else {
      CAVE_MachE_sil_test_B.PtTWRRrot_c =
        CAVE_MachE_sil_test_P.DiscreteTimeIntegrator7_gainval_g *
        CAVE_MachE_sil_test_B.omegawheel_k +
        CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE_i;
    }

    /* End of DiscreteIntegrator: '<S78>/Discrete-Time Integrator7' */

    /* Memory: '<S78>/Memory3' */
    CAVE_MachE_sil_test_B.Memory3_d =
      CAVE_MachE_sil_test_DW.Memory3_PreviousInput_c;

    /* Saturate: '<S78>/Saturation3' */
    if (CAVE_MachE_sil_test_B.Memory3_d >
        CAVE_MachE_sil_test_P.Saturation3_UpperSat_n) {
      CAVE_MachE_sil_test_B.Saturation3_d =
        CAVE_MachE_sil_test_P.Saturation3_UpperSat_n;
    } else if (CAVE_MachE_sil_test_B.Memory3_d <
               CAVE_MachE_sil_test_P.Saturation3_LowerSat_o) {
      CAVE_MachE_sil_test_B.Saturation3_d =
        CAVE_MachE_sil_test_P.Saturation3_LowerSat_o;
    } else {
      CAVE_MachE_sil_test_B.Saturation3_d = CAVE_MachE_sil_test_B.Memory3_d;
    }

    /* End of Saturate: '<S78>/Saturation3' */

    /* Product: '<S78>/Product3' incorporates:
     *  Constant: '<S75>/Constant1'
     */
    CAVE_MachE_sil_test_B.BrakeTrqFL = CAVE_MachE_sil_test_P.Constant1_Value_m *
      CAVE_MachE_sil_test_B.Saturation3_d;

    /* Gain: '<S78>/Gain5' */
    CAVE_MachE_sil_test_B.Gain5_m = CAVE_MachE_sil_test_P.Gain5_Gain_a *
      CAVE_MachE_sil_test_B.BrakeTrqFL;

    /* Memory: '<S78>/Memory1' */
    CAVE_MachE_sil_test_B.Memory1_o =
      CAVE_MachE_sil_test_DW.Memory1_PreviousInput_d;

    /* Saturate: '<S78>/Saturation1' */
    if (CAVE_MachE_sil_test_B.Memory1_o >
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_n) {
      CAVE_MachE_sil_test_B.Saturation1_b =
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_n;
    } else if (CAVE_MachE_sil_test_B.Memory1_o <
               CAVE_MachE_sil_test_P.Saturation1_LowerSat_lc) {
      CAVE_MachE_sil_test_B.Saturation1_b =
        CAVE_MachE_sil_test_P.Saturation1_LowerSat_lc;
    } else {
      CAVE_MachE_sil_test_B.Saturation1_b = CAVE_MachE_sil_test_B.Memory1_o;
    }

    /* End of Saturate: '<S78>/Saturation1' */

    /* Product: '<S78>/Product1' incorporates:
     *  Constant: '<S75>/Constant1'
     */
    CAVE_MachE_sil_test_B.BrakeTrqFR = CAVE_MachE_sil_test_P.Constant1_Value_m *
      CAVE_MachE_sil_test_B.Saturation1_b;

    /* Gain: '<S78>/Gain6' */
    CAVE_MachE_sil_test_B.Gain6_b = CAVE_MachE_sil_test_P.Gain6_Gain_p *
      CAVE_MachE_sil_test_B.BrakeTrqFR;

    /* Memory: '<S78>/Memory2' */
    CAVE_MachE_sil_test_B.Memory2_n =
      CAVE_MachE_sil_test_DW.Memory2_PreviousInput_f;

    /* Saturate: '<S78>/Saturation2' */
    if (CAVE_MachE_sil_test_B.Memory2_n >
        CAVE_MachE_sil_test_P.Saturation2_UpperSat_d) {
      CAVE_MachE_sil_test_B.Saturation2_f =
        CAVE_MachE_sil_test_P.Saturation2_UpperSat_d;
    } else if (CAVE_MachE_sil_test_B.Memory2_n <
               CAVE_MachE_sil_test_P.Saturation2_LowerSat_l) {
      CAVE_MachE_sil_test_B.Saturation2_f =
        CAVE_MachE_sil_test_P.Saturation2_LowerSat_l;
    } else {
      CAVE_MachE_sil_test_B.Saturation2_f = CAVE_MachE_sil_test_B.Memory2_n;
    }

    /* End of Saturate: '<S78>/Saturation2' */

    /* Product: '<S78>/Product2' incorporates:
     *  Constant: '<S75>/Constant'
     */
    CAVE_MachE_sil_test_B.BrakeTrqRL = CAVE_MachE_sil_test_P.Constant_Value_m *
      CAVE_MachE_sil_test_B.Saturation2_f;

    /* Gain: '<S78>/Gain7' */
    CAVE_MachE_sil_test_B.Gain7_g = CAVE_MachE_sil_test_P.Gain7_Gain_f *
      CAVE_MachE_sil_test_B.BrakeTrqRL;

    /* Memory: '<S78>/Memory4' */
    CAVE_MachE_sil_test_B.Memory4_m =
      CAVE_MachE_sil_test_DW.Memory4_PreviousInput_h;

    /* Saturate: '<S78>/Saturation4' */
    if (CAVE_MachE_sil_test_B.Memory4_m >
        CAVE_MachE_sil_test_P.Saturation4_UpperSat_e) {
      CAVE_MachE_sil_test_B.Saturation4_g =
        CAVE_MachE_sil_test_P.Saturation4_UpperSat_e;
    } else if (CAVE_MachE_sil_test_B.Memory4_m <
               CAVE_MachE_sil_test_P.Saturation4_LowerSat_n) {
      CAVE_MachE_sil_test_B.Saturation4_g =
        CAVE_MachE_sil_test_P.Saturation4_LowerSat_n;
    } else {
      CAVE_MachE_sil_test_B.Saturation4_g = CAVE_MachE_sil_test_B.Memory4_m;
    }

    /* End of Saturate: '<S78>/Saturation4' */

    /* Product: '<S78>/Product4' incorporates:
     *  Constant: '<S75>/Constant'
     */
    CAVE_MachE_sil_test_B.BrakeTrqRR = CAVE_MachE_sil_test_P.Constant_Value_m *
      CAVE_MachE_sil_test_B.Saturation4_g;

    /* Gain: '<S78>/Gain8' */
    CAVE_MachE_sil_test_B.Gain8_a = CAVE_MachE_sil_test_P.Gain8_Gain_i *
      CAVE_MachE_sil_test_B.BrakeTrqRR;

    /* Constant: '<S78>/Ignition' */
    CAVE_MachE_sil_test_B.PTIgnition_f = CAVE_MachE_sil_test_P.Ignition_Value_l;

    /* Constant: '<S78>/Operation Error' */
    CAVE_MachE_sil_test_B.OperationError_b =
      CAVE_MachE_sil_test_P.OperationError_Value_o;

    /* Constant: '<S78>/Operation State Driving' */
    CAVE_MachE_sil_test_B.OperationStateDriving_j =
      CAVE_MachE_sil_test_P.OperationStateDriving_Value_m;

    /* MinMax: '<S98>/MinMax' incorporates:
     *  Constant: '<S74>/Tau'
     *  Constant: '<S98>/dt'
     */
    Bias = fmax(CAVE_MachE_sil_test_P.dt_Value, CAVE_MachE_sil_test_P.Tau_Value);
    CAVE_MachE_sil_test_B.MinMax = Bias;

    /* Product: '<S98>/Divide1' incorporates:
     *  Constant: '<S98>/dt'
     */
    CAVE_MachE_sil_test_B.Divide1 = CAVE_MachE_sil_test_P.dt_Value /
      CAVE_MachE_sil_test_B.MinMax;

    /* Product: '<S98>/Product' */
    CAVE_MachE_sil_test_B.Product_k = CAVE_MachE_sil_test_B.VectorConcatenate_l
      [0] * CAVE_MachE_sil_test_B.Divide1;

    /* Sum: '<S98>/Sum3' incorporates:
     *  Constant: '<S98>/one'
     */
    CAVE_MachE_sil_test_B.Sum3 = CAVE_MachE_sil_test_P.one_Value -
      CAVE_MachE_sil_test_B.Divide1;

    /* Memory: '<S98>/Memory' */
    CAVE_MachE_sil_test_B.Memory_h =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_g;

    /* Product: '<S98>/Product2' */
    CAVE_MachE_sil_test_B.Product2_c = CAVE_MachE_sil_test_B.Sum3 *
      CAVE_MachE_sil_test_B.Memory_h;

    /* Sum: '<S98>/Add' */
    CAVE_MachE_sil_test_B.Add_b = CAVE_MachE_sil_test_B.Product_k +
      CAVE_MachE_sil_test_B.Product2_c;

    /* DataTypeConversion: '<S98>/Data Type Conversion' */
    CAVE_MachE_sil_test_B.DataTypeConversion = (real32_T)
      CAVE_MachE_sil_test_B.Add_b;

    /* DataTypeConversion: '<S74>/Data Type Conversion1' */
    CAVE_MachE_sil_test_B.LF = CAVE_MachE_sil_test_B.DataTypeConversion;

    /* MinMax: '<S99>/MinMax' incorporates:
     *  Constant: '<S74>/Tau'
     *  Constant: '<S99>/dt'
     */
    Bias = fmax(CAVE_MachE_sil_test_P.dt_Value_l,
                CAVE_MachE_sil_test_P.Tau_Value);
    CAVE_MachE_sil_test_B.MinMax_f = Bias;

    /* Product: '<S99>/Divide1' incorporates:
     *  Constant: '<S99>/dt'
     */
    CAVE_MachE_sil_test_B.Divide1_o = CAVE_MachE_sil_test_P.dt_Value_l /
      CAVE_MachE_sil_test_B.MinMax_f;

    /* Product: '<S99>/Product' */
    CAVE_MachE_sil_test_B.Product_p = CAVE_MachE_sil_test_B.VectorConcatenate_l
      [1] * CAVE_MachE_sil_test_B.Divide1_o;

    /* Sum: '<S99>/Sum3' incorporates:
     *  Constant: '<S99>/one'
     */
    CAVE_MachE_sil_test_B.Sum3_h = CAVE_MachE_sil_test_P.one_Value_o -
      CAVE_MachE_sil_test_B.Divide1_o;

    /* Memory: '<S99>/Memory' */
    CAVE_MachE_sil_test_B.Memory_c =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_p;

    /* Product: '<S99>/Product2' */
    CAVE_MachE_sil_test_B.Product2_i = CAVE_MachE_sil_test_B.Sum3_h *
      CAVE_MachE_sil_test_B.Memory_c;

    /* Sum: '<S99>/Add' */
    CAVE_MachE_sil_test_B.Add_k = CAVE_MachE_sil_test_B.Product_p +
      CAVE_MachE_sil_test_B.Product2_i;

    /* DataTypeConversion: '<S99>/Data Type Conversion' */
    CAVE_MachE_sil_test_B.DataTypeConversion_a = (real32_T)
      CAVE_MachE_sil_test_B.Add_k;

    /* DataTypeConversion: '<S74>/Data Type Conversion2' */
    CAVE_MachE_sil_test_B.RF = CAVE_MachE_sil_test_B.DataTypeConversion_a;

    /* MinMax: '<S100>/MinMax' incorporates:
     *  Constant: '<S100>/dt'
     *  Constant: '<S74>/Tau'
     */
    Bias = fmax(CAVE_MachE_sil_test_P.dt_Value_l0,
                CAVE_MachE_sil_test_P.Tau_Value);
    CAVE_MachE_sil_test_B.MinMax_l = Bias;

    /* Product: '<S100>/Divide1' incorporates:
     *  Constant: '<S100>/dt'
     */
    CAVE_MachE_sil_test_B.Divide1_d = CAVE_MachE_sil_test_P.dt_Value_l0 /
      CAVE_MachE_sil_test_B.MinMax_l;

    /* Product: '<S100>/Product' */
    CAVE_MachE_sil_test_B.Product_j = CAVE_MachE_sil_test_B.VectorConcatenate_l
      [2] * CAVE_MachE_sil_test_B.Divide1_d;

    /* Sum: '<S100>/Sum3' incorporates:
     *  Constant: '<S100>/one'
     */
    CAVE_MachE_sil_test_B.Sum3_b = CAVE_MachE_sil_test_P.one_Value_l -
      CAVE_MachE_sil_test_B.Divide1_d;

    /* Memory: '<S100>/Memory' */
    CAVE_MachE_sil_test_B.Memory_b =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_h;

    /* Product: '<S100>/Product2' */
    CAVE_MachE_sil_test_B.Product2_a = CAVE_MachE_sil_test_B.Sum3_b *
      CAVE_MachE_sil_test_B.Memory_b;

    /* Sum: '<S100>/Add' */
    CAVE_MachE_sil_test_B.Add_d3 = CAVE_MachE_sil_test_B.Product_j +
      CAVE_MachE_sil_test_B.Product2_a;

    /* DataTypeConversion: '<S100>/Data Type Conversion' */
    CAVE_MachE_sil_test_B.DataTypeConversion_c = (real32_T)
      CAVE_MachE_sil_test_B.Add_d3;

    /* DataTypeConversion: '<S74>/Data Type Conversion3' */
    CAVE_MachE_sil_test_B.LR = CAVE_MachE_sil_test_B.DataTypeConversion_c;

    /* MinMax: '<S101>/MinMax' incorporates:
     *  Constant: '<S101>/dt'
     *  Constant: '<S74>/Tau'
     */
    Bias = fmax(CAVE_MachE_sil_test_P.dt_Value_n,
                CAVE_MachE_sil_test_P.Tau_Value);
    CAVE_MachE_sil_test_B.MinMax_h = Bias;

    /* Product: '<S101>/Divide1' incorporates:
     *  Constant: '<S101>/dt'
     */
    CAVE_MachE_sil_test_B.Divide1_n = CAVE_MachE_sil_test_P.dt_Value_n /
      CAVE_MachE_sil_test_B.MinMax_h;

    /* Product: '<S101>/Product' */
    CAVE_MachE_sil_test_B.Product_b3 =
      CAVE_MachE_sil_test_B.VectorConcatenate_l[3] *
      CAVE_MachE_sil_test_B.Divide1_n;

    /* Sum: '<S101>/Sum3' incorporates:
     *  Constant: '<S101>/one'
     */
    CAVE_MachE_sil_test_B.Sum3_k = CAVE_MachE_sil_test_P.one_Value_p -
      CAVE_MachE_sil_test_B.Divide1_n;

    /* Memory: '<S101>/Memory' */
    CAVE_MachE_sil_test_B.Memory_a =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_e;

    /* Product: '<S101>/Product2' */
    CAVE_MachE_sil_test_B.Product2_n = CAVE_MachE_sil_test_B.Sum3_k *
      CAVE_MachE_sil_test_B.Memory_a;

    /* Sum: '<S101>/Add' */
    CAVE_MachE_sil_test_B.Add_c = CAVE_MachE_sil_test_B.Product_b3 +
      CAVE_MachE_sil_test_B.Product2_n;

    /* DataTypeConversion: '<S101>/Data Type Conversion' */
    CAVE_MachE_sil_test_B.DataTypeConversion_n = (real32_T)
      CAVE_MachE_sil_test_B.Add_c;

    /* DataTypeConversion: '<S74>/Data Type Conversion4' */
    CAVE_MachE_sil_test_B.RR = CAVE_MachE_sil_test_B.DataTypeConversion_n;

    /* Switch: '<S74>/TorqueIn' incorporates:
     *  Constant: '<S74>/ManualTrq_Nm'
     *  Constant: '<S76>/MasterSw'
     */
    if (CAVE_MachE_sil_test_P.MasterSw_Value >
        CAVE_MachE_sil_test_P.TorqueIn_Threshold) {
      CAVE_MachE_sil_test_B.TorqueIn[0] = CAVE_MachE_sil_test_B.LF;
      CAVE_MachE_sil_test_B.TorqueIn[1] = CAVE_MachE_sil_test_B.RF;
      CAVE_MachE_sil_test_B.TorqueIn[2] = CAVE_MachE_sil_test_B.LR;
      CAVE_MachE_sil_test_B.TorqueIn[3] = CAVE_MachE_sil_test_B.RR;
    } else {
      CAVE_MachE_sil_test_B.TorqueIn[0] =
        CAVE_MachE_sil_test_P.ManualTrq_Nm_Value;
      CAVE_MachE_sil_test_B.TorqueIn[1] =
        CAVE_MachE_sil_test_P.ManualTrq_Nm_Value;
      CAVE_MachE_sil_test_B.TorqueIn[2] =
        CAVE_MachE_sil_test_P.ManualTrq_Nm_Value;
      CAVE_MachE_sil_test_B.TorqueIn[3] =
        CAVE_MachE_sil_test_P.ManualTrq_Nm_Value;
    }

    /* End of Switch: '<S74>/TorqueIn' */

    /* Saturate: '<S74>/SatTrq' */
    Bias = CAVE_MachE_sil_test_B.TorqueIn[0];
    if (Bias > CAVE_MachE_sil_test_P.SatTrq_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatTrq_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatTrq_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatTrq_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.SatTrq[0] = Bias;

    /* RelationalOperator: '<S96>/Compare' incorporates:
     *  Constant: '<S96>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_p[0] = (CAVE_MachE_sil_test_B.SatTrq[0] >=
      CAVE_MachE_sil_test_P.CompareToConstant_const_m);

    /* RelationalOperator: '<S97>/Compare' incorporates:
     *  Constant: '<S97>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_k[0] = (CAVE_MachE_sil_test_B.SatTrq[0] <=
      CAVE_MachE_sil_test_P.CompareToConstant1_const);

    /* Logic: '<S74>/Logical Operator' */
    CAVE_MachE_sil_test_B.LogicalOperator_ap[0] =
      (CAVE_MachE_sil_test_B.Compare_p[0] && CAVE_MachE_sil_test_B.Compare_k[0]);

    /* Switch: '<S74>/TorqueIn ' */
    if (CAVE_MachE_sil_test_B.LogicalOperator_ap[0]) {
      CAVE_MachE_sil_test_B.Trq[0] = 0.0;
    } else {
      CAVE_MachE_sil_test_B.Trq[0] = CAVE_MachE_sil_test_B.SatTrq[0];
    }

    /* Saturate: '<S74>/SatTrq' */
    Bias = CAVE_MachE_sil_test_B.TorqueIn[1];
    if (Bias > CAVE_MachE_sil_test_P.SatTrq_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatTrq_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatTrq_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatTrq_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.SatTrq[1] = Bias;

    /* RelationalOperator: '<S96>/Compare' incorporates:
     *  Constant: '<S96>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_p[1] = (CAVE_MachE_sil_test_B.SatTrq[1] >=
      CAVE_MachE_sil_test_P.CompareToConstant_const_m);

    /* RelationalOperator: '<S97>/Compare' incorporates:
     *  Constant: '<S97>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_k[1] = (CAVE_MachE_sil_test_B.SatTrq[1] <=
      CAVE_MachE_sil_test_P.CompareToConstant1_const);

    /* Logic: '<S74>/Logical Operator' */
    CAVE_MachE_sil_test_B.LogicalOperator_ap[1] =
      (CAVE_MachE_sil_test_B.Compare_p[1] && CAVE_MachE_sil_test_B.Compare_k[1]);

    /* Switch: '<S74>/TorqueIn ' */
    if (CAVE_MachE_sil_test_B.LogicalOperator_ap[1]) {
      CAVE_MachE_sil_test_B.Trq[1] = 0.0;
    } else {
      CAVE_MachE_sil_test_B.Trq[1] = CAVE_MachE_sil_test_B.SatTrq[1];
    }

    /* Saturate: '<S74>/SatTrq' */
    Bias = CAVE_MachE_sil_test_B.TorqueIn[2];
    if (Bias > CAVE_MachE_sil_test_P.SatTrq_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatTrq_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatTrq_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatTrq_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.SatTrq[2] = Bias;

    /* RelationalOperator: '<S96>/Compare' incorporates:
     *  Constant: '<S96>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_p[2] = (CAVE_MachE_sil_test_B.SatTrq[2] >=
      CAVE_MachE_sil_test_P.CompareToConstant_const_m);

    /* RelationalOperator: '<S97>/Compare' incorporates:
     *  Constant: '<S97>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_k[2] = (CAVE_MachE_sil_test_B.SatTrq[2] <=
      CAVE_MachE_sil_test_P.CompareToConstant1_const);

    /* Logic: '<S74>/Logical Operator' */
    CAVE_MachE_sil_test_B.LogicalOperator_ap[2] =
      (CAVE_MachE_sil_test_B.Compare_p[2] && CAVE_MachE_sil_test_B.Compare_k[2]);

    /* Switch: '<S74>/TorqueIn ' */
    if (CAVE_MachE_sil_test_B.LogicalOperator_ap[2]) {
      CAVE_MachE_sil_test_B.Trq[2] = 0.0;
    } else {
      CAVE_MachE_sil_test_B.Trq[2] = CAVE_MachE_sil_test_B.SatTrq[2];
    }

    /* Saturate: '<S74>/SatTrq' */
    Bias = CAVE_MachE_sil_test_B.TorqueIn[3];
    if (Bias > CAVE_MachE_sil_test_P.SatTrq_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatTrq_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatTrq_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatTrq_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.SatTrq[3] = Bias;

    /* RelationalOperator: '<S96>/Compare' incorporates:
     *  Constant: '<S96>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_p[3] = (CAVE_MachE_sil_test_B.SatTrq[3] >=
      CAVE_MachE_sil_test_P.CompareToConstant_const_m);

    /* RelationalOperator: '<S97>/Compare' incorporates:
     *  Constant: '<S97>/Constant'
     */
    CAVE_MachE_sil_test_B.Compare_k[3] = (CAVE_MachE_sil_test_B.SatTrq[3] <=
      CAVE_MachE_sil_test_P.CompareToConstant1_const);

    /* Logic: '<S74>/Logical Operator' */
    CAVE_MachE_sil_test_B.LogicalOperator_ap[3] =
      (CAVE_MachE_sil_test_B.Compare_p[3] && CAVE_MachE_sil_test_B.Compare_k[3]);

    /* Switch: '<S74>/TorqueIn ' */
    if (CAVE_MachE_sil_test_B.LogicalOperator_ap[3]) {
      CAVE_MachE_sil_test_B.Trq[3] = 0.0;
    } else {
      CAVE_MachE_sil_test_B.Trq[3] = CAVE_MachE_sil_test_B.SatTrq[3];
    }

    /* MultiPortSwitch: '<S14>/Multiport Switch' incorporates:
     *  Constant: '<S14>/DriverSwitch'
     */
    if ((int32_T)CAVE_MachE_sil_test_P.DriverSwitch_Value == 0) {
      CAVE_MachE_sil_test_B.MultiportSwitch_n[0] = CAVE_MachE_sil_test_B.Trq[0];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[1] = CAVE_MachE_sil_test_B.Trq[1];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[2] = CAVE_MachE_sil_test_B.Trq[2];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[3] = CAVE_MachE_sil_test_B.Trq[3];
    } else {
      CAVE_MachE_sil_test_B.MultiportSwitch_n[0] =
        CAVE_MachE_sil_test_B.Memory_j[0];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[1] =
        CAVE_MachE_sil_test_B.Memory_j[1];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[2] =
        CAVE_MachE_sil_test_B.Memory_j[2];
      CAVE_MachE_sil_test_B.MultiportSwitch_n[3] =
        CAVE_MachE_sil_test_B.Memory_j[3];
    }

    /* End of MultiPortSwitch: '<S14>/Multiport Switch' */
  }

  /* SecondOrderIntegrator: '<S334>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_d =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_m[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_b =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_m[1];

  /* Sum: '<S334>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_a = CAVE_MachE_sil_test_B.VectorConcatenate_g[0] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_d;

  /* Saturate: '<S334>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_a > CAVE_MachE_sil_test_P.Saturation_UpperSat_n)
  {
    CAVE_MachE_sil_test_B.Saturation_m =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_n;
  } else if (CAVE_MachE_sil_test_B.Sum6_a <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_i) {
    CAVE_MachE_sil_test_B.Saturation_m =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_i;
  } else {
    CAVE_MachE_sil_test_B.Saturation_m = CAVE_MachE_sil_test_B.Sum6_a;
  }

  /* End of Saturate: '<S334>/Saturation' */

  /* Sum: '<S80>/Add2' incorporates:
   *  Constant: '<S80>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_g = CAVE_MachE_sil_test_P.Constant9_Value -
    CAVE_MachE_sil_test_B.Saturation_m;

  /* Saturate: '<S80>/Saturation' */
  if (CAVE_MachE_sil_test_B.Add2_g > CAVE_MachE_sil_test_P.Saturation_UpperSat_l)
  {
    CAVE_MachE_sil_test_B.Saturation_a =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_l;
  } else if (CAVE_MachE_sil_test_B.Add2_g <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_p0s) {
    CAVE_MachE_sil_test_B.Saturation_a =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_p0s;
  } else {
    CAVE_MachE_sil_test_B.Saturation_a = CAVE_MachE_sil_test_B.Add2_g;
  }

  /* End of Saturate: '<S80>/Saturation' */

  /* Product: '<S80>/Product3' */
  CAVE_MachE_sil_test_B.Product3_k = CAVE_MachE_sil_test_B.Integrator_a *
    CAVE_MachE_sil_test_B.Saturation_a;

  /* Sum: '<S80>/Add1' */
  CAVE_MachE_sil_test_B.Add1_c = (CAVE_MachE_sil_test_B.Product3_k -
    CAVE_MachE_sil_test_B.MultiportSwitch_n[0]) -
    CAVE_MachE_sil_test_B.CarTrq_T2WFL;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S80>/Memory' */
    CAVE_MachE_sil_test_B.Memory_m =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_o;

    /* MATLAB Function: '<S80>/MATLAB Function' */
    CAVE_MachE_sil_test_MATLABFunction(CAVE_MachE_sil_test_B.MultiportSwitch_n[0],
      CAVE_MachE_sil_test_B.Memory_m, &CAVE_MachE_sil_test_B.sf_MATLABFunction);

    /* Switch: '<S86>/Switch' incorporates:
     *  Constant: '<S86>/Zero'
     *  Constant: '<S86>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_c != 0.0) {
      CAVE_MachE_sil_test_B.Switch_c = CAVE_MachE_sil_test_P.Zero1_Value_c;
    } else {
      CAVE_MachE_sil_test_B.Switch_c = CAVE_MachE_sil_test_P.Zero_Value;
    }

    /* End of Switch: '<S86>/Switch' */
  }

  /* Product: '<S86>/Divide' */
  CAVE_MachE_sil_test_B.Divide_fj = CAVE_MachE_sil_test_B.Add1_c /
    CAVE_MachE_sil_test_B.Switch_c;

  /* Gain: '<S80>/Output Damping' */
  CAVE_MachE_sil_test_B.OutputDamping = CAVE_MachE_sil_test_P.OutputDamping_Gain
    * CAVE_MachE_sil_test_B.omegawheel;

  /* Sum: '<S80>/Sum' */
  CAVE_MachE_sil_test_B.Sum_d = (0.0 - CAVE_MachE_sil_test_B.Divide_fj) -
    CAVE_MachE_sil_test_B.OutputDamping;

  /* Switch: '<S80>/Switch' incorporates:
   *  Constant: '<S80>/Constant'
   */
  if (CAVE_MachE_sil_test_B.sf_MATLABFunction.switchFlag >
      CAVE_MachE_sil_test_P.Switch_Threshold_b) {
    CAVE_MachE_sil_test_B.Switch_p = CAVE_MachE_sil_test_P.Constant_Value_lx;
  } else {
    CAVE_MachE_sil_test_B.Switch_p = CAVE_MachE_sil_test_B.Sum_d;
  }

  /* End of Switch: '<S80>/Switch' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S85>/Switch' incorporates:
     *  Constant: '<S85>/Zero'
     *  Constant: '<S85>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_h != 0.0) {
      CAVE_MachE_sil_test_B.Switch_cx = CAVE_MachE_sil_test_P.Zero1_Value_h;
    } else {
      CAVE_MachE_sil_test_B.Switch_cx = CAVE_MachE_sil_test_P.Zero_Value_e;
    }

    /* End of Switch: '<S85>/Switch' */
  }

  /* Sum: '<S80>/Sum3' */
  CAVE_MachE_sil_test_B.Sum3_c = (CAVE_MachE_sil_test_B.CarTrq_T2WFL -
    CAVE_MachE_sil_test_B.BrakeTrqFL) + CAVE_MachE_sil_test_B.MultiportSwitch_n
    [0];

  /* Product: '<S85>/Divide' */
  CAVE_MachE_sil_test_B.Divide_b = CAVE_MachE_sil_test_B.Sum3_c /
    CAVE_MachE_sil_test_B.Switch_cx;

  /* SecondOrderIntegrator: '<S335>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_n =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_i[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_g =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_i[1];

  /* Sum: '<S335>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_k = CAVE_MachE_sil_test_B.VectorConcatenate_g[1] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_n;

  /* Saturate: '<S335>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_k > CAVE_MachE_sil_test_P.Saturation_UpperSat_i)
  {
    CAVE_MachE_sil_test_B.Saturation_b =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_i;
  } else if (CAVE_MachE_sil_test_B.Sum6_k <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_d) {
    CAVE_MachE_sil_test_B.Saturation_b =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_d;
  } else {
    CAVE_MachE_sil_test_B.Saturation_b = CAVE_MachE_sil_test_B.Sum6_k;
  }

  /* End of Saturate: '<S335>/Saturation' */

  /* Sum: '<S81>/Add2' incorporates:
   *  Constant: '<S81>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_j = CAVE_MachE_sil_test_P.Constant9_Value_i -
    CAVE_MachE_sil_test_B.Saturation_b;

  /* Saturate: '<S81>/Saturation' */
  if (CAVE_MachE_sil_test_B.Add2_j >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_e2) {
    CAVE_MachE_sil_test_B.Saturation_k =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_e2;
  } else if (CAVE_MachE_sil_test_B.Add2_j <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_f) {
    CAVE_MachE_sil_test_B.Saturation_k =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_f;
  } else {
    CAVE_MachE_sil_test_B.Saturation_k = CAVE_MachE_sil_test_B.Add2_j;
  }

  /* End of Saturate: '<S81>/Saturation' */

  /* Product: '<S81>/Product3' */
  CAVE_MachE_sil_test_B.Product3_lg = CAVE_MachE_sil_test_B.Integrator_l *
    CAVE_MachE_sil_test_B.Saturation_k;

  /* Sum: '<S81>/Add1' */
  CAVE_MachE_sil_test_B.Add1_l = (CAVE_MachE_sil_test_B.Product3_lg -
    CAVE_MachE_sil_test_B.MultiportSwitch_n[1]) -
    CAVE_MachE_sil_test_B.CarTrq_T2WFR;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S81>/Memory' */
    CAVE_MachE_sil_test_B.Memory_cj =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_gr;

    /* MATLAB Function: '<S81>/MATLAB Function' */
    CAVE_MachE_sil_test_MATLABFunction(CAVE_MachE_sil_test_B.MultiportSwitch_n[1],
      CAVE_MachE_sil_test_B.Memory_cj,
      &CAVE_MachE_sil_test_B.sf_MATLABFunction_h);

    /* Switch: '<S89>/Switch' incorporates:
     *  Constant: '<S89>/Zero'
     *  Constant: '<S89>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_g != 0.0) {
      CAVE_MachE_sil_test_B.Switch_f = CAVE_MachE_sil_test_P.Zero1_Value_g;
    } else {
      CAVE_MachE_sil_test_B.Switch_f = CAVE_MachE_sil_test_P.Zero_Value_p;
    }

    /* End of Switch: '<S89>/Switch' */
  }

  /* Product: '<S89>/Divide' */
  CAVE_MachE_sil_test_B.Divide_n = CAVE_MachE_sil_test_B.Add1_l /
    CAVE_MachE_sil_test_B.Switch_f;

  /* Gain: '<S81>/Output Damping' */
  CAVE_MachE_sil_test_B.OutputDamping_k =
    CAVE_MachE_sil_test_P.OutputDamping_Gain_o * CAVE_MachE_sil_test_B.Divide_n;

  /* Sum: '<S81>/Sum3' */
  CAVE_MachE_sil_test_B.Sum3_ko = (CAVE_MachE_sil_test_B.CarTrq_T2WFR -
    CAVE_MachE_sil_test_B.BrakeTrqFR) + CAVE_MachE_sil_test_B.MultiportSwitch_n
    [1];

  /* Switch: '<S81>/Switch' incorporates:
   *  Constant: '<S81>/Constant'
   */
  if (CAVE_MachE_sil_test_B.sf_MATLABFunction_h.switchFlag >
      CAVE_MachE_sil_test_P.Switch_Threshold_i) {
    CAVE_MachE_sil_test_B.Switch_k = CAVE_MachE_sil_test_P.Constant_Value_bj;
  } else {
    CAVE_MachE_sil_test_B.Switch_k = CAVE_MachE_sil_test_B.OutputDamping_k;
  }

  /* End of Switch: '<S81>/Switch' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S88>/Switch' incorporates:
     *  Constant: '<S88>/Zero'
     *  Constant: '<S88>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_j != 0.0) {
      CAVE_MachE_sil_test_B.Switch_j = CAVE_MachE_sil_test_P.Zero1_Value_j;
    } else {
      CAVE_MachE_sil_test_B.Switch_j = CAVE_MachE_sil_test_P.Zero_Value_i;
    }

    /* End of Switch: '<S88>/Switch' */
  }

  /* Product: '<S88>/Divide' */
  CAVE_MachE_sil_test_B.Divide_k = CAVE_MachE_sil_test_B.Sum3_ko /
    CAVE_MachE_sil_test_B.Switch_j;

  /* SecondOrderIntegrator: '<S336>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_o4 =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_e[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_d =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_e[1];

  /* Sum: '<S336>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_k0 = CAVE_MachE_sil_test_B.VectorConcatenate_g[2] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_o4;

  /* Saturate: '<S336>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_k0 >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ew) {
    CAVE_MachE_sil_test_B.Saturation_i =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ew;
  } else if (CAVE_MachE_sil_test_B.Sum6_k0 <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_ejw) {
    CAVE_MachE_sil_test_B.Saturation_i =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_ejw;
  } else {
    CAVE_MachE_sil_test_B.Saturation_i = CAVE_MachE_sil_test_B.Sum6_k0;
  }

  /* End of Saturate: '<S336>/Saturation' */

  /* Sum: '<S82>/Add2' incorporates:
   *  Constant: '<S82>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_b = CAVE_MachE_sil_test_P.Constant9_Value_d -
    CAVE_MachE_sil_test_B.Saturation_i;

  /* Saturate: '<S82>/Saturation' */
  if (CAVE_MachE_sil_test_B.Add2_b >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_bk) {
    CAVE_MachE_sil_test_B.Saturation_d0 =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_bk;
  } else if (CAVE_MachE_sil_test_B.Add2_b <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_du) {
    CAVE_MachE_sil_test_B.Saturation_d0 =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_du;
  } else {
    CAVE_MachE_sil_test_B.Saturation_d0 = CAVE_MachE_sil_test_B.Add2_b;
  }

  /* End of Saturate: '<S82>/Saturation' */

  /* Product: '<S82>/Product3' */
  CAVE_MachE_sil_test_B.Product3_e = CAVE_MachE_sil_test_B.Integrator_fm *
    CAVE_MachE_sil_test_B.Saturation_d0;

  /* Sum: '<S82>/Add1' */
  CAVE_MachE_sil_test_B.Add1_ge1 = (CAVE_MachE_sil_test_B.Product3_e -
    CAVE_MachE_sil_test_B.MultiportSwitch_n[2]) -
    CAVE_MachE_sil_test_B.CarTrq_T2WRL;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* MATLAB Function: '<S82>/MATLAB Function' */
    CAVE_MachE_sil_test_MATLABFunction(CAVE_MachE_sil_test_B.MultiportSwitch_n[2],
      0.0, &CAVE_MachE_sil_test_B.sf_MATLABFunction_j);

    /* Memory: '<S82>/Memory' */
    CAVE_MachE_sil_test_B.Memory_n =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_b;

    /* Switch: '<S92>/Switch' incorporates:
     *  Constant: '<S92>/Zero'
     *  Constant: '<S92>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_g5 != 0.0) {
      CAVE_MachE_sil_test_B.Switch_pa = CAVE_MachE_sil_test_P.Zero1_Value_g5;
    } else {
      CAVE_MachE_sil_test_B.Switch_pa = CAVE_MachE_sil_test_P.Zero_Value_f;
    }

    /* End of Switch: '<S92>/Switch' */
  }

  /* Gain: '<S82>/Output Damping' */
  CAVE_MachE_sil_test_B.OutputDamping_m =
    CAVE_MachE_sil_test_P.OutputDamping_Gain_m *
    CAVE_MachE_sil_test_B.omegawheel_g;

  /* Product: '<S92>/Divide' */
  CAVE_MachE_sil_test_B.Divide_ox = CAVE_MachE_sil_test_B.Add1_ge1 /
    CAVE_MachE_sil_test_B.Switch_pa;

  /* Sum: '<S82>/Sum' */
  CAVE_MachE_sil_test_B.Sum_h = (0.0 - CAVE_MachE_sil_test_B.Divide_ox) -
    CAVE_MachE_sil_test_B.OutputDamping_m;

  /* Sum: '<S82>/Sum3' */
  CAVE_MachE_sil_test_B.Sum3_f = (CAVE_MachE_sil_test_B.CarTrq_T2WRL -
    CAVE_MachE_sil_test_B.BrakeTrqRL) + CAVE_MachE_sil_test_B.MultiportSwitch_n
    [2];

  /* Switch: '<S82>/Switch' incorporates:
   *  Constant: '<S82>/Constant'
   */
  if (CAVE_MachE_sil_test_B.sf_MATLABFunction_j.switchFlag >
      CAVE_MachE_sil_test_P.Switch_Threshold_c) {
    CAVE_MachE_sil_test_B.Switch_cv = CAVE_MachE_sil_test_P.Constant_Value_i5;
  } else {
    CAVE_MachE_sil_test_B.Switch_cv = CAVE_MachE_sil_test_B.Sum_h;
  }

  /* End of Switch: '<S82>/Switch' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S91>/Switch' incorporates:
     *  Constant: '<S91>/Zero'
     *  Constant: '<S91>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_ja != 0.0) {
      CAVE_MachE_sil_test_B.Switch_o = CAVE_MachE_sil_test_P.Zero1_Value_ja;
    } else {
      CAVE_MachE_sil_test_B.Switch_o = CAVE_MachE_sil_test_P.Zero_Value_j;
    }

    /* End of Switch: '<S91>/Switch' */
  }

  /* Product: '<S91>/Divide' */
  CAVE_MachE_sil_test_B.Divide_d = CAVE_MachE_sil_test_B.Sum3_f /
    CAVE_MachE_sil_test_B.Switch_o;

  /* SecondOrderIntegrator: '<S337>/Integrator, Second-Order' */
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_p =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_jk[0];
  CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_hy =
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_jk[1];

  /* Sum: '<S337>/Sum6' */
  CAVE_MachE_sil_test_B.Sum6_fh = CAVE_MachE_sil_test_B.VectorConcatenate_g[3] -
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_p;

  /* Saturate: '<S337>/Saturation' */
  if (CAVE_MachE_sil_test_B.Sum6_fh >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ik) {
    CAVE_MachE_sil_test_B.Saturation_o =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ik;
  } else if (CAVE_MachE_sil_test_B.Sum6_fh <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_a) {
    CAVE_MachE_sil_test_B.Saturation_o =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_a;
  } else {
    CAVE_MachE_sil_test_B.Saturation_o = CAVE_MachE_sil_test_B.Sum6_fh;
  }

  /* End of Saturate: '<S337>/Saturation' */

  /* Sum: '<S83>/Add2' incorporates:
   *  Constant: '<S83>/Constant9'
   */
  CAVE_MachE_sil_test_B.Add2_n = CAVE_MachE_sil_test_P.Constant9_Value_j -
    CAVE_MachE_sil_test_B.Saturation_o;

  /* Saturate: '<S83>/Saturation' */
  if (CAVE_MachE_sil_test_B.Add2_n > CAVE_MachE_sil_test_P.Saturation_UpperSat_c)
  {
    CAVE_MachE_sil_test_B.Saturation_c =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_c;
  } else if (CAVE_MachE_sil_test_B.Add2_n <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_m) {
    CAVE_MachE_sil_test_B.Saturation_c =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_m;
  } else {
    CAVE_MachE_sil_test_B.Saturation_c = CAVE_MachE_sil_test_B.Add2_n;
  }

  /* End of Saturate: '<S83>/Saturation' */

  /* Product: '<S83>/Product3' */
  CAVE_MachE_sil_test_B.Product3_m = CAVE_MachE_sil_test_B.Integrator_b *
    CAVE_MachE_sil_test_B.Saturation_c;

  /* Sum: '<S83>/Add1' */
  CAVE_MachE_sil_test_B.Add1_m = (CAVE_MachE_sil_test_B.Product3_m -
    CAVE_MachE_sil_test_B.MultiportSwitch_n[3]) -
    CAVE_MachE_sil_test_B.CarTrq_T2WRR;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S95>/Switch' incorporates:
     *  Constant: '<S95>/Zero'
     *  Constant: '<S95>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_cq != 0.0) {
      CAVE_MachE_sil_test_B.Switch_m = CAVE_MachE_sil_test_P.Zero1_Value_cq;
    } else {
      CAVE_MachE_sil_test_B.Switch_m = CAVE_MachE_sil_test_P.Zero_Value_jk;
    }

    /* End of Switch: '<S95>/Switch' */
  }

  /* Product: '<S95>/Divide' */
  CAVE_MachE_sil_test_B.Divide_h = CAVE_MachE_sil_test_B.Add1_m /
    CAVE_MachE_sil_test_B.Switch_m;

  /* Gain: '<S83>/Gain' */
  CAVE_MachE_sil_test_B.Gain_n = CAVE_MachE_sil_test_P.Gain_Gain_j *
    CAVE_MachE_sil_test_B.Divide_h;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S83>/Memory' */
    CAVE_MachE_sil_test_B.Memory_b2 =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_eq;

    /* MATLAB Function: '<S83>/MATLAB Function' */
    CAVE_MachE_sil_test_MATLABFunction(CAVE_MachE_sil_test_B.MultiportSwitch_n[3],
      CAVE_MachE_sil_test_B.Memory_b2,
      &CAVE_MachE_sil_test_B.sf_MATLABFunction_g);
  }

  /* Sum: '<S83>/Sum3' */
  CAVE_MachE_sil_test_B.Sum3_d = (CAVE_MachE_sil_test_B.CarTrq_T2WRR -
    CAVE_MachE_sil_test_B.BrakeTrqRR) + CAVE_MachE_sil_test_B.MultiportSwitch_n
    [3];

  /* Switch: '<S83>/Switch' incorporates:
   *  Constant: '<S83>/Constant'
   */
  if (CAVE_MachE_sil_test_B.sf_MATLABFunction_g.switchFlag >
      CAVE_MachE_sil_test_P.Switch_Threshold_bu) {
    CAVE_MachE_sil_test_B.Switch_cn = CAVE_MachE_sil_test_P.Constant_Value_ds;
  } else {
    CAVE_MachE_sil_test_B.Switch_cn = CAVE_MachE_sil_test_B.Gain_n;
  }

  /* End of Switch: '<S83>/Switch' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Switch: '<S94>/Switch' incorporates:
     *  Constant: '<S94>/Zero'
     *  Constant: '<S94>/Zero1'
     */
    if (CAVE_MachE_sil_test_P.Zero1_Value_n != 0.0) {
      CAVE_MachE_sil_test_B.Switch_cw = CAVE_MachE_sil_test_P.Zero1_Value_n;
    } else {
      CAVE_MachE_sil_test_B.Switch_cw = CAVE_MachE_sil_test_P.Zero_Value_jw;
    }

    /* End of Switch: '<S94>/Switch' */

    /* Constant: '<S78>/Zero1' */
    CAVE_MachE_sil_test_B.Zero1_k = CAVE_MachE_sil_test_P.Zero1_Value_a;

    /* Constant: '<S78>/Zero2' */
    CAVE_MachE_sil_test_B.Zero2_o = CAVE_MachE_sil_test_P.Zero2_Value_p;

    /* Constant: '<S78>/Zero3' */
    CAVE_MachE_sil_test_B.Zero3_o = CAVE_MachE_sil_test_P.Zero3_Value_j;

    /* Constant: '<S78>/Zero4' */
    CAVE_MachE_sil_test_B.Zero4_h = CAVE_MachE_sil_test_P.Zero4_Value_i;

    /* Constant: '<S78>/Zero5' */
    CAVE_MachE_sil_test_B.Zero5_n = CAVE_MachE_sil_test_P.Zero5_Value_l;

    /* Product: '<S79>/Product' incorporates:
     *  Constant: '<S78>/Constant'
     *  Constant: '<S79>/Constant'
     */
    CAVE_MachE_sil_test_B.DiffTrq = CAVE_MachE_sil_test_P.Constant_Value_eh *
      CAVE_MachE_sil_test_P.Constant_Value_ib;

    /* Gain: '<S79>/Gain10' */
    CAVE_MachE_sil_test_B.Gain10 = CAVE_MachE_sil_test_P.Gain10_Gain *
      CAVE_MachE_sil_test_B.DiffTrq;

    /* Saturate: '<S73>/Saturation1' */
    if (0.0 > CAVE_MachE_sil_test_P.Saturation1_UpperSat_nu) {
      CAVE_MachE_sil_test_B.Saturation1_i =
        CAVE_MachE_sil_test_P.Saturation1_UpperSat_nu;
    } else if (0.0 < CAVE_MachE_sil_test_P.Saturation1_LowerSat_f) {
      CAVE_MachE_sil_test_B.Saturation1_i =
        CAVE_MachE_sil_test_P.Saturation1_LowerSat_f;
    } else {
      CAVE_MachE_sil_test_B.Saturation1_i = 0.0;
    }

    /* End of Saturate: '<S73>/Saturation1' */

    /* Constant: '<S75>/Zero1' */
    CAVE_MachE_sil_test_B.BrakeIFTrq_Reg_trgRR =
      CAVE_MachE_sil_test_P.Zero1_Value_jp;

    /* Constant: '<S75>/Zero2' */
    CAVE_MachE_sil_test_B.BrakeIFTrq_DriveSrc_trgd3 =
      CAVE_MachE_sil_test_P.Zero2_Value_g;

    /* Constant: '<S75>/Zero5' */
    CAVE_MachE_sil_test_B.BrakeIFTrq_PBRR = CAVE_MachE_sil_test_P.Zero5_Value_j;
  }

  /* Product: '<S94>/Divide' */
  CAVE_MachE_sil_test_B.Divide_i = CAVE_MachE_sil_test_B.Sum3_d /
    CAVE_MachE_sil_test_B.Switch_cw;

  /* Gain: '<S76>/rads2rpm' */
  CAVE_MachE_sil_test_B.rads2rpm[0] = CAVE_MachE_sil_test_P.rads2rpm_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[0];
  CAVE_MachE_sil_test_B.rads2rpm[1] = CAVE_MachE_sil_test_P.rads2rpm_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[1];
  CAVE_MachE_sil_test_B.rads2rpm[2] = CAVE_MachE_sil_test_P.rads2rpm_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[2];
  CAVE_MachE_sil_test_B.rads2rpm[3] = CAVE_MachE_sil_test_P.rads2rpm_Gain *
    CAVE_MachE_sil_test_B.Saturation_fw[3];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* SignalConversion generated from: '<S106>/ SFunction ' incorporates:
     *  Chart: '<S102>/Band-Aid'
     */
    CAVE_MachE_sil_test_B.TmpSignalConversionAtSFunctionInport6[0] =
      CAVE_MachE_sil_test_B.LF;
    CAVE_MachE_sil_test_B.TmpSignalConversionAtSFunctionInport6[1] =
      CAVE_MachE_sil_test_B.RF;
    CAVE_MachE_sil_test_B.TmpSignalConversionAtSFunctionInport6[2] =
      CAVE_MachE_sil_test_B.LR;
    CAVE_MachE_sil_test_B.TmpSignalConversionAtSFunctionInport6[3] =
      CAVE_MachE_sil_test_B.RR;

    /* Chart: '<S102>/Band-Aid' incorporates:
     *  Constant: '<S102>/SettleTime_s'
     *  Constant: '<S102>/VehSpdThr_kph'
     *  Constant: '<S102>/dStop'
     *  Constant: '<S4>/Constant'
     */
    if (CAVE_MachE_sil_test_DW.temporalCounter_i1 < MAX_uint32_T) {
      CAVE_MachE_sil_test_DW.temporalCounter_i1++;
    }

    /* Gateway: Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid */
    /* During: Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid */
    if (CAVE_MachE_sil_test_DW.is_active_c9_CAVE_MachE_sil_test == 0U) {
      /* Entry: Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid */
      CAVE_MachE_sil_test_DW.is_active_c9_CAVE_MachE_sil_test = 1U;

      /* Entry Internal: Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/Band-Aid/Band-Aid */
      /* Transition: '<S106>:27' */
      CAVE_MachE_sil_test_DW.is_c9_CAVE_MachE_sil_test =
        CAVE_MachE_sil_test_IN_dStop;

      /* Entry 'dStop': '<S106>:24' */
      CAVE_MachE_sil_test_B.OutMode = 0.0;
      CAVE_MachE_sil_test_B.State = 3.0;
    } else if (CAVE_MachE_sil_test_DW.is_c9_CAVE_MachE_sil_test ==
               CAVE_MachE_sil_test_IN_Running) {
      /* During 'Running': '<S106>:23' */
      if (CAVE_MachE_sil_test_P.dStop_Value != 0.0) {
        /* Transition: '<S106>:26' */
        /* Exit Internal 'Running': '<S106>:23' */
        /* Exit Internal 'SpeedMode': '<S106>:8' */
        CAVE_MachE_sil_test_DW.is_SpeedMode =
          CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;
        CAVE_MachE_sil_test_DW.is_Running =
          CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;
        CAVE_MachE_sil_test_DW.is_c9_CAVE_MachE_sil_test =
          CAVE_MachE_sil_test_IN_dStop;

        /* Entry 'dStop': '<S106>:24' */
        CAVE_MachE_sil_test_B.OutMode = 0.0;
        CAVE_MachE_sil_test_B.State = 3.0;
      } else if (CAVE_MachE_sil_test_DW.is_Running ==
                 CAVE_MachE_sil_test_IN_SpeedMode) {
        CAVE_MachE_sil_test_B.OutMode = 1.0;

        /* During 'SpeedMode': '<S106>:8' */
        sf_internal_predicateOutput = false;
        y = 0;
        exitg1 = false;
        while ((!exitg1) && (y < 4)) {
          if (!(CAVE_MachE_sil_test_B.rads2rpm[y] <
                CAVE_MachE_sil_test_P.SpdLatchThr_rpm_Value)) {
            y++;
          } else {
            sf_internal_predicateOutput = true;
            exitg1 = true;
          }
        }

        sf_internal_predicateOutput = (sf_internal_predicateOutput &&
          (CAVE_MachE_sil_test_B.VehSpdKph <
           CAVE_MachE_sil_test_P.VehSpdThr_kph_Value) &&
          (CAVE_MachE_sil_test_P.Constant_Value_k != 0.0));
        if (sf_internal_predicateOutput) {
          /* Transition: '<S106>:6' */
          /* Exit Internal 'SpeedMode': '<S106>:8' */
          CAVE_MachE_sil_test_DW.is_SpeedMode =
            CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;
          CAVE_MachE_sil_test_DW.is_Running = CAVE_MachE_sil_test_IN_TorqueMode;
          CAVE_MachE_sil_test_DW.temporalCounter_i1 = 0U;

          /* Entry 'TorqueMode': '<S106>:1' */
          CAVE_MachE_sil_test_B.OutMode = 0.0;
        } else if (CAVE_MachE_sil_test_DW.is_SpeedMode ==
                   CAVE_MachE_sil_test_IN_Blend) {
          /* During 'Blend': '<S106>:3' */
          /* Transition: '<S106>:5' */
          CAVE_MachE_sil_test_DW.is_SpeedMode = CAVE_MachE_sil_test_IN_Driving;

          /* Entry 'Driving': '<S106>:4' */
        } else {
          /* During 'Driving': '<S106>:4' */
          CAVE_MachE_sil_test_B.OutSpd[0] = CAVE_MachE_sil_test_B.rads2rpm[0];
          CAVE_MachE_sil_test_B.OutSpd[1] = CAVE_MachE_sil_test_B.rads2rpm[1];
          CAVE_MachE_sil_test_B.OutSpd[2] = CAVE_MachE_sil_test_B.rads2rpm[2];
          CAVE_MachE_sil_test_B.OutSpd[3] = CAVE_MachE_sil_test_B.rads2rpm[3];
          CAVE_MachE_sil_test_B.State = 0.2;
        }
      } else {
        CAVE_MachE_sil_test_B.OutMode = 0.0;

        /* During 'TorqueMode': '<S106>:1' */
        if ((CAVE_MachE_sil_test_DW.temporalCounter_i1 >= (uint32_T)ceil
             (CAVE_MachE_sil_test_P.SettleTime_s_Value * 1000.0)) ||
            (!(CAVE_MachE_sil_test_P.Constant_Value_k != 0.0))) {
          /* Transition: '<S106>:7' */
          CAVE_MachE_sil_test_DW.is_Running = CAVE_MachE_sil_test_IN_SpeedMode;

          /* Entry 'SpeedMode': '<S106>:8' */
          CAVE_MachE_sil_test_B.OutMode = 1.0;
          CAVE_MachE_sil_test_DW.is_SpeedMode = CAVE_MachE_sil_test_IN_Blend;

          /* Entry 'Blend': '<S106>:3' */
          CAVE_MachE_sil_test_B.OutSpd[0] = CAVE_MachE_sil_test_B.rads2rpm[0];
          CAVE_MachE_sil_test_B.OutSpd[1] = CAVE_MachE_sil_test_B.rads2rpm[1];
          CAVE_MachE_sil_test_B.OutSpd[2] = CAVE_MachE_sil_test_B.rads2rpm[2];
          CAVE_MachE_sil_test_B.OutSpd[3] = CAVE_MachE_sil_test_B.rads2rpm[3];
          CAVE_MachE_sil_test_B.State = 0.1;
        } else {
          CAVE_MachE_sil_test_B.State = 1.0;
        }
      }
    } else {
      CAVE_MachE_sil_test_B.OutMode = 0.0;

      /* During 'dStop': '<S106>:24' */
      if (!(CAVE_MachE_sil_test_P.dStop_Value != 0.0)) {
        /* Transition: '<S106>:25' */
        CAVE_MachE_sil_test_DW.is_c9_CAVE_MachE_sil_test =
          CAVE_MachE_sil_test_IN_Running;

        /* Entry Internal 'Running': '<S106>:23' */
        /* Transition: '<S106>:9' */
        CAVE_MachE_sil_test_DW.is_Running = CAVE_MachE_sil_test_IN_SpeedMode;

        /* Entry 'SpeedMode': '<S106>:8' */
        CAVE_MachE_sil_test_B.OutMode = 1.0;

        /* Entry Internal 'SpeedMode': '<S106>:8' */
        /* Transition: '<S106>:11' */
        CAVE_MachE_sil_test_DW.is_SpeedMode = CAVE_MachE_sil_test_IN_Blend;

        /* Entry 'Blend': '<S106>:3' */
        CAVE_MachE_sil_test_B.OutSpd[0] = CAVE_MachE_sil_test_B.rads2rpm[0];
        CAVE_MachE_sil_test_B.OutSpd[1] = CAVE_MachE_sil_test_B.rads2rpm[1];
        CAVE_MachE_sil_test_B.OutSpd[2] = CAVE_MachE_sil_test_B.rads2rpm[2];
        CAVE_MachE_sil_test_B.OutSpd[3] = CAVE_MachE_sil_test_B.rads2rpm[3];
        CAVE_MachE_sil_test_B.State = 0.1;
      }
    }

    /* Constant: '<S102>/BrkLatchThr_Nm' */
    CAVE_MachE_sil_test_B.BrkLatchThr_Nm =
      CAVE_MachE_sil_test_P.BrkLatchThr_Nm_Value;

    /* Switch: '<S76>/ModeOut' incorporates:
     *  Constant: '<S103>/SpTqLF'
     *  Constant: '<S103>/SpTqLR'
     *  Constant: '<S103>/SpTqRF'
     *  Constant: '<S103>/SpTqRR'
     *  Constant: '<S76>/MasterSw'
     */
    if (CAVE_MachE_sil_test_P.MasterSw_Value >
        CAVE_MachE_sil_test_P.ModeOut_Threshold) {
      CAVE_MachE_sil_test_B.ModeOut[0] = CAVE_MachE_sil_test_B.OutMode;
      CAVE_MachE_sil_test_B.ModeOut[1] = CAVE_MachE_sil_test_B.OutMode;
      CAVE_MachE_sil_test_B.ModeOut[2] = CAVE_MachE_sil_test_B.OutMode;
      CAVE_MachE_sil_test_B.ModeOut[3] = CAVE_MachE_sil_test_B.OutMode;
    } else {
      CAVE_MachE_sil_test_B.ModeOut[0] = CAVE_MachE_sil_test_P.SpTqRR_Value;
      CAVE_MachE_sil_test_B.ModeOut[1] = CAVE_MachE_sil_test_P.SpTqLR_Value;
      CAVE_MachE_sil_test_B.ModeOut[2] = CAVE_MachE_sil_test_P.SpTqRF_Value;
      CAVE_MachE_sil_test_B.ModeOut[3] = CAVE_MachE_sil_test_P.SpTqLF_Value;
    }

    /* End of Switch: '<S76>/ModeOut' */

    /* Switch: '<S76>/SpeedOut' incorporates:
     *  Constant: '<S105>/RefLF_NmORrpm'
     *  Constant: '<S105>/RefLR_NmORrpm'
     *  Constant: '<S105>/RefRF_NmORrpm'
     *  Constant: '<S105>/RefRR_NmORrpm'
     *  Constant: '<S76>/MasterSw'
     */
    if (CAVE_MachE_sil_test_P.MasterSw_Value >
        CAVE_MachE_sil_test_P.SpeedOut_Threshold) {
      CAVE_MachE_sil_test_B.SpeedOut[0] = CAVE_MachE_sil_test_B.OutSpd[0];
      CAVE_MachE_sil_test_B.SpeedOut[1] = CAVE_MachE_sil_test_B.OutSpd[1];
      CAVE_MachE_sil_test_B.SpeedOut[2] = CAVE_MachE_sil_test_B.OutSpd[2];
      CAVE_MachE_sil_test_B.SpeedOut[3] = CAVE_MachE_sil_test_B.OutSpd[3];
    } else {
      CAVE_MachE_sil_test_B.SpeedOut[0] =
        CAVE_MachE_sil_test_P.RefLF_NmORrpm_Value;
      CAVE_MachE_sil_test_B.SpeedOut[1] =
        CAVE_MachE_sil_test_P.RefRF_NmORrpm_Value;
      CAVE_MachE_sil_test_B.SpeedOut[2] =
        CAVE_MachE_sil_test_P.RefLR_NmORrpm_Value;
      CAVE_MachE_sil_test_B.SpeedOut[3] =
        CAVE_MachE_sil_test_P.RefRR_NmORrpm_Value;
    }

    /* End of Switch: '<S76>/SpeedOut' */

    /* RateLimiter: '<S76>/RateLimSpd' */
    Bias = CAVE_MachE_sil_test_B.SpeedOut[0] - CAVE_MachE_sil_test_DW.PrevY[0];
    if (Bias > CAVE_MachE_sil_test_P.RateLimSpd_RisingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[0] = CAVE_MachE_sil_test_DW.PrevY[0] +
        CAVE_MachE_sil_test_P.RateLimSpd_RisingLim;
    } else if (Bias < CAVE_MachE_sil_test_P.RateLimSpd_FallingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[0] = CAVE_MachE_sil_test_DW.PrevY[0] +
        CAVE_MachE_sil_test_P.RateLimSpd_FallingLim;
    } else {
      CAVE_MachE_sil_test_B.RateLimSpd[0] = CAVE_MachE_sil_test_B.SpeedOut[0];
    }

    CAVE_MachE_sil_test_DW.PrevY[0] = CAVE_MachE_sil_test_B.RateLimSpd[0];

    /* Saturate: '<S76>/SatSpd' */
    Bias = CAVE_MachE_sil_test_B.RateLimSpd[0];
    if (Bias > CAVE_MachE_sil_test_P.SatSpd_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatSpd_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatSpd_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatSpd_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.Spd_d[0] = Bias;

    /* RateLimiter: '<S76>/RateLimSpd' */
    Bias = CAVE_MachE_sil_test_B.SpeedOut[1] - CAVE_MachE_sil_test_DW.PrevY[1];
    if (Bias > CAVE_MachE_sil_test_P.RateLimSpd_RisingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[1] = CAVE_MachE_sil_test_DW.PrevY[1] +
        CAVE_MachE_sil_test_P.RateLimSpd_RisingLim;
    } else if (Bias < CAVE_MachE_sil_test_P.RateLimSpd_FallingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[1] = CAVE_MachE_sil_test_DW.PrevY[1] +
        CAVE_MachE_sil_test_P.RateLimSpd_FallingLim;
    } else {
      CAVE_MachE_sil_test_B.RateLimSpd[1] = CAVE_MachE_sil_test_B.SpeedOut[1];
    }

    CAVE_MachE_sil_test_DW.PrevY[1] = CAVE_MachE_sil_test_B.RateLimSpd[1];

    /* Saturate: '<S76>/SatSpd' */
    Bias = CAVE_MachE_sil_test_B.RateLimSpd[1];
    if (Bias > CAVE_MachE_sil_test_P.SatSpd_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatSpd_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatSpd_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatSpd_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.Spd_d[1] = Bias;

    /* RateLimiter: '<S76>/RateLimSpd' */
    Bias = CAVE_MachE_sil_test_B.SpeedOut[2] - CAVE_MachE_sil_test_DW.PrevY[2];
    if (Bias > CAVE_MachE_sil_test_P.RateLimSpd_RisingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[2] = CAVE_MachE_sil_test_DW.PrevY[2] +
        CAVE_MachE_sil_test_P.RateLimSpd_RisingLim;
    } else if (Bias < CAVE_MachE_sil_test_P.RateLimSpd_FallingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[2] = CAVE_MachE_sil_test_DW.PrevY[2] +
        CAVE_MachE_sil_test_P.RateLimSpd_FallingLim;
    } else {
      CAVE_MachE_sil_test_B.RateLimSpd[2] = CAVE_MachE_sil_test_B.SpeedOut[2];
    }

    CAVE_MachE_sil_test_DW.PrevY[2] = CAVE_MachE_sil_test_B.RateLimSpd[2];

    /* Saturate: '<S76>/SatSpd' */
    Bias = CAVE_MachE_sil_test_B.RateLimSpd[2];
    if (Bias > CAVE_MachE_sil_test_P.SatSpd_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatSpd_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatSpd_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatSpd_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.Spd_d[2] = Bias;

    /* RateLimiter: '<S76>/RateLimSpd' */
    Bias = CAVE_MachE_sil_test_B.SpeedOut[3] - CAVE_MachE_sil_test_DW.PrevY[3];
    if (Bias > CAVE_MachE_sil_test_P.RateLimSpd_RisingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[3] = CAVE_MachE_sil_test_DW.PrevY[3] +
        CAVE_MachE_sil_test_P.RateLimSpd_RisingLim;
    } else if (Bias < CAVE_MachE_sil_test_P.RateLimSpd_FallingLim) {
      CAVE_MachE_sil_test_B.RateLimSpd[3] = CAVE_MachE_sil_test_DW.PrevY[3] +
        CAVE_MachE_sil_test_P.RateLimSpd_FallingLim;
    } else {
      CAVE_MachE_sil_test_B.RateLimSpd[3] = CAVE_MachE_sil_test_B.SpeedOut[3];
    }

    CAVE_MachE_sil_test_DW.PrevY[3] = CAVE_MachE_sil_test_B.RateLimSpd[3];

    /* Saturate: '<S76>/SatSpd' */
    Bias = CAVE_MachE_sil_test_B.RateLimSpd[3];
    if (Bias > CAVE_MachE_sil_test_P.SatSpd_UpperSat) {
      Bias = CAVE_MachE_sil_test_P.SatSpd_UpperSat;
    } else {
      if (Bias < CAVE_MachE_sil_test_P.SatSpd_LowerSat) {
        Bias = CAVE_MachE_sil_test_P.SatSpd_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.Spd_d[3] = Bias;

    /* MultiPortSwitch: '<S76>/ModeSwitch' incorporates:
     *  Constant: '<S76>/ZeroTorque'
     */
    if ((int32_T)CAVE_MachE_sil_test_B.OutMode == 0) {
      CAVE_MachE_sil_test_B.ModeSwitch[0] =
        CAVE_MachE_sil_test_P.ZeroTorque_Value;
      CAVE_MachE_sil_test_B.ModeSwitch[1] =
        CAVE_MachE_sil_test_P.ZeroTorque_Value;
      CAVE_MachE_sil_test_B.ModeSwitch[2] =
        CAVE_MachE_sil_test_P.ZeroTorque_Value;
      CAVE_MachE_sil_test_B.ModeSwitch[3] =
        CAVE_MachE_sil_test_P.ZeroTorque_Value;
    } else {
      CAVE_MachE_sil_test_B.ModeSwitch[0] = CAVE_MachE_sil_test_B.Spd_d[0];
      CAVE_MachE_sil_test_B.ModeSwitch[1] = CAVE_MachE_sil_test_B.Spd_d[1];
      CAVE_MachE_sil_test_B.ModeSwitch[2] = CAVE_MachE_sil_test_B.Spd_d[2];
      CAVE_MachE_sil_test_B.ModeSwitch[3] = CAVE_MachE_sil_test_B.Spd_d[3];
    }

    /* End of MultiPortSwitch: '<S76>/ModeSwitch' */

    /* MATLAB Function: '<S107>/Defloater' */
    CAVE_MachE_sil_test_Defloater(CAVE_MachE_sil_test_B.ModeSwitch[0],
      &CAVE_MachE_sil_test_B.sf_Defloater);

    /* MATLAB Function: '<S108>/Defloater' */
    CAVE_MachE_sil_test_Defloater(CAVE_MachE_sil_test_B.ModeSwitch[1],
      &CAVE_MachE_sil_test_B.sf_Defloater_l);

    /* MATLAB Function: '<S109>/Defloater' */
    CAVE_MachE_sil_test_Defloater(CAVE_MachE_sil_test_B.ModeSwitch[2],
      &CAVE_MachE_sil_test_B.sf_Defloater_d);

    /* MATLAB Function: '<S110>/Defloater' */
    CAVE_MachE_sil_test_Defloater(CAVE_MachE_sil_test_B.ModeSwitch[3],
      &CAVE_MachE_sil_test_B.sf_Defloater_e);

    /* MATLAB Function: '<S104>/MATLAB Function' incorporates:
     *  Constant: '<S76>/SystemActive'
     */
    /* MATLAB Function 'Vehicle Under Test/Vehicle/Rototest and ORNL Interface/To Rototest with Band-Aid/RotoTestEthTx/MATLAB Function': '<S111>:1' */
    /* '<S111>:1:3' */
    y = 0;
    if ((CAVE_MachE_sil_test_B.ModeOut[0] == 1.0) &&
        (CAVE_MachE_sil_test_P.SystemActive_Value == 0)) {
      /* '<S111>:1:4' */
      /* '<S111>:1:5' */
      y = 30;
    } else if ((!(CAVE_MachE_sil_test_B.ModeOut[0] == 0.0)) ||
               (CAVE_MachE_sil_test_P.SystemActive_Value != 0)) {
      if ((CAVE_MachE_sil_test_B.ModeOut[0] == 1.0) &&
          (CAVE_MachE_sil_test_P.SystemActive_Value == 1)) {
        /* '<S111>:1:8' */
        /* '<S111>:1:9' */
        y = 31;
      } else {
        if ((CAVE_MachE_sil_test_B.ModeOut[0] == 0.0) &&
            (CAVE_MachE_sil_test_P.SystemActive_Value == 1)) {
          /* '<S111>:1:10' */
          /* '<S111>:1:11' */
          y = 1;
        }
      }
    } else {
      /* '<S111>:1:6' */
      /* '<S111>:1:7' */
    }

    /* '<S111>:1:14' */
    CAVE_MachE_sil_test_B.y = (uint8_T)y;

    /* End of MATLAB Function: '<S104>/MATLAB Function' */

    /* Gain: '<S112>/Bit7' */
    CAVE_MachE_sil_test_B.Reserved = 0U;

    /* Gain: '<S112>/Bit6' */
    CAVE_MachE_sil_test_B.Reserved_o = 0U;

    /* Gain: '<S112>/Bit5' */
    CAVE_MachE_sil_test_B.Reserved_j = 0U;

    /* Gain: '<S112>/Bit4' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = floor
      (CAVE_MachE_sil_test_P.Bit4_Gain * CAVE_MachE_sil_test_B.ModeOut[0]);
    if (rtIsNaN(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2) ||
        rtIsInf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2)) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = 0.0;
    } else {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = fmod
        (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2, 256.0);
    }

    CAVE_MachE_sil_test_B.SpTqRR = (uint8_T)
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 < 0.0 ? (int32_T)
       (uint8_T)-(int8_T)(uint8_T)
       -rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 : (int32_T)
       (uint8_T)rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2);

    /* End of Gain: '<S112>/Bit4' */

    /* Gain: '<S112>/Bit3' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = floor
      (CAVE_MachE_sil_test_P.Bit3_Gain * CAVE_MachE_sil_test_B.ModeOut[1]);
    if (rtIsNaN(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2) ||
        rtIsInf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2)) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = 0.0;
    } else {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = fmod
        (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2, 256.0);
    }

    CAVE_MachE_sil_test_B.SpTqLR = (uint8_T)
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 < 0.0 ? (int32_T)
       (uint8_T)-(int8_T)(uint8_T)
       -rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 : (int32_T)
       (uint8_T)rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2);

    /* End of Gain: '<S112>/Bit3' */

    /* Gain: '<S112>/Bit2' */
    CAVE_MachE_sil_test_B.SpTqRF = CAVE_MachE_sil_test_P.Bit2_Gain *
      CAVE_MachE_sil_test_B.ModeOut[2];

    /* Gain: '<S112>/Bit1' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = floor
      (CAVE_MachE_sil_test_P.Bit1_Gain * CAVE_MachE_sil_test_B.ModeOut[3]);
    if (rtIsNaN(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2) ||
        rtIsInf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2)) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = 0.0;
    } else {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = fmod
        (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2, 256.0);
    }

    CAVE_MachE_sil_test_B.SpTqLF = (uint8_T)
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 < 0.0 ? (int32_T)
       (uint8_T)-(int8_T)(uint8_T)
       -rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 : (int32_T)
       (uint8_T)rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2);

    /* End of Gain: '<S112>/Bit1' */

    /* Gain: '<S112>/Bit0' incorporates:
     *  Constant: '<S76>/SystemActive'
     */
    CAVE_MachE_sil_test_B.SystemActive = (uint8_T)(((uint32_T)
      CAVE_MachE_sil_test_P.Bit0_Gain * CAVE_MachE_sil_test_P.SystemActive_Value)
      >> 7);

    /* Sum: '<S112>/Add' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = floor((((real_T)
      ((((CAVE_MachE_sil_test_B.Reserved + CAVE_MachE_sil_test_B.Reserved_o) +
         CAVE_MachE_sil_test_B.Reserved_j) + CAVE_MachE_sil_test_B.SpTqRR) +
       CAVE_MachE_sil_test_B.SpTqLR) + CAVE_MachE_sil_test_B.SpTqRF) + (real_T)
      CAVE_MachE_sil_test_B.SpTqLF) + (real_T)CAVE_MachE_sil_test_B.SystemActive);
    if (rtIsNaN(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2) ||
        rtIsInf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2)) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = 0.0;
    } else {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = fmod
        (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2, 256.0);
    }

    CAVE_MachE_sil_test_B.SystemCtrlBits = (uint8_T)
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 < 0.0 ? (int32_T)
       (uint8_T)-(int8_T)(uint8_T)
       -rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 : (int32_T)
       (uint8_T)rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2);

    /* End of Sum: '<S112>/Add' */

    /* UnitDelay: '<S113>/Output' */
    CAVE_MachE_sil_test_B.Output = CAVE_MachE_sil_test_DW.Output_DSTATE;

    /* Sum: '<S118>/FixPt Sum1' incorporates:
     *  Constant: '<S118>/FixPt Constant'
     */
    CAVE_MachE_sil_test_B.FixPtSum1 = (uint8_T)((uint32_T)
      CAVE_MachE_sil_test_B.Output + CAVE_MachE_sil_test_P.FixPtConstant_Value);

    /* Switch: '<S119>/FixPt Switch' incorporates:
     *  Constant: '<S119>/Constant'
     */
    if (CAVE_MachE_sil_test_B.FixPtSum1 >
        CAVE_MachE_sil_test_P.WrapToZero_Threshold) {
      CAVE_MachE_sil_test_B.FixPtSwitch =
        CAVE_MachE_sil_test_P.Constant_Value_ej;
    } else {
      CAVE_MachE_sil_test_B.FixPtSwitch = CAVE_MachE_sil_test_B.FixPtSum1;
    }

    /* End of Switch: '<S119>/FixPt Switch' */

    /* S-Function (any2byte): '<S104>/uint16 (unsigned 16) 2' incorporates:
     *  Constant: '<S104>/ProtocolVer'
     */

    /* Pack: <S104>/uint16 (unsigned 16) 2 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint16unsigned162[0],
                  &CAVE_MachE_sil_test_P.ProtocolVer_Value,
                  2);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 1' */

    /* Pack: <S104>/uint8 (unsigned 8) 1 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned81,
                  &CAVE_MachE_sil_test_B.Output,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 2' incorporates:
     *  Constant: '<S104>/Constant3'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 2 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned82,
                  &CAVE_MachE_sil_test_P.Constant3_Value_m,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 3' incorporates:
     *  Constant: '<S104>/Constant2'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 3 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned83,
                  &CAVE_MachE_sil_test_P.Constant2_Value_j,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 4' incorporates:
     *  Constant: '<S104>/Constant1'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 4 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned84,
                  &CAVE_MachE_sil_test_P.Constant1_Value_o1,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 5' incorporates:
     *  Constant: '<S104>/SystemCtrlBits2'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 5 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned85,
                  &CAVE_MachE_sil_test_P.SystemCtrlBits2_Value,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 7' incorporates:
     *  Constant: '<S104>/ID2'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 7 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned87,
                  &CAVE_MachE_sil_test_P.ID2_Value,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 8' incorporates:
     *  Constant: '<S104>/ID1'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 8 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned88,
                  &CAVE_MachE_sil_test_P.ID1_Value,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 9' */

    /* Pack: <S104>/uint8 (unsigned 8) 9 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned89,
                  &CAVE_MachE_sil_test_B.y,
                  1);

    /* S-Function (any2byte): '<S104>/uint8 (unsigned 8) 6' incorporates:
     *  Constant: '<S104>/Constant'
     */

    /* Pack: <S104>/uint8 (unsigned 8) 6 */
    (void) memcpy(&CAVE_MachE_sil_test_B.uint8unsigned86,
                  &CAVE_MachE_sil_test_P.Constant_Value_f,
                  1);

    /* SignalConversion generated from: '<S120>/Vector Concatenate1' incorporates:
     *  Constant: '<S2>/Friction'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate1[0] =
      CAVE_MachE_sil_test_P.Friction_Value[0];

    /* SignalConversion generated from: '<S120>/Vector Concatenate1' incorporates:
     *  Constant: '<S2>/Friction'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate1[1] =
      CAVE_MachE_sil_test_P.Friction_Value[1];

    /* SignalConversion generated from: '<S120>/Vector Concatenate1' incorporates:
     *  Constant: '<S2>/Friction'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate1[2] =
      CAVE_MachE_sil_test_P.Friction_Value[2];

    /* SignalConversion generated from: '<S120>/Vector Concatenate1' incorporates:
     *  Constant: '<S2>/Friction'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate1[3] =
      CAVE_MachE_sil_test_P.Friction_Value[3];

    /* SignalConversion generated from: '<S122>/Vector Concatenate1' incorporates:
     *  Constant: '<S122>/Constant'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate2_b[2] =
      CAVE_MachE_sil_test_P.Constant_Value_jx;

    /* SignalConversion generated from: '<S122>/Vector Concatenate1' incorporates:
     *  Constant: '<S122>/Constant'
     */
    CAVE_MachE_sil_test_B.VectorConcatenate2_b[3] =
      CAVE_MachE_sil_test_P.Constant_Value_jx;

    /* Backlash: '<S130>/Backlash' */
    Bias = CAVE_MachE_sil_test_P.KinematicSteering_Db / 2.0;
    if (CAVE_MachE_sil_test_B.SteeringCmd < CAVE_MachE_sil_test_DW.PrevY_e -
        Bias) {
      CAVE_MachE_sil_test_B.Backlash = CAVE_MachE_sil_test_B.SteeringCmd + Bias;
    } else if (CAVE_MachE_sil_test_B.SteeringCmd <=
               CAVE_MachE_sil_test_DW.PrevY_e + Bias) {
      CAVE_MachE_sil_test_B.Backlash = CAVE_MachE_sil_test_DW.PrevY_e;
    } else {
      CAVE_MachE_sil_test_B.Backlash = CAVE_MachE_sil_test_B.SteeringCmd - Bias;
    }

    /* End of Backlash: '<S130>/Backlash' */

    /* Constant: '<S130>/Constant' */
    CAVE_MachE_sil_test_B.PwrLoss = CAVE_MachE_sil_test_P.Constant_Value_a;

    /* Constant: '<S131>/Constant' */
    CAVE_MachE_sil_test_B.InstStrgRatio =
      CAVE_MachE_sil_test_P.KinematicSteering_StrgRatio;

    /* Saturate: '<S130>/Saturation' */
    Bias = -CAVE_MachE_sil_test_P.KinematicSteering_StrgRng;
    if (CAVE_MachE_sil_test_B.Backlash >
        CAVE_MachE_sil_test_P.KinematicSteering_StrgRng) {
      CAVE_MachE_sil_test_B.Saturation_oi =
        CAVE_MachE_sil_test_P.KinematicSteering_StrgRng;
    } else if (CAVE_MachE_sil_test_B.Backlash < Bias) {
      CAVE_MachE_sil_test_B.Saturation_oi = Bias;
    } else {
      CAVE_MachE_sil_test_B.Saturation_oi = CAVE_MachE_sil_test_B.Backlash;
    }

    /* End of Saturate: '<S130>/Saturation' */

    /* Gain: '<S131>/Gain' */
    Bias = 1.0 / CAVE_MachE_sil_test_P.KinematicSteering_StrgRatio;
    CAVE_MachE_sil_test_B.Gain_e = Bias * CAVE_MachE_sil_test_B.Saturation_oi;

    /* Gain: '<S131>/Gain1' */
    Bias = 1.0 / CAVE_MachE_sil_test_P.KinematicSteering_StrgRatio;
    CAVE_MachE_sil_test_B.Gain1_g = Bias * 0.0;

    /* Sum: '<S132>/Add' */
    CAVE_MachE_sil_test_B.Add_bk = 0.0;

    /* Gain: '<S131>/Gain2' */
    Bias = 1.0 / CAVE_MachE_sil_test_P.KinematicSteering_StrgRatio;
    CAVE_MachE_sil_test_B.TrqIn = Bias * CAVE_MachE_sil_test_B.Add_bk;

    /* UnaryMinus: '<S130>/Unary Minus' */
    CAVE_MachE_sil_test_B.TrqIn_p = -CAVE_MachE_sil_test_B.TrqIn;

    /* UnaryMinus: '<S129>/Unary Minus1' */
    CAVE_MachE_sil_test_B.UnaryMinus1 = -CAVE_MachE_sil_test_B.Gain_e;

    /* Switch: '<S129>/Switch' incorporates:
     *  Constant: '<S129>/index'
     */
    if (CAVE_MachE_sil_test_P.index_Value >
        CAVE_MachE_sil_test_P.Switch_Threshold_d) {
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[0] =
        CAVE_MachE_sil_test_B.Gain_e;
    } else {
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[0] =
        CAVE_MachE_sil_test_B.UnaryMinus1;
    }

    /* End of Switch: '<S129>/Switch' */

    /* UnaryMinus: '<S129>/Unary Minus' */
    CAVE_MachE_sil_test_B.UnaryMinus_k = -CAVE_MachE_sil_test_B.Gain_e;

    /* Switch: '<S129>/Switch1' incorporates:
     *  Constant: '<S129>/index'
     */
    if (CAVE_MachE_sil_test_P.index_Value >
        CAVE_MachE_sil_test_P.Switch1_Threshold_f) {
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[1] =
        CAVE_MachE_sil_test_B.Gain_e;
    } else {
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[1] =
        CAVE_MachE_sil_test_B.UnaryMinus_k;
    }

    /* End of Switch: '<S129>/Switch1' */
  }

  /* SignalConversion generated from: '<S327>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n[0] =
    CAVE_MachE_sil_test_B.Integrator_a;

  /* SignalConversion generated from: '<S327>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n[1] =
    CAVE_MachE_sil_test_B.Integrator_l;

  /* SignalConversion generated from: '<S327>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n[2] =
    CAVE_MachE_sil_test_B.Integrator_fm;

  /* SignalConversion generated from: '<S327>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n[3] =
    CAVE_MachE_sil_test_B.Integrator_b;

  /* DeadZone: '<S313>/Dead Zone2' */
  if (CAVE_MachE_sil_test_B.VectorConcatenate_n[0] >
      CAVE_MachE_sil_test_P.DeadZone2_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[0] -
      CAVE_MachE_sil_test_P.DeadZone2_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_n[0] >=
             CAVE_MachE_sil_test_P.DeadZone2_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[0] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[0] -
      CAVE_MachE_sil_test_P.DeadZone2_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_n[1] >
      CAVE_MachE_sil_test_P.DeadZone2_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[1] -
      CAVE_MachE_sil_test_P.DeadZone2_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_n[1] >=
             CAVE_MachE_sil_test_P.DeadZone2_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[1] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[1] -
      CAVE_MachE_sil_test_P.DeadZone2_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_n[2] >
      CAVE_MachE_sil_test_P.DeadZone2_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[2] -
      CAVE_MachE_sil_test_P.DeadZone2_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_n[2] >=
             CAVE_MachE_sil_test_P.DeadZone2_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[2] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[2] -
      CAVE_MachE_sil_test_P.DeadZone2_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_n[3] >
      CAVE_MachE_sil_test_P.DeadZone2_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[3] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[3] -
      CAVE_MachE_sil_test_P.DeadZone2_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_n[3] >=
             CAVE_MachE_sil_test_P.DeadZone2_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[3] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[3] =
      CAVE_MachE_sil_test_B.VectorConcatenate_n[3] -
      CAVE_MachE_sil_test_P.DeadZone2_Start;
  }

  /* End of DeadZone: '<S313>/Dead Zone2' */

  /* Integrator: '<S348>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_fu =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_i;

  /* SignalConversion generated from: '<S328>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_pf[0] =
    CAVE_MachE_sil_test_B.Integrator_fu;

  /* Integrator: '<S373>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_n =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_b0;

  /* SignalConversion generated from: '<S328>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_pf[1] =
    CAVE_MachE_sil_test_B.Integrator_n;

  /* Integrator: '<S398>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_e = CAVE_MachE_sil_test_X.Integrator_CSTATE_n;

  /* SignalConversion generated from: '<S328>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_pf[2] =
    CAVE_MachE_sil_test_B.Integrator_e;

  /* Integrator: '<S423>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_d = CAVE_MachE_sil_test_X.Integrator_CSTATE_m;

  /* SignalConversion generated from: '<S328>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_pf[3] =
    CAVE_MachE_sil_test_B.Integrator_d;

  /* DeadZone: '<S313>/Dead Zone3' */
  if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[0] >
      CAVE_MachE_sil_test_P.DeadZone3_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[4] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[0] -
      CAVE_MachE_sil_test_P.DeadZone3_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[0] >=
             CAVE_MachE_sil_test_P.DeadZone3_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[4] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[4] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[0] -
      CAVE_MachE_sil_test_P.DeadZone3_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[1] >
      CAVE_MachE_sil_test_P.DeadZone3_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[5] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[1] -
      CAVE_MachE_sil_test_P.DeadZone3_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[1] >=
             CAVE_MachE_sil_test_P.DeadZone3_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[5] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[5] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[1] -
      CAVE_MachE_sil_test_P.DeadZone3_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[2] >
      CAVE_MachE_sil_test_P.DeadZone3_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[6] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[2] -
      CAVE_MachE_sil_test_P.DeadZone3_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[2] >=
             CAVE_MachE_sil_test_P.DeadZone3_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[6] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[6] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[2] -
      CAVE_MachE_sil_test_P.DeadZone3_Start;
  }

  if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[3] >
      CAVE_MachE_sil_test_P.DeadZone3_End) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[7] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[3] -
      CAVE_MachE_sil_test_P.DeadZone3_End;
  } else if (CAVE_MachE_sil_test_B.VectorConcatenate_pf[3] >=
             CAVE_MachE_sil_test_P.DeadZone3_Start) {
    CAVE_MachE_sil_test_B.VectorConcatenate8[7] = 0.0;
  } else {
    CAVE_MachE_sil_test_B.VectorConcatenate8[7] =
      CAVE_MachE_sil_test_B.VectorConcatenate_pf[3] -
      CAVE_MachE_sil_test_P.DeadZone3_Start;
  }

  /* End of DeadZone: '<S313>/Dead Zone3' */

  /* Integrator: '<S310>/Integrator1' */
  CAVE_MachE_sil_test_B.Integrator1[0] =
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[0];
  CAVE_MachE_sil_test_B.Integrator1[1] =
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[1];
  CAVE_MachE_sil_test_B.Integrator1[2] =
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[2];
  CAVE_MachE_sil_test_B.Integrator1[3] =
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[3];

  /* Saturate: '<S128>/Saturation' */
  Bias = CAVE_MachE_sil_test_B.Integrator1[0];
  if (Bias > CAVE_MachE_sil_test_P.Saturation_UpperSat_nn) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_nn;
  } else {
    if (Bias < CAVE_MachE_sil_test_P.Saturation_LowerSat_d5) {
      Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_d5;
    }
  }

  CAVE_MachE_sil_test_B.Saturation_du[0] = Bias;
  Bias = CAVE_MachE_sil_test_B.Integrator1[1];
  if (Bias > CAVE_MachE_sil_test_P.Saturation_UpperSat_nn) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_nn;
  } else {
    if (Bias < CAVE_MachE_sil_test_P.Saturation_LowerSat_d5) {
      Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_d5;
    }
  }

  CAVE_MachE_sil_test_B.Saturation_du[1] = Bias;
  Bias = CAVE_MachE_sil_test_B.Integrator1[2];
  if (Bias > CAVE_MachE_sil_test_P.Saturation_UpperSat_nn) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_nn;
  } else {
    if (Bias < CAVE_MachE_sil_test_P.Saturation_LowerSat_d5) {
      Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_d5;
    }
  }

  CAVE_MachE_sil_test_B.Saturation_du[2] = Bias;
  Bias = CAVE_MachE_sil_test_B.Integrator1[3];
  if (Bias > CAVE_MachE_sil_test_P.Saturation_UpperSat_nn) {
    Bias = CAVE_MachE_sil_test_P.Saturation_UpperSat_nn;
  } else {
    if (Bias < CAVE_MachE_sil_test_P.Saturation_LowerSat_d5) {
      Bias = CAVE_MachE_sil_test_P.Saturation_LowerSat_d5;
    }
  }

  CAVE_MachE_sil_test_B.Saturation_du[3] = Bias;

  /* End of Saturate: '<S128>/Saturation' */

  /* SignalConversion generated from: '<S313>/Vector Concatenate8' */
  CAVE_MachE_sil_test_B.VectorConcatenate8[8] =
    CAVE_MachE_sil_test_B.Saturation_du[0];
  CAVE_MachE_sil_test_B.VectorConcatenate8[9] =
    CAVE_MachE_sil_test_B.Saturation_du[1];
  CAVE_MachE_sil_test_B.VectorConcatenate8[10] =
    CAVE_MachE_sil_test_B.Saturation_du[2];
  CAVE_MachE_sil_test_B.VectorConcatenate8[11] =
    CAVE_MachE_sil_test_B.Saturation_du[3];

  /* Math: '<S313>/Math Function' */
  for (ibmat = 0; ibmat < 4; ibmat++) {
    CAVE_MachE_sil_test_B.MathFunction_d[3 * ibmat] =
      CAVE_MachE_sil_test_B.VectorConcatenate8[ibmat];
    CAVE_MachE_sil_test_B.MathFunction_d[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.VectorConcatenate8[ibmat + 4];
    CAVE_MachE_sil_test_B.MathFunction_d[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.VectorConcatenate8[ibmat + 8];
  }

  /* End of Math: '<S313>/Math Function' */

  /* Integrator: '<S311>/Integrator1' */
  memcpy(&CAVE_MachE_sil_test_B.Integrator1_d[0],
         &CAVE_MachE_sil_test_X.Integrator1_CSTATE_m[0], 12U * sizeof(real_T));

  /* Selector: '<S128>/Selector3' */
  CAVE_MachE_sil_test_B.CamberAngles[0] = CAVE_MachE_sil_test_B.Integrator1_d[0];
  CAVE_MachE_sil_test_B.CamberAngles[1] = CAVE_MachE_sil_test_B.Integrator1_d[3];
  CAVE_MachE_sil_test_B.CamberAngles[2] = CAVE_MachE_sil_test_B.Integrator1_d[6];
  CAVE_MachE_sil_test_B.CamberAngles[3] = CAVE_MachE_sil_test_B.Integrator1_d[9];

  /* ManualSwitch: '<S128>/Manual Switch6' incorporates:
   *  Constant: '<S128>/Constant3'
   */
  if (CAVE_MachE_sil_test_P.ManualSwitch6_CurrentSetting == 1) {
    CAVE_MachE_sil_test_B.CamberAngles_l[0] =
      CAVE_MachE_sil_test_P.Constant3_Value_o[0];
    CAVE_MachE_sil_test_B.CamberAngles_l[1] =
      CAVE_MachE_sil_test_P.Constant3_Value_o[1];
    CAVE_MachE_sil_test_B.CamberAngles_l[2] =
      CAVE_MachE_sil_test_P.Constant3_Value_o[2];
    CAVE_MachE_sil_test_B.CamberAngles_l[3] =
      CAVE_MachE_sil_test_P.Constant3_Value_o[3];
  } else {
    CAVE_MachE_sil_test_B.CamberAngles_l[0] =
      CAVE_MachE_sil_test_B.CamberAngles[0];
    CAVE_MachE_sil_test_B.CamberAngles_l[1] =
      CAVE_MachE_sil_test_B.CamberAngles[1];
    CAVE_MachE_sil_test_B.CamberAngles_l[2] =
      CAVE_MachE_sil_test_B.CamberAngles[2];
    CAVE_MachE_sil_test_B.CamberAngles_l[3] =
      CAVE_MachE_sil_test_B.CamberAngles[3];
  }

  /* End of ManualSwitch: '<S128>/Manual Switch6' */

  /* Sum: '<S318>/Add2' incorporates:
   *  Constant: '<S318>/Constant1'
   */
  CAVE_MachE_sil_test_B.Add2_k[0] = CAVE_MachE_sil_test_B.CamberAngles_l[0] +
    CAVE_MachE_sil_test_P.Constant1_Value_o;
  CAVE_MachE_sil_test_B.Add2_k[1] = CAVE_MachE_sil_test_B.CamberAngles_l[1] +
    CAVE_MachE_sil_test_P.Constant1_Value_o;
  CAVE_MachE_sil_test_B.Add2_k[2] = CAVE_MachE_sil_test_B.CamberAngles_l[2] +
    CAVE_MachE_sil_test_P.Constant1_Value_o;
  CAVE_MachE_sil_test_B.Add2_k[3] = CAVE_MachE_sil_test_B.CamberAngles_l[3] +
    CAVE_MachE_sil_test_P.Constant1_Value_o;

  /* Reshape: '<S318>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1[0] = CAVE_MachE_sil_test_B.Add2_k[0];
  CAVE_MachE_sil_test_B.Reshape1[1] = CAVE_MachE_sil_test_B.Add2_k[1];
  CAVE_MachE_sil_test_B.Reshape1[2] = CAVE_MachE_sil_test_B.Add2_k[2];
  CAVE_MachE_sil_test_B.Reshape1[3] = CAVE_MachE_sil_test_B.Add2_k[3];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S318>/Reshape2' incorporates:
     *  Constant: '<S318>/Constant3'
     */
    CAVE_MachE_sil_test_B.Reshape2[0] = CAVE_MachE_sil_test_P.Constant3_Value_e
      [0];
    CAVE_MachE_sil_test_B.Reshape2[1] = CAVE_MachE_sil_test_P.Constant3_Value_e
      [1];
    CAVE_MachE_sil_test_B.Reshape2[2] = CAVE_MachE_sil_test_P.Constant3_Value_e
      [2];
    CAVE_MachE_sil_test_B.Reshape2[3] = CAVE_MachE_sil_test_P.Constant3_Value_e
      [3];
  }

  /* Selector: '<S128>/Selector2' */
  CAVE_MachE_sil_test_B.WheelAngles[0] = CAVE_MachE_sil_test_B.Integrator1_d[2];
  CAVE_MachE_sil_test_B.WheelAngles[1] = CAVE_MachE_sil_test_B.Integrator1_d[5];
  CAVE_MachE_sil_test_B.WheelAngles[2] = CAVE_MachE_sil_test_B.Integrator1_d[8];
  CAVE_MachE_sil_test_B.WheelAngles[3] = CAVE_MachE_sil_test_B.Integrator1_d[11];

  /* Sum: '<S318>/Add1' incorporates:
   *  Constant: '<S318>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_f[0] = CAVE_MachE_sil_test_B.WheelAngles[0] +
    CAVE_MachE_sil_test_P.Constant2_Value_p[0];

  /* Reshape: '<S318>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape[0] = CAVE_MachE_sil_test_B.Add1_f[0];

  /* Concatenate: '<S318>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3[0] = CAVE_MachE_sil_test_B.Reshape1[0];
  CAVE_MachE_sil_test_B.VectorConcatenate3[1] = CAVE_MachE_sil_test_B.Reshape2[0];
  CAVE_MachE_sil_test_B.VectorConcatenate3[2] = CAVE_MachE_sil_test_B.Reshape[0];

  /* Sum: '<S318>/Add1' incorporates:
   *  Constant: '<S318>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_f[1] = CAVE_MachE_sil_test_B.WheelAngles[1] +
    CAVE_MachE_sil_test_P.Constant2_Value_p[1];

  /* Reshape: '<S318>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape[1] = CAVE_MachE_sil_test_B.Add1_f[1];

  /* Concatenate: '<S318>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3[3] = CAVE_MachE_sil_test_B.Reshape1[1];
  CAVE_MachE_sil_test_B.VectorConcatenate3[4] = CAVE_MachE_sil_test_B.Reshape2[1];
  CAVE_MachE_sil_test_B.VectorConcatenate3[5] = CAVE_MachE_sil_test_B.Reshape[1];

  /* Sum: '<S318>/Add1' incorporates:
   *  Constant: '<S318>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_f[2] = CAVE_MachE_sil_test_B.WheelAngles[2] +
    CAVE_MachE_sil_test_P.Constant2_Value_p[2];

  /* Reshape: '<S318>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape[2] = CAVE_MachE_sil_test_B.Add1_f[2];

  /* Concatenate: '<S318>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3[6] = CAVE_MachE_sil_test_B.Reshape1[2];
  CAVE_MachE_sil_test_B.VectorConcatenate3[7] = CAVE_MachE_sil_test_B.Reshape2[2];
  CAVE_MachE_sil_test_B.VectorConcatenate3[8] = CAVE_MachE_sil_test_B.Reshape[2];

  /* Sum: '<S318>/Add1' incorporates:
   *  Constant: '<S318>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_f[3] = CAVE_MachE_sil_test_B.WheelAngles[3] +
    CAVE_MachE_sil_test_P.Constant2_Value_p[3];

  /* Reshape: '<S318>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape[3] = CAVE_MachE_sil_test_B.Add1_f[3];

  /* Concatenate: '<S318>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3[9] = CAVE_MachE_sil_test_B.Reshape1[3];
  CAVE_MachE_sil_test_B.VectorConcatenate3[10] = CAVE_MachE_sil_test_B.Reshape2
    [3];
  CAVE_MachE_sil_test_B.VectorConcatenate3[11] = CAVE_MachE_sil_test_B.Reshape[3];

  /* Outputs for Iterator SubSystem: '<S313>/Wheel to Body Transform' incorporates:
   *  ForEach: '<S319>/For Each'
   */
  for (ForEach_itr_j = 0; ForEach_itr_j < 4; ForEach_itr_j++) {
    /* ForEachSliceSelector generated from: '<S319>/WheelAngles' */
    Bias = CAVE_MachE_sil_test_B.VectorConcatenate3[3 * ForEach_itr_j];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.VectorConcatenate3[3 * ForEach_itr_j + 1];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 =
      CAVE_MachE_sil_test_B.VectorConcatenate3[3 * ForEach_itr_j + 2];

    /* SignalConversion generated from: '<S320>/sincos' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[0] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[1] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[2] = Bias;

    /* Trigonometry: '<S320>/sincos' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[0];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[1];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
      TmpSignalConversionAtsincosInport1[2];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

    /* Fcn: '<S320>/Fcn11' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0];

    /* Fcn: '<S320>/Fcn21' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0];

    /* Fcn: '<S320>/Fcn31' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0];

    /* Fcn: '<S320>/Fcn12' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[3] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0];

    /* Fcn: '<S320>/Fcn22' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[4] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0];

    /* Fcn: '<S320>/Fcn32' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[5] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0];

    /* Fcn: '<S320>/Fcn13' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[6] =
      -CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1];

    /* Fcn: '<S320>/Fcn23' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[7] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1];

    /* Fcn: '<S320>/Fcn33' */
    CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[8] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1];
    for (y = 0; y < 9; y++) {
      /* Reshape: '<S321>/Reshape (9) to [3x3] column-major' */
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
        Reshape9to3x3columnmajor[y] =
        CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].VectorConcatenate[y];

      /* Product: '<S319>/Divide1' */
      u[y] = CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
        Reshape9to3x3columnmajor[y];
    }

    /* Product: '<S319>/Divide1' incorporates:
     *  ForEachSliceSelector generated from: '<S319>/Forces'
     */
    b_a[0] = CAVE_MachE_sil_test_B.MathFunction_d[3 * ForEach_itr_j];
    b_a[1] = CAVE_MachE_sil_test_B.MathFunction_d[3 * ForEach_itr_j + 1];
    b_a[2] = CAVE_MachE_sil_test_B.MathFunction_d[3 * ForEach_itr_j + 2];
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[ibmat] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[ibmat] +=
        u[ibmat] * b_a[0];
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[ibmat] +=
        u[ibmat + 3] * b_a[1];
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[ibmat] +=
        u[ibmat + 6] * b_a[2];
    }

    /* ForEachSliceAssignment generated from: '<S319>/Fz' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fz_at_inport_0[ForEach_itr_j] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[2];

    /* ForEachSliceAssignment generated from: '<S319>/Fy' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[ForEach_itr_j] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[1];

    /* ForEachSliceAssignment generated from: '<S319>/Fx' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[ForEach_itr_j] =
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[0];
  }

  /* End of Outputs for SubSystem: '<S313>/Wheel to Body Transform' */

  /* Reshape: '<S139>/Reshape6' */
  CAVE_MachE_sil_test_B.Reshape6[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[0];
  CAVE_MachE_sil_test_B.Reshape6[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[1];
  CAVE_MachE_sil_test_B.Reshape6[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[2];
  CAVE_MachE_sil_test_B.Reshape6[3] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[3];

  /* Reshape: '<S140>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_i[0] = CAVE_MachE_sil_test_B.Reshape6[0];
  CAVE_MachE_sil_test_B.Reshape3_i[1] = CAVE_MachE_sil_test_B.Reshape6[1];

  /* Reshape: '<S139>/Reshape7' */
  CAVE_MachE_sil_test_B.Reshape7[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[0];
  CAVE_MachE_sil_test_B.Reshape7[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[1];
  CAVE_MachE_sil_test_B.Reshape7[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[2];
  CAVE_MachE_sil_test_B.Reshape7[3] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[3];

  /* Reshape: '<S140>/Reshape4' */
  CAVE_MachE_sil_test_B.Reshape4_e[0] = CAVE_MachE_sil_test_B.Reshape7[0];

  /* Concatenate: '<S140>/Matrix Concatenate4' */
  CAVE_MachE_sil_test_B.MatrixConcatenate4[0] =
    CAVE_MachE_sil_test_B.Reshape3_i[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate4[1] =
    CAVE_MachE_sil_test_B.Reshape4_e[0];

  /* Reshape: '<S140>/Reshape4' */
  CAVE_MachE_sil_test_B.Reshape4_e[1] = CAVE_MachE_sil_test_B.Reshape7[1];

  /* Concatenate: '<S140>/Matrix Concatenate4' */
  CAVE_MachE_sil_test_B.MatrixConcatenate4[2] =
    CAVE_MachE_sil_test_B.Reshape3_i[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate4[3] =
    CAVE_MachE_sil_test_B.Reshape4_e[1];

  /* SignalConversion generated from: '<S341>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_er[0] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1;

  /* SignalConversion generated from: '<S341>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_er[1] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_a;

  /* SignalConversion generated from: '<S341>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_er[2] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_o;

  /* SignalConversion generated from: '<S341>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_er[3] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o1_h;

  /* UnaryMinus: '<S317>/Unary Minus1' */
  CAVE_MachE_sil_test_B.z[0] = -CAVE_MachE_sil_test_B.VectorConcatenate_er[0];

  /* Reshape: '<S139>/Reshape' */
  CAVE_MachE_sil_test_B.z_k[0] = CAVE_MachE_sil_test_B.z[0];

  /* UnaryMinus: '<S317>/Unary Minus1' */
  CAVE_MachE_sil_test_B.z[1] = -CAVE_MachE_sil_test_B.VectorConcatenate_er[1];

  /* Reshape: '<S139>/Reshape' */
  CAVE_MachE_sil_test_B.z_k[1] = CAVE_MachE_sil_test_B.z[1];

  /* UnaryMinus: '<S317>/Unary Minus1' */
  CAVE_MachE_sil_test_B.z[2] = -CAVE_MachE_sil_test_B.VectorConcatenate_er[2];

  /* Reshape: '<S139>/Reshape' */
  CAVE_MachE_sil_test_B.z_k[2] = CAVE_MachE_sil_test_B.z[2];

  /* UnaryMinus: '<S317>/Unary Minus1' */
  CAVE_MachE_sil_test_B.z[3] = -CAVE_MachE_sil_test_B.VectorConcatenate_er[3];

  /* Reshape: '<S139>/Reshape' */
  CAVE_MachE_sil_test_B.z_k[3] = CAVE_MachE_sil_test_B.z[3];

  /* Reshape: '<S140>/Reshape9' */
  CAVE_MachE_sil_test_B.Reshape9[0] = CAVE_MachE_sil_test_B.z_k[0];
  CAVE_MachE_sil_test_B.Reshape9[1] = CAVE_MachE_sil_test_B.z_k[1];

  /* SignalConversion generated from: '<S342>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_m[0] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2;

  /* SignalConversion generated from: '<S342>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_m[1] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_n;

  /* SignalConversion generated from: '<S342>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_m[2] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_p;

  /* SignalConversion generated from: '<S342>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_m[3] =
    CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_h;

  /* UnaryMinus: '<S317>/Unary Minus2' */
  CAVE_MachE_sil_test_B.zdot[0] = -CAVE_MachE_sil_test_B.VectorConcatenate_m[0];

  /* Reshape: '<S139>/Reshape2' */
  CAVE_MachE_sil_test_B.zdot_o[0] = CAVE_MachE_sil_test_B.zdot[0];

  /* UnaryMinus: '<S317>/Unary Minus2' */
  CAVE_MachE_sil_test_B.zdot[1] = -CAVE_MachE_sil_test_B.VectorConcatenate_m[1];

  /* Reshape: '<S139>/Reshape2' */
  CAVE_MachE_sil_test_B.zdot_o[1] = CAVE_MachE_sil_test_B.zdot[1];

  /* UnaryMinus: '<S317>/Unary Minus2' */
  CAVE_MachE_sil_test_B.zdot[2] = -CAVE_MachE_sil_test_B.VectorConcatenate_m[2];

  /* Reshape: '<S139>/Reshape2' */
  CAVE_MachE_sil_test_B.zdot_o[2] = CAVE_MachE_sil_test_B.zdot[2];

  /* UnaryMinus: '<S317>/Unary Minus2' */
  CAVE_MachE_sil_test_B.zdot[3] = -CAVE_MachE_sil_test_B.VectorConcatenate_m[3];

  /* Reshape: '<S139>/Reshape2' */
  CAVE_MachE_sil_test_B.zdot_o[3] = CAVE_MachE_sil_test_B.zdot[3];

  /* Reshape: '<S140>/Reshape8' */
  CAVE_MachE_sil_test_B.Reshape8[0] = CAVE_MachE_sil_test_B.zdot_o[0];

  /* Concatenate: '<S140>/Matrix Concatenate2' */
  CAVE_MachE_sil_test_B.MatrixConcatenate2[0] = CAVE_MachE_sil_test_B.Reshape9[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate2[1] = CAVE_MachE_sil_test_B.Reshape8[0];

  /* Reshape: '<S140>/Reshape8' */
  CAVE_MachE_sil_test_B.Reshape8[1] = CAVE_MachE_sil_test_B.zdot_o[1];

  /* Concatenate: '<S140>/Matrix Concatenate2' */
  CAVE_MachE_sil_test_B.MatrixConcatenate2[2] = CAVE_MachE_sil_test_B.Reshape9[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate2[3] = CAVE_MachE_sil_test_B.Reshape8[1];

  /* SignalConversion generated from: '<S238>/sincos' */
  CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[0] =
    CAVE_MachE_sil_test_B.phithetapsi[2];
  CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[1] =
    CAVE_MachE_sil_test_B.phithetapsi[1];
  CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[2] =
    CAVE_MachE_sil_test_B.phithetapsi[0];

  /* Trigonometry: '<S257>/sincos' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[0];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1[0] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2[0] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[1];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1[1] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2[1] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[2];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1[2] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2[2] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

  /* Fcn: '<S257>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[0] =
    CAVE_MachE_sil_test_B.sincos_o2[1] * CAVE_MachE_sil_test_B.sincos_o2[0];

  /* Fcn: '<S257>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[1] =
    CAVE_MachE_sil_test_B.sincos_o1[2] * CAVE_MachE_sil_test_B.sincos_o1[1] *
    CAVE_MachE_sil_test_B.sincos_o2[0] - CAVE_MachE_sil_test_B.sincos_o2[2] *
    CAVE_MachE_sil_test_B.sincos_o1[0];

  /* Fcn: '<S257>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[2] =
    CAVE_MachE_sil_test_B.sincos_o2[2] * CAVE_MachE_sil_test_B.sincos_o1[1] *
    CAVE_MachE_sil_test_B.sincos_o2[0] + CAVE_MachE_sil_test_B.sincos_o1[2] *
    CAVE_MachE_sil_test_B.sincos_o1[0];

  /* Fcn: '<S257>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[3] =
    CAVE_MachE_sil_test_B.sincos_o2[1] * CAVE_MachE_sil_test_B.sincos_o1[0];

  /* Fcn: '<S257>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[4] =
    CAVE_MachE_sil_test_B.sincos_o1[2] * CAVE_MachE_sil_test_B.sincos_o1[1] *
    CAVE_MachE_sil_test_B.sincos_o1[0] + CAVE_MachE_sil_test_B.sincos_o2[2] *
    CAVE_MachE_sil_test_B.sincos_o2[0];

  /* Fcn: '<S257>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[5] =
    CAVE_MachE_sil_test_B.sincos_o2[2] * CAVE_MachE_sil_test_B.sincos_o1[1] *
    CAVE_MachE_sil_test_B.sincos_o1[0] - CAVE_MachE_sil_test_B.sincos_o1[2] *
    CAVE_MachE_sil_test_B.sincos_o2[0];

  /* Fcn: '<S257>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[6] =
    -CAVE_MachE_sil_test_B.sincos_o1[1];

  /* Fcn: '<S257>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[7] =
    CAVE_MachE_sil_test_B.sincos_o1[2] * CAVE_MachE_sil_test_B.sincos_o2[1];

  /* Fcn: '<S257>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_l5[8] =
    CAVE_MachE_sil_test_B.sincos_o2[2] * CAVE_MachE_sil_test_B.sincos_o2[1];

  /* Reshape: '<S261>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_l5[0], 9U * sizeof(real_T));

  /* Math: '<S256>/Transpose1' */
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Transpose1[3 * ibmat] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor[ibmat];
    CAVE_MachE_sil_test_B.Transpose1[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor[ibmat + 3];
    CAVE_MachE_sil_test_B.Transpose1[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor[ibmat + 6];
  }

  /* End of Math: '<S256>/Transpose1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* MATLAB Function: '<S217>/vehdyncginert' */
    /* MATLAB Function 'Vehicle Body 6DOF/vehdyncginert': '<S229>:1' */
    /* '<S229>:1:3' */
    b_a[0] = CAVE_MachE_sil_test_P.ChassisDistCg2FrontAxle;
    b_a[1] = -CAVE_MachE_sil_test_P.VehicleBody6DOF1_d;
    b_a[2] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_h;
    for (y = 0; y < 3; y++) {
      ibmat = y * 7;
      for (ibtile = 0; ibtile < 7; ibtile++) {
        b_b[ibmat + ibtile] = b_a[y];
      }
    }

    for (y = 0; y < 3; y++) {
      ibmat = y * 7;
      for (ibtile = 0; ibtile < 7; ibtile++) {
        c_b[ibmat + ibtile] = e[y];
      }

      R[y << 3] = 0.0;
    }

    R[1] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z1R[0] * (real_T)c_b[0] + b_b
      [0];
    R[2] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z2R[0] * (real_T)c_b[1] + b_b
      [1];
    R[3] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z3R[0] * (real_T)c_b[2] + b_b
      [2];
    R[4] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z4R[0] * (real_T)c_b[3] + b_b
      [3];
    R[5] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z5R[0] * (real_T)c_b[4] + b_b
      [4];
    R[6] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z6R[0] * (real_T)c_b[5] + b_b
      [5];
    R[7] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z7R[0] * (real_T)c_b[6] + b_b
      [6];
    R[9] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z1R[1] * (real_T)c_b[7] + b_b
      [7];
    R[10] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z2R[1] * (real_T)c_b[8] +
      b_b[8];
    R[11] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z3R[1] * (real_T)c_b[9] +
      b_b[9];
    R[12] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z4R[1] * (real_T)c_b[10] +
      b_b[10];
    R[13] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z5R[1] * (real_T)c_b[11] +
      b_b[11];
    R[14] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z6R[1] * (real_T)c_b[12] +
      b_b[12];
    R[15] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z7R[1] * (real_T)c_b[13] +
      b_b[13];
    R[17] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z1R[2] * (real_T)c_b[14] +
      b_b[14];
    R[18] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z2R[2] * (real_T)c_b[15] +
      b_b[15];
    R[19] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z3R[2] * (real_T)c_b[16] +
      b_b[16];
    R[20] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z4R[2] * (real_T)c_b[17] +
      b_b[17];
    R[21] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z5R[2] * (real_T)c_b[18] +
      b_b[18];
    R[22] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z6R[2] * (real_T)c_b[19] +
      b_b[19];
    R[23] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z7R[2] * (real_T)c_b[20] +
      b_b[20];
    M[0] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_m;
    M[1] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z1m;
    M[2] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z2m;
    M[3] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z3m;
    M[4] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z4m;
    M[5] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z5m;
    M[6] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z6m;
    M[7] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z7m;
    y = -1;
    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_Iveh[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z1I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z2I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z3I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z4I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z5I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z6I[ibmat];
    }

    for (ibmat = 0; ibmat < 9; ibmat++) {
      y++;
      Imat[y] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_z7I[ibmat];
    }

    for (ibmat = 0; ibmat < 3; ibmat++) {
      ibtile = ibmat << 3;
      memcpy(&d_b[ibtile], &M[0], sizeof(real_T) << 3U);
    }

    Bias = M[0];
    for (y = 0; y < 7; y++) {
      Bias += M[y + 1];
    }

    for (ibmat = 0; ibmat < 24; ibmat++) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = d_b[ibmat];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
        rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 * R[ibmat] / Bias;
      d_b[ibmat] = rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    }

    for (y = 0; y < 3; y++) {
      ibmat = y << 3;
      CAVE_MachE_sil_test_B.Rbar[y] = d_b[ibmat];
      for (ibtile = 0; ibtile < 7; ibtile++) {
        CAVE_MachE_sil_test_B.Rbar[y] += d_b[(ibmat + ibtile) + 1];
      }
    }

    Bias = M[0];
    for (y = 0; y < 7; y++) {
      Bias += M[y + 1];
    }

    for (y = 0; y < 3; y++) {
      ibmat = y << 3;
      for (ibtile = 0; ibtile < 8; ibtile++) {
        d_b[ibmat + ibtile] = CAVE_MachE_sil_test_B.Rbar[y];
      }
    }

    for (ibmat = 0; ibmat < 24; ibmat++) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = R[ibmat];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 -= d_b[ibmat];
      R[ibmat] = rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    }

    for (y = 0; y < 8; y++) {
      b_a[0] = R[y];
      b_a[1] = R[y + 8];
      b_a[2] = R[y + 16];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = b_a[0] * R[y];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 += R[y + 8] * b_a[1];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 += R[y + 16] * b_a[2];
      M_0 = M[y];
      for (ibmat = 0; ibmat < 3; ibmat++) {
        rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = R[(ibmat << 3) +
          y] * R[y];
        u[ibmat] = rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 *
          (real_T)f_b[ibmat] -
          rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
        rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = R[(ibmat << 3) +
          y] * R[y + 8];
        u[ibmat + 3] = (real_T)f_b[ibmat + 3] *
          rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 -
          rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
        rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = R[(ibmat << 3) +
          y] * R[y + 16];
        u[ibmat + 6] = (real_T)f_b[ibmat + 6] *
          rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 -
          rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
      }

      for (ibmat = 0; ibmat < 3; ibmat++) {
        Itemp[3 * ibmat + 9 * y] = u[3 * ibmat] * M_0 * (real_T)f[3 * ibmat] +
          Imat[3 * ibmat + 9 * y];
        Itemp[(3 * ibmat + 9 * y) + 1] = u[3 * ibmat + 1] * M_0 * (real_T)f[3 *
          ibmat + 1] + Imat[(3 * ibmat + 9 * y) + 1];
        Itemp[(3 * ibmat + 9 * y) + 2] = u[3 * ibmat + 2] * M_0 * (real_T)f[3 *
          ibmat + 2] + Imat[(3 * ibmat + 9 * y) + 2];
      }
    }

    memcpy(&CAVE_MachE_sil_test_B.Ibar[0], &Itemp[0], 9U * sizeof(real_T));
    for (y = 0; y < 7; y++) {
      ibmat = (y + 1) * 9;
      for (ibtile = 0; ibtile < 9; ibtile++) {
        CAVE_MachE_sil_test_B.Ibar[ibtile] += Itemp[ibmat + ibtile];
      }
    }

    CAVE_MachE_sil_test_B.Xbar[0] =
      CAVE_MachE_sil_test_P.ChassisDistCg2FrontAxle -
      CAVE_MachE_sil_test_B.Rbar[0];
    CAVE_MachE_sil_test_B.Xbar[1] = CAVE_MachE_sil_test_P.ChassisDistCg2RearAxle
      + CAVE_MachE_sil_test_B.Rbar[0];
    CAVE_MachE_sil_test_B.Xbar[2] = CAVE_MachE_sil_test_P.VehicleBody6DOF1_h -
      CAVE_MachE_sil_test_B.Rbar[2];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_w[0] / 2.0;
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_w[1] / 2.0;
    CAVE_MachE_sil_test_B.Wbar[0] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 +
      CAVE_MachE_sil_test_B.Rbar[1];
    CAVE_MachE_sil_test_B.Wbar[1] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 -
      CAVE_MachE_sil_test_B.Rbar[1];
    CAVE_MachE_sil_test_B.Wbar[2] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 +
      CAVE_MachE_sil_test_B.Rbar[1];
    CAVE_MachE_sil_test_B.Wbar[3] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 -
      CAVE_MachE_sil_test_B.Rbar[1];

    /* '<S229>:1:3' */
    CAVE_MachE_sil_test_B.Mbar = Bias;

    /* '<S229>:1:3' */
    CAVE_MachE_sil_test_B.HPbar[0] = CAVE_MachE_sil_test_B.Xbar[0];
    CAVE_MachE_sil_test_B.HPbar[3] = CAVE_MachE_sil_test_B.Xbar[0];
    CAVE_MachE_sil_test_B.HPbar[6] = -CAVE_MachE_sil_test_B.Xbar[1];
    CAVE_MachE_sil_test_B.HPbar[9] = -CAVE_MachE_sil_test_B.Xbar[1];
    CAVE_MachE_sil_test_B.HPbar[1] = -CAVE_MachE_sil_test_B.Wbar[0];
    CAVE_MachE_sil_test_B.HPbar[4] = CAVE_MachE_sil_test_B.Wbar[1];
    CAVE_MachE_sil_test_B.HPbar[7] = -CAVE_MachE_sil_test_B.Wbar[2];
    CAVE_MachE_sil_test_B.HPbar[10] = CAVE_MachE_sil_test_B.Wbar[3];
    CAVE_MachE_sil_test_B.HPbar[2] = CAVE_MachE_sil_test_B.Xbar[2];
    CAVE_MachE_sil_test_B.HPbar[5] = CAVE_MachE_sil_test_B.Xbar[2];
    CAVE_MachE_sil_test_B.HPbar[8] = CAVE_MachE_sil_test_B.Xbar[2];
    CAVE_MachE_sil_test_B.HPbar[11] = CAVE_MachE_sil_test_B.Xbar[2];

    /* End of MATLAB Function: '<S217>/vehdyncginert' */

    /* Selector: '<S251>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1[0] = CAVE_MachE_sil_test_B.HPbar[0];

    /* Reshape: '<S259>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_n[0] = CAVE_MachE_sil_test_B.Selector1[0];

    /* Selector: '<S251>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1[1] = CAVE_MachE_sil_test_B.HPbar[1];

    /* Reshape: '<S259>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_n[1] = CAVE_MachE_sil_test_B.Selector1[1];

    /* Selector: '<S251>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1[2] = CAVE_MachE_sil_test_B.HPbar[2];

    /* Reshape: '<S259>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_n[2] = CAVE_MachE_sil_test_B.Selector1[2];
  }

  /* Product: '<S259>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_n[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_n[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_n[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_l[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_l[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_l[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_l[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S259>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_l[y] = CAVE_MachE_sil_test_B.Product_l[y];

    /* Sum: '<S256>/Add' */
    CAVE_MachE_sil_test_B.Add_b3[y] = CAVE_MachE_sil_test_B.xeyeze[y] +
      CAVE_MachE_sil_test_B.Reshape2_l[y];

    /* Trigonometry: '<S265>/sincos' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[y];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.sincos_o1_h[y] = Bias;
    CAVE_MachE_sil_test_B.sincos_o2_g[y] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  }

  /* End of Product: '<S259>/Product' */

  /* Fcn: '<S265>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[0] =
    CAVE_MachE_sil_test_B.sincos_o2_g[1] * CAVE_MachE_sil_test_B.sincos_o2_g[0];

  /* Fcn: '<S265>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[1] =
    CAVE_MachE_sil_test_B.sincos_o1_h[2] * CAVE_MachE_sil_test_B.sincos_o1_h[1] *
    CAVE_MachE_sil_test_B.sincos_o2_g[0] - CAVE_MachE_sil_test_B.sincos_o2_g[2] *
    CAVE_MachE_sil_test_B.sincos_o1_h[0];

  /* Fcn: '<S265>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[2] =
    CAVE_MachE_sil_test_B.sincos_o2_g[2] * CAVE_MachE_sil_test_B.sincos_o1_h[1] *
    CAVE_MachE_sil_test_B.sincos_o2_g[0] + CAVE_MachE_sil_test_B.sincos_o1_h[2] *
    CAVE_MachE_sil_test_B.sincos_o1_h[0];

  /* Fcn: '<S265>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[3] =
    CAVE_MachE_sil_test_B.sincos_o2_g[1] * CAVE_MachE_sil_test_B.sincos_o1_h[0];

  /* Fcn: '<S265>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[4] =
    CAVE_MachE_sil_test_B.sincos_o1_h[2] * CAVE_MachE_sil_test_B.sincos_o1_h[1] *
    CAVE_MachE_sil_test_B.sincos_o1_h[0] + CAVE_MachE_sil_test_B.sincos_o2_g[2] *
    CAVE_MachE_sil_test_B.sincos_o2_g[0];

  /* Fcn: '<S265>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[5] =
    CAVE_MachE_sil_test_B.sincos_o2_g[2] * CAVE_MachE_sil_test_B.sincos_o1_h[1] *
    CAVE_MachE_sil_test_B.sincos_o1_h[0] - CAVE_MachE_sil_test_B.sincos_o1_h[2] *
    CAVE_MachE_sil_test_B.sincos_o2_g[0];

  /* Fcn: '<S265>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[6] =
    -CAVE_MachE_sil_test_B.sincos_o1_h[1];

  /* Fcn: '<S265>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[7] =
    CAVE_MachE_sil_test_B.sincos_o1_h[2] * CAVE_MachE_sil_test_B.sincos_o2_g[1];

  /* Fcn: '<S265>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ps[8] =
    CAVE_MachE_sil_test_B.sincos_o2_g[2] * CAVE_MachE_sil_test_B.sincos_o2_g[1];

  /* Reshape: '<S269>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_m[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_ps[0], 9U * sizeof(real_T));

  /* Math: '<S264>/Transpose1' */
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Transpose1_p[3 * ibmat] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_m[ibmat];
    CAVE_MachE_sil_test_B.Transpose1_p[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_m[ibmat + 3];
    CAVE_MachE_sil_test_B.Transpose1_p[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_m[ibmat + 6];
  }

  /* End of Math: '<S264>/Transpose1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S252>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_h[0] = CAVE_MachE_sil_test_B.HPbar[3];

    /* Reshape: '<S267>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_k[0] = CAVE_MachE_sil_test_B.Selector1_h[0];

    /* Selector: '<S252>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_h[1] = CAVE_MachE_sil_test_B.HPbar[4];

    /* Reshape: '<S267>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_k[1] = CAVE_MachE_sil_test_B.Selector1_h[1];

    /* Selector: '<S252>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_h[2] = CAVE_MachE_sil_test_B.HPbar[5];

    /* Reshape: '<S267>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_k[2] = CAVE_MachE_sil_test_B.Selector1_h[2];
  }

  /* Product: '<S267>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_p[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_k[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_k[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_k[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_a[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_a[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_a[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_a[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S267>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_i[y] = CAVE_MachE_sil_test_B.Product_a[y];

    /* Sum: '<S264>/Add' */
    CAVE_MachE_sil_test_B.Add_kn[y] = CAVE_MachE_sil_test_B.xeyeze[y] +
      CAVE_MachE_sil_test_B.Reshape2_i[y];

    /* Trigonometry: '<S285>/sincos' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[y];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.sincos_o1_l[y] = Bias;
    CAVE_MachE_sil_test_B.sincos_o2_m[y] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  }

  /* End of Product: '<S267>/Product' */

  /* Fcn: '<S285>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[0] =
    CAVE_MachE_sil_test_B.sincos_o2_m[1] * CAVE_MachE_sil_test_B.sincos_o2_m[0];

  /* Fcn: '<S285>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[1] =
    CAVE_MachE_sil_test_B.sincos_o1_l[2] * CAVE_MachE_sil_test_B.sincos_o1_l[1] *
    CAVE_MachE_sil_test_B.sincos_o2_m[0] - CAVE_MachE_sil_test_B.sincos_o2_m[2] *
    CAVE_MachE_sil_test_B.sincos_o1_l[0];

  /* Fcn: '<S285>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[2] =
    CAVE_MachE_sil_test_B.sincos_o2_m[2] * CAVE_MachE_sil_test_B.sincos_o1_l[1] *
    CAVE_MachE_sil_test_B.sincos_o2_m[0] + CAVE_MachE_sil_test_B.sincos_o1_l[2] *
    CAVE_MachE_sil_test_B.sincos_o1_l[0];

  /* Fcn: '<S285>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[3] =
    CAVE_MachE_sil_test_B.sincos_o2_m[1] * CAVE_MachE_sil_test_B.sincos_o1_l[0];

  /* Fcn: '<S285>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[4] =
    CAVE_MachE_sil_test_B.sincos_o1_l[2] * CAVE_MachE_sil_test_B.sincos_o1_l[1] *
    CAVE_MachE_sil_test_B.sincos_o1_l[0] + CAVE_MachE_sil_test_B.sincos_o2_m[2] *
    CAVE_MachE_sil_test_B.sincos_o2_m[0];

  /* Fcn: '<S285>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[5] =
    CAVE_MachE_sil_test_B.sincos_o2_m[2] * CAVE_MachE_sil_test_B.sincos_o1_l[1] *
    CAVE_MachE_sil_test_B.sincos_o1_l[0] - CAVE_MachE_sil_test_B.sincos_o1_l[2] *
    CAVE_MachE_sil_test_B.sincos_o2_m[0];

  /* Fcn: '<S285>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[6] =
    -CAVE_MachE_sil_test_B.sincos_o1_l[1];

  /* Fcn: '<S285>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[7] =
    CAVE_MachE_sil_test_B.sincos_o1_l[2] * CAVE_MachE_sil_test_B.sincos_o2_m[1];

  /* Fcn: '<S285>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_o[8] =
    CAVE_MachE_sil_test_B.sincos_o2_m[2] * CAVE_MachE_sil_test_B.sincos_o2_m[1];

  /* Reshape: '<S289>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_mx[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_o[0], 9U * sizeof(real_T));

  /* Math: '<S284>/Transpose1' */
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Transpose1_f[3 * ibmat] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_mx[ibmat];
    CAVE_MachE_sil_test_B.Transpose1_f[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_mx[ibmat + 3];
    CAVE_MachE_sil_test_B.Transpose1_f[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_mx[ibmat + 6];
  }

  /* End of Math: '<S284>/Transpose1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S254>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_c[0] = CAVE_MachE_sil_test_B.HPbar[6];

    /* Reshape: '<S287>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_b[0] = CAVE_MachE_sil_test_B.Selector1_c[0];

    /* Selector: '<S254>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_c[1] = CAVE_MachE_sil_test_B.HPbar[7];

    /* Reshape: '<S287>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_b[1] = CAVE_MachE_sil_test_B.Selector1_c[1];

    /* Selector: '<S254>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_c[2] = CAVE_MachE_sil_test_B.HPbar[8];

    /* Reshape: '<S287>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_b[2] = CAVE_MachE_sil_test_B.Selector1_c[2];
  }

  /* Product: '<S287>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_f[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_b[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_b[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_b[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_aw[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_aw[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_aw[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_aw[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S287>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_n[y] = CAVE_MachE_sil_test_B.Product_aw[y];

    /* Sum: '<S284>/Add' */
    CAVE_MachE_sil_test_B.Add_cn[y] = CAVE_MachE_sil_test_B.xeyeze[y] +
      CAVE_MachE_sil_test_B.Reshape2_n[y];

    /* Trigonometry: '<S293>/sincos' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[y];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.sincos_o1_c[y] = Bias;
    CAVE_MachE_sil_test_B.sincos_o2_gs[y] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  }

  /* End of Product: '<S287>/Product' */

  /* Fcn: '<S293>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[0] =
    CAVE_MachE_sil_test_B.sincos_o2_gs[1] * CAVE_MachE_sil_test_B.sincos_o2_gs[0];

  /* Fcn: '<S293>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[1] =
    CAVE_MachE_sil_test_B.sincos_o1_c[2] * CAVE_MachE_sil_test_B.sincos_o1_c[1] *
    CAVE_MachE_sil_test_B.sincos_o2_gs[0] - CAVE_MachE_sil_test_B.sincos_o2_gs[2]
    * CAVE_MachE_sil_test_B.sincos_o1_c[0];

  /* Fcn: '<S293>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[2] =
    CAVE_MachE_sil_test_B.sincos_o2_gs[2] * CAVE_MachE_sil_test_B.sincos_o1_c[1]
    * CAVE_MachE_sil_test_B.sincos_o2_gs[0] + CAVE_MachE_sil_test_B.sincos_o1_c
    [2] * CAVE_MachE_sil_test_B.sincos_o1_c[0];

  /* Fcn: '<S293>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[3] =
    CAVE_MachE_sil_test_B.sincos_o2_gs[1] * CAVE_MachE_sil_test_B.sincos_o1_c[0];

  /* Fcn: '<S293>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[4] =
    CAVE_MachE_sil_test_B.sincos_o1_c[2] * CAVE_MachE_sil_test_B.sincos_o1_c[1] *
    CAVE_MachE_sil_test_B.sincos_o1_c[0] + CAVE_MachE_sil_test_B.sincos_o2_gs[2]
    * CAVE_MachE_sil_test_B.sincos_o2_gs[0];

  /* Fcn: '<S293>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[5] =
    CAVE_MachE_sil_test_B.sincos_o2_gs[2] * CAVE_MachE_sil_test_B.sincos_o1_c[1]
    * CAVE_MachE_sil_test_B.sincos_o1_c[0] - CAVE_MachE_sil_test_B.sincos_o1_c[2]
    * CAVE_MachE_sil_test_B.sincos_o2_gs[0];

  /* Fcn: '<S293>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[6] =
    -CAVE_MachE_sil_test_B.sincos_o1_c[1];

  /* Fcn: '<S293>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[7] =
    CAVE_MachE_sil_test_B.sincos_o1_c[2] * CAVE_MachE_sil_test_B.sincos_o2_gs[1];

  /* Fcn: '<S293>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_n5[8] =
    CAVE_MachE_sil_test_B.sincos_o2_gs[2] * CAVE_MachE_sil_test_B.sincos_o2_gs[1];

  /* Reshape: '<S297>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_k[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_n5[0], 9U * sizeof(real_T));

  /* Math: '<S292>/Transpose1' */
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Transpose1_fn[3 * ibmat] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_k[ibmat];
    CAVE_MachE_sil_test_B.Transpose1_fn[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_k[ibmat + 3];
    CAVE_MachE_sil_test_B.Transpose1_fn[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_k[ibmat + 6];
  }

  /* End of Math: '<S292>/Transpose1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S255>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_l[0] = CAVE_MachE_sil_test_B.HPbar[9];

    /* Reshape: '<S295>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_kj[0] = CAVE_MachE_sil_test_B.Selector1_l[0];

    /* Selector: '<S255>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_l[1] = CAVE_MachE_sil_test_B.HPbar[10];

    /* Reshape: '<S295>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_kj[1] = CAVE_MachE_sil_test_B.Selector1_l[1];

    /* Selector: '<S255>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_l[2] = CAVE_MachE_sil_test_B.HPbar[11];

    /* Reshape: '<S295>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_kj[2] = CAVE_MachE_sil_test_B.Selector1_l[2];
  }

  /* Product: '<S295>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_fn[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_kj[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_kj[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_kj[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_e[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_e[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_e[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_e[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S295>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_e[y] = CAVE_MachE_sil_test_B.Product_e[y];

    /* Sum: '<S292>/Add' */
    CAVE_MachE_sil_test_B.Add_f[y] = CAVE_MachE_sil_test_B.xeyeze[y] +
      CAVE_MachE_sil_test_B.Reshape2_e[y];

    /* Reshape: '<S138>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_i[y] = CAVE_MachE_sil_test_B.Add_b3[y];
    CAVE_MachE_sil_test_B.Reshape_i[y + 3] = CAVE_MachE_sil_test_B.Add_kn[y];
    CAVE_MachE_sil_test_B.Reshape_i[y + 6] = CAVE_MachE_sil_test_B.Add_cn[y];
    CAVE_MachE_sil_test_B.Reshape_i[y + 9] = CAVE_MachE_sil_test_B.Add_f[y];
  }

  /* End of Product: '<S295>/Product' */

  /* Sum: '<S138>/Sum' incorporates:
   *  Constant: '<S138>/Inertial Frame CG to Axle Offset'
   */
  for (y = 0; y < 12; y++) {
    CAVE_MachE_sil_test_B.P[y] = CAVE_MachE_sil_test_B.Reshape_i[y] -
      CAVE_MachE_sil_test_P.InertialFrameCGtoAxleOffset_Value[y];
  }

  /* End of Sum: '<S138>/Sum' */
  for (y = 0; y < 2; y++) {
    /* Selector: '<S134>/Selector10' */
    CAVE_MachE_sil_test_B.Selector10[3 * y] = CAVE_MachE_sil_test_B.P[3 * y];
    CAVE_MachE_sil_test_B.Selector10[3 * y + 1] = CAVE_MachE_sil_test_B.P[3 * y
      + 1];
    CAVE_MachE_sil_test_B.Selector10[3 * y + 2] = CAVE_MachE_sil_test_B.P[3 * y
      + 2];

    /* Selector: '<S140>/Selector' */
    CAVE_MachE_sil_test_B.Selector[y] = CAVE_MachE_sil_test_B.Selector10[3 * y +
      2];

    /* Reshape: '<S140>/Reshape7' */
    CAVE_MachE_sil_test_B.Reshape7_p[y] = CAVE_MachE_sil_test_B.Selector[y];
  }

  /* Integrator: '<S218>/p,q,r ' */
  CAVE_MachE_sil_test_B.pqr[0] = CAVE_MachE_sil_test_X.pqr_CSTATE[0];
  CAVE_MachE_sil_test_B.pqr[1] = CAVE_MachE_sil_test_X.pqr_CSTATE[1];
  CAVE_MachE_sil_test_B.pqr[2] = CAVE_MachE_sil_test_X.pqr_CSTATE[2];

  /* Product: '<S262>/j x k' */
  CAVE_MachE_sil_test_B.jxk = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1[2];

  /* Product: '<S262>/k x i' */
  CAVE_MachE_sil_test_B.kxi = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1[0];

  /* Product: '<S262>/i x j' */
  CAVE_MachE_sil_test_B.ixj = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1[1];

  /* Product: '<S263>/k x j' */
  CAVE_MachE_sil_test_B.kxj = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1[1];

  /* Product: '<S263>/i x k' */
  CAVE_MachE_sil_test_B.ixk = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1[2];

  /* Product: '<S263>/j x i' */
  CAVE_MachE_sil_test_B.jxi = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1[0];

  /* Sum: '<S260>/Sum' */
  CAVE_MachE_sil_test_B.Sum_j[0] = CAVE_MachE_sil_test_B.jxk -
    CAVE_MachE_sil_test_B.kxj;
  CAVE_MachE_sil_test_B.Sum_j[1] = CAVE_MachE_sil_test_B.kxi -
    CAVE_MachE_sil_test_B.ixk;
  CAVE_MachE_sil_test_B.Sum_j[2] = CAVE_MachE_sil_test_B.ixj -
    CAVE_MachE_sil_test_B.jxi;

  /* Sum: '<S256>/Add1' */
  CAVE_MachE_sil_test_B.Add1_a[0] = CAVE_MachE_sil_test_B.Sum_j[0] +
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.Add1_a[1] = CAVE_MachE_sil_test_B.Sum_j[1] +
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.Add1_a[2] = CAVE_MachE_sil_test_B.Sum_j[2] +
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Product: '<S270>/j x k' */
  CAVE_MachE_sil_test_B.jxk_f = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_h[2];

  /* Product: '<S270>/k x i' */
  CAVE_MachE_sil_test_B.kxi_g = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_h[0];

  /* Product: '<S270>/i x j' */
  CAVE_MachE_sil_test_B.ixj_o = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_h[1];

  /* Product: '<S271>/k x j' */
  CAVE_MachE_sil_test_B.kxj_b = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_h[1];

  /* Product: '<S271>/i x k' */
  CAVE_MachE_sil_test_B.ixk_k = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_h[2];

  /* Product: '<S271>/j x i' */
  CAVE_MachE_sil_test_B.jxi_b = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_h[0];

  /* Sum: '<S268>/Sum' */
  CAVE_MachE_sil_test_B.Sum_k[0] = CAVE_MachE_sil_test_B.jxk_f -
    CAVE_MachE_sil_test_B.kxj_b;
  CAVE_MachE_sil_test_B.Sum_k[1] = CAVE_MachE_sil_test_B.kxi_g -
    CAVE_MachE_sil_test_B.ixk_k;
  CAVE_MachE_sil_test_B.Sum_k[2] = CAVE_MachE_sil_test_B.ixj_o -
    CAVE_MachE_sil_test_B.jxi_b;

  /* Sum: '<S264>/Add1' */
  CAVE_MachE_sil_test_B.Add1_my[0] = CAVE_MachE_sil_test_B.Sum_k[0] +
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.Add1_my[1] = CAVE_MachE_sil_test_B.Sum_k[1] +
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.Add1_my[2] = CAVE_MachE_sil_test_B.Sum_k[2] +
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Product: '<S290>/j x k' */
  CAVE_MachE_sil_test_B.jxk_g = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_c[2];

  /* Product: '<S290>/k x i' */
  CAVE_MachE_sil_test_B.kxi_k = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_c[0];

  /* Product: '<S290>/i x j' */
  CAVE_MachE_sil_test_B.ixj_b = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_c[1];

  /* Product: '<S291>/k x j' */
  CAVE_MachE_sil_test_B.kxj_d = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_c[1];

  /* Product: '<S291>/i x k' */
  CAVE_MachE_sil_test_B.ixk_c = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_c[2];

  /* Product: '<S291>/j x i' */
  CAVE_MachE_sil_test_B.jxi_j = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_c[0];

  /* Sum: '<S288>/Sum' */
  CAVE_MachE_sil_test_B.Sum_hm[0] = CAVE_MachE_sil_test_B.jxk_g -
    CAVE_MachE_sil_test_B.kxj_d;
  CAVE_MachE_sil_test_B.Sum_hm[1] = CAVE_MachE_sil_test_B.kxi_k -
    CAVE_MachE_sil_test_B.ixk_c;
  CAVE_MachE_sil_test_B.Sum_hm[2] = CAVE_MachE_sil_test_B.ixj_b -
    CAVE_MachE_sil_test_B.jxi_j;

  /* Sum: '<S284>/Add1' */
  CAVE_MachE_sil_test_B.Add1_b[0] = CAVE_MachE_sil_test_B.Sum_hm[0] +
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.Add1_b[1] = CAVE_MachE_sil_test_B.Sum_hm[1] +
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.Add1_b[2] = CAVE_MachE_sil_test_B.Sum_hm[2] +
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Product: '<S298>/j x k' */
  CAVE_MachE_sil_test_B.jxk_gi = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_l[2];

  /* Product: '<S298>/k x i' */
  CAVE_MachE_sil_test_B.kxi_o = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_l[0];

  /* Product: '<S298>/i x j' */
  CAVE_MachE_sil_test_B.ixj_e = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_l[1];

  /* Product: '<S299>/k x j' */
  CAVE_MachE_sil_test_B.kxj_e = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Selector1_l[1];

  /* Product: '<S299>/i x k' */
  CAVE_MachE_sil_test_B.ixk_b = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Selector1_l[2];

  /* Product: '<S299>/j x i' */
  CAVE_MachE_sil_test_B.jxi_o = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Selector1_l[0];

  /* Sum: '<S296>/Sum' */
  CAVE_MachE_sil_test_B.Sum_p[0] = CAVE_MachE_sil_test_B.jxk_gi -
    CAVE_MachE_sil_test_B.kxj_e;
  CAVE_MachE_sil_test_B.Sum_p[1] = CAVE_MachE_sil_test_B.kxi_o -
    CAVE_MachE_sil_test_B.ixk_b;
  CAVE_MachE_sil_test_B.Sum_p[2] = CAVE_MachE_sil_test_B.ixj_e -
    CAVE_MachE_sil_test_B.jxi_o;

  /* Sum: '<S292>/Add1' */
  CAVE_MachE_sil_test_B.Add1_p[0] = CAVE_MachE_sil_test_B.Sum_p[0] +
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.Add1_p[1] = CAVE_MachE_sil_test_B.Sum_p[1] +
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.Add1_p[2] = CAVE_MachE_sil_test_B.Sum_p[2] +
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Reshape: '<S138>/Reshape2' */
  CAVE_MachE_sil_test_B.V[0] = CAVE_MachE_sil_test_B.Add1_a[0];
  CAVE_MachE_sil_test_B.V[3] = CAVE_MachE_sil_test_B.Add1_my[0];
  CAVE_MachE_sil_test_B.V[6] = CAVE_MachE_sil_test_B.Add1_b[0];
  CAVE_MachE_sil_test_B.V[9] = CAVE_MachE_sil_test_B.Add1_p[0];
  CAVE_MachE_sil_test_B.V[1] = CAVE_MachE_sil_test_B.Add1_a[1];
  CAVE_MachE_sil_test_B.V[4] = CAVE_MachE_sil_test_B.Add1_my[1];
  CAVE_MachE_sil_test_B.V[7] = CAVE_MachE_sil_test_B.Add1_b[1];
  CAVE_MachE_sil_test_B.V[10] = CAVE_MachE_sil_test_B.Add1_p[1];
  CAVE_MachE_sil_test_B.V[2] = CAVE_MachE_sil_test_B.Add1_a[2];
  CAVE_MachE_sil_test_B.V[5] = CAVE_MachE_sil_test_B.Add1_my[2];
  CAVE_MachE_sil_test_B.V[8] = CAVE_MachE_sil_test_B.Add1_b[2];
  CAVE_MachE_sil_test_B.V[11] = CAVE_MachE_sil_test_B.Add1_p[2];
  for (y = 0; y < 2; y++) {
    /* Selector: '<S134>/Selector11' */
    CAVE_MachE_sil_test_B.Selector11[3 * y] = CAVE_MachE_sil_test_B.V[3 * y];
    CAVE_MachE_sil_test_B.Selector11[3 * y + 1] = CAVE_MachE_sil_test_B.V[3 * y
      + 1];
    CAVE_MachE_sil_test_B.Selector11[3 * y + 2] = CAVE_MachE_sil_test_B.V[3 * y
      + 2];

    /* Selector: '<S140>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_f[y] = CAVE_MachE_sil_test_B.Selector11[3 *
      y + 2];

    /* Reshape: '<S140>/Reshape6' */
    CAVE_MachE_sil_test_B.Reshape6_l[y] = CAVE_MachE_sil_test_B.Selector1_f[y];

    /* Concatenate: '<S140>/Matrix Concatenate3' */
    CAVE_MachE_sil_test_B.MatrixConcatenate3[y << 1] =
      CAVE_MachE_sil_test_B.Reshape7_p[y];
    CAVE_MachE_sil_test_B.MatrixConcatenate3[(y << 1) + 1] =
      CAVE_MachE_sil_test_B.Reshape6_l[y];
  }

  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S137>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_a[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[0];
    CAVE_MachE_sil_test_B.Reshape1_a[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate2_b[1];
  }

  for (y = 0; y < 2; y++) {
    /* Selector: '<S140>/Selector4' */
    CAVE_MachE_sil_test_B.Selector4[y] = CAVE_MachE_sil_test_B.Selector11[3 * y];

    /* Reshape: '<S140>/Reshape15' */
    CAVE_MachE_sil_test_B.Reshape15[y] = CAVE_MachE_sil_test_B.Selector4[y];

    /* Selector: '<S140>/Selector5' */
    CAVE_MachE_sil_test_B.Selector5[y] = CAVE_MachE_sil_test_B.Selector11[3 * y
      + 1];

    /* Reshape: '<S140>/Reshape13' */
    CAVE_MachE_sil_test_B.Reshape13[y] = CAVE_MachE_sil_test_B.Selector5[y];

    /* Concatenate: '<S140>/Matrix Concatenate6' */
    CAVE_MachE_sil_test_B.MatrixConcatenate6[3 * y] =
      CAVE_MachE_sil_test_B.Reshape15[y];
    CAVE_MachE_sil_test_B.MatrixConcatenate6[3 * y + 1] =
      CAVE_MachE_sil_test_B.Reshape13[y];
    CAVE_MachE_sil_test_B.MatrixConcatenate6[3 * y + 2] =
      CAVE_MachE_sil_test_B.Reshape8[y];

    /* Selector: '<S134>/Selector12' */
    CAVE_MachE_sil_test_B.xdot[y] = CAVE_MachE_sil_test_B.MatrixConcatenate6[3 *
      y];

    /* Selector: '<S134>/Selector18' */
    CAVE_MachE_sil_test_B.Selector18[3 * y] = CAVE_MachE_sil_test_B.V[(y + 2) *
      3];
    CAVE_MachE_sil_test_B.Selector18[3 * y + 1] = CAVE_MachE_sil_test_B.V[(y + 2)
      * 3 + 1];
    CAVE_MachE_sil_test_B.Selector18[3 * y + 2] = CAVE_MachE_sil_test_B.V[(y + 2)
      * 3 + 2];

    /* Selector: '<S135>/Selector7' */
    CAVE_MachE_sil_test_B.Selector7[y] = CAVE_MachE_sil_test_B.Selector18[3 * y];

    /* Selector: '<S135>/Selector8' */
    CAVE_MachE_sil_test_B.Selector8[y] = CAVE_MachE_sil_test_B.Selector18[3 * y
      + 1];
  }

  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S135>/Reshape15' incorporates:
     *  Constant: '<S135>/Axle Number3'
     */
    CAVE_MachE_sil_test_B.Reshape15_o[0] =
      CAVE_MachE_sil_test_P.AxleNumber3_Value[0];
    CAVE_MachE_sil_test_B.Reshape15_o[1] =
      CAVE_MachE_sil_test_P.AxleNumber3_Value[1];

    /* Reshape: '<S141>/Reshape1' incorporates:
     *  Constant: '<S141>/Constant1'
     */
    CAVE_MachE_sil_test_B.Reshape1_p = CAVE_MachE_sil_test_P.Constant1_Value_h;
  }

  /* Concatenate: '<S135>/Matrix Concatenate5' */
  CAVE_MachE_sil_test_B.MatrixConcatenate5[0] = CAVE_MachE_sil_test_B.Selector7
    [0];
  CAVE_MachE_sil_test_B.MatrixConcatenate5[1] = CAVE_MachE_sil_test_B.Selector8
    [0];
  CAVE_MachE_sil_test_B.MatrixConcatenate5[2] =
    CAVE_MachE_sil_test_B.Reshape15_o[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate5[3] = CAVE_MachE_sil_test_B.Selector7
    [1];
  CAVE_MachE_sil_test_B.MatrixConcatenate5[4] = CAVE_MachE_sil_test_B.Selector8
    [1];
  CAVE_MachE_sil_test_B.MatrixConcatenate5[5] =
    CAVE_MachE_sil_test_B.Reshape15_o[1];

  /* SignalConversion generated from: '<S141>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate[0] = CAVE_MachE_sil_test_B.Reshape1_p;

  /* SignalConversion generated from: '<S141>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate[1] = CAVE_MachE_sil_test_B.Reshape1_p;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Memory: '<S141>/Memory1' */
    CAVE_MachE_sil_test_B.Memory1_b[0] =
      CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[0];
    CAVE_MachE_sil_test_B.Memory1_b[1] =
      CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[1];

    /* Memory: '<S141>/Memory' */
    CAVE_MachE_sil_test_B.Memory_d =
      CAVE_MachE_sil_test_DW.Memory_PreviousInput_b0;
  }

  /* Reshape: '<S135>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_c[0] = CAVE_MachE_sil_test_B.z_k[2];
  CAVE_MachE_sil_test_B.Reshape3_c[1] = CAVE_MachE_sil_test_B.z_k[3];

  /* Switch: '<S141>/Switch' */
  if (CAVE_MachE_sil_test_B.Memory_d) {
    CAVE_MachE_sil_test_B.Switch_k4[0] = CAVE_MachE_sil_test_B.Memory1_b[0];
    CAVE_MachE_sil_test_B.Switch_k4[1] = CAVE_MachE_sil_test_B.Memory1_b[1];
  } else {
    CAVE_MachE_sil_test_B.Switch_k4[0] = CAVE_MachE_sil_test_B.Reshape3_c[0];
    CAVE_MachE_sil_test_B.Switch_k4[1] = CAVE_MachE_sil_test_B.Reshape3_c[1];
  }

  /* End of Switch: '<S141>/Switch' */

  /* Reshape: '<S141>/Reshape21' */
  CAVE_MachE_sil_test_B.Reshape21[0] = CAVE_MachE_sil_test_B.Switch_k4[0];
  CAVE_MachE_sil_test_B.Reshape21[1] = CAVE_MachE_sil_test_B.Switch_k4[1];

  /* Sum: '<S141>/Sum of Elements' */
  Bias = CAVE_MachE_sil_test_B.Reshape21[0];
  Bias += CAVE_MachE_sil_test_B.Reshape21[1];
  CAVE_MachE_sil_test_B.SumofElements = Bias;

  /* Gain: '<S141>/Mean Wheel Position' */
  CAVE_MachE_sil_test_B.MatrixConcatenate[2] =
    CAVE_MachE_sil_test_P.MeanWheelPosition_Gain *
    CAVE_MachE_sil_test_B.SumofElements;

  /* Outputs for Iterator SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' incorporates:
   *  ForEach: '<S143>/For Each'
   */
  for (ForEach_itr_c = 0; ForEach_itr_c < 1; ForEach_itr_c++) {
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Reshape: '<S154>/Reshape' incorporates:
       *  Constant: '<S154>/Track coordinates in axle body frame1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[0] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Trackcoordinatesinaxlebodyframe1_Value
        [0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[1] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Trackcoordinatesinaxlebodyframe1_Value
        [1];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[2] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Trackcoordinatesinaxlebodyframe1_Value
        [2];
    }

    /* SignalConversion generated from: '<S154>/Matrix Concatenate4' incorporates:
     *  Constant: '<S154>/Track coordinates in axle body frame2'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[0] =
      CAVE_MachE_sil_test_P.CoreSubsys_m.Trackcoordinatesinaxlebodyframe2_Value;

    /* Integrator: '<S154>/ ' */
    /* Limited  Integrator  */
    if (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE >=
        CAVE_MachE_sil_test_P.CoreSubsys_m._UpperSat) {
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m._UpperSat;
    } else {
      if (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE <=
          CAVE_MachE_sil_test_P.CoreSubsys_m._LowerSat) {
        CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE =
          CAVE_MachE_sil_test_P.CoreSubsys_m._LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].u =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE;

    /* End of Integrator: '<S154>/ ' */

    /* Trigonometry: '<S154>/Trigonometric Function' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction =
      cos(CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].u);

    /* SignalConversion generated from: '<S154>/Matrix Concatenate4' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction;

    /* Trigonometry: '<S154>/Trigonometric Function1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction1 =
      sin(CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].u);

    /* SignalConversion generated from: '<S154>/Matrix Concatenate4' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction1;

    /* SignalConversion generated from: '<S154>/Matrix Concatenate5' incorporates:
     *  Constant: '<S154>/Track coordinates in axle body frame2'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[0] =
      CAVE_MachE_sil_test_P.CoreSubsys_m.Trackcoordinatesinaxlebodyframe2_Value;

    /* Gain: '<S154>/Gain' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[1] =
      CAVE_MachE_sil_test_P.CoreSubsys_m.Gain_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction1;

    /* SignalConversion generated from: '<S154>/Matrix Concatenate5' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction;

    /* Concatenate: '<S154>/Matrix Concatenate3' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[0];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[0];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[0];

    /* Integrator: '<S150>/cg coordinates' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[0] =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[0];

    /* Concatenate: '<S154>/Matrix Concatenate3' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[3] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[1];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[4] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[1];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[5] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[1];

    /* Integrator: '<S150>/cg coordinates' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[1] =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[1];

    /* Concatenate: '<S154>/Matrix Concatenate3' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[6] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[2];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[7] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[2];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[8] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[2];

    /* Integrator: '<S150>/cg coordinates' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[2] =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[2];

    /* Math: '<S150>/Math Function1' */
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction1[3 * ibmat]
        = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c]
        .MatrixConcatenate3[ibmat];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction1[3 * ibmat
        + 1] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        MatrixConcatenate3[ibmat + 3];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction1[3 * ibmat
        + 2] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        MatrixConcatenate3[ibmat + 6];
    }

    /* End of Math: '<S150>/Math Function1' */

    /* Integrator: '<S148>/Vz' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zdot =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vz_CSTATE;

    /* SignalConversion generated from: '<S150>/Matrix Multiply2' incorporates:
     *  Constant: '<S148>/Fy1'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[0] =
      CAVE_MachE_sil_test_P.CoreSubsys_m.Fy1_Value;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[1] =
      CAVE_MachE_sil_test_P.CoreSubsys_m.Fy1_Value;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zdot;

    /* Product: '<S150>/Matrix Multiply2' */
    memcpy(&u[0], &CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
           MathFunction1[0], 9U * sizeof(real_T));
    b_a[0] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[0];
    b_a[1] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[1];
    b_a[2] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      TmpSignalConversionAtMatrixMultiply2Inport2[2];
    for (ibmat = 0; ibmat < 3; ibmat++) {
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = u[ibmat] * b_a[0];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[ibmat + 3] *
        b_a[1];
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[ibmat + 6] *
        b_a[2];
      tmp[ibmat] = rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
    }

    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[0] = tmp[0];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[1] = tmp[1];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[2] = tmp[2];

    /* End of Product: '<S150>/Matrix Multiply2' */

    /* Integrator: '<S148>/Vy1' */
    /* Limited  Integrator  */
    if (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE >=
        CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_UpperSat) {
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_UpperSat;
    } else {
      if (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE <=
          CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_LowerSat) {
        CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE =
          CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_LowerSat;
      }
    }

    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE;

    /* End of Integrator: '<S148>/Vy1' */
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Selector: '<S151>/y axis track coordinates' incorporates:
       *  Constant: '<S151>/Track coordinates in axle body frame'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates[0]
        = CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackCoords[1];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates[1]
        = CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackCoords[4];

      /* Math: '<S151>/Math Function' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [0] * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates[0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [1] * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates[1];

      /* Selector: '<S149>/y axis track coordinates' incorporates:
       *  Constant: '<S149>/Suspension connection point coordinates in axle body frame'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates_o
        [0] = CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_SuspCoords[1];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates_o
        [1] = CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_SuspCoords[4];
    }

    /* Product: '<S148>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product1 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p *
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zdot;

    /* Sum: '<S148>/Sum' incorporates:
     *  Constant: '<S148>/Fy'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product1 +
      CAVE_MachE_sil_test_P.CoreSubsys_m.Fy_Value;

    /* Integrator: '<S148>/Vy' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydot =
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy_CSTATE;

    /* Product: '<S148>/Vyp' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydotp =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p *
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydot;

    /* ForEachSliceAssignment generated from: '<S143>/p' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_p_at_inport_0 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p;

    /* ForEachSliceAssignment generated from: '<S143>/cgV' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[ForEach_itr_c * 3] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[ForEach_itr_c * 3 +
      1] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[ForEach_itr_c * 3 +
      2] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[2];

    /* ForEachSliceAssignment generated from: '<S143>/cgP' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[ForEach_itr_c * 3] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[ForEach_itr_c * 3 +
      1] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[ForEach_itr_c * 3 +
      2] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[2];

    /* ForEachSliceAssignment generated from: '<S143>/DCM' */
    ibmat = ForEach_itr_c * 3;
    for (y = 0; y < 3; y++) {
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[ibmat + 3 * y] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate3[3 *
        y];
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[(ibmat + 3 * y) +
        1] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        MatrixConcatenate3[3 * y + 1];
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[(ibmat + 3 * y) +
        2] = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        MatrixConcatenate3[3 * y + 2];
    }

    /* End of ForEachSliceAssignment generated from: '<S143>/DCM' */
  }

  /* End of Outputs for SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */

  /* Reshape: '<S135>/Reshape8' */
  CAVE_MachE_sil_test_B.Reshape8_d[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[0];

  /* Sum: '<S141>/Sum' */
  CAVE_MachE_sil_test_B.Sum_f[0] = CAVE_MachE_sil_test_B.MatrixConcatenate[0] +
    CAVE_MachE_sil_test_B.Reshape8_d[0];

  /* Reshape: '<S135>/Reshape9' */
  CAVE_MachE_sil_test_B.Reshape9_c[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[0];

  /* Reshape: '<S135>/Reshape8' */
  CAVE_MachE_sil_test_B.Reshape8_d[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[1];

  /* Sum: '<S141>/Sum' */
  CAVE_MachE_sil_test_B.Sum_f[1] = CAVE_MachE_sil_test_B.MatrixConcatenate[1] +
    CAVE_MachE_sil_test_B.Reshape8_d[1];

  /* Reshape: '<S135>/Reshape9' */
  CAVE_MachE_sil_test_B.Reshape9_c[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[1];

  /* Reshape: '<S135>/Reshape8' */
  CAVE_MachE_sil_test_B.Reshape8_d[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgP_at_inport_0[2];

  /* Sum: '<S141>/Sum' */
  CAVE_MachE_sil_test_B.Sum_f[2] = CAVE_MachE_sil_test_B.MatrixConcatenate[2] +
    CAVE_MachE_sil_test_B.Reshape8_d[2];

  /* Reshape: '<S135>/Reshape9' */
  CAVE_MachE_sil_test_B.Reshape9_c[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_cgV_at_inport_0[2];

  /* Outputs for Iterator SubSystem: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' incorporates:
   *  ForEach: '<S142>/For Each'
   */
  for (ForEach_itr_o = 0; ForEach_itr_o < 2; ForEach_itr_o++) {
    /* ForEachSliceSelector generated from: '<S142>/Axle Number' incorporates:
     *  Constant: '<S135>/Axle Number2'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_e4 =
      CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[ForEach_itr_o];
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Constant: '<S142>/Constant1' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[0] =
        CAVE_MachE_sil_test_P.CoreSubsys.Constant1_Value;

      /* Sum: '<S146>/Sum' incorporates:
       *  Constant: '<S146>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum =
        rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_e4 -
        CAVE_MachE_sil_test_P.CoreSubsys.Constant1_Value_b;

      /* Gain: '<S146>/DCM Staring Row' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].DCMStaringRow =
        CAVE_MachE_sil_test_P.CoreSubsys.DCMStaringRow_Gain *
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum;

      /* Sum: '<S146>/Sum1' incorporates:
       *  Constant: '<S146>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1 =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].DCMStaringRow +
        CAVE_MachE_sil_test_P.CoreSubsys.Constant1_Value_b;
    }

    /* Selector: '<S142>/Selector6' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6 =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_p_at_inport_0;

    /* Trigonometry: '<S142>/Trigonometric Function' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction = sin
      (CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6);

    /* Gain: '<S142>/Gain' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Gain =
      CAVE_MachE_sil_test_P.CoreSubsys.Gain_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction;

    /* Selector: '<S146>/Select DCM' */
    ibmat = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1;
    for (y = 0; y < 3; y++) {
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[3 * y] =
        CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[(3 * y + ibmat)
        - 1];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[3 * y + 1] =
        CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[3 * y + ibmat];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[3 * y + 2] =
        CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_DCM_at_inport_0[(3 * y + ibmat)
        + 1];
    }

    /* End of Selector: '<S146>/Select DCM' */

    /* Math: '<S142>/Math Function' */
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction[3 * ibmat] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[ibmat];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction[3 * ibmat + 1]
        = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[ibmat + 3];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction[3 * ibmat + 2]
        = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[ibmat + 6];
    }

    /* End of Math: '<S142>/Math Function' */
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Selector: '<S142>/Selector1' incorporates:
       *  Constant: '<S135>/Track Number2'
       *  Constant: '<S142>/Suspension axle connection coordinates in axle body frame'
       *  ForEachSliceSelector generated from: '<S142>/Coordinate Number'
       */
      ibmat = (int32_T)CAVE_MachE_sil_test_P.TrackNumber2_Value[ForEach_itr_o];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[0] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_SuspCoords[(ibmat -
        1) * 3];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[1] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_SuspCoords[(ibmat -
        1) * 3 + 1];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[2] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_SuspCoords[(ibmat -
        1) * 3 + 2];
    }

    /* Product: '<S142>/Matrix Multiply1' */
    memcpy(&u[0], &CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction
           [0], 9U * sizeof(real_T));
    b_a[0] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[0];
    b_a[1] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[1];
    b_a[2] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[2];

    /* Product: '<S142>/Product' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[1] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Gain *
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6;

    /* Trigonometry: '<S142>/Trigonometric Function1' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction1 = cos
      (CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6);

    /* Product: '<S142>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[2] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6 *
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction1;
    for (y = 0; y < 3; y++) {
      /* Product: '<S142>/Matrix Multiply1' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[y] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[y] += u[y]
        * b_a[0];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[y] += u[y
        + 3] * b_a[1];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[y] += u[y
        + 6] * b_a[2];

      /* Selector: '<S142>/Selector2' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[y] = CAVE_MachE_sil_test_B.Sum_f[y];

      /* Sum: '<S142>/Sum1' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[y] +
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[y];

      /* Selector: '<S142>/Selector5' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[y] =
        CAVE_MachE_sil_test_B.Reshape9_c[y];

      /* Product: '<S142>/Product2' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product2[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[y] *
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[1];

      /* Sum: '<S142>/Sum3' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[y] +
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product2[y];
    }

    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Selector: '<S142>/Selector' incorporates:
       *  Constant: '<S135>/Track Number2'
       *  Constant: '<S142>/Track coordinates in axle body frame'
       *  ForEachSliceSelector generated from: '<S142>/Coordinate Number'
       */
      ibmat = (int32_T)CAVE_MachE_sil_test_P.TrackNumber2_Value[ForEach_itr_o];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[0] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackCoords[(ibmat -
        1) * 3];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[1] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackCoords[(ibmat -
        1) * 3 + 1];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[2] =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackCoords[(ibmat -
        1) * 3 + 2];
    }

    /* Product: '<S142>/Matrix Multiply3' */
    memcpy(&u[0], &CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction
           [0], 9U * sizeof(real_T));
    b_a[0] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[0];
    b_a[1] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[1];
    b_a[2] = CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[2];
    for (y = 0; y < 3; y++) {
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o]
        .Rotationeffectonpositions[y] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o]
        .Rotationeffectonpositions[y] += u[y] * b_a[0];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o]
        .Rotationeffectonpositions[y] += u[y + 3] * b_a[1];
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o]
        .Rotationeffectonpositions[y] += u[y + 6] * b_a[2];

      /* Sum: '<S142>/Sum4' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[y] +
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Rotationeffectonpositions[y];

      /* Product: '<S142>/Product1' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product1[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[1] *
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[y];

      /* Sum: '<S142>/Sum2' */
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[y] =
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[y] +
        CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product1[y];
    }

    /* End of Product: '<S142>/Matrix Multiply3' */

    /* ForEachSliceAssignment generated from: '<S142>/WhlV' */
    ibmat = ForEach_itr_o * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlV_at_inport_0[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlV_at_inport_0[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlV_at_inport_0[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[2];

    /* ForEachSliceAssignment generated from: '<S142>/WhlP' */
    ibmat = ForEach_itr_o * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlP_at_inport_0[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlP_at_inport_0[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlP_at_inport_0[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[2];

    /* ForEachSliceAssignment generated from: '<S142>/SuspV' */
    ibmat = ForEach_itr_o * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspV_at_inport_0[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspV_at_inport_0[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspV_at_inport_0[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[2];

    /* ForEachSliceAssignment generated from: '<S142>/SuspP' */
    ibmat = ForEach_itr_o * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspP_at_inport_0[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspP_at_inport_0[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspP_at_inport_0[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[2];
  }

  /* End of Outputs for SubSystem: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
  for (y = 0; y < 6; y++) {
    /* Reshape: '<S135>/Reshape5' */
    CAVE_MachE_sil_test_B.Reshape5[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlV_at_inport_0[y];

    /* Sum: '<S135>/Sum2' */
    CAVE_MachE_sil_test_B.Sum2[y] = CAVE_MachE_sil_test_B.MatrixConcatenate5[y]
      + CAVE_MachE_sil_test_B.Reshape5[y];
  }

  /* Selector: '<S134>/Selector14' */
  CAVE_MachE_sil_test_B.xdot[2] = CAVE_MachE_sil_test_B.Sum2[0];

  /* Selector: '<S134>/Selector13' */
  CAVE_MachE_sil_test_B.ydot[0] = CAVE_MachE_sil_test_B.MatrixConcatenate6[1];

  /* Selector: '<S134>/Selector15' */
  CAVE_MachE_sil_test_B.ydot[2] = CAVE_MachE_sil_test_B.Sum2[1];

  /* Selector: '<S134>/Selector14' */
  CAVE_MachE_sil_test_B.xdot[3] = CAVE_MachE_sil_test_B.Sum2[3];

  /* Selector: '<S134>/Selector13' */
  CAVE_MachE_sil_test_B.ydot[1] = CAVE_MachE_sil_test_B.MatrixConcatenate6[4];

  /* Selector: '<S134>/Selector15' */
  CAVE_MachE_sil_test_B.ydot[3] = CAVE_MachE_sil_test_B.Sum2[4];

  /* Reshape: '<S317>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_o[0] = CAVE_MachE_sil_test_B.zdot[0];

  /* Concatenate: '<S128>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[0] = CAVE_MachE_sil_test_B.xdot[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[1] = CAVE_MachE_sil_test_B.ydot[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[2] =
    CAVE_MachE_sil_test_B.Reshape1_o[0];

  /* Reshape: '<S317>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_o[1] = CAVE_MachE_sil_test_B.zdot[1];

  /* Concatenate: '<S128>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[3] = CAVE_MachE_sil_test_B.xdot[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[4] = CAVE_MachE_sil_test_B.ydot[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[5] =
    CAVE_MachE_sil_test_B.Reshape1_o[1];

  /* Reshape: '<S317>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_o[2] = CAVE_MachE_sil_test_B.zdot[2];

  /* Concatenate: '<S128>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[6] = CAVE_MachE_sil_test_B.xdot[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[7] = CAVE_MachE_sil_test_B.ydot[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[8] =
    CAVE_MachE_sil_test_B.Reshape1_o[2];

  /* Reshape: '<S317>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_o[3] = CAVE_MachE_sil_test_B.zdot[3];

  /* Concatenate: '<S128>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[9] = CAVE_MachE_sil_test_B.xdot[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[10] = CAVE_MachE_sil_test_B.ydot[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_j[11] =
    CAVE_MachE_sil_test_B.Reshape1_o[3];

  /* Sum: '<S443>/Add2' incorporates:
   *  Constant: '<S443>/Constant1'
   */
  CAVE_MachE_sil_test_B.Add2_f[0] = CAVE_MachE_sil_test_B.CamberAngles_l[0] +
    CAVE_MachE_sil_test_P.Constant1_Value_c;
  CAVE_MachE_sil_test_B.Add2_f[1] = CAVE_MachE_sil_test_B.CamberAngles_l[1] +
    CAVE_MachE_sil_test_P.Constant1_Value_c;
  CAVE_MachE_sil_test_B.Add2_f[2] = CAVE_MachE_sil_test_B.CamberAngles_l[2] +
    CAVE_MachE_sil_test_P.Constant1_Value_c;
  CAVE_MachE_sil_test_B.Add2_f[3] = CAVE_MachE_sil_test_B.CamberAngles_l[3] +
    CAVE_MachE_sil_test_P.Constant1_Value_c;

  /* Reshape: '<S443>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_i[0] = CAVE_MachE_sil_test_B.Add2_f[0];
  CAVE_MachE_sil_test_B.Reshape1_i[1] = CAVE_MachE_sil_test_B.Add2_f[1];
  CAVE_MachE_sil_test_B.Reshape1_i[2] = CAVE_MachE_sil_test_B.Add2_f[2];
  CAVE_MachE_sil_test_B.Reshape1_i[3] = CAVE_MachE_sil_test_B.Add2_f[3];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S443>/Reshape2' incorporates:
     *  Constant: '<S443>/Constant3'
     */
    CAVE_MachE_sil_test_B.Reshape2_p[0] =
      CAVE_MachE_sil_test_P.Constant3_Value_oo[0];
    CAVE_MachE_sil_test_B.Reshape2_p[1] =
      CAVE_MachE_sil_test_P.Constant3_Value_oo[1];
    CAVE_MachE_sil_test_B.Reshape2_p[2] =
      CAVE_MachE_sil_test_P.Constant3_Value_oo[2];
    CAVE_MachE_sil_test_B.Reshape2_p[3] =
      CAVE_MachE_sil_test_P.Constant3_Value_oo[3];
  }

  /* Sum: '<S443>/Add1' incorporates:
   *  Constant: '<S443>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_h[0] = CAVE_MachE_sil_test_B.WheelAngles[0] +
    CAVE_MachE_sil_test_P.Constant2_Value_f[0];

  /* Reshape: '<S443>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_p[0] = CAVE_MachE_sil_test_B.Add1_h[0];

  /* Concatenate: '<S443>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[0] =
    CAVE_MachE_sil_test_B.Reshape1_i[0];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[1] =
    CAVE_MachE_sil_test_B.Reshape2_p[0];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[2] =
    CAVE_MachE_sil_test_B.Reshape_p[0];

  /* Sum: '<S443>/Add1' incorporates:
   *  Constant: '<S443>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_h[1] = CAVE_MachE_sil_test_B.WheelAngles[1] +
    CAVE_MachE_sil_test_P.Constant2_Value_f[1];

  /* Reshape: '<S443>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_p[1] = CAVE_MachE_sil_test_B.Add1_h[1];

  /* Concatenate: '<S443>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[3] =
    CAVE_MachE_sil_test_B.Reshape1_i[1];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[4] =
    CAVE_MachE_sil_test_B.Reshape2_p[1];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[5] =
    CAVE_MachE_sil_test_B.Reshape_p[1];

  /* Sum: '<S443>/Add1' incorporates:
   *  Constant: '<S443>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_h[2] = CAVE_MachE_sil_test_B.WheelAngles[2] +
    CAVE_MachE_sil_test_P.Constant2_Value_f[2];

  /* Reshape: '<S443>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_p[2] = CAVE_MachE_sil_test_B.Add1_h[2];

  /* Concatenate: '<S443>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[6] =
    CAVE_MachE_sil_test_B.Reshape1_i[2];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[7] =
    CAVE_MachE_sil_test_B.Reshape2_p[2];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[8] =
    CAVE_MachE_sil_test_B.Reshape_p[2];

  /* Sum: '<S443>/Add1' incorporates:
   *  Constant: '<S443>/Constant2'
   */
  CAVE_MachE_sil_test_B.Add1_h[3] = CAVE_MachE_sil_test_B.WheelAngles[3] +
    CAVE_MachE_sil_test_P.Constant2_Value_f[3];

  /* Reshape: '<S443>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_p[3] = CAVE_MachE_sil_test_B.Add1_h[3];

  /* Concatenate: '<S443>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[9] =
    CAVE_MachE_sil_test_B.Reshape1_i[3];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[10] =
    CAVE_MachE_sil_test_B.Reshape2_p[3];
  CAVE_MachE_sil_test_B.VectorConcatenate3_p[11] =
    CAVE_MachE_sil_test_B.Reshape_p[3];

  /* Outputs for Iterator SubSystem: '<S315>/Wheel to Body Transform' incorporates:
   *  ForEach: '<S444>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* ForEachSliceSelector generated from: '<S444>/WheelAngles' */
    Bias = CAVE_MachE_sil_test_B.VectorConcatenate3_p[3 * ForEach_itr];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.VectorConcatenate3_p[3 * ForEach_itr + 1];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 =
      CAVE_MachE_sil_test_B.VectorConcatenate3_p[3 * ForEach_itr + 2];

    /* SignalConversion generated from: '<S445>/sincos' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[0] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[1] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[2] = Bias;

    /* Trigonometry: '<S445>/sincos' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[0];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[1];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
      TmpSignalConversionAtsincosInport1[2];
    Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
      (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] = Bias;
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

    /* Fcn: '<S445>/Fcn11' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0];

    /* Fcn: '<S445>/Fcn21' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0];

    /* Fcn: '<S445>/Fcn31' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0];

    /* Fcn: '<S445>/Fcn12' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[3] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0];

    /* Fcn: '<S445>/Fcn22' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[4] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0];

    /* Fcn: '<S445>/Fcn32' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[5] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0];

    /* Fcn: '<S445>/Fcn13' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[6] =
      -CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1];

    /* Fcn: '<S445>/Fcn23' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[7] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1];

    /* Fcn: '<S445>/Fcn33' */
    CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[8] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] *
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1];
    for (y = 0; y < 9; y++) {
      /* Reshape: '<S446>/Reshape (9) to [3x3] column-major' */
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr]
        .Reshape9to3x3columnmajor[y] =
        CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].VectorConcatenate[y];

      /* Product: '<S444>/Divide1' */
      u[y] = CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
        Reshape9to3x3columnmajor[y];
    }

    /* Product: '<S444>/Divide1' incorporates:
     *  ForEachSliceSelector generated from: '<S444>/VelVeh'
     */
    b_a[0] = CAVE_MachE_sil_test_B.MatrixConcatenate_j[3 * ForEach_itr];
    b_a[1] = CAVE_MachE_sil_test_B.MatrixConcatenate_j[3 * ForEach_itr + 1];
    b_a[2] = CAVE_MachE_sil_test_B.MatrixConcatenate_j[3 * ForEach_itr + 2];
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[ibmat] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[ibmat] +=
        u[ibmat] * b_a[0];
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[ibmat] +=
        u[ibmat + 3] * b_a[1];
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[ibmat] +=
        u[ibmat + 6] * b_a[2];
    }

    /* ForEachSliceAssignment generated from: '<S444>/zdotWheel' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_zdotWheel_at_inport_0[ForEach_itr] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[2];

    /* ForEachSliceAssignment generated from: '<S444>/ydotWheel' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[ForEach_itr] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[1];

    /* ForEachSliceAssignment generated from: '<S444>/xdotWheel' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[ForEach_itr] =
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[0];
  }

  /* End of Outputs for SubSystem: '<S315>/Wheel to Body Transform' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S136>/Reshape2' incorporates:
     *  Constant: '<S136>/SteerRates'
     */
    CAVE_MachE_sil_test_B.Reshape2_b[0] =
      CAVE_MachE_sil_test_P.SteerRates_Value[0];

    /* Concatenate: '<S136>/Matrix Concatenate1' */
    CAVE_MachE_sil_test_B.MatrixConcatenate1[0] =
      CAVE_MachE_sil_test_B.Reshape2_b[0];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[1] =
      CAVE_MachE_sil_test_B.Reshape2_b[0];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[2] =
      CAVE_MachE_sil_test_B.Reshape2_b[0];

    /* Reshape: '<S136>/Reshape3' incorporates:
     *  Constant: '<S136>/Constant'
     */
    CAVE_MachE_sil_test_B.Reshape3_n[0] =
      CAVE_MachE_sil_test_P.Constant_Value_lp[0];

    /* Reshape: '<S136>/Reshape2' incorporates:
     *  Constant: '<S136>/SteerRates'
     */
    CAVE_MachE_sil_test_B.Reshape2_b[1] =
      CAVE_MachE_sil_test_P.SteerRates_Value[1];

    /* Concatenate: '<S136>/Matrix Concatenate1' */
    CAVE_MachE_sil_test_B.MatrixConcatenate1[3] =
      CAVE_MachE_sil_test_B.Reshape2_b[1];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[4] =
      CAVE_MachE_sil_test_B.Reshape2_b[1];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[5] =
      CAVE_MachE_sil_test_B.Reshape2_b[1];

    /* Reshape: '<S136>/Reshape3' incorporates:
     *  Constant: '<S136>/Constant'
     */
    CAVE_MachE_sil_test_B.Reshape3_n[1] =
      CAVE_MachE_sil_test_P.Constant_Value_lp[1];

    /* Reshape: '<S136>/Reshape2' incorporates:
     *  Constant: '<S136>/SteerRates'
     */
    CAVE_MachE_sil_test_B.Reshape2_b[2] =
      CAVE_MachE_sil_test_P.SteerRates_Value[2];

    /* Concatenate: '<S136>/Matrix Concatenate1' */
    CAVE_MachE_sil_test_B.MatrixConcatenate1[6] =
      CAVE_MachE_sil_test_B.Reshape2_b[2];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[7] =
      CAVE_MachE_sil_test_B.Reshape2_b[2];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[8] =
      CAVE_MachE_sil_test_B.Reshape2_b[2];

    /* Reshape: '<S136>/Reshape3' incorporates:
     *  Constant: '<S136>/Constant'
     */
    CAVE_MachE_sil_test_B.Reshape3_n[2] =
      CAVE_MachE_sil_test_P.Constant_Value_lp[2];

    /* Reshape: '<S136>/Reshape2' incorporates:
     *  Constant: '<S136>/SteerRates'
     */
    CAVE_MachE_sil_test_B.Reshape2_b[3] =
      CAVE_MachE_sil_test_P.SteerRates_Value[3];

    /* Concatenate: '<S136>/Matrix Concatenate1' */
    CAVE_MachE_sil_test_B.MatrixConcatenate1[9] =
      CAVE_MachE_sil_test_B.Reshape2_b[3];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[10] =
      CAVE_MachE_sil_test_B.Reshape2_b[3];
    CAVE_MachE_sil_test_B.MatrixConcatenate1[11] =
      CAVE_MachE_sil_test_B.Reshape2_b[3];

    /* Reshape: '<S136>/Reshape3' incorporates:
     *  Constant: '<S136>/Constant'
     */
    CAVE_MachE_sil_test_B.Reshape3_n[3] =
      CAVE_MachE_sil_test_P.Constant_Value_lp[3];
  }

  /* Reshape: '<S136>/Reshape4' */
  CAVE_MachE_sil_test_B.Reshape4_g[0] = CAVE_MachE_sil_test_B.pqr[2];
  CAVE_MachE_sil_test_B.Reshape4_g[1] = CAVE_MachE_sil_test_B.pqr[2];
  CAVE_MachE_sil_test_B.Reshape4_g[2] = CAVE_MachE_sil_test_B.pqr[2];
  CAVE_MachE_sil_test_B.Reshape4_g[3] = CAVE_MachE_sil_test_B.pqr[2];

  /* Concatenate: '<S136>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[0] =
    CAVE_MachE_sil_test_B.Reshape3_n[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[1] =
    CAVE_MachE_sil_test_B.Reshape3_n[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[2] =
    CAVE_MachE_sil_test_B.Reshape4_g[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[3] =
    CAVE_MachE_sil_test_B.Reshape3_n[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[4] =
    CAVE_MachE_sil_test_B.Reshape3_n[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[5] =
    CAVE_MachE_sil_test_B.Reshape4_g[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[6] =
    CAVE_MachE_sil_test_B.Reshape3_n[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[7] =
    CAVE_MachE_sil_test_B.Reshape3_n[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[8] =
    CAVE_MachE_sil_test_B.Reshape4_g[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[9] =
    CAVE_MachE_sil_test_B.Reshape3_n[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[10] =
    CAVE_MachE_sil_test_B.Reshape3_n[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_f[11] =
    CAVE_MachE_sil_test_B.Reshape4_g[3];

  /* Sum: '<S136>/Add' */
  for (y = 0; y < 12; y++) {
    CAVE_MachE_sil_test_B.AngVel[y] = CAVE_MachE_sil_test_B.MatrixConcatenate1[y]
      + CAVE_MachE_sil_test_B.MatrixConcatenate_f[y];
  }

  /* End of Sum: '<S136>/Add' */

  /* Selector: '<S128>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_d[0] = CAVE_MachE_sil_test_B.AngVel[2];

  /* Reshape: '<S128>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_e[0] = CAVE_MachE_sil_test_B.Selector1_d[0];

  /* Selector: '<S128>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_d[1] = CAVE_MachE_sil_test_B.AngVel[5];

  /* Reshape: '<S128>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_e[1] = CAVE_MachE_sil_test_B.Selector1_d[1];

  /* Selector: '<S128>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_d[2] = CAVE_MachE_sil_test_B.AngVel[8];

  /* Reshape: '<S128>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_e[2] = CAVE_MachE_sil_test_B.Selector1_d[2];

  /* Selector: '<S128>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_d[3] = CAVE_MachE_sil_test_B.AngVel[11];

  /* Reshape: '<S128>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_e[3] = CAVE_MachE_sil_test_B.Selector1_d[3];

  /* UnaryMinus: '<S315>/Unary Minus' */
  CAVE_MachE_sil_test_B.UnaryMinus_m[0] = -CAVE_MachE_sil_test_B.Reshape1_e[0];
  CAVE_MachE_sil_test_B.UnaryMinus_m[1] = -CAVE_MachE_sil_test_B.Reshape1_e[1];
  CAVE_MachE_sil_test_B.UnaryMinus_m[2] = -CAVE_MachE_sil_test_B.Reshape1_e[2];
  CAVE_MachE_sil_test_B.UnaryMinus_m[3] = -CAVE_MachE_sil_test_B.Reshape1_e[3];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    for (y = 0; y < 4; y++) {
      /* Reshape: '<S316>/Reshape' */
      CAVE_MachE_sil_test_B.Reshape_ic[y] =
        CAVE_MachE_sil_test_B.VectorConcatenate1[y];

      /* Reshape: '<S316>/Reshape1' */
      CAVE_MachE_sil_test_B.Reshape1_j[y] =
        CAVE_MachE_sil_test_B.VectorConcatenate1[y];

      /* Concatenate: '<S316>/Vector Concatenate' incorporates:
       *  Constant: '<S316>/0'
       *  Constant: '<S316>/ones'
       *  Constant: '<S316>/ones2'
       */
      CAVE_MachE_sil_test_B.VectorConcatenate_pm[27 * y] =
        CAVE_MachE_sil_test_P.ones2_Value[y];
      CAVE_MachE_sil_test_B.VectorConcatenate_pm[27 * y + 1] =
        CAVE_MachE_sil_test_B.Reshape_ic[y];
      CAVE_MachE_sil_test_B.VectorConcatenate_pm[27 * y + 2] =
        CAVE_MachE_sil_test_B.Reshape1_j[y];
      CAVE_MachE_sil_test_B.VectorConcatenate_pm[27 * y + 3] =
        CAVE_MachE_sil_test_P.u_Value[y];
      memcpy(&CAVE_MachE_sil_test_B.VectorConcatenate_pm[y * 27 + 4],
             &CAVE_MachE_sil_test_P.ones_Value[y * 23], 23U * sizeof(real_T));
    }

    /* Selector: '<S322>/Selector9' */
    memcpy(&CAVE_MachE_sil_test_B.Selector9[0],
           &CAVE_MachE_sil_test_B.VectorConcatenate_pm[0], 27U * sizeof(real_T));
  }

  /* MATLAB Function: '<S344>/Magic Tire Const Input' incorporates:
   *  Constant: '<S312>/Pressure'
   */
  Bias = 1.0 * CAVE_MachE_sil_test_P.Pressure_const;
  CAVE_MachE_sil_test_MagicTireConstInput(CAVE_MachE_sil_test_B.sf_LockUp.Omega,
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[0],
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[0],
    CAVE_MachE_sil_test_B.UnaryMinus_m[0], CAVE_MachE_sil_test_B.CamberAngles_l
    [0], Bias, CAVE_MachE_sil_test_B.Selector9, CAVE_MachE_sil_test_B.Saturation,
    CAVE_MachE_sil_test_B.Integrator_a, CAVE_MachE_sil_test_B.Integrator_fu,
    &CAVE_MachE_sil_test_B.VectorConcatenate_jk[0],
    &CAVE_MachE_sil_test_B.VectorConcatenate_b[0],
    &CAVE_MachE_sil_test_B.VectorConcatenate_ol[0],
    &CAVE_MachE_sil_test_B.VectorConcatenate_bc[0],
    &CAVE_MachE_sil_test_B.sf_MagicTireConstInput,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_ALPMAX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_ALPMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_BREFF,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_CAMMAX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_CAMMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_DREFF,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_FNOMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_FREFF,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_FZMAX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_FZMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_KPUMAX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_KPUMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_LATERAL_STIFFNESS,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_LONGITUDINAL_STIFFNESS,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_LONGVL,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_NOMPRES,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCFY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PCY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDXP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDXP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDXP3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDYP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDYP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDYP3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PDYP4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PECP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PECP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEX4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PEY5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PFZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHYP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHYP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHYP3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PHYP4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY6,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKY7,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PKYP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPMX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPX4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPY5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PPZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PRESMAX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PRESMIN,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PVX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PVX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PVY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_PVY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBRP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ10,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ6,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QBZ9,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QCRP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QCRP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QCZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDRP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDRP2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDTP1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ10,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ11,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ8,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QDZ9,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QEZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QEZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QEZ3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QEZ5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QHZ3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QHZ4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX10,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX11,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX12,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX13,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX14,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX6,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX7,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX8,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSX9,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY6,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY7,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_QSY8,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FCX,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FCY,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FCY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FZ1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_FZ3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_RA1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_RA2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_RB1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_RB2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_RE0,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_V1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_Q_V2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBX3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RBY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RCX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RCY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_REX1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_REX2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_REY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_REY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RHY1,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RHY2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RVY3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RVY4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RVY5,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_RVY6,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_SSZ2,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_SSZ3,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_SSZ4,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_UNLOADED_RADIUS,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VERTICAL_STIFFNESS,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW,
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_WIDTH);

  /* SignalConversion generated from: '<S333>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_md[0] =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Re;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S322>/Selector19' */
    memcpy(&CAVE_MachE_sil_test_B.Selector19[0],
           &CAVE_MachE_sil_test_B.VectorConcatenate_pm[27], 27U * sizeof(real_T));
  }

  /* MATLAB Function: '<S369>/Magic Tire Const Input' incorporates:
   *  Constant: '<S312>/Pressure'
   */
  Bias = 1.0 * CAVE_MachE_sil_test_P.Pressure_const;
  CAVE_MachE_sil_test_MagicTireConstInput
    (CAVE_MachE_sil_test_B.sf_LockUp_n.Omega,
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[1],
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[1],
     CAVE_MachE_sil_test_B.UnaryMinus_m[1],
     CAVE_MachE_sil_test_B.CamberAngles_l[1], Bias,
     CAVE_MachE_sil_test_B.Selector19, CAVE_MachE_sil_test_B.Saturation_d,
     CAVE_MachE_sil_test_B.Integrator_l, CAVE_MachE_sil_test_B.Integrator_n,
     &CAVE_MachE_sil_test_B.VectorConcatenate_jk[1],
     &CAVE_MachE_sil_test_B.VectorConcatenate_b[1],
     &CAVE_MachE_sil_test_B.VectorConcatenate_ol[1],
     &CAVE_MachE_sil_test_B.VectorConcatenate_bc[1],
     &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_ALPMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_ALPMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_BREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_CAMMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_CAMMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_DREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_FNOMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_FREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_FZMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_FZMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_KPUMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_KPUMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_LATERAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_LONGITUDINAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_LONGVL,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_NOMPRES,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCFY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDXP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDXP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDXP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PDYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PECP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PECP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PEY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PFZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PHYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PKYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPMX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PPZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PRESMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PRESMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PVX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PVX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_PVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QBZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QCRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QCRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QCZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDTP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QDZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QEZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QEZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QEZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QEZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QHZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QHZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX12,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX13,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX14,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSX9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_QSY8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FCX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FCY,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FCY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_FZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_RA1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_RA2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_RB1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_RB2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_RE0,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_V1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_Q_V2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RBY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_REX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_REX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_REY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_REY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RHY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RHY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RVY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_RVY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_SSZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_SSZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_SSZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_UNLOADED_RADIUS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VERTICAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_WIDTH);

  /* SignalConversion generated from: '<S333>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_md[1] =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Re;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S322>/Selector29' */
    memcpy(&CAVE_MachE_sil_test_B.Selector29[0],
           &CAVE_MachE_sil_test_B.VectorConcatenate_pm[54], 27U * sizeof(real_T));
  }

  /* MATLAB Function: '<S394>/Magic Tire Const Input' incorporates:
   *  Constant: '<S312>/Pressure'
   */
  Bias = 1.0 * CAVE_MachE_sil_test_P.Pressure_const;
  CAVE_MachE_sil_test_MagicTireConstInput
    (CAVE_MachE_sil_test_B.sf_LockUp_h.Omega,
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[2],
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[2],
     CAVE_MachE_sil_test_B.UnaryMinus_m[2],
     CAVE_MachE_sil_test_B.CamberAngles_l[2], Bias,
     CAVE_MachE_sil_test_B.Selector29, CAVE_MachE_sil_test_B.Saturation_j,
     CAVE_MachE_sil_test_B.Integrator_fm, CAVE_MachE_sil_test_B.Integrator_e,
     &CAVE_MachE_sil_test_B.VectorConcatenate_jk[2],
     &CAVE_MachE_sil_test_B.VectorConcatenate_b[2],
     &CAVE_MachE_sil_test_B.VectorConcatenate_ol[2],
     &CAVE_MachE_sil_test_B.VectorConcatenate_bc[2],
     &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_ALPMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_ALPMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_BREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_CAMMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_CAMMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_DREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_FNOMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_FREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_FZMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_FZMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_KPUMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_KPUMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_LATERAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_LONGITUDINAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_LONGVL,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_NOMPRES,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCFY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDXP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDXP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDXP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PDYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PECP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PECP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PEY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PFZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PHYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PKYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPMX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PPZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PRESMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PRESMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PVX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PVX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_PVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QBZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QCRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QCRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QCZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDTP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QDZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QEZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QEZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QEZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QEZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QHZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QHZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX12,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX13,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX14,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSX9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_QSY8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FCX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FCY,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FCY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_FZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_RA1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_RA2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_RB1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_RB2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_RE0,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_V1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_Q_V2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RBY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_REX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_REX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_REY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_REY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RHY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RHY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RVY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_RVY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_SSZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_SSZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_SSZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_UNLOADED_RADIUS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VERTICAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_WIDTH);

  /* SignalConversion generated from: '<S333>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_md[2] =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Re;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S322>/Selector39' */
    memcpy(&CAVE_MachE_sil_test_B.Selector39[0],
           &CAVE_MachE_sil_test_B.VectorConcatenate_pm[81], 27U * sizeof(real_T));
  }

  /* MATLAB Function: '<S419>/Magic Tire Const Input' incorporates:
   *  Constant: '<S312>/Pressure'
   */
  Bias = 1.0 * CAVE_MachE_sil_test_P.Pressure_const;
  CAVE_MachE_sil_test_MagicTireConstInput
    (CAVE_MachE_sil_test_B.sf_LockUp_c.Omega,
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[3],
     CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[3],
     CAVE_MachE_sil_test_B.UnaryMinus_m[3],
     CAVE_MachE_sil_test_B.CamberAngles_l[3], Bias,
     CAVE_MachE_sil_test_B.Selector39, CAVE_MachE_sil_test_B.Saturation_nz,
     CAVE_MachE_sil_test_B.Integrator_b, CAVE_MachE_sil_test_B.Integrator_d,
     &CAVE_MachE_sil_test_B.VectorConcatenate_jk[3],
     &CAVE_MachE_sil_test_B.VectorConcatenate_b[3],
     &CAVE_MachE_sil_test_B.VectorConcatenate_ol[3],
     &CAVE_MachE_sil_test_B.VectorConcatenate_bc[3],
     &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_ALPMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_ALPMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_BREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_CAMMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_CAMMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_DREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_FNOMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_FREFF,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_FZMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_FZMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_KPUMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_KPUMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_LATERAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_LONGITUDINAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_LONGVL,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_NOMPRES,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCFY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDXP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDXP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDXP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PDYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PECP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PECP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PEY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PFZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHYP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHYP3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PHYP4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PKYP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPMX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PPZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PRESMAX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PRESMIN,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PVX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PVX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_PVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QBZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QCRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QCRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QCZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDRP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDRP2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDTP1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QDZ9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QEZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QEZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QEZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QEZ5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QHZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QHZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX10,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX11,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX12,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX13,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX14,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSX9,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY7,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_QSY8,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FCX,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FCY,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FCY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FZ1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_FZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_RA1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_RA2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_RB1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_RB2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_RE0,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_V1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_Q_V2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBX3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RBY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RCX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RCY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_REX1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_REX2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_REY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_REY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RHY1,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RHY2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RVY3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RVY4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RVY5,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_RVY6,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_SSZ2,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_SSZ3,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_SSZ4,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_UNLOADED_RADIUS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VERTICAL_STIFFNESS,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW,
     CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_WIDTH);

  /* SignalConversion generated from: '<S333>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_md[3] =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Re;

  /* Reshape: '<S139>/Reshape1' */
  CAVE_MachE_sil_test_B.Re[0] = CAVE_MachE_sil_test_B.VectorConcatenate_md[0];
  CAVE_MachE_sil_test_B.Re[1] = CAVE_MachE_sil_test_B.VectorConcatenate_md[1];
  CAVE_MachE_sil_test_B.Re[2] = CAVE_MachE_sil_test_B.VectorConcatenate_md[2];
  CAVE_MachE_sil_test_B.Re[3] = CAVE_MachE_sil_test_B.VectorConcatenate_md[3];

  /* Reshape: '<S140>/Reshape17' */
  CAVE_MachE_sil_test_B.Reshape17[0] = CAVE_MachE_sil_test_B.Re[0];
  CAVE_MachE_sil_test_B.Reshape17[1] = CAVE_MachE_sil_test_B.Re[1];

  /* Reshape: '<S139>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_cq[0] =
    CAVE_MachE_sil_test_B.VectorConcatenate_jk[0];
  CAVE_MachE_sil_test_B.Reshape3_cq[1] =
    CAVE_MachE_sil_test_B.VectorConcatenate_jk[1];
  CAVE_MachE_sil_test_B.Reshape3_cq[2] =
    CAVE_MachE_sil_test_B.VectorConcatenate_jk[2];
  CAVE_MachE_sil_test_B.Reshape3_cq[3] =
    CAVE_MachE_sil_test_B.VectorConcatenate_jk[3];

  /* SignalConversion generated from: '<S331>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_g3[0] =
    CAVE_MachE_sil_test_B.Integrator_k;

  /* SignalConversion generated from: '<S331>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_g3[1] =
    CAVE_MachE_sil_test_B.Integrator_f;

  /* SignalConversion generated from: '<S331>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_g3[2] =
    CAVE_MachE_sil_test_B.Integrator_ah;

  /* SignalConversion generated from: '<S331>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_g3[3] =
    CAVE_MachE_sil_test_B.Integrator_p;

  /* Reshape: '<S139>/Reshape4' */
  CAVE_MachE_sil_test_B.Reshape4_ee[0] =
    CAVE_MachE_sil_test_B.VectorConcatenate_g3[0];
  CAVE_MachE_sil_test_B.Reshape4_ee[1] =
    CAVE_MachE_sil_test_B.VectorConcatenate_g3[1];
  CAVE_MachE_sil_test_B.Reshape4_ee[2] =
    CAVE_MachE_sil_test_B.VectorConcatenate_g3[2];
  CAVE_MachE_sil_test_B.Reshape4_ee[3] =
    CAVE_MachE_sil_test_B.VectorConcatenate_g3[3];

  /* Reshape: '<S139>/Reshape5' */
  CAVE_MachE_sil_test_B.Reshape5_j[0] =
    CAVE_MachE_sil_test_B.VectorConcatenate_b[0];
  CAVE_MachE_sil_test_B.Reshape5_j[1] =
    CAVE_MachE_sil_test_B.VectorConcatenate_b[1];
  CAVE_MachE_sil_test_B.Reshape5_j[2] =
    CAVE_MachE_sil_test_B.VectorConcatenate_b[2];
  CAVE_MachE_sil_test_B.Reshape5_j[3] =
    CAVE_MachE_sil_test_B.VectorConcatenate_b[3];

  /* Concatenate: '<S139>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.M[0] = CAVE_MachE_sil_test_B.Reshape3_cq[0];
  CAVE_MachE_sil_test_B.M[1] = CAVE_MachE_sil_test_B.Reshape4_ee[0];
  CAVE_MachE_sil_test_B.M[2] = CAVE_MachE_sil_test_B.Reshape5_j[0];
  CAVE_MachE_sil_test_B.M[3] = CAVE_MachE_sil_test_B.Reshape3_cq[1];
  CAVE_MachE_sil_test_B.M[4] = CAVE_MachE_sil_test_B.Reshape4_ee[1];
  CAVE_MachE_sil_test_B.M[5] = CAVE_MachE_sil_test_B.Reshape5_j[1];
  CAVE_MachE_sil_test_B.M[6] = CAVE_MachE_sil_test_B.Reshape3_cq[2];
  CAVE_MachE_sil_test_B.M[7] = CAVE_MachE_sil_test_B.Reshape4_ee[2];
  CAVE_MachE_sil_test_B.M[8] = CAVE_MachE_sil_test_B.Reshape5_j[2];
  CAVE_MachE_sil_test_B.M[9] = CAVE_MachE_sil_test_B.Reshape3_cq[3];
  CAVE_MachE_sil_test_B.M[10] = CAVE_MachE_sil_test_B.Reshape4_ee[3];
  CAVE_MachE_sil_test_B.M[11] = CAVE_MachE_sil_test_B.Reshape5_j[3];

  /* Selector: '<S134>/Selector9' */
  for (ibmat = 0; ibmat < 2; ibmat++) {
    CAVE_MachE_sil_test_B.Selector9_l[3 * ibmat] = CAVE_MachE_sil_test_B.M[3 *
      ibmat];
    CAVE_MachE_sil_test_B.Selector9_l[3 * ibmat + 1] = CAVE_MachE_sil_test_B.M[3
      * ibmat + 1];
    CAVE_MachE_sil_test_B.Selector9_l[3 * ibmat + 2] = CAVE_MachE_sil_test_B.M[3
      * ibmat + 2];
  }

  /* End of Selector: '<S134>/Selector9' */

  /* Reshape: '<S140>/Reshape18' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape18[y] = CAVE_MachE_sil_test_B.Selector9_l[y];
  }

  /* End of Reshape: '<S140>/Reshape18' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S140>/Reshape19' */
    CAVE_MachE_sil_test_B.Reshape19[0] = 0.0;
    CAVE_MachE_sil_test_B.Reshape19[1] = 0.0;
  }

  /* Outputs for Iterator SubSystem: '<S140>/For each track and axle combination calculate suspension forces and moments' incorporates:
   *  ForEach: '<S181>/For Each'
   */
  for (ForEach_itr_k = 0; ForEach_itr_k < 2; ForEach_itr_k++) {
    /* ForEachSliceSelector generated from: '<S181>/Axle Number' incorporates:
     *  Constant: '<S140>/Axle Number'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 =
      CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_AxleNumVec[ForEach_itr_k];

    /* ForEachSliceSelector generated from: '<S181>/Track Number' incorporates:
     *  Constant: '<S140>/Track Number'
     */
    rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0 =
      CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_TrackNumVec[ForEach_itr_k];
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* SignalConversion generated from: '<S189>/Selector3' incorporates:
       *  Constant: '<S189>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[0] =
        CAVE_MachE_sil_test_P.CoreSubsys_d.Constant_Value;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[1] =
        CAVE_MachE_sil_test_B.Reshape1_a[0];
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[2] =
        CAVE_MachE_sil_test_B.Reshape1_a[1];

      /* Selector: '<S188>/Selector1' incorporates:
       *  Constant: '<S188>/Vehicle Vehicle Track Offset3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1 =
        CAVE_MachE_sil_test_P.CoreSubsys_d.VehicleVehicleTrackOffset3_Value
        [(int32_T)rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 - 1];

      /* Sum: '<S188>/Sum2' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1 +
        rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0;

      /* Selector: '<S189>/Selector5' incorporates:
       *  Constant: '<S189>/Vehicle Vehicle Track Offset1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector5 =
        CAVE_MachE_sil_test_P.CoreSubsys_d.VehicleVehicleTrackOffset1_Value;

      /* Product: '<S189>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector5;

      /* Selector: '<S189>/Selector3' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3 =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[(int32_T)
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product];

      /* Abs: '<S193>/Abs2' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs2 = fabs
        (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3);

      /* RelationalOperator: '<S196>/Relational Operator' incorporates:
       *  Constant: '<S196>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCamberSteeringSlope_AxleNums);

      /* DataTypeConversion: '<S196>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator;

      /* Product: '<S196>/Product' incorporates:
       *  Constant: '<S193>/Constant3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_n =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_CamberStrgSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion;

      /* Sum: '<S196>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_n;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements = Bias;

      /* Product: '<S193>/Product2' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2 =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs2 *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements;

      /* RelationalOperator: '<S195>/Relational Operator' incorporates:
       *  Constant: '<S195>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCamberSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S195>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l;

      /* Product: '<S195>/Product' incorporates:
       *  Constant: '<S193>/Constant4'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_h =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Camber *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p;

      /* Sum: '<S195>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_h;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_d = Bias;

      /* RelationalOperator: '<S194>/Relational Operator' incorporates:
       *  Constant: '<S194>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_o =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCamberHeightSlope_AxleNums);

      /* DataTypeConversion: '<S194>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p5 =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_o;

      /* Product: '<S194>/Product' incorporates:
       *  Constant: '<S193>/Constant5'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_d =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_CamberHslp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p5;

      /* Sum: '<S194>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_d;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_i = Bias;

      /* RelationalOperator: '<S214>/Relational Operator' incorporates:
       *  Constant: '<S214>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_a =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SteeringHeightSlopeBySteeredAxle_AxleNums);

      /* DataTypeConversion: '<S214>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_a =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_a;

      /* Product: '<S214>/Product' incorporates:
       *  Constant: '<S213>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_m =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_StrgHgtSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_a;

      /* Sum: '<S214>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_m;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_k = Bias;

      /* Abs: '<S203>/Abs' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs = fabs
        (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3);

      /* Product: '<S203>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_hq =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_k *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs;

      /* RelationalOperator: '<S209>/Relational Operator' incorporates:
       *  Constant: '<S209>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_c =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S209>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_c =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_c;

      /* Product: '<S209>/Product' incorporates:
       *  Constant: '<S204>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_k =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Kz *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_c;

      /* Sum: '<S209>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_k;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g = Bias;

      /* RelationalOperator: '<S208>/Relational Operator' incorporates:
       *  Constant: '<S208>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_g =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S208>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_o =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_g;

      /* Product: '<S208>/Product' incorporates:
       *  Constant: '<S204>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_c =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_F0z *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_o;

      /* Sum: '<S208>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_c;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o = Bias;

      /* Product: '<S204>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4 = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o;
    }

    /* Selector: '<S188>/Selector2' */
    ibmat = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2;

    /* Selector: '<S188>/Selector' */
    y = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2;

    /* Selector: '<S188>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate2[(ibmat - 1) << 1];

    /* Selector: '<S188>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3[(y - 1) << 1];

    /* Selector: '<S188>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate2[((ibmat - 1) << 1) + 1];

    /* Selector: '<S188>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3[((y - 1) << 1) + 1];

    /* Sum: '<S203>/Add' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add =
      (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_hq -
       CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[0]) +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[0];

    /* Sum: '<S204>/Add4' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4;

    /* Gain: '<S204>/Height Sign Convention' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention =
      CAVE_MachE_sil_test_P.CoreSubsys_d.HeightSignConvention_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4;

    /* Product: '<S193>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_i *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention;

    /* Sum: '<S193>/Sum2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2_b =
      (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2 +
       CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_d) +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Abs: '<S193>/Abs1' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs1 = fabs
        (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3);

      /* RelationalOperator: '<S199>/Relational Operator' incorporates:
       *  Constant: '<S199>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_ag =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCasterSteeringSlope_AxleNums);

      /* DataTypeConversion: '<S199>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_m =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_ag;

      /* Product: '<S199>/Product' incorporates:
       *  Constant: '<S193>/Constant6'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_b =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_CasterStrgSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_m;

      /* Sum: '<S199>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_b;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o0 = Bias;

      /* Product: '<S193>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4_j =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs1 *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o0;

      /* RelationalOperator: '<S198>/Relational Operator' incorporates:
       *  Constant: '<S198>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l5 =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCasterSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S198>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_ck =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l5;

      /* Product: '<S198>/Product' incorporates:
       *  Constant: '<S193>/Constant7'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Caster *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_ck;

      /* Sum: '<S198>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_j = Bias;

      /* RelationalOperator: '<S197>/Relational Operator' incorporates:
       *  Constant: '<S197>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_e =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCasterHeightSlope_AxleNums);

      /* DataTypeConversion: '<S197>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_b =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_e;

      /* Product: '<S197>/Product' incorporates:
       *  Constant: '<S193>/Constant8'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_g =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_CasterHslp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_b;

      /* Sum: '<S197>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_g;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_c = Bias;

      /* Abs: '<S193>/Abs' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs_d = fabs
        (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3);

      /* RelationalOperator: '<S202>/Relational Operator' incorporates:
       *  Constant: '<S202>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_n =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectToeSteeringSlope_AxleNums);

      /* DataTypeConversion: '<S202>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_d =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_n;

      /* Product: '<S202>/Product' incorporates:
       *  Constant: '<S193>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_i =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_ToeStrgSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_d;

      /* Sum: '<S202>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_i;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_oi = Bias;

      /* Product: '<S193>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_lx =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs_d *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_oi;

      /* RelationalOperator: '<S201>/Relational Operator' incorporates:
       *  Constant: '<S201>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_p =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectToeSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S201>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_f =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_p;

      /* Product: '<S201>/Product' incorporates:
       *  Constant: '<S193>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_e =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Toe *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_f;

      /* Sum: '<S201>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_e;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_f = Bias;

      /* RelationalOperator: '<S200>/Relational Operator' incorporates:
       *  Constant: '<S200>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_m =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.SelectRollSteerSlope_AxleNums);

      /* DataTypeConversion: '<S200>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_pi =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_m;

      /* Product: '<S200>/Product' incorporates:
       *  Constant: '<S193>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_o =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_RollStrgSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_pi;

      /* Sum: '<S200>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_o;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_h = Bias;
    }

    /* Product: '<S193>/Product5' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_c *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention;

    /* Sum: '<S193>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1 =
      (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4_j +
       CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_j) +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5;

    /* Product: '<S193>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_h *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention;

    /* Sum: '<S193>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum =
      (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_lx +
       CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_f) +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1;

    /* Product: '<S204>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3_n =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4 *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S207>/Relational Operator' incorporates:
       *  Constant: '<S207>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_mm =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S207>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p4 =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_mm;

      /* Product: '<S207>/Product' incorporates:
       *  Constant: '<S204>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l4 =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Cz *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p4;

      /* Sum: '<S207>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l4;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_p = Bias;
    }

    /* Sum: '<S203>/Add2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[1] -
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[1];

    /* Product: '<S204>/Product5' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5_c =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_p *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2;

    /* Sum: '<S204>/Add1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add1 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3_n +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5_c;

    /* Product: '<S204>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_o =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add1 *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2;

    /* Signum: '<S204>/Sign1' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4;
    if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 < 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 = -1.0;
    } else if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 > 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 = 1.0;
    } else if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 == 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 = 0.0;
    } else {
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 = (rtNaN);
    }

    /* End of Signum: '<S204>/Sign1' */

    /* Product: '<S204>/Product2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2_d =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4 *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3_n;

    /* SignalConversion generated from: '<S181>/Info' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2_b;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[3] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[4] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_o;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[5] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2_d;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S210>/Relational Operator' incorporates:
       *  Constant: '<S210>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_nm =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_d.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S210>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_g =
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_nm;

      /* Product: '<S210>/Product' incorporates:
       *  Constant: '<S204>/Constant3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_gk =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Hmax *
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_g;

      /* Sum: '<S210>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_gk;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_dv = Bias;
    }

    /* Outputs for Atomic SubSystem: '<S206>/Max stop reached' */
    CAVE_MachE_sil_test_Maxstopreached(CAVE_MachE_sil_test_M,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_p,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_dv,
      &CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Maxstopreached,
      &CAVE_MachE_sil_test_P.CoreSubsys_d.Maxstopreached,
      CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Hmax);

    /* End of Outputs for SubSystem: '<S206>/Max stop reached' */

    /* Outputs for Atomic SubSystem: '<S206>/Min stop reached' */
    CAVE_MachE_sil_test_Minstopreached(CAVE_MachE_sil_test_M,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_p,
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_dv,
      &CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Minstopreached,
      &CAVE_MachE_sil_test_P.CoreSubsys_d.Minstopreached,
      CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_Hmax);

    /* End of Outputs for SubSystem: '<S206>/Min stop reached' */

    /* Sum: '<S206>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_l =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Maxstopreached.Product4
      + CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      Minstopreached.Product4;

    /* Sum: '<S204>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_p =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add1 +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_l;

    /* Gain: '<S203>/Vehicle Force Sign' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleForceSign =
      CAVE_MachE_sil_test_P.CoreSubsys_d.VehicleForceSign_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_p;

    /* Selector: '<S191>/Selector1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1_c =
      CAVE_MachE_sil_test_B.Reshape4_e[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 - 1];

    /* Selector: '<S191>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector_h =
      CAVE_MachE_sil_test_B.Reshape17[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 - 1];

    /* Gain: '<S203>/Sign convention' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleHeight =
      CAVE_MachE_sil_test_P.CoreSubsys_d.Signconvention_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add;

    /* Sum: '<S191>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_f =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector_h +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleHeight;

    /* Product: '<S191>/Product' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_j =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1_c *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_f;

    /* UnaryMinus: '<S191>/Unary Minus' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].UnaryMinus =
      -CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_j;

    /* Selector: '<S191>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2_g =
      CAVE_MachE_sil_test_B.Reshape3_i[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 - 1];

    /* Product: '<S191>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_l =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_f *
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2_g;

    /* Reshape: '<S191>/Reshape' incorporates:
     *  Constant: '<S191>/Constant'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].UnaryMinus;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_l;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[2] =
      CAVE_MachE_sil_test_P.CoreSubsys_d.Constant_Value_j;

    /* Selector: '<S191>/Selector3' */
    ibmat = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[0] =
      CAVE_MachE_sil_test_B.Reshape18[(ibmat - 1) * 3];

    /* Sum: '<S191>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[0];

    /* Selector: '<S191>/Selector3' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[1] =
      CAVE_MachE_sil_test_B.Reshape18[(ibmat - 1) * 3 + 1];

    /* Sum: '<S191>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[1] +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[1];

    /* Selector: '<S191>/Selector3' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[2] =
      CAVE_MachE_sil_test_B.Reshape18[(ibmat - 1) * 3 + 2];

    /* Sum: '<S191>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[2] +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[2];

    /* Sum: '<S193>/Sum3' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum3 =
      (CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3 -
       CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_f) +
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum;

    /* SignalConversion generated from: '<S181>/WhlAng' */
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2_b;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1;
    CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum3;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Selector: '<S188>/Selector3' */
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_i =
        CAVE_MachE_sil_test_B.Reshape19[(int32_T)
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 - 1];
    }

    /* ForEachSliceAssignment generated from: '<S181>/WhlFz' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0[ForEach_itr_k] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_p;

    /* ForEachSliceAssignment generated from: '<S181>/WhlAng' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0[ForEach_itr_k *
      3] = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0[ForEach_itr_k *
      3 + 1] = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0[ForEach_itr_k *
      3 + 2] = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2];

    /* ForEachSliceAssignment generated from: '<S181>/VehM' */
    ibmat = ForEach_itr_k * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[2];

    /* ForEachSliceAssignment generated from: '<S181>/VehFz' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0[ForEach_itr_k] =
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleForceSign;

    /* ForEachSliceAssignment generated from: '<S181>/Info' */
    for (y = 0; y < 6; y++) {
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Info_at_inport_0[y +
        ForEach_itr_k * 6] = CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[y];
    }

    /* End of ForEachSliceAssignment generated from: '<S181>/Info' */
  }

  /* End of Outputs for SubSystem: '<S140>/For each track and axle combination calculate suspension forces and moments' */

  /* Outputs for Iterator SubSystem: '<S182>/For Each Axle With Anti-Sway' incorporates:
   *  ForEach: '<S183>/For Each'
   */
  for (ForEach_itr_h = 0; ForEach_itr_h < 1; ForEach_itr_h++) {
    /* ForEachSliceSelector generated from: '<S183>/Axle Number' incorporates:
     *  Constant: '<S182>/Axles Using Anti-Sway'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i =
      CAVE_MachE_sil_test_P.AxlesUsingAntiSway_Value;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S186>/Relational Operator' incorporates:
       *  Constant: '<S186>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i ==
         CAVE_MachE_sil_test_P.CoreSubsys_n.AntiSwayArmRadiusByAxle_AxleNums);

      /* DataTypeConversion: '<S186>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator;

      /* Product: '<S186>/Product' incorporates:
       *  Constant: '<S184>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_AntiSwayR *
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion;

      /* Sum: '<S186>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements = Bias;

      /* RelationalOperator: '<S185>/Relational Operator' incorporates:
       *  Constant: '<S185>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_p =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i ==
         CAVE_MachE_sil_test_P.CoreSubsys_n.AntiSwayArmNeutralAngleByAxle_AxleNums);

      /* DataTypeConversion: '<S185>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_b =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_p;

      /* Product: '<S185>/Product' incorporates:
       *  Constant: '<S184>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_o =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_AntiSwayNtrlAng *
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_b;

      /* Sum: '<S185>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_o;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_a = Bias;

      /* Trigonometry: '<S184>/Trigonometric Function' */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction =
        tan(CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_a);

      /* Product: '<S184>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Z0 =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements *
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction;

      /* Selector: '<S183>/Selector1' incorporates:
       *  Constant: '<S183>/Vehicle Vehicle Track Offset3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector1 =
        CAVE_MachE_sil_test_P.CoreSubsys_n.VehicleVehicleTrackOffset3_Value
        [(int32_T)rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i - 1];

      /* Sum: '<S183>/Sum2' incorporates:
       *  Constant: '<S183>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector1 +
        CAVE_MachE_sil_test_P.CoreSubsys_n.Constant_Value[0];
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector1 +
        CAVE_MachE_sil_test_P.CoreSubsys_n.Constant_Value[1];
    }

    /* Selector: '<S183>/Selector5' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate2[0];
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate2[2];

    /* Selector: '<S183>/Selector3' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] - 1];

    /* Selector: '<S183>/Selector6' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3[0];

    /* Selector: '<S183>/Selector3' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] - 1];

    /* Selector: '<S183>/Selector6' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3[2];

    /* Selector: '<S183>/Selector4' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] - 1];
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] - 1];

    /* Sum: '<S184>/Sum3' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum3 =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[0];

    /* Sum: '<S184>/Sum6' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum6 =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[1] -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[1];

    /* Sum: '<S184>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Z0 -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum3;
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Z0 -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum6;

    /* Product: '<S184>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[0] = 1.0 /
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[0];

    /* Lookup_n-D: '<S184>/Angle Tangent Limit' */
    Bias = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[0];
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[0] =
      look1_binlcpw(Bias,
                    CAVE_MachE_sil_test_P.CoreSubsys_n.AngleTangentLimit_bp01Data,
                    CAVE_MachE_sil_test_P.CoreSubsys_n.AngleTangentLimit_tableData,
                    1U);

    /* Product: '<S184>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[1] = 1.0 /
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[1];

    /* Lookup_n-D: '<S184>/Angle Tangent Limit' */
    Bias = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[1];
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[1] =
      look1_binlcpw(Bias,
                    CAVE_MachE_sil_test_P.CoreSubsys_n.AngleTangentLimit_bp01Data,
                    CAVE_MachE_sil_test_P.CoreSubsys_n.AngleTangentLimit_tableData,
                    1U);
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S187>/Relational Operator' incorporates:
       *  Constant: '<S187>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_o =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_i ==
         CAVE_MachE_sil_test_P.CoreSubsys_n.AntiSwayBarTorsionSpringConstantByAxle_AxleNums);

      /* DataTypeConversion: '<S187>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_bj =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_o;

      /* Product: '<S187>/Product' incorporates:
       *  Constant: '<S184>/Constant3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_p =
        CAVE_MachE_sil_test_P.independentSuspensionsMacPherson_AntiSwayTrsK *
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_bj;

      /* Sum: '<S187>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_p;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_e = Bias;
    }

    /* Trigonometry: '<S184>/Trigonometric Function1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[0] =
      atan(CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[0]);

    /* Sum: '<S184>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_a -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[0];

    /* Trigonometry: '<S184>/Trigonometric Function2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[0] =
      cos(CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[0]);

    /* Trigonometry: '<S184>/Trigonometric Function1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[1] =
      atan(CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[1]);

    /* Sum: '<S184>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_a -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[1];

    /* Trigonometry: '<S184>/Trigonometric Function2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[1] =
      cos(CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[1]);

    /* Sum: '<S184>/Sum2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].deltaTheta =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[1];

    /* Product: '<S184>/Product4' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].antiswaybartorque =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].deltaTheta *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_e;

    /* Product: '<S184>/Product2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product2 = 1.0 /
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].antiswaybartorque;

    /* Product: '<S184>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product2 *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[0];

    /* Selector: '<S183>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[0] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] - 1];

    /* Sum: '<S184>/Sum4' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[0] -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[0];

    /* Selector: '<S183>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[0] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] - 1];

    /* Sum: '<S184>/Sum5' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[0];

    /* Product: '<S184>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product2 *
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[1];

    /* Selector: '<S183>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[1] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] - 1];

    /* Sum: '<S184>/Sum4' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[1] -
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[1];

    /* Selector: '<S183>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[1] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] - 1];

    /* Sum: '<S184>/Sum5' */
    CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[1] +
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[1];

    /* ForEachSliceAssignment generated from: '<S183>/WhlFzAs' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFzAs_at_inport_0[ForEach_itr_h <<
      1] = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFzAs_at_inport_0[(ForEach_itr_h <<
      1) + 1] = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[1];

    /* ForEachSliceAssignment generated from: '<S183>/VehFzAs' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFzAs_at_inport_0[ForEach_itr_h <<
      1] = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFzAs_at_inport_0[(ForEach_itr_h <<
      1) + 1] = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[1];
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* ForEachSliceAssignment generated from: '<S183>/AntiSwayFzInd' */
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[ForEach_itr_h
        << 1] = CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0];
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0
        [(ForEach_itr_h << 1) + 1] =
        CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1];
    }
  }

  /* End of Outputs for SubSystem: '<S182>/For Each Axle With Anti-Sway' */

  /* Assignment: '<S182>/Assign WhlFz' */
  CAVE_MachE_sil_test_B.AssignWhlFz[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0[0];
  CAVE_MachE_sil_test_B.AssignWhlFz[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0[1];
  CAVE_MachE_sil_test_B.AssignWhlFz[(int32_T)
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[0] - 1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFzAs_at_inport_0[0];
  CAVE_MachE_sil_test_B.AssignWhlFz[(int32_T)
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[1] - 1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFzAs_at_inport_0[1];

  /* Reshape: '<S140>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_m[0] = CAVE_MachE_sil_test_B.AssignWhlFz[0];

  /* Concatenate: '<S140>/Matrix Concatenate1' */
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[0] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[1] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[2] =
    CAVE_MachE_sil_test_B.Reshape2_m[0];

  /* Reshape: '<S140>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_m[1] = CAVE_MachE_sil_test_B.AssignWhlFz[1];

  /* Concatenate: '<S140>/Matrix Concatenate1' */
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[3] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[4] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate1_p[5] =
    CAVE_MachE_sil_test_B.Reshape2_m[1];

  /* Reshape: '<S135>/Reshape10' */
  CAVE_MachE_sil_test_B.Reshape10[0] = CAVE_MachE_sil_test_B.Reshape6[2];

  /* Reshape: '<S135>/Reshape11' */
  CAVE_MachE_sil_test_B.Reshape11[0] = CAVE_MachE_sil_test_B.Reshape7[2];

  /* Concatenate: '<S135>/Matrix Concatenate2' */
  CAVE_MachE_sil_test_B.MatrixConcatenate2_h[0] =
    CAVE_MachE_sil_test_B.Reshape10[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate2_h[1] =
    CAVE_MachE_sil_test_B.Reshape11[0];

  /* Reshape: '<S135>/Reshape10' */
  CAVE_MachE_sil_test_B.Reshape10[1] = CAVE_MachE_sil_test_B.Reshape6[3];

  /* Reshape: '<S135>/Reshape11' */
  CAVE_MachE_sil_test_B.Reshape11[1] = CAVE_MachE_sil_test_B.Reshape7[3];

  /* Concatenate: '<S135>/Matrix Concatenate2' */
  CAVE_MachE_sil_test_B.MatrixConcatenate2_h[2] =
    CAVE_MachE_sil_test_B.Reshape10[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate2_h[3] =
    CAVE_MachE_sil_test_B.Reshape11[1];
  for (y = 0; y < 6; y++) {
    /* SignalConversion generated from: '<S134>/Matrix Concatenate4' */
    CAVE_MachE_sil_test_B.F[y] = CAVE_MachE_sil_test_B.MatrixConcatenate1_p[y];

    /* Reshape: '<S135>/Reshape7' */
    CAVE_MachE_sil_test_B.Reshape7_j[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspP_at_inport_0[y];

    /* Reshape: '<S135>/Reshape6' */
    CAVE_MachE_sil_test_B.Reshape6_a[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_SuspV_at_inport_0[y];
  }

  /* Selector: '<S135>/Selector' */
  CAVE_MachE_sil_test_B.Selector_i[0] = CAVE_MachE_sil_test_B.Reshape7_j[2];

  /* Sum: '<S145>/Sum' */
  CAVE_MachE_sil_test_B.Sum_n[0] = CAVE_MachE_sil_test_B.Reshape3_c[0] -
    CAVE_MachE_sil_test_B.Selector_i[0];

  /* Gain: '<S145>/Carrier to Axle Compliance' */
  CAVE_MachE_sil_test_B.CarriertoAxleCompliance[0] =
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_KzWhlAxl *
    CAVE_MachE_sil_test_B.Sum_n[0];

  /* Selector: '<S135>/Selector' */
  CAVE_MachE_sil_test_B.Selector_i[1] = CAVE_MachE_sil_test_B.Reshape7_j[5];

  /* Sum: '<S145>/Sum' */
  CAVE_MachE_sil_test_B.Sum_n[1] = CAVE_MachE_sil_test_B.Reshape3_c[1] -
    CAVE_MachE_sil_test_B.Selector_i[1];

  /* Gain: '<S145>/Carrier to Axle Compliance' */
  CAVE_MachE_sil_test_B.CarriertoAxleCompliance[1] =
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_KzWhlAxl *
    CAVE_MachE_sil_test_B.Sum_n[1];

  /* Selector: '<S135>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_fq[0] = CAVE_MachE_sil_test_B.Reshape6_a[2];

  /* Reshape: '<S135>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_ms[0] = CAVE_MachE_sil_test_B.zdot_o[2];

  /* Sum: '<S145>/Sum2' */
  CAVE_MachE_sil_test_B.Sum2_h[0] = CAVE_MachE_sil_test_B.Reshape2_ms[0] -
    CAVE_MachE_sil_test_B.Selector1_fq[0];

  /* Gain: '<S145>/Carrier to Axle Damping' */
  CAVE_MachE_sil_test_B.CarriertoAxleDamping[0] =
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_CzWhlAxl *
    CAVE_MachE_sil_test_B.Sum2_h[0];

  /* Selector: '<S135>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_fq[1] = CAVE_MachE_sil_test_B.Reshape6_a[5];

  /* Reshape: '<S135>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_ms[1] = CAVE_MachE_sil_test_B.zdot_o[3];

  /* Sum: '<S145>/Sum2' */
  CAVE_MachE_sil_test_B.Sum2_h[1] = CAVE_MachE_sil_test_B.Reshape2_ms[1] -
    CAVE_MachE_sil_test_B.Selector1_fq[1];

  /* Gain: '<S145>/Carrier to Axle Damping' */
  CAVE_MachE_sil_test_B.CarriertoAxleDamping[1] =
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_CzWhlAxl *
    CAVE_MachE_sil_test_B.Sum2_h[1];

  /* Sum: '<S145>/Sum1' incorporates:
   *  Constant: '<S145>/Preload'
   */
  CAVE_MachE_sil_test_B.Sum1[0] =
    (CAVE_MachE_sil_test_B.CarriertoAxleCompliance[0] +
     CAVE_MachE_sil_test_B.CarriertoAxleDamping[0]) -
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_F0zWhlAxl;
  CAVE_MachE_sil_test_B.Sum1[1] =
    (CAVE_MachE_sil_test_B.CarriertoAxleCompliance[1] +
     CAVE_MachE_sil_test_B.CarriertoAxleDamping[1]) -
    CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_F0zWhlAxl;

  /* Gain: '<S135>/Gain' */
  CAVE_MachE_sil_test_B.Gain_g[0] = CAVE_MachE_sil_test_P.Gain_Gain_l *
    CAVE_MachE_sil_test_B.Sum1[0];
  CAVE_MachE_sil_test_B.Gain_g[1] = CAVE_MachE_sil_test_P.Gain_Gain_l *
    CAVE_MachE_sil_test_B.Sum1[1];

  /* Concatenate: '<S135>/Matrix Concatenate6' */
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[0] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[1] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[2] = CAVE_MachE_sil_test_B.Gain_g[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[3] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[4] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate6_c[5] = CAVE_MachE_sil_test_B.Gain_g[1];
  for (y = 0; y < 6; y++) {
    /* SignalConversion generated from: '<S134>/Matrix Concatenate4' */
    CAVE_MachE_sil_test_B.F[y + 6] =
      CAVE_MachE_sil_test_B.MatrixConcatenate6_c[y];

    /* Reshape: '<S140>/Reshape14' */
    CAVE_MachE_sil_test_B.Reshape14[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0[y];

    /* SignalConversion generated from: '<S134>/Matrix Concatenate5' */
    CAVE_MachE_sil_test_B.M_i[y] = CAVE_MachE_sil_test_B.Reshape14[y];
  }

  for (y = 0; y < 2; y++) {
    /* Concatenate: '<S135>/Matrix Concatenate1' */
    CAVE_MachE_sil_test_B.MatrixConcatenate1_f[y << 1] =
      CAVE_MachE_sil_test_B.Selector_i[y];
    CAVE_MachE_sil_test_B.MatrixConcatenate1_f[(y << 1) + 1] =
      CAVE_MachE_sil_test_B.Selector1_fq[y];

    /* Selector: '<S134>/Selector1' */
    CAVE_MachE_sil_test_B.Selector1_fc[3 * y] = CAVE_MachE_sil_test_B.P[(y + 2) *
      3];
    CAVE_MachE_sil_test_B.Selector1_fc[3 * y + 1] = CAVE_MachE_sil_test_B.P[(y +
      2) * 3 + 1];
    CAVE_MachE_sil_test_B.Selector1_fc[3 * y + 2] = CAVE_MachE_sil_test_B.P[(y +
      2) * 3 + 2];

    /* Selector: '<S135>/Selector3' */
    CAVE_MachE_sil_test_B.Selector3[y] = CAVE_MachE_sil_test_B.Selector1_fc[3 *
      y + 2];

    /* Reshape: '<S135>/Reshape12' */
    CAVE_MachE_sil_test_B.Reshape12[y] = CAVE_MachE_sil_test_B.Selector3[y];

    /* Selector: '<S135>/Selector4' */
    CAVE_MachE_sil_test_B.Selector4_n[y] = CAVE_MachE_sil_test_B.Selector18[3 *
      y + 2];

    /* Reshape: '<S135>/Reshape13' */
    CAVE_MachE_sil_test_B.Reshape13_c[y] = CAVE_MachE_sil_test_B.Selector4_n[y];

    /* Concatenate: '<S135>/Matrix Concatenate3' */
    CAVE_MachE_sil_test_B.MatrixConcatenate3_c[y << 1] =
      CAVE_MachE_sil_test_B.Reshape12[y];
    CAVE_MachE_sil_test_B.MatrixConcatenate3_c[(y << 1) + 1] =
      CAVE_MachE_sil_test_B.Reshape13_c[y];

    /* Reshape: '<S135>/Reshape4' */
    CAVE_MachE_sil_test_B.Reshape4_j[y] = CAVE_MachE_sil_test_B.Re[y + 2];
  }

  /* Selector: '<S134>/Selector17' */
  for (ibmat = 0; ibmat < 2; ibmat++) {
    CAVE_MachE_sil_test_B.Selector17[3 * ibmat] = CAVE_MachE_sil_test_B.M[(ibmat
      + 2) * 3];
    CAVE_MachE_sil_test_B.Selector17[3 * ibmat + 1] = CAVE_MachE_sil_test_B.M
      [(ibmat + 2) * 3 + 1];
    CAVE_MachE_sil_test_B.Selector17[3 * ibmat + 2] = CAVE_MachE_sil_test_B.M
      [(ibmat + 2) * 3 + 2];
  }

  /* End of Selector: '<S134>/Selector17' */

  /* Outputs for Iterator SubSystem: '<S135>/For each track and axle combination calculate suspension forces and moments' incorporates:
   *  ForEach: '<S144>/For Each'
   */
  for (ForEach_itr_hh = 0; ForEach_itr_hh < 2; ForEach_itr_hh++) {
    /* ForEachSliceSelector generated from: '<S144>/Axle Number' incorporates:
     *  Constant: '<S135>/Axle Number'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l =
      CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[ForEach_itr_hh];

    /* ForEachSliceSelector generated from: '<S144>/Track Number' incorporates:
     *  Constant: '<S135>/Track Number'
     */
    rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0_a =
      CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_TrackNumVec[ForEach_itr_hh];
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S164>/Relational Operator' incorporates:
       *  Constant: '<S164>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectCamberSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S164>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator;

      /* Product: '<S164>/Product' incorporates:
       *  Constant: '<S162>/Constant4'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Camber *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion;

      /* Sum: '<S164>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements = Bias;

      /* RelationalOperator: '<S163>/Relational Operator' incorporates:
       *  Constant: '<S163>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_p =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectCamberHeightSlope_AxleNums);

      /* DataTypeConversion: '<S163>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_m =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_p;

      /* Product: '<S163>/Product' incorporates:
       *  Constant: '<S162>/Constant5'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_d =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_CamberHslp *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_m;

      /* Sum: '<S163>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_d;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_f = Bias;

      /* SignalConversion generated from: '<S158>/Selector3' incorporates:
       *  Constant: '<S158>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtSelector3Inport1[0] =
        CAVE_MachE_sil_test_P.CoreSubsys_p.Constant_Value_m;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtSelector3Inport1[1] = 0.0;

      /* Selector: '<S157>/Selector1' incorporates:
       *  Constant: '<S157>/Vehicle Vehicle Track Offset3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1 =
        CAVE_MachE_sil_test_P.CoreSubsys_p.VehicleVehicleTrackOffset3_Value
        [(int32_T)rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l - 1];

      /* Sum: '<S157>/Sum2' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1 +
        rtb_ImpSel_InsertedFor_TrackNumber_at_outport_0_a;

      /* Selector: '<S158>/Selector5' incorporates:
       *  Constant: '<S158>/Vehicle Vehicle Track Offset1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector5 =
        CAVE_MachE_sil_test_P.CoreSubsys_p.VehicleVehicleTrackOffset1_Value;

      /* Product: '<S158>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_g =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector5;

      /* Selector: '<S158>/Selector3' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3 =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtSelector3Inport1[(int32_T)
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_g];

      /* Abs: '<S169>/Abs' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Abs = fabs
        (CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3);

      /* Product: '<S169>/Product' incorporates:
       *  Constant: '<S179>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_dj =
        CAVE_MachE_sil_test_P.CoreSubsys_p.Constant_Value *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Abs;

      /* RelationalOperator: '<S175>/Relational Operator' incorporates:
       *  Constant: '<S175>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_j =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S175>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_c =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_j;

      /* Product: '<S175>/Product' incorporates:
       *  Constant: '<S170>/Constant'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_b =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Kz *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_c;

      /* Sum: '<S175>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_b;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g = Bias;

      /* RelationalOperator: '<S174>/Relational Operator' incorporates:
       *  Constant: '<S174>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_d =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S174>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_l =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_d;

      /* Product: '<S174>/Product' incorporates:
       *  Constant: '<S170>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_h =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_F0z *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_l;

      /* Sum: '<S174>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_h;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_m = Bias;

      /* Product: '<S170>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product4 = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_m;
    }

    /* Selector: '<S157>/Selector2' */
    ibmat = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2;

    /* Selector: '<S157>/Selector' */
    y = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2;

    /* Selector: '<S157>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate1_f[(ibmat - 1) << 1];

    /* Selector: '<S157>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[0] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3_c[(y - 1) << 1];

    /* Selector: '<S157>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate1_f[((ibmat - 1) << 1) + 1];

    /* Selector: '<S157>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[1] =
      CAVE_MachE_sil_test_B.MatrixConcatenate3_c[((y - 1) << 1) + 1];

    /* Sum: '<S169>/Add' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add =
      (CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_dj -
       CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[0]) +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[0];

    /* Sum: '<S170>/Add4' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product4;

    /* Gain: '<S170>/Height Sign Convention' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention =
      CAVE_MachE_sil_test_P.CoreSubsys_p.HeightSignConvention_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4;

    /* Product: '<S162>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_f *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention;

    /* Sum: '<S162>/Sum2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2_b =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S166>/Relational Operator' incorporates:
       *  Constant: '<S166>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_l =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectCasterSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S166>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_d =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_l;

      /* Product: '<S166>/Product' incorporates:
       *  Constant: '<S162>/Constant7'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Caster *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_d;

      /* Sum: '<S166>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_mz = Bias;

      /* RelationalOperator: '<S165>/Relational Operator' incorporates:
       *  Constant: '<S165>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_b =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectCasterHeightSlope_AxleNums);

      /* DataTypeConversion: '<S165>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_k =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_b;

      /* Product: '<S165>/Product' incorporates:
       *  Constant: '<S162>/Constant8'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_bk =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_CasterHslp *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_k;

      /* Sum: '<S165>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_bk;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_b = Bias;

      /* RelationalOperator: '<S168>/Relational Operator' incorporates:
       *  Constant: '<S168>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_c =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectToeSteeringCenter_AxleNums);

      /* DataTypeConversion: '<S168>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_a =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_c;

      /* Product: '<S168>/Product' incorporates:
       *  Constant: '<S162>/Constant1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_j =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Toe *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_a;

      /* Sum: '<S168>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_j;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_d = Bias;

      /* RelationalOperator: '<S167>/Relational Operator' incorporates:
       *  Constant: '<S167>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_a =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.SelectRollSteerSlope_AxleNums);

      /* DataTypeConversion: '<S167>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_p =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_a;

      /* Product: '<S167>/Product' incorporates:
       *  Constant: '<S162>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l2 =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_RollStrgSlp *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_p;

      /* Sum: '<S167>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l2;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_i = Bias;
    }

    /* Product: '<S162>/Product5' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_b *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention;

    /* Sum: '<S162>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_mz +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5;

    /* Product: '<S162>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_i *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention;

    /* Sum: '<S162>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_d +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1;

    /* Product: '<S170>/Product3' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3_d =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4 *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S173>/Relational Operator' incorporates:
       *  Constant: '<S173>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_g =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S173>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_f =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_g;

      /* Product: '<S173>/Product' incorporates:
       *  Constant: '<S170>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_c =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Cz *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_f;

      /* Sum: '<S173>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_c;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_e = Bias;
    }

    /* Sum: '<S169>/Add2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[1] -
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[1];

    /* Product: '<S170>/Product5' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5_d =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_e *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2;

    /* Sum: '<S170>/Add1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add1 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3_d +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5_d;

    /* Product: '<S170>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_a =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add1 *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2;

    /* Signum: '<S170>/Sign1' */
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4;
    if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 < 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 = -1.0;
    } else if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 > 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 = 1.0;
    } else if (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 == 0.0) {
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 = 0.0;
    } else {
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 = (rtNaN);
    }

    /* End of Signum: '<S170>/Sign1' */

    /* Product: '<S170>/Product2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product2 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4 *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3_d;

    /* SignalConversion generated from: '<S144>/Info' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2_b;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[3] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[4] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_a;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[5] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product2;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S176>/Relational Operator' incorporates:
       *  Constant: '<S176>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_e =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0_l ==
         CAVE_MachE_sil_test_P.CoreSubsys_p.Constrainedspringdampercombination_AxleNums);

      /* DataTypeConversion: '<S176>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_fh =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_e;

      /* Product: '<S176>/Product' incorporates:
       *  Constant: '<S170>/Constant3'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_i =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Hmax *
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_fh;

      /* Sum: '<S176>/Sum of Elements' */
      Bias = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_i;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_fu = Bias;
    }

    /* Outputs for Atomic SubSystem: '<S172>/Max stop reached' */
    CAVE_MachE_sil_test_Maxstopreached(CAVE_MachE_sil_test_M,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_e,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_fu,
      &CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Maxstopreached,
      &CAVE_MachE_sil_test_P.CoreSubsys_p.Maxstopreached,
      CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Hmax);

    /* End of Outputs for SubSystem: '<S172>/Max stop reached' */

    /* Outputs for Atomic SubSystem: '<S172>/Min stop reached' */
    CAVE_MachE_sil_test_Minstopreached(CAVE_MachE_sil_test_M,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_e,
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_fu,
      &CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Minstopreached,
      &CAVE_MachE_sil_test_P.CoreSubsys_p.Minstopreached,
      CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_Hmax);

    /* End of Outputs for SubSystem: '<S172>/Min stop reached' */

    /* Sum: '<S172>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_a =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Maxstopreached.Product4
      + CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      Minstopreached.Product4;

    /* Sum: '<S170>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_e =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add1 +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_a;

    /* Gain: '<S169>/Vehicle Force Sign' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleForceSign =
      CAVE_MachE_sil_test_P.CoreSubsys_p.VehicleForceSign_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_e;

    /* Selector: '<S160>/Selector1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1_g =
      CAVE_MachE_sil_test_B.Reshape11[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 - 1];

    /* Selector: '<S160>/Selector' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector_d =
      CAVE_MachE_sil_test_B.Reshape4_j[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 - 1];

    /* Gain: '<S169>/Sign convention' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleHeight =
      CAVE_MachE_sil_test_P.CoreSubsys_p.Signconvention_Gain *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add;

    /* Sum: '<S160>/Sum' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_h =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector_d +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleHeight;

    /* Product: '<S160>/Product' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l5 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1_g *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_h;

    /* UnaryMinus: '<S160>/Unary Minus' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].UnaryMinus =
      -CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l5;

    /* Selector: '<S160>/Selector2' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2_g =
      CAVE_MachE_sil_test_B.Reshape10[(int32_T)
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 - 1];

    /* Product: '<S160>/Product1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_p =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_h *
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2_g;

    /* Reshape: '<S160>/Reshape' incorporates:
     *  Constant: '<S160>/Constant'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].UnaryMinus;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_p;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[2] =
      CAVE_MachE_sil_test_P.CoreSubsys_p.Constant_Value_a;

    /* Selector: '<S160>/Selector3' */
    ibmat = (int32_T)CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[0] =
      CAVE_MachE_sil_test_B.Selector17[(ibmat - 1) * 3];
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[1] =
      CAVE_MachE_sil_test_B.Selector17[(ibmat - 1) * 3 + 1];
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[2] =
      CAVE_MachE_sil_test_B.Selector17[(ibmat - 1) * 3 + 2];

    /* Sum: '<S160>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[0] +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[0];
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[1] +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[1];
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[2] +
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[2];

    /* Sum: '<S162>/Sum3' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum3 =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum -
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_d;

    /* SignalConversion generated from: '<S144>/WhlAng' */
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2_b;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1;
    CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum3;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* Reshape: '<S144>/Reshape19' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape19[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape19[1] = 0.0;

      /* Selector: '<S157>/Selector3' */
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_e =
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape19[(int32_T)
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 - 1];
    }

    /* ForEachSliceAssignment generated from: '<S144>/WhlFz' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlFz_at_inport_0_l[ForEach_itr_hh]
      = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_e;

    /* ForEachSliceAssignment generated from: '<S144>/WhlAng' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0_p[ForEach_itr_hh
      * 3] = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0_p[ForEach_itr_hh
      * 3 + 1] = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0_p[ForEach_itr_hh
      * 3 + 2] = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
      TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2];

    /* ForEachSliceAssignment generated from: '<S144>/VehM' */
    ibmat = ForEach_itr_hh * 3;
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0_b[ibmat] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[0];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0_b[ibmat + 1] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[1];
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0_b[ibmat + 2] =
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[2];

    /* ForEachSliceAssignment generated from: '<S144>/VehFz' */
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0_j[ForEach_itr_hh]
      = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleForceSign;

    /* ForEachSliceAssignment generated from: '<S144>/Info' */
    for (y = 0; y < 6; y++) {
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Info_at_inport_0_i[y +
        ForEach_itr_hh * 6] = CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh]
        .TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[y];
    }

    /* End of ForEachSliceAssignment generated from: '<S144>/Info' */
  }

  /* End of Outputs for SubSystem: '<S135>/For each track and axle combination calculate suspension forces and moments' */

  /* Assignment: '<S182>/Assign VehFz' */
  CAVE_MachE_sil_test_B.AssignVehFz[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0[0];
  CAVE_MachE_sil_test_B.AssignVehFz[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0[1];
  CAVE_MachE_sil_test_B.AssignVehFz[(int32_T)
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[0] - 1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFzAs_at_inport_0[0];
  CAVE_MachE_sil_test_B.AssignVehFz[(int32_T)
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_AntiSwayFzInd_at_inport_0[1] - 1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFzAs_at_inport_0[1];

  /* Reshape: '<S140>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_g[0] = CAVE_MachE_sil_test_B.AssignVehFz[0];

  /* Concatenate: '<S140>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[0] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[1] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[2] =
    CAVE_MachE_sil_test_B.Reshape1_g[0];

  /* Reshape: '<S140>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_g[1] = CAVE_MachE_sil_test_B.AssignVehFz[1];

  /* Concatenate: '<S140>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[3] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[4] =
    CAVE_MachE_sil_test_B.MatrixConcatenate4[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_fi[5] =
    CAVE_MachE_sil_test_B.Reshape1_g[1];
  for (y = 0; y < 6; y++) {
    /* Reshape: '<S135>/Reshape18' */
    CAVE_MachE_sil_test_B.Reshape18_p[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehM_at_inport_0_b[y];

    /* SignalConversion generated from: '<S134>/Matrix Concatenate5' */
    CAVE_MachE_sil_test_B.M_i[y + 6] = CAVE_MachE_sil_test_B.Reshape18_p[y];

    /* SignalConversion generated from: '<S134>/Matrix Concatenate6' */
    CAVE_MachE_sil_test_B.F_k[y] = CAVE_MachE_sil_test_B.MatrixConcatenate_fi[y];
  }

  /* Reshape: '<S135>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_h[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0_j[0];
  CAVE_MachE_sil_test_B.Reshape1_h[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_VehFz_at_inport_0_j[1];

  /* Concatenate: '<S135>/Matrix Concatenate' */
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[0] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[1] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[1];
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[3] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[2];
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[4] =
    CAVE_MachE_sil_test_B.MatrixConcatenate2_h[3];
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[2] =
    CAVE_MachE_sil_test_B.Reshape1_h[0];
  CAVE_MachE_sil_test_B.MatrixConcatenate_o[5] =
    CAVE_MachE_sil_test_B.Reshape1_h[1];

  /* SignalConversion generated from: '<S134>/Matrix Concatenate6' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.F_k[y + 6] =
      CAVE_MachE_sil_test_B.MatrixConcatenate_o[y];
  }

  /* End of SignalConversion generated from: '<S134>/Matrix Concatenate6' */

  /* Reshape: '<S140>/Reshape5' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Ang[y] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0[y];
  }

  /* End of Reshape: '<S140>/Reshape5' */

  /* Reshape: '<S135>/Reshape14' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Ang[y + 6] =
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_WhlAng_at_inport_0_p[y];
  }

  /* End of Reshape: '<S135>/Reshape14' */

  /* Reshape: '<S135>/Reshape' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape_ps[0],
         &CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Info_at_inport_0_i[0], 12U *
         sizeof(real_T));

  /* Selector: '<S135>/Camber select' */
  CAVE_MachE_sil_test_B.Camberselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[0];
  CAVE_MachE_sil_test_B.Camberselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[6];

  /* Selector: '<S135>/Caster select' */
  CAVE_MachE_sil_test_B.Casterselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[1];
  CAVE_MachE_sil_test_B.Casterselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[7];

  /* Selector: '<S135>/Energy select' */
  CAVE_MachE_sil_test_B.Energyselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[5];
  CAVE_MachE_sil_test_B.Energyselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[11];

  /* Selector: '<S135>/Height select' */
  CAVE_MachE_sil_test_B.Heightselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[3];
  CAVE_MachE_sil_test_B.Heightselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[9];

  /* Selector: '<S135>/Power select' */
  CAVE_MachE_sil_test_B.Powerselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[4];
  CAVE_MachE_sil_test_B.Powerselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[10];

  /* Reshape: '<S135>/Reshape16' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape16[y] =
      CAVE_MachE_sil_test_B.MatrixConcatenate_o[y];
  }

  /* End of Reshape: '<S135>/Reshape16' */

  /* Reshape: '<S135>/Reshape17' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape17_m[y] = CAVE_MachE_sil_test_B.Reshape18_p[y];
  }

  /* End of Reshape: '<S135>/Reshape17' */

  /* Reshape: '<S135>/Reshape19' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape19_o[y] = CAVE_MachE_sil_test_B.Sum2[y];
  }

  /* End of Reshape: '<S135>/Reshape19' */

  /* Reshape: '<S135>/Reshape20' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape20[y] =
      CAVE_MachE_sil_test_B.MatrixConcatenate6_c[y];
  }

  /* End of Reshape: '<S135>/Reshape20' */

  /* Selector: '<S135>/Toe select' */
  CAVE_MachE_sil_test_B.Toeselect[0] = CAVE_MachE_sil_test_B.Reshape_ps[2];
  CAVE_MachE_sil_test_B.Toeselect[1] = CAVE_MachE_sil_test_B.Reshape_ps[8];

  /* Reshape: '<S140>/Reshape' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape_m[0],
         &CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Info_at_inport_0[0], 12U *
         sizeof(real_T));

  /* Selector: '<S140>/Camber select' */
  CAVE_MachE_sil_test_B.Camberselect_m[0] = CAVE_MachE_sil_test_B.Reshape_m[0];
  CAVE_MachE_sil_test_B.Camberselect_m[1] = CAVE_MachE_sil_test_B.Reshape_m[6];

  /* Selector: '<S140>/Caster select' */
  CAVE_MachE_sil_test_B.Casterselect_l[0] = CAVE_MachE_sil_test_B.Reshape_m[1];
  CAVE_MachE_sil_test_B.Casterselect_l[1] = CAVE_MachE_sil_test_B.Reshape_m[7];

  /* Selector: '<S140>/Energy select' */
  CAVE_MachE_sil_test_B.Energyselect_p[0] = CAVE_MachE_sil_test_B.Reshape_m[5];
  CAVE_MachE_sil_test_B.Energyselect_p[1] = CAVE_MachE_sil_test_B.Reshape_m[11];

  /* Selector: '<S140>/Height select' */
  CAVE_MachE_sil_test_B.Heightselect_l[0] = CAVE_MachE_sil_test_B.Reshape_m[3];
  CAVE_MachE_sil_test_B.Heightselect_l[1] = CAVE_MachE_sil_test_B.Reshape_m[9];

  /* Selector: '<S140>/Power select' */
  CAVE_MachE_sil_test_B.Powerselect_n[0] = CAVE_MachE_sil_test_B.Reshape_m[4];
  CAVE_MachE_sil_test_B.Powerselect_n[1] = CAVE_MachE_sil_test_B.Reshape_m[10];

  /* Reshape: '<S140>/Reshape10' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape10_g[y] =
      CAVE_MachE_sil_test_B.MatrixConcatenate_fi[y];
  }

  /* End of Reshape: '<S140>/Reshape10' */

  /* Reshape: '<S140>/Reshape11' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape11_g[y] = CAVE_MachE_sil_test_B.Reshape14[y];
  }

  /* End of Reshape: '<S140>/Reshape11' */

  /* Reshape: '<S140>/Reshape12' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape12_b[y] =
      CAVE_MachE_sil_test_B.MatrixConcatenate1_p[y];
  }

  /* End of Reshape: '<S140>/Reshape12' */

  /* Reshape: '<S140>/Reshape16' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Reshape16_i[y] =
      CAVE_MachE_sil_test_B.MatrixConcatenate6[y];
  }

  /* End of Reshape: '<S140>/Reshape16' */

  /* Selector: '<S140>/Toe select' */
  CAVE_MachE_sil_test_B.Toeselect_h[0] = CAVE_MachE_sil_test_B.Reshape_m[2];
  CAVE_MachE_sil_test_B.Toeselect_h[1] = CAVE_MachE_sil_test_B.Reshape_m[8];

  /* TransferFcn: '<S120>/Transfer Fcn5' */
  CAVE_MachE_sil_test_B.ActTqLF_Nm_ = 0.0;
  CAVE_MachE_sil_test_B.ActTqLF_Nm_ += CAVE_MachE_sil_test_P.TransferFcn5_C *
    CAVE_MachE_sil_test_X.TransferFcn5_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn6' */
  CAVE_MachE_sil_test_B.ActTqRF_Nm_ = 0.0;
  CAVE_MachE_sil_test_B.ActTqRF_Nm_ += CAVE_MachE_sil_test_P.TransferFcn6_C *
    CAVE_MachE_sil_test_X.TransferFcn6_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn7' */
  CAVE_MachE_sil_test_B.ActTqLR_Nm_ = 0.0;
  CAVE_MachE_sil_test_B.ActTqLR_Nm_ += CAVE_MachE_sil_test_P.TransferFcn7_C *
    CAVE_MachE_sil_test_X.TransferFcn7_CSTATE;

  /* TransferFcn: '<S120>/Transfer Fcn8' */
  CAVE_MachE_sil_test_B.ActTqRR_Nm_ = 0.0;
  CAVE_MachE_sil_test_B.ActTqRR_Nm_ += CAVE_MachE_sil_test_P.TransferFcn8_C *
    CAVE_MachE_sil_test_X.TransferFcn8_CSTATE;

  /* Trigonometry: '<S238>/sincos' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[0];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_a[0] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_p[0] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[1];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_a[1] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_p[1] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[2];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_a[2] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_p[2] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

  /* Fcn: '<S238>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[0] =
    CAVE_MachE_sil_test_B.sincos_o2_p[1] * CAVE_MachE_sil_test_B.sincos_o2_p[0];

  /* Fcn: '<S238>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[1] =
    CAVE_MachE_sil_test_B.sincos_o1_a[2] * CAVE_MachE_sil_test_B.sincos_o1_a[1] *
    CAVE_MachE_sil_test_B.sincos_o2_p[0] - CAVE_MachE_sil_test_B.sincos_o2_p[2] *
    CAVE_MachE_sil_test_B.sincos_o1_a[0];

  /* Fcn: '<S238>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[2] =
    CAVE_MachE_sil_test_B.sincos_o2_p[2] * CAVE_MachE_sil_test_B.sincos_o1_a[1] *
    CAVE_MachE_sil_test_B.sincos_o2_p[0] + CAVE_MachE_sil_test_B.sincos_o1_a[2] *
    CAVE_MachE_sil_test_B.sincos_o1_a[0];

  /* Fcn: '<S238>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[3] =
    CAVE_MachE_sil_test_B.sincos_o2_p[1] * CAVE_MachE_sil_test_B.sincos_o1_a[0];

  /* Fcn: '<S238>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[4] =
    CAVE_MachE_sil_test_B.sincos_o1_a[2] * CAVE_MachE_sil_test_B.sincos_o1_a[1] *
    CAVE_MachE_sil_test_B.sincos_o1_a[0] + CAVE_MachE_sil_test_B.sincos_o2_p[2] *
    CAVE_MachE_sil_test_B.sincos_o2_p[0];

  /* Fcn: '<S238>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[5] =
    CAVE_MachE_sil_test_B.sincos_o2_p[2] * CAVE_MachE_sil_test_B.sincos_o1_a[1] *
    CAVE_MachE_sil_test_B.sincos_o1_a[0] - CAVE_MachE_sil_test_B.sincos_o1_a[2] *
    CAVE_MachE_sil_test_B.sincos_o2_p[0];

  /* Fcn: '<S238>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[6] =
    -CAVE_MachE_sil_test_B.sincos_o1_a[1];

  /* Fcn: '<S238>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[7] =
    CAVE_MachE_sil_test_B.sincos_o1_a[2] * CAVE_MachE_sil_test_B.sincos_o2_p[1];

  /* Fcn: '<S238>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_b5[8] =
    CAVE_MachE_sil_test_B.sincos_o2_p[2] * CAVE_MachE_sil_test_B.sincos_o2_p[1];

  /* Reshape: '<S240>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_b5[0], 9U * sizeof(real_T));

  /* Trigonometry: '<S239>/sincos' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[0];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_o[0] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_gn[0] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[1];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_o[1] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_gn[1] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[2];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_o[2] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_gn[2] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

  /* Fcn: '<S239>/phidot' */
  CAVE_MachE_sil_test_B.phidot = (CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o1_o[0] + CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o2_gn[0]) * (CAVE_MachE_sil_test_B.sincos_o1_o
    [1] / CAVE_MachE_sil_test_B.sincos_o2_gn[1]) + CAVE_MachE_sil_test_B.pqr[0];

  /* Fcn: '<S239>/thetadot' */
  CAVE_MachE_sil_test_B.thetadot = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o2_gn[0] - CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o1_o[0];

  /* Fcn: '<S239>/psidot' */
  CAVE_MachE_sil_test_B.psidot = (CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o1_o[0] + CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o2_gn[0]) / CAVE_MachE_sil_test_B.sincos_o2_gn
    [1];

  /* SignalConversion generated from: '<S230>/phi theta psi' */
  CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[0] =
    CAVE_MachE_sil_test_B.phidot;
  CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[1] =
    CAVE_MachE_sil_test_B.thetadot;
  CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[2] =
    CAVE_MachE_sil_test_B.psidot;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    for (ibmat = 0; ibmat < 3; ibmat++) {
      /* Concatenate: '<S232>/Matrix Concatenation' incorporates:
       *  Constant: '<S217>/Constant1'
       */
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat] =
        CAVE_MachE_sil_test_B.Ibar[3 * ibmat];
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 3] =
        CAVE_MachE_sil_test_P.Constant1_Value_j[3 * ibmat];

      /* Selector: '<S231>/Selector' */
      CAVE_MachE_sil_test_B.Selector_f[3 * ibmat] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat];

      /* Concatenate: '<S232>/Matrix Concatenation' incorporates:
       *  Constant: '<S217>/Constant1'
       */
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 1] =
        CAVE_MachE_sil_test_B.Ibar[3 * ibmat + 1];
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 4] =
        CAVE_MachE_sil_test_P.Constant1_Value_j[3 * ibmat + 1];

      /* Selector: '<S231>/Selector' */
      CAVE_MachE_sil_test_B.Selector_f[3 * ibmat + 1] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 1];

      /* Concatenate: '<S232>/Matrix Concatenation' incorporates:
       *  Constant: '<S217>/Constant1'
       */
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 2] =
        CAVE_MachE_sil_test_B.Ibar[3 * ibmat + 2];
      CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 5] =
        CAVE_MachE_sil_test_P.Constant1_Value_j[3 * ibmat + 2];

      /* Selector: '<S231>/Selector' */
      CAVE_MachE_sil_test_B.Selector_f[3 * ibmat + 2] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 2];
    }
  }

  /* Reshape: '<S242>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_ar[0] = CAVE_MachE_sil_test_B.pqr[0];
  CAVE_MachE_sil_test_B.Reshape1_ar[1] = CAVE_MachE_sil_test_B.pqr[1];
  CAVE_MachE_sil_test_B.Reshape1_ar[2] = CAVE_MachE_sil_test_B.pqr[2];

  /* Product: '<S242>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Selector_f[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_ar[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_ar[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_ar[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_g[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_g[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_g[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_g[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S242>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_f[y] = CAVE_MachE_sil_test_B.Product_g[y];
  }

  /* End of Product: '<S242>/Product' */

  /* Product: '<S244>/i x j' */
  CAVE_MachE_sil_test_B.ixj_p = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Reshape2_f[1];

  /* Product: '<S244>/j x k' */
  CAVE_MachE_sil_test_B.jxk_j = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Reshape2_f[2];

  /* Product: '<S244>/k x i' */
  CAVE_MachE_sil_test_B.kxi_e = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Reshape2_f[0];

  /* Product: '<S245>/i x k' */
  CAVE_MachE_sil_test_B.ixk_k3 = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Reshape2_f[2];

  /* Product: '<S245>/j x i' */
  CAVE_MachE_sil_test_B.jxi_b1 = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Reshape2_f[0];

  /* Product: '<S245>/k x j' */
  CAVE_MachE_sil_test_B.kxj_f = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Reshape2_f[1];

  /* Sum: '<S241>/Sum' */
  CAVE_MachE_sil_test_B.Sum_a[0] = CAVE_MachE_sil_test_B.jxk_j -
    CAVE_MachE_sil_test_B.kxj_f;
  CAVE_MachE_sil_test_B.Sum_a[1] = CAVE_MachE_sil_test_B.kxi_e -
    CAVE_MachE_sil_test_B.ixk_k3;
  CAVE_MachE_sil_test_B.Sum_a[2] = CAVE_MachE_sil_test_B.ixj_p -
    CAVE_MachE_sil_test_B.jxi_b1;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S231>/Selector1' */
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.Selector1_g[3 * ibmat] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 3];
      CAVE_MachE_sil_test_B.Selector1_g[3 * ibmat + 1] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 4];
      CAVE_MachE_sil_test_B.Selector1_g[3 * ibmat + 2] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 5];
    }

    /* End of Selector: '<S231>/Selector1' */
  }

  /* Reshape: '<S243>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_f[0] = CAVE_MachE_sil_test_B.pqr[0];
  CAVE_MachE_sil_test_B.Reshape1_f[1] = CAVE_MachE_sil_test_B.pqr[1];
  CAVE_MachE_sil_test_B.Reshape1_f[2] = CAVE_MachE_sil_test_B.pqr[2];

  /* Product: '<S243>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Selector1_g[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_f[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_f[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_f[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_je[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_je[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_je[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_je[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S243>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_g[y] = CAVE_MachE_sil_test_B.Product_je[y];
  }

  /* End of Product: '<S243>/Product' */

  /* Reshape: '<S227>/Reshape' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape_c[0], &CAVE_MachE_sil_test_B.F_k[0], 12U
         * sizeof(real_T));

  /* Selector: '<S227>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_g[0] = CAVE_MachE_sil_test_B.Reshape_c[2];

  /* Reshape: '<S227>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_j[0] = CAVE_MachE_sil_test_B.Selector3_g[0];

  /* Product: '<S222>/Product2' */
  CAVE_MachE_sil_test_B.Product2_h[0] = CAVE_MachE_sil_test_B.Reshape3_j[0] *
    CAVE_MachE_sil_test_B.Wbar[0];

  /* Selector: '<S227>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2[0] = CAVE_MachE_sil_test_B.Reshape_c[1];

  /* Reshape: '<S227>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_pd[0] = CAVE_MachE_sil_test_B.Selector2[0];

  /* Product: '<S222>/Product1' */
  CAVE_MachE_sil_test_B.Product1_k[0] = CAVE_MachE_sil_test_B.Xbar[2] *
    CAVE_MachE_sil_test_B.Reshape2_pd[0];

  /* Selector: '<S227>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_g[1] = CAVE_MachE_sil_test_B.Reshape_c[5];

  /* Reshape: '<S227>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_j[1] = CAVE_MachE_sil_test_B.Selector3_g[1];

  /* Product: '<S222>/Product2' */
  CAVE_MachE_sil_test_B.Product2_h[1] = CAVE_MachE_sil_test_B.Reshape3_j[1] *
    CAVE_MachE_sil_test_B.Wbar[1];

  /* Selector: '<S227>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2[1] = CAVE_MachE_sil_test_B.Reshape_c[4];

  /* Reshape: '<S227>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_pd[1] = CAVE_MachE_sil_test_B.Selector2[1];

  /* Product: '<S222>/Product1' */
  CAVE_MachE_sil_test_B.Product1_k[1] = CAVE_MachE_sil_test_B.Xbar[2] *
    CAVE_MachE_sil_test_B.Reshape2_pd[1];

  /* Selector: '<S227>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_g[2] = CAVE_MachE_sil_test_B.Reshape_c[8];

  /* Reshape: '<S227>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_j[2] = CAVE_MachE_sil_test_B.Selector3_g[2];

  /* Product: '<S222>/Product2' */
  CAVE_MachE_sil_test_B.Product2_h[2] = CAVE_MachE_sil_test_B.Reshape3_j[2] *
    CAVE_MachE_sil_test_B.Wbar[2];

  /* Selector: '<S227>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2[2] = CAVE_MachE_sil_test_B.Reshape_c[7];

  /* Reshape: '<S227>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_pd[2] = CAVE_MachE_sil_test_B.Selector2[2];

  /* Product: '<S222>/Product1' */
  CAVE_MachE_sil_test_B.Product1_k[2] = CAVE_MachE_sil_test_B.Xbar[2] *
    CAVE_MachE_sil_test_B.Reshape2_pd[2];

  /* Selector: '<S227>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_g[3] = CAVE_MachE_sil_test_B.Reshape_c[11];

  /* Reshape: '<S227>/Reshape3' */
  CAVE_MachE_sil_test_B.Reshape3_j[3] = CAVE_MachE_sil_test_B.Selector3_g[3];

  /* Product: '<S222>/Product2' */
  CAVE_MachE_sil_test_B.Product2_h[3] = CAVE_MachE_sil_test_B.Reshape3_j[3] *
    CAVE_MachE_sil_test_B.Wbar[3];

  /* Selector: '<S227>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2[3] = CAVE_MachE_sil_test_B.Reshape_c[10];

  /* Reshape: '<S227>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_pd[3] = CAVE_MachE_sil_test_B.Selector2[3];

  /* Product: '<S222>/Product1' */
  CAVE_MachE_sil_test_B.Product1_k[3] = CAVE_MachE_sil_test_B.Xbar[2] *
    CAVE_MachE_sil_test_B.Reshape2_pd[3];

  /* Sum: '<S222>/Sum of Elements1' */
  CAVE_MachE_sil_test_B.SumofElements1 = ((CAVE_MachE_sil_test_B.Product2_h[0] -
    CAVE_MachE_sil_test_B.Product2_h[1]) + CAVE_MachE_sil_test_B.Product2_h[2])
    - CAVE_MachE_sil_test_B.Product2_h[3];

  /* Sum: '<S222>/Sum of Elements15' */
  Bias = CAVE_MachE_sil_test_B.Product1_k[0];

  /* Selector: '<S227>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_hl[0] = CAVE_MachE_sil_test_B.Reshape_c[0];

  /* Reshape: '<S227>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_b3[0] = CAVE_MachE_sil_test_B.Selector1_hl[0];

  /* Product: '<S222>/Product' */
  CAVE_MachE_sil_test_B.Product_m[0] = CAVE_MachE_sil_test_B.Reshape1_b3[0] *
    CAVE_MachE_sil_test_B.Xbar[2];

  /* Sum: '<S222>/Sum of Elements15' */
  Bias += CAVE_MachE_sil_test_B.Product1_k[1];

  /* Selector: '<S227>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_hl[1] = CAVE_MachE_sil_test_B.Reshape_c[3];

  /* Reshape: '<S227>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_b3[1] = CAVE_MachE_sil_test_B.Selector1_hl[1];

  /* Product: '<S222>/Product' */
  CAVE_MachE_sil_test_B.Product_m[1] = CAVE_MachE_sil_test_B.Reshape1_b3[1] *
    CAVE_MachE_sil_test_B.Xbar[2];

  /* Sum: '<S222>/Sum of Elements15' */
  Bias += CAVE_MachE_sil_test_B.Product1_k[2];

  /* Selector: '<S227>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_hl[2] = CAVE_MachE_sil_test_B.Reshape_c[6];

  /* Reshape: '<S227>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_b3[2] = CAVE_MachE_sil_test_B.Selector1_hl[2];

  /* Product: '<S222>/Product' */
  CAVE_MachE_sil_test_B.Product_m[2] = CAVE_MachE_sil_test_B.Reshape1_b3[2] *
    CAVE_MachE_sil_test_B.Xbar[2];

  /* Sum: '<S222>/Sum of Elements15' */
  Bias += CAVE_MachE_sil_test_B.Product1_k[3];

  /* Selector: '<S227>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_hl[3] = CAVE_MachE_sil_test_B.Reshape_c[9];

  /* Reshape: '<S227>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_b3[3] = CAVE_MachE_sil_test_B.Selector1_hl[3];

  /* Product: '<S222>/Product' */
  CAVE_MachE_sil_test_B.Product_m[3] = CAVE_MachE_sil_test_B.Reshape1_b3[3] *
    CAVE_MachE_sil_test_B.Xbar[2];

  /* Sum: '<S222>/Sum of Elements15' */
  CAVE_MachE_sil_test_B.SumofElements15 = Bias;

  /* Sum: '<S222>/Sum of Elements6' */
  CAVE_MachE_sil_test_B.VectorConcatenate_f[0] = (0.0 -
    CAVE_MachE_sil_test_B.SumofElements1) -
    CAVE_MachE_sil_test_B.SumofElements15;

  /* Sum: '<S222>/Sum of Elements2' */
  CAVE_MachE_sil_test_B.SumofElements2 = CAVE_MachE_sil_test_B.Reshape3_j[0] +
    CAVE_MachE_sil_test_B.Reshape3_j[1];

  /* Product: '<S222>/Product4' */
  CAVE_MachE_sil_test_B.Product4_i = CAVE_MachE_sil_test_B.Xbar[0] *
    CAVE_MachE_sil_test_B.SumofElements2;

  /* Sum: '<S222>/Sum of Elements4' */
  CAVE_MachE_sil_test_B.SumofElements4 = CAVE_MachE_sil_test_B.Reshape3_j[2] +
    CAVE_MachE_sil_test_B.Reshape3_j[3];

  /* Product: '<S222>/Product5' */
  CAVE_MachE_sil_test_B.Product5 = CAVE_MachE_sil_test_B.SumofElements4 *
    CAVE_MachE_sil_test_B.Xbar[1];

  /* Sum: '<S222>/Sum of Elements3' */
  CAVE_MachE_sil_test_B.SumofElements3 = CAVE_MachE_sil_test_B.Product5 -
    CAVE_MachE_sil_test_B.Product4_i;

  /* Sum: '<S222>/Sum of Elements16' */
  Bias = CAVE_MachE_sil_test_B.Product_m[0];

  /* Product: '<S222>/Product3' */
  CAVE_MachE_sil_test_B.Product3_o[0] = CAVE_MachE_sil_test_B.Reshape1_b3[0] *
    CAVE_MachE_sil_test_B.Wbar[0];

  /* Selector: '<S228>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_a[0] = CAVE_MachE_sil_test_B.M_i[0];

  /* Sum: '<S222>/Sum of Elements16' */
  Bias += CAVE_MachE_sil_test_B.Product_m[1];

  /* Product: '<S222>/Product3' */
  CAVE_MachE_sil_test_B.Product3_o[1] = CAVE_MachE_sil_test_B.Reshape1_b3[1] *
    CAVE_MachE_sil_test_B.Wbar[1];

  /* Selector: '<S228>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_a[1] = CAVE_MachE_sil_test_B.M_i[3];

  /* Sum: '<S222>/Sum of Elements16' */
  Bias += CAVE_MachE_sil_test_B.Product_m[2];

  /* Product: '<S222>/Product3' */
  CAVE_MachE_sil_test_B.Product3_o[2] = CAVE_MachE_sil_test_B.Reshape1_b3[2] *
    CAVE_MachE_sil_test_B.Wbar[2];

  /* Selector: '<S228>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_a[2] = CAVE_MachE_sil_test_B.M_i[6];

  /* Sum: '<S222>/Sum of Elements16' */
  Bias += CAVE_MachE_sil_test_B.Product_m[3];

  /* Product: '<S222>/Product3' */
  CAVE_MachE_sil_test_B.Product3_o[3] = CAVE_MachE_sil_test_B.Reshape1_b3[3] *
    CAVE_MachE_sil_test_B.Wbar[3];

  /* Selector: '<S228>/Selector1' */
  CAVE_MachE_sil_test_B.Selector1_a[3] = CAVE_MachE_sil_test_B.M_i[9];

  /* Sum: '<S222>/Sum of Elements16' */
  CAVE_MachE_sil_test_B.SumofElements16 = Bias;

  /* Sum: '<S222>/Sum of Elements9' */
  CAVE_MachE_sil_test_B.VectorConcatenate_f[1] =
    CAVE_MachE_sil_test_B.SumofElements3 + CAVE_MachE_sil_test_B.SumofElements16;

  /* Sum: '<S222>/Sum of Elements10' */
  CAVE_MachE_sil_test_B.SumofElements10 = CAVE_MachE_sil_test_B.Reshape2_pd[0] +
    CAVE_MachE_sil_test_B.Reshape2_pd[1];

  /* Product: '<S222>/Product6' */
  CAVE_MachE_sil_test_B.Product6 = CAVE_MachE_sil_test_B.Xbar[0] *
    CAVE_MachE_sil_test_B.SumofElements10;

  /* Sum: '<S222>/Sum of Elements11' */
  CAVE_MachE_sil_test_B.SumofElements11 = CAVE_MachE_sil_test_B.Reshape2_pd[2] +
    CAVE_MachE_sil_test_B.Reshape2_pd[3];

  /* Product: '<S222>/Product7' */
  CAVE_MachE_sil_test_B.Product7 = CAVE_MachE_sil_test_B.SumofElements11 *
    CAVE_MachE_sil_test_B.Xbar[1];

  /* Sum: '<S222>/Sum of Elements8' */
  CAVE_MachE_sil_test_B.SumofElements8 = CAVE_MachE_sil_test_B.Product6 -
    CAVE_MachE_sil_test_B.Product7;

  /* Sum: '<S222>/Sum of Elements7' */
  CAVE_MachE_sil_test_B.SumofElements7 = ((CAVE_MachE_sil_test_B.Product3_o[0] -
    CAVE_MachE_sil_test_B.Product3_o[1]) + CAVE_MachE_sil_test_B.Product3_o[2])
    - CAVE_MachE_sil_test_B.Product3_o[3];

  /* Sum: '<S222>/Sum of Elements5' */
  CAVE_MachE_sil_test_B.VectorConcatenate_f[2] =
    CAVE_MachE_sil_test_B.SumofElements8 + CAVE_MachE_sil_test_B.SumofElements7;

  /* Sum: '<S228>/Sum of Elements' */
  Bias = CAVE_MachE_sil_test_B.Selector1_a[0];

  /* Selector: '<S228>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2_n[0] = CAVE_MachE_sil_test_B.M_i[1];

  /* Sum: '<S228>/Sum of Elements' */
  Bias += CAVE_MachE_sil_test_B.Selector1_a[1];

  /* Selector: '<S228>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2_n[1] = CAVE_MachE_sil_test_B.M_i[4];

  /* Sum: '<S228>/Sum of Elements' */
  Bias += CAVE_MachE_sil_test_B.Selector1_a[2];

  /* Selector: '<S228>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2_n[2] = CAVE_MachE_sil_test_B.M_i[7];

  /* Sum: '<S228>/Sum of Elements' */
  Bias += CAVE_MachE_sil_test_B.Selector1_a[3];

  /* Selector: '<S228>/Selector2' */
  CAVE_MachE_sil_test_B.Selector2_n[3] = CAVE_MachE_sil_test_B.M_i[10];

  /* Sum: '<S228>/Sum of Elements' */
  CAVE_MachE_sil_test_B.VectorConcatenate_nl[0] = Bias;

  /* Sum: '<S228>/Sum of Elements1' */
  Bias = CAVE_MachE_sil_test_B.Selector2_n[0];

  /* Selector: '<S228>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_i[0] = CAVE_MachE_sil_test_B.M_i[2];

  /* Sum: '<S228>/Sum of Elements1' */
  Bias += CAVE_MachE_sil_test_B.Selector2_n[1];

  /* Selector: '<S228>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_i[1] = CAVE_MachE_sil_test_B.M_i[5];

  /* Sum: '<S228>/Sum of Elements1' */
  Bias += CAVE_MachE_sil_test_B.Selector2_n[2];

  /* Selector: '<S228>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_i[2] = CAVE_MachE_sil_test_B.M_i[8];

  /* Sum: '<S228>/Sum of Elements1' */
  Bias += CAVE_MachE_sil_test_B.Selector2_n[3];

  /* Selector: '<S228>/Selector3' */
  CAVE_MachE_sil_test_B.Selector3_i[3] = CAVE_MachE_sil_test_B.M_i[11];

  /* Sum: '<S228>/Sum of Elements1' */
  CAVE_MachE_sil_test_B.VectorConcatenate_nl[1] = Bias;

  /* Sum: '<S228>/Sum of Elements2' */
  Bias = CAVE_MachE_sil_test_B.Selector3_i[0];
  Bias += CAVE_MachE_sil_test_B.Selector3_i[1];
  Bias += CAVE_MachE_sil_test_B.Selector3_i[2];
  Bias += CAVE_MachE_sil_test_B.Selector3_i[3];
  CAVE_MachE_sil_test_B.VectorConcatenate_nl[2] = Bias;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Reshape: '<S219>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_p5[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_j[0];
    CAVE_MachE_sil_test_B.Reshape1_p5[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_j[1];
    CAVE_MachE_sil_test_B.Reshape1_p5[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_j[2];
  }

  /* Product: '<S219>/Inertial to Body' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[0], 9U *
         sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_p5[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_p5[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_p5[2];
  for (ibmat = 0; ibmat < 3; ibmat++) {
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = u[ibmat] * b_a[0];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[ibmat + 3] * b_a
      [1];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[ibmat + 6] * b_a
      [2];
    CAVE_MachE_sil_test_B.InertialtoBody[ibmat] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;
  }

  /* End of Product: '<S219>/Inertial to Body' */

  /* Sum: '<S250>/Add1' */
  CAVE_MachE_sil_test_B.Add1_n[0] = CAVE_MachE_sil_test_B.UnitConversion[0] -
    CAVE_MachE_sil_test_B.InertialtoBody[0];
  CAVE_MachE_sil_test_B.Add1_n[1] = CAVE_MachE_sil_test_B.UnitConversion[1] -
    CAVE_MachE_sil_test_B.InertialtoBody[1];
  CAVE_MachE_sil_test_B.Add1_n[2] = CAVE_MachE_sil_test_B.UnitConversion[2] -
    CAVE_MachE_sil_test_B.InertialtoBody[2];

  /* Product: '<S250>/Product' */
  CAVE_MachE_sil_test_B.Product_kk[0] = CAVE_MachE_sil_test_B.Add1_n[0] *
    CAVE_MachE_sil_test_B.Add1_n[0];
  CAVE_MachE_sil_test_B.Product_kk[1] = CAVE_MachE_sil_test_B.Add1_n[1] *
    CAVE_MachE_sil_test_B.Add1_n[1];
  CAVE_MachE_sil_test_B.Product_kk[2] = CAVE_MachE_sil_test_B.Add1_n[2] *
    CAVE_MachE_sil_test_B.Add1_n[2];

  /* Sum: '<S250>/Sum of Elements' */
  Bias = CAVE_MachE_sil_test_B.Product_kk[0];
  Bias += CAVE_MachE_sil_test_B.Product_kk[1];
  Bias += CAVE_MachE_sil_test_B.Product_kk[2];
  CAVE_MachE_sil_test_B.SumofElements_l = Bias;

  /* Sqrt: '<S250>/Sqrt' */
  CAVE_MachE_sil_test_B.Sqrt = sqrt(CAVE_MachE_sil_test_B.SumofElements_l);

  /* Product: '<S250>/Product2' */
  CAVE_MachE_sil_test_B.Product2_m = CAVE_MachE_sil_test_B.Sqrt *
    CAVE_MachE_sil_test_B.Sqrt;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Constant: '<S250>/Constant' */
    CAVE_MachE_sil_test_B.VectorConcatenate_fs[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_Cd;
  }

  /* Trigonometry: '<S250>/Trigonometric Function' */
  CAVE_MachE_sil_test_B.TrigonometricFunction = rt_atan2d_snf
    (CAVE_MachE_sil_test_B.Add1_n[1], CAVE_MachE_sil_test_B.Add1_n[0]);

  /* Lookup_n-D: '<S250>/Cs' */
  CAVE_MachE_sil_test_B.VectorConcatenate_fs[1] = look1_binlcpw
    (CAVE_MachE_sil_test_B.TrigonometricFunction,
     CAVE_MachE_sil_test_P.VehicleBody6DOF1_beta_w,
     CAVE_MachE_sil_test_P.VehicleBody6DOF1_Cs, 30U);
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Constant: '<S250>/Constant1' */
    CAVE_MachE_sil_test_B.VectorConcatenate_fs[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_Cl;
  }

  /* Lookup_n-D: '<S250>/Crm' */
  CAVE_MachE_sil_test_B.VectorConcatenate_fs[3] = look1_binlxpw
    (CAVE_MachE_sil_test_B.TrigonometricFunction,
     CAVE_MachE_sil_test_P.Crm_bp01Data, CAVE_MachE_sil_test_P.Crm_tableData, 1U);

  /* Gain: '<S250>/4' */
  CAVE_MachE_sil_test_B.u[0] = CAVE_MachE_sil_test_P.u_Gain[0] *
    CAVE_MachE_sil_test_B.Add1_n[0];

  /* Trigonometry: '<S250>/Tanh' */
  CAVE_MachE_sil_test_B.Tanh[0] = tanh(CAVE_MachE_sil_test_B.u[0]);

  /* Gain: '<S250>/4' */
  CAVE_MachE_sil_test_B.u[1] = CAVE_MachE_sil_test_P.u_Gain[1] *
    CAVE_MachE_sil_test_B.Add1_n[1];

  /* Trigonometry: '<S250>/Tanh' */
  CAVE_MachE_sil_test_B.Tanh[1] = tanh(CAVE_MachE_sil_test_B.u[1]);

  /* Gain: '<S250>/4' */
  CAVE_MachE_sil_test_B.u[2] = CAVE_MachE_sil_test_P.u_Gain[2] *
    CAVE_MachE_sil_test_B.Add1_n[2];

  /* Trigonometry: '<S250>/Tanh' */
  CAVE_MachE_sil_test_B.Tanh[2] = tanh(CAVE_MachE_sil_test_B.u[2]);

  /* Product: '<S250>/Product5' incorporates:
   *  Constant: '<S250>/Constant2'
   */
  CAVE_MachE_sil_test_B.VectorConcatenate_fs[4] = CAVE_MachE_sil_test_B.Tanh[0] *
    CAVE_MachE_sil_test_P.VehicleBody6DOF1_Cpm;

  /* Lookup_n-D: '<S250>/Cym' */
  CAVE_MachE_sil_test_B.VectorConcatenate_fs[5] = look1_binlxpw
    (CAVE_MachE_sil_test_B.TrigonometricFunction,
     CAVE_MachE_sil_test_P.VehicleBody6DOF1_beta_w,
     CAVE_MachE_sil_test_P.VehicleBody6DOF1_Cym, 30U);

  /* Gain: '<S250>/.5.*A.*Pabs.//R.//T' */
  Bias = 0.5 * CAVE_MachE_sil_test_P.ChassisFrontalArea *
    CAVE_MachE_sil_test_P.EnvPrs / CAVE_MachE_sil_test_P.DragForce_R;
  for (y = 0; y < 6; y++) {
    /* Product: '<S250>/Product1' incorporates:
     *  Constant: '<S217>/AirTempConstant'
     */
    CAVE_MachE_sil_test_B.Product1_g3[y] = CAVE_MachE_sil_test_B.Product2_m *
      CAVE_MachE_sil_test_B.VectorConcatenate_fs[y] /
      CAVE_MachE_sil_test_P.EnvTemp;

    /* Gain: '<S250>/.5.*A.*Pabs.//R.//T' */
    CAVE_MachE_sil_test_B.uAPabsRT[y] = Bias *
      CAVE_MachE_sil_test_B.Product1_g3[y];
  }

  /* Product: '<S250>/Product4' incorporates:
   *  Constant: '<S250>/Constant3'
   */
  Bias = CAVE_MachE_sil_test_P.ChassisDistCg2FrontAxle +
    CAVE_MachE_sil_test_P.ChassisDistCg2RearAxle;
  CAVE_MachE_sil_test_B.Product4_p[0] = CAVE_MachE_sil_test_B.uAPabsRT[3] * Bias;

  /* UnaryMinus: '<S217>/Unary Minus1' */
  CAVE_MachE_sil_test_B.UnaryMinus1_g[0] = -CAVE_MachE_sil_test_B.Product4_p[0];

  /* Sum: '<S217>/Add' incorporates:
   *  Constant: '<S216>/Constant1'
   */
  CAVE_MachE_sil_test_B.Add_n[0] = ((CAVE_MachE_sil_test_B.VectorConcatenate_f[0]
    + CAVE_MachE_sil_test_P.Constant1_Value_cs[0]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_nl[0]) +
    CAVE_MachE_sil_test_B.UnaryMinus1_g[0];

  /* Sum: '<S231>/Sum2' */
  CAVE_MachE_sil_test_B.Sum2_a[0] = (CAVE_MachE_sil_test_B.Add_n[0] -
    CAVE_MachE_sil_test_B.Reshape2_g[0]) - CAVE_MachE_sil_test_B.Sum_a[0];

  /* Reshape: '<S231>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_m[0] = CAVE_MachE_sil_test_B.Sum2_a[0];

  /* Product: '<S250>/Product4' */
  CAVE_MachE_sil_test_B.Product4_p[1] = CAVE_MachE_sil_test_B.uAPabsRT[4] * Bias;

  /* UnaryMinus: '<S217>/Unary Minus1' */
  CAVE_MachE_sil_test_B.UnaryMinus1_g[1] = -CAVE_MachE_sil_test_B.Product4_p[1];

  /* Sum: '<S217>/Add' incorporates:
   *  Constant: '<S216>/Constant1'
   */
  CAVE_MachE_sil_test_B.Add_n[1] = ((CAVE_MachE_sil_test_B.VectorConcatenate_f[1]
    + CAVE_MachE_sil_test_P.Constant1_Value_cs[1]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_nl[1]) +
    CAVE_MachE_sil_test_B.UnaryMinus1_g[1];

  /* Sum: '<S231>/Sum2' */
  CAVE_MachE_sil_test_B.Sum2_a[1] = (CAVE_MachE_sil_test_B.Add_n[1] -
    CAVE_MachE_sil_test_B.Reshape2_g[1]) - CAVE_MachE_sil_test_B.Sum_a[1];

  /* Reshape: '<S231>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_m[1] = CAVE_MachE_sil_test_B.Sum2_a[1];

  /* Product: '<S250>/Product4' */
  CAVE_MachE_sil_test_B.Product4_p[2] = CAVE_MachE_sil_test_B.uAPabsRT[5] * Bias;

  /* UnaryMinus: '<S217>/Unary Minus1' */
  CAVE_MachE_sil_test_B.UnaryMinus1_g[2] = -CAVE_MachE_sil_test_B.Product4_p[2];

  /* Sum: '<S217>/Add' incorporates:
   *  Constant: '<S216>/Constant1'
   */
  CAVE_MachE_sil_test_B.Add_n[2] = ((CAVE_MachE_sil_test_B.VectorConcatenate_f[2]
    + CAVE_MachE_sil_test_P.Constant1_Value_cs[2]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_nl[2]) +
    CAVE_MachE_sil_test_B.UnaryMinus1_g[2];

  /* Sum: '<S231>/Sum2' */
  CAVE_MachE_sil_test_B.Sum2_a[2] = (CAVE_MachE_sil_test_B.Add_n[2] -
    CAVE_MachE_sil_test_B.Reshape2_g[2]) - CAVE_MachE_sil_test_B.Sum_a[2];

  /* Reshape: '<S231>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_m[2] = CAVE_MachE_sil_test_B.Sum2_a[2];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Selector: '<S231>/Selector2' */
    for (ibmat = 0; ibmat < 3; ibmat++) {
      CAVE_MachE_sil_test_B.Selector2_k[3 * ibmat] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat];
      CAVE_MachE_sil_test_B.Selector2_k[3 * ibmat + 1] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 1];
      CAVE_MachE_sil_test_B.Selector2_k[3 * ibmat + 2] =
        CAVE_MachE_sil_test_B.MatrixConcatenation[6 * ibmat + 2];
    }

    /* End of Selector: '<S231>/Selector2' */
  }

  /* Product: '<S231>/Product2' */
  rt_mrdivide_U1d1x3_U2d3x3_Yd1x3_snf(CAVE_MachE_sil_test_B.Reshape1_m,
    CAVE_MachE_sil_test_B.Selector2_k, CAVE_MachE_sil_test_B.Product2_d);

  /* Reshape: '<S231>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_o[0] = CAVE_MachE_sil_test_B.Product2_d[0];
  CAVE_MachE_sil_test_B.Reshape_o[1] = CAVE_MachE_sil_test_B.Product2_d[1];
  CAVE_MachE_sil_test_B.Reshape_o[2] = CAVE_MachE_sil_test_B.Product2_d[2];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* UnitConversion: '<S236>/Unit Conversion' */
    /* Unit Conversion - from: m/s to: m/s
       Expression: output = (1*input) + (0) */
    CAVE_MachE_sil_test_B.UnitConversion_k[0] = 0.0;
    CAVE_MachE_sil_test_B.UnitConversion_k[1] = 0.0;
    CAVE_MachE_sil_test_B.UnitConversion_k[2] = 0.0;

    /* Outputs for Iterator SubSystem: '<S246>/For Each Subsystem' incorporates:
     *  ForEach: '<S247>/For Each'
     */
    for (ForEach_itr_f = 0; ForEach_itr_f < 1; ForEach_itr_f++) {
      /* ForEachSliceSelector generated from: '<S247>/Vre' */
      ibmat = ForEach_itr_f * 3;
      Bias = CAVE_MachE_sil_test_B.UnitConversion_k[ibmat];

      /* Product: '<S247>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[0] = 0.0 * Bias;

      /* ForEachSliceAssignment generated from: '<S247>/F' */
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[0];

      /* ForEachSliceSelector generated from: '<S247>/Vre' */
      Bias = CAVE_MachE_sil_test_B.UnitConversion_k[ibmat + 1];

      /* Product: '<S247>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[1] = 0.0 * Bias;

      /* ForEachSliceAssignment generated from: '<S247>/F' */
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[1];

      /* ForEachSliceSelector generated from: '<S247>/Vre' */
      Bias = CAVE_MachE_sil_test_B.UnitConversion_k[ibmat + 2];

      /* Product: '<S247>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[2] = 0.0 * Bias;

      /* ForEachSliceAssignment generated from: '<S247>/F' */
      CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[2] =
        CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[2];
    }

    /* End of Outputs for SubSystem: '<S246>/For Each Subsystem' */

    /* Sum: '<S246>/Sum of Elements' */
    Bias = CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[0];
    CAVE_MachE_sil_test_B.SumofElements_e[0] = Bias;
    Bias = CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[1];
    CAVE_MachE_sil_test_B.SumofElements_e[1] = Bias;
    Bias = CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_F_at_inport_0[2];
    CAVE_MachE_sil_test_B.SumofElements_e[2] = Bias;

    /* SignalConversion generated from: '<S221>/Vector Concatenate' incorporates:
     *  Constant: '<S221>/0'
     */
    CAVE_MachE_sil_test_B.Inertialgravityvector[0] =
      CAVE_MachE_sil_test_P.u_Value_g;

    /* SignalConversion generated from: '<S221>/Vector Concatenate' incorporates:
     *  Constant: '<S221>/0'
     */
    CAVE_MachE_sil_test_B.Inertialgravityvector[1] =
      CAVE_MachE_sil_test_P.u_Value_g;

    /* Constant: '<S221>/g' */
    CAVE_MachE_sil_test_B.Inertialgravityvector[2] =
      CAVE_MachE_sil_test_P.EnvGravCnst;

    /* Reshape: '<S232>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_h[0] = CAVE_MachE_sil_test_B.SumofElements_e[0];

    /* Product: '<S221>/Product' */
    CAVE_MachE_sil_test_B.Fg_I[0] = CAVE_MachE_sil_test_B.Inertialgravityvector
      [0] * CAVE_MachE_sil_test_B.Mbar;

    /* Reshape: '<S221>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_ox[0] = CAVE_MachE_sil_test_B.Fg_I[0];

    /* Reshape: '<S232>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_h[1] = CAVE_MachE_sil_test_B.SumofElements_e[1];

    /* Product: '<S221>/Product' */
    CAVE_MachE_sil_test_B.Fg_I[1] = CAVE_MachE_sil_test_B.Inertialgravityvector
      [1] * CAVE_MachE_sil_test_B.Mbar;

    /* Reshape: '<S221>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_ox[1] = CAVE_MachE_sil_test_B.Fg_I[1];

    /* Reshape: '<S232>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_h[2] = CAVE_MachE_sil_test_B.SumofElements_e[2];

    /* Product: '<S221>/Product' */
    CAVE_MachE_sil_test_B.Fg_I[2] = CAVE_MachE_sil_test_B.Inertialgravityvector
      [2] * CAVE_MachE_sil_test_B.Mbar;

    /* Reshape: '<S221>/Reshape' */
    CAVE_MachE_sil_test_B.Reshape_ox[2] = CAVE_MachE_sil_test_B.Fg_I[2];
  }

  /* Product: '<S221>/Inertial to Body' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[0], 9U *
         sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape_ox[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape_ox[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape_ox[2];
  for (y = 0; y < 3; y++) {
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 = u[y] * b_a[0];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[y + 3] * b_a[1];
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2 += u[y + 6] * b_a[2];
    CAVE_MachE_sil_test_B.Fg_B[y] =
      rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_2;

    /* Product: '<S250>/Product3' */
    CAVE_MachE_sil_test_B.Product3_kb[y] = CAVE_MachE_sil_test_B.Tanh[y] *
      CAVE_MachE_sil_test_B.uAPabsRT[y];
  }

  /* End of Product: '<S221>/Inertial to Body' */

  /* Sum: '<S227>/Sum of Elements' */
  Bias = CAVE_MachE_sil_test_B.Reshape1_b3[0];
  Bias += CAVE_MachE_sil_test_B.Reshape1_b3[1];
  Bias += CAVE_MachE_sil_test_B.Reshape1_b3[2];
  Bias += CAVE_MachE_sil_test_B.Reshape1_b3[3];
  CAVE_MachE_sil_test_B.VectorConcatenate_pf5[0] = Bias;

  /* Sum: '<S227>/Sum of Elements1' */
  Bias = CAVE_MachE_sil_test_B.Reshape2_pd[0];
  Bias += CAVE_MachE_sil_test_B.Reshape2_pd[1];
  Bias += CAVE_MachE_sil_test_B.Reshape2_pd[2];
  Bias += CAVE_MachE_sil_test_B.Reshape2_pd[3];
  CAVE_MachE_sil_test_B.VectorConcatenate_pf5[1] = Bias;

  /* Sum: '<S227>/Sum of Elements2' */
  Bias = CAVE_MachE_sil_test_B.Reshape3_j[0];
  Bias += CAVE_MachE_sil_test_B.Reshape3_j[1];
  Bias += CAVE_MachE_sil_test_B.Reshape3_j[2];
  Bias += CAVE_MachE_sil_test_B.Reshape3_j[3];
  CAVE_MachE_sil_test_B.VectorConcatenate_pf5[2] = Bias;

  /* Sum: '<S217>/Sum' incorporates:
   *  Constant: '<S216>/Constant'
   */
  CAVE_MachE_sil_test_B.Sum_o[0] = ((CAVE_MachE_sil_test_P.Constant_Value_kk[0]
    + CAVE_MachE_sil_test_B.Fg_B[0]) - CAVE_MachE_sil_test_B.Product3_kb[0]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_pf5[0];

  /* Sum: '<S232>/Sum' */
  CAVE_MachE_sil_test_B.Sum_op[0] = CAVE_MachE_sil_test_B.Sum_o[0] +
    CAVE_MachE_sil_test_B.Reshape_h[0];

  /* Product: '<S218>/Product' */
  CAVE_MachE_sil_test_B.Product_fl[0] = CAVE_MachE_sil_test_B.Sum_op[0] /
    CAVE_MachE_sil_test_B.Mbar;

  /* Sum: '<S217>/Sum' incorporates:
   *  Constant: '<S216>/Constant'
   */
  CAVE_MachE_sil_test_B.Sum_o[1] = ((CAVE_MachE_sil_test_P.Constant_Value_kk[1]
    + CAVE_MachE_sil_test_B.Fg_B[1]) - CAVE_MachE_sil_test_B.Product3_kb[1]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_pf5[1];

  /* Sum: '<S232>/Sum' */
  CAVE_MachE_sil_test_B.Sum_op[1] = CAVE_MachE_sil_test_B.Sum_o[1] +
    CAVE_MachE_sil_test_B.Reshape_h[1];

  /* Product: '<S218>/Product' */
  CAVE_MachE_sil_test_B.Product_fl[1] = CAVE_MachE_sil_test_B.Sum_op[1] /
    CAVE_MachE_sil_test_B.Mbar;

  /* Sum: '<S217>/Sum' incorporates:
   *  Constant: '<S216>/Constant'
   */
  CAVE_MachE_sil_test_B.Sum_o[2] = ((CAVE_MachE_sil_test_P.Constant_Value_kk[2]
    + CAVE_MachE_sil_test_B.Fg_B[2]) - CAVE_MachE_sil_test_B.Product3_kb[2]) +
    CAVE_MachE_sil_test_B.VectorConcatenate_pf5[2];

  /* Sum: '<S232>/Sum' */
  CAVE_MachE_sil_test_B.Sum_op[2] = CAVE_MachE_sil_test_B.Sum_o[2] +
    CAVE_MachE_sil_test_B.Reshape_h[2];

  /* Product: '<S218>/Product' */
  CAVE_MachE_sil_test_B.Product_fl[2] = CAVE_MachE_sil_test_B.Sum_op[2] /
    CAVE_MachE_sil_test_B.Mbar;

  /* Product: '<S248>/j x k' */
  CAVE_MachE_sil_test_B.jxk_h = CAVE_MachE_sil_test_B.ubvbwb[1] *
    CAVE_MachE_sil_test_B.pqr[2];

  /* Product: '<S248>/k x i' */
  CAVE_MachE_sil_test_B.kxi_f = CAVE_MachE_sil_test_B.ubvbwb[2] *
    CAVE_MachE_sil_test_B.pqr[0];

  /* Product: '<S248>/i x j' */
  CAVE_MachE_sil_test_B.ixj_ol = CAVE_MachE_sil_test_B.ubvbwb[0] *
    CAVE_MachE_sil_test_B.pqr[1];

  /* Product: '<S249>/k x j' */
  CAVE_MachE_sil_test_B.kxj_k = CAVE_MachE_sil_test_B.ubvbwb[2] *
    CAVE_MachE_sil_test_B.pqr[1];

  /* Product: '<S249>/i x k' */
  CAVE_MachE_sil_test_B.ixk_d = CAVE_MachE_sil_test_B.ubvbwb[0] *
    CAVE_MachE_sil_test_B.pqr[2];

  /* Product: '<S249>/j x i' */
  CAVE_MachE_sil_test_B.jxi_jh = CAVE_MachE_sil_test_B.ubvbwb[1] *
    CAVE_MachE_sil_test_B.pqr[0];

  /* Sum: '<S233>/Sum' */
  CAVE_MachE_sil_test_B.Sum_kr[0] = CAVE_MachE_sil_test_B.jxk_h -
    CAVE_MachE_sil_test_B.kxj_k;
  CAVE_MachE_sil_test_B.Sum_kr[1] = CAVE_MachE_sil_test_B.kxi_f -
    CAVE_MachE_sil_test_B.ixk_d;
  CAVE_MachE_sil_test_B.Sum_kr[2] = CAVE_MachE_sil_test_B.ixj_ol -
    CAVE_MachE_sil_test_B.jxi_jh;

  /* Sum: '<S218>/Sum' */
  CAVE_MachE_sil_test_B.Sum_dx[0] = CAVE_MachE_sil_test_B.Product_fl[0] +
    CAVE_MachE_sil_test_B.Sum_kr[0];
  CAVE_MachE_sil_test_B.Sum_dx[1] = CAVE_MachE_sil_test_B.Product_fl[1] +
    CAVE_MachE_sil_test_B.Sum_kr[1];
  CAVE_MachE_sil_test_B.Sum_dx[2] = CAVE_MachE_sil_test_B.Product_fl[2] +
    CAVE_MachE_sil_test_B.Sum_kr[2];
  for (y = 0; y < 3; y++) {
    /* Math: '<S218>/Transpose' */
    CAVE_MachE_sil_test_B.Transpose[3 * y] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[y];
    CAVE_MachE_sil_test_B.Transpose[3 * y + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[y + 3];
    CAVE_MachE_sil_test_B.Transpose[3 * y + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_i[y + 6];

    /* Reshape: '<S237>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_d[y] = CAVE_MachE_sil_test_B.ubvbwb[y];
  }

  /* Product: '<S237>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_d[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_d[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_d[2];
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Product_aa[ibmat] = 0.0;
    CAVE_MachE_sil_test_B.Product_aa[ibmat] += u[ibmat] * b_a[0];
    CAVE_MachE_sil_test_B.Product_aa[ibmat] += u[ibmat + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_aa[ibmat] += u[ibmat + 6] * b_a[2];
  }

  /* End of Product: '<S237>/Product' */

  /* Reshape: '<S237>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_me[0] = CAVE_MachE_sil_test_B.Product_aa[0];
  CAVE_MachE_sil_test_B.Reshape2_me[1] = CAVE_MachE_sil_test_B.Product_aa[1];
  CAVE_MachE_sil_test_B.Reshape2_me[2] = CAVE_MachE_sil_test_B.Product_aa[2];

  /* UnitConversion: '<S235>/Unit Conversion' */
  /* Unit Conversion - from: m/s to: m/s
     Expression: output = (1*input) + (0) */
  CAVE_MachE_sil_test_B.UnitConversion_j[0] = CAVE_MachE_sil_test_B.Reshape2_me
    [0];
  CAVE_MachE_sil_test_B.UnitConversion_j[1] = CAVE_MachE_sil_test_B.Reshape2_me
    [1];
  CAVE_MachE_sil_test_B.UnitConversion_j[2] = CAVE_MachE_sil_test_B.Reshape2_me
    [2];
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Constant: '<S217>/Constant' */
    memcpy(&CAVE_MachE_sil_test_B.Constant_o[0],
           &CAVE_MachE_sil_test_P.Constant_Value_kb[0], 12U * sizeof(real_T));
  }

  /* Product: '<S224>/Product8' */
  CAVE_MachE_sil_test_B.VectorConcatenate_a[0] = CAVE_MachE_sil_test_B.Sum_o[0] *
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.VectorConcatenate_a[1] = CAVE_MachE_sil_test_B.Sum_o[1] *
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.VectorConcatenate_a[2] = CAVE_MachE_sil_test_B.Sum_o[2] *
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Product: '<S224>/Product1' */
  CAVE_MachE_sil_test_B.VectorConcatenate_a[3] = CAVE_MachE_sil_test_B.Add_n[0] *
    CAVE_MachE_sil_test_B.pqr[0];
  CAVE_MachE_sil_test_B.VectorConcatenate_a[4] = CAVE_MachE_sil_test_B.Add_n[1] *
    CAVE_MachE_sil_test_B.pqr[1];
  CAVE_MachE_sil_test_B.VectorConcatenate_a[5] = CAVE_MachE_sil_test_B.Add_n[2] *
    CAVE_MachE_sil_test_B.pqr[2];

  /* Abs: '<S224>/Abs' */
  for (y = 0; y < 6; y++) {
    CAVE_MachE_sil_test_B.Abs_f5[y] = fabs
      (CAVE_MachE_sil_test_B.VectorConcatenate_a[y]);
  }

  /* End of Abs: '<S224>/Abs' */

  /* UnaryMinus: '<S217>/Unary Minus' */
  CAVE_MachE_sil_test_B.UnaryMinus_o[0] = -CAVE_MachE_sil_test_B.Product3_kb[0];
  CAVE_MachE_sil_test_B.UnaryMinus_o[1] = -CAVE_MachE_sil_test_B.Product3_kb[1];
  CAVE_MachE_sil_test_B.UnaryMinus_o[2] = -CAVE_MachE_sil_test_B.Product3_kb[2];

  /* Product: '<S224>/Product9' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[0] =
    CAVE_MachE_sil_test_B.UnitConversion[0] *
    CAVE_MachE_sil_test_B.UnaryMinus_o[0];
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[1] =
    CAVE_MachE_sil_test_B.UnitConversion[1] *
    CAVE_MachE_sil_test_B.UnaryMinus_o[1];
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[2] =
    CAVE_MachE_sil_test_B.UnitConversion[2] *
    CAVE_MachE_sil_test_B.UnaryMinus_o[2];

  /* Product: '<S224>/Product2' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[3] = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.UnaryMinus1_g[0];
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[4] = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.UnaryMinus1_g[1];
  CAVE_MachE_sil_test_B.VectorConcatenate1_j[5] = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.UnaryMinus1_g[2];

  /* Sum: '<S224>/Sum of Elements' */
  Bias = -0.0;
  for (y = 0; y < 6; y++) {
    /* Abs: '<S224>/Abs1' */
    CAVE_MachE_sil_test_B.Abs1_b[y] = fabs
      (CAVE_MachE_sil_test_B.VectorConcatenate1_j[y]);

    /* Sum: '<S224>/Sum of Elements' */
    Bias += CAVE_MachE_sil_test_B.Abs_f5[y];
  }

  /* Sum: '<S224>/Sum of Elements' */
  CAVE_MachE_sil_test_B.SumofElements_g = Bias;

  /* Sum: '<S224>/Sum of Elements1' */
  Bias = -0.0;
  for (y = 0; y < 6; y++) {
    Bias += CAVE_MachE_sil_test_B.Abs1_b[y];
  }

  CAVE_MachE_sil_test_B.SumofElements1_d = Bias;

  /* End of Sum: '<S224>/Sum of Elements1' */

  /* Reshape: '<S258>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_k0[0] = CAVE_MachE_sil_test_B.Sum_j[0];
  CAVE_MachE_sil_test_B.Reshape1_k0[1] = CAVE_MachE_sil_test_B.Sum_j[1];
  CAVE_MachE_sil_test_B.Reshape1_k0[2] = CAVE_MachE_sil_test_B.Sum_j[2];

  /* Product: '<S258>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_k0[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_k0[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_k0[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_c[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_c[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_c[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_c[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S258>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_a[y] = CAVE_MachE_sil_test_B.Product_c[y];
  }

  /* End of Product: '<S258>/Product' */

  /* Sum: '<S256>/Add4' */
  CAVE_MachE_sil_test_B.V_wb[0] = CAVE_MachE_sil_test_B.UnitConversion_j[0] +
    CAVE_MachE_sil_test_B.Reshape2_a[0];
  CAVE_MachE_sil_test_B.V_wb[1] = CAVE_MachE_sil_test_B.UnitConversion_j[1] +
    CAVE_MachE_sil_test_B.Reshape2_a[1];
  CAVE_MachE_sil_test_B.V_wb[2] = CAVE_MachE_sil_test_B.UnitConversion_j[2] +
    CAVE_MachE_sil_test_B.Reshape2_a[2];

  /* Reshape: '<S266>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_dx[0] = CAVE_MachE_sil_test_B.Sum_k[0];
  CAVE_MachE_sil_test_B.Reshape1_dx[1] = CAVE_MachE_sil_test_B.Sum_k[1];
  CAVE_MachE_sil_test_B.Reshape1_dx[2] = CAVE_MachE_sil_test_B.Sum_k[2];

  /* Product: '<S266>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_p[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_dx[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_dx[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_dx[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_cv[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_cv[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_cv[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_cv[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S266>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_c[y] = CAVE_MachE_sil_test_B.Product_cv[y];
  }

  /* End of Product: '<S266>/Product' */

  /* Sum: '<S264>/Add4' */
  CAVE_MachE_sil_test_B.V_wb_k[0] = CAVE_MachE_sil_test_B.UnitConversion_j[0] +
    CAVE_MachE_sil_test_B.Reshape2_c[0];
  CAVE_MachE_sil_test_B.V_wb_k[1] = CAVE_MachE_sil_test_B.UnitConversion_j[1] +
    CAVE_MachE_sil_test_B.Reshape2_c[1];
  CAVE_MachE_sil_test_B.V_wb_k[2] = CAVE_MachE_sil_test_B.UnitConversion_j[2] +
    CAVE_MachE_sil_test_B.Reshape2_c[2];

  /* Trigonometry: '<S274>/sincos' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[0];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_hn[0] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_d[0] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[1];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_hn[1] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_d[1] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtsincosInport1[2];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_hn[2] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_d[2] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

  /* Fcn: '<S274>/Fcn11' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[0] =
    CAVE_MachE_sil_test_B.sincos_o2_d[1] * CAVE_MachE_sil_test_B.sincos_o2_d[0];

  /* Fcn: '<S274>/Fcn21' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[1] =
    CAVE_MachE_sil_test_B.sincos_o1_hn[2] * CAVE_MachE_sil_test_B.sincos_o1_hn[1]
    * CAVE_MachE_sil_test_B.sincos_o2_d[0] - CAVE_MachE_sil_test_B.sincos_o2_d[2]
    * CAVE_MachE_sil_test_B.sincos_o1_hn[0];

  /* Fcn: '<S274>/Fcn31' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[2] =
    CAVE_MachE_sil_test_B.sincos_o2_d[2] * CAVE_MachE_sil_test_B.sincos_o1_hn[1]
    * CAVE_MachE_sil_test_B.sincos_o2_d[0] + CAVE_MachE_sil_test_B.sincos_o1_hn
    [2] * CAVE_MachE_sil_test_B.sincos_o1_hn[0];

  /* Fcn: '<S274>/Fcn12' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[3] =
    CAVE_MachE_sil_test_B.sincos_o2_d[1] * CAVE_MachE_sil_test_B.sincos_o1_hn[0];

  /* Fcn: '<S274>/Fcn22' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[4] =
    CAVE_MachE_sil_test_B.sincos_o1_hn[2] * CAVE_MachE_sil_test_B.sincos_o1_hn[1]
    * CAVE_MachE_sil_test_B.sincos_o1_hn[0] + CAVE_MachE_sil_test_B.sincos_o2_d
    [2] * CAVE_MachE_sil_test_B.sincos_o2_d[0];

  /* Fcn: '<S274>/Fcn32' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[5] =
    CAVE_MachE_sil_test_B.sincos_o2_d[2] * CAVE_MachE_sil_test_B.sincos_o1_hn[1]
    * CAVE_MachE_sil_test_B.sincos_o1_hn[0] -
    CAVE_MachE_sil_test_B.sincos_o1_hn[2] * CAVE_MachE_sil_test_B.sincos_o2_d[0];

  /* Fcn: '<S274>/Fcn13' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[6] =
    -CAVE_MachE_sil_test_B.sincos_o1_hn[1];

  /* Fcn: '<S274>/Fcn23' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[7] =
    CAVE_MachE_sil_test_B.sincos_o1_hn[2] * CAVE_MachE_sil_test_B.sincos_o2_d[1];

  /* Fcn: '<S274>/Fcn33' */
  CAVE_MachE_sil_test_B.VectorConcatenate_am[8] =
    CAVE_MachE_sil_test_B.sincos_o2_d[2] * CAVE_MachE_sil_test_B.sincos_o2_d[1];

  /* Reshape: '<S281>/Reshape (9) to [3x3] column-major' */
  memcpy(&CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_e[0],
         &CAVE_MachE_sil_test_B.VectorConcatenate_am[0], 9U * sizeof(real_T));

  /* Math: '<S272>/Transpose1' */
  for (ibmat = 0; ibmat < 3; ibmat++) {
    CAVE_MachE_sil_test_B.Transpose1_e[3 * ibmat] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_e[ibmat];
    CAVE_MachE_sil_test_B.Transpose1_e[3 * ibmat + 1] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_e[ibmat + 3];
    CAVE_MachE_sil_test_B.Transpose1_e[3 * ibmat + 2] =
      CAVE_MachE_sil_test_B.Reshape9to3x3columnmajor_e[ibmat + 6];
  }

  /* End of Math: '<S272>/Transpose1' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Constant: '<S253>/longOff' */
    CAVE_MachE_sil_test_B.VectorConcatenate_i[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_longOff;

    /* Constant: '<S253>/latOff' */
    CAVE_MachE_sil_test_B.VectorConcatenate_i[1] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_latOff;

    /* Constant: '<S253>/vertOff ' */
    CAVE_MachE_sil_test_B.VectorConcatenate_i[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_vertOff;

    /* Sum: '<S253>/Subtract' */
    CAVE_MachE_sil_test_B.Subtract_g[0] =
      CAVE_MachE_sil_test_B.VectorConcatenate_i[0] + CAVE_MachE_sil_test_B.Rbar
      [0];

    /* Reshape: '<S276>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_iq[0] = CAVE_MachE_sil_test_B.Subtract_g[0];

    /* Sum: '<S253>/Subtract' */
    CAVE_MachE_sil_test_B.Subtract_g[1] =
      CAVE_MachE_sil_test_B.VectorConcatenate_i[1] + CAVE_MachE_sil_test_B.Rbar
      [1];

    /* Reshape: '<S276>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_iq[1] = CAVE_MachE_sil_test_B.Subtract_g[1];

    /* Sum: '<S253>/Subtract' */
    CAVE_MachE_sil_test_B.Subtract_g[2] =
      CAVE_MachE_sil_test_B.VectorConcatenate_i[2] + CAVE_MachE_sil_test_B.Rbar
      [2];

    /* Reshape: '<S276>/Reshape1' */
    CAVE_MachE_sil_test_B.Reshape1_iq[2] = CAVE_MachE_sil_test_B.Subtract_g[2];
  }

  /* Product: '<S276>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_e[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_iq[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_iq[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_iq[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_kf[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_kf[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_kf[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_kf[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S276>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_j[y] = CAVE_MachE_sil_test_B.Product_kf[y];
  }

  /* End of Product: '<S276>/Product' */

  /* Sum: '<S272>/Add' */
  CAVE_MachE_sil_test_B.Add_h[0] = CAVE_MachE_sil_test_B.xeyeze[0] +
    CAVE_MachE_sil_test_B.Reshape2_j[0];
  CAVE_MachE_sil_test_B.Add_h[1] = CAVE_MachE_sil_test_B.xeyeze[1] +
    CAVE_MachE_sil_test_B.Reshape2_j[1];
  CAVE_MachE_sil_test_B.Add_h[2] = CAVE_MachE_sil_test_B.xeyeze[2] +
    CAVE_MachE_sil_test_B.Reshape2_j[2];

  /* Product: '<S282>/j x k' */
  CAVE_MachE_sil_test_B.jxk_c = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Subtract_g[2];

  /* Product: '<S282>/k x i' */
  CAVE_MachE_sil_test_B.kxi_o2 = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Subtract_g[0];

  /* Product: '<S282>/i x j' */
  CAVE_MachE_sil_test_B.ixj_f = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Subtract_g[1];

  /* Product: '<S283>/k x j' */
  CAVE_MachE_sil_test_B.kxj_j = CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.Subtract_g[1];

  /* Product: '<S283>/i x k' */
  CAVE_MachE_sil_test_B.ixk_m = CAVE_MachE_sil_test_B.pqr[0] *
    CAVE_MachE_sil_test_B.Subtract_g[2];

  /* Product: '<S283>/j x i' */
  CAVE_MachE_sil_test_B.jxi_f = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.Subtract_g[0];

  /* Sum: '<S277>/Sum' */
  CAVE_MachE_sil_test_B.Sum_l[0] = CAVE_MachE_sil_test_B.jxk_c -
    CAVE_MachE_sil_test_B.kxj_j;
  CAVE_MachE_sil_test_B.Sum_l[1] = CAVE_MachE_sil_test_B.kxi_o2 -
    CAVE_MachE_sil_test_B.ixk_m;
  CAVE_MachE_sil_test_B.Sum_l[2] = CAVE_MachE_sil_test_B.ixj_f -
    CAVE_MachE_sil_test_B.jxi_f;

  /* Sum: '<S272>/Add1' */
  CAVE_MachE_sil_test_B.Add1_o[0] = CAVE_MachE_sil_test_B.Sum_l[0] +
    CAVE_MachE_sil_test_B.UnitConversion[0];
  CAVE_MachE_sil_test_B.Add1_o[1] = CAVE_MachE_sil_test_B.Sum_l[1] +
    CAVE_MachE_sil_test_B.UnitConversion[1];
  CAVE_MachE_sil_test_B.Add1_o[2] = CAVE_MachE_sil_test_B.Sum_l[2] +
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Reshape: '<S275>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_ff[0] = CAVE_MachE_sil_test_B.Sum_l[0];
  CAVE_MachE_sil_test_B.Reshape1_ff[1] = CAVE_MachE_sil_test_B.Sum_l[1];
  CAVE_MachE_sil_test_B.Reshape1_ff[2] = CAVE_MachE_sil_test_B.Sum_l[2];

  /* Product: '<S275>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_e[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_ff[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_ff[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_ff[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_i[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_i[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_i[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_i[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S275>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_ca[y] = CAVE_MachE_sil_test_B.Product_i[y];
  }

  /* End of Product: '<S275>/Product' */

  /* Sum: '<S272>/Add4' */
  CAVE_MachE_sil_test_B.V_wb_o[0] = CAVE_MachE_sil_test_B.UnitConversion_j[0] +
    CAVE_MachE_sil_test_B.Reshape2_ca[0];
  CAVE_MachE_sil_test_B.V_wb_o[1] = CAVE_MachE_sil_test_B.UnitConversion_j[1] +
    CAVE_MachE_sil_test_B.Reshape2_ca[1];
  CAVE_MachE_sil_test_B.V_wb_o[2] = CAVE_MachE_sil_test_B.UnitConversion_j[2] +
    CAVE_MachE_sil_test_B.Reshape2_ca[2];

  /* Fcn: '<S278>/Fcn' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.Add1_o[0] / 0.01;
  Bias = rt_powd_snf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1, 2.0);
  CAVE_MachE_sil_test_B.Fcn_i = 0.02 / (3.0 - Bias);

  /* RelationalOperator: '<S279>/Compare' incorporates:
   *  Constant: '<S279>/Constant'
   */
  Bias = -CAVE_MachE_sil_test_P.VehicleBody6DOF1_xdot_tol;
  CAVE_MachE_sil_test_B.Compare_ku = (CAVE_MachE_sil_test_B.Add1_o[0] >= Bias);

  /* RelationalOperator: '<S280>/Compare' incorporates:
   *  Constant: '<S280>/Constant'
   */
  CAVE_MachE_sil_test_B.Compare_o = (CAVE_MachE_sil_test_B.Add1_o[0] <=
    CAVE_MachE_sil_test_P.VehicleBody6DOF1_xdot_tol);

  /* Logic: '<S278>/Logical Operator' */
  CAVE_MachE_sil_test_B.LogicalOperator_p = (CAVE_MachE_sil_test_B.Compare_ku &&
    CAVE_MachE_sil_test_B.Compare_o);

  /* Abs: '<S278>/Abs' */
  CAVE_MachE_sil_test_B.Abs_la = fabs(CAVE_MachE_sil_test_B.Add1_o[0]);

  /* Switch: '<S278>/Switch' */
  if (CAVE_MachE_sil_test_B.LogicalOperator_p) {
    CAVE_MachE_sil_test_B.Switch_d = CAVE_MachE_sil_test_B.Fcn_i;
  } else {
    CAVE_MachE_sil_test_B.Switch_d = CAVE_MachE_sil_test_B.Abs_la;
  }

  /* End of Switch: '<S278>/Switch' */

  /* Product: '<S273>/Divide' */
  CAVE_MachE_sil_test_B.Divide_f2 = CAVE_MachE_sil_test_B.Add1_o[1] /
    CAVE_MachE_sil_test_B.Switch_d;

  /* Trigonometry: '<S273>/Trigonometric Function' */
  CAVE_MachE_sil_test_B.Beta = atan(CAVE_MachE_sil_test_B.Divide_f2);

  /* Reshape: '<S286>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_kx[0] = CAVE_MachE_sil_test_B.Sum_hm[0];
  CAVE_MachE_sil_test_B.Reshape1_kx[1] = CAVE_MachE_sil_test_B.Sum_hm[1];
  CAVE_MachE_sil_test_B.Reshape1_kx[2] = CAVE_MachE_sil_test_B.Sum_hm[2];

  /* Product: '<S286>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_f[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_kx[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_kx[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_kx[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_d4[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_d4[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_d4[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_d4[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S286>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_js[y] = CAVE_MachE_sil_test_B.Product_d4[y];
  }

  /* End of Product: '<S286>/Product' */

  /* Sum: '<S284>/Add4' */
  CAVE_MachE_sil_test_B.V_wb_n[0] = CAVE_MachE_sil_test_B.UnitConversion_j[0] +
    CAVE_MachE_sil_test_B.Reshape2_js[0];
  CAVE_MachE_sil_test_B.V_wb_n[1] = CAVE_MachE_sil_test_B.UnitConversion_j[1] +
    CAVE_MachE_sil_test_B.Reshape2_js[1];
  CAVE_MachE_sil_test_B.V_wb_n[2] = CAVE_MachE_sil_test_B.UnitConversion_j[2] +
    CAVE_MachE_sil_test_B.Reshape2_js[2];

  /* Reshape: '<S294>/Reshape1' */
  CAVE_MachE_sil_test_B.Reshape1_bj[0] = CAVE_MachE_sil_test_B.Sum_p[0];
  CAVE_MachE_sil_test_B.Reshape1_bj[1] = CAVE_MachE_sil_test_B.Sum_p[1];
  CAVE_MachE_sil_test_B.Reshape1_bj[2] = CAVE_MachE_sil_test_B.Sum_p[2];

  /* Product: '<S294>/Product' */
  memcpy(&u[0], &CAVE_MachE_sil_test_B.Transpose1_fn[0], 9U * sizeof(real_T));
  b_a[0] = CAVE_MachE_sil_test_B.Reshape1_bj[0];
  b_a[1] = CAVE_MachE_sil_test_B.Reshape1_bj[1];
  b_a[2] = CAVE_MachE_sil_test_B.Reshape1_bj[2];
  for (y = 0; y < 3; y++) {
    CAVE_MachE_sil_test_B.Product_dl[y] = 0.0;
    CAVE_MachE_sil_test_B.Product_dl[y] += u[y] * b_a[0];
    CAVE_MachE_sil_test_B.Product_dl[y] += u[y + 3] * b_a[1];
    CAVE_MachE_sil_test_B.Product_dl[y] += u[y + 6] * b_a[2];

    /* Reshape: '<S294>/Reshape2' */
    CAVE_MachE_sil_test_B.Reshape2_n1[y] = CAVE_MachE_sil_test_B.Product_dl[y];
  }

  /* End of Product: '<S294>/Product' */

  /* Sum: '<S292>/Add4' */
  CAVE_MachE_sil_test_B.V_wb_j[0] = CAVE_MachE_sil_test_B.UnitConversion_j[0] +
    CAVE_MachE_sil_test_B.Reshape2_n1[0];
  CAVE_MachE_sil_test_B.V_wb_j[1] = CAVE_MachE_sil_test_B.UnitConversion_j[1] +
    CAVE_MachE_sil_test_B.Reshape2_n1[1];
  CAVE_MachE_sil_test_B.V_wb_j[2] = CAVE_MachE_sil_test_B.UnitConversion_j[2] +
    CAVE_MachE_sil_test_B.Reshape2_n1[2];

  /* Trigonometry: '<S302>/sincos' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[0];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_m[0] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_md[0] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[1];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_m[1] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_md[1] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.phithetapsi[2];
  Bias = sin(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 = cos
    (rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1);
  CAVE_MachE_sil_test_B.sincos_o1_m[2] = Bias;
  CAVE_MachE_sil_test_B.sincos_o2_md[2] =
    rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1;

  /* Fcn: '<S302>/thetadot' */
  CAVE_MachE_sil_test_B.thetadot_e = CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o2_md[0] - CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o1_m[0];

  /* Product: '<S308>/j x k' */
  CAVE_MachE_sil_test_B.jxk_e = CAVE_MachE_sil_test_B.thetadot_e *
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Fcn: '<S302>/psidot' */
  CAVE_MachE_sil_test_B.psidot_g = (CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o1_m[0] + CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o2_md[0]) / CAVE_MachE_sil_test_B.sincos_o2_md
    [1];

  /* Product: '<S308>/k x i' */
  CAVE_MachE_sil_test_B.kxi_ki = CAVE_MachE_sil_test_B.psidot_g *
    CAVE_MachE_sil_test_B.UnitConversion[0];

  /* Fcn: '<S302>/phidot' */
  CAVE_MachE_sil_test_B.phidot_m = (CAVE_MachE_sil_test_B.pqr[1] *
    CAVE_MachE_sil_test_B.sincos_o1_m[0] + CAVE_MachE_sil_test_B.pqr[2] *
    CAVE_MachE_sil_test_B.sincos_o2_md[0]) * (CAVE_MachE_sil_test_B.sincos_o1_m
    [1] / CAVE_MachE_sil_test_B.sincos_o2_md[1]) + CAVE_MachE_sil_test_B.pqr[0];

  /* Product: '<S308>/i x j' */
  CAVE_MachE_sil_test_B.ixj_d = CAVE_MachE_sil_test_B.phidot_m *
    CAVE_MachE_sil_test_B.UnitConversion[1];

  /* Product: '<S309>/k x j' */
  CAVE_MachE_sil_test_B.kxj_g = CAVE_MachE_sil_test_B.psidot_g *
    CAVE_MachE_sil_test_B.UnitConversion[1];

  /* Product: '<S309>/i x k' */
  CAVE_MachE_sil_test_B.ixk_k2 = CAVE_MachE_sil_test_B.phidot_m *
    CAVE_MachE_sil_test_B.UnitConversion[2];

  /* Product: '<S309>/j x i' */
  CAVE_MachE_sil_test_B.jxi_d = CAVE_MachE_sil_test_B.thetadot_e *
    CAVE_MachE_sil_test_B.UnitConversion[0];

  /* Sum: '<S303>/Sum' */
  CAVE_MachE_sil_test_B.Sum_e[0] = CAVE_MachE_sil_test_B.jxk_e -
    CAVE_MachE_sil_test_B.kxj_g;
  CAVE_MachE_sil_test_B.Sum_e[1] = CAVE_MachE_sil_test_B.kxi_ki -
    CAVE_MachE_sil_test_B.ixk_k2;
  CAVE_MachE_sil_test_B.Sum_e[2] = CAVE_MachE_sil_test_B.ixj_d -
    CAVE_MachE_sil_test_B.jxi_d;

  /* Sum: '<S226>/Add' */
  CAVE_MachE_sil_test_B.Add_hc[0] = CAVE_MachE_sil_test_B.Sum_e[0] +
    CAVE_MachE_sil_test_B.Sum_dx[0];
  CAVE_MachE_sil_test_B.Add_hc[1] = CAVE_MachE_sil_test_B.Sum_e[1] +
    CAVE_MachE_sil_test_B.Sum_dx[1];
  CAVE_MachE_sil_test_B.Add_hc[2] = CAVE_MachE_sil_test_B.Sum_e[2] +
    CAVE_MachE_sil_test_B.Sum_dx[2];

  /* Fcn: '<S305>/Fcn' */
  rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1 =
    CAVE_MachE_sil_test_B.UnitConversion[0] / 0.01;
  Bias = rt_powd_snf(rtb_ImpSel_InsertedFor_WheelAngles_at_outport_0_idx_1, 2.0);
  CAVE_MachE_sil_test_B.Fcn_c = 0.02 / (3.0 - Bias);

  /* RelationalOperator: '<S306>/Compare' incorporates:
   *  Constant: '<S306>/Constant'
   */
  Bias = -CAVE_MachE_sil_test_P.VehicleBody6DOF1_xdot_tol;
  CAVE_MachE_sil_test_B.Compare_lx = (CAVE_MachE_sil_test_B.UnitConversion[0] >=
    Bias);

  /* RelationalOperator: '<S307>/Compare' incorporates:
   *  Constant: '<S307>/Constant'
   */
  CAVE_MachE_sil_test_B.Compare_b = (CAVE_MachE_sil_test_B.UnitConversion[0] <=
    CAVE_MachE_sil_test_P.VehicleBody6DOF1_xdot_tol);

  /* Logic: '<S305>/Logical Operator' */
  CAVE_MachE_sil_test_B.LogicalOperator_j = (CAVE_MachE_sil_test_B.Compare_lx &&
    CAVE_MachE_sil_test_B.Compare_b);

  /* Abs: '<S305>/Abs' */
  CAVE_MachE_sil_test_B.Abs_h = fabs(CAVE_MachE_sil_test_B.UnitConversion[0]);

  /* Switch: '<S305>/Switch' */
  if (CAVE_MachE_sil_test_B.LogicalOperator_j) {
    CAVE_MachE_sil_test_B.Switch_nu = CAVE_MachE_sil_test_B.Fcn_c;
  } else {
    CAVE_MachE_sil_test_B.Switch_nu = CAVE_MachE_sil_test_B.Abs_h;
  }

  /* End of Switch: '<S305>/Switch' */

  /* Product: '<S301>/Divide' */
  CAVE_MachE_sil_test_B.Divide_g = CAVE_MachE_sil_test_B.UnitConversion[1] /
    CAVE_MachE_sil_test_B.Switch_nu;

  /* Trigonometry: '<S301>/Trigonometric Function' */
  CAVE_MachE_sil_test_B.Beta_j = atan(CAVE_MachE_sil_test_B.Divide_g);

  /* Integrator: '<S226>/Integrator' */
  CAVE_MachE_sil_test_B.Integrator_i[0] =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[0];
  CAVE_MachE_sil_test_B.Integrator_i[1] =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[1];
  CAVE_MachE_sil_test_B.Integrator_i[2] =
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[2];

  /* UnitConversion: '<S226>/Unit Conversion1' */
  /* Unit Conversion - from: m/s^2 to: m/s^2
     Expression: output = (1*input) + (0) */
  CAVE_MachE_sil_test_B.UnitConversion1[0] = CAVE_MachE_sil_test_B.Sum_dx[0];
  CAVE_MachE_sil_test_B.UnitConversion1[1] = CAVE_MachE_sil_test_B.Sum_dx[1];
  CAVE_MachE_sil_test_B.UnitConversion1[2] = CAVE_MachE_sil_test_B.Sum_dx[2];

  /* UnitConversion: '<S226>/Unit Conversion3' */
  /* Unit Conversion - from: m/s^2 to: gn
     Expression: output = (0.101972*input) + (0) */
  CAVE_MachE_sil_test_B.UnitConversion3[0] = 0.10197162129779282 *
    CAVE_MachE_sil_test_B.Add_hc[0];
  CAVE_MachE_sil_test_B.UnitConversion3[1] = 0.10197162129779282 *
    CAVE_MachE_sil_test_B.Add_hc[1];
  CAVE_MachE_sil_test_B.UnitConversion3[2] = 0.10197162129779282 *
    CAVE_MachE_sil_test_B.Add_hc[2];

  /* Selector: '<S227>/Selector4' incorporates:
   *  Selector: '<S227>/Selector5'
   *  Selector: '<S227>/Selector6'
   */
  memcpy(&CAVE_MachE_sil_test_B.VectorConcatenate2_k[3],
         &CAVE_MachE_sil_test_B.Reshape_c[3], sizeof(real_T) << 3U);

  /* Selector: '<S227>/Selector6' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_k[11] =
    CAVE_MachE_sil_test_B.Reshape_c[11];

  /* Selector: '<S227>/Selector7' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_k[0] =
    CAVE_MachE_sil_test_B.Reshape_c[0];
  CAVE_MachE_sil_test_B.VectorConcatenate2_k[1] =
    CAVE_MachE_sil_test_B.Reshape_c[1];
  CAVE_MachE_sil_test_B.VectorConcatenate2_k[2] =
    CAVE_MachE_sil_test_B.Reshape_c[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate1' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_l[0] = CAVE_MachE_sil_test_B.Add_b3[1];

  /* SignalConversion generated from: '<S127>/Vector Concatenate1' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_l[1] = CAVE_MachE_sil_test_B.Add_kn[1];

  /* SignalConversion generated from: '<S127>/Vector Concatenate1' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_l[2] = CAVE_MachE_sil_test_B.Add_cn[1];

  /* SignalConversion generated from: '<S127>/Vector Concatenate1' */
  CAVE_MachE_sil_test_B.VectorConcatenate1_l[3] = CAVE_MachE_sil_test_B.Add_f[1];

  /* SignalConversion generated from: '<S127>/Vector Concatenate2' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_bt[0] = CAVE_MachE_sil_test_B.Add_b3
    [0];

  /* SignalConversion generated from: '<S127>/Vector Concatenate2' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_bt[1] = CAVE_MachE_sil_test_B.Add_kn
    [0];

  /* SignalConversion generated from: '<S127>/Vector Concatenate2' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_bt[2] = CAVE_MachE_sil_test_B.Add_cn
    [0];

  /* SignalConversion generated from: '<S127>/Vector Concatenate2' */
  CAVE_MachE_sil_test_B.VectorConcatenate2_bt[3] = CAVE_MachE_sil_test_B.Add_f[0];

  /* SignalConversion generated from: '<S127>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_a[0] = CAVE_MachE_sil_test_B.Add_b3[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_a[1] = CAVE_MachE_sil_test_B.Add_kn[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_a[2] = CAVE_MachE_sil_test_B.Add_cn[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate3' */
  CAVE_MachE_sil_test_B.VectorConcatenate3_a[3] = CAVE_MachE_sil_test_B.Add_f[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate4' */
  CAVE_MachE_sil_test_B.VectorConcatenate4[0] = CAVE_MachE_sil_test_B.Selector1
    [2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate4' */
  CAVE_MachE_sil_test_B.VectorConcatenate4[1] =
    CAVE_MachE_sil_test_B.Selector1_h[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate4' */
  CAVE_MachE_sil_test_B.VectorConcatenate4[2] =
    CAVE_MachE_sil_test_B.Selector1_c[2];

  /* SignalConversion generated from: '<S127>/Vector Concatenate4' */
  CAVE_MachE_sil_test_B.VectorConcatenate4[3] =
    CAVE_MachE_sil_test_B.Selector1_l[2];

  /* Sum: '<S127>/Subtract' */
  CAVE_MachE_sil_test_B.Subtract_p[0] =
    (CAVE_MachE_sil_test_B.VectorConcatenate3_a[0] -
     CAVE_MachE_sil_test_B.VectorConcatenate4[0]) - CAVE_MachE_sil_test_B.z[0];
  CAVE_MachE_sil_test_B.Subtract_p[1] =
    (CAVE_MachE_sil_test_B.VectorConcatenate3_a[1] -
     CAVE_MachE_sil_test_B.VectorConcatenate4[1]) - CAVE_MachE_sil_test_B.z[1];
  CAVE_MachE_sil_test_B.Subtract_p[2] =
    (CAVE_MachE_sil_test_B.VectorConcatenate3_a[2] -
     CAVE_MachE_sil_test_B.VectorConcatenate4[2]) - CAVE_MachE_sil_test_B.z[2];
  CAVE_MachE_sil_test_B.Subtract_p[3] =
    (CAVE_MachE_sil_test_B.VectorConcatenate3_a[3] -
     CAVE_MachE_sil_test_B.VectorConcatenate4[3]) - CAVE_MachE_sil_test_B.z[3];

  /* Selector: '<S128>/Selector' */
  CAVE_MachE_sil_test_B.Selector_b[0] = CAVE_MachE_sil_test_B.F[2];
  CAVE_MachE_sil_test_B.Selector_b[1] = CAVE_MachE_sil_test_B.F[5];
  CAVE_MachE_sil_test_B.Selector_b[2] = CAVE_MachE_sil_test_B.F[8];
  CAVE_MachE_sil_test_B.Selector_b[3] = CAVE_MachE_sil_test_B.F[11];

  /* Reshape: '<S128>/Reshape2' */
  CAVE_MachE_sil_test_B.Reshape2_o[0] = CAVE_MachE_sil_test_B.Selector_b[0];
  CAVE_MachE_sil_test_B.Reshape2_o[1] = CAVE_MachE_sil_test_B.Selector_b[1];
  CAVE_MachE_sil_test_B.Reshape2_o[2] = CAVE_MachE_sil_test_B.Selector_b[2];
  CAVE_MachE_sil_test_B.Reshape2_o[3] = CAVE_MachE_sil_test_B.Selector_b[3];

  /* Sum: '<S310>/Sum' */
  CAVE_MachE_sil_test_B.Sum_dw[0] = CAVE_MachE_sil_test_B.Reshape2_o[0] -
    CAVE_MachE_sil_test_B.Integrator1[0];
  CAVE_MachE_sil_test_B.Sum_dw[1] = CAVE_MachE_sil_test_B.Reshape2_o[1] -
    CAVE_MachE_sil_test_B.Integrator1[1];
  CAVE_MachE_sil_test_B.Sum_dw[2] = CAVE_MachE_sil_test_B.Reshape2_o[2] -
    CAVE_MachE_sil_test_B.Integrator1[2];
  CAVE_MachE_sil_test_B.Sum_dw[3] = CAVE_MachE_sil_test_B.Reshape2_o[3] -
    CAVE_MachE_sil_test_B.Integrator1[3];

  /* Product: '<S310>/Divide' incorporates:
   *  Constant: '<S310>/Constant'
   */
  CAVE_MachE_sil_test_B.Divide_ah[0] = CAVE_MachE_sil_test_B.Sum_dw[0] *
    CAVE_MachE_sil_test_P.ContLPF_wc;
  CAVE_MachE_sil_test_B.Divide_ah[1] = CAVE_MachE_sil_test_B.Sum_dw[1] *
    CAVE_MachE_sil_test_P.ContLPF_wc;
  CAVE_MachE_sil_test_B.Divide_ah[2] = CAVE_MachE_sil_test_B.Sum_dw[2] *
    CAVE_MachE_sil_test_P.ContLPF_wc;
  CAVE_MachE_sil_test_B.Divide_ah[3] = CAVE_MachE_sil_test_B.Sum_dw[3] *
    CAVE_MachE_sil_test_P.ContLPF_wc;

  /* Sum: '<S311>/Sum' */
  for (y = 0; y < 12; y++) {
    CAVE_MachE_sil_test_B.Sum_ja[y] = CAVE_MachE_sil_test_B.Ang[y] -
      CAVE_MachE_sil_test_B.Integrator1_d[y];
  }

  /* End of Sum: '<S311>/Sum' */

  /* Product: '<S311>/Divide' incorporates:
   *  Constant: '<S311>/Constant'
   */
  for (y = 0; y < 12; y++) {
    CAVE_MachE_sil_test_B.Divide_d4[y] = CAVE_MachE_sil_test_B.Sum_ja[y] *
      CAVE_MachE_sil_test_P.ContLPF1_wc;
  }

  /* End of Product: '<S311>/Divide' */

  /* SignalConversion generated from: '<S313>/Vector Concatenate5' */
  CAVE_MachE_sil_test_B.VectorConcatenate5[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[0];
  CAVE_MachE_sil_test_B.VectorConcatenate5[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[1];

  /* SignalConversion generated from: '<S313>/Vector Concatenate5' */
  CAVE_MachE_sil_test_B.VectorConcatenate5[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[0];
  CAVE_MachE_sil_test_B.VectorConcatenate5[3] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[1];

  /* SignalConversion generated from: '<S313>/Vector Concatenate6' */
  CAVE_MachE_sil_test_B.VectorConcatenate6[0] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[2];
  CAVE_MachE_sil_test_B.VectorConcatenate6[1] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[3];

  /* SignalConversion generated from: '<S313>/Vector Concatenate6' */
  CAVE_MachE_sil_test_B.VectorConcatenate6[2] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[2];
  CAVE_MachE_sil_test_B.VectorConcatenate6[3] =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fy_at_inport_0[3];

  /* Reshape: '<S313>/Reshape6' */
  CAVE_MachE_sil_test_B.Reshape6_ll[0] =
    CAVE_MachE_sil_test_B.VectorConcatenate_h[0];
  CAVE_MachE_sil_test_B.Reshape6_ll[1] =
    CAVE_MachE_sil_test_B.VectorConcatenate_h[1];
  CAVE_MachE_sil_test_B.Reshape6_ll[2] =
    CAVE_MachE_sil_test_B.VectorConcatenate_h[2];
  CAVE_MachE_sil_test_B.Reshape6_ll[3] =
    CAVE_MachE_sil_test_B.VectorConcatenate_h[3];

  /* Reshape: '<S317>/Reshape' */
  CAVE_MachE_sil_test_B.Reshape_h0[0] = CAVE_MachE_sil_test_B.z[0];
  CAVE_MachE_sil_test_B.Reshape_h0[1] = CAVE_MachE_sil_test_B.z[1];
  CAVE_MachE_sil_test_B.Reshape_h0[2] = CAVE_MachE_sil_test_B.z[2];
  CAVE_MachE_sil_test_B.Reshape_h0[3] = CAVE_MachE_sil_test_B.z[3];

  /* SignalConversion: '<S343>/Signal Copy' */
  CAVE_MachE_sil_test_B.SignalCopy =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput.FzTire;

  /* Product: '<S347>/Product2' */
  CAVE_MachE_sil_test_B.Product2_mt = CAVE_MachE_sil_test_B.sf_LockUp.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Re;

  /* Sum: '<S347>/Add1' */
  CAVE_MachE_sil_test_B.Add1_cq =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[0] -
    CAVE_MachE_sil_test_B.Product2_mt;

  /* Abs: '<S347>/Abs' */
  CAVE_MachE_sil_test_B.Abs_e = fabs(CAVE_MachE_sil_test_B.Add1_cq);

  /* Sum: '<S347>/Add' */
  CAVE_MachE_sil_test_B.Add_j = CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Fx
    - CAVE_MachE_sil_test_B.Integrator_a;

  /* DeadZone: '<S347>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_e >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone = CAVE_MachE_sil_test_B.Abs_e -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_e >= CAVE_MachE_sil_test_P.DeadZone_Start)
  {
    CAVE_MachE_sil_test_B.DeadZone = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone = CAVE_MachE_sil_test_B.Abs_e -
      CAVE_MachE_sil_test_P.DeadZone_Start;
  }

  /* End of DeadZone: '<S347>/Dead Zone' */

  /* Saturate: '<S347>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_g) {
    CAVE_MachE_sil_test_B.Saturation1_h =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_g;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_d) {
    CAVE_MachE_sil_test_B.Saturation1_h =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_d;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_h =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x;
  }

  /* End of Saturate: '<S347>/Saturation1' */

  /* Product: '<S347>/Product3' */
  CAVE_MachE_sil_test_B.Product3_es = CAVE_MachE_sil_test_B.DeadZone /
    CAVE_MachE_sil_test_B.Saturation1_h;

  /* Saturate: '<S347>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_es >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_h4) {
    CAVE_MachE_sil_test_B.Saturation_lr =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_h4;
  } else if (CAVE_MachE_sil_test_B.Product3_es <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_oj) {
    CAVE_MachE_sil_test_B.Saturation_lr =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_oj;
  } else {
    CAVE_MachE_sil_test_B.Saturation_lr = CAVE_MachE_sil_test_B.Product3_es;
  }

  /* End of Saturate: '<S347>/Saturation' */

  /* Product: '<S347>/Product1' */
  CAVE_MachE_sil_test_B.Fdot = CAVE_MachE_sil_test_B.Saturation_lr *
    CAVE_MachE_sil_test_B.Add_j;

  /* Product: '<S348>/Product2' */
  CAVE_MachE_sil_test_B.Product2_j = CAVE_MachE_sil_test_B.sf_LockUp.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Re;

  /* Sum: '<S348>/Add1' */
  CAVE_MachE_sil_test_B.Add1_k4 =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[0] -
    CAVE_MachE_sil_test_B.Product2_j;

  /* Abs: '<S348>/Abs' */
  CAVE_MachE_sil_test_B.Abs_g = fabs(CAVE_MachE_sil_test_B.Add1_k4);

  /* Sum: '<S348>/Add' */
  CAVE_MachE_sil_test_B.Add_l = CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Fy
    - CAVE_MachE_sil_test_B.Integrator_fu;

  /* DeadZone: '<S348>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_g >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_h = CAVE_MachE_sil_test_B.Abs_g -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_g >=
             CAVE_MachE_sil_test_P.DeadZone_Start_m) {
    CAVE_MachE_sil_test_B.DeadZone_h = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_h = CAVE_MachE_sil_test_B.Abs_g -
      CAVE_MachE_sil_test_P.DeadZone_Start_m;
  }

  /* End of DeadZone: '<S348>/Dead Zone' */

  /* Saturate: '<S348>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_y >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_j) {
    CAVE_MachE_sil_test_B.Saturation1_c =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_j;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_y <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_d1) {
    CAVE_MachE_sil_test_B.Saturation1_c =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_d1;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_c =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_y;
  }

  /* End of Saturate: '<S348>/Saturation1' */

  /* Product: '<S348>/Product3' */
  CAVE_MachE_sil_test_B.Product3_fx = CAVE_MachE_sil_test_B.DeadZone_h /
    CAVE_MachE_sil_test_B.Saturation1_c;

  /* Saturate: '<S348>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_fx >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_o) {
    CAVE_MachE_sil_test_B.Saturation_g5 =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_o;
  } else if (CAVE_MachE_sil_test_B.Product3_fx <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_dq) {
    CAVE_MachE_sil_test_B.Saturation_g5 =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_dq;
  } else {
    CAVE_MachE_sil_test_B.Saturation_g5 = CAVE_MachE_sil_test_B.Product3_fx;
  }

  /* End of Saturate: '<S348>/Saturation' */

  /* Product: '<S348>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_a = CAVE_MachE_sil_test_B.Saturation_g5 *
    CAVE_MachE_sil_test_B.Add_l;

  /* Product: '<S350>/Product2' */
  CAVE_MachE_sil_test_B.Product2_ht = CAVE_MachE_sil_test_B.sf_LockUp.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Re;

  /* Sum: '<S350>/Add1' */
  CAVE_MachE_sil_test_B.Add1_al =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[0] -
    CAVE_MachE_sil_test_B.Product2_ht;

  /* Abs: '<S350>/Abs' */
  CAVE_MachE_sil_test_B.Abs_n = fabs(CAVE_MachE_sil_test_B.Add1_al);

  /* Sum: '<S350>/Add' */
  CAVE_MachE_sil_test_B.Add_m = CAVE_MachE_sil_test_B.sf_MagicTireConstInput.My
    - CAVE_MachE_sil_test_B.Integrator_k;

  /* DeadZone: '<S350>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_n >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_j = CAVE_MachE_sil_test_B.Abs_n -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_n >=
             CAVE_MachE_sil_test_P.DeadZone_Start_l) {
    CAVE_MachE_sil_test_B.DeadZone_j = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_j = CAVE_MachE_sil_test_B.Abs_n -
      CAVE_MachE_sil_test_P.DeadZone_Start_l;
  }

  /* End of DeadZone: '<S350>/Dead Zone' */

  /* Saturate: '<S350>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_p) {
    CAVE_MachE_sil_test_B.Saturation1_bf =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_p;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_m) {
    CAVE_MachE_sil_test_B.Saturation1_bf =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_m;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_bf =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput.sig_x;
  }

  /* End of Saturate: '<S350>/Saturation1' */

  /* Product: '<S350>/Product3' */
  CAVE_MachE_sil_test_B.Product3_ix = CAVE_MachE_sil_test_B.DeadZone_j /
    CAVE_MachE_sil_test_B.Saturation1_bf;

  /* Saturate: '<S350>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_ix >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_f) {
    CAVE_MachE_sil_test_B.Saturation_dk =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_f;
  } else if (CAVE_MachE_sil_test_B.Product3_ix <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_d4) {
    CAVE_MachE_sil_test_B.Saturation_dk =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_d4;
  } else {
    CAVE_MachE_sil_test_B.Saturation_dk = CAVE_MachE_sil_test_B.Product3_ix;
  }

  /* End of Saturate: '<S350>/Saturation' */

  /* Product: '<S350>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_h = CAVE_MachE_sil_test_B.Saturation_dk *
    CAVE_MachE_sil_test_B.Add_m;

  /* Switch: '<S345>/Switch' incorporates:
   *  Constant: '<S345>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6 > CAVE_MachE_sil_test_P.Switch_Threshold_bx) {
    CAVE_MachE_sil_test_B.Switch_a =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2;
  } else {
    CAVE_MachE_sil_test_B.Switch_a = CAVE_MachE_sil_test_P.Constant_Value_jx1;
  }

  /* End of Switch: '<S345>/Switch' */

  /* Gain: '<S345>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_m =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_VERTICAL_DAMPING *
    CAVE_MachE_sil_test_B.Switch_a;

  /* Sum: '<S345>/Sum2' incorporates:
   *  Constant: '<S345>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_GRAVITY *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_MASS;
  CAVE_MachE_sil_test_B.Sum2_g = ((CAVE_MachE_sil_test_B.SignalCopy + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[0]) - CAVE_MachE_sil_test_B.Gain2_m;

  /* Gain: '<S345>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_MASS;
  CAVE_MachE_sil_test_B.Gain1_ge = Bias * CAVE_MachE_sil_test_B.Sum2_g;

  /* SignalConversion: '<S368>/Signal Copy' */
  CAVE_MachE_sil_test_B.SignalCopy_e =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.FzTire;

  /* Product: '<S372>/Product2' */
  CAVE_MachE_sil_test_B.Product2_k = CAVE_MachE_sil_test_B.sf_LockUp_n.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Re;

  /* Sum: '<S372>/Add1' */
  CAVE_MachE_sil_test_B.Add1_k1 =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[1] -
    CAVE_MachE_sil_test_B.Product2_k;

  /* Abs: '<S372>/Abs' */
  CAVE_MachE_sil_test_B.Abs_k = fabs(CAVE_MachE_sil_test_B.Add1_k1);

  /* Sum: '<S372>/Add' */
  CAVE_MachE_sil_test_B.Add_bv =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Fx -
    CAVE_MachE_sil_test_B.Integrator_l;

  /* DeadZone: '<S372>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_k >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_f = CAVE_MachE_sil_test_B.Abs_k -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_k >=
             CAVE_MachE_sil_test_P.DeadZone_Start_k) {
    CAVE_MachE_sil_test_B.DeadZone_f = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_f = CAVE_MachE_sil_test_B.Abs_k -
      CAVE_MachE_sil_test_P.DeadZone_Start_k;
  }

  /* End of DeadZone: '<S372>/Dead Zone' */

  /* Saturate: '<S372>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_k) {
    CAVE_MachE_sil_test_B.Saturation1_fy =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_k;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_dd) {
    CAVE_MachE_sil_test_B.Saturation1_fy =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_dd;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_fy =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x;
  }

  /* End of Saturate: '<S372>/Saturation1' */

  /* Product: '<S372>/Product3' */
  CAVE_MachE_sil_test_B.Product3_oh = CAVE_MachE_sil_test_B.DeadZone_f /
    CAVE_MachE_sil_test_B.Saturation1_fy;

  /* Saturate: '<S372>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_oh >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ig) {
    CAVE_MachE_sil_test_B.Saturation_jp =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ig;
  } else if (CAVE_MachE_sil_test_B.Product3_oh <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_f1) {
    CAVE_MachE_sil_test_B.Saturation_jp =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_f1;
  } else {
    CAVE_MachE_sil_test_B.Saturation_jp = CAVE_MachE_sil_test_B.Product3_oh;
  }

  /* End of Saturate: '<S372>/Saturation' */

  /* Product: '<S372>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_f = CAVE_MachE_sil_test_B.Saturation_jp *
    CAVE_MachE_sil_test_B.Add_bv;

  /* Product: '<S373>/Product2' */
  CAVE_MachE_sil_test_B.Product2_o = CAVE_MachE_sil_test_B.sf_LockUp_n.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Re;

  /* Sum: '<S373>/Add1' */
  CAVE_MachE_sil_test_B.Add1_nb =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[1] -
    CAVE_MachE_sil_test_B.Product2_o;

  /* Abs: '<S373>/Abs' */
  CAVE_MachE_sil_test_B.Abs_o = fabs(CAVE_MachE_sil_test_B.Add1_nb);

  /* Sum: '<S373>/Add' */
  CAVE_MachE_sil_test_B.Add_mi =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Fy -
    CAVE_MachE_sil_test_B.Integrator_n;

  /* DeadZone: '<S373>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_o >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_b = CAVE_MachE_sil_test_B.Abs_o -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_o >=
             CAVE_MachE_sil_test_P.DeadZone_Start_my) {
    CAVE_MachE_sil_test_B.DeadZone_b = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_b = CAVE_MachE_sil_test_B.Abs_o -
      CAVE_MachE_sil_test_P.DeadZone_Start_my;
  }

  /* End of DeadZone: '<S373>/Dead Zone' */

  /* Saturate: '<S373>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_y >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_d) {
    CAVE_MachE_sil_test_B.Saturation1_b1 =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_d;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_y <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_n) {
    CAVE_MachE_sil_test_B.Saturation1_b1 =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_n;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_b1 =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_y;
  }

  /* End of Saturate: '<S373>/Saturation1' */

  /* Product: '<S373>/Product3' */
  CAVE_MachE_sil_test_B.Product3_a = CAVE_MachE_sil_test_B.DeadZone_b /
    CAVE_MachE_sil_test_B.Saturation1_b1;

  /* Saturate: '<S373>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_a >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_aq) {
    CAVE_MachE_sil_test_B.Saturation_p =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_aq;
  } else if (CAVE_MachE_sil_test_B.Product3_a <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_g) {
    CAVE_MachE_sil_test_B.Saturation_p =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_g;
  } else {
    CAVE_MachE_sil_test_B.Saturation_p = CAVE_MachE_sil_test_B.Product3_a;
  }

  /* End of Saturate: '<S373>/Saturation' */

  /* Product: '<S373>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_l = CAVE_MachE_sil_test_B.Saturation_p *
    CAVE_MachE_sil_test_B.Add_mi;

  /* Product: '<S375>/Product2' */
  CAVE_MachE_sil_test_B.Product2_kj = CAVE_MachE_sil_test_B.sf_LockUp_n.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Re;

  /* Sum: '<S375>/Add1' */
  CAVE_MachE_sil_test_B.Add1_bl =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[1] -
    CAVE_MachE_sil_test_B.Product2_kj;

  /* Abs: '<S375>/Abs' */
  CAVE_MachE_sil_test_B.Abs_a = fabs(CAVE_MachE_sil_test_B.Add1_bl);

  /* Sum: '<S375>/Add' */
  CAVE_MachE_sil_test_B.Add_fx =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.My -
    CAVE_MachE_sil_test_B.Integrator_f;

  /* DeadZone: '<S375>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_a >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_p = CAVE_MachE_sil_test_B.Abs_a -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_a >=
             CAVE_MachE_sil_test_P.DeadZone_Start_g) {
    CAVE_MachE_sil_test_B.DeadZone_p = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_p = CAVE_MachE_sil_test_B.Abs_a -
      CAVE_MachE_sil_test_P.DeadZone_Start_g;
  }

  /* End of DeadZone: '<S375>/Dead Zone' */

  /* Saturate: '<S375>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_c) {
    CAVE_MachE_sil_test_B.Saturation1_k =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_c;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_dh) {
    CAVE_MachE_sil_test_B.Saturation1_k =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_dh;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_k =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.sig_x;
  }

  /* End of Saturate: '<S375>/Saturation1' */

  /* Product: '<S375>/Product3' */
  CAVE_MachE_sil_test_B.Product3_j1 = CAVE_MachE_sil_test_B.DeadZone_p /
    CAVE_MachE_sil_test_B.Saturation1_k;

  /* Saturate: '<S375>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_j1 >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_hc) {
    CAVE_MachE_sil_test_B.Saturation_fp =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_hc;
  } else if (CAVE_MachE_sil_test_B.Product3_j1 <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_l) {
    CAVE_MachE_sil_test_B.Saturation_fp =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_l;
  } else {
    CAVE_MachE_sil_test_B.Saturation_fp = CAVE_MachE_sil_test_B.Product3_j1;
  }

  /* End of Saturate: '<S375>/Saturation' */

  /* Product: '<S375>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_o = CAVE_MachE_sil_test_B.Saturation_fp *
    CAVE_MachE_sil_test_B.Add_fx;

  /* Switch: '<S370>/Switch' incorporates:
   *  Constant: '<S370>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_p > CAVE_MachE_sil_test_P.Switch_Threshold_bq)
  {
    CAVE_MachE_sil_test_B.Switch_o0 =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_n;
  } else {
    CAVE_MachE_sil_test_B.Switch_o0 = CAVE_MachE_sil_test_P.Constant_Value_a2;
  }

  /* End of Switch: '<S370>/Switch' */

  /* Gain: '<S370>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_o =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_VERTICAL_DAMPING *
    CAVE_MachE_sil_test_B.Switch_o0;

  /* Sum: '<S370>/Sum2' incorporates:
   *  Constant: '<S370>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_GRAVITY *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_MASS;
  CAVE_MachE_sil_test_B.Sum2_d = ((CAVE_MachE_sil_test_B.SignalCopy_e + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[1]) - CAVE_MachE_sil_test_B.Gain2_o;

  /* Gain: '<S370>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_MASS;
  CAVE_MachE_sil_test_B.Gain1_p = Bias * CAVE_MachE_sil_test_B.Sum2_d;

  /* SignalConversion: '<S393>/Signal Copy' */
  CAVE_MachE_sil_test_B.SignalCopy_g =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.FzTire;

  /* Product: '<S397>/Product2' */
  CAVE_MachE_sil_test_B.Product2_ji = CAVE_MachE_sil_test_B.sf_LockUp_h.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Re;

  /* Sum: '<S397>/Add1' */
  CAVE_MachE_sil_test_B.Add1_oe =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[2] -
    CAVE_MachE_sil_test_B.Product2_ji;

  /* Abs: '<S397>/Abs' */
  CAVE_MachE_sil_test_B.Abs_kh = fabs(CAVE_MachE_sil_test_B.Add1_oe);

  /* Sum: '<S397>/Add' */
  CAVE_MachE_sil_test_B.Add_du =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Fx -
    CAVE_MachE_sil_test_B.Integrator_fm;

  /* DeadZone: '<S397>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_kh >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_d = CAVE_MachE_sil_test_B.Abs_kh -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_kh >=
             CAVE_MachE_sil_test_P.DeadZone_Start_d) {
    CAVE_MachE_sil_test_B.DeadZone_d = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_d = CAVE_MachE_sil_test_B.Abs_kh -
      CAVE_MachE_sil_test_P.DeadZone_Start_d;
  }

  /* End of DeadZone: '<S397>/Dead Zone' */

  /* Saturate: '<S397>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_pk) {
    CAVE_MachE_sil_test_B.Saturation1_dh =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_pk;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_h) {
    CAVE_MachE_sil_test_B.Saturation1_dh =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_h;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_dh =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x;
  }

  /* End of Saturate: '<S397>/Saturation1' */

  /* Product: '<S397>/Product3' */
  CAVE_MachE_sil_test_B.Product3_oq = CAVE_MachE_sil_test_B.DeadZone_d /
    CAVE_MachE_sil_test_B.Saturation1_dh;

  /* Saturate: '<S397>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_oq >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_co) {
    CAVE_MachE_sil_test_B.Saturation_gr =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_co;
  } else if (CAVE_MachE_sil_test_B.Product3_oq <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_d1) {
    CAVE_MachE_sil_test_B.Saturation_gr =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_d1;
  } else {
    CAVE_MachE_sil_test_B.Saturation_gr = CAVE_MachE_sil_test_B.Product3_oq;
  }

  /* End of Saturate: '<S397>/Saturation' */

  /* Product: '<S397>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_d = CAVE_MachE_sil_test_B.Saturation_gr *
    CAVE_MachE_sil_test_B.Add_du;

  /* Product: '<S398>/Product2' */
  CAVE_MachE_sil_test_B.Product2_ii = CAVE_MachE_sil_test_B.sf_LockUp_h.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Re;

  /* Sum: '<S398>/Add1' */
  CAVE_MachE_sil_test_B.Add1_gr =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[2] -
    CAVE_MachE_sil_test_B.Product2_ii;

  /* Abs: '<S398>/Abs' */
  CAVE_MachE_sil_test_B.Abs_hy = fabs(CAVE_MachE_sil_test_B.Add1_gr);

  /* Sum: '<S398>/Add' */
  CAVE_MachE_sil_test_B.Add_dy =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Fy -
    CAVE_MachE_sil_test_B.Integrator_e;

  /* DeadZone: '<S398>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_hy >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_e = CAVE_MachE_sil_test_B.Abs_hy -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_hy >=
             CAVE_MachE_sil_test_P.DeadZone_Start_h) {
    CAVE_MachE_sil_test_B.DeadZone_e = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_e = CAVE_MachE_sil_test_B.Abs_hy -
      CAVE_MachE_sil_test_P.DeadZone_Start_h;
  }

  /* End of DeadZone: '<S398>/Dead Zone' */

  /* Saturate: '<S398>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_y >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_b) {
    CAVE_MachE_sil_test_B.Saturation1_ke =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_b;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_y <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_dq) {
    CAVE_MachE_sil_test_B.Saturation1_ke =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_dq;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_ke =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_y;
  }

  /* End of Saturate: '<S398>/Saturation1' */

  /* Product: '<S398>/Product3' */
  CAVE_MachE_sil_test_B.Product3_ol = CAVE_MachE_sil_test_B.DeadZone_e /
    CAVE_MachE_sil_test_B.Saturation1_ke;

  /* Saturate: '<S398>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_ol >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ak) {
    CAVE_MachE_sil_test_B.Saturation_ku =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ak;
  } else if (CAVE_MachE_sil_test_B.Product3_ol <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_jc) {
    CAVE_MachE_sil_test_B.Saturation_ku =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_jc;
  } else {
    CAVE_MachE_sil_test_B.Saturation_ku = CAVE_MachE_sil_test_B.Product3_ol;
  }

  /* End of Saturate: '<S398>/Saturation' */

  /* Product: '<S398>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_b = CAVE_MachE_sil_test_B.Saturation_ku *
    CAVE_MachE_sil_test_B.Add_dy;

  /* Product: '<S400>/Product2' */
  CAVE_MachE_sil_test_B.Product2_f = CAVE_MachE_sil_test_B.sf_LockUp_h.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Re;

  /* Sum: '<S400>/Add1' */
  CAVE_MachE_sil_test_B.Add1_ln =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[2] -
    CAVE_MachE_sil_test_B.Product2_f;

  /* Abs: '<S400>/Abs' */
  CAVE_MachE_sil_test_B.Abs_fr = fabs(CAVE_MachE_sil_test_B.Add1_ln);

  /* Sum: '<S400>/Add' */
  CAVE_MachE_sil_test_B.Add_o =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.My -
    CAVE_MachE_sil_test_B.Integrator_ah;

  /* DeadZone: '<S400>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_fr >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_a = CAVE_MachE_sil_test_B.Abs_fr -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_fr >=
             CAVE_MachE_sil_test_P.DeadZone_Start_c) {
    CAVE_MachE_sil_test_B.DeadZone_a = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_a = CAVE_MachE_sil_test_B.Abs_fr -
      CAVE_MachE_sil_test_P.DeadZone_Start_c;
  }

  /* End of DeadZone: '<S400>/Dead Zone' */

  /* Saturate: '<S400>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_ne) {
    CAVE_MachE_sil_test_B.Saturation1_hj =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_ne;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_b) {
    CAVE_MachE_sil_test_B.Saturation1_hj =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_b;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_hj =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.sig_x;
  }

  /* End of Saturate: '<S400>/Saturation1' */

  /* Product: '<S400>/Product3' */
  CAVE_MachE_sil_test_B.Product3_lt = CAVE_MachE_sil_test_B.DeadZone_a /
    CAVE_MachE_sil_test_B.Saturation1_hj;

  /* Saturate: '<S400>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_lt >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_p) {
    CAVE_MachE_sil_test_B.Saturation_gl =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_p;
  } else if (CAVE_MachE_sil_test_B.Product3_lt <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_jx) {
    CAVE_MachE_sil_test_B.Saturation_gl =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_jx;
  } else {
    CAVE_MachE_sil_test_B.Saturation_gl = CAVE_MachE_sil_test_B.Product3_lt;
  }

  /* End of Saturate: '<S400>/Saturation' */

  /* Product: '<S400>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_c = CAVE_MachE_sil_test_B.Saturation_gl *
    CAVE_MachE_sil_test_B.Add_o;

  /* Switch: '<S395>/Switch' incorporates:
   *  Constant: '<S395>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_f > CAVE_MachE_sil_test_P.Switch_Threshold_a) {
    CAVE_MachE_sil_test_B.Switch_bz =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_p;
  } else {
    CAVE_MachE_sil_test_B.Switch_bz = CAVE_MachE_sil_test_P.Constant_Value_ex;
  }

  /* End of Switch: '<S395>/Switch' */

  /* Gain: '<S395>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_h =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_VERTICAL_DAMPING *
    CAVE_MachE_sil_test_B.Switch_bz;

  /* Sum: '<S395>/Sum2' incorporates:
   *  Constant: '<S395>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_GRAVITY *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_MASS;
  CAVE_MachE_sil_test_B.Sum2_dp = ((CAVE_MachE_sil_test_B.SignalCopy_g + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[2]) - CAVE_MachE_sil_test_B.Gain2_h;

  /* Gain: '<S395>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_MASS;
  CAVE_MachE_sil_test_B.Gain1_o = Bias * CAVE_MachE_sil_test_B.Sum2_dp;

  /* SignalConversion: '<S418>/Signal Copy' */
  CAVE_MachE_sil_test_B.SignalCopy_k =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.FzTire;

  /* Product: '<S422>/Product2' */
  CAVE_MachE_sil_test_B.Product2_b = CAVE_MachE_sil_test_B.sf_LockUp_c.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Re;

  /* Sum: '<S422>/Add1' */
  CAVE_MachE_sil_test_B.Add1_e =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[3] -
    CAVE_MachE_sil_test_B.Product2_b;

  /* Abs: '<S422>/Abs' */
  CAVE_MachE_sil_test_B.Abs_j = fabs(CAVE_MachE_sil_test_B.Add1_e);

  /* Sum: '<S422>/Add' */
  CAVE_MachE_sil_test_B.Add_lr =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Fx -
    CAVE_MachE_sil_test_B.Integrator_b;

  /* DeadZone: '<S422>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_j >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_l = CAVE_MachE_sil_test_B.Abs_j -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_j >=
             CAVE_MachE_sil_test_P.DeadZone_Start_e) {
    CAVE_MachE_sil_test_B.DeadZone_l = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_l = CAVE_MachE_sil_test_B.Abs_j -
      CAVE_MachE_sil_test_P.DeadZone_Start_e;
  }

  /* End of DeadZone: '<S422>/Dead Zone' */

  /* Saturate: '<S422>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_kn) {
    CAVE_MachE_sil_test_B.Saturation1_kd =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_kn;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_na) {
    CAVE_MachE_sil_test_B.Saturation1_kd =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_na;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_kd =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x;
  }

  /* End of Saturate: '<S422>/Saturation1' */

  /* Product: '<S422>/Product3' */
  CAVE_MachE_sil_test_B.Product3_ip = CAVE_MachE_sil_test_B.DeadZone_l /
    CAVE_MachE_sil_test_B.Saturation1_kd;

  /* Saturate: '<S422>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_ip >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_nj) {
    CAVE_MachE_sil_test_B.Saturation_e =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_nj;
  } else if (CAVE_MachE_sil_test_B.Product3_ip <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_pi) {
    CAVE_MachE_sil_test_B.Saturation_e =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_pi;
  } else {
    CAVE_MachE_sil_test_B.Saturation_e = CAVE_MachE_sil_test_B.Product3_ip;
  }

  /* End of Saturate: '<S422>/Saturation' */

  /* Product: '<S422>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_m = CAVE_MachE_sil_test_B.Saturation_e *
    CAVE_MachE_sil_test_B.Add_lr;

  /* Product: '<S423>/Product2' */
  CAVE_MachE_sil_test_B.Product2_ip = CAVE_MachE_sil_test_B.sf_LockUp_c.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Re;

  /* Sum: '<S423>/Add1' */
  CAVE_MachE_sil_test_B.Add1_ok =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_ydotWheel_at_inport_0[3] -
    CAVE_MachE_sil_test_B.Product2_ip;

  /* Abs: '<S423>/Abs' */
  CAVE_MachE_sil_test_B.Abs_az = fabs(CAVE_MachE_sil_test_B.Add1_ok);

  /* Sum: '<S423>/Add' */
  CAVE_MachE_sil_test_B.Add_j5 =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Fy -
    CAVE_MachE_sil_test_B.Integrator_d;

  /* DeadZone: '<S423>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_az >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_bq = CAVE_MachE_sil_test_B.Abs_az -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_az >=
             CAVE_MachE_sil_test_P.DeadZone_Start_a) {
    CAVE_MachE_sil_test_B.DeadZone_bq = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_bq = CAVE_MachE_sil_test_B.Abs_az -
      CAVE_MachE_sil_test_P.DeadZone_Start_a;
  }

  /* End of DeadZone: '<S423>/Dead Zone' */

  /* Saturate: '<S423>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_y >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_a1) {
    CAVE_MachE_sil_test_B.Saturation1_o =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_a1;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_y <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_p) {
    CAVE_MachE_sil_test_B.Saturation1_o =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_p;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_o =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_y;
  }

  /* End of Saturate: '<S423>/Saturation1' */

  /* Product: '<S423>/Product3' */
  CAVE_MachE_sil_test_B.Product3_e2 = CAVE_MachE_sil_test_B.DeadZone_bq /
    CAVE_MachE_sil_test_B.Saturation1_o;

  /* Saturate: '<S423>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_e2 >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_dh) {
    CAVE_MachE_sil_test_B.Saturation_fl =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_dh;
  } else if (CAVE_MachE_sil_test_B.Product3_e2 <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_eje) {
    CAVE_MachE_sil_test_B.Saturation_fl =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_eje;
  } else {
    CAVE_MachE_sil_test_B.Saturation_fl = CAVE_MachE_sil_test_B.Product3_e2;
  }

  /* End of Saturate: '<S423>/Saturation' */

  /* Product: '<S423>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_g = CAVE_MachE_sil_test_B.Saturation_fl *
    CAVE_MachE_sil_test_B.Add_j5;

  /* Product: '<S425>/Product2' */
  CAVE_MachE_sil_test_B.Product2_ol = CAVE_MachE_sil_test_B.sf_LockUp_c.Omega *
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Re;

  /* Sum: '<S425>/Add1' */
  CAVE_MachE_sil_test_B.Add1_lr =
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_xdotWheel_at_inport_0[3] -
    CAVE_MachE_sil_test_B.Product2_ol;

  /* Abs: '<S425>/Abs' */
  CAVE_MachE_sil_test_B.Abs_fn = fabs(CAVE_MachE_sil_test_B.Add1_lr);

  /* Sum: '<S425>/Add' */
  CAVE_MachE_sil_test_B.Add_fb =
    CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.My -
    CAVE_MachE_sil_test_B.Integrator_p;

  /* DeadZone: '<S425>/Dead Zone' */
  if (CAVE_MachE_sil_test_B.Abs_fn >
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW) {
    CAVE_MachE_sil_test_B.DeadZone_i = CAVE_MachE_sil_test_B.Abs_fn -
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VXLOW;
  } else if (CAVE_MachE_sil_test_B.Abs_fn >=
             CAVE_MachE_sil_test_P.DeadZone_Start_b) {
    CAVE_MachE_sil_test_B.DeadZone_i = 0.0;
  } else {
    CAVE_MachE_sil_test_B.DeadZone_i = CAVE_MachE_sil_test_B.Abs_fn -
      CAVE_MachE_sil_test_P.DeadZone_Start_b;
  }

  /* End of DeadZone: '<S425>/Dead Zone' */

  /* Saturate: '<S425>/Saturation1' */
  if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x >
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_fc) {
    CAVE_MachE_sil_test_B.Saturation1_l =
      CAVE_MachE_sil_test_P.Saturation1_UpperSat_fc;
  } else if (CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x <
             CAVE_MachE_sil_test_P.Saturation1_LowerSat_dn) {
    CAVE_MachE_sil_test_B.Saturation1_l =
      CAVE_MachE_sil_test_P.Saturation1_LowerSat_dn;
  } else {
    CAVE_MachE_sil_test_B.Saturation1_l =
      CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.sig_x;
  }

  /* End of Saturate: '<S425>/Saturation1' */

  /* Product: '<S425>/Product3' */
  CAVE_MachE_sil_test_B.Product3_d = CAVE_MachE_sil_test_B.DeadZone_i /
    CAVE_MachE_sil_test_B.Saturation1_l;

  /* Saturate: '<S425>/Saturation' */
  if (CAVE_MachE_sil_test_B.Product3_d >
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ep) {
    CAVE_MachE_sil_test_B.Saturation_pu =
      CAVE_MachE_sil_test_P.Saturation_UpperSat_ep;
  } else if (CAVE_MachE_sil_test_B.Product3_d <
             CAVE_MachE_sil_test_P.Saturation_LowerSat_gb) {
    CAVE_MachE_sil_test_B.Saturation_pu =
      CAVE_MachE_sil_test_P.Saturation_LowerSat_gb;
  } else {
    CAVE_MachE_sil_test_B.Saturation_pu = CAVE_MachE_sil_test_B.Product3_d;
  }

  /* End of Saturate: '<S425>/Saturation' */

  /* Product: '<S425>/Product1' */
  CAVE_MachE_sil_test_B.Fdot_bw = CAVE_MachE_sil_test_B.Saturation_pu *
    CAVE_MachE_sil_test_B.Add_fb;

  /* Switch: '<S420>/Switch' incorporates:
   *  Constant: '<S420>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_o > CAVE_MachE_sil_test_P.Switch_Threshold_l) {
    CAVE_MachE_sil_test_B.Switch_i =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_h;
  } else {
    CAVE_MachE_sil_test_B.Switch_i = CAVE_MachE_sil_test_P.Constant_Value_im;
  }

  /* End of Switch: '<S420>/Switch' */

  /* Gain: '<S420>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_c =
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_VERTICAL_DAMPING *
    CAVE_MachE_sil_test_B.Switch_i;

  /* Sum: '<S420>/Sum2' incorporates:
   *  Constant: '<S420>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_GRAVITY *
    CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_MASS;
  CAVE_MachE_sil_test_B.Sum2_p = ((CAVE_MachE_sil_test_B.SignalCopy_k + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[3]) - CAVE_MachE_sil_test_B.Gain2_c;

  /* Gain: '<S420>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_MASS;
  CAVE_MachE_sil_test_B.Gain1_b = Bias * CAVE_MachE_sil_test_B.Sum2_p;

  /* SignalConversion generated from: '<S329>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ag[0] =
    CAVE_MachE_sil_test_B.Saturation_du[0];

  /* SignalConversion generated from: '<S329>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ag[1] =
    CAVE_MachE_sil_test_B.Saturation_du[1];

  /* SignalConversion generated from: '<S329>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ag[2] =
    CAVE_MachE_sil_test_B.Saturation_du[2];

  /* SignalConversion generated from: '<S329>/Vector Concatenate' */
  CAVE_MachE_sil_test_B.VectorConcatenate_ag[3] =
    CAVE_MachE_sil_test_B.Saturation_du[3];

  /* Switch: '<S334>/Switch' incorporates:
   *  Constant: '<S334>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_a > CAVE_MachE_sil_test_P.Switch_Threshold_k) {
    CAVE_MachE_sil_test_B.Switch_cu =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_b;
  } else {
    CAVE_MachE_sil_test_B.Switch_cu = CAVE_MachE_sil_test_P.Constant_Value_dr;
  }

  /* End of Switch: '<S334>/Switch' */

  /* Gain: '<S334>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_b =
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_bz *
    CAVE_MachE_sil_test_B.Switch_cu;

  /* Sum: '<S334>/Sum2' incorporates:
   *  Constant: '<S334>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_g *
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_m;
  CAVE_MachE_sil_test_B.Sum2_e = ((CAVE_MachE_sil_test_B.SignalCopy + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[0]) - CAVE_MachE_sil_test_B.Gain2_b;

  /* Gain: '<S334>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_m;
  CAVE_MachE_sil_test_B.Gain1_m = Bias * CAVE_MachE_sil_test_B.Sum2_e;

  /* Switch: '<S335>/Switch' incorporates:
   *  Constant: '<S335>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_k > CAVE_MachE_sil_test_P.Switch_Threshold_e) {
    CAVE_MachE_sil_test_B.Switch_e =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_g;
  } else {
    CAVE_MachE_sil_test_B.Switch_e = CAVE_MachE_sil_test_P.Constant_Value_ic;
  }

  /* End of Switch: '<S335>/Switch' */

  /* Gain: '<S335>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_g =
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_bz *
    CAVE_MachE_sil_test_B.Switch_e;

  /* Sum: '<S335>/Sum2' incorporates:
   *  Constant: '<S335>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_g *
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_m;
  CAVE_MachE_sil_test_B.Sum2_dd = ((CAVE_MachE_sil_test_B.SignalCopy_e + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[1]) - CAVE_MachE_sil_test_B.Gain2_g;

  /* Gain: '<S335>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_m;
  CAVE_MachE_sil_test_B.Gain1_na = Bias * CAVE_MachE_sil_test_B.Sum2_dd;

  /* Switch: '<S336>/Switch' incorporates:
   *  Constant: '<S336>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_k0 > CAVE_MachE_sil_test_P.Switch_Threshold_ci)
  {
    CAVE_MachE_sil_test_B.Switch_op =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_d;
  } else {
    CAVE_MachE_sil_test_B.Switch_op = CAVE_MachE_sil_test_P.Constant_Value_lt;
  }

  /* End of Switch: '<S336>/Switch' */

  /* Gain: '<S336>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_k =
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_bz *
    CAVE_MachE_sil_test_B.Switch_op;

  /* Sum: '<S336>/Sum2' incorporates:
   *  Constant: '<S336>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_g *
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_m;
  CAVE_MachE_sil_test_B.Sum2_f = ((CAVE_MachE_sil_test_B.SignalCopy_g + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[2]) - CAVE_MachE_sil_test_B.Gain2_k;

  /* Gain: '<S336>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_m;
  CAVE_MachE_sil_test_B.Gain1_er = Bias * CAVE_MachE_sil_test_B.Sum2_f;

  /* Switch: '<S337>/Switch' incorporates:
   *  Constant: '<S337>/Constant'
   */
  if (CAVE_MachE_sil_test_B.Sum6_fh > CAVE_MachE_sil_test_P.Switch_Threshold_j)
  {
    CAVE_MachE_sil_test_B.Switch_fy =
      CAVE_MachE_sil_test_B.IntegratorSecondOrder_o2_hy;
  } else {
    CAVE_MachE_sil_test_B.Switch_fy = CAVE_MachE_sil_test_P.Constant_Value_jw;
  }

  /* End of Switch: '<S337>/Switch' */

  /* Gain: '<S337>/Gain2' */
  CAVE_MachE_sil_test_B.Gain2_d =
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_bz *
    CAVE_MachE_sil_test_B.Switch_fy;

  /* Sum: '<S337>/Sum2' incorporates:
   *  Constant: '<S337>/Fg'
   */
  Bias = CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_g *
    CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_m;
  CAVE_MachE_sil_test_B.Sum2_hm = ((CAVE_MachE_sil_test_B.SignalCopy_k + Bias) -
    CAVE_MachE_sil_test_B.Saturation_du[3]) - CAVE_MachE_sil_test_B.Gain2_d;

  /* Gain: '<S337>/Gain1' */
  Bias = 1.0 / CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_m;
  CAVE_MachE_sil_test_B.Gain1_n4 = Bias * CAVE_MachE_sil_test_B.Sum2_hm;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Lookup_n-D: '<S1>/Ackerman steer' */
    CAVE_MachE_sil_test_B.SteeringCmd_c = look1_binlxpw(0.0,
      CAVE_MachE_sil_test_P.Ackermansteer_bp01Data,
      CAVE_MachE_sil_test_P.Ackermansteer_tableData, 188U);

    /* Gain: '<Root>/Gain' */
    CAVE_MachE_sil_test_B.Gain_f = CAVE_MachE_sil_test_P.Gain_Gain_e0 * 0.0;

    /* Gain: '<Root>/Gain1' */
    CAVE_MachE_sil_test_B.Gain1_la = CAVE_MachE_sil_test_P.Gain1_Gain_l * 0.0;
  }
}

/* Model update function */
void CAVE_MachE_sil_test_update(void)
{
  /* local scratch DWork variables */
  int32_T ForEach_itr_c;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0;
  real_T tmpForInput;
  real_T tmpForInput_0;
  real_T tmpForInput_idx_0;
  real_T tmpForInput_idx_1;
  real_T tmp;
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    /* Update for Memory: '<S30>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput = CAVE_MachE_sil_test_B.Merge;

    /* Update for Memory: '<S30>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput = CAVE_MachE_sil_test_B.Spd;

    /* Update for Memory: '<S12>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_f =
      CAVE_MachE_sil_test_B.Divide_f;

    /* Update for Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[0] =
      CAVE_MachE_sil_test_B.WheelTorqueOut[0];
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[1] =
      CAVE_MachE_sil_test_B.WheelTorqueOut[1];
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[2] =
      CAVE_MachE_sil_test_B.WheelTorqueOut[2];
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[3] =
      CAVE_MachE_sil_test_B.WheelTorqueOut[3];

    /* Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator4' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE =
      CAVE_MachE_sil_test_B.PTWFLrot;

    /* Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator5' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE =
      CAVE_MachE_sil_test_B.PTWFRrot;

    /* Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator6' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE =
      CAVE_MachE_sil_test_B.PTWRLrot;

    /* Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator7' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE =
      CAVE_MachE_sil_test_B.PtTWRRrot;

    /* Update for Memory: '<S72>/Memory3' */
    CAVE_MachE_sil_test_DW.Memory3_PreviousInput =
      CAVE_MachE_sil_test_B.MultiportSwitch1[0];

    /* Update for Memory: '<S72>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_f =
      CAVE_MachE_sil_test_B.MultiportSwitch1[1];

    /* Update for Memory: '<S72>/Memory2' */
    CAVE_MachE_sil_test_DW.Memory2_PreviousInput =
      CAVE_MachE_sil_test_B.MultiportSwitch1[2];

    /* Update for Memory: '<S72>/Memory4' */
    CAVE_MachE_sil_test_DW.Memory4_PreviousInput =
      CAVE_MachE_sil_test_B.MultiportSwitch1[3];

    /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator4' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE_b = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE_k =
      CAVE_MachE_sil_test_B.PTWFLrot_a;

    /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator5' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE_g = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE_a =
      CAVE_MachE_sil_test_B.PTWFRrot_n;

    /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator6' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE_p = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE_j =
      CAVE_MachE_sil_test_B.PTWRLrot_n;

    /* Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator7' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE_g = 0U;
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE_i =
      CAVE_MachE_sil_test_B.PtTWRRrot_c;

    /* Update for Memory: '<S78>/Memory3' */
    CAVE_MachE_sil_test_DW.Memory3_PreviousInput_c =
      CAVE_MachE_sil_test_B.omegawheel;

    /* Update for Memory: '<S78>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_d =
      CAVE_MachE_sil_test_B.omegawheel_d;

    /* Update for Memory: '<S78>/Memory2' */
    CAVE_MachE_sil_test_DW.Memory2_PreviousInput_f =
      CAVE_MachE_sil_test_B.omegawheel_g;

    /* Update for Memory: '<S78>/Memory4' */
    CAVE_MachE_sil_test_DW.Memory4_PreviousInput_h =
      CAVE_MachE_sil_test_B.omegawheel_k;

    /* Update for Memory: '<S98>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_g = CAVE_MachE_sil_test_B.Add_b;

    /* Update for Memory: '<S99>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_p = CAVE_MachE_sil_test_B.Add_k;

    /* Update for Memory: '<S100>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_h = CAVE_MachE_sil_test_B.Add_d3;

    /* Update for Memory: '<S101>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_e = CAVE_MachE_sil_test_B.Add_c;

    /* Update for Memory: '<S80>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_o =
      CAVE_MachE_sil_test_B.omegawheel;

    /* Update for Memory: '<S81>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_gr =
      CAVE_MachE_sil_test_B.omegawheel_d;

    /* Update for Memory: '<S82>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_b =
      CAVE_MachE_sil_test_B.omegawheel_g;

    /* Update for Memory: '<S83>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_eq =
      CAVE_MachE_sil_test_B.omegawheel_k;

    /* Update for UnitDelay: '<S113>/Output' */
    CAVE_MachE_sil_test_DW.Output_DSTATE = CAVE_MachE_sil_test_B.FixPtSwitch;

    /* Update for Backlash: '<S130>/Backlash' */
    CAVE_MachE_sil_test_DW.PrevY_e = CAVE_MachE_sil_test_B.Backlash;

    /* Update for Memory: '<S141>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[0] =
      CAVE_MachE_sil_test_B.Switch_k4[0];
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[1] =
      CAVE_MachE_sil_test_B.Switch_k4[1];

    /* Update for Memory: '<S141>/Memory' incorporates:
     *  Constant: '<S141>/Constant'
     */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_b0 =
      CAVE_MachE_sil_test_P.Constant_Value_ig;
  }

  /* Update for Integrator: '<S23>/Integrator Limited' */
  CAVE_MachE_sil_test_DW.IntegratorLimited_IWORK = 0;

  /* Update for Iterator SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */
  for (ForEach_itr_c = 0; ForEach_itr_c < 1; ForEach_itr_c++) {
    /* Update for ForEachSliceSelector generated from: '<S143>/Axle Number' incorporates:
     *  Constant: '<S135>/Axle Number1'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 =
      CAVE_MachE_sil_test_P.AxleNumber1_Value;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S151>/Relational Operator' incorporates:
       *  Constant: '<S151>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[0] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[0]);

      /* DataTypeConversion: '<S151>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[0];

      /* RelationalOperator: '<S151>/Relational Operator' incorporates:
       *  Constant: '<S151>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[1] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[1]);

      /* DataTypeConversion: '<S151>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[1];

      /* Product: '<S151>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [1];
    }

    /* DotProduct: '<S151>/Dot Product1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.Sum1[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.Sum1[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[0];
    tmpForInput = tmpForInput_idx_0 * tmp;
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[1];
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1 = tmpForInput;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S155>/Relational Operator' incorporates:
       *  Constant: '<S155>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_j =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums);

      /* DataTypeConversion: '<S155>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_j =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_j;

      /* Product: '<S155>/Product' incorporates:
       *  Constant: '<S151>/Axle M'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_g =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlM *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_j;

      /* Sum: '<S155>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_g;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements =
        tmpForInput_idx_0;

      /* Product: '<S151>/Product2' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements;

      /* RelationalOperator: '<S156>/Relational Operator' incorporates:
       *  Constant: '<S156>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_g =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums);

      /* DataTypeConversion: '<S156>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_a =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_g;

      /* Product: '<S156>/Product' incorporates:
       *  Constant: '<S151>/Axle I1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_k =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlIxx *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_a;

      /* Sum: '<S156>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_k;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b =
        tmpForInput_idx_0;

      /* Sum: '<S151>/Sum' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[0] +
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product3' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[0] = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[0];

      /* Sum: '<S151>/Sum' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[1] +
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product3' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[1] = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[1];

      /* RelationalOperator: '<S149>/Relational Operator' incorporates:
       *  Constant: '<S149>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[0] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[0]);

      /* DataTypeConversion: '<S149>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[0];

      /* RelationalOperator: '<S149>/Relational Operator' incorporates:
       *  Constant: '<S149>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[1] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[1]);

      /* DataTypeConversion: '<S149>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[1];

      /* Product: '<S149>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[0]
        * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates_o[0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[1]
        * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates_o[1];
    }

    /* Selector: '<S151>/x axis wheel moments' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[0] =
      CAVE_MachE_sil_test_B.Selector17[0];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[1] =
      CAVE_MachE_sil_test_B.Selector17[3];

    /* DotProduct: '<S151>/Dot Product2' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[0];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[0];
    tmpForInput_0 = tmpForInput * tmp;
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[1];
    tmpForInput_0 += tmpForInput * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct2 =
      tmpForInput_0;

    /* Sum: '<S151>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1 +
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct2;

    /* DotProduct: '<S149>/Dot Product1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.Reshape1_h[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.Reshape1_h[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[0];
    tmpForInput = tmpForInput_idx_0 * tmp;
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[1];
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1_h =
      tmpForInput;

    /* Gain: '<S149>/Suspension Moment Direction On Solid Axle' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionMomentDirectionOnSolidAxle =
      CAVE_MachE_sil_test_P.CoreSubsys_m.SuspensionMomentDirectionOnSolidAxle_Gain
      * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1_h;

    /* Sum: '<S147>/Sum1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1;
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionMomentDirectionOnSolidAxle;
    tmpForInput_idx_0 += tmpForInput_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1_n = tmpForInput_idx_0;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S153>/Relational Operator' incorporates:
       *  Constant: '<S153>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_d =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums);

      /* DataTypeConversion: '<S153>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_e =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_d;

      /* Product: '<S153>/Product' incorporates:
       *  Constant: '<S148>/Axle I'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_p =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlIxx *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_e;

      /* Sum: '<S153>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_p;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h =
        tmpForInput_idx_0;
    }

    /* Product: '<S148>/Divide' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1_n /
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h;

    /* DotProduct: '<S151>/Dot Product' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion[1];
    tmpForInput_0 = CAVE_MachE_sil_test_B.Sum1[0];
    tmp = CAVE_MachE_sil_test_B.Sum1[1];
    tmpForInput = tmpForInput_idx_0;
    tmpForInput_0 *= tmpForInput;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion_k[0];
    tmpForInput_idx_0 = tmpForInput;

    /* DotProduct: '<S151>/Dot Product' */
    tmpForInput = tmpForInput_idx_1;
    tmpForInput_0 += tmpForInput * tmp;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion_k[1];
    tmpForInput_idx_1 = tmpForInput;

    /* DotProduct: '<S151>/Dot Product' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct = tmpForInput_0;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput_0 = CAVE_MachE_sil_test_B.Reshape1_h[0];
    tmp = CAVE_MachE_sil_test_B.Reshape1_h[1];
    tmpForInput = tmpForInput_idx_0 * tmpForInput_0;
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct_o = tmpForInput;

    /* Gain: '<S149>/Suspension Force Direction On Solid Axle' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionForceDirectionOnSolidAxle =
      CAVE_MachE_sil_test_P.CoreSubsys_m.SuspensionForceDirectionOnSolidAxle_Gain
      * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct_o;

    /* Sum: '<S147>/Sum' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DotProduct;
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionForceDirectionOnSolidAxle;
    tmpForInput_idx_0 += tmpForInput_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_f = tmpForInput_idx_0;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S152>/Relational Operator' incorporates:
       *  Constant: '<S152>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_p =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums_h);

      /* DataTypeConversion: '<S152>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_g =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_p;

      /* Product: '<S152>/Product' incorporates:
       *  Constant: '<S148>/Axle M'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_j =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlM *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_g;

      /* Sum: '<S152>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_j;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h5 =
        tmpForInput_idx_0;
    }

    /* Product: '<S148>/Divide1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Divide1 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_f /
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h5;

    /* Sum: '<S148>/Sum2' incorporates:
     *  Constant: '<S148>/g (Earth)'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum2 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Divide1 +
      CAVE_MachE_sil_test_P.CoreSubsys_m.gEarth_Value;

    /* Sum: '<S148>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zddot =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum2 -
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydotp;
  }

  /* End of Update for SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */
  if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
    rt_ertODEUpdateContinuousStates(&CAVE_MachE_sil_test_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++CAVE_MachE_sil_test_M->Timing.clockTick0)) {
    ++CAVE_MachE_sil_test_M->Timing.clockTickH0;
  }

  CAVE_MachE_sil_test_M->Timing.t[0] = rtsiGetSolverStopTime
    (&CAVE_MachE_sil_test_M->solverInfo);

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
    CAVE_MachE_sil_test_M->Timing.clockTick1++;
    if (!CAVE_MachE_sil_test_M->Timing.clockTick1) {
      CAVE_MachE_sil_test_M->Timing.clockTickH1++;
    }
  }
}

/* Derivatives for root system: '<Root>' */
void CAVE_MachE_sil_test_derivatives(void)
{
  /* local scratch DWork variables */
  int32_T ForEach_itr_c;
  boolean_T lsat;
  boolean_T usat;
  real_T rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0;
  real_T tmpForInput;
  real_T tmpForInput_0;
  real_T tmpForInput_idx_0;
  real_T tmpForInput_idx_1;
  real_T tmp;
  XDot_CAVE_MachE_sil_test_T *_rtXdot;
  _rtXdot = ((XDot_CAVE_MachE_sil_test_T *) CAVE_MachE_sil_test_M->derivs);

  /* Derivatives for Integrator: '<S218>/xe,ye,ze' */
  _rtXdot->xeyeze_CSTATE[0] = CAVE_MachE_sil_test_B.Reshape2_me[0];
  _rtXdot->xeyeze_CSTATE[1] = CAVE_MachE_sil_test_B.Reshape2_me[1];
  _rtXdot->xeyeze_CSTATE[2] = CAVE_MachE_sil_test_B.Reshape2_me[2];

  /* Derivatives for Integrator: '<S230>/phi theta psi' */
  _rtXdot->phithetapsi_CSTATE[0] =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[0];
  _rtXdot->phithetapsi_CSTATE[1] =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[1];
  _rtXdot->phithetapsi_CSTATE[2] =
    CAVE_MachE_sil_test_B.TmpSignalConversionAtphithetapsiInport1[2];

  /* Derivatives for Integrator: '<S218>/ub,vb,wb' */
  _rtXdot->ubvbwb_CSTATE[0] = CAVE_MachE_sil_test_B.Sum_dx[0];
  _rtXdot->ubvbwb_CSTATE[1] = CAVE_MachE_sil_test_B.Sum_dx[1];
  _rtXdot->ubvbwb_CSTATE[2] = CAVE_MachE_sil_test_B.Sum_dx[2];

  /* Derivatives for Integrator: '<S9>/Integrator' */
  _rtXdot->Integrator_CSTATE = CAVE_MachE_sil_test_B.VectorConcatenate_e[0];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE = 0.0;
  _rtXdot->TransferFcn3_CSTATE += CAVE_MachE_sil_test_P.TransferFcn3_A *
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE;
  _rtXdot->TransferFcn3_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_g3[0];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += CAVE_MachE_sil_test_P.TransferFcn1_A *
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_g3[1];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE = 0.0;
  _rtXdot->TransferFcn2_CSTATE += CAVE_MachE_sil_test_P.TransferFcn2_A *
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_g3[2];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn4' */
  _rtXdot->TransferFcn4_CSTATE = 0.0;
  _rtXdot->TransferFcn4_CSTATE += CAVE_MachE_sil_test_P.TransferFcn4_A *
    CAVE_MachE_sil_test_X.TransferFcn4_CSTATE;
  _rtXdot->TransferFcn4_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_g3[3];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += CAVE_MachE_sil_test_P.TransferFcn_A *
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[0];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn9' */
  _rtXdot->TransferFcn9_CSTATE = 0.0;
  _rtXdot->TransferFcn9_CSTATE += CAVE_MachE_sil_test_P.TransferFcn9_A *
    CAVE_MachE_sil_test_X.TransferFcn9_CSTATE;
  _rtXdot->TransferFcn9_CSTATE +=
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[1];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn10' */
  _rtXdot->TransferFcn10_CSTATE = 0.0;
  _rtXdot->TransferFcn10_CSTATE += CAVE_MachE_sil_test_P.TransferFcn10_A *
    CAVE_MachE_sil_test_X.TransferFcn10_CSTATE;
  _rtXdot->TransferFcn10_CSTATE +=
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[2];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn11' */
  _rtXdot->TransferFcn11_CSTATE = 0.0;
  _rtXdot->TransferFcn11_CSTATE += CAVE_MachE_sil_test_P.TransferFcn11_A *
    CAVE_MachE_sil_test_X.TransferFcn11_CSTATE;
  _rtXdot->TransferFcn11_CSTATE +=
    CAVE_MachE_sil_test_B.ImpAsg_InsertedFor_Fx_at_inport_0[3];

  /* Derivatives for Integrator: '<S39>/Integrator' */
  _rtXdot->Integrator_CSTATE_k = CAVE_MachE_sil_test_B.Sum;

  /* Derivatives for TransferFcn: '<S4>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_i = 0.0;
  _rtXdot->TransferFcn_CSTATE_i += CAVE_MachE_sil_test_P.TransferFcn_A_k *
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE_i;
  _rtXdot->TransferFcn_CSTATE_i += CAVE_MachE_sil_test_B.MultiportSwitch1[0];

  /* Derivatives for TransferFcn: '<S4>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE_i = 0.0;
  _rtXdot->TransferFcn1_CSTATE_i += CAVE_MachE_sil_test_P.TransferFcn1_A_c *
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE_i;
  _rtXdot->TransferFcn1_CSTATE_i += CAVE_MachE_sil_test_B.MultiportSwitch1[1];

  /* Derivatives for TransferFcn: '<S4>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE_h = 0.0;
  _rtXdot->TransferFcn2_CSTATE_h += CAVE_MachE_sil_test_P.TransferFcn2_A_j *
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE_h;
  _rtXdot->TransferFcn2_CSTATE_h += CAVE_MachE_sil_test_B.MultiportSwitch1[2];

  /* Derivatives for TransferFcn: '<S4>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE_h = 0.0;
  _rtXdot->TransferFcn3_CSTATE_h += CAVE_MachE_sil_test_P.TransferFcn3_A_j *
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE_h;
  _rtXdot->TransferFcn3_CSTATE_h += CAVE_MachE_sil_test_B.MultiportSwitch1[3];

  /* Derivatives for Integrator: '<S23>/Integrator Limited' */
  lsat = (CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE <=
          CAVE_MachE_sil_test_P.IntegratorLimited_LowerSat);
  usat = (CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE >=
          CAVE_MachE_sil_test_P.BattChrgCapcty);
  if (((!lsat) && (!usat)) || (lsat && (CAVE_MachE_sil_test_B.Gain1_i > 0.0)) ||
      (usat && (CAVE_MachE_sil_test_B.Gain1_i < 0.0))) {
    _rtXdot->IntegratorLimited_CSTATE = CAVE_MachE_sil_test_B.Gain1_i;
  } else {
    /* in saturation */
    _rtXdot->IntegratorLimited_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<S23>/Integrator Limited' */

  /* Derivatives for Integrator: '<S350>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = CAVE_MachE_sil_test_B.Fdot_h;

  /* Derivatives for Integrator: '<S347>/Integrator' */
  _rtXdot->Integrator_CSTATE_o = CAVE_MachE_sil_test_B.Fdot;

  /* Derivatives for SecondOrderIntegrator: '<S345>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE[1];
    _rtXdot->IntegratorSecondOrder_CSTATE[1] = CAVE_MachE_sil_test_B.Gain1_ge;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S345>/Integrator, Second-Order' */

  /* Derivatives for Chart: '<S346>/LockUp' */
  CAVE_MachE_sil_test_LockUp_Deriv(&CAVE_MachE_sil_test_B.sf_LockUp,
    &CAVE_MachE_sil_test_DW.sf_LockUp, &_rtXdot->sf_LockUp);

  /* Derivatives for Integrator: '<S375>/Integrator' */
  _rtXdot->Integrator_CSTATE_c = CAVE_MachE_sil_test_B.Fdot_o;

  /* Derivatives for Integrator: '<S372>/Integrator' */
  _rtXdot->Integrator_CSTATE_ch = CAVE_MachE_sil_test_B.Fdot_f;

  /* Derivatives for SecondOrderIntegrator: '<S370>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_d == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_l[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_l[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_l[1] = CAVE_MachE_sil_test_B.Gain1_p;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S370>/Integrator, Second-Order' */

  /* Derivatives for Chart: '<S371>/LockUp' */
  CAVE_MachE_sil_test_LockUp_Deriv(&CAVE_MachE_sil_test_B.sf_LockUp_n,
    &CAVE_MachE_sil_test_DW.sf_LockUp_n, &_rtXdot->sf_LockUp_n);

  /* Derivatives for Integrator: '<S400>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = CAVE_MachE_sil_test_B.Fdot_c;

  /* Derivatives for Integrator: '<S397>/Integrator' */
  _rtXdot->Integrator_CSTATE_b = CAVE_MachE_sil_test_B.Fdot_d;

  /* Derivatives for SecondOrderIntegrator: '<S395>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_h == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_ln[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_ln[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_ln[1] = CAVE_MachE_sil_test_B.Gain1_o;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S395>/Integrator, Second-Order' */

  /* Derivatives for Chart: '<S396>/LockUp' */
  CAVE_MachE_sil_test_LockUp_Deriv(&CAVE_MachE_sil_test_B.sf_LockUp_h,
    &CAVE_MachE_sil_test_DW.sf_LockUp_h, &_rtXdot->sf_LockUp_h);

  /* Derivatives for Integrator: '<S425>/Integrator' */
  _rtXdot->Integrator_CSTATE_a = CAVE_MachE_sil_test_B.Fdot_bw;

  /* Derivatives for Integrator: '<S422>/Integrator' */
  _rtXdot->Integrator_CSTATE_g = CAVE_MachE_sil_test_B.Fdot_m;

  /* Derivatives for SecondOrderIntegrator: '<S420>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_l == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_j[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_j[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_j[1] = CAVE_MachE_sil_test_B.Gain1_b;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S420>/Integrator, Second-Order' */

  /* Derivatives for Chart: '<S421>/LockUp' */
  CAVE_MachE_sil_test_LockUp_Deriv(&CAVE_MachE_sil_test_B.sf_LockUp_c,
    &CAVE_MachE_sil_test_DW.sf_LockUp_c, &_rtXdot->sf_LockUp_c);

  /* Derivatives for Integrator: '<S80>/omega wheel' */
  _rtXdot->omegaWheel = CAVE_MachE_sil_test_B.Switch_p;

  /* Derivatives for Integrator: '<S81>/omega wheel' */
  _rtXdot->omegaWheel_j = CAVE_MachE_sil_test_B.Switch_k;

  /* Derivatives for Integrator: '<S82>/omega wheel' */
  _rtXdot->omegaWheel_d = CAVE_MachE_sil_test_B.Switch_cv;

  /* Derivatives for Integrator: '<S83>/omega wheel' */
  _rtXdot->omegaWheel_jl = CAVE_MachE_sil_test_B.Switch_cn;

  /* Derivatives for SecondOrderIntegrator: '<S334>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_a == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_m[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_m[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_m[1] = CAVE_MachE_sil_test_B.Gain1_m;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S334>/Integrator, Second-Order' */

  /* Derivatives for SecondOrderIntegrator: '<S335>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_o == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_i[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_i[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_i[1] = CAVE_MachE_sil_test_B.Gain1_na;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S335>/Integrator, Second-Order' */

  /* Derivatives for SecondOrderIntegrator: '<S336>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_dh == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_e[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_e[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_e[1] = CAVE_MachE_sil_test_B.Gain1_er;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S336>/Integrator, Second-Order' */

  /* Derivatives for SecondOrderIntegrator: '<S337>/Integrator, Second-Order' */
  if (CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_hl == 0) {
    _rtXdot->IntegratorSecondOrder_CSTATE_jk[0] =
      CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_jk[1];
    _rtXdot->IntegratorSecondOrder_CSTATE_jk[1] = CAVE_MachE_sil_test_B.Gain1_n4;
  }

  /* End of Derivatives for SecondOrderIntegrator: '<S337>/Integrator, Second-Order' */

  /* Derivatives for Integrator: '<S348>/Integrator' */
  _rtXdot->Integrator_CSTATE_i = CAVE_MachE_sil_test_B.Fdot_a;

  /* Derivatives for Integrator: '<S373>/Integrator' */
  _rtXdot->Integrator_CSTATE_b0 = CAVE_MachE_sil_test_B.Fdot_l;

  /* Derivatives for Integrator: '<S398>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = CAVE_MachE_sil_test_B.Fdot_b;

  /* Derivatives for Integrator: '<S423>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = CAVE_MachE_sil_test_B.Fdot_g;

  /* Derivatives for Integrator: '<S310>/Integrator1' */
  _rtXdot->Integrator1_CSTATE[0] = CAVE_MachE_sil_test_B.Divide_ah[0];
  _rtXdot->Integrator1_CSTATE[1] = CAVE_MachE_sil_test_B.Divide_ah[1];
  _rtXdot->Integrator1_CSTATE[2] = CAVE_MachE_sil_test_B.Divide_ah[2];
  _rtXdot->Integrator1_CSTATE[3] = CAVE_MachE_sil_test_B.Divide_ah[3];

  /* Derivatives for Integrator: '<S311>/Integrator1' */
  memcpy(&_rtXdot->Integrator1_CSTATE_m[0], &CAVE_MachE_sil_test_B.Divide_d4[0],
         12U * sizeof(real_T));

  /* Derivatives for Integrator: '<S218>/p,q,r ' */
  _rtXdot->pqr_CSTATE[0] = CAVE_MachE_sil_test_B.Reshape_o[0];
  _rtXdot->pqr_CSTATE[1] = CAVE_MachE_sil_test_B.Reshape_o[1];
  _rtXdot->pqr_CSTATE[2] = CAVE_MachE_sil_test_B.Reshape_o[2];

  /* Derivatives for Iterator SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */
  for (ForEach_itr_c = 0; ForEach_itr_c < 1; ForEach_itr_c++) {
    /* Derivatives for ForEachSliceSelector generated from: '<S143>/Axle Number' incorporates:
     *  Constant: '<S135>/Axle Number1'
     */
    rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 =
      CAVE_MachE_sil_test_P.AxleNumber1_Value;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S151>/Relational Operator' incorporates:
       *  Constant: '<S151>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[0] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[0]);

      /* DataTypeConversion: '<S151>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[0];

      /* RelationalOperator: '<S151>/Relational Operator' incorporates:
       *  Constant: '<S151>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[1] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[1]);

      /* DataTypeConversion: '<S151>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[1];

      /* Product: '<S151>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates
        [1];
    }

    /* DotProduct: '<S151>/Dot Product1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.Sum1[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.Sum1[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[0];
    tmpForInput = tmpForInput_idx_0 * tmp;
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[1];
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1 = tmpForInput;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S155>/Relational Operator' incorporates:
       *  Constant: '<S155>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_j =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums);

      /* DataTypeConversion: '<S155>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_j =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_j;

      /* Product: '<S155>/Product' incorporates:
       *  Constant: '<S151>/Axle M'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_g =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlM *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_j;

      /* Sum: '<S155>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_g;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements =
        tmpForInput_idx_0;

      /* Product: '<S151>/Product2' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements;

      /* RelationalOperator: '<S156>/Relational Operator' incorporates:
       *  Constant: '<S156>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_g =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectYAxisAxleMassMomentofInertiaByAxle_AxleNums);

      /* DataTypeConversion: '<S156>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_a =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_g;

      /* Product: '<S156>/Product' incorporates:
       *  Constant: '<S151>/Axle I1'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_k =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlIxx *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_a;

      /* Sum: '<S156>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_k;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b =
        tmpForInput_idx_0;

      /* Sum: '<S151>/Sum' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[0] +
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product3' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[0] = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[0];

      /* Sum: '<S151>/Sum' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[1] +
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product3' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[1] = 1.0 /
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b;

      /* Product: '<S151>/Product4' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[1];

      /* RelationalOperator: '<S149>/Relational Operator' incorporates:
       *  Constant: '<S149>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[0] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[0]);

      /* DataTypeConversion: '<S149>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[0];

      /* RelationalOperator: '<S149>/Relational Operator' incorporates:
       *  Constant: '<S149>/Constant2'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[1] =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxleNumVec[1]);

      /* DataTypeConversion: '<S149>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[1];

      /* Product: '<S149>/Product' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[0] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[0]
        * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates_o[0];
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[1] =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[1]
        * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        yaxistrackcoordinates_o[1];
    }

    /* Selector: '<S151>/x axis wheel moments' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[0] =
      CAVE_MachE_sil_test_B.Selector17[0];
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[1] =
      CAVE_MachE_sil_test_B.Selector17[3];

    /* DotProduct: '<S151>/Dot Product2' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[0];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[0];
    tmpForInput_0 = tmpForInput * tmp;
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[1];
    tmpForInput_0 += tmpForInput * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct2 =
      tmpForInput_0;

    /* Sum: '<S151>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1 +
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct2;

    /* DotProduct: '<S149>/Dot Product1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.Reshape1_h[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.Reshape1_h[1];
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[0];
    tmpForInput = tmpForInput_idx_0 * tmp;
    tmp = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[1];
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1_h =
      tmpForInput;

    /* Gain: '<S149>/Suspension Moment Direction On Solid Axle' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionMomentDirectionOnSolidAxle =
      CAVE_MachE_sil_test_P.CoreSubsys_m.SuspensionMomentDirectionOnSolidAxle_Gain
      * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1_h;

    /* Sum: '<S147>/Sum1' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1;
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionMomentDirectionOnSolidAxle;
    tmpForInput_idx_0 += tmpForInput_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1_n = tmpForInput_idx_0;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S153>/Relational Operator' incorporates:
       *  Constant: '<S153>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_d =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectXAxisAxleMassMomentofInertiaByAxle_AxleNums);

      /* DataTypeConversion: '<S153>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_e =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_d;

      /* Product: '<S153>/Product' incorporates:
       *  Constant: '<S148>/Axle I'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_p =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlIxx *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_e;

      /* Sum: '<S153>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_p;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h =
        tmpForInput_idx_0;
    }

    /* Product: '<S148>/Divide' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1_n /
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h;

    /* DotProduct: '<S151>/Dot Product' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion[0];
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion[1];
    tmpForInput_0 = CAVE_MachE_sil_test_B.Sum1[0];
    tmp = CAVE_MachE_sil_test_B.Sum1[1];
    tmpForInput = tmpForInput_idx_0;
    tmpForInput_0 *= tmpForInput;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion_k[0];
    tmpForInput_idx_0 = tmpForInput;

    /* DotProduct: '<S151>/Dot Product' */
    tmpForInput = tmpForInput_idx_1;
    tmpForInput_0 += tmpForInput * tmp;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DataTypeConversion_k[1];
    tmpForInput_idx_1 = tmpForInput;

    /* DotProduct: '<S151>/Dot Product' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct = tmpForInput_0;

    /* DotProduct: '<S149>/Dot Product' */
    tmpForInput_0 = CAVE_MachE_sil_test_B.Reshape1_h[0];
    tmp = CAVE_MachE_sil_test_B.Reshape1_h[1];
    tmpForInput = tmpForInput_idx_0 * tmpForInput_0;
    tmpForInput += tmpForInput_idx_1 * tmp;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct_o = tmpForInput;

    /* Gain: '<S149>/Suspension Force Direction On Solid Axle' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionForceDirectionOnSolidAxle =
      CAVE_MachE_sil_test_P.CoreSubsys_m.SuspensionForceDirectionOnSolidAxle_Gain
      * CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct_o;

    /* Sum: '<S147>/Sum' */
    tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      DotProduct;
    tmpForInput_idx_1 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
      SuspensionForceDirectionOnSolidAxle;
    tmpForInput_idx_0 += tmpForInput_idx_1;
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_f = tmpForInput_idx_0;
    if (rtmIsMajorTimeStep(CAVE_MachE_sil_test_M)) {
      /* RelationalOperator: '<S152>/Relational Operator' incorporates:
       *  Constant: '<S152>/Axle Numbers'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_p =
        (rtb_ImpSel_InsertedFor_AxleNumber_at_outport_0 ==
         CAVE_MachE_sil_test_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums_h);

      /* DataTypeConversion: '<S152>/Data Type Conversion' */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_g =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_p;

      /* Product: '<S152>/Product' incorporates:
       *  Constant: '<S148>/Axle M'
       */
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_j =
        CAVE_MachE_sil_test_P.SolidAxleSuspensionCoilSpring_AxlM *
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_g;

      /* Sum: '<S152>/Sum of Elements' */
      tmpForInput_idx_0 = CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        Product_j;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h5 =
        tmpForInput_idx_0;
    }

    /* Product: '<S148>/Divide1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Divide1 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_f /
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h5;

    /* Sum: '<S148>/Sum2' incorporates:
     *  Constant: '<S148>/g (Earth)'
     */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum2 =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Divide1 +
      CAVE_MachE_sil_test_P.CoreSubsys_m.gEarth_Value;

    /* Sum: '<S148>/Sum1' */
    CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zddot =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum2 -
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydotp;

    /* Derivatives for Integrator: '<S154>/ ' */
    lsat = (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE <=
            CAVE_MachE_sil_test_P.CoreSubsys_m._LowerSat);
    usat = (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE >=
            CAVE_MachE_sil_test_P.CoreSubsys_m._UpperSat);
    if (((!lsat) && (!usat)) || (lsat &&
         (CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p > 0.0)) || (usat &&
         (CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p < 0.0))) {
      _rtXdot->CoreSubsys_m[ForEach_itr_c]._CSTATE =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p;
    } else {
      /* in saturation */
      _rtXdot->CoreSubsys_m[ForEach_itr_c]._CSTATE = 0.0;
    }

    /* End of Derivatives for Integrator: '<S154>/ ' */

    /* Derivatives for Integrator: '<S150>/cg coordinates' */
    _rtXdot->CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[0] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[0];
    _rtXdot->CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[1] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[1];
    _rtXdot->CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[2] =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[2];

    /* Derivatives for Integrator: '<S148>/Vz' */
    _rtXdot->CoreSubsys_m[ForEach_itr_c].Vz_CSTATE =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zddot;

    /* Derivatives for Integrator: '<S148>/Vy1' */
    lsat = (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE <=
            CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_LowerSat);
    usat = (CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE >=
            CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_UpperSat);
    if (((!lsat) && (!usat)) || (lsat &&
         (CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot > 0.0)) ||
        (usat && (CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot < 0.0)))
    {
      _rtXdot->CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE =
        CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot;
    } else {
      /* in saturation */
      _rtXdot->CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE = 0.0;
    }

    /* End of Derivatives for Integrator: '<S148>/Vy1' */

    /* Derivatives for Integrator: '<S148>/Vy' */
    _rtXdot->CoreSubsys_m[ForEach_itr_c].Vy_CSTATE =
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum;
  }

  /* End of Derivatives for SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn5' */
  _rtXdot->TransferFcn5_CSTATE = 0.0;
  _rtXdot->TransferFcn5_CSTATE += CAVE_MachE_sil_test_P.TransferFcn5_A *
    CAVE_MachE_sil_test_X.TransferFcn5_CSTATE;
  _rtXdot->TransferFcn5_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_l[0];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn6' */
  _rtXdot->TransferFcn6_CSTATE = 0.0;
  _rtXdot->TransferFcn6_CSTATE += CAVE_MachE_sil_test_P.TransferFcn6_A *
    CAVE_MachE_sil_test_X.TransferFcn6_CSTATE;
  _rtXdot->TransferFcn6_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_l[1];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn7' */
  _rtXdot->TransferFcn7_CSTATE = 0.0;
  _rtXdot->TransferFcn7_CSTATE += CAVE_MachE_sil_test_P.TransferFcn7_A *
    CAVE_MachE_sil_test_X.TransferFcn7_CSTATE;
  _rtXdot->TransferFcn7_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_l[2];

  /* Derivatives for TransferFcn: '<S120>/Transfer Fcn8' */
  _rtXdot->TransferFcn8_CSTATE = 0.0;
  _rtXdot->TransferFcn8_CSTATE += CAVE_MachE_sil_test_P.TransferFcn8_A *
    CAVE_MachE_sil_test_X.TransferFcn8_CSTATE;
  _rtXdot->TransferFcn8_CSTATE += CAVE_MachE_sil_test_B.VectorConcatenate_l[3];

  /* Derivatives for Integrator: '<S226>/Integrator' */
  _rtXdot->Integrator_CSTATE_j[0] = CAVE_MachE_sil_test_B.UnitConversion[0];
  _rtXdot->Integrator_CSTATE_j[1] = CAVE_MachE_sil_test_B.UnitConversion[1];
  _rtXdot->Integrator_CSTATE_j[2] = CAVE_MachE_sil_test_B.UnitConversion[2];
}

/* Model initialize function */
void CAVE_MachE_sil_test_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  CAVE_MachE_sil_test_P.Saturation_UpperSat = rtInf;
  CAVE_MachE_sil_test_P.Saturation1_UpperSat = rtInf;
  CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_e = rtInf;
  CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_k = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_b = rtInf;
  CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_i = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_d = rtInf;
  CAVE_MachE_sil_test_P.DisallowNegativeBrakeTorque_UpperSat_c = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_h = rtInf;
  CAVE_MachE_sil_test_P.Saturation1_UpperSat_l = rtInf;
  CAVE_MachE_sil_test_P.Saturation1_UpperSat_ak = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_n = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_l = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_i = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_e2 = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_ew = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_bk = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_ik = rtInf;
  CAVE_MachE_sil_test_P.Saturation_UpperSat_c = rtInf;

  /* initialize real-time model */
  (void) memset((void *)CAVE_MachE_sil_test_M, 0,
                sizeof(RT_MODEL_CAVE_MachE_sil_test_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&CAVE_MachE_sil_test_M->solverInfo,
                          &CAVE_MachE_sil_test_M->Timing.simTimeStep);
    rtsiSetTPtr(&CAVE_MachE_sil_test_M->solverInfo, &rtmGetTPtr
                (CAVE_MachE_sil_test_M));
    rtsiSetStepSizePtr(&CAVE_MachE_sil_test_M->solverInfo,
                       &CAVE_MachE_sil_test_M->Timing.stepSize0);
    rtsiSetdXPtr(&CAVE_MachE_sil_test_M->solverInfo,
                 &CAVE_MachE_sil_test_M->derivs);
    rtsiSetContStatesPtr(&CAVE_MachE_sil_test_M->solverInfo, (real_T **)
                         &CAVE_MachE_sil_test_M->contStates);
    rtsiSetNumContStatesPtr(&CAVE_MachE_sil_test_M->solverInfo,
      &CAVE_MachE_sil_test_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&CAVE_MachE_sil_test_M->solverInfo,
      &CAVE_MachE_sil_test_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&CAVE_MachE_sil_test_M->solverInfo,
      &CAVE_MachE_sil_test_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&CAVE_MachE_sil_test_M->solverInfo,
      &CAVE_MachE_sil_test_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&CAVE_MachE_sil_test_M->solverInfo,
                          (&rtmGetErrorStatus(CAVE_MachE_sil_test_M)));
    rtsiSetRTModelPtr(&CAVE_MachE_sil_test_M->solverInfo, CAVE_MachE_sil_test_M);
  }

  rtsiSetSimTimeStep(&CAVE_MachE_sil_test_M->solverInfo, MAJOR_TIME_STEP);
  CAVE_MachE_sil_test_M->intgData.y = CAVE_MachE_sil_test_M->odeY;
  CAVE_MachE_sil_test_M->intgData.f[0] = CAVE_MachE_sil_test_M->odeF[0];
  CAVE_MachE_sil_test_M->intgData.f[1] = CAVE_MachE_sil_test_M->odeF[1];
  CAVE_MachE_sil_test_M->intgData.f[2] = CAVE_MachE_sil_test_M->odeF[2];
  CAVE_MachE_sil_test_M->intgData.f[3] = CAVE_MachE_sil_test_M->odeF[3];
  CAVE_MachE_sil_test_M->contStates = ((X_CAVE_MachE_sil_test_T *)
    &CAVE_MachE_sil_test_X);
  CAVE_MachE_sil_test_M->periodicContStateIndices = ((int_T*)
    CAVE_MachE_sil_test_PeriodicIndX);
  CAVE_MachE_sil_test_M->periodicContStateRanges = ((real_T*)
    CAVE_MachE_sil_test_PeriodicRngX);
  rtsiSetSolverData(&CAVE_MachE_sil_test_M->solverInfo, (void *)
                    &CAVE_MachE_sil_test_M->intgData);
  rtsiSetSolverName(&CAVE_MachE_sil_test_M->solverInfo,"ode4");
  rtmSetTPtr(CAVE_MachE_sil_test_M, &CAVE_MachE_sil_test_M->Timing.tArray[0]);
  CAVE_MachE_sil_test_M->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond(CAVE_MachE_sil_test_M, 1);

  /* block I/O */
  (void) memset(((void *) &CAVE_MachE_sil_test_B), 0,
                sizeof(B_CAVE_MachE_sil_test_T));

  /* states (continuous) */
  {
    (void) memset((void *)&CAVE_MachE_sil_test_X, 0,
                  sizeof(X_CAVE_MachE_sil_test_T));
  }

  /* Periodic continuous states */
  {
    (void) memset((void*) CAVE_MachE_sil_test_PeriodicIndX, 0,
                  3*sizeof(int_T));
    (void) memset((void*) CAVE_MachE_sil_test_PeriodicRngX, 0,
                  6*sizeof(real_T));
  }

  /* states (dwork) */
  (void) memset((void *)&CAVE_MachE_sil_test_DW, 0,
                sizeof(DW_CAVE_MachE_sil_test_T));

  {
    /* user code (registration function declaration) */
    /*Initialize global TRC pointers. */
    CAVE_MachE_sil_test_rti_init_trc_pointers();
  }

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;
    int32_T ForEach_itr_j;
    int32_T ForEach_itr_f;
    int32_T ForEach_itr_k;
    int32_T ForEach_itr_h;
    int32_T ForEach_itr_hh;
    int32_T ForEach_itr_c;
    int32_T ForEach_itr_o;
    int32_T i;

    /* Start for If: '<S39>/If' */
    CAVE_MachE_sil_test_DW.If_ActiveSubsystem = -1;

    /* Start for Iterator SubSystem: '<S313>/Wheel to Body Transform' */
    for (ForEach_itr_j = 0; ForEach_itr_j < 4; ForEach_itr_j++) {
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
        TmpSignalConversionAtsincosInport1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
        TmpSignalConversionAtsincosInport1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
        TmpSignalConversionAtsincosInport1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].sincos_o2[2] = 0.0;
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
             VectorConcatenate[0], 0, 9U * sizeof(real_T));
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].
             Reshape9to3x3columnmajor[0], 0, 9U * sizeof(real_T));
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pn[ForEach_itr_j].Divide1[2] = 0.0;
    }

    /* End of Start for SubSystem: '<S313>/Wheel to Body Transform' */

    /* Start for Iterator SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */
    for (ForEach_itr_c = 0; ForEach_itr_c < 1; ForEach_itr_c++) {
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].u = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].TrigonometricFunction1 =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Reshape[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate4[2] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixConcatenate5[2] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].cgcoordinates[2] = 0.0;
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
             MatrixConcatenate3[0], 0, 9U * sizeof(real_T));
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction1[0],
             0, 9U * sizeof(real_T));
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zdot = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        TmpSignalConversionAtMatrixMultiply2Inport2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        TmpSignalConversionAtMatrixMultiply2Inport2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        TmpSignalConversionAtMatrixMultiply2Inport2[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MatrixMultiply2[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates[0]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates[1]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].MathFunction[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates_o
        [0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].yaxistrackcoordinates_o
        [1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydot = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].ydotp = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_j =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_j =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_g =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_a =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_k = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[0] =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[0] =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator[1] =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_e[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product3[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product4[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].xaxiswheelmoments[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_i[1] =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_k[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_b[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct1_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        SuspensionMomentDirectionOnSolidAxle = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum1_n = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_d =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_e =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].pdot = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DotProduct_o = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].
        SuspensionForceDirectionOnSolidAxle = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum_f = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].RelationalOperator_p =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].DataTypeConversion_g =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Product_j = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].SumofElements_h5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Divide1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].Sum2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_m[ForEach_itr_c].zddot = 0.0;
    }

    /* End of Start for SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */

    /* Start for Iterator SubSystem: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
    for (ForEach_itr_o = 0; ForEach_itr_o < 2; ForEach_itr_o++) {
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector6 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Gain = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].DCMStaringRow = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1 = 0.0;
      memset(&CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].SelectDCM[0], 0,
             9U * sizeof(real_T));
      memset(&CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MathFunction[0], 0,
             9U * sizeof(real_T));
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].TrigonometricFunction1 =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Rotationeffectonpositions
        [0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Rotationeffectonpositions
        [1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixMultiply1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].
        Translationeffectonpositions[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum1_l[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector5[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].MatrixConcatenate4[2] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product2[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum3[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Selector[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Rotationeffectonpositions
        [2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum4[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Product1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys[ForEach_itr_o].Sum2[2] = 0.0;
    }

    /* End of Start for SubSystem: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */

    /* Start for Iterator SubSystem: '<S315>/Wheel to Body Transform' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
        TmpSignalConversionAtsincosInport1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
        TmpSignalConversionAtsincosInport1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
        TmpSignalConversionAtsincosInport1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].sincos_o2[2] = 0.0;
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
             VectorConcatenate[0], 0, 9U * sizeof(real_T));
      memset(&CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].
             Reshape9to3x3columnmajor[0], 0, 9U * sizeof(real_T));
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_pna[ForEach_itr].Divide1[2] = 0.0;
    }

    /* End of Start for SubSystem: '<S315>/Wheel to Body Transform' */

    /* Start for Iterator SubSystem: '<S140>/For each track and axle combination calculate suspension forces and moments' */
    for (ForEach_itr_k = 0; ForEach_itr_k < 2; ForEach_itr_k++) {
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtSelector3Inport1[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_n = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_o =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p5 =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_i = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_a =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_a =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_m = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_k = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_hq = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_c =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_c =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_k = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_g =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_o =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_c = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add4 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].HeightSignConvention =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum2_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_ag =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_m =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_o0 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product4_j = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_l5 =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_ck =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_j = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_e =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_b =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_c = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Abs_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_n =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_d =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_i = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_oi = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_lx = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_p =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_f =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_e = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_f = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_m =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_pi =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_o = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product3_n = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_mm =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_p4 =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_l4 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product5_c = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Add1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_o = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sign1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product2_d = 0.0;
      for (i = 0; i < 6; i++) {
        CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
          TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[i] =
          0.0;
      }

      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].RelationalOperator_nm =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].DataTypeConversion_g =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_gk = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].SumofElements_dv = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_l = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleForceSign = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector1_c = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].VehicleHeight = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum_f = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product_j = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].UnaryMinus = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector2_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Product1_l = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Reshape[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_k[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Sum1_f[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Selector3_i = 0.0;

      /* Start for Atomic SubSystem: '<S206>/Max stop reached' */
      CAVE_MachE_sil_test_Maxstopreached_Start
        (&CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Maxstopreached);

      /* End of Start for SubSystem: '<S206>/Max stop reached' */

      /* Start for Atomic SubSystem: '<S206>/Min stop reached' */
      CAVE_MachE_sil_test_Minstopreached_Start
        (&CAVE_MachE_sil_test_B.CoreSubsys_d[ForEach_itr_k].Minstopreached);

      /* End of Start for SubSystem: '<S206>/Min stop reached' */
    }

    /* End of Start for SubSystem: '<S140>/For each track and axle combination calculate suspension forces and moments' */

    /* Start for Iterator SubSystem: '<S182>/For Each Axle With Anti-Sway' */
    for (ForEach_itr_h = 0; ForEach_itr_h < 1; ForEach_itr_h++) {
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_p =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_b =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_o = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_a = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Z0 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum6 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].RelationalOperator_o =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].DataTypeConversion_bj =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product_p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].SumofElements_e = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].deltaTheta = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].antiswaybartorque = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[0] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[0]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[0]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector5[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector3[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector6[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector4[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].AngleTangentLimit[1] =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction1[1]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].TrigonometricFunction2[1]
        = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Product3[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum4[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Selector2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_n[ForEach_itr_h].Sum5[1] = 0.0;
    }

    /* End of Start for SubSystem: '<S182>/For Each Axle With Anti-Sway' */

    /* Start for Iterator SubSystem: '<S135>/For each track and axle combination calculate suspension forces and moments' */
    for (ForEach_itr_hh = 0; ForEach_itr_hh < 2; ForEach_itr_hh++) {
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_p =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_m =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_f = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Abs = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_dj = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtSelector3Inport1[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtSelector3Inport1[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_j =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_c =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_d =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_l =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_m = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product4 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add4 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].HeightSignConvention =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum2_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_l =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_d =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_mz = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_b =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_k =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_bk = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_b = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_c =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_a =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_j = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_a =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_p =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_i = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product3_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_g =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_f =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_c = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_e = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add2 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product5_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Add1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_a = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sign1 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product2 = 0.0;
      for (i = 0; i < 6; i++) {
        CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
          TmpSignalConversionAtImpAsg_InsertedFor_Info_at_inport_0Inport1[i] =
          0.0;
      }

      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].RelationalOperator_e =
        false;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].DataTypeConversion_fh =
        0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_i = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].SumofElements_fu = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_a = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_e = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleForceSign = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector1_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector_d = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].VehicleHeight = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum_h = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product_l5 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].UnaryMinus = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector2_g = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Product1_p = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum3 = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_o[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Sum1_n[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].
        TmpSignalConversionAtImpAsg_InsertedFor_WhlAng_at_inport_0Inpor[2] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape19[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Reshape19[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Selector3_e = 0.0;

      /* Start for Atomic SubSystem: '<S172>/Max stop reached' */
      CAVE_MachE_sil_test_Maxstopreached_Start
        (&CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Maxstopreached);

      /* End of Start for SubSystem: '<S172>/Max stop reached' */

      /* Start for Atomic SubSystem: '<S172>/Min stop reached' */
      CAVE_MachE_sil_test_Minstopreached_Start
        (&CAVE_MachE_sil_test_B.CoreSubsys_p[ForEach_itr_hh].Minstopreached);

      /* End of Start for SubSystem: '<S172>/Min stop reached' */
    }

    /* End of Start for SubSystem: '<S135>/For each track and axle combination calculate suspension forces and moments' */

    /* Start for Iterator SubSystem: '<S246>/For Each Subsystem' */
    for (ForEach_itr_f = 0; ForEach_itr_f < 1; ForEach_itr_f++) {
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[0] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[1] = 0.0;
      CAVE_MachE_sil_test_B.CoreSubsys_b[ForEach_itr_f].Product[2] = 0.0;
    }

    /* End of Start for SubSystem: '<S246>/For Each Subsystem' */
  }

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr_c;
    real_T y;
    int32_T i;

    /* InitializeConditions for Integrator: '<S218>/xe,ye,ze' */
    CAVE_MachE_sil_test_X.xeyeze_CSTATE[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_Xe_o[0];

    /* InitializeConditions for Integrator: '<S230>/phi theta psi' */
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_eul_o[0];

    /* InitializeConditions for Integrator: '<S218>/ub,vb,wb' */
    CAVE_MachE_sil_test_X.ubvbwb_CSTATE[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_xbdot_o[0];

    /* InitializeConditions for Integrator: '<S218>/xe,ye,ze' */
    CAVE_MachE_sil_test_X.xeyeze_CSTATE[1] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_Xe_o[1];

    /* InitializeConditions for Integrator: '<S230>/phi theta psi' */
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[1] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_eul_o[1];

    /* InitializeConditions for Integrator: '<S218>/ub,vb,wb' */
    CAVE_MachE_sil_test_X.ubvbwb_CSTATE[1] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_xbdot_o[1];

    /* InitializeConditions for Integrator: '<S218>/xe,ye,ze' */
    CAVE_MachE_sil_test_X.xeyeze_CSTATE[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_Xe_o[2];

    /* InitializeConditions for Integrator: '<S230>/phi theta psi' */
    CAVE_MachE_sil_test_X.phithetapsi_CSTATE[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_eul_o[2];

    /* InitializeConditions for Integrator: '<S218>/ub,vb,wb' */
    CAVE_MachE_sil_test_X.ubvbwb_CSTATE[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_xbdot_o[2];

    /* InitializeConditions for Integrator: '<S9>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE =
      CAVE_MachE_sil_test_P.Integrator_IC;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn3' */
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn1' */
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn2' */
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn4' */
    CAVE_MachE_sil_test_X.TransferFcn4_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn' */
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn9' */
    CAVE_MachE_sil_test_X.TransferFcn9_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn10' */
    CAVE_MachE_sil_test_X.TransferFcn10_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn11' */
    CAVE_MachE_sil_test_X.TransferFcn11_CSTATE = 0.0;

    /* InitializeConditions for Memory: '<S30>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput =
      CAVE_MachE_sil_test_P.Memory_InitialCondition;

    /* InitializeConditions for Memory: '<S30>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput =
      CAVE_MachE_sil_test_P.Memory1_InitialCondition;

    /* InitializeConditions for Integrator: '<S39>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_k =
      CAVE_MachE_sil_test_P.Integrator_IC_m;

    /* InitializeConditions for TransferFcn: '<S4>/Transfer Fcn' */
    CAVE_MachE_sil_test_X.TransferFcn_CSTATE_i = 0.0;

    /* InitializeConditions for TransferFcn: '<S4>/Transfer Fcn1' */
    CAVE_MachE_sil_test_X.TransferFcn1_CSTATE_i = 0.0;

    /* InitializeConditions for TransferFcn: '<S4>/Transfer Fcn2' */
    CAVE_MachE_sil_test_X.TransferFcn2_CSTATE_h = 0.0;

    /* InitializeConditions for TransferFcn: '<S4>/Transfer Fcn3' */
    CAVE_MachE_sil_test_X.TransferFcn3_CSTATE_h = 0.0;

    /* InitializeConditions for Memory: '<S12>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_f =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_p;

    /* InitializeConditions for Integrator: '<S23>/Integrator Limited' */
    if (rtmIsFirstInitCond(CAVE_MachE_sil_test_M)) {
      CAVE_MachE_sil_test_X.IntegratorLimited_CSTATE = 0.0;
    }

    CAVE_MachE_sil_test_DW.IntegratorLimited_IWORK = 1;

    /* End of InitializeConditions for Integrator: '<S23>/Integrator Limited' */

    /* InitializeConditions for Integrator: '<S350>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_p =
      CAVE_MachE_sil_test_P.Integrator_IC_f;

    /* InitializeConditions for Integrator: '<S347>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_o =
      CAVE_MachE_sil_test_P.Integrator_IC_a;

    /* InitializeConditions for SecondOrderIntegrator: '<S345>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE[1] =
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE = 0;

    /* InitializeConditions for Integrator: '<S375>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_c =
      CAVE_MachE_sil_test_P.Integrator_IC_fs;

    /* InitializeConditions for Integrator: '<S372>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_ch =
      CAVE_MachE_sil_test_P.Integrator_IC_k;

    /* InitializeConditions for SecondOrderIntegrator: '<S370>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_l[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_l[1] =
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_d = 0;

    /* InitializeConditions for Integrator: '<S400>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_d =
      CAVE_MachE_sil_test_P.Integrator_IC_e;

    /* InitializeConditions for Integrator: '<S397>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_b =
      CAVE_MachE_sil_test_P.Integrator_IC_mh;

    /* InitializeConditions for SecondOrderIntegrator: '<S395>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_ln[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_ln[1] =
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_h = 0;

    /* InitializeConditions for Integrator: '<S425>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_a =
      CAVE_MachE_sil_test_P.Integrator_IC_d;

    /* InitializeConditions for Integrator: '<S422>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_g =
      CAVE_MachE_sil_test_P.Integrator_IC_eh;

    /* InitializeConditions for SecondOrderIntegrator: '<S420>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_j[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_j[1] =
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_l = 0;

    /* InitializeConditions for Integrator: '<S80>/omega wheel' */
    CAVE_MachE_sil_test_X.omegaWheel = CAVE_MachE_sil_test_P.omegawheel_IC;

    /* InitializeConditions for Integrator: '<S81>/omega wheel' */
    CAVE_MachE_sil_test_X.omegaWheel_j = CAVE_MachE_sil_test_P.omegawheel_IC_g;

    /* InitializeConditions for Integrator: '<S82>/omega wheel' */
    CAVE_MachE_sil_test_X.omegaWheel_d = CAVE_MachE_sil_test_P.omegawheel_IC_gv;

    /* InitializeConditions for Integrator: '<S83>/omega wheel' */
    CAVE_MachE_sil_test_X.omegaWheel_jl = CAVE_MachE_sil_test_P.omegawheel_IC_m;

    /* InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator4' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator4_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator5' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator5_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator6' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator6_IC;

    /* InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator7' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator7_IC;

    /* InitializeConditions for Memory: '<S72>/Memory3' */
    CAVE_MachE_sil_test_DW.Memory3_PreviousInput =
      CAVE_MachE_sil_test_P.Memory3_InitialCondition;

    /* InitializeConditions for Memory: '<S72>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_f =
      CAVE_MachE_sil_test_P.Memory1_InitialCondition_f;

    /* InitializeConditions for Memory: '<S72>/Memory2' */
    CAVE_MachE_sil_test_DW.Memory2_PreviousInput =
      CAVE_MachE_sil_test_P.Memory2_InitialCondition;

    /* InitializeConditions for Memory: '<S72>/Memory4' */
    CAVE_MachE_sil_test_DW.Memory4_PreviousInput =
      CAVE_MachE_sil_test_P.Memory4_InitialCondition;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator4' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE_k =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator4_IC_o;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator5' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_DSTATE_a =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator5_IC_j;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator6' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_DSTATE_j =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator6_IC_f;

    /* InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator7' */
    CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_DSTATE_i =
      CAVE_MachE_sil_test_P.DiscreteTimeIntegrator7_IC_o;

    /* InitializeConditions for Memory: '<S78>/Memory3' */
    CAVE_MachE_sil_test_DW.Memory3_PreviousInput_c =
      CAVE_MachE_sil_test_P.Memory3_InitialCondition_j;

    /* InitializeConditions for Memory: '<S78>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_d =
      CAVE_MachE_sil_test_P.Memory1_InitialCondition_a;

    /* InitializeConditions for Memory: '<S78>/Memory2' */
    CAVE_MachE_sil_test_DW.Memory2_PreviousInput_f =
      CAVE_MachE_sil_test_P.Memory2_InitialCondition_k;

    /* InitializeConditions for Memory: '<S78>/Memory4' */
    CAVE_MachE_sil_test_DW.Memory4_PreviousInput_h =
      CAVE_MachE_sil_test_P.Memory4_InitialCondition_h;

    /* InitializeConditions for Memory: '<S98>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_g =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_b;

    /* InitializeConditions for Memory: '<S99>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_p =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_pw;

    /* InitializeConditions for Memory: '<S100>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_h =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_l;

    /* InitializeConditions for Memory: '<S101>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_e =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_bd;

    /* InitializeConditions for SecondOrderIntegrator: '<S334>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_m[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_m[1] =
      CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_a = 0;

    /* InitializeConditions for Memory: '<S80>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_o =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_a;

    /* InitializeConditions for SecondOrderIntegrator: '<S335>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_i[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_i[1] =
      CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse1_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_o = 0;

    /* InitializeConditions for Memory: '<S81>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_gr =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_j;

    /* InitializeConditions for SecondOrderIntegrator: '<S336>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_e[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_e[1] =
      CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse2_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_dh = 0;

    /* InitializeConditions for Memory: '<S82>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_b =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_n;

    /* InitializeConditions for SecondOrderIntegrator: '<S337>/Integrator, Second-Order' */
    y = -CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_zo;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_jk[0] = y;
    CAVE_MachE_sil_test_X.IntegratorSecondOrder_CSTATE_jk[1] =
      CAVE_MachE_sil_test_P.VerticalWheelandUnsprungMassResponse3_zdoto;
    CAVE_MachE_sil_test_DW.IntegratorSecondOrder_MODE_hl = 0;

    /* InitializeConditions for Memory: '<S83>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_eq =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_e;

    /* InitializeConditions for UnitDelay: '<S113>/Output' */
    CAVE_MachE_sil_test_DW.Output_DSTATE =
      CAVE_MachE_sil_test_P.Output_InitialCondition;

    /* InitializeConditions for Backlash: '<S130>/Backlash' */
    CAVE_MachE_sil_test_DW.PrevY_e =
      CAVE_MachE_sil_test_P.Backlash_InitialOutput;

    /* InitializeConditions for Integrator: '<S348>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_i =
      CAVE_MachE_sil_test_P.Integrator_IC_o;

    /* InitializeConditions for Integrator: '<S373>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_b0 =
      CAVE_MachE_sil_test_P.Integrator_IC_d3;

    /* InitializeConditions for Integrator: '<S398>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_n =
      CAVE_MachE_sil_test_P.Integrator_IC_b;

    /* InitializeConditions for Integrator: '<S423>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_m =
      CAVE_MachE_sil_test_P.Integrator_IC_l;

    /* InitializeConditions for Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[0] =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_o;

    /* InitializeConditions for RateLimiter: '<S76>/RateLimSpd' */
    CAVE_MachE_sil_test_DW.PrevY[0] = CAVE_MachE_sil_test_P.RateLimSpd_IC;

    /* InitializeConditions for Integrator: '<S310>/Integrator1' */
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[0] =
      CAVE_MachE_sil_test_P.Integrator1_IC;

    /* InitializeConditions for Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[1] =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_o;

    /* InitializeConditions for RateLimiter: '<S76>/RateLimSpd' */
    CAVE_MachE_sil_test_DW.PrevY[1] = CAVE_MachE_sil_test_P.RateLimSpd_IC;

    /* InitializeConditions for Integrator: '<S310>/Integrator1' */
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[1] =
      CAVE_MachE_sil_test_P.Integrator1_IC;

    /* InitializeConditions for Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[2] =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_o;

    /* InitializeConditions for RateLimiter: '<S76>/RateLimSpd' */
    CAVE_MachE_sil_test_DW.PrevY[2] = CAVE_MachE_sil_test_P.RateLimSpd_IC;

    /* InitializeConditions for Integrator: '<S310>/Integrator1' */
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[2] =
      CAVE_MachE_sil_test_P.Integrator1_IC;

    /* InitializeConditions for Memory: '<S71>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_j[3] =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_o;

    /* InitializeConditions for RateLimiter: '<S76>/RateLimSpd' */
    CAVE_MachE_sil_test_DW.PrevY[3] = CAVE_MachE_sil_test_P.RateLimSpd_IC;

    /* InitializeConditions for Integrator: '<S310>/Integrator1' */
    CAVE_MachE_sil_test_X.Integrator1_CSTATE[3] =
      CAVE_MachE_sil_test_P.Integrator1_IC;

    /* InitializeConditions for Integrator: '<S311>/Integrator1' */
    for (i = 0; i < 12; i++) {
      CAVE_MachE_sil_test_X.Integrator1_CSTATE_m[i] =
        CAVE_MachE_sil_test_P.Integrator1_IC_k;
    }

    /* End of InitializeConditions for Integrator: '<S311>/Integrator1' */

    /* InitializeConditions for Integrator: '<S218>/p,q,r ' */
    CAVE_MachE_sil_test_X.pqr_CSTATE[0] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_p_o[0];
    CAVE_MachE_sil_test_X.pqr_CSTATE[1] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_p_o[1];
    CAVE_MachE_sil_test_X.pqr_CSTATE[2] =
      CAVE_MachE_sil_test_P.VehicleBody6DOF1_p_o[2];

    /* InitializeConditions for Memory: '<S141>/Memory1' */
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[0] =
      CAVE_MachE_sil_test_P.Memory1_InitialCondition_p;
    CAVE_MachE_sil_test_DW.Memory1_PreviousInput_n[1] =
      CAVE_MachE_sil_test_P.Memory1_InitialCondition_p;

    /* InitializeConditions for Memory: '<S141>/Memory' */
    CAVE_MachE_sil_test_DW.Memory_PreviousInput_b0 =
      CAVE_MachE_sil_test_P.Memory_InitialCondition_jw;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn5' */
    CAVE_MachE_sil_test_X.TransferFcn5_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn6' */
    CAVE_MachE_sil_test_X.TransferFcn6_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn7' */
    CAVE_MachE_sil_test_X.TransferFcn7_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S120>/Transfer Fcn8' */
    CAVE_MachE_sil_test_X.TransferFcn8_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S226>/Integrator' */
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[0] =
      CAVE_MachE_sil_test_P.Integrator_IC_fb[0];
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[1] =
      CAVE_MachE_sil_test_P.Integrator_IC_fb[1];
    CAVE_MachE_sil_test_X.Integrator_CSTATE_j[2] =
      CAVE_MachE_sil_test_P.Integrator_IC_fb[2];

    /* SystemInitialize for Merge: '<S39>/Merge' */
    CAVE_MachE_sil_test_B.Merge = CAVE_MachE_sil_test_P.Merge_InitialOutput;

    /* SystemInitialize for Chart: '<S346>/LockUp' */
    CAVE_MachE_sil_test_LockUp_Init(&CAVE_MachE_sil_test_B.sf_LockUp,
      &CAVE_MachE_sil_test_DW.sf_LockUp, &CAVE_MachE_sil_test_P.sf_LockUp,
      &CAVE_MachE_sil_test_X.sf_LockUp,
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF1_omegao);

    /* SystemInitialize for Chart: '<S371>/LockUp' */
    CAVE_MachE_sil_test_LockUp_Init(&CAVE_MachE_sil_test_B.sf_LockUp_n,
      &CAVE_MachE_sil_test_DW.sf_LockUp_n, &CAVE_MachE_sil_test_P.sf_LockUp_n,
      &CAVE_MachE_sil_test_X.sf_LockUp_n,
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF2_omegao);

    /* SystemInitialize for Chart: '<S396>/LockUp' */
    CAVE_MachE_sil_test_LockUp_Init(&CAVE_MachE_sil_test_B.sf_LockUp_h,
      &CAVE_MachE_sil_test_DW.sf_LockUp_h, &CAVE_MachE_sil_test_P.sf_LockUp_h,
      &CAVE_MachE_sil_test_X.sf_LockUp_h,
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF3_omegao);

    /* SystemInitialize for Chart: '<S421>/LockUp' */
    CAVE_MachE_sil_test_LockUp_Init(&CAVE_MachE_sil_test_B.sf_LockUp_c,
      &CAVE_MachE_sil_test_DW.sf_LockUp_c, &CAVE_MachE_sil_test_P.sf_LockUp_c,
      &CAVE_MachE_sil_test_X.sf_LockUp_c,
      CAVE_MachE_sil_test_P.CombinedSlipWheel2DOF4_omegao);

    /* SystemInitialize for Chart: '<S102>/Band-Aid' */
    CAVE_MachE_sil_test_DW.is_Running = CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;
    CAVE_MachE_sil_test_DW.is_SpeedMode =
      CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;
    CAVE_MachE_sil_test_DW.temporalCounter_i1 = 0U;
    CAVE_MachE_sil_test_DW.is_active_c9_CAVE_MachE_sil_test = 0U;
    CAVE_MachE_sil_test_DW.is_c9_CAVE_MachE_sil_test =
      CAVE_MachE_sil_test_IN_NO_ACTIVE_CHILD_k;

    /* SystemInitialize for Iterator SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */
    for (ForEach_itr_c = 0; ForEach_itr_c < 1; ForEach_itr_c++) {
      /* InitializeConditions for Integrator: '<S154>/ ' */
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c]._CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m._IC;

      /* InitializeConditions for Integrator: '<S150>/cg coordinates' */
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[0] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.cgcoordinates_IC;
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[1] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.cgcoordinates_IC;
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].cgcoordinates_CSTATE[2] =
        CAVE_MachE_sil_test_P.CoreSubsys_m.cgcoordinates_IC;

      /* InitializeConditions for Integrator: '<S148>/Vz' */
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vz_CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Vz_IC;

      /* InitializeConditions for Integrator: '<S148>/Vy1' */
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy1_CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Vy1_IC;

      /* InitializeConditions for Integrator: '<S148>/Vy' */
      CAVE_MachE_sil_test_X.CoreSubsys_m[ForEach_itr_c].Vy_CSTATE =
        CAVE_MachE_sil_test_P.CoreSubsys_m.Vy_IC;
    }

    /* End of SystemInitialize for SubSystem: '<S135>/For each axle calculate axle cg positions and velocities' */

    /* InitializeConditions for root-level periodic continuous states */
    {
      int_T rootPeriodicContStateIndices[3] = { 3, 4, 5 };

      real_T rootPeriodicContStateRanges[6] = { -3.1415926535897931,
        3.1415926535897931, -3.1415926535897931, 3.1415926535897931,
        -3.1415926535897931, 3.1415926535897931 };

      (void) memcpy((void*)CAVE_MachE_sil_test_PeriodicIndX,
                    rootPeriodicContStateIndices,
                    3*sizeof(int_T));
      (void) memcpy((void*)CAVE_MachE_sil_test_PeriodicRngX,
                    rootPeriodicContStateRanges,
                    6*sizeof(real_T));
    }

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond(CAVE_MachE_sil_test_M)) {
      rtmSetFirstInitCond(CAVE_MachE_sil_test_M, 0);
    }
  }

  /* Enable for DiscreteIntegrator: '<S72>/Discrete-Time Integrator4' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE = 1U;

  /* Enable for DiscreteIntegrator: '<S72>/Discrete-Time Integrator5' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE = 1U;

  /* Enable for DiscreteIntegrator: '<S72>/Discrete-Time Integrator6' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE = 1U;

  /* Enable for DiscreteIntegrator: '<S72>/Discrete-Time Integrator7' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE = 1U;

  /* Enable for DiscreteIntegrator: '<S78>/Discrete-Time Integrator4' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE_b = 1U;

  /* Enable for DiscreteIntegrator: '<S78>/Discrete-Time Integrator5' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator5_SYSTEM_ENABLE_g = 1U;

  /* Enable for DiscreteIntegrator: '<S78>/Discrete-Time Integrator6' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator6_SYSTEM_ENABLE_p = 1U;

  /* Enable for DiscreteIntegrator: '<S78>/Discrete-Time Integrator7' */
  CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator7_SYSTEM_ENABLE_g = 1U;
}

/* Model terminate function */
void CAVE_MachE_sil_test_terminate(void)
{
  /* (no terminate code required) */
}
