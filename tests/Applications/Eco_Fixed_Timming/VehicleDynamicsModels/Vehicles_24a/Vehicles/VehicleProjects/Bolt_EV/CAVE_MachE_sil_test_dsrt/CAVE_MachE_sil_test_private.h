/*
 * CAVE_MachE_sil_test_private.h
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

#ifndef RTW_HEADER_CAVE_MachE_sil_test_private_h_
#define RTW_HEADER_CAVE_MachE_sil_test_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "CAVE_MachE_sil_test.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_mrdivide_U1d1x3_U2d3x3_Yd1x3_snf(const real_T u0[3], const real_T
  u1[9], real_T y[3]);
extern real_T look1_binlcpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
extern real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
extern real_T look1_binlcapw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
extern real_T look2_binlcapw(real_T u0, real_T u1, const real_T bp0[], const
  real_T bp1[], const real_T table[], const uint32_T maxIndex[], uint32_T stride);
extern void CAVE_MachE_sil_test_MATLABFunction(real_T rtu_axleTorq, real_T
  rtu_wheelSpd, B_MATLABFunction_CAVE_MachE_sil_test_T *localB);
extern void CAVE_MachE_sil_test_Defloater(real_T rtu_defloatMe,
  B_Defloater_CAVE_MachE_sil_test_T *localB);
extern void CAVE_MachE_sil_test_Minstopreached_Start
  (B_Minstopreached_CAVE_MachE_sil_test_T *localB);
extern void CAVE_MachE_sil_test_Minstopreached(RT_MODEL_CAVE_MachE_sil_test_T *
  const CAVE_MachE_sil_test_M, real_T rtu_x, real_T rtu_xdot, real_T rtu_k,
  real_T rtu_c, real_T rtu_xmax, B_Minstopreached_CAVE_MachE_sil_test_T *localB,
  P_Minstopreached_CAVE_MachE_sil_test_T *localP, real_T rtp_Hmax);
extern void CAVE_MachE_sil_test_Maxstopreached_Start
  (B_Maxstopreached_CAVE_MachE_sil_test_T *localB);
extern void CAVE_MachE_sil_test_Maxstopreached(RT_MODEL_CAVE_MachE_sil_test_T *
  const CAVE_MachE_sil_test_M, real_T rtu_x, real_T rtu_xdot, real_T rtu_k,
  real_T rtu_c, real_T rtu_xmax, B_Maxstopreached_CAVE_MachE_sil_test_T *localB,
  P_Maxstopreached_CAVE_MachE_sil_test_T *localP, real_T rtp_Hmax);
extern void CAVE_MachE_sil_test_MagicTireConstInput(real_T rtu_Omega, real_T
  rtu_Vx, real_T rtu_Vy, real_T rtu_psidot, real_T rtu_Gamma, real_T rtu_TirePrs,
  const real_T rtu_ScaleFactors[27], real_T rtu_rhoz, real_T rtu_Fx_ext, real_T
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
  real_T rtp_VXLOW, real_T rtp_WIDTH);
extern void CAVE_MachE_sil_test_LockUp_Init(B_LockUp_CAVE_MachE_sil_test_T
  *localB, DW_LockUp_CAVE_MachE_sil_test_T *localDW,
  P_LockUp_CAVE_MachE_sil_test_T *localP, X_LockUp_CAVE_MachE_sil_test_T *localX,
  real_T rtp_omegao);
extern void CAVE_MachE_sil_test_LockUp_Deriv(B_LockUp_CAVE_MachE_sil_test_T
  *localB, DW_LockUp_CAVE_MachE_sil_test_T *localDW,
  XDot_LockUp_CAVE_MachE_sil_test_T *localXdot);
extern void CAVE_MachE_sil_test_LockUp(RT_MODEL_CAVE_MachE_sil_test_T * const
  CAVE_MachE_sil_test_M, real_T rtu_Tout, real_T rtu_Tfmaxs, real_T rtu_Tfmaxk,
  B_LockUp_CAVE_MachE_sil_test_T *localB, DW_LockUp_CAVE_MachE_sil_test_T
  *localDW, P_LockUp_CAVE_MachE_sil_test_T *localP,
  X_LockUp_CAVE_MachE_sil_test_T *localX, real_T rtp_omegao, real_T rtp_br,
  real_T rtp_Iyy, real_T rtp_OmegaTol);

/* private model entry point functions */
extern void CAVE_MachE_sil_test_derivatives(void);

#endif                           /* RTW_HEADER_CAVE_MachE_sil_test_private_h_ */
