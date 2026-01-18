/*
 * CAVE_MachE_sil_test_types.h
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

#ifndef RTW_HEADER_CAVE_MachE_sil_test_types_h_
#define RTW_HEADER_CAVE_MachE_sil_test_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_m2VjwNiXoluKspK4Fr7zNG_
#define DEFINED_TYPEDEF_FOR_struct_m2VjwNiXoluKspK4Fr7zNG_

typedef struct {
  real_T VehSpd;
  real_T AxleTorq;
  real_T BattSoc;
  real_T BattCurr;
} struct_m2VjwNiXoluKspK4Fr7zNG;

#endif

/* Parameters for system: '<S135>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct P_CoreSubsys_CAVE_MachE_sil_test_T_
  P_CoreSubsys_CAVE_MachE_sil_test_T;

/* Parameters for system: '<S135>/For each axle calculate axle cg positions and velocities' */
typedef struct P_CoreSubsys_CAVE_MachE_sil_test_g_T_
  P_CoreSubsys_CAVE_MachE_sil_test_g_T;

/* Parameters for system: '<S172>/Min stop reached' */
typedef struct P_Minstopreached_CAVE_MachE_sil_test_T_
  P_Minstopreached_CAVE_MachE_sil_test_T;

/* Parameters for system: '<S172>/Max stop reached' */
typedef struct P_Maxstopreached_CAVE_MachE_sil_test_T_
  P_Maxstopreached_CAVE_MachE_sil_test_T;

/* Parameters for system: '<S135>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_CAVE_MachE_sil_test_d_T_
  P_CoreSubsys_CAVE_MachE_sil_test_d_T;

/* Parameters for system: '<S182>/For Each Axle With Anti-Sway' */
typedef struct P_CoreSubsys_CAVE_MachE_sil_test_e_T_
  P_CoreSubsys_CAVE_MachE_sil_test_e_T;

/* Parameters for system: '<S140>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_CAVE_MachE_sil_test_a_T_
  P_CoreSubsys_CAVE_MachE_sil_test_a_T;

/* Parameters for system: '<S346>/LockUp' */
typedef struct P_LockUp_CAVE_MachE_sil_test_T_ P_LockUp_CAVE_MachE_sil_test_T;

/* Parameters (default storage) */
typedef struct P_CAVE_MachE_sil_test_T_ P_CAVE_MachE_sil_test_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_CAVE_MachE_sil_test_T RT_MODEL_CAVE_MachE_sil_test_T;

#endif                             /* RTW_HEADER_CAVE_MachE_sil_test_types_h_ */
