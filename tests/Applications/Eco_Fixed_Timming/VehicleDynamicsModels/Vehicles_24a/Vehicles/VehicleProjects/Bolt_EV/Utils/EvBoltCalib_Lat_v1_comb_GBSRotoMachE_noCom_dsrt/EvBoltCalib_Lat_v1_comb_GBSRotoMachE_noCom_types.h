/*
 * EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_types.h
 *
 * Code generation for model "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom".
 *
 * Model version              : 1.29
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Mon Jul 10 17:21:16 2023
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_types_h_
#define RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_types_h_
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

#ifndef DEFINED_TYPEDEF_FOR_struct_LqXVUGdvzWUJgx4oCacWrF_
#define DEFINED_TYPEDEF_FOR_struct_LqXVUGdvzWUJgx4oCacWrF_

typedef struct {
  real_T Time[16001];
  real_T Speed[16001];
} struct_LqXVUGdvzWUJgx4oCacWrF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_KVvNvmPYkDwFa3qoVeWWJC_
#define DEFINED_TYPEDEF_FOR_struct_KVvNvmPYkDwFa3qoVeWWJC_

typedef struct {
  real_T Time[241687];
  real_T Speed[241687];
} struct_KVvNvmPYkDwFa3qoVeWWJC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_H6OzwuYMZP7H9bFCBCIB9C_
#define DEFINED_TYPEDEF_FOR_struct_H6OzwuYMZP7H9bFCBCIB9C_

typedef struct {
  real_T Time[600];
  real_T Speed[600];
} struct_H6OzwuYMZP7H9bFCBCIB9C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_UDems7iN2Kzz7DJ6EtAQJH_
#define DEFINED_TYPEDEF_FOR_struct_UDems7iN2Kzz7DJ6EtAQJH_

typedef struct {
  real_T Time[3612];
  real_T Speed[3612];
} struct_UDems7iN2Kzz7DJ6EtAQJH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_akRwpZPsceW5LlhYlX5DVF_
#define DEFINED_TYPEDEF_FOR_struct_akRwpZPsceW5LlhYlX5DVF_

typedef struct {
  real_T Time[41037];
  real_T Speed[41037];
} struct_akRwpZPsceW5LlhYlX5DVF;

#endif

/* Parameters for system: '<S304>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Parameters for system: '<S304>/For each axle calculate axle cg positions and velocities' */
typedef struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_g_T_
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_g_T;

/* Parameters for system: '<S341>/Min stop reached' */
typedef struct P_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_
  P_Minstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Parameters for system: '<S341>/Max stop reached' */
typedef struct P_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_
  P_Maxstopreached_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Parameters for system: '<S304>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_d_T_
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_d_T;

/* Parameters for system: '<S351>/For Each Axle With Anti-Sway' */
typedef struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T_
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_e_T;

/* Parameters for system: '<S309>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_a_T_
  P_CoreSubsys_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_a_T;

/* Parameters for system: '<S515>/LockUp' */
typedef struct P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_
  P_LockUp_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Parameters (default storage) */
typedef struct P_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T_
  P_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T
  RT_MODEL_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_T;

#endif      /* RTW_HEADER_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_types_h_ */
