/*
 * CAVE_MachE_dSPACE_250912_types.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "CAVE_MachE_dSPACE_250912".
 *
 * Model version              : 10.86
 * Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
 * C source code generated on : Sun Sep 21 17:10:19 2025
 *
 * Target selection: dsrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Custom Processor->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef CAVE_MachE_dSPACE_250912_types_h_
#define CAVE_MachE_dSPACE_250912_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_VehDataBus_
#define DEFINED_TYPEDEF_FOR_VehDataBus_

typedef struct {
  uint8_T id[50];
  real_T idLength;
  uint8_T type[50];
  real_T typeLength;
  uint8_T vehicleClass[50];
  real_T vehicleClassLength;
  real_T speed;
  real_T acceleration;
  real_T positionX;
  real_T positionY;
  real_T positionZ;
  real_T heading;
  real_T color;
  uint8_T linkId[50];
  real_T linkIdLength;
  real_T laneId;
  real_T distanceTravel;
  real_T speedDesired;
  real_T accelerationDesired;
  real_T hasPrecedingVehicle;
  uint8_T precedingVehicleId[50];
  real_T precedingVehicleIdLength;
  real_T precedingVehicleDistance;
  real_T precedingVehicleSpeed;
  uint8_T signalLightId[50];
  real_T signalLightIdLength;
  real_T signalLightHeadId;
  real_T signalLightDistance;
  real_T signalLightColor;
  real_T speedLimit;
  real_T speedLimitNext;
  real_T speedLimitChangeDistance;
  uint8_T linkIdNext[50];
  real_T linkIdNextLength;
  real_T grade;
  real_T activeLaneChange;
} VehDataBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_RLXElOtgrzvJMUIzjpiLaD_
#define DEFINED_TYPEDEF_FOR_struct_RLXElOtgrzvJMUIzjpiLaD_

typedef struct {
  real_T warmupTime;
  real_T speedInit;
  real_T tLookahead;
  real_T smoothWindow;
  real_T speedSource;
} struct_RLXElOtgrzvJMUIzjpiLaD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_m2VjwNiXoluKspK4Fr7zNG_
#define DEFINED_TYPEDEF_FOR_struct_m2VjwNiXoluKspK4Fr7zNG_

typedef struct {
  real_T VehSpd;
  real_T AxleTorq;
  real_T BattSoc;
  real_T BattCurr;
} struct_m2VjwNiXoluKspK4Fr7zNG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_H6OzwuYMZP7H9bFCBCIB9C_
#define DEFINED_TYPEDEF_FOR_struct_H6OzwuYMZP7H9bFCBCIB9C_

typedef struct {
  real_T Time[600];
  real_T Speed[600];
} struct_H6OzwuYMZP7H9bFCBCIB9C;

#endif

#ifndef struct_tag_ss9j38eBfDiWAKQLv2egin
#define struct_tag_ss9j38eBfDiWAKQLv2egin

struct tag_ss9j38eBfDiWAKQLv2egin
{
  real_T id;
  real_T type;
  real_T vehicleClass;
  real_T speed;
  real_T acceleration;
  real_T positionX;
  real_T positionY;
  real_T positionZ;
  real_T heading;
  real_T color;
  real_T linkId;
  real_T laneId;
  real_T distanceTravel;
  real_T speedDesired;
  real_T accelerationDesired;
  real_T hasPrecedingVehicle;
  real_T precedingVehicleId;
  real_T precedingVehicleDistance;
  real_T precedingVehicleSpeed;
  real_T signalLightId;
  real_T signalLightHeadId;
  real_T signalLightDistance;
  real_T signalLightColor;
  real_T speedLimit;
  real_T speedLimitNext;
  real_T speedLimitChangeDistance;
  real_T linkIdNext;
  real_T grade;
  real_T activeLaneChange;
};

#endif                                 /* struct_tag_ss9j38eBfDiWAKQLv2egin */

#ifndef typedef_ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T
#define typedef_ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T

typedef struct tag_ss9j38eBfDiWAKQLv2egin
  ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T;

#endif           /* typedef_ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T */

#ifndef struct_tag_W0lk2ZYCukCHtnuvU0BHXF
#define struct_tag_W0lk2ZYCukCHtnuvU0BHXF

struct tag_W0lk2ZYCukCHtnuvU0BHXF
{
  int32_T isInitialized;
  real_T initialSpeed;
  real_T speedLookAheadHorizon;
  real_T speedUpperBound;
  real_T timePrevious;
  real_T speedPrevious;
  real_T timeNext;
  real_T speedNext;
  real_T connectionState;
  real_T timeSimulator;
  real_T simulatorStartTime;
  real_T timeStepSimulator;
  real_T timeStepSimulatorPrevious;
  real_T timeReceivePrevious;
  real_T simState;
  real_T RealSimDelay;
  real_T speedLookAheadPrevious;
  real_T accelerationDesiredPrevious;
  real_T speedInterpolationMode;
};

#endif                                 /* struct_tag_W0lk2ZYCukCHtnuvU0BHXF */

#ifndef typedef_RealSimInterpSpeed_CAVE_MachE_dSPACE_250912_T
#define typedef_RealSimInterpSpeed_CAVE_MachE_dSPACE_250912_T

typedef struct tag_W0lk2ZYCukCHtnuvU0BHXF
  RealSimInterpSpeed_CAVE_MachE_dSPACE_250912_T;

#endif               /* typedef_RealSimInterpSpeed_CAVE_MachE_dSPACE_250912_T */

#ifndef struct_tag_NxiLgMg7tFtpMJqY0pHxpD
#define struct_tag_NxiLgMg7tFtpMJqY0pHxpD

struct tag_NxiLgMg7tFtpMJqY0pHxpD
{
  int32_T isInitialized;
  real_T VehicleMessageFieldDefInputVec[29];
  ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T VehicleMessageFieldDef;
  real_T msgHeaderSize;
  real_T msgEachHeaderSize;
  real_T vehMsgIdentifer;
  real_T enableDebug;
  VehDataBus VehicleDataEmpty;
  real_T simStatePrevious;
  real_T tPrevious;
  real_T isVehicleInNetworkPrevious;
  VehDataBus VehicleDataPrevious;
};

#endif                                 /* struct_tag_NxiLgMg7tFtpMJqY0pHxpD */

#ifndef typedef_RealSimDepack_CAVE_MachE_dSPACE_250912_T
#define typedef_RealSimDepack_CAVE_MachE_dSPACE_250912_T

typedef struct tag_NxiLgMg7tFtpMJqY0pHxpD
  RealSimDepack_CAVE_MachE_dSPACE_250912_T;

#endif                    /* typedef_RealSimDepack_CAVE_MachE_dSPACE_250912_T */

#ifndef struct_tag_BwCILnACiDS8DxUhsaFI7D
#define struct_tag_BwCILnACiDS8DxUhsaFI7D

struct tag_BwCILnACiDS8DxUhsaFI7D
{
  int32_T isInitialized;
  real_T VehicleMessageFieldDefInputVec[29];
  ss9j38eBfDiWAKQLv2egin_CAVE_MachE_dSPACE_250912_T VehicleMessageFieldDef;
  real_T msgHeaderSize;
  real_T msgEachHeaderSize;
};

#endif                                 /* struct_tag_BwCILnACiDS8DxUhsaFI7D */

#ifndef typedef_RealSimPack_CAVE_MachE_dSPACE_250912_T
#define typedef_RealSimPack_CAVE_MachE_dSPACE_250912_T

typedef struct tag_BwCILnACiDS8DxUhsaFI7D RealSimPack_CAVE_MachE_dSPACE_250912_T;

#endif                      /* typedef_RealSimPack_CAVE_MachE_dSPACE_250912_T */

/* Parameters for system: '<S216>/If Action Subsystem1' */
typedef struct P_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T_
  P_IfActionSubsystem1_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S274>/For each axle and track calculate suspension and wheel positions and velocities' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S274>/For each axle calculate axle cg positions and velocities' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_f_T;

/* Parameters for system: '<S314>/Min stop reached' */
typedef struct P_Minstopreached_CAVE_MachE_dSPACE_250912_T_
  P_Minstopreached_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S314>/Max stop reached' */
typedef struct P_Maxstopreached_CAVE_MachE_dSPACE_250912_T_
  P_Maxstopreached_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S274>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_j_T;

/* Parameters for system: '<S324>/For Each Axle With Anti-Sway' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_e_T;

/* Parameters for system: '<S279>/For each track and axle combination calculate suspension forces and moments' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_h_T;

/* Parameters for system: '<S512>/Magic Tire Const Input' */
typedef struct P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T_
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S526>/detectSlip' */
typedef struct P_detectSlip_CAVE_MachE_dSPACE_250912_T_
  P_detectSlip_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S526>/detectLockup' */
typedef struct P_detectLockup_CAVE_MachE_dSPACE_250912_T_
  P_detectLockup_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S526>/Locked' */
typedef struct P_Locked_CAVE_MachE_dSPACE_250912_T_
  P_Locked_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S526>/Slipping' */
typedef struct P_Slipping_CAVE_MachE_dSPACE_250912_T_
  P_Slipping_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S525>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_T;

/* Parameters for system: '<S521>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_l_T;

/* Parameters for system: '<S553>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_i_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_i_T;

/* Parameters for system: '<S549>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_ls_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_ls_T;

/* Parameters for system: '<S581>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_a_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_a_T;

/* Parameters for system: '<S577>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_a_T;

/* Parameters for system: '<S609>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_l_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_l_T;

/* Parameters for system: '<S605>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_d_T;

/* Parameters for system: '<S624>/Magic Tire Const Input' */
typedef struct P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T_
  P_MagicTireConstInput_CAVE_MachE_dSPACE_250912_i_T;

/* Parameters for system: '<S637>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_h_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_h_T;

/* Parameters for system: '<S633>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_jo_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_jo_T;

/* Parameters for system: '<S665>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_p_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_p_T;

/* Parameters for system: '<S661>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_dg_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_dg_T;

/* Parameters for system: '<S693>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_g_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_g_T;

/* Parameters for system: '<S689>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_c_T;

/* Parameters for system: '<S721>/Clutch' */
typedef struct P_Clutch_CAVE_MachE_dSPACE_250912_pj_T_
  P_Clutch_CAVE_MachE_dSPACE_250912_pj_T;

/* Parameters for system: '<S717>/Clutch Scalar Parameters' */
typedef struct P_CoreSubsys_CAVE_MachE_dSPACE_250912_f4_T_
  P_CoreSubsys_CAVE_MachE_dSPACE_250912_f4_T;

/* Parameters (default storage) */
typedef struct P_CAVE_MachE_dSPACE_250912_T_ P_CAVE_MachE_dSPACE_250912_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_CAVE_MachE_dSPACE_250912_T
  RT_MODEL_CAVE_MachE_dSPACE_250912_T;

#endif                                 /* CAVE_MachE_dSPACE_250912_types_h_ */
