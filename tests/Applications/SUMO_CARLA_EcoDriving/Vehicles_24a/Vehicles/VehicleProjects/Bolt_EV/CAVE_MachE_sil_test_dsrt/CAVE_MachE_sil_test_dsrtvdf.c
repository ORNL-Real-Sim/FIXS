/***************************************************************************

   Source file CAVE_MachE_sil_test_dsrtvdf.c:

   Definition of function that initializes the global TRC pointers

   4.2p1 (11-Feb-2020)
   Tue Aug 22 23:31:18 2023

   Copyright 2023, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "CAVE_MachE_sil_test_dsrtvdf.h"
#include "CAVE_MachE_sil_test.h"
#include "CAVE_MachE_sil_test_private.h"

/* Compiler options to turn off optimization. */
#if !defined(DS_OPTIMIZE_INIT_TRC_POINTERS)
#ifdef _MCCPPC

#pragma options -nOt -nOr -nOi -nOx

#endif

#ifdef __GNUC__

#pragma GCC optimize ("O0")

#endif

#ifdef _MSC_VER

#pragma optimize ("", off)

#endif
#endif

/* Definition of Global pointers to data type transitions (for TRC-file access) */
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_0 = NULL;
volatile real32_T *p_0_CAVE_MachE_sil_test_real32_T_1 = NULL;
volatile uint8_T *p_0_CAVE_MachE_sil_test_uint8_T_2 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_3 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_4 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_5 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_6 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_7 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_8 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_9 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_10 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_11 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_12 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_13 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_14 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_15 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_16 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_17 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_18 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_19 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_20 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_21 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_22 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_23 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_24 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_25 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_26 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_27 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_28 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_29 = NULL;
volatile boolean_T *p_0_CAVE_MachE_sil_test_boolean_T_30 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_31 = NULL;
volatile uint8_T *p_0_CAVE_MachE_sil_test_uint8_T_32 = NULL;
volatile uint8_T *p_0_CAVE_MachE_sil_test_uint8_T_33 = NULL;
volatile uint8_T *p_0_CAVE_MachE_sil_test_uint8_T_34 = NULL;
volatile uint8_T *p_0_CAVE_MachE_sil_test_uint8_T_35 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_36 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_37 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_38 = NULL;
volatile real_T *p_0_CAVE_MachE_sil_test_real_T_39 = NULL;
volatile struct_m2VjwNiXoluKspK4Fr7zNG
  *p_1_CAVE_MachE_sil_test_struct_m2VjwNiXoluKspK4Fr7zNG_0 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_1 = NULL;
volatile uint8_T *p_1_CAVE_MachE_sil_test_uint8_T_2 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_3 = NULL;
volatile uint32_T *p_1_CAVE_MachE_sil_test_uint32_T_4 = NULL;
volatile uint16_T *p_1_CAVE_MachE_sil_test_uint16_T_5 = NULL;
volatile boolean_T *p_1_CAVE_MachE_sil_test_boolean_T_6 = NULL;
volatile uint8_T *p_1_CAVE_MachE_sil_test_uint8_T_7 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_8 = NULL;
volatile boolean_T *p_1_CAVE_MachE_sil_test_boolean_T_9 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_10 = NULL;
volatile boolean_T *p_1_CAVE_MachE_sil_test_boolean_T_11 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_12 = NULL;
volatile boolean_T *p_1_CAVE_MachE_sil_test_boolean_T_13 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_14 = NULL;
volatile boolean_T *p_1_CAVE_MachE_sil_test_boolean_T_15 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_16 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_17 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_18 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_19 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_20 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_21 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_22 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_23 = NULL;
volatile real_T *p_1_CAVE_MachE_sil_test_real_T_24 = NULL;
volatile real_T *p_2_CAVE_MachE_sil_test_real_T_0 = NULL;
volatile int32_T *p_2_CAVE_MachE_sil_test_int32_T_2 = NULL;
volatile uint32_T *p_2_CAVE_MachE_sil_test_uint32_T_3 = NULL;
volatile int_T *p_2_CAVE_MachE_sil_test_int_T_4 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_5 = NULL;
volatile int8_T *p_2_CAVE_MachE_sil_test_int8_T_6 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_7 = NULL;
volatile boolean_T *p_2_CAVE_MachE_sil_test_boolean_T_8 = NULL;
volatile real_T *p_2_CAVE_MachE_sil_test_real_T_9 = NULL;
volatile boolean_T *p_2_CAVE_MachE_sil_test_boolean_T_10 = NULL;
volatile int8_T *p_2_CAVE_MachE_sil_test_int8_T_11 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_12 = NULL;
volatile real_T *p_2_CAVE_MachE_sil_test_real_T_13 = NULL;
volatile boolean_T *p_2_CAVE_MachE_sil_test_boolean_T_14 = NULL;
volatile int8_T *p_2_CAVE_MachE_sil_test_int8_T_15 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_16 = NULL;
volatile real_T *p_2_CAVE_MachE_sil_test_real_T_17 = NULL;
volatile boolean_T *p_2_CAVE_MachE_sil_test_boolean_T_18 = NULL;
volatile int8_T *p_2_CAVE_MachE_sil_test_int8_T_19 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_20 = NULL;
volatile real_T *p_2_CAVE_MachE_sil_test_real_T_21 = NULL;
volatile boolean_T *p_2_CAVE_MachE_sil_test_boolean_T_22 = NULL;
volatile int8_T *p_2_CAVE_MachE_sil_test_int8_T_23 = NULL;
volatile uint8_T *p_2_CAVE_MachE_sil_test_uint8_T_24 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_0 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_1 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_2 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_3 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_4 = NULL;
volatile real_T *p_3_CAVE_MachE_sil_test_real_T_5 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_CAVE_MachE_sil_test_real_T_0 = &CAVE_MachE_sil_test_B.xeyeze[0];
  p_0_CAVE_MachE_sil_test_real32_T_1 = &CAVE_MachE_sil_test_B.DataTypeConversion;
  p_0_CAVE_MachE_sil_test_uint8_T_2 = &CAVE_MachE_sil_test_B.Reserved;
  p_0_CAVE_MachE_sil_test_boolean_T_3 = &CAVE_MachE_sil_test_B.Compare;
  p_0_CAVE_MachE_sil_test_real_T_4 = &CAVE_MachE_sil_test_B.CoreSubsys_pna[3].
    TmpSignalConversionAtsincosInport1[0];
  p_0_CAVE_MachE_sil_test_real_T_5 = &CAVE_MachE_sil_test_B.sf_LockUp_c.Tout;
  p_0_CAVE_MachE_sil_test_boolean_T_6 =
    &CAVE_MachE_sil_test_B.sf_LockUp_c.RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_7 =
    &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_k.Fx;
  p_0_CAVE_MachE_sil_test_real_T_8 = &CAVE_MachE_sil_test_B.sf_LockUp_h.Tout;
  p_0_CAVE_MachE_sil_test_boolean_T_9 =
    &CAVE_MachE_sil_test_B.sf_LockUp_h.RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_10 =
    &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_f.Fx;
  p_0_CAVE_MachE_sil_test_real_T_11 = &CAVE_MachE_sil_test_B.sf_LockUp_n.Tout;
  p_0_CAVE_MachE_sil_test_boolean_T_12 =
    &CAVE_MachE_sil_test_B.sf_LockUp_n.RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_13 =
    &CAVE_MachE_sil_test_B.sf_MagicTireConstInput_j.Fx;
  p_0_CAVE_MachE_sil_test_real_T_14 = &CAVE_MachE_sil_test_B.sf_LockUp.Tout;
  p_0_CAVE_MachE_sil_test_boolean_T_15 =
    &CAVE_MachE_sil_test_B.sf_LockUp.RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_16 =
    &CAVE_MachE_sil_test_B.sf_MagicTireConstInput.Fx;
  p_0_CAVE_MachE_sil_test_real_T_17 = &CAVE_MachE_sil_test_B.CoreSubsys_pn[3].
    TmpSignalConversionAtsincosInport1[0];
  p_0_CAVE_MachE_sil_test_real_T_18 = &CAVE_MachE_sil_test_B.CoreSubsys_b[0].
    Product[0];
  p_0_CAVE_MachE_sil_test_real_T_19 = &CAVE_MachE_sil_test_B.CoreSubsys_d[1].
    TmpSignalConversionAtSelector3Inport1[0];
  p_0_CAVE_MachE_sil_test_boolean_T_20 = &CAVE_MachE_sil_test_B.CoreSubsys_d[1].
    RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_21 = &CAVE_MachE_sil_test_B.CoreSubsys_d[1].
    Maxstopreached.Sum1;
  p_0_CAVE_MachE_sil_test_real_T_22 = &CAVE_MachE_sil_test_B.CoreSubsys_d[1].
    Minstopreached.Sum1;
  p_0_CAVE_MachE_sil_test_real_T_23 = &CAVE_MachE_sil_test_B.CoreSubsys_n[0].
    DataTypeConversion;
  p_0_CAVE_MachE_sil_test_boolean_T_24 = &CAVE_MachE_sil_test_B.CoreSubsys_n[0].
    RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_25 = &CAVE_MachE_sil_test_B.CoreSubsys_p[1].
    DataTypeConversion;
  p_0_CAVE_MachE_sil_test_boolean_T_26 = &CAVE_MachE_sil_test_B.CoreSubsys_p[1].
    RelationalOperator;
  p_0_CAVE_MachE_sil_test_real_T_27 = &CAVE_MachE_sil_test_B.CoreSubsys_p[1].
    Maxstopreached.Sum1;
  p_0_CAVE_MachE_sil_test_real_T_28 = &CAVE_MachE_sil_test_B.CoreSubsys_p[1].
    Minstopreached.Sum1;
  p_0_CAVE_MachE_sil_test_real_T_29 = &CAVE_MachE_sil_test_B.CoreSubsys_m[0].
    Reshape[0];
  p_0_CAVE_MachE_sil_test_boolean_T_30 = &CAVE_MachE_sil_test_B.CoreSubsys_m[0].
    RelationalOperator[0];
  p_0_CAVE_MachE_sil_test_real_T_31 = &CAVE_MachE_sil_test_B.CoreSubsys[1].
    Selector6;
  p_0_CAVE_MachE_sil_test_uint8_T_32 =
    &CAVE_MachE_sil_test_B.sf_Defloater_e.byte1;
  p_0_CAVE_MachE_sil_test_uint8_T_33 =
    &CAVE_MachE_sil_test_B.sf_Defloater_d.byte1;
  p_0_CAVE_MachE_sil_test_uint8_T_34 =
    &CAVE_MachE_sil_test_B.sf_Defloater_l.byte1;
  p_0_CAVE_MachE_sil_test_uint8_T_35 = &CAVE_MachE_sil_test_B.sf_Defloater.byte1;
  p_0_CAVE_MachE_sil_test_real_T_36 =
    &CAVE_MachE_sil_test_B.sf_MATLABFunction_g.switchFlag;
  p_0_CAVE_MachE_sil_test_real_T_37 =
    &CAVE_MachE_sil_test_B.sf_MATLABFunction_j.switchFlag;
  p_0_CAVE_MachE_sil_test_real_T_38 =
    &CAVE_MachE_sil_test_B.sf_MATLABFunction_h.switchFlag;
  p_0_CAVE_MachE_sil_test_real_T_39 =
    &CAVE_MachE_sil_test_B.sf_MATLABFunction.switchFlag;
  p_1_CAVE_MachE_sil_test_struct_m2VjwNiXoluKspK4Fr7zNG_0 =
    &CAVE_MachE_sil_test_P.InitStates[0];
  p_1_CAVE_MachE_sil_test_real_T_1 =
    &CAVE_MachE_sil_test_P.BattChargeLimitMaxPwr;
  p_1_CAVE_MachE_sil_test_uint8_T_2 =
    &CAVE_MachE_sil_test_P.WrapToZero_Threshold;
  p_1_CAVE_MachE_sil_test_real_T_3 =
    &CAVE_MachE_sil_test_P.Interpolatedzerocrossing_tableData[0];
  p_1_CAVE_MachE_sil_test_uint32_T_4 = &CAVE_MachE_sil_test_P.R_maxIndex[0];
  p_1_CAVE_MachE_sil_test_uint16_T_5 = &CAVE_MachE_sil_test_P.ProtocolVer_Value;
  p_1_CAVE_MachE_sil_test_boolean_T_6 =
    &CAVE_MachE_sil_test_P.Memory_InitialCondition_jw;
  p_1_CAVE_MachE_sil_test_uint8_T_7 = &CAVE_MachE_sil_test_P.Bit6_Gain;
  p_1_CAVE_MachE_sil_test_real_T_8 =
    &CAVE_MachE_sil_test_P.sf_LockUp_c.Constant_Value;
  p_1_CAVE_MachE_sil_test_boolean_T_9 = &CAVE_MachE_sil_test_P.sf_LockUp_c.yn_Y0;
  p_1_CAVE_MachE_sil_test_real_T_10 =
    &CAVE_MachE_sil_test_P.sf_LockUp_h.Constant_Value;
  p_1_CAVE_MachE_sil_test_boolean_T_11 =
    &CAVE_MachE_sil_test_P.sf_LockUp_h.yn_Y0;
  p_1_CAVE_MachE_sil_test_real_T_12 =
    &CAVE_MachE_sil_test_P.sf_LockUp_n.Constant_Value;
  p_1_CAVE_MachE_sil_test_boolean_T_13 =
    &CAVE_MachE_sil_test_P.sf_LockUp_n.yn_Y0;
  p_1_CAVE_MachE_sil_test_real_T_14 =
    &CAVE_MachE_sil_test_P.sf_LockUp.Constant_Value;
  p_1_CAVE_MachE_sil_test_boolean_T_15 = &CAVE_MachE_sil_test_P.sf_LockUp.yn_Y0;
  p_1_CAVE_MachE_sil_test_real_T_16 =
    &CAVE_MachE_sil_test_P.CoreSubsys_d.SelectCamberSteeringSlope_AxleNums;
  p_1_CAVE_MachE_sil_test_real_T_17 =
    &CAVE_MachE_sil_test_P.CoreSubsys_d.Maxstopreached.Gain5_Gain;
  p_1_CAVE_MachE_sil_test_real_T_18 =
    &CAVE_MachE_sil_test_P.CoreSubsys_d.Minstopreached.Gain5_Gain;
  p_1_CAVE_MachE_sil_test_real_T_19 =
    &CAVE_MachE_sil_test_P.CoreSubsys_n.AntiSwayArmRadiusByAxle_AxleNums;
  p_1_CAVE_MachE_sil_test_real_T_20 =
    &CAVE_MachE_sil_test_P.CoreSubsys_p.SelectCamberSteeringCenter_AxleNums;
  p_1_CAVE_MachE_sil_test_real_T_21 =
    &CAVE_MachE_sil_test_P.CoreSubsys_p.Maxstopreached.Gain5_Gain;
  p_1_CAVE_MachE_sil_test_real_T_22 =
    &CAVE_MachE_sil_test_P.CoreSubsys_p.Minstopreached.Gain5_Gain;
  p_1_CAVE_MachE_sil_test_real_T_23 =
    &CAVE_MachE_sil_test_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums;
  p_1_CAVE_MachE_sil_test_real_T_24 =
    &CAVE_MachE_sil_test_P.CoreSubsys.Constant1_Value;
  p_2_CAVE_MachE_sil_test_real_T_0 =
    &CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_DSTATE;
  p_2_CAVE_MachE_sil_test_int32_T_2 = &CAVE_MachE_sil_test_DW.Product2_DWORK2[0];
  p_2_CAVE_MachE_sil_test_uint32_T_3 =
    &CAVE_MachE_sil_test_DW.temporalCounter_i1;
  p_2_CAVE_MachE_sil_test_int_T_4 =
    &CAVE_MachE_sil_test_DW.IntegratorLimited_IWORK;
  p_2_CAVE_MachE_sil_test_uint8_T_5 = &CAVE_MachE_sil_test_DW.Output_DSTATE;
  p_2_CAVE_MachE_sil_test_int8_T_6 = &CAVE_MachE_sil_test_DW.If_ActiveSubsystem;
  p_2_CAVE_MachE_sil_test_uint8_T_7 =
    &CAVE_MachE_sil_test_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE;
  p_2_CAVE_MachE_sil_test_boolean_T_8 =
    &CAVE_MachE_sil_test_DW.IntegratorSecondOrder_DWORK1;
  p_2_CAVE_MachE_sil_test_real_T_9 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_c.lastMajorTime;
  p_2_CAVE_MachE_sil_test_boolean_T_10 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_c.UnitDelay_DSTATE;
  p_2_CAVE_MachE_sil_test_int8_T_11 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_c.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_sil_test_uint8_T_12 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_c.is_active_c6_autolibshared;
  p_2_CAVE_MachE_sil_test_real_T_13 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_h.lastMajorTime;
  p_2_CAVE_MachE_sil_test_boolean_T_14 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_h.UnitDelay_DSTATE;
  p_2_CAVE_MachE_sil_test_int8_T_15 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_h.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_sil_test_uint8_T_16 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_h.is_active_c6_autolibshared;
  p_2_CAVE_MachE_sil_test_real_T_17 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_n.lastMajorTime;
  p_2_CAVE_MachE_sil_test_boolean_T_18 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_n.UnitDelay_DSTATE;
  p_2_CAVE_MachE_sil_test_int8_T_19 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_n.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_sil_test_uint8_T_20 =
    &CAVE_MachE_sil_test_DW.sf_LockUp_n.is_active_c6_autolibshared;
  p_2_CAVE_MachE_sil_test_real_T_21 =
    &CAVE_MachE_sil_test_DW.sf_LockUp.lastMajorTime;
  p_2_CAVE_MachE_sil_test_boolean_T_22 =
    &CAVE_MachE_sil_test_DW.sf_LockUp.UnitDelay_DSTATE;
  p_2_CAVE_MachE_sil_test_int8_T_23 =
    &CAVE_MachE_sil_test_DW.sf_LockUp.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_sil_test_uint8_T_24 =
    &CAVE_MachE_sil_test_DW.sf_LockUp.is_active_c6_autolibshared;
  p_3_CAVE_MachE_sil_test_real_T_0 = &CAVE_MachE_sil_test_X.xeyeze_CSTATE[0];
  p_3_CAVE_MachE_sil_test_real_T_1 =
    &CAVE_MachE_sil_test_X.sf_LockUp_c.omegaWheel_l;
  p_3_CAVE_MachE_sil_test_real_T_2 =
    &CAVE_MachE_sil_test_X.sf_LockUp_h.omegaWheel_l;
  p_3_CAVE_MachE_sil_test_real_T_3 =
    &CAVE_MachE_sil_test_X.sf_LockUp_n.omegaWheel_l;
  p_3_CAVE_MachE_sil_test_real_T_4 =
    &CAVE_MachE_sil_test_X.sf_LockUp.omegaWheel_l;
  p_3_CAVE_MachE_sil_test_real_T_5 = &CAVE_MachE_sil_test_X.CoreSubsys_m[0].
    _CSTATE;
}

void CAVE_MachE_sil_test_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}
