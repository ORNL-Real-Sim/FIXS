/***************************************************************************

   Source file EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrtvdf.c:

   Definition of function that initializes the global TRC pointers

   4.2p1 (11-Feb-2020)
   Mon Jul 10 17:21:16 2023

   Copyright 2023, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrtvdf.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_private.h"

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
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_1 = NULL;
volatile uint32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_2 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_3 =
  NULL;
volatile int32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_4 =
  NULL;
volatile uint16_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_5 =
  NULL;
volatile int16_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int16_T_6 =
  NULL;
volatile uint8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_7 =
  NULL;
volatile int8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_8 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_9 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_10 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_11 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_12 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_13 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_14 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_15 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_16 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_17 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_18 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_19 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_20 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_21 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_22 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_23 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_24 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_25 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_26 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_27 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_28 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_29 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_30 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_31 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_32 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_33 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_34 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_35 = NULL;
volatile boolean_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_36 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_37 = NULL;
volatile uint8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_38 =
  NULL;
volatile uint8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_39 =
  NULL;
volatile uint8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_40 =
  NULL;
volatile uint8_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_41 =
  NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_42 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_43 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_44 = NULL;
volatile real_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_45 = NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_46 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_47 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_48 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_49 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_50 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_51 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_52 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_53 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_54 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_55 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_56 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_57 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_58 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_59 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_60 =
  NULL;
volatile real32_T *p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_61 =
  NULL;
volatile struct_KVvNvmPYkDwFa3qoVeWWJC
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_KVvNvmPYkDwFa3qoVeWWJC_0
  = NULL;
volatile struct_akRwpZPsceW5LlhYlX5DVF
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_akRwpZPsceW5LlhYlX5DVF_1
  = NULL;
volatile struct_LqXVUGdvzWUJgx4oCacWrF
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_LqXVUGdvzWUJgx4oCacWrF_2
  = NULL;
volatile struct_UDems7iN2Kzz7DJ6EtAQJH
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_UDems7iN2Kzz7DJ6EtAQJH_3
  = NULL;
volatile struct_H6OzwuYMZP7H9bFCBCIB9C
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_H6OzwuYMZP7H9bFCBCIB9C_4
  = NULL;
volatile struct_m2VjwNiXoluKspK4Fr7zNG
  *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_m2VjwNiXoluKspK4Fr7zNG_5
  = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_6 = NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_7 =
  NULL;
volatile uint8_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_8 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_9 = NULL;
volatile int32_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_10 =
  NULL;
volatile uint32_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_11 =
  NULL;
volatile uint16_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_12 =
  NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_13 =
  NULL;
volatile int8_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_14 = NULL;
volatile uint8_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_15 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_16 = NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_17 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_18 = NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_19 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_20 = NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_21 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_22 = NULL;
volatile boolean_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_23 =
  NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_24 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_25 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_26 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_27 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_28 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_29 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_30 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_31 = NULL;
volatile real_T *p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_32 = NULL;
volatile real_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_0 = NULL;
volatile int32_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_2 =
  NULL;
volatile uint32_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_3 =
  NULL;
volatile int_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int_T_4 = NULL;
volatile uint16_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_5 =
  NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_6 =
  NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_7 =
  NULL;
volatile int8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_8 = NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_9 =
  NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_10 =
  NULL;
volatile real_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_11 = NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_12 =
  NULL;
volatile int8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_13 = NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_14 =
  NULL;
volatile real_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_15 = NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_16 =
  NULL;
volatile int8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_17 = NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_18 =
  NULL;
volatile real_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_19 = NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_20 =
  NULL;
volatile int8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_21 = NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_22 =
  NULL;
volatile real_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_23 = NULL;
volatile boolean_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_24 =
  NULL;
volatile int8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_25 = NULL;
volatile uint8_T *p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_26 =
  NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_0 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_1 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_2 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_3 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_4 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_5 = NULL;
volatile real_T *p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_6 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_1 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.ubvbwb[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_2 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.PowertrainData_10_HS1Counter;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_3 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.DataTypeConversion_c;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_4 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.ActualPositionValue;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_5 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrkTot_Tq_RqArbValue;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int16_T_6 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.Modeofoperation;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_7 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.TrnRng_D_RqValue;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_8 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.DataTypeConversion_g;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_9 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.Memory_du;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_10 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_pna[3].
    TmpSignalConversionAtsincosInport1[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_11 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_c.Tout;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_12 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_c.RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_13 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MagicTireConstInput_k.Fx;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_14 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_h.Tout;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_15 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_h.RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_16 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MagicTireConstInput_f.Fx;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_17 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_n.Tout;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_18 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp_n.RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_19 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MagicTireConstInput_j.Fx;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_20 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp.Tout;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_21 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_LockUp.RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_22 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MagicTireConstInput.Fx;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_23 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_pn[3].
    TmpSignalConversionAtsincosInport1[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_24 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_b[0].Product[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_25 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_d[1].
    TmpSignalConversionAtSelector3Inport1[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_26 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_d[1].
    RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_27 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_d[1].
    Maxstopreached.Sum1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_28 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_d[1].
    Minstopreached.Sum1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_29 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_n[0].
    DataTypeConversion;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_30 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_n[0].
    RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_31 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_p[1].
    DataTypeConversion;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_32 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_p[1].
    RelationalOperator;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_33 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_p[1].
    Maxstopreached.Sum1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_34 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_p[1].
    Minstopreached.Sum1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_35 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_m[0].Reshape[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_36 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys_m[0].
    RelationalOperator[0];
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_37 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.CoreSubsys[1].Selector6;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_38 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Defloater_g.byte1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_39 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Defloater_n.byte1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_40 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Defloater_h.byte1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_41 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Defloater.byte1;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_42 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MATLABFunction_f.switchFlag;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_43 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MATLABFunction_c.switchFlag;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_44 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MATLABFunction_n.switchFlag;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_45 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_MATLABFunction.switchFlag;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_46 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_a.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_47 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_ew.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_48 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_e.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_49 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_bz.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_50 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_j.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_51 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_d.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_52 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_ns.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_53 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_p.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_54 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_m.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_55 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_f.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_56 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_b1.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_57 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_bp.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_58 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_b.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_59 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_n.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_60 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater_c.float32;
  p_0_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real32_T_61 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.sf_Floater.float32;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_KVvNvmPYkDwFa3qoVeWWJC_0
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.MCT;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_akRwpZPsceW5LlhYlX5DVF_1
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.pMCT;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_LqXVUGdvzWUJgx4oCacWrF_2
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoastDown;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_UDems7iN2Kzz7DJ6EtAQJH_3
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.limitedMCT;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_H6OzwuYMZP7H9bFCBCIB9C_4
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.US06;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_struct_m2VjwNiXoluKspK4Fr7zNG_5
    = &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.InitStates[0];
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_6 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.BattChargeLimitMaxPwr;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_7 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.DetectFallNegative_vinit;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_8 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.WrapToZero_Threshold;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_9 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.Out1_Y0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_10 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.Gain_Gain_o0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_11 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.R_maxIndex[0];
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_12 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.ProtocolVer_Value;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_13 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.Memory_InitialCondition_j;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_14 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.Gain1_Gain_o0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_15 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.Gain7_Gain_n;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_16 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_c.Constant_Value;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_17 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_c.yn_Y0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_18 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_h.Constant_Value;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_19 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_h.yn_Y0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_20 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_n.Constant_Value;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_21 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp_n.yn_Y0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_22 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp.Constant_Value;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_23 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.sf_LockUp.yn_Y0;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_24 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_d.SelectCamberSteeringSlope_AxleNums;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_25 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_d.Maxstopreached.Gain5_Gain;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_26 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_d.Minstopreached.Gain5_Gain;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_27 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_n.AntiSwayArmRadiusByAxle_AxleNums;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_28 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_p.SelectCamberSteeringCenter_AxleNums;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_29 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_p.Maxstopreached.Gain5_Gain;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_30 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_p.Minstopreached.Gain5_Gain;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_31 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys_m.SelectAxleMassByAxle_AxleNums;
  p_1_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_32 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_P.CoreSubsys.Constant1_Value;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_0 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.DiscreteTimeIntegrator4_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int32_T_2 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.Product2_DWORK2[0];
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint32_T_3 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.temporalCounter_i1;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int_T_4 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.IntegratorLimited_IWORK;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint16_T_5 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.temporalCounter_i2;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_6 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.Output_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_7 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.DelayInput1_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_8 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.If_ActiveSubsystem;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_9 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.DiscreteTimeIntegrator4_SYSTEM_ENABLE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_10 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.Memory_PreviousInput_b0;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_11 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_c.lastMajorTime;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_12 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_c.UnitDelay_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_13 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_c.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_14 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_c.is_active_c6_autolibshared;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_15 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_h.lastMajorTime;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_16 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_h.UnitDelay_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_17 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_h.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_18 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_h.is_active_c6_autolibshared;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_19 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_n.lastMajorTime;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_20 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_n.UnitDelay_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_21 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_n.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_22 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp_n.is_active_c6_autolibshared;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_23 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp.lastMajorTime;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_boolean_T_24 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp.UnitDelay_DSTATE;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_int8_T_25 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_uint8_T_26 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DW.sf_LockUp.is_active_c6_autolibshared;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_0 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.ubvbwb_CSTATE[0];
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_1 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.sf_LockUp_c.omegaWheel_l3;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_2 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.sf_LockUp_h.omegaWheel_l3;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_3 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.sf_LockUp_n.omegaWheel_l3;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_4 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.sf_LockUp.omegaWheel_l3;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_5 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.CoreSubsys_m[0]._CSTATE;
  p_3_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_real_T_6 =
    &EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_X.Limits5050_CSTATE;
}

void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}
