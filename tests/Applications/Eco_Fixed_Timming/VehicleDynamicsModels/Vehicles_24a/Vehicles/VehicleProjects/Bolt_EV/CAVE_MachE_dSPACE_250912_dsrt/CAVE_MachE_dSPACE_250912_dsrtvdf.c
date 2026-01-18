/***************************************************************************

   Source file CAVE_MachE_dSPACE_250912_dsrtvdf.c:

   Definition of function that initializes the global TRC pointers

   24.1 (02-May-2024)
   Sun Sep 21 17:10:19 2025

   Copyright 2025, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "CAVE_MachE_dSPACE_250912_dsrtvdf.h"
#include "CAVE_MachE_dSPACE_250912.h"
#include "CAVE_MachE_dSPACE_250912_private.h"

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
volatile VehDataBus *p_0_CAVE_MachE_dSPACE_250912_VehDataBus_0 = NULL;
volatile int64_T *p_0_CAVE_MachE_dSPACE_250912_int64_T_1 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_2 = NULL;
volatile uint32_T *p_0_CAVE_MachE_dSPACE_250912_uint32_T_3 = NULL;
volatile int32_T *p_0_CAVE_MachE_dSPACE_250912_int32_T_4 = NULL;
volatile uint16_T *p_0_CAVE_MachE_dSPACE_250912_uint16_T_5 = NULL;
volatile int16_T *p_0_CAVE_MachE_dSPACE_250912_int16_T_6 = NULL;
volatile uint8_T *p_0_CAVE_MachE_dSPACE_250912_uint8_T_7 = NULL;
volatile int8_T *p_0_CAVE_MachE_dSPACE_250912_int8_T_8 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_9 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_10 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_11 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_12 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_13 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_14 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_15 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_16 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_17 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_18 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_19 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_20 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_21 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_22 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_23 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_24 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_25 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_26 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_27 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_28 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_29 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_30 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_31 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_32 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_33 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_34 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_35 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_36 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_37 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_38 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_39 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_40 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_41 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_42 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_43 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_44 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_45 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_46 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_47 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_48 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_49 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_50 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_51 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_52 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_53 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_54 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_55 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_56 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_57 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_58 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_59 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_60 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_61 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_62 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_63 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_64 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_65 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_66 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_67 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_68 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_69 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_70 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_71 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_72 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_73 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_74 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_75 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_76 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_77 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_78 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_79 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_80 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_81 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_82 = NULL;
volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_83 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_84 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_85 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_86 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_87 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_88 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_89 = NULL;
volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_90 = NULL;
volatile struct_H6OzwuYMZP7H9bFCBCIB9C
  *p_1_CAVE_MachE_dSPACE_250912_struct_H6OzwuYMZP7H9bFCBCIB9C_0 = NULL;
volatile struct_RLXElOtgrzvJMUIzjpiLaD
  *p_1_CAVE_MachE_dSPACE_250912_struct_RLXElOtgrzvJMUIzjpiLaD_1 = NULL;
volatile struct_m2VjwNiXoluKspK4Fr7zNG
  *p_1_CAVE_MachE_dSPACE_250912_struct_m2VjwNiXoluKspK4Fr7zNG_2 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_3 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_4 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_5 = NULL;
volatile int32_T *p_1_CAVE_MachE_dSPACE_250912_int32_T_6 = NULL;
volatile uint32_T *p_1_CAVE_MachE_dSPACE_250912_uint32_T_7 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_8 = NULL;
volatile int8_T *p_1_CAVE_MachE_dSPACE_250912_int8_T_9 = NULL;
volatile uint8_T *p_1_CAVE_MachE_dSPACE_250912_uint8_T_10 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_11 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_12 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_13 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_14 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_15 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_16 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_17 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_18 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_19 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_20 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_21 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_22 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_23 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_24 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_25 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_26 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_27 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_28 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_29 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_30 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_31 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_32 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_33 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_34 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_35 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_36 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_37 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_38 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_39 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_40 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_41 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_42 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_43 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_44 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_45 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_46 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_47 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_48 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_49 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_50 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_51 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_52 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_53 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_54 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_55 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_56 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_57 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_58 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_59 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_60 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_61 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_62 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_63 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_64 = NULL;
volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_65 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_66 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_67 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_68 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_69 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_70 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_71 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_72 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_73 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_74 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_75 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_76 = NULL;
volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_77 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_3 = NULL;
volatile uint32_T *p_2_CAVE_MachE_dSPACE_250912_uint32_T_5 = NULL;
volatile int_T *p_2_CAVE_MachE_dSPACE_250912_int_T_6 = NULL;
volatile uint16_T *p_2_CAVE_MachE_dSPACE_250912_uint16_T_7 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_8 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_9 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_10 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_11 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_12 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_13 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_14 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_15 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_16 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_17 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_18 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_19 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_20 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_21 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_22 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_23 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_24 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_25 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_26 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_27 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_28 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_29 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_30 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_31 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_32 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_33 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_34 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_35 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_36 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_37 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_38 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_39 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_40 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_41 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_42 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_43 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_44 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_45 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_46 = NULL;
volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_47 = NULL;
volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_48 = NULL;
volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_49 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_50 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_51 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_52 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_53 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_54 = NULL;
volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_55 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_0 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_1 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_2 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_3 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_4 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_5 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_6 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_7 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_8 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_9 = NULL;
volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_10 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_CAVE_MachE_dSPACE_250912_VehDataBus_0 =
    &CAVE_MachE_dSPACE_250912_B.BusConversion_InsertedFor_RealSimPack_at_inport_2_BusCreator1;
  p_0_CAVE_MachE_dSPACE_250912_int64_T_1 = &CAVE_MachE_dSPACE_250912_B.Gain;
  p_0_CAVE_MachE_dSPACE_250912_real_T_2 =
    &CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1Time;
  p_0_CAVE_MachE_dSPACE_250912_uint32_T_3 =
    &CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1Counter;
  p_0_CAVE_MachE_dSPACE_250912_int32_T_4 =
    &CAVE_MachE_dSPACE_250912_B.ConnectionState_f;
  p_0_CAVE_MachE_dSPACE_250912_uint16_T_5 =
    &CAVE_MachE_dSPACE_250912_B.BrkTot_Tq_RqArbValue;
  p_0_CAVE_MachE_dSPACE_250912_int16_T_6 =
    &CAVE_MachE_dSPACE_250912_B.Modeofoperation;
  p_0_CAVE_MachE_dSPACE_250912_uint8_T_7 =
    &CAVE_MachE_dSPACE_250912_B.TrnRng_D_RqValue;
  p_0_CAVE_MachE_dSPACE_250912_int8_T_8 =
    &CAVE_MachE_dSPACE_250912_B.DataTypeConversion_h;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_9 =
    &CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1State;
  p_0_CAVE_MachE_dSPACE_250912_real_T_10 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_pna[3].
    TmpSignalConversionAtSinCosInport1[0];
  p_0_CAVE_MachE_dSPACE_250912_real_T_11 =
    &CAVE_MachE_dSPACE_250912_B.sf_MATLABFunction3.switchFlag;
  p_0_CAVE_MachE_dSPACE_250912_real_T_12 =
    &CAVE_MachE_dSPACE_250912_B.sf_MATLABFunction2.switchFlag;
  p_0_CAVE_MachE_dSPACE_250912_real_T_13 =
    &CAVE_MachE_dSPACE_250912_B.sf_MATLABFunction1.switchFlag;
  p_0_CAVE_MachE_dSPACE_250912_real_T_14 =
    &CAVE_MachE_dSPACE_250912_B.sf_MATLABFunction_n.switchFlag;
  p_0_CAVE_MachE_dSPACE_250912_real_T_15 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_16 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_17 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_18 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_19 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_20 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_k[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_21 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_gz.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_22 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_23 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_24 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_25 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_26 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_27 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_d[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_28 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_f.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_29 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_30 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_31 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_32 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_33 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_34 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_e[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_35 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_a.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_36 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_37 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_38 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_39 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_40 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_41 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_od[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_42 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_e.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_43 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_44 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_45 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_46 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_47 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_48 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_n[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_49 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_m.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_50 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_51 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_52 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_53 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_54 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_55 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_cu[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_56 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_d.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_57 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_58 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_59 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_60 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_61 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_62 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_c[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_63 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput_g.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_64 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].sf_Clutch.Tout;
  p_0_CAVE_MachE_dSPACE_250912_real_T_65 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].sf_Clutch.Slipping.omegawheel;
  p_0_CAVE_MachE_dSPACE_250912_real_T_66 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].
    sf_Clutch.detectLockup.OutputDamping;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_67 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].
    sf_Clutch.detectLockup.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_68 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].sf_Clutch.detectSlip.Abs;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_69 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_h[0].
    sf_Clutch.detectSlip.RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_70 =
    &CAVE_MachE_dSPACE_250912_B.sf_MagicTireConstInput.Fx;
  p_0_CAVE_MachE_dSPACE_250912_real_T_71 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_pn[3].
    TmpSignalConversionAtSinCosInport1[0];
  p_0_CAVE_MachE_dSPACE_250912_real_T_72 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_b[1].
    TmpSignalConversionAtSelector3Inport1[0];
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_73 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_b[1].RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_74 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_b[1].Maxstopreached.Sum1;
  p_0_CAVE_MachE_dSPACE_250912_real_T_75 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_b[1].Minstopreached.Sum1;
  p_0_CAVE_MachE_dSPACE_250912_real_T_76 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_p[0].DataTypeConversion;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_77 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_p[0].RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_78 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_o[1].DataTypeConversion;
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_79 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_o[1].RelationalOperator;
  p_0_CAVE_MachE_dSPACE_250912_real_T_80 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_o[1].Maxstopreached.Sum1;
  p_0_CAVE_MachE_dSPACE_250912_real_T_81 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_o[1].Minstopreached.Sum1;
  p_0_CAVE_MachE_dSPACE_250912_real_T_82 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_g[0].Reshape[0];
  p_0_CAVE_MachE_dSPACE_250912_boolean_T_83 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys_g[0].RelationalOperator[0];
  p_0_CAVE_MachE_dSPACE_250912_real_T_84 =
    &CAVE_MachE_dSPACE_250912_B.CoreSubsys[1].Selector6;
  p_0_CAVE_MachE_dSPACE_250912_real_T_85 =
    &CAVE_MachE_dSPACE_250912_B.IfActionSubsystem1_a.Divide;
  p_0_CAVE_MachE_dSPACE_250912_real_T_86 =
    &CAVE_MachE_dSPACE_250912_B.IfActionSubsystem1.Divide;
  p_0_CAVE_MachE_dSPACE_250912_real_T_87 =
    &CAVE_MachE_dSPACE_250912_B.PassThrough_e.u;
  p_0_CAVE_MachE_dSPACE_250912_real_T_88 =
    &CAVE_MachE_dSPACE_250912_B.PassThrough_k.u;
  p_0_CAVE_MachE_dSPACE_250912_real_T_89 =
    &CAVE_MachE_dSPACE_250912_B.PassThrough_i.u;
  p_0_CAVE_MachE_dSPACE_250912_real_T_90 =
    &CAVE_MachE_dSPACE_250912_B.PassThrough.u;
  p_1_CAVE_MachE_dSPACE_250912_struct_H6OzwuYMZP7H9bFCBCIB9C_0 =
    &CAVE_MachE_dSPACE_250912_P.US06;
  p_1_CAVE_MachE_dSPACE_250912_struct_RLXElOtgrzvJMUIzjpiLaD_1 =
    &CAVE_MachE_dSPACE_250912_P.RealSimPara;
  p_1_CAVE_MachE_dSPACE_250912_struct_m2VjwNiXoluKspK4Fr7zNG_2 =
    &CAVE_MachE_dSPACE_250912_P.InitStates[0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_3 =
    &CAVE_MachE_dSPACE_250912_P.BattChargeLimitMaxPwr;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_4 =
    &CAVE_MachE_dSPACE_250912_P.DetectRisePositive_vinit;
  p_1_CAVE_MachE_dSPACE_250912_real_T_5 =
    &CAVE_MachE_dSPACE_250912_P.RealSimInterpSpeed_speedUpperBound;
  p_1_CAVE_MachE_dSPACE_250912_int32_T_6 =
    &CAVE_MachE_dSPACE_250912_P.Gain_Gain_jp;
  p_1_CAVE_MachE_dSPACE_250912_uint32_T_7 =
    &CAVE_MachE_dSPACE_250912_P.R_maxIndex[0];
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_8 =
    &CAVE_MachE_dSPACE_250912_P.Memory_InitialCondition_h5;
  p_1_CAVE_MachE_dSPACE_250912_int8_T_9 =
    &CAVE_MachE_dSPACE_250912_P.Gain1_Gain_i;
  p_1_CAVE_MachE_dSPACE_250912_uint8_T_10 =
    &CAVE_MachE_dSPACE_250912_P.ManualSwitch2_CurrentSetting;
  p_1_CAVE_MachE_dSPACE_250912_real_T_11 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_12 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_13 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_14 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_15 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_16 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_k.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_17 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_gz.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_18 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_19 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_20 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_21 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_22 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_23 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_d.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_24 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_f.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_25 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_26 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_27 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_28 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_29 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_30 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_e.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_31 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_a.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_32 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_33 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_34 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_35 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_36 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_37 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_od.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_38 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_e.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_39 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_40 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_41 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_42 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_43 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_44 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_n.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_45 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_m.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_46 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_47 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_48 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_49 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_50 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_51 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_cu.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_52 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_d.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_53 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_54 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_55 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_56 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_57 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_58 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_c.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_59 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput_g.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_60 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.Clutch_OmegaTol;
  p_1_CAVE_MachE_dSPACE_250912_real_T_61 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.sf_Clutch.Slipping.u_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_62 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.sf_Clutch.Locked.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_63 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.sf_Clutch.detectLockup.Constant_Value;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_64 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.sf_Clutch.detectLockup.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_boolean_T_65 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_h.sf_Clutch.detectSlip.yn_Y0;
  p_1_CAVE_MachE_dSPACE_250912_real_T_66 =
    &CAVE_MachE_dSPACE_250912_P.sf_MagicTireConstInput.MagicTireConstInput_vdynMF
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_67 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_b.SelectCamberSteeringSlope_AxleNums;
  p_1_CAVE_MachE_dSPACE_250912_real_T_68 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_b.Maxstopreached.Gain5_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_69 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_b.Minstopreached.Gain5_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_70 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_p.AntiSwayArmRadiusByAxle_AxleNums;
  p_1_CAVE_MachE_dSPACE_250912_real_T_71 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_o.SelectCamberSteeringCenter_AxleNums;
  p_1_CAVE_MachE_dSPACE_250912_real_T_72 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_o.Maxstopreached.Gain5_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_73 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_o.Minstopreached.Gain5_Gain;
  p_1_CAVE_MachE_dSPACE_250912_real_T_74 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys_g.SelectAxleMassByAxle_AxleNums;
  p_1_CAVE_MachE_dSPACE_250912_real_T_75 =
    &CAVE_MachE_dSPACE_250912_P.CoreSubsys.Constant1_Value;
  p_1_CAVE_MachE_dSPACE_250912_real_T_76 =
    &CAVE_MachE_dSPACE_250912_P.IfActionSubsystem1_a.Interpolatedzerocrossing_tableData
    [0];
  p_1_CAVE_MachE_dSPACE_250912_real_T_77 =
    &CAVE_MachE_dSPACE_250912_P.IfActionSubsystem1.Interpolatedzerocrossing_tableData
    [0];
  p_2_CAVE_MachE_dSPACE_250912_real_T_3 =
    &CAVE_MachE_dSPACE_250912_DW.UnitDelay_DSTATE[0];
  p_2_CAVE_MachE_dSPACE_250912_uint32_T_5 =
    &CAVE_MachE_dSPACE_250912_DW.m_bpIndex;
  p_2_CAVE_MachE_dSPACE_250912_int_T_6 =
    &CAVE_MachE_dSPACE_250912_DW.Integrator_IWORK;
  p_2_CAVE_MachE_dSPACE_250912_uint16_T_7 =
    &CAVE_MachE_dSPACE_250912_DW.temporalCounter_i1_l;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_8 =
    &CAVE_MachE_dSPACE_250912_DW.DelayInput1_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_9 =
    &CAVE_MachE_dSPACE_250912_DW.If_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_10 =
    &CAVE_MachE_dSPACE_250912_DW.is_active_c11_CAVE_MachE_dSPACE_250912;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_11 =
    &CAVE_MachE_dSPACE_250912_DW.IntegratorSecondOrder_DWORK1;
  p_2_CAVE_MachE_dSPACE_250912_real_T_12 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_k[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_13 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_k[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_14 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_k[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_15 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_k[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_16 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_k[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_17 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_d[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_18 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_d[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_19 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_d[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_20 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_d[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_21 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_d[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_22 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_e[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_23 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_e[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_24 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_e[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_25 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_e[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_26 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_e[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_27 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_od[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_28 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_od[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_29 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_od[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_30 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_od[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_31 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_od[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_32 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_n[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_33 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_n[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_34 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_n[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_35 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_n[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_36 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_n[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_37 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_cu[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_38 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_cu[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_39 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_cu[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_40 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_cu[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_41 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_cu[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_42 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_c[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_43 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_c[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_44 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_c[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_45 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_c[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_46 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_c[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_real_T_47 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_h[0].sf_Clutch.lastMajorTime;
  p_2_CAVE_MachE_dSPACE_250912_int8_T_48 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_h[0].
    sf_Clutch.TmpIfAtSlippingInport3_ActiveSubsystem;
  p_2_CAVE_MachE_dSPACE_250912_uint8_T_49 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_h[0].
    sf_Clutch.is_active_c8_autolibshared;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_50 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_h[0].sf_Clutch.Slipping_entered;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_51 =
    &CAVE_MachE_dSPACE_250912_DW.CoreSubsys_h[0].
    sf_Clutch.detectLockup.UnitDelay_DSTATE;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_52 =
    &CAVE_MachE_dSPACE_250912_DW.PassThrough_e.PassThrough_MODE;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_53 =
    &CAVE_MachE_dSPACE_250912_DW.PassThrough_k.PassThrough_MODE;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_54 =
    &CAVE_MachE_dSPACE_250912_DW.PassThrough_i.PassThrough_MODE;
  p_2_CAVE_MachE_dSPACE_250912_boolean_T_55 =
    &CAVE_MachE_dSPACE_250912_DW.PassThrough.PassThrough_MODE;
  p_3_CAVE_MachE_dSPACE_250912_real_T_0 =
    &CAVE_MachE_dSPACE_250912_X.Integrator_CSTATE[0];
  p_3_CAVE_MachE_dSPACE_250912_real_T_1 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_k[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_2 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_d[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_3 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_e[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_4 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_od[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_5 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_n[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_6 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_cu[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_7 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_c[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_8 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_h[0].sf_Clutch.Slipping.omegaWheel;
  p_3_CAVE_MachE_dSPACE_250912_real_T_9 =
    &CAVE_MachE_dSPACE_250912_X.CoreSubsys_g[0]._CSTATE;
  p_3_CAVE_MachE_dSPACE_250912_real_T_10 =
    &CAVE_MachE_dSPACE_250912_X.Limits5050_CSTATE;
}

void CAVE_MachE_dSPACE_250912_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}
