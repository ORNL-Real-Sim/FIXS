/*********************** dSPACE target specific file *************************

   Header file CAVE_MachE_dSPACE_250912_dsrtvdf.h:

   Declaration of function that initializes the global TRC pointers

   24.1 (02-May-2024)
   Sun Sep 21 17:10:19 2025

   Copyright 2025, dSPACE GmbH. All rights reserved.

 *****************************************************************************/
#ifndef _DSRT_CAVE_MachE_dSPACE_250912_DSRTVDF_HEADER_
#define _DSRT_CAVE_MachE_dSPACE_250912_DSRTVDF_HEADER_

/* Include the model header file. */
#include "CAVE_MachE_dSPACE_250912.h"
#include "CAVE_MachE_dSPACE_250912_private.h"
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

/*
 *  Declare the global TRC pointers
 */
EXTERN_C volatile VehDataBus *p_0_CAVE_MachE_dSPACE_250912_VehDataBus_0;
EXTERN_C volatile int64_T *p_0_CAVE_MachE_dSPACE_250912_int64_T_1;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_2;
EXTERN_C volatile uint32_T *p_0_CAVE_MachE_dSPACE_250912_uint32_T_3;
EXTERN_C volatile int32_T *p_0_CAVE_MachE_dSPACE_250912_int32_T_4;
EXTERN_C volatile uint16_T *p_0_CAVE_MachE_dSPACE_250912_uint16_T_5;
EXTERN_C volatile int16_T *p_0_CAVE_MachE_dSPACE_250912_int16_T_6;
EXTERN_C volatile uint8_T *p_0_CAVE_MachE_dSPACE_250912_uint8_T_7;
EXTERN_C volatile int8_T *p_0_CAVE_MachE_dSPACE_250912_int8_T_8;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_9;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_10;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_11;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_12;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_13;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_14;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_15;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_16;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_17;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_18;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_19;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_20;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_21;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_22;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_23;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_24;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_25;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_26;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_27;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_28;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_29;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_30;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_31;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_32;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_33;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_34;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_35;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_36;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_37;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_38;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_39;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_40;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_41;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_42;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_43;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_44;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_45;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_46;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_47;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_48;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_49;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_50;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_51;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_52;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_53;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_54;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_55;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_56;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_57;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_58;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_59;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_60;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_61;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_62;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_63;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_64;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_65;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_66;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_67;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_68;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_69;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_70;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_71;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_72;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_73;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_74;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_75;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_76;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_77;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_78;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_79;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_80;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_81;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_82;
EXTERN_C volatile boolean_T *p_0_CAVE_MachE_dSPACE_250912_boolean_T_83;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_84;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_85;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_86;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_87;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_88;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_89;
EXTERN_C volatile real_T *p_0_CAVE_MachE_dSPACE_250912_real_T_90;
EXTERN_C volatile struct_H6OzwuYMZP7H9bFCBCIB9C
  *p_1_CAVE_MachE_dSPACE_250912_struct_H6OzwuYMZP7H9bFCBCIB9C_0;
EXTERN_C volatile struct_RLXElOtgrzvJMUIzjpiLaD
  *p_1_CAVE_MachE_dSPACE_250912_struct_RLXElOtgrzvJMUIzjpiLaD_1;
EXTERN_C volatile struct_m2VjwNiXoluKspK4Fr7zNG
  *p_1_CAVE_MachE_dSPACE_250912_struct_m2VjwNiXoluKspK4Fr7zNG_2;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_3;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_4;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_5;
EXTERN_C volatile int32_T *p_1_CAVE_MachE_dSPACE_250912_int32_T_6;
EXTERN_C volatile uint32_T *p_1_CAVE_MachE_dSPACE_250912_uint32_T_7;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_8;
EXTERN_C volatile int8_T *p_1_CAVE_MachE_dSPACE_250912_int8_T_9;
EXTERN_C volatile uint8_T *p_1_CAVE_MachE_dSPACE_250912_uint8_T_10;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_11;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_12;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_13;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_14;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_15;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_16;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_17;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_18;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_19;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_20;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_21;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_22;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_23;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_24;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_25;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_26;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_27;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_28;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_29;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_30;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_31;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_32;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_33;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_34;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_35;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_36;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_37;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_38;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_39;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_40;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_41;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_42;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_43;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_44;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_45;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_46;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_47;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_48;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_49;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_50;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_51;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_52;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_53;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_54;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_55;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_56;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_57;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_58;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_59;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_60;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_61;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_62;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_63;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_64;
EXTERN_C volatile boolean_T *p_1_CAVE_MachE_dSPACE_250912_boolean_T_65;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_66;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_67;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_68;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_69;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_70;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_71;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_72;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_73;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_74;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_75;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_76;
EXTERN_C volatile real_T *p_1_CAVE_MachE_dSPACE_250912_real_T_77;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_3;
EXTERN_C volatile uint32_T *p_2_CAVE_MachE_dSPACE_250912_uint32_T_5;
EXTERN_C volatile int_T *p_2_CAVE_MachE_dSPACE_250912_int_T_6;
EXTERN_C volatile uint16_T *p_2_CAVE_MachE_dSPACE_250912_uint16_T_7;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_8;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_9;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_10;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_11;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_12;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_13;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_14;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_15;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_16;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_17;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_18;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_19;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_20;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_21;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_22;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_23;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_24;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_25;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_26;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_27;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_28;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_29;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_30;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_31;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_32;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_33;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_34;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_35;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_36;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_37;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_38;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_39;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_40;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_41;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_42;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_43;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_44;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_45;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_46;
EXTERN_C volatile real_T *p_2_CAVE_MachE_dSPACE_250912_real_T_47;
EXTERN_C volatile int8_T *p_2_CAVE_MachE_dSPACE_250912_int8_T_48;
EXTERN_C volatile uint8_T *p_2_CAVE_MachE_dSPACE_250912_uint8_T_49;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_50;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_51;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_52;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_53;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_54;
EXTERN_C volatile boolean_T *p_2_CAVE_MachE_dSPACE_250912_boolean_T_55;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_0;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_1;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_2;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_3;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_4;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_5;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_6;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_7;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_8;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_9;
EXTERN_C volatile real_T *p_3_CAVE_MachE_dSPACE_250912_real_T_10;

/*
 *  Declare the general function for TRC pointer initialization
 */
EXTERN_C void CAVE_MachE_dSPACE_250912_rti_init_trc_pointers(void);

#endif                      /* _DSRT_CAVE_MachE_dSPACE_250912_DSRTVDF_HEADER_ */
