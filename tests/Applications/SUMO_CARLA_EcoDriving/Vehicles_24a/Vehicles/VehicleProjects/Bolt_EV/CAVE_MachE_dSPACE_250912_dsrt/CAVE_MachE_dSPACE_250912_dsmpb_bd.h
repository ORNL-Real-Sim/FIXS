/*********************** dSPACE target specific header file ********************
   Include file CAVE_MachE_dSPACE_250912_dsmpb_bd.h:

   Type definitions used for bus access points

   Sun Sep 21 17:10:19 2025

   (c) Copyright 2022, dSPACE GmbH. All rights reserved.

 *******************************************************************************/

#ifndef _DSMPB_CAVE_MachE_dSPACE_250912_BD_HEADER_
#define _DSMPB_CAVE_MachE_dSPACE_250912_BD_HEADER_
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

#ifndef DATA_PORT_ACCESS_POINT_API_VERSION
#error The data port access point API version is undefined.
#elif (DATA_PORT_ACCESS_POINT_API_VERSION > 2) || (DATA_PORT_ACCESS_POINT_API_VERSION < 1)
#error The defined data port access point API version is unsupported.
#endif

#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2)
#include "CAVE_MachE_dSPACE_250912_types.h"

typedef struct {
  uint32_T BusElement_1;
  boolean_T BusElement_2;
  real_T BusElement_3;
  real_T BusElement_4;
  uint8_T BusElement_5;
  uint8_T BusElement_6;
  uint8_T BusElement_7;
  uint8_T BusElement_8;
  uint8_T BusElement_9;
  uint8_T BusElement_10;
  uint8_T BusElement_11;
  uint32_T BusElement_12;
  boolean_T BusElement_13;
  real_T BusElement_14;
  real_T BusElement_15;
  real_T BusElement_16;
  real_T BusElement_17;
  real_T BusElement_18;
  real_T BusElement_19;
  uint32_T BusElement_20;
  boolean_T BusElement_21;
  real_T BusElement_22;
  real_T BusElement_23;
  uint8_T BusElement_24;
  real_T BusElement_25;
  uint8_T BusElement_26;
  uint8_T BusElement_27;
  uint32_T BusElement_28;
  boolean_T BusElement_29;
  real_T BusElement_30;
  real_T BusElement_31;
  real_T BusElement_32;
  real_T BusElement_33;
  uint32_T BusElement_34;
  boolean_T BusElement_35;
  real_T BusElement_36;
  real_T BusElement_37;
  uint8_T BusElement_38;
  uint8_T BusElement_39;
  uint16_T BusElement_40;
  uint16_T BusElement_41;
  uint8_T BusElement_42;
  real_T BusElement_43;
  real_T BusElement_44;
} read_CAVE_MachE_dSPACE_250912_DataInport1_P1;

/* Definition of bus struct for block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In, Port:  1 */
EXTERN_C read_CAVE_MachE_dSPACE_250912_DataInport1_P1
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1;

/* InitializeBusStructs() is called in init phase of model code */
EXTERN_C void InitializeBusStructs(void);

#define DATA_PORT_MUST_INITIALIZE_BUSSTRUCTS
#endif
#endif
