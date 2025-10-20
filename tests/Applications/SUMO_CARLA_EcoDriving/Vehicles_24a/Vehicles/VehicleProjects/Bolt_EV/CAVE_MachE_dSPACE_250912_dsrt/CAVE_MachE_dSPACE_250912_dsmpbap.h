/*********************** dSPACE target specific header file ********************
   Include file CAVE_MachE_dSPACE_250912_dsmpbap.h:

   Definitions used for access points

   Sun Sep 21 17:10:19 2025

   (c) Copyright 2022, dSPACE GmbH. All rights reserved.

 *******************************************************************************/

#ifndef _DSMPB_CAVE_MachE_dSPACE_250912_HEADER_
#define _DSMPB_CAVE_MachE_dSPACE_250912_HEADER_
#include "rtwtypes.h"
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

/* External declarations for access points prototypes */

/* Include header file with bus struct typedefs                               */
#include "CAVE_MachE_dSPACE_250912_dsmpb_bd.h"

/*                                                                            */
/* Declarations of read/write and trigger access points                       */
/*                                                                            */
#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2)

/* Read bus access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In, Port: 1   */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1
  (read_CAVE_MachE_dSPACE_250912_DataInport1_P1* OutputBusPortSignalPtr);

#endif

#if (DATA_PORT_ACCESS_POINT_API_VERSION == 1)

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S1(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 2 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S2(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 3 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S3(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 4 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S4(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 5 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S5(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 6 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S6(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 7 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S7(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 8 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S8(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 9 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S9(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 10 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S10(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 11 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S11(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 12 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S12(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 13 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S13(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 14 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S14(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 15 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S15(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 16 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S16(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 17 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S17(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 18 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S18(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 19 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S19(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 20 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S20(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 21 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S21(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 22 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S22(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 23 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S23(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 24 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S24(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 25 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S25(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 26 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S26(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 27 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S27(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 28 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S28(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 29 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S29(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 30 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S30(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 31 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S31(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 32 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S32(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 33 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S33(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 34 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S34(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 35 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S35(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 36 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S36(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 37 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S37(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 38 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S38(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 39 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S39(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 40 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S40(uint16_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 41 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S41(uint16_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 42 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S42(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 43 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S43(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In untyped bus port 1 signal 44 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport1_P1_S44(real_T
  * OutputPortSignalPtr);

#endif

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_In [TCP (1)], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport2_P1_S1(uint8_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_In [TCP (1)], non-bus port 2 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport2_P2_S1(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_In [TCP (1)], non-bus port 3 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport2_P3_S1(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_In [TCP (1)], non-bus port 4 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport2_P4_S1(uint32_T
  * OutputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Receive_Out [TCP (1)], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport1_P1_S1(const
  uint32_T* InputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Status//Control_In [TCP (1)], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport3_P1_S1(int32_T
  * OutputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/Status//Control_Out [TCP (1)], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport2_P1_S1(const
  boolean_T* InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP Out/Transmit_Out [TCP (1)], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport3_P1_S1(const uint8_T*
  InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP Out/Transmit_Out [TCP (1)], non-bus port 2 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport3_P2_S1(const
  uint32_T* InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP Out/Transmit_Out [TCP (1)], non-bus port 3 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport3_P3_S1(const
  boolean_T* InputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Ethernet Setup, non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport4_P1_S1(real_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Transmit_In [TCP (1)], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport5_P1_S1(boolean_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/FIXS/RealSim Core/dSPACE TCP In/Transmit_In [TCP (1)], non-bus port 2 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport5_P2_S1(uint32_T
  * OutputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ModesOfOperation\ISignal Value\ModesOfOperation Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport4_P1_S1(const signed
  char* InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Enable\RPDO3 Enable [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport5_P1_S1(const
  boolean_T* InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/RPDO3\PDU Features\PDU Cyclic Timing Control\RPDO3 Timing Control Period [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport6_P1_S1(const real_T*
  InputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/ CAN Settings/TPDO3\PDU Features\PDU RX Status\TPDO3 Counter [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport6_P1_S1(uint32_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\StatusWord0\ISignal Value\StatusWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport7_P1_S1(int16_T
  * OutputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ActualPosition\ISignal Value\ActualPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport8_P1_S1(int32_T
  * OutputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\ControlWord0\ISignal Value\ControlWord0 Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport7_P1_S1(const int16_T*
  InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/RPDO3\TargetPosition\ISignal Value\TargetPosition Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\TX\PDUs], non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport8_P1_S1(const int32_T*
  InputPortSignalPtr);

/* Read access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/MOOG Brake Actuator/TPDO3\ModesOfOperationDisplay\ISignal Value\ModesOfOperationDisplay Value [BrakeAct_Config\Simulated ECUs\CANOpen_MOOG_02\Controller\RX\PDUs], non-bus port 1 */
EXTERN_C void ap_read_CAVE_MachE_dSPACE_250912_DataInport9_P1_S1(signed char
  * OutputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (1), non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport9_P1_S1(const real_T*
  InputPortSignalPtr);

/* Write access point of block CAVE_MachE_dSPACE_250912/Vehicle Under Test2/Vehicle/Actuation and Outputs/Vehicle Accel Pedal Voltage Output/Voltage Out (2), non-bus port 1 */
EXTERN_C void ap_write_CAVE_MachE_dSPACE_250912_DataOutport10_P1_S1(const real_T*
  InputPortSignalPtr);

/*                                                                            */
/* Declarations of function module access points                              */
/*                                                                            */
/* Function module access point of system <Root>    */
EXTERN_C void ap_entry_CAVE_MachE_dSPACE_250912_SIDRoot_TID1();
EXTERN_C void ap_exit_CAVE_MachE_dSPACE_250912_SIDRoot_TID1();

#endif
