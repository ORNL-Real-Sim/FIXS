/*********************** dSPACE target specific implementation file ********************
   Implementation file CAVE_MachE_dSPACE_250912_dsmpb_bd.c:

   Implementation file for API vs. 2 access points and size functions.

   Sun Sep 21 17:10:19 2025

   (c) Copyright 2022, dSPACE GmbH. All rights reserved.

 ****************************************************************************************/

#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2)
#include "CAVE_MachE_dSPACE_250912.h"

void InitializeBusStructs(void)
{
  /* Initialization of struct for untyped bus signal of Data Inport block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In, port: 1 */
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_1 =
    CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1Counter;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_2 =
    CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1State;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_3 =
    CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1Time;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_4 =
    CAVE_MachE_dSPACE_250912_B.PowertrainData_10_HS1DeltaTime;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_5 =
    CAVE_MachE_dSPACE_250912_B.TrnRng_D_RqValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_6 =
    CAVE_MachE_dSPACE_250912_B.TrnPrkSys_D_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_7 =
    CAVE_MachE_dSPACE_250912_B.GearLvr_D_ActlDrvValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_8 =
    CAVE_MachE_dSPACE_250912_B.GearPos_No_CsValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_9 =
    CAVE_MachE_dSPACE_250912_B.GearPos_D_TrgValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_10 =
    CAVE_MachE_dSPACE_250912_B.GearPos_No_CntValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_11 =
    CAVE_MachE_dSPACE_250912_B.GearPos_D_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_12 =
    CAVE_MachE_dSPACE_250912_B.WheelSpeed_HS1Counter;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_13 =
    CAVE_MachE_dSPACE_250912_B.WheelSpeed_HS1State;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_14 =
    CAVE_MachE_dSPACE_250912_B.WheelSpeed_HS1Time;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_15 =
    CAVE_MachE_dSPACE_250912_B.WheelSpeed_HS1DeltaTime;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_16 =
    CAVE_MachE_dSPACE_250912_B.WhlRr_W_MeasValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_17 =
    CAVE_MachE_dSPACE_250912_B.WhlRl_W_MeasValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_18 =
    CAVE_MachE_dSPACE_250912_B.WhlFr_W_MeasValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_19 =
    CAVE_MachE_dSPACE_250912_B.WhlFl_W_MeasValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_20 =
    CAVE_MachE_dSPACE_250912_B.BrakeSysFeatures_HS1Counter;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_21 =
    CAVE_MachE_dSPACE_250912_B.BrakeSysFeatures_HS1State;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_22 =
    CAVE_MachE_dSPACE_250912_B.BrakeSysFeatures_HS1Time;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_23 =
    CAVE_MachE_dSPACE_250912_B.BrakeSysFeatures_HS1DeltaTime;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_24 =
    CAVE_MachE_dSPACE_250912_B.VehVActlBrk_No_CsValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_25 =
    CAVE_MachE_dSPACE_250912_B.Veh_V_ActlBrkValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_26 =
    CAVE_MachE_dSPACE_250912_B.VehVActlBrk_No_CntValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_27 =
    CAVE_MachE_dSPACE_250912_B.VehVActlBrk_D_QfValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_28 =
    CAVE_MachE_dSPACE_250912_B.EngVehicleSpThrottle_HS1Counter;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_29 =
    CAVE_MachE_dSPACE_250912_B.EngVehicleSpThrottle_HS1State;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_30 =
    CAVE_MachE_dSPACE_250912_B.EngVehicleSpThrottle_HS1Time;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_31 =
    CAVE_MachE_dSPACE_250912_B.EngVehicleSpThrottle_HS1DeltaTime;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_32 =
    CAVE_MachE_dSPACE_250912_B.EngAout3_N_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_33 =
    CAVE_MachE_dSPACE_250912_B.ApedPos_Pc_ActlArbValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_34 =
    CAVE_MachE_dSPACE_250912_B.BrakeSnData_4_HS1Counter;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_35 =
    CAVE_MachE_dSPACE_250912_B.BrakeSnData_4_HS1State;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_36 =
    CAVE_MachE_dSPACE_250912_B.BrakeSnData_4_HS1Time;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_37 =
    CAVE_MachE_dSPACE_250912_B.BrakeSnData_4_HS1DeltaTime;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_38 =
    CAVE_MachE_dSPACE_250912_B.BrkTotTqRqArb_No_CsValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_39 =
    CAVE_MachE_dSPACE_250912_B.BrkTotTqRqArb_No_CntValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_40 =
    CAVE_MachE_dSPACE_250912_B.BrkTot_Tq_RqArbValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_41 =
    CAVE_MachE_dSPACE_250912_B.BrkTot_Tq_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_42 =
    CAVE_MachE_dSPACE_250912_B.HsaStat_D_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_43 =
    CAVE_MachE_dSPACE_250912_B.BattTrac_U2_ActlValue;
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1.BusElement_44 =
    CAVE_MachE_dSPACE_250912_B.BattTrac_I2_ActlValue;
}

/* Definition of bus struct for block CAVE_MachE_dSPACE_250912/MachE_HS1_Config_In, Port:  1 */
read_CAVE_MachE_dSPACE_250912_DataInport1_P1
  Bus_read_CAVE_MachE_dSPACE_250912_DataInport1_P1;

#endif
