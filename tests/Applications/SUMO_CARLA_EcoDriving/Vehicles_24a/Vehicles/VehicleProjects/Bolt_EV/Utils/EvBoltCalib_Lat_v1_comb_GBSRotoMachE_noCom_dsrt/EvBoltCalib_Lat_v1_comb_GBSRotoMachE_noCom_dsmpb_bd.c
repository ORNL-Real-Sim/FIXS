/*********************** dSPACE target specific implementation file ********************
   Implementation file EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsmpb_bd.c:

   Implementation file for API vs. 2 access points for untyped bus signals.

   Mon Jul 10 17:21:16 2023

   (c) Copyright 2019, dSPACE GmbH. All rights reserved.

 ****************************************************************************************/

#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2)
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom.h"

void initBusStructs(void)
{
  /* Initialization of struct for untyped bus signal of Data Inport block EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In, port: 1 */
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_1
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.PowertrainData_10_HS1Counter;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_2
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.PowertrainData_10_HS1State;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_3
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.PowertrainData_10_HS1Time;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_4
    =
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.PowertrainData_10_HS1DeltaTime;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_5
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.TrnRng_D_RqValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_6
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.TrnPrkSys_D_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_7
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.GearLvr_D_ActlDrvValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_8
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.GearPos_No_CsValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_9
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.GearPos_D_TrgValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_10
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.GearPos_No_CntValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_11
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.GearPos_D_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_12
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WheelSpeed_HS1Counter;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_13
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WheelSpeed_HS1State;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_14
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WheelSpeed_HS1Time;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_15
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WheelSpeed_HS1DeltaTime;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_16
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WhlRr_W_MeasValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_17
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WhlRl_W_MeasValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_18
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WhlFr_W_MeasValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_19
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.WhlFl_W_MeasValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_20
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSysFeatures_HS1Counter;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_21
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSysFeatures_HS1State;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_22
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSysFeatures_HS1Time;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_23
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSysFeatures_HS1DeltaTime;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_24
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.VehVActlBrk_No_CsValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_25
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.Veh_V_ActlBrkValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_26
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.VehVActlBrk_No_CntValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_27
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.VehVActlBrk_D_QfValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_28
    =
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.EngVehicleSpThrottle_HS1Counter;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_29
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.EngVehicleSpThrottle_HS1State;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_30
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.EngVehicleSpThrottle_HS1Time;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_31
    =
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.EngVehicleSpThrottle_HS1DeltaTime;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_32
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.EngAout3_N_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_33
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.ApedPos_Pc_ActlArbValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_34
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSnData_4_HS1Counter;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_35
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSnData_4_HS1State;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_36
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSnData_4_HS1Time;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_37
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrakeSnData_4_HS1DeltaTime;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_38
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrkTotTqRqArb_No_CsValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_39
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrkTotTqRqArb_No_CntValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_40
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrkTot_Tq_RqArbValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_41
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BrkTot_Tq_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_42
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.HsaStat_D_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_43
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BattTrac_U2_ActlValue;
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1.BusElement_44
    = EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_B.BattTrac_I2_ActlValue;
}

/* Definition of bus struct for block EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom/MachE_HS1_Config_In, Port:  1 */
read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1
  Bus_read_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DataInport1_P1;

#endif
