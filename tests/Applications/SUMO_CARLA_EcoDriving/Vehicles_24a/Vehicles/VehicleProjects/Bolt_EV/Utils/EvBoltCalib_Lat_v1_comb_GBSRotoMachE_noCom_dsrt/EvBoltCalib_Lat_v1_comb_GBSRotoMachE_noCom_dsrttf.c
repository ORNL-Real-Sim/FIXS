/*****************************************************************************

   Include file EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrttf.c:

   Definition of task functions.

   Mon Jul 10 17:21:16 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header files */
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrttf.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_private.h"

/* Task function for TID0 */
void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTMdlOutputs0()
{
  /* Call to Simulink model output function */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_output();
}

void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTMdlUpdate0()
{
  /* Call to Simulink model upadte function */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_update();
}
