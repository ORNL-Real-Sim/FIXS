/**************************************************************************** *
 * FILE :
 *  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrtmdlfcn.c
 *
 *
 *
 * Copyright 2017, dSPACE GmbH. All rights reserved
 *
   \****************************************************************************/

#ifndef TEMP_DSRT_RTI
#include <rtmodel.h>
#include "rtwtypes.h"
#include "EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrtmdlfcn.h"
#include <stdio.h>

int32_T DSRTStopSimulation = 0;
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_initialize();
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_terminate();

/* Initialization of model */
void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTInitMdl(void)
{
  /* Initialize model */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_initialize();

  /* Check for initialization errors */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForErrorStatus();

  /* Reinit Stop Simulation flag*/
  DSRTStopSimulation = 0;
}

/* Model start function */
void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTStartMdl(void)
{
  uint8_T lastApplStateStopped;
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APLastApplStateStopped
    (&lastApplStateStopped);
  if (lastApplStateStopped) {
    /* Reinit main simulation structure */
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTInitMdl();
  }
}

/* Model stop function */
void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTStopMdl(void)
{
  /* Call terminate function */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_terminate();

  /* Check for model error status */
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForErrorStatus();
}

/* Check for model error status */
void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForErrorStatus()
{
  const char_T* errorStatus = rtmGetErrorStatus
    (EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_M);
  if (errorStatus != (char *) NULL && !strcmp(errorStatus, "Simulation finished"))
  {
    /* The RTM errorStatus field has been set */
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APTerminateSimulation();
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(
      "Model 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom' has set error status:");
    EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(errorStatus);
  }
}

/* Check for simulation stop conditions */
void
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForSimulationStopCondition
  (void)
{
  const char_T* errorStatus = rtmGetErrorStatus
    (EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_M);
  if (errorStatus != (char *) NULL) {
    if (DSRTStopSimulation != 1) {
      /* The RTM errorStatus field has been set */
      EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APTerminateSimulation();
      EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(
        "Model 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom' has set error status:");
      EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(errorStatus);
      DSRTStopSimulation = 1;
    }
  } else if (rtmGetStopRequested(EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_M))
  {
    if (DSRTStopSimulation != 1) {
      EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APStopSimulation();
      EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(
        "Model 'EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom': A Simulink Stop Simulation block or request has been executed.");
      DSRTStopSimulation = 1;
    }
  }
}

#endif
