/**************************************************************************** *
 * FILE :
 *  CAVE_MachE_dSPACE_250912_dsrtmdlfcn.c
 *
 *
 *
 * Copyright 2022, dSPACE GmbH. All rights reserved
 *
   \****************************************************************************/

#ifndef TEMP_DSRT_RTI
#include <rtmodel.h>
#include "rtwtypes.h"
#include "CAVE_MachE_dSPACE_250912_dsrtmdlfcn.h"
#include <stdio.h>

int32_T DSRTStopSimulation = 0;
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_initialize();
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_terminate();

/* Initialization of model */
void CAVE_MachE_dSPACE_250912_DSRTInitMdl(void)
{
  /* Initialize model */
  CAVE_MachE_dSPACE_250912_initialize();

  /* Initialize structs generated for untyped bus signals, and for bus element ports that pass buses */
#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2) && defined(DATA_PORT_MUST_INITIALIZE_BUSSTRUCTS)

  InitializeBusStructs();

#endif

  /* Check for initialization errors */
  CAVE_MachE_dSPACE_250912_DSRTCheckForErrorStatus();

  /* Reinit Stop Simulation flag*/
  DSRTStopSimulation = 0;
}

/* Model start function */
void CAVE_MachE_dSPACE_250912_DSRTStartMdl(void)
{
  uint8_T lastApplStateStopped;
  CAVE_MachE_dSPACE_250912_APLastApplStateStopped(&lastApplStateStopped);
  if (lastApplStateStopped) {
    /* Reinit main simulation structure */
    CAVE_MachE_dSPACE_250912_DSRTInitMdl();
  }
}

/* Model stop function */
void CAVE_MachE_dSPACE_250912_DSRTStopMdl(void)
{
  /* Call terminate function */
  CAVE_MachE_dSPACE_250912_terminate();

  /* Check for model error status */
  CAVE_MachE_dSPACE_250912_DSRTCheckForErrorStatus();
}

/* Check for model error status */
void CAVE_MachE_dSPACE_250912_DSRTCheckForErrorStatus()
{
  const char_T* errorStatus = rtmGetErrorStatus(CAVE_MachE_dSPACE_250912_M);
  if (errorStatus != NULL && strcmp(errorStatus, "Simulation finished") == 0) {
    /* The RTM errorStatus field has been set */
    CAVE_MachE_dSPACE_250912_APTerminateSimulation();
    CAVE_MachE_dSPACE_250912_APPrintMessage(
      "Model 'CAVE_MachE_dSPACE_250912' error status:");
    CAVE_MachE_dSPACE_250912_APPrintMessage(errorStatus);
  }
}

/* Check for simulation stop conditions */
void CAVE_MachE_dSPACE_250912_DSRTCheckForSimulationStopCondition(void)
{
  const char_T* errorStatus = rtmGetErrorStatus(CAVE_MachE_dSPACE_250912_M);
  if (errorStatus != NULL) {
    if (DSRTStopSimulation != 1) {
      /* The RTM errorStatus field was set */
      CAVE_MachE_dSPACE_250912_APTerminateSimulation();
      CAVE_MachE_dSPACE_250912_APPrintMessage(
        "Model 'CAVE_MachE_dSPACE_250912' error status:");
      CAVE_MachE_dSPACE_250912_APPrintMessage(errorStatus);
      DSRTStopSimulation = 1;
    }
  } else if (rtmGetStopRequested(CAVE_MachE_dSPACE_250912_M)) {
    if (DSRTStopSimulation != 1) {
      CAVE_MachE_dSPACE_250912_APStopSimulation();
      CAVE_MachE_dSPACE_250912_APPrintMessage(
        "Model 'CAVE_MachE_dSPACE_250912': Executed a Simulink Stop Simulation block or request.");
      DSRTStopSimulation = 1;
    }
  }
}

#endif
