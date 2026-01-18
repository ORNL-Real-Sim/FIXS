/**************************************************************************** *
 * FILE :
 *  CAVE_MachE_sil_test_dsrtmdlfcn.c
 *
 *
 *
 * Copyright 2017, dSPACE GmbH. All rights reserved
 *
   \****************************************************************************/

#ifndef TEMP_DSRT_RTI
#include <rtmodel.h>
#include "rtwtypes.h"
#include "CAVE_MachE_sil_test_dsrtmdlfcn.h"
#include <stdio.h>

int32_T DSRTStopSimulation = 0;
EXTERN_C_DECL void CAVE_MachE_sil_test_initialize();
EXTERN_C_DECL void CAVE_MachE_sil_test_terminate();

/* Initialization of model */
void CAVE_MachE_sil_test_DSRTInitMdl(void)
{
  /* Initialize model */
  CAVE_MachE_sil_test_initialize();

  /* Check for initialization errors */
  CAVE_MachE_sil_test_DSRTCheckForErrorStatus();

  /* Reinit Stop Simulation flag*/
  DSRTStopSimulation = 0;
}

/* Model start function */
void CAVE_MachE_sil_test_DSRTStartMdl(void)
{
  uint8_T lastApplStateStopped;
  CAVE_MachE_sil_test_APLastApplStateStopped(&lastApplStateStopped);
  if (lastApplStateStopped) {
    /* Reinit main simulation structure */
    CAVE_MachE_sil_test_DSRTInitMdl();
  }
}

/* Model stop function */
void CAVE_MachE_sil_test_DSRTStopMdl(void)
{
  /* Call terminate function */
  CAVE_MachE_sil_test_terminate();

  /* Check for model error status */
  CAVE_MachE_sil_test_DSRTCheckForErrorStatus();
}

/* Check for model error status */
void CAVE_MachE_sil_test_DSRTCheckForErrorStatus()
{
  const char_T* errorStatus = rtmGetErrorStatus(CAVE_MachE_sil_test_M);
  if (errorStatus != (char *) NULL && !strcmp(errorStatus, "Simulation finished"))
  {
    /* The RTM errorStatus field has been set */
    CAVE_MachE_sil_test_APTerminateSimulation();
    CAVE_MachE_sil_test_APPrintMessage(
      "Model 'CAVE_MachE_sil_test' has set error status:");
    CAVE_MachE_sil_test_APPrintMessage(errorStatus);
  }
}

/* Check for simulation stop conditions */
void CAVE_MachE_sil_test_DSRTCheckForSimulationStopCondition(void)
{
  const char_T* errorStatus = rtmGetErrorStatus(CAVE_MachE_sil_test_M);
  if (errorStatus != (char *) NULL) {
    if (DSRTStopSimulation != 1) {
      /* The RTM errorStatus field has been set */
      CAVE_MachE_sil_test_APTerminateSimulation();
      CAVE_MachE_sil_test_APPrintMessage(
        "Model 'CAVE_MachE_sil_test' has set error status:");
      CAVE_MachE_sil_test_APPrintMessage(errorStatus);
      DSRTStopSimulation = 1;
    }
  } else if (rtmGetStopRequested(CAVE_MachE_sil_test_M)) {
    if (DSRTStopSimulation != 1) {
      CAVE_MachE_sil_test_APStopSimulation();
      CAVE_MachE_sil_test_APPrintMessage(
        "Model 'CAVE_MachE_sil_test': A Simulink Stop Simulation block or request has been executed.");
      DSRTStopSimulation = 1;
    }
  }
}

#endif
