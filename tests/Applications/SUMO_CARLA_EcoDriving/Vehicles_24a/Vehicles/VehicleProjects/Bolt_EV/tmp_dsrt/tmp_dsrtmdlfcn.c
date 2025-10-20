/**************************************************************************** *
 * FILE :
 *  tmp_dsrtmdlfcn.c
 *
 *
 *
 * Copyright 2017, dSPACE GmbH. All rights reserved
 *
   \****************************************************************************/

#ifndef TEMP_DSRT_RTI
#include <rtmodel.h>
#include "rtwtypes.h"
#include "tmp_dsrtmdlfcn.h"
#include <stdio.h>

int32_T DSRTStopSimulation = 0;
EXTERN_C_DECL void tmp_initialize();
EXTERN_C_DECL void tmp_terminate();

/* Initialization of model */
void tmp_DSRTInitMdl(void)
{
  /* Initialize model */
  tmp_initialize();

  /* Check for initialization errors */
  tmp_DSRTCheckForErrorStatus();

  /* Reinit Stop Simulation flag*/
  DSRTStopSimulation = 0;
}

/* Model start function */
void tmp_DSRTStartMdl(void)
{
  uint8_T lastApplStateStopped;
  tmp_APLastApplStateStopped(&lastApplStateStopped);
  if (lastApplStateStopped) {
    /* Reinit main simulation structure */
    tmp_DSRTInitMdl();
  }
}

/* Model stop function */
void tmp_DSRTStopMdl(void)
{
  /* Call terminate function */
  tmp_terminate();

  /* Check for model error status */
  tmp_DSRTCheckForErrorStatus();
}

/* Check for model error status */
void tmp_DSRTCheckForErrorStatus()
{
  const char_T* errorStatus = rtmGetErrorStatus(tmp_M);
  if (errorStatus != (char *) NULL && !strcmp(errorStatus, "Simulation finished"))
  {
    /* The RTM errorStatus field has been set */
    tmp_APTerminateSimulation();
    tmp_APPrintMessage("Model 'tmp' has set error status:");
    tmp_APPrintMessage(errorStatus);
  }
}

/* Check for simulation stop conditions */
void tmp_DSRTCheckForSimulationStopCondition(void)
{
  const char_T* errorStatus = rtmGetErrorStatus(tmp_M);
  if (errorStatus != (char *) NULL) {
    if (DSRTStopSimulation != 1) {
      /* The RTM errorStatus field has been set */
      tmp_APTerminateSimulation();
      tmp_APPrintMessage("Model 'tmp' has set error status:");
      tmp_APPrintMessage(errorStatus);
      DSRTStopSimulation = 1;
    }
  } else if (rtmGetStopRequested(tmp_M)) {
    if (DSRTStopSimulation != 1) {
      tmp_APStopSimulation();
      tmp_APPrintMessage("Model 'tmp': A Simulink Stop Simulation block or request has been executed.");
      DSRTStopSimulation = 1;
    }
  }
}

#endif
