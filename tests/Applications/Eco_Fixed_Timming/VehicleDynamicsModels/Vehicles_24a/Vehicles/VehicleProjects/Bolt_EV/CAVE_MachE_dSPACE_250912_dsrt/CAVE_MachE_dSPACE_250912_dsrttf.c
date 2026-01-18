/*****************************************************************************

   Include file CAVE_MachE_dSPACE_250912_dsrttf.c:

   Definition of task functions.

   Sun Sep 21 17:10:19 2025

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header files */
#include "CAVE_MachE_dSPACE_250912_dsrttf.h"
#include "CAVE_MachE_dSPACE_250912.h"
#include "CAVE_MachE_dSPACE_250912_private.h"

/* Task function for TID0 */
void CAVE_MachE_dSPACE_250912_DSRTMdlOutputs0()
{
  /* Call to Simulink model output function */
  CAVE_MachE_dSPACE_250912_output();
}

void CAVE_MachE_dSPACE_250912_DSRTMdlUpdate0()
{
  /* Call to Simulink model update function */
  CAVE_MachE_dSPACE_250912_update();
}
