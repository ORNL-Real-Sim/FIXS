/*****************************************************************************

   Include file CAVE_MachE_sil_test_dsrttf.c:

   Definition of task functions.

   Tue Aug 22 23:31:18 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header files */
#include "CAVE_MachE_sil_test_dsrttf.h"
#include "CAVE_MachE_sil_test.h"
#include "CAVE_MachE_sil_test_private.h"

/* Task function for TID0 */
void CAVE_MachE_sil_test_DSRTMdlOutputs0()
{
  /* Call to Simulink model output function */
  CAVE_MachE_sil_test_output();
}

void CAVE_MachE_sil_test_DSRTMdlUpdate0()
{
  /* Call to Simulink model upadte function */
  CAVE_MachE_sil_test_update();
}
