/*****************************************************************************

   Include file tmp_dsrttf.c:

   Definition of task functions.

   Mon Jul 10 17:36:05 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header files */
#include "tmp_dsrttf.h"
#include "tmp.h"
#include "tmp_private.h"

/* Task function for TID0 */
void tmp_DSRTMdlOutputs0()
{
  /* Call to Simulink model output function */
  tmp_output();
}

void tmp_DSRTMdlUpdate0()
{
  /* Call to Simulink model upadte function */
  tmp_update();
}
