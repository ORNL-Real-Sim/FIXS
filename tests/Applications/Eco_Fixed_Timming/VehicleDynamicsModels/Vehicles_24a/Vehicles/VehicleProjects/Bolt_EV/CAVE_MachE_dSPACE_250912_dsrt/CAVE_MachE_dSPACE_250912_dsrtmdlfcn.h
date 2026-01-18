/*****************************************************************************

   Include file CAVE_MachE_dSPACE_250912_dsrtmdlfcn.h:

   Definition of model functions.

   Sun Sep 21 17:10:19 2025

   Copyright 2020, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_CAVE_MachE_dSPACE_250912_DSRTMDLFCN_HEADER_
#define _DSRT_CAVE_MachE_dSPACE_250912_DSRTMDLFCN_HEADER_
#if defined(EXTERN_C_DECL)
#undef EXTERN_C_DECL
#endif

#if defined(__cplusplus)
#define EXTERN_C_DECL                  extern "C"
#else
#define EXTERN_C_DECL                  extern
#endif

#include "rtwtypes.h"
#include <string.h>

/* Model functions declarations */
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_DSRTInitMdl(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_DSRTStartMdl(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_DSRTStopMdl(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_DSRTCheckForSimulationStopCondition
  (void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_DSRTCheckForErrorStatus(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_APTerminateSimulation(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_APStopSimulation(void);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_APLastApplStateStopped(uint8_T*);
EXTERN_C_DECL void CAVE_MachE_dSPACE_250912_APPrintMessage(const char_T*);

#endif                   /* _DSRT_CAVE_MachE_dSPACE_250912_DSRTMDLFCN_HEADER_ */
