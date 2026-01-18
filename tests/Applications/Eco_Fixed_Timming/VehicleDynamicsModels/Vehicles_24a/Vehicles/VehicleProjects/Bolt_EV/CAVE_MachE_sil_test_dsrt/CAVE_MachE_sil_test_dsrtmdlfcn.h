/*****************************************************************************

   Include file CAVE_MachE_sil_test_dsrtmdlfcn.h:

   Definition of model functions.

   Tue Aug 22 23:31:18 2023

   Copyright 2017, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_CAVE_MachE_sil_test_DSRTMDLFCN_HEADER_
#define _DSRT_CAVE_MachE_sil_test_DSRTMDLFCN_HEADER_
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
EXTERN_C_DECL void CAVE_MachE_sil_test_DSRTInitMdl(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_DSRTStartMdl(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_DSRTStopMdl(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_DSRTCheckForSimulationStopCondition(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_DSRTCheckForErrorStatus(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_APTerminateSimulation(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_APStopSimulation(void);
EXTERN_C_DECL void CAVE_MachE_sil_test_APLastApplStateStopped(uint8_T*);
EXTERN_C_DECL void CAVE_MachE_sil_test_APPrintMessage(const char_T*);

#endif                        /* _DSRT_CAVE_MachE_sil_test_DSRTMDLFCN_HEADER_ */
