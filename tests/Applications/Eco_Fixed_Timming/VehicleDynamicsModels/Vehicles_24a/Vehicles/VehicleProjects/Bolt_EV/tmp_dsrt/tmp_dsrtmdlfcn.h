/*****************************************************************************

   Include file tmp_dsrtmdlfcn.h:

   Definition of model functions.

   Mon Jul 10 17:36:05 2023

   Copyright 2017, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_tmp_DSRTMDLFCN_HEADER_
#define _DSRT_tmp_DSRTMDLFCN_HEADER_
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
EXTERN_C_DECL void tmp_DSRTInitMdl(void);
EXTERN_C_DECL void tmp_DSRTStartMdl(void);
EXTERN_C_DECL void tmp_DSRTStopMdl(void);
EXTERN_C_DECL void tmp_DSRTCheckForSimulationStopCondition(void);
EXTERN_C_DECL void tmp_DSRTCheckForErrorStatus(void);
EXTERN_C_DECL void tmp_APTerminateSimulation(void);
EXTERN_C_DECL void tmp_APStopSimulation(void);
EXTERN_C_DECL void tmp_APLastApplStateStopped(uint8_T*);
EXTERN_C_DECL void tmp_APPrintMessage(const char_T*);

#endif                                 /* _DSRT_tmp_DSRTMDLFCN_HEADER_ */
