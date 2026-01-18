/*****************************************************************************

   Include file EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_dsrtmdlfcn.h:

   Definition of model functions.

   Mon Jul 10 17:21:16 2023

   Copyright 2017, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTMDLFCN_HEADER_
#define _DSRT_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTMDLFCN_HEADER_
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
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTInitMdl(void);
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTStartMdl(void);
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTStopMdl(void);
EXTERN_C_DECL void
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForSimulationStopCondition
  (void);
EXTERN_C_DECL void
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTCheckForErrorStatus(void);
EXTERN_C_DECL void
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APTerminateSimulation(void);
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APStopSimulation
  (void);
EXTERN_C_DECL void
  EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APLastApplStateStopped(uint8_T*);
EXTERN_C_DECL void EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_APPrintMessage(
  const char_T*);

#endif /* _DSRT_EvBoltCalib_Lat_v1_comb_GBSRotoMachE_noCom_DSRTMDLFCN_HEADER_ */
