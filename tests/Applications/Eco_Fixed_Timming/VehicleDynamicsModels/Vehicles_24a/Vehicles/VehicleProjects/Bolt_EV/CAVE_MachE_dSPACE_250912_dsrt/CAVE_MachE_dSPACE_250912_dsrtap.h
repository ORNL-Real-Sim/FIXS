/*********************** dSPACE target specific file *************************

   Include file CAVE_MachE_dSPACE_250912_dsrtap.h:

   Definitions for access points to Simulink root ports.

   Sun Sep 21 17:10:19 2025

   Copyright 2022, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_CAVE_MachE_dSPACE_250912_DSRTAP_HEADER_
#define _DSRT_CAVE_MachE_dSPACE_250912_DSRTAP_HEADER_
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

#ifndef DATA_PORT_ACCESS_POINT_API_VERSION
#error The data port access point API version is undefined.
#elif (DATA_PORT_ACCESS_POINT_API_VERSION > 2) || (DATA_PORT_ACCESS_POINT_API_VERSION < 1)
#error The defined data port access point API version is unsupported.
#endif

#if (DATA_PORT_ACCESS_POINT_API_VERSION == 2)
#include "CAVE_MachE_dSPACE_250912_types.h"
#else
#include "rtwtypes.h"
#endif

/* Extern declarations for access points */
#endif                       /* _DSRT_CAVE_MachE_dSPACE_250912_DSRTAP_HEADER_ */
