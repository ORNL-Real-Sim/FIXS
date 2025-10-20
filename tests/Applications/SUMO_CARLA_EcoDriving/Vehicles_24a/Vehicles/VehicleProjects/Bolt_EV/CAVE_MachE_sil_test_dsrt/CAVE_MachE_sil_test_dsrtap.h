/*********************** dSPACE target specific file *************************

   Include file CAVE_MachE_sil_test_dsrtap.h:

   Definitions for access points to Simulink root ports.

   Tue Aug 22 23:31:18 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_CAVE_MachE_sil_test_DSRTAP_HEADER_
#define _DSRT_CAVE_MachE_sil_test_DSRTAP_HEADER_
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
#include "CAVE_MachE_sil_test_types.h"
#else
#include "rtwtypes.h"
#endif

/* Extern declarations for access points */
#ifdef HAVE_CAVE_MachE_sil_test_DSRTAP_CUSTOM
#include "CAVE_MachE_sil_test_dsrtap_custom.h"
#endif
#endif                            /* _DSRT_CAVE_MachE_sil_test_DSRTAP_HEADER_ */
