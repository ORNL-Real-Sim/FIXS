/*********************** dSPACE target specific file *************************

   Include file tmp_dsrtmf.h:

   Definition of model functions.

   Mon Jul 10 17:36:05 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_tmp_DSRTMF_HEADER_
#define _DSRT_tmp_DSRTMF_HEADER_
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

/* Model functions declarations */
#define tmp_dsrt_mdl_ApSimEngineOnInitIoPreRtosInit()
#define dsrt_mdl_timesync_simstate()
#define tmp_dsrt_mdl_ApSimEngineIdle()
#endif                                 /* _DSRT_tmp_DSRTMF_HEADER_ */

/****** [EOF] ****************************************************************/
