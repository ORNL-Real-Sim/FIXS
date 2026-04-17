/*********************** dSPACE target specific file *************************

   Include file tmp_dsrttf.h:

   Extern declarations of task functions.

   Mon Jul 10 17:36:05 2023

   Copyright 2019, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#ifndef _DSRT_tmp_DSRTTF_HEADER_
#define _DSRT_tmp_DSRTTF_HEADER_
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

EXTERN_C void tmp_DSRTMdlOutputs0(void);
EXTERN_C void tmp_DSRTMdlUpdate0(void);

#endif                                 /* _DSRT_tmp_DSRTTF_HEADER_ */
