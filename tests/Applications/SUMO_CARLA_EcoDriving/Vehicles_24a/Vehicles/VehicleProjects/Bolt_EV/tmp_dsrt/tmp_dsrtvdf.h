/*********************** dSPACE target specific file *************************

   Header file tmp_dsrtvdf.h:

   Declaration of function that initializes the global TRC pointers

   4.2p1 (11-Feb-2020)
   Mon Jul 10 17:36:05 2023

   Copyright 2023, dSPACE GmbH. All rights reserved.

 *****************************************************************************/
#ifndef _DSRT_tmp_DSRTVDF_HEADER_
#define _DSRT_tmp_DSRTVDF_HEADER_

/* Include the model header file. */
#include "tmp.h"
#include "tmp_private.h"
#ifdef EXTERN_C
#undef EXTERN_C
#endif

#ifdef __cplusplus
#define EXTERN_C                       extern "C"
#else
#define EXTERN_C                       extern
#endif

/*
 *  Declare the global TRC pointers
 */
EXTERN_C volatile real_T *p_0_tmp_real_T_0;
EXTERN_C volatile real_T *p_1_tmp_real_T_0;

/*
 *  Declare the general function for TRC pointer initialization
 */
EXTERN_C void tmp_rti_init_trc_pointers(void);

#endif                                 /* _DSRT_tmp_DSRTVDF_HEADER_ */
