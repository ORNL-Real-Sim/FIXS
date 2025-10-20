/***************************************************************************

   Source file tmp_dsrtvdf.c:

   Definition of function that initializes the global TRC pointers

   4.2p1 (11-Feb-2020)
   Mon Jul 10 17:36:05 2023

   Copyright 2023, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

/* Include header file. */
#include "tmp_dsrtvdf.h"
#include "tmp.h"
#include "tmp_private.h"

/* Compiler options to turn off optimization. */
#if !defined(DS_OPTIMIZE_INIT_TRC_POINTERS)
#ifdef _MCCPPC

#pragma options -nOt -nOr -nOi -nOx

#endif

#ifdef __GNUC__

#pragma GCC optimize ("O0")

#endif

#ifdef _MSC_VER

#pragma optimize ("", off)

#endif
#endif

/* Definition of Global pointers to data type transitions (for TRC-file access) */
volatile real_T *p_0_tmp_real_T_0 = NULL;
volatile real_T *p_1_tmp_real_T_0 = NULL;

/*
 *  Declare the functions, that initially assign TRC pointers
 */
static void rti_init_trc_pointers_0(void);

/* Global pointers to data type transitions are separated in different functions to avoid overloading */
static void rti_init_trc_pointers_0(void)
{
  p_0_tmp_real_T_0 = &tmp_B.sinewave;
  p_1_tmp_real_T_0 = &tmp_P.SineWave_Amp;
}

void tmp_rti_init_trc_pointers(void)
{
  rti_init_trc_pointers_0();
}
