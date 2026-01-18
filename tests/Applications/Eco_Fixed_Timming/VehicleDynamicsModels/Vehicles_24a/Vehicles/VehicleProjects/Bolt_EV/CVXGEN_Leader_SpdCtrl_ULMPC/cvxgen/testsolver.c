/* Produced by CVXGEN, 2022-12-01 16:21:36 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.5507979025745755;
  params.v_ref_1[0] = 0.8325912904724193;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.2909047389129444;
  params.v_ref_2[0] = 0.04331042079065206;
  params.v_ref_3[0] = 1.5717878173906188;
  params.v_ref_4[0] = 1.5851723557337523;
  params.v_ref_5[0] = -1.497658758144655;
  params.v_ref_6[0] = -1.171028487447253;
  params.v_ref_7[0] = -1.7941311867966805;
  params.v_ref_8[0] = -0.23676062539745413;
  params.v_ref_9[0] = -1.8804951564857322;
  params.v_ref_10[0] = -0.17266710242115568;
  params.v_ref_11[0] = 0.596576190459043;
  params.v_ref_12[0] = -0.8860508694080989;
  params.v_ref_13[0] = 0.7050196079205251;
  params.v_ref_14[0] = 0.3634512696654033;
  params.v_ref_15[0] = -1.9040724704913385;
  params.v_ref_16[0] = 0.23541635196352795;
  params.v_ref_17[0] = -0.9629902123701384;
  params.v_ref_18[0] = -0.3395952119597214;
  params.v_ref_19[0] = -0.865899672914725;
  params.v_ref_20[0] = 0.7725516732519853;
  params.v_ref_21[0] = -0.23818512931704205;
  params.A[0] = -1.372529046100147;
  params.x_0[0] = 0.17859607212737894;
  params.B[0] = 1.1212590580454682;
  params.d_k[0] = -0.774545870495281;
  params.vmin[0] = -1.1121684642712744;
  params.vmax[0] = -0.44811496977740495;
  params.deltav_min[0] = 1.7455345994417217;
  params.deltav_max[0] = 1.9039816898917352;
  params.delta_vcmd_min[0] = 0.6895347036512547;
  params.delta_vcmd_max[0] = 1.6113364341535923;
  params.vcmd_min[0] = 1.383003485172717;
  params.v_cmd_last[0] = -0.48802383468444344;
  params.vcmd_max[0] = -1.631131964513103;
}
