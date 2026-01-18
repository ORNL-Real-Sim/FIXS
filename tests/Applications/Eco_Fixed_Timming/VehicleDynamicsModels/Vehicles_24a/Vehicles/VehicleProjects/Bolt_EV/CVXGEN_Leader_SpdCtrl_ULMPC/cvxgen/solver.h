/* Produced by CVXGEN, 2022-12-01 16:21:36 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double R[1];
  double v_ref_1[1];
  double Q[1];
  double v_ref_2[1];
  double v_ref_3[1];
  double v_ref_4[1];
  double v_ref_5[1];
  double v_ref_6[1];
  double v_ref_7[1];
  double v_ref_8[1];
  double v_ref_9[1];
  double v_ref_10[1];
  double v_ref_11[1];
  double v_ref_12[1];
  double v_ref_13[1];
  double v_ref_14[1];
  double v_ref_15[1];
  double v_ref_16[1];
  double v_ref_17[1];
  double v_ref_18[1];
  double v_ref_19[1];
  double v_ref_20[1];
  double v_ref_21[1];
  double A[1];
  double x_0[1];
  double B[1];
  double d_k[1];
  double vmin[1];
  double vmax[1];
  double deltav_min[1];
  double deltav_max[1];
  double delta_vcmd_min[1];
  double delta_vcmd_max[1];
  double vcmd_min[1];
  double v_cmd_last[1];
  double vcmd_max[1];
  double *v_ref[22];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *u_0; /* 1 rows. */
  double *x_1; /* 1 rows. */
  double *x_2; /* 1 rows. */
  double *x_3; /* 1 rows. */
  double *x_4; /* 1 rows. */
  double *x_5; /* 1 rows. */
  double *x_6; /* 1 rows. */
  double *x_7; /* 1 rows. */
  double *x_8; /* 1 rows. */
  double *x_9; /* 1 rows. */
  double *x_10; /* 1 rows. */
  double *x_11; /* 1 rows. */
  double *x_12; /* 1 rows. */
  double *x_13; /* 1 rows. */
  double *x_14; /* 1 rows. */
  double *x_15; /* 1 rows. */
  double *x_16; /* 1 rows. */
  double *x_17; /* 1 rows. */
  double *x_18; /* 1 rows. */
  double *x_19; /* 1 rows. */
  double *x_20; /* 1 rows. */
  double *x_21; /* 1 rows. */
  double *u_1; /* 1 rows. */
  double *u_2; /* 1 rows. */
  double *u_3; /* 1 rows. */
  double *u_4; /* 1 rows. */
  double *u_5; /* 1 rows. */
  double *u_6; /* 1 rows. */
  double *u_7; /* 1 rows. */
  double *u_8; /* 1 rows. */
  double *u_9; /* 1 rows. */
  double *u_10; /* 1 rows. */
  double *u_11; /* 1 rows. */
  double *u_12; /* 1 rows. */
  double *u_13; /* 1 rows. */
  double *u_14; /* 1 rows. */
  double *u_15; /* 1 rows. */
  double *u_16; /* 1 rows. */
  double *u_17; /* 1 rows. */
  double *u_18; /* 1 rows. */
  double *u_19; /* 1 rows. */
  double *u_20; /* 1 rows. */
  double *u[21];
  double *x[22];
} Vars;
typedef struct Workspace_t {
  double h[88];
  double s_inv[88];
  double s_inv_z[88];
  double b[41];
  double q[42];
  double rhs[259];
  double x[259];
  double *s;
  double *z;
  double *y;
  double lhs_aff[259];
  double lhs_cc[259];
  double buffer[259];
  double buffer2[259];
  double KKT[496];
  double L[319];
  double d[259];
  double v[259];
  double d_inv[259];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_707373629440[1];
  double quad_173000376320[1];
  double quad_70838104064[1];
  double quad_56861499392[1];
  double quad_991650349056[1];
  double quad_520570773504[1];
  double quad_70692519936[1];
  double quad_895518380032[1];
  double quad_786727555072[1];
  double quad_965331148800[1];
  double quad_617404977152[1];
  double quad_811699965952[1];
  double quad_526165987328[1];
  double quad_846159978496[1];
  double quad_206481264640[1];
  double quad_378133397504[1];
  double quad_598395097088[1];
  double quad_959236141056[1];
  double quad_680671576064[1];
  double quad_3538714624[1];
  double quad_347969859584[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
