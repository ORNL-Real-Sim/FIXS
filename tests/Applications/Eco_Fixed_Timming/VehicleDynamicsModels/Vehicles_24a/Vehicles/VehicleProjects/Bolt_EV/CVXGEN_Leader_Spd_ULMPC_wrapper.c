
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <solver.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
Vars vars;
Params params;
Workspace work;
Settings settings;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void CVXGEN_Leader_Spd_ULMPC_Outputs_wrapper(const real_T *vmax,
			const real_T *vmin,
			const real_T *deltav_max,
			const real_T *deltav_min,
			const real_T *vcmd_max,
			const real_T *vcmd_min,
			const real_T *delta_vcmd_max,
			const real_T *delta_vcmd_min,
			const real_T *A,
			const real_T *B,
			const real_T *d_k,
			const real_T *Q,
			const real_T *R,
			const real_T *v_ref,
			const real_T *v_cmd_last,
			const real_T *x_0,
			real_T *delta_v_cmd_opt,
			real_T *v_cmd_opt)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
set_defaults();
setup_indexing();
settings.verbose   = 0;
settings.eps       = 1e-6;
settings.resid_tol = 1e-4;
settings.max_iters = 25;       

params.vmax[0]       = vmax[0];
params.vmin[0]       = vmin[0];
params.deltav_max[0] = deltav_max[0];
params.deltav_min[0] = deltav_min[0];

params.vcmd_max[0]   = vcmd_max[0];
params.vcmd_min[0]   = vcmd_min[0];
params.delta_vcmd_max[0] = delta_vcmd_max[0];
params.delta_vcmd_min[0] = delta_vcmd_min[0];

params.A[0] = A[0];
params.B[0] = B[0];
params.d_k[0] = d_k[0];

params.Q[0] = Q[0];

params.R[0]= R[0];

for (int i = 1; i < 22; i++){
    params.v_ref[i][0] = v_ref[i-1];
}

params.v_cmd_last[0] = v_cmd_last[0];

params.x_0[0] = x_0[0];

solve();

delta_v_cmd_opt[0]  = vars.u_0[0];
v_cmd_opt[0]        = vars.u_0[0] + v_cmd_last[0];
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


