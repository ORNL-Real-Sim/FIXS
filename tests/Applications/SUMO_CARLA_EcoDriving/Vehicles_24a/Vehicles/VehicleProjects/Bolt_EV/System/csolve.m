% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(u_0, R) + quad_form(x_1 - v_ref_1, Q) + quad_form(x_2 - v_ref_2, Q) + quad_form(x_3 - v_ref_3, Q) + quad_form(x_4 - v_ref_4, Q) + quad_form(x_5 - v_ref_5, Q) + quad_form(x_6 - v_ref_6, Q) + quad_form(x_7 - v_ref_7, Q) + quad_form(x_8 - v_ref_8, Q) + quad_form(x_9 - v_ref_9, Q) + quad_form(x_10 - v_ref_10, Q) + quad_form(x_11 - v_ref_11, Q) + quad_form(x_12 - v_ref_12, Q) + quad_form(x_13 - v_ref_13, Q) + quad_form(x_14 - v_ref_14, Q) + quad_form(x_15 - v_ref_15, Q) + quad_form(x_16 - v_ref_16, Q) + quad_form(x_17 - v_ref_17, Q) + quad_form(x_18 - v_ref_18, Q) + quad_form(x_19 - v_ref_19, Q) + quad_form(x_20 - v_ref_20, Q) + quad_form(x_21 - v_ref_21, Q))
%   subject to
%     x_1 == A*x_0 + B*u_0 + d_k
%     x_2 == A*x_1 + B*u_1 + d_k
%     x_3 == A*x_2 + B*u_2 + d_k
%     x_4 == A*x_3 + B*u_3 + d_k
%     x_5 == A*x_4 + B*u_4 + d_k
%     x_6 == A*x_5 + B*u_5 + d_k
%     x_7 == A*x_6 + B*u_6 + d_k
%     x_8 == A*x_7 + B*u_7 + d_k
%     x_9 == A*x_8 + B*u_8 + d_k
%     x_10 == A*x_9 + B*u_9 + d_k
%     x_11 == A*x_10 + B*u_10 + d_k
%     x_12 == A*x_11 + B*u_11 + d_k
%     x_13 == A*x_12 + B*u_12 + d_k
%     x_14 == A*x_13 + B*u_13 + d_k
%     x_15 == A*x_14 + B*u_14 + d_k
%     x_16 == A*x_15 + B*u_15 + d_k
%     x_17 == A*x_16 + B*u_16 + d_k
%     x_18 == A*x_17 + B*u_17 + d_k
%     x_19 == A*x_18 + B*u_18 + d_k
%     x_20 == A*x_19 + B*u_19 + d_k
%     x_21 == A*x_20 + B*u_20 + d_k
%     vmin <= x_1
%     vmin <= x_2
%     vmin <= x_3
%     vmin <= x_4
%     vmin <= x_5
%     vmin <= x_6
%     vmin <= x_7
%     vmin <= x_8
%     vmin <= x_9
%     vmin <= x_10
%     vmin <= x_11
%     vmin <= x_12
%     vmin <= x_13
%     vmin <= x_14
%     vmin <= x_15
%     vmin <= x_16
%     vmin <= x_17
%     vmin <= x_18
%     vmin <= x_19
%     vmin <= x_20
%     vmin <= x_21
%     x_1 <= vmax
%     x_2 <= vmax
%     x_3 <= vmax
%     x_4 <= vmax
%     x_5 <= vmax
%     x_6 <= vmax
%     x_7 <= vmax
%     x_8 <= vmax
%     x_9 <= vmax
%     x_10 <= vmax
%     x_11 <= vmax
%     x_12 <= vmax
%     x_13 <= vmax
%     x_14 <= vmax
%     x_15 <= vmax
%     x_16 <= vmax
%     x_17 <= vmax
%     x_18 <= vmax
%     x_19 <= vmax
%     x_20 <= vmax
%     x_21 <= vmax
%     deltav_min <= x_1 - x_0
%     deltav_min <= x_2 - x_1
%     deltav_min <= x_3 - x_2
%     deltav_min <= x_4 - x_3
%     deltav_min <= x_5 - x_4
%     deltav_min <= x_6 - x_5
%     deltav_min <= x_7 - x_6
%     deltav_min <= x_8 - x_7
%     deltav_min <= x_9 - x_8
%     deltav_min <= x_10 - x_9
%     deltav_min <= x_11 - x_10
%     deltav_min <= x_12 - x_11
%     deltav_min <= x_13 - x_12
%     deltav_min <= x_14 - x_13
%     deltav_min <= x_15 - x_14
%     deltav_min <= x_16 - x_15
%     deltav_min <= x_17 - x_16
%     deltav_min <= x_18 - x_17
%     deltav_min <= x_19 - x_18
%     deltav_min <= x_20 - x_19
%     deltav_min <= x_21 - x_20
%     x_1 - x_0 <= deltav_max
%     x_2 - x_1 <= deltav_max
%     x_3 - x_2 <= deltav_max
%     x_4 - x_3 <= deltav_max
%     x_5 - x_4 <= deltav_max
%     x_6 - x_5 <= deltav_max
%     x_7 - x_6 <= deltav_max
%     x_8 - x_7 <= deltav_max
%     x_9 - x_8 <= deltav_max
%     x_10 - x_9 <= deltav_max
%     x_11 - x_10 <= deltav_max
%     x_12 - x_11 <= deltav_max
%     x_13 - x_12 <= deltav_max
%     x_14 - x_13 <= deltav_max
%     x_15 - x_14 <= deltav_max
%     x_16 - x_15 <= deltav_max
%     x_17 - x_16 <= deltav_max
%     x_18 - x_17 <= deltav_max
%     x_19 - x_18 <= deltav_max
%     x_20 - x_19 <= deltav_max
%     x_21 - x_20 <= deltav_max
%     delta_vcmd_min <= u_0
%     u_0 <= delta_vcmd_max
%     u_1 == 0
%     u_2 == 0
%     u_3 == 0
%     u_4 == 0
%     u_5 == 0
%     u_6 == 0
%     u_7 == 0
%     u_8 == 0
%     u_9 == 0
%     u_10 == 0
%     u_11 == 0
%     u_12 == 0
%     u_13 == 0
%     u_14 == 0
%     u_15 == 0
%     u_16 == 0
%     u_17 == 0
%     u_18 == 0
%     u_19 == 0
%     u_20 == 0
%     vcmd_min <= v_cmd_last + u_0
%     v_cmd_last + u_0 <= vcmd_max
%
% with variables
%      u_0   1 x 1
%      u_1   1 x 1
%      u_2   1 x 1
%      u_3   1 x 1
%      u_4   1 x 1
%      u_5   1 x 1
%      u_6   1 x 1
%      u_7   1 x 1
%      u_8   1 x 1
%      u_9   1 x 1
%     u_10   1 x 1
%     u_11   1 x 1
%     u_12   1 x 1
%     u_13   1 x 1
%     u_14   1 x 1
%     u_15   1 x 1
%     u_16   1 x 1
%     u_17   1 x 1
%     u_18   1 x 1
%     u_19   1 x 1
%     u_20   1 x 1
%      x_1   1 x 1
%      x_2   1 x 1
%      x_3   1 x 1
%      x_4   1 x 1
%      x_5   1 x 1
%      x_6   1 x 1
%      x_7   1 x 1
%      x_8   1 x 1
%      x_9   1 x 1
%     x_10   1 x 1
%     x_11   1 x 1
%     x_12   1 x 1
%     x_13   1 x 1
%     x_14   1 x 1
%     x_15   1 x 1
%     x_16   1 x 1
%     x_17   1 x 1
%     x_18   1 x 1
%     x_19   1 x 1
%     x_20   1 x 1
%     x_21   1 x 1
%
% and parameters
%        A   1 x 1
%        B   1 x 1
%        Q   1 x 1    PSD
%        R   1 x 1    PSD
%      d_k   1 x 1
% delta_vcmd_max   1 x 1
% delta_vcmd_min   1 x 1
% deltav_max   1 x 1
% deltav_min   1 x 1
% v_cmd_last   1 x 1
%  v_ref_1   1 x 1
%  v_ref_2   1 x 1
%  v_ref_3   1 x 1
%  v_ref_4   1 x 1
%  v_ref_5   1 x 1
%  v_ref_6   1 x 1
%  v_ref_7   1 x 1
%  v_ref_8   1 x 1
%  v_ref_9   1 x 1
% v_ref_10   1 x 1
% v_ref_11   1 x 1
% v_ref_12   1 x 1
% v_ref_13   1 x 1
% v_ref_14   1 x 1
% v_ref_15   1 x 1
% v_ref_16   1 x 1
% v_ref_17   1 x 1
% v_ref_18   1 x 1
% v_ref_19   1 x 1
% v_ref_20   1 x 1
% v_ref_21   1 x 1
% vcmd_max   1 x 1
% vcmd_min   1 x 1
%     vmax   1 x 1
%     vmin   1 x 1
%      x_0   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.x_0, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2022-12-01 16:21:35 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
