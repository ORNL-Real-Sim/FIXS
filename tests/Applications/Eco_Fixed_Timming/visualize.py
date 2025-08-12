
import os
import warnings
warnings.filterwarnings('ignore')
import pandas as pd
import matplotlib as mpl
from evaluation_utils.data_utils import get_data_dics
from evaluation_utils.visualize_utils import (plot_ego_eval_two_conditions, 
                                              plot_ego_eval_two_conditions_compared_to_base, 
                                              plot_system_eval_two_conditions, 
                                              plot_space_time_diagram_two_conditions, 
                                              plot_speed_acc_distribution,
                                              plot_speed_acc_distribution_comparison,
                                              plot_ego_eval_two_conditions_merged,
                                              plot_speed_acc_distribution_comparison_ego_upstream,
                                              plot_ego_speed_acc_profile_comparison)
OUTPUT_DIR = r'.\\Results'
os.makedirs(OUTPUT_DIR, exist_ok=True)
SUMO_START_TIME = 28800
# Increase font sizes globally
mpl.rcParams['axes.titlesize'] = 40
mpl.rcParams['axes.labelsize'] = 20
mpl.rcParams['xtick.labelsize'] = 10
mpl.rcParams['ytick.labelsize'] = 15
mpl.rcParams['legend.fontsize'] = 30
mpl.rcParams['figure.titlesize'] = 30
# line width
mpl.rcParams['lines.linewidth'] = 4
data_dic_with_dynamics_after_calibration, data_dic_without_dynamics_after_calibration = get_data_dics('.\\Experiments_Sumo\\ACV3')
# data_dic_with_dynamics_after_calibration, data_dic_without_dynamics_after_calibration = get_data_dics('.\\Experiments_Sumo\\Debug')

# data_dic_with_dynamics_after_calibration = {
#     -1.0: data_dic_with_dynamics_after_calibration[-1.0],
#     0.0: data_dic_with_dynamics_after_calibration[0.0],
#     10.0: data_dic_with_dynamics_after_calibration[10.0],
#     20.0: data_dic_with_dynamics_after_calibration[20.0],
#     50.0: data_dic_with_dynamics_after_calibration[50.0],
#     100.0: data_dic_with_dynamics_after_calibration[100.0]
# }
# data_dic_without_dynamics_after_calibration = {
#     -1.0: data_dic_without_dynamics_after_calibration[-1.0],
#     0.0: data_dic_without_dynamics_after_calibration[0.0],
#     10.0: data_dic_without_dynamics_after_calibration[10.0],
#     20.0: data_dic_without_dynamics_after_calibration[20.0],
#     50.0: data_dic_without_dynamics_after_calibration[50.0],
#     100.0: data_dic_without_dynamics_after_calibration[100.0]
# }

# plot_ego_eval_two_conditions_compared_to_base(data_dic_with_dynamics_after_calibration, 
#                                               data_dic_without_dynamics_after_calibration, 
#                                               condition_1_label = 'Simulink Vehicle Dynamics', 
#                                               condition_2_label = 'SUMO Default Dynamics', 
#                                               simulink=True)

# plot_ego_eval_two_conditions_merged(data_dic_with_dynamics_after_calibration, 
#                              data_dic_without_dynamics_after_calibration, 
#                              condition_1_label = 'With Vehicle Dynamics', 
#                              condition_2_label = 'Without Vehicle Dynamics', 
#                              simulink=True)

vtMicroCoeff = pd.read_csv(r'evaluation_utils\\coefficients\\VTMicroCoeff.csv')
VehicleSrcCoeff = pd.read_csv(r'evaluation_utils\\coefficients\\VehicleSrcCoeff.csv')
data_dic_with_dynamics_after_calibration.pop(-1.0)
data_dic_without_dynamics_after_calibration.pop(-1.0)
# plot_system_eval_two_conditions(data_dic_with_dynamics_after_calibration, 
#                                 data_dic_without_dynamics_after_calibration, 
#                                 vtMicroCoeff=vtMicroCoeff,
#                                 VehicleSrcCoeff=VehicleSrcCoeff,
#                                 condition_1_label = 'Simulink Vehicle Dynamics', 
#                                 condition_2_label = 'SUMO Default Dynamics',
#                                 output_dir = OUTPUT_DIR)

plot_space_time_diagram_two_conditions(data_dic_with_dynamics_after_calibration,
                                        data_dic_without_dynamics_after_calibration,
                                        condition_1_label = 'Simulink Vehicle Dynamics',
                                        condition_2_label = 'SUMO Default Dynamics',
                                        output_dir = OUTPUT_DIR)

# plot_space_time_diagram_two_conditions(data_dic_without_dynamics_after_calibration,
#                                         None,
#                                         condition_1_label = 'Without Vehicle Dynamics',
#                                         condition_2_label = None,
#                                         output_dir = OUTPUT_DIR)
# data_dic_with_dynamics_after_calibration.pop(-1.0)
# data_dic_without_dynamics_after_calibration.pop(-1.0)
# plot_speed_acc_distribution(data_dic_with_dynamics_after_calibration,
#                              data_label = 'With Vehicle Dynamics',
#                              output_dir = OUTPUT_DIR)
# plot_speed_acc_distribution(data_dic_without_dynamics_after_calibration,
#                              data_label = 'Without Vehicle Dynamics',
#                              output_dir = OUTPUT_DIR)
# plot_speed_acc_distribution_comparison(data_dic_with_dynamics_after_calibration,
#                                         data_dic_without_dynamics_after_calibration,
#                                         condition_1_label = 'With Vehicle Dynamics',
#                                         condition_2_label = 'Without Vehicle Dynamics',
#                                         output_dir = OUTPUT_DIR)

# plot_speed_acc_distribution_comparison_ego_upstream(data_dic_with_dynamics_after_calibration,
#                                                 data_dic_without_dynamics_after_calibration,
#                                                 condition_1_label = 'Simulink Vehicle Dynamics',
#                                                 condition_2_label = 'SUMO Default Dynamics',
#                                                 output_dir = OUTPUT_DIR,
#                                                 mode = 'ego')
# plot_speed_acc_distribution_comparison_ego_upstream(data_dic_with_dynamics_after_calibration,
#                                                 data_dic_without_dynamics_after_calibration,
#                                                 condition_1_label = 'Simulink Vehicle Dynamics',
#                                                 condition_2_label = 'SUMO Default Dynamics',
#                                                 output_dir = OUTPUT_DIR,
#                                                 upstream_max_vehicles_count=2,
#                                                 mode = 'upstream')
# plot_speed_acc_distribution_comparison_ego_upstream(data_dic_with_dynamics_after_calibration,
#                                                 data_dic_without_dynamics_after_calibration,
#                                                 condition_1_label = 'Simulink Vehicle Dynamics',
#                                                 condition_2_label = 'SUMO Default Dynamics',
#                                                 output_dir = OUTPUT_DIR,
#                                                 upstream_max_vehicles_count=5,
#                                                 mode = 'upstream')
# plot_ego_speed_acc_profile_comparison(data_dic_with_dynamics_after_calibration,
#                                   data_dic_without_dynamics_after_calibration,
#                                   condition_1_label = 'Simulink Vehicle Dynamics',
#                                   condition_2_label = 'SUMO Default Dynamics',
#                                   output_dir = OUTPUT_DIR,
#                                   direction='WB',
#                                   trip_id='0',
#                                   include_acceleration=False,
#                                   mode = 'over_distance')
# plot_ego_speed_acc_profile_comparison(data_dic_with_dynamics_after_calibration,
#                                   data_dic_without_dynamics_after_calibration,
#                                   condition_1_label = 'Simulink Vehicle Dynamics',
#                                   condition_2_label = 'SUMO Default Dynamics',
#                                   output_dir = OUTPUT_DIR,
#                                   direction='WB',
#                                   trip_id='0',
#                                   include_acceleration=False,
#                                   mode = 'over_time')