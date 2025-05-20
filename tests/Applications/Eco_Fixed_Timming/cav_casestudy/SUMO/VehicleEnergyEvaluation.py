import pandas as pd
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
from spaceTimePlotWithSignals import read_trajectory_xml, remove_none_edges
from utils1 import vtcpmf_fuel_model, energyVTMicro, cal_trac_energy, getICVenergy, getEVenergy
import os
import time
from scipy.spatial.distance import cdist
from spaceTimePlotWithSignals import get_exclude_turn_traj


def get_traj_eval(eco_driving_folder, step_length, vtMicroCoeff, df_vehicle_src_coeff, refer_coord, direction='Both', veh_no='system', major_corridor_only=True):
    trajectory_data_eco_driving = read_trajectory_xml(os.path.join(eco_driving_folder, 'fcd.xml'), refer_coord)
    trajectory_data_eco_driving = trajectory_data_eco_driving[(trajectory_data_eco_driving['time'].astype(float) >= 29100)
                                                              & (trajectory_data_eco_driving['time'].astype(float) <= 32100)].reset_index(drop=True)

    if major_corridor_only:
        # only keep the major corridor trajectories
        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']

        trajectory_data_eco_driving['segment'] = trajectory_data_eco_driving['lane'].str.split('_').str[0]
        trajectory_data_eco_driving['direction'] = np.where(trajectory_data_eco_driving['segment'].isin(wb_lanes), "WB",
                                                np.where(trajectory_data_eco_driving['segment'].isin(eb_lanes), "EB", None))
        trajectory_data_eco_driving = trajectory_data_eco_driving.sort_values(by=['id', 'time'], ignore_index=True)

        trajectory_data_eco_driving = trajectory_data_eco_driving[(trajectory_data_eco_driving['direction'].notnull())
                                                                  | (trajectory_data_eco_driving['segment'].str.contains(':'))]

        # Forward fill and backward fill within each group
        trajectory_data_eco_driving = remove_none_edges(trajectory_data_eco_driving, group_col="id", value_col="direction")

        # Forward fill and backward fill within each group
        trajectory_data_eco_driving["direction"] = trajectory_data_eco_driving.groupby("id")["direction"].ffill()

        # remove turning part of the trajectories
        trajectory_data_eco_driving = get_exclude_turn_traj(trajectory_data_eco_driving, exclude_turning_traj=True)

        if direction != 'Both':
            trajectory_data_eco_driving = trajectory_data_eco_driving[trajectory_data_eco_driving['direction'] == direction].reset_index(drop=True)

    # clean speed and acceleration, m/s and m/s2
    trajectory_data_eco_driving['speed'] = trajectory_data_eco_driving['speed'].astype(float)
    trajectory_data_eco_driving['acceleration'] = (trajectory_data_eco_driving['speed'].shift(-1) - trajectory_data_eco_driving['speed'])/step_length  # m/s2
    # trajectory_data_eco_driving['acceleration'] = (trajectory_data_eco_driving['speed'] - trajectory_data_eco_driving['speed'].shift(1))/step_length  # m/s2

    # re-estimate the travel distance second by second
    trajectory_data_eco_driving['distance_to_next'] = np.sqrt(
        (trajectory_data_eco_driving['x'].astype(float) - trajectory_data_eco_driving['x'].astype(float).shift(-1)) ** 2 +
        (trajectory_data_eco_driving['y'].astype(float) - trajectory_data_eco_driving['y'].astype(float).shift(-1)) ** 2
    )

    # delete the last two seconds trajectory for every vehicle (due to the calculation of acceleration)
    trajectory_data_eco_driving = trajectory_data_eco_driving.groupby('id').apply(lambda x: x.iloc[:-1] if len(x) > 1 else x).reset_index(drop=True)

    # trajectory_data_eco_driving['acceleration'] = trajectory_data_eco_driving['acceleration'].round(6)
    trajectory_data_eco_driving = trajectory_data_eco_driving[trajectory_data_eco_driving['acceleration'].notna()].reset_index(drop=True)
    trajectory_data_eco_driving['speed_kph'] = trajectory_data_eco_driving['speed'] * 3.6

    # trajectory_data_eco_driving['fuel_rate_vt_cpmf'] = np.vectorize(vtcpmf_fuel_model)(trajectory_data_eco_driving)
    start_time = time.time()

    # call the vtcpmf model to get the fuel consumption
    trajectory_data_eco_driving['fuel_rate_vt_cpmf'] = trajectory_data_eco_driving.apply(lambda row: vtcpmf_fuel_model(row['speed_kph'], row['acceleration']), axis=1)

    # call the VT-Micro function to get the energy consumption
    trajectory_data_eco_driving = energyVTMicro(trajectory_data_eco_driving, vtMicroCoeff, M=1.6443)

    # call the tractive energy function to get the tractive energy
    trajectory_data_eco_driving = cal_trac_energy(trajectory_data_eco_driving, df_vehicle_src_coeff)

    # call the tractive power-based energy evaluation from Yunli
    trajectory_data_eco_driving['fuel_rate_yunli_energy_ICV_liter'] = trajectory_data_eco_driving.apply(lambda row: getICVenergy(row['speed'], row['acceleration']), axis=1)
    trajectory_data_eco_driving['battery_power_yunli_energy_EV_kw'] = trajectory_data_eco_driving.apply(lambda row: getEVenergy(row['speed'], row['acceleration']), axis=1)

    trajectory_data_energy_sum = trajectory_data_eco_driving.groupby(by='id', as_index=False).agg({'speed_kph': ['first', 'last'],
                                                                                                   'distance': ['first', 'last'],
                                                                                                   'distance_to_next': 'sum',
                                                                                                   'time': ['first', 'last'],
                                                                                                   'fuel_rate_vt_cpmf': 'sum',
                                                                                                   'instantFuelConsumptionVTMicro': 'sum',
                                                                                                   'trac_power': 'sum',
                                                                                                   'trac_power_regen': 'sum',
                                                                                                   'fuel_rate_yunli_energy_ICV_liter': 'sum',
                                                                                                   'battery_power_yunli_energy_EV_kw': 'sum'})

    trajectory_data_energy_sum.columns = ['id', 'speed_kph_first', 'speed_kph_last', 'distance_first', 'distance_last', 'distance_to_next_sum',
                                          'time_first', 'time_last', 'fuel_consume_vt_cpmf_liter', 'fuel_rate_vt_micro_liter',
                                          'trac_energy_kj', 'trac_energy_regen_kj', 'fuel_consume_yunli_icv_liter', 'energy_consume_yunli_ev_kj']

    trajectory_data_energy_sum['travel_dist_m'] = abs(trajectory_data_energy_sum['distance_last'] - trajectory_data_energy_sum['distance_first'])
    trajectory_data_energy_sum['travel_time'] = trajectory_data_energy_sum['time_last'].astype(float) - trajectory_data_energy_sum['time_first'].astype(float)

    trajectory_data_energy_sum['fuel_consume_vt_cpmf_liter'] = trajectory_data_energy_sum['fuel_consume_vt_cpmf_liter'] * step_length
    trajectory_data_energy_sum['fuel_rate_vt_micro_liter'] = trajectory_data_energy_sum['fuel_rate_vt_micro_liter'] * step_length
    trajectory_data_energy_sum['trac_energy_kj'] = trajectory_data_energy_sum['trac_energy_kj'] * step_length
    trajectory_data_energy_sum['trac_energy_regen_kj'] = trajectory_data_energy_sum['trac_energy_regen_kj'] * step_length
    trajectory_data_energy_sum['fuel_consume_yunli_icv_liter'] = trajectory_data_energy_sum['fuel_consume_yunli_icv_liter'] * step_length
    trajectory_data_energy_sum['energy_consume_yunli_ev_kj'] = trajectory_data_energy_sum['energy_consume_yunli_ev_kj'] * step_length

    if veh_no == 'ego':
        trajectory_data_energy_sum = trajectory_data_energy_sum[trajectory_data_energy_sum['id'] == veh_no].reset_index(drop=True)

    print("Time Spent in the Last Step", time.time() - start_time)

    system_fuel_consume_vt_cpmf_liter = trajectory_data_energy_sum['fuel_consume_vt_cpmf_liter'].sum()
    system_fuel_consume_vt_micro_liter = trajectory_data_energy_sum['fuel_rate_vt_micro_liter'].sum()
    system_energy_consume_tractive_kj = trajectory_data_energy_sum['trac_energy_kj'].sum()
    system_energy_consume_tractive_regen_kj = trajectory_data_energy_sum['trac_energy_regen_kj'].sum()
    system_fuel_consume_yunli_icv_liter = trajectory_data_energy_sum['fuel_consume_yunli_icv_liter'].sum()
    system_energy_consume_yunli_ev_kj = trajectory_data_energy_sum['energy_consume_yunli_ev_kj'].sum()

    system_travel_dist_m = trajectory_data_energy_sum['travel_dist_m'].sum()
    system_travel_dist2_m = trajectory_data_energy_sum['distance_to_next_sum'].sum()
    system_travel_time_s = trajectory_data_energy_sum['travel_time'].sum()

    return (trajectory_data_energy_sum, system_fuel_consume_vt_cpmf_liter, system_fuel_consume_vt_micro_liter, system_energy_consume_tractive_kj,
            system_energy_consume_tractive_regen_kj, system_fuel_consume_yunli_icv_liter, system_energy_consume_yunli_ev_kj, system_travel_dist2_m, system_travel_time_s)


def plot_energy_comparison(trajectory_data_baseline, trajectory_data_eco_driving):
    print('Test')

    # plot
    fig = plt.figure()
    fig.set_size_inches(15, 10)
    plt.rcParams.update({'font.size': 20})

    return None


if __name__ == "__main__":
    # define a reference point coordinates at the leftmost point to calculate the distance over the corridor
    refer_coord = [200, 709]
    step_length = 1

    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/0%_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/100%_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/0%_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/10%_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V2/MPR/100%_Seed100_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR/100%_Seed100_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR/20%_Seed100_Subscription_1Hz'
    # eco_driving_folder = r'Shallowford_before_calibration_V3/MPR/100%_Seed100_Subscription_1Hz'
    eco_driving_folder = r'Shallowford_before_calibration_V3/MPR/50%_Seed100_Subscription_1Hz'

    # baseline_folder = r'BaselineWithoutDynamicsMaxRecall_10Hz_LoopRoute'
    vtMicroCoeff = pd.read_csv(r'VTMicroCoeff.csv')
    df_vehicle_src_coeff = pd.read_csv(r'VehicleSrcCoeff.csv')

    (energy_evaluation_summary, system_fuel_consume_vt_cpmf_liter, system_fuel_consume_vt_micro_liter,
     system_energy_consume_tractive_kj, system_energy_consume_tractive_regen_kj, system_fuel_consume_yunli_icv_liter,
     system_energy_consume_yunli_ev_kj, system_travel_dist2_m, system_travel_time_s) = (get_traj_eval(eco_driving_folder, step_length, vtMicroCoeff, df_vehicle_src_coeff, refer_coord, 'Both'))

    # trajectory_data_baseline = get_traj_eval(baseline_folder, step_length)
    system_fuel_efficiency_cpmf_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_cpmf_liter/3.78541)
    system_fuel_efficiency_micro_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_micro_liter/3.78541)
    system_fuel_efficiency_tractive_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_tractive_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
    system_fuel_efficiency_yunli_icv_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_yunli_icv_liter/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
    system_fuel_efficiency_yunli_ev_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_yunli_ev_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)

    print("\n system_fuel_consume_vt_cpmf_liter", system_fuel_consume_vt_cpmf_liter,
          "\n system_fuel_efficiency_cpmf_mpg", system_fuel_efficiency_cpmf_mpg, '\n',
          "\n system_fuel_consume_vt_micro_liter", system_fuel_consume_vt_micro_liter,
          "\n system_fuel_efficiency_micro_mpg", system_fuel_efficiency_micro_mpg, '\n',
          "\n system_energy_consume_tractive_kj", system_energy_consume_tractive_kj,
          "\n system_fuel_efficiency_tractive_mpg", system_fuel_efficiency_tractive_mpg, '\n',
          "\n system_fuel_consume_yunli_icv_liter", system_fuel_consume_yunli_icv_liter,
          "\n system_fuel_efficiency_yunli_icv_mpg", system_fuel_efficiency_yunli_icv_mpg, '\n',
          "\n system_energy_consume_yunli_ev_kj", system_energy_consume_yunli_ev_kj,
          "\n system_fuel_efficiency_yunli_ev_mpg", system_fuel_efficiency_yunli_ev_mpg, '\n',

          "\n system_travel_dist2_mile", system_travel_dist2_m/1609.34,
          "\n system_travel_time_h", system_travel_time_s/3600,
          "\n system_space_mean_speed_mph", (system_travel_dist2_m/system_travel_time_s) * 2.23694,
          "\n vehicle_counts", len(energy_evaluation_summary),"\n")

    print('Test')