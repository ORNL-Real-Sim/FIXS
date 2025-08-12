import pandas as pd
from cav_casestudy.SUMO.spaceTimePlotWithSignals import read_trajectory_xml, trajectory_process
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.stats import ks_2samp, mannwhitneyu, ttest_ind, wasserstein_distance

def get_acceleration_distance(data_dic: dict[float, dict[str, pd.DataFrame]]):
    for pr in data_dic:
        profile = data_dic[pr]['ego_profile']
        profile[profile['ActualSpeed'] < 0]['ActualSpeed'] = 0
        profile['Acceleration'] = profile['ActualSpeed'].diff()

        # fill the nan value with 0
        profile['Acceleration'] = profile['Acceleration'].fillna(0)

        for group_name, group in profile.groupby('TripId'):
            # claculate the energy consumption first Fuel Consumption = integral of Fuel vol flow rate
            energy_consumption = group['BatteryPower'] * group['Time'].diff().fillna(1)
            energy_consumption = energy_consumption.cumsum()
            energy_consumption_total = energy_consumption.iloc[-1] * 264.172 # convert to gallon
            # take the direction of the trip
            direction = group['Direction'].iloc[0]

            # calculate the per trip distance
            time = group['Time'] - group['Time'].iloc[0]
            distance = group['ActualSpeed'] * time.diff().fillna(1)
            distance = distance.cumsum()
            # set the distance for the trip
            profile.loc[profile['TripId'] == group_name, 'TravelDistance'] = distance
        profile['TravelDistance'] = profile['TravelDistance'].fillna(0)
        data_dic[pr]['ego_profile'] = profile
    return data_dic

def assign_trip_id(df: pd.DataFrame, distance_threshold: float):
        for veh_id, group in df.groupby('id'):
            group['tripId'] = (abs(group['distance'].diff()) > distance_threshold).cumsum()
            df.loc[df['id'] == veh_id, 'tripId'] = group['tripId']

        return df
    

def get_ev_energy_consumption(data: pd.DataFrame, step_length: float = 0.1):
    battery_power = data['BatteryPower']
    vehicle_speed = data['ActualSpeed']
    time = data['Time']
    # power w to kw
    battery_power = battery_power / 1000
    # tio usgal equivalent
    battery_power = battery_power / 33.7
    # time in hours
    battery_power = battery_power / 3600
    # gal to m^3
    battery_power = battery_power * 0.00378541
    # power * time step to get energy consumption
    energy_consumption = sum(battery_power * step_length)
    energy_consumption_total = energy_consumption * 264.172 # convert to gallon
    # calculate the distance in miles
    distance = sum(vehicle_speed * step_length)
    distance_total = distance * 0.000621371 # convert to miles
    # calculate the mpge
    mpge = distance_total / energy_consumption_total
    return energy_consumption_total


def getEVenergy(v: float, a: float) -> float:
    # Input:
    #   v: speed in m/s
    #   a: acceleration in m/s^2
    # Output:
    #   batteryPowerKw: battery power in kW
    #
    #   after getting power of each (v,a), multiply all battery power with delta time: batteryTotalKwh=sum(batteryPowerKw*dt/3600)
    #   after getting batteryTotalKwh, calculate equivalent gallon as batteryGallonEquivalent = batteryTotalKwh/33.7
    #   then can calculate MPGe using MPGe=distanceTotalMile/batteryGallonEquivalent
    p00 = 6.3331
    p10 = 0.6529
    p01 = 0.5910
    p20 = 0.0239
    p11 = 1.8354
    batteryPowerKw = p00 + p10*v + p01*a + p20*v**2 + p11*v*a
    return batteryPowerKw 


def get_energy_consumption(inupt_data_dic, separate_trips=False,direction='Both', simulink=True):
    trip_counts_wb = []
    trip_labels_wb = []
    trip_data_all_wb = []

    trip_counts_eb = []
    trip_labels_eb = []
    trip_data_all_eb = []

    for penetration_rate, data_dic in inupt_data_dic.items():
        if simulink:
            relative_time = 28800
            refer_coord = [160, 735]
            ego_profile: pd.DataFrame = data_dic['ego_profile'].copy()
            ego_profile['Time'] = ego_profile['Time'].astype(float) + 28985 - relative_time

            path = data_dic['path']
            data_all = read_trajectory_xml(os.path.join(path, 'fcd.xml'), refer_coord=refer_coord)
            
            wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
            eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']

            data_all = trajectory_process(data_all, eb_lanes, wb_lanes, relative_time)

            data_all = data_all[data_all['id'] == 'ego']
            data_all['time'] = data_all['time'].astype(float)
            data_all['distance_to_next'] = np.sqrt(
                (data_all['x'].astype(float) - data_all['x'].astype(float).shift(-1)) ** 2 +
                (data_all['y'].astype(float) - data_all['y'].astype(float).shift(-1)) ** 2
            )
            ego_profile['Direction'] = np.where(
                ego_profile['Time'].isin(data_all[data_all['direction'] == 'WB']['time']), 'WB',
                np.where(ego_profile['Time'].isin(data_all[data_all['direction'] == 'EB']['time']), 'EB', 'Unknown')
            )
            data_wb_part = ego_profile[ego_profile['Direction'] == 'WB'].copy()
            data_wb_part['distance_to_next'] = data_wb_part['Time'].map(data_all.set_index('time')['distance_to_next'])
            data_wb_part['distance'] = data_wb_part['Time'].map(data_all.set_index('time')['distance'])
            data_eb_part = ego_profile[ego_profile['Direction'] == 'EB'].copy()
            data_eb_part['distance_to_next'] = data_eb_part['Time'].map(data_all.set_index('time')['distance_to_next'])
            data_eb_part['distance'] = data_eb_part['Time'].map(data_all.set_index('time')['distance'])

            data_wb_part['id'] = 'ego'
            data_wb_part = assign_trip_id(data_wb_part, 100)
            data_eb_part['id'] = 'ego'
            data_eb_part = assign_trip_id(data_eb_part, 100)
            data_wb_part.to_csv(os.path.join(path, 'data_wb_part.csv'))
            data_eb_part.to_csv(os.path.join(path, 'data_eb_part.csv'))
        trip_data_wb = []
        trip_labels_wb.append(f'{penetration_rate} %' if penetration_rate != -1.0 else 'Base')
        if separate_trips:
            for trip_id, group in data_wb_part.groupby('tripId'):
                if simulink:
                    energy_consumption_total = get_ev_energy_consumption(group)
                    distance_total = group['distance_to_next'].sum() / 1609.34
                    mpge = distance_total / energy_consumption_total
                    trip_data_wb.append(mpge)
                    print(f'{penetration_rate} % WB: {mpge}')
                else:
                    raise ValueError("Simulink is not set to True. Please set it to True to use this function.")
            # cut the first and last one
            trip_counts_wb.append(len(trip_data_wb))
            trip_data_wb = trip_data_wb[1:-1]
            trip_data_all_wb.append(trip_data_wb)
        else:
            energy_consumption_total = get_ev_energy_consumption(data_wb_part)
            distance_total = data_wb_part['distance_to_next'].sum() / 1609.34
            mpge = distance_total / energy_consumption_total
            trip_counts_wb.append(1)
            trip_data_all_wb.append(mpge)
            print(f'{penetration_rate} % WB: {mpge}')
        

        trip_data_eb = []
        trip_labels_eb.append(f'{penetration_rate} %' if penetration_rate != -1.0 else 'Base')
        if separate_trips:
            for trip_id, group in data_eb_part.groupby('tripId'):
                if simulink:
                    energy_consumption_total = get_ev_energy_consumption(group)
                    distance_total = group['distance_to_next'].sum() / 1609.34
                    mpge = distance_total / energy_consumption_total
                    trip_data_eb.append(mpge)
                    print(f'{penetration_rate} % EB: {mpge}')
            trip_counts_eb.append(len(trip_data_eb))
            trip_data_eb = trip_data_eb[1:-1]
            trip_data_all_eb.append(trip_data_eb)
        else:
            energy_consumption_total = get_ev_energy_consumption(data_eb_part)
            distance_total = data_eb_part['distance_to_next'].sum() / 1609.34
            mpge = distance_total / energy_consumption_total
            trip_counts_eb.append(1)
            trip_data_all_eb.append(mpge)
            print(f'{penetration_rate} % EB: {mpge}')
            
    if direction == 'Both':
        # add based on labels
        trip_data_all = []
        trip_counts = []
        trip_labels = []
        for i in range(len(trip_labels_wb)):
            trip_data_all.append((trip_data_all_wb[i] + trip_data_all_eb[i]))
            trip_counts.append(trip_counts_wb[i] + trip_counts_eb[i])
            trip_labels.append(trip_labels_wb[i])
    
    elif direction == 'WB':
        trip_data_all = trip_data_all_wb
        trip_counts = trip_counts_wb
        trip_labels = trip_labels_wb
    elif direction == 'EB':
        trip_data_all = trip_data_all_eb
        trip_counts = trip_counts_eb
        trip_labels = trip_labels_eb
    else:
        raise ValueError("Invalid direction. Use 'Both', 'WB', or 'EB'.")


    return trip_data_all, trip_counts, trip_labels


# def plot_fuel_consumption_(data_dic_with_dynamics_before_calibration, data_dic_without_dynamics_before_calibration, data_dic_with_dynamics_after_calibration, data_dic_without_dynamics_after_calibration , simulink=True):
#     # Extract trip data and labels
#     # trip_data_before_calibration_with_dynamics -- list
#     trip_data_before_calibration_with_dynamics, trip_counts_before_calibration_with_dynamics, trip_labels_before_calibration_with_dynamics = get_energy_consumption(data_dic_with_dynamics_before_calibration, direction='WB', simulink=simulink)
#     trip_data_before_calibration_without_dynamics, trip_counts_before_calibration_without_dynamics, trip_labels_before_calibration_without_dynamics = get_energy_consumption(data_dic_without_dynamics_before_calibration,direction='WB', simulink=simulink)
    
#     trip_data_after_calibration_with_dynamics, trip_counts_after_calibration_with_dynamics, trip_labels_after_calibration_with_dynamics = get_energy_consumption(data_dic_with_dynamics_after_calibration,direction='WB', simulink=simulink)
#     trip_data_after_calibration_without_dynamics, trip_counts_after_calibration_without_dynamics, trip_labels_after_calibration_without_dynamics = get_energy_consumption(data_dic_without_dynamics_after_calibration,direction='WB', simulink=simulink)
#     # Calculate global y-axis limits
#     all_trip_data = trip_data_before_calibration_with_dynamics + trip_data_before_calibration_without_dynamics + trip_data_after_calibration_with_dynamics + trip_data_after_calibration_without_dynamics
#     # y_min, y_max = min(map(min, all_trip_data)) * 0.9, max(map(max, all_trip_data)) * 1.1  # Adding buffer for clarity
#     y_min, y_max = 60, 120
#     # use persentile to set the y axis limit
#     # Create subplots
#     fig, axes = plt.subplots(2, 2, figsize=(12, 8))
#     # fig, axes = plt.subplots(1, 2, figsize=(12, 4))
#     # Define function to create boxplots
#     def create_boxplot(ax, trip_data, trip_labels, title, trip_counts):
#         boxplot = ax.boxplot(
#             trip_data,
#             tick_labels=trip_labels,
#             showfliers=False,
#             patch_artist=True,  # Fill the boxes with color
#             boxprops=dict(facecolor='lightblue', color='blue'),  # Box properties
#             medianprops=dict(color='red', linewidth=2),  # Median line properties
#             whiskerprops=dict(color='blue', linewidth=1.5),  # Whisker properties
#             capprops=dict(color='blue', linewidth=1.5),  # Cap properties
#             flierprops=dict(marker='o', color='red', alpha=0.6)  # Outlier properties
#         )
#         ax.set_ylabel('Fuel Efficiency (MPGe)', fontsize=12, fontweight='bold')
#         ax.set_title(title, fontsize=14, fontweight='bold')
#         ax.grid(True, linestyle='--', alpha=0.6)
#         ax.set_ylim(y_min, y_max)  # Set uniform y-axis limits

#         # Add sample sizes as annotations
#         for i, count in enumerate(trip_counts):
#             ax.text(i + 1, y_max * 0.95, f'n={count}', ha='center', va='top', fontweight='bold', fontsize=10)

#     # Create four boxplots
#     create_boxplot(axes[0, 0], trip_data_before_calibration_with_dynamics, trip_labels_before_calibration_with_dynamics, 'Before Calibration (With Vehicle Dynamics)', trip_counts_before_calibration_with_dynamics)
#     create_boxplot(axes[0, 1], trip_data_before_calibration_without_dynamics, trip_labels_before_calibration_without_dynamics, 'Before Calibration (Without Vehicle Dynamics)', trip_counts_before_calibration_without_dynamics)
#     create_boxplot(axes[1, 0], trip_data_after_calibration_with_dynamics, trip_labels_after_calibration_with_dynamics, 'After Calibration (With Vehicle Dynamics)', trip_counts_after_calibration_with_dynamics)
#     create_boxplot(axes[1, 1], trip_data_after_calibration_without_dynamics, trip_labels_after_calibration_without_dynamics, 'After Calibration (Without Vehicle Dynamics)', trip_counts_after_calibration_without_dynamics)


#     # Adjust layout and show plot
#     plt.tight_layout()
#     plt.show()


import sumolib
from collections import deque
def get_lane_geometry(net_path):
    net: sumolib.net.Net = sumolib.net.readNet(net_path)
    lane_data = {}

    for edge in net.getEdges():
        edge: sumolib.net.edge.Edge = edge
        for lane in edge.getLanes():
            lane: sumolib.net.lane.Lane = lane
            lane_id = lane.getID()
            shape = lane.getShape()
            successors = [conn.getToLane().getID() for conn in lane.getOutgoing()]
            lane_data[lane_id] = {
                "edge_id": edge.getID(),
                # "speed": lane.getSpeed(),
                # "length": lane.getLength(),
                # "shape": shape,
                "successors": successors
            }

    # Build reverse links (predecessors)
    for lane_id in lane_data:
        lane_data[lane_id]["predecessors"] = []

    for lane_id, info in lane_data.items():
        for succ in info["successors"]:
            if succ in lane_data:
                lane_data[succ]["predecessors"].append(lane_id)
    return lane_data

def find_upstream_lanes(lane_data, start_lane_id, max_depth=3):
    visited = set()
    queue = deque([(start_lane_id, 0)])
    upstream_lanes = set()

    while queue:
        current_lane, depth = queue.popleft()
        print(current_lane)
        if depth > max_depth or current_lane in visited:
            continue
        visited.add(current_lane)
        upstream_lanes.add(current_lane)
        predecessors = lane_data.get(current_lane, {}).get("predecessors", [])
        print(predecessors)
        for pred in predecessors:
            queue.append((pred, depth + 1))
    return upstream_lanes


def get_upstream_veh_data_for_one_veh(row_data_ego, data_all_t, upsteam_vehcles_max_cnt=5):
    ego_x, ego_y = row_data_ego['x'], row_data_ego['y']
    up_stream_lanes = row_data_ego['up_stream_lanes']
    data_all_t = data_all_t[data_all_t['id'] != 'ego']
    candidates = data_all_t[data_all_t['lane'].isin(up_stream_lanes)].copy()

    if candidates.empty:
        return pd.DataFrame(columns=['speed', 'acceleration', 'distance'])

    candidates['distance'] = np.sqrt((candidates['x'] - ego_x) ** 2 + (candidates['y'] - ego_y) ** 2)
    candidates_sorted = candidates.sort_values(by='distance').head(upsteam_vehcles_max_cnt)

    return candidates_sorted[['id', 'speed', 'acceleration', 'distance']]


def get_upstream_veh_data(input_data_dic, upsteam_vehcles_max_cnt=5, source_veh_id='ego', step_length=0.1):
    all_results = {}

    for penetration_rate, data_dic in input_data_dic.items():
        relative_time = 28800
        refer_coord = [160, 735]
        path = data_dic['path']

        # Load and preprocess
        lane_geometry = get_lane_geometry(os.path.join(path, 'chatt.net.xml'))
        data_all = read_trajectory_xml(os.path.join(path, 'fcd.xml'), refer_coord=refer_coord)
        data_all['speed'] = data_all['speed'].astype(float)
        data_all['acceleration'] = (data_all['speed'].shift(-1) - data_all['speed']) / step_length

        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']
        data_all = trajectory_process(data_all, eb_lanes, wb_lanes, relative_time)

        data_wb = data_all[data_all['direction'] == 'WB'].reset_index(drop=True)
        data_wb_ego = data_wb[data_wb['id'] == source_veh_id].copy()

        if data_wb_ego.empty:
            all_results[penetration_rate] = pd.DataFrame()
            continue

        # Compute upstream lanes once for each ego lane
        lane_to_upstream = {}
        for lane in data_wb_ego['lane'].unique():
            lane_to_upstream[lane] = find_upstream_lanes(lane_geometry, lane)

        data_wb_ego['up_stream_lanes'] = data_wb_ego['lane'].map(lane_to_upstream)

        # Compute upstream vehicle data per row
        upstream_results = []
        for _, row in data_wb_ego.iterrows():
            t = row['time']
            data_all_t = data_all[data_all['time'] == t]

            upstream_veh_df = get_upstream_veh_data_for_one_veh(row, data_all_t, upsteam_vehcles_max_cnt)
            speeds = upstream_veh_df['speed'].tolist()
            accs = upstream_veh_df['acceleration'].tolist()

            # Pad if needed
            while len(speeds) < upsteam_vehcles_max_cnt:
                speeds.append(np.nan)
                accs.append(np.nan)

            result_row = {
                f"veh{i+1}_speed": speeds[i] for i in range(upsteam_vehcles_max_cnt)
            }
            result_row.update({
                f"veh{i+1}_acc": accs[i] for i in range(upsteam_vehcles_max_cnt)
            })
            result_row['time'] = t
            upstream_results.append(result_row)

        all_results[penetration_rate] = pd.DataFrame(upstream_results)

    return all_results


def compare_distributions(dist1, dist2):
    """
    Compare two 1D non-normal distributions using robust statistical metrics.
    
    Parameters:
    - dist1, dist2: array-like samples
    
    Returns:
    - Dictionary of comparison results
    """
    dist1 = np.asarray(dist1)
    dist2 = np.asarray(dist2)
    
    ks_result = ks_2samp(dist1, dist2)
    mannwhitney_result = mannwhitneyu(dist1, dist2, alternative='two-sided')

    return {
        "mean_diff": np.mean(dist1) - np.mean(dist2),
        "median_diff": np.median(dist1) - np.median(dist2),
        "std_diff": np.std(dist1) - np.std(dist2),
        "ks_stat": ks_result.statistic,
        "ks_p_value": ks_result.pvalue,
        "wasserstein_distance": wasserstein_distance(dist1, dist2),
        "mannwhitney_p": mannwhitney_result.pvalue
    }