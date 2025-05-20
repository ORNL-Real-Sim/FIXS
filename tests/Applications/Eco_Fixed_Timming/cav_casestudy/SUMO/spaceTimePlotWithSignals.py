import pandas as pd
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
import os
import plotly.graph_objects as go
import plotly.io as pio


##############################################################
# Parse the signal changes (SaveTLSSwitchStates) XML file
##############################################################
def read_signal_xml(signal_file):
    tree = ET.parse(signal_file)
    root = tree.getroot()

    # Extract data from XML into a list of dictionaries
    data_list = []

    for tls in root.findall('tlsState'):
        data_list.append(tls.attrib)

    # Convert the list of dictionaries to a dataframe
    sumoSignalResult = pd.DataFrame(data_list)

    return sumoSignalResult


def read_route_xml(route_file):
    tree = ET.parse(route_file)
    root = tree.getroot()

    # Extract data from XML into a list of dictionaries
    data_list = []
    vehicle_routes = {}

    for vehicle  in root.findall('vehicle'):
        vehicle_id = vehicle.get('id')
        route_elem = vehicle.find('route')
        if route_elem is not None:
            edges = route_elem.get('edges')
            vehicle_routes[vehicle_id] = edges.split()

    # Turn the dictionary into a list of rows
    rows = [{'vehicle_id': vid, 'edges': edges} for vid, edges in vehicle_routes.items()]
    sumo_input_routes = pd.DataFrame(rows)

    return sumo_input_routes


# plot signal changes
def plot_signal_changes_xml(sumosignalresult, sumoSignalConfig, plot_start_time, duration_limit, direction='WB'):
    print(direction)
    sumoSignalConfig = sumoSignalConfig[sumoSignalConfig['approach_direction'] == direction].reset_index(drop=True)

    for sc in sumoSignalConfig.id.unique():
        print(sc)
        phase_num = sumoSignalConfig[sumoSignalConfig['id'] == sc]['name'].values[0]
        movement_indexes = eval(sumoSignalConfig[sumoSignalConfig['id'] == sc]['movement_index'].values[0])
        x_loc = sumoSignalConfig[sumoSignalConfig['id'] == sc]['distance'].values[0]

        signal_changes_tem = sumosignalresult[sumosignalresult['id'] == sc].reset_index(drop=True)
        signal_changes_tem['target_phase_state'] = signal_changes_tem['state'].str[movement_indexes[-1]: movement_indexes[-1] + 1]

        # only keep phase changes for the targeted phase
        signal_changes_tem = signal_changes_tem[signal_changes_tem['target_phase_state'] != signal_changes_tem['target_phase_state'].shift()]
        signal_changes_tem = signal_changes_tem[signal_changes_tem['target_phase_state'].isin(['G', 'y', 'r'])]
        signal_changes_tem['target_phase_state'] = signal_changes_tem['target_phase_state'].map({'G': 'green', 'y': 'yellow', 'r': 'red'})

        cnt = 1
        prev_idx = 0
        prev_clr = signal_changes_tem.loc[prev_idx,'target_phase_state']
        for index, row in signal_changes_tem.iterrows():
            if index >= 1:
                print(index)
                y1 = [x_loc, x_loc]
                x1 = [float(signal_changes_tem.loc[prev_idx,'time']), float(row.time)]
                plt.plot(x1, y1, color=prev_clr, linewidth=7, solid_capstyle='butt')
                c = row.target_phase_state

                prev_idx = index
                prev_clr = c
                cnt += 1
            else:
                continue

    plt.title('Shallowford Rd Space-Time Diagram')
    plt.xlabel('Simulation Second (s)', fontsize=24)
    plt.ylabel('Distance along '+ direction +' Direction', fontsize=24)
    plt.xlim(plot_start_time, plot_start_time + duration_limit)
    plt.yticks(sumoSignalConfig['distance'].values, sumoSignalConfig['int_name'].values, fontsize=18)
    plt.gca().invert_yaxis()


##############################################################
# parse the vehicle trajectory data
##############################################################
def read_trajectory_xml(trajectory_file, refer_coord):
    tree = ET.parse(trajectory_file)
    root = tree.getroot()

    # Extract data from XML into a list of dictionaries
    data_list = []

    for ts in root.findall('timestep'):
        for traj in ts.findall('vehicle'):
            # print(traj.attrib)
            tem_dict = traj.attrib
            tem_dict.update(ts.attrib)
            data_list.append(tem_dict)

    trajectory = pd.DataFrame(data_list)
    # Calculate the relative distance for each point
    trajectory['distance'] = np.sqrt((trajectory['x'].astype(float) - refer_coord[0]) ** 2 + (trajectory['y'].astype(float) - refer_coord[1]) ** 2)

    return trajectory


def remove_none_edges(df, group_col, value_col):
    """
    Removes all leading and trailing rows with None values in a specific column for each group.

    Args:
    - df (pd.DataFrame): Input dataframe.
    - group_col (str): Column name to group by.
    - value_col (str): Column name to check for None values.

    Returns:
    - pd.DataFrame: DataFrame with leading and trailing None values removed for each group.
    """
    def process_group(group):
        # Find the first non-None index
        first_valid_index = group[value_col].first_valid_index()
        # Find the last non-None index
        last_valid_index = group[value_col].last_valid_index()

        # Slice the group to exclude leading and trailing None rows
        return group.loc[first_valid_index:last_valid_index]

    # Apply the function to each group and recombine
    return df.groupby(group_col, group_keys=False).apply(process_group)


# Function to keep items from the last index containing ":"
def keep_after_last_index_with_colon(lst):
    # Find all indexes that contain ":"
    int_list = [":12", ":9", ":8", ":10", ":3", ":2"]
    if lst[-1] in ['-304', '-295']:
        return lst
    else:
        indexes = [i for i, item in enumerate(lst) if isinstance(item, str) and item in int_list]
        # Return the sliced list from the last matched index onward
        if len(indexes) >= 2:
            return lst[:indexes[-2] + 1]
        else:
            return None


def get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True):
    veh_traj_agg = trajectory_data.groupby('id')['segment'].agg(lambda x: list(dict.fromkeys(x))).reset_index()
    veh_traj_agg['updated_segment_lists'] = veh_traj_agg['segment'].apply(keep_after_last_index_with_colon)

    trajectory_data = trajectory_data.join(veh_traj_agg[['id', 'updated_segment_lists']].set_index('id'), on='id')

    if exclude_turning_traj == True:
        trajectory_data = trajectory_data[trajectory_data.apply(lambda row: isinstance(row['updated_segment_lists'], list) and row['segment'] in row['updated_segment_lists'], axis=1)]

    return trajectory_data


def trajectory_process(trajectory_data, eb_lanes, wb_lanes, relative_time):
    trajectory_data['segment'] = trajectory_data['lane'].str.split('_').str[0]
    trajectory_data['direction'] = np.where(trajectory_data['segment'].isin(wb_lanes), "WB",
                                            np.where(trajectory_data['segment'].isin(eb_lanes), "EB", None))
    trajectory_data['time'] = trajectory_data['time'].astype(float)
    trajectory_data = trajectory_data.sort_values(by=['id', 'time'], ignore_index=True)

    trajectory_data = trajectory_data[(trajectory_data['direction'].notnull())
                                      | (trajectory_data['segment'].str.contains(':'))]

    # remove the first few None values and last None values, which means those are on the start and end connectors.
    trajectory_data = remove_none_edges(trajectory_data, group_col="id", value_col="direction")

    # Forward fill and backward fill within each group
    trajectory_data["direction"] = trajectory_data.groupby("id")["direction"].ffill()

    # modify the timestamp to be consistent with VISSIM
    trajectory_data['time'] = trajectory_data['time'].astype(float) - relative_time

    trajectory_data = trajectory_data[trajectory_data['direction'].isin(['EB', 'WB'])]

    trajectory_data = get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True)

    return trajectory_data


def plot_trajectory(trajectory_data, trajectory_data_full, num_veh_limit, highlight_veh_number='None'):
    line_width = 1
    # trajectory_data = get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True)

    trajectory_data['time'] = trajectory_data['time'].astype(float)
    trajectory_data = trajectory_data[(trajectory_data['time'] >= 300) & (trajectory_data['time'] <= 3300)].reset_index(drop=True)
    trajectory_data['distance'] = trajectory_data['distance'].astype(float)
    trajectory_data = trajectory_data[trajectory_data['id'] != highlight_veh_number]

    if num_veh_limit > len(trajectory_data.id.unique()):
        num_veh_limit = len(trajectory_data.id.unique())
    else:
        pass

    # for i, veh in enumerate(trajectory_data.id.unique()[0:num_veh_limit]):
    #     print(len(trajectory_data.id.unique()), i)
    #     dfs = trajectory_data[trajectory_data['id'] == veh].reset_index(drop=True)
    #     # if the O and D are all on the major road, then plot.
    #     # if dfs['Link'].iloc[0] in linkinfo['Link'].unique().tolist() and dfs['Link'].iloc[-1] in linkinfo['Link'].unique().tolist():
    #     clr = 'grey'
    #     plt.plot(dfs['time'].astype(float), dfs['distance'].astype(float), color=clr, linewidth=line_width, solid_capstyle='butt')

    sns.lineplot(data=trajectory_data[trajectory_data['type'] == 'HDV'], x='time', y='distance', hue='id', legend=False, palette=['lightgrey'])
    sns.lineplot(data=trajectory_data[trajectory_data['type'] == 'CAV'], x='time', y='distance', hue='id', legend=False, palette=['blue'])

    print('Plot Trajectories: ', len(trajectory_data.id.unique()))

    # highlight a specific vehicle
    if highlight_veh_number != 'None':
        dfs = trajectory_data_full[trajectory_data_full['id'] == highlight_veh_number]
        clr = 'lightgreen'
        line_width = 4
        plt.plot(dfs['time'].astype(float), dfs['distance'].astype(float), color=clr, linewidth=line_width, solid_capstyle='butt')


def plot_trajectory_plotly(trajectory_data, trajectory_data_full, num_veh_limit, highlight_veh_number=None, output_html="trajectory_plot.html"):
    # trajectory_data = get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True)
    # Ensure 'time' and 'distance' columns are floats
    trajectory_data['time'] = trajectory_data['time'].astype(float)
    # trajectory_data = trajectory_data[(trajectory_data['time'] >= 300) & (trajectory_data['time'] <= 3300)].reset_index(drop=True)
    trajectory_data['distance'] = trajectory_data['distance'].astype(float)

    # Remove the highlighted vehicle from the main plot (if applicable)
    if highlight_veh_number is not None:
        trajectory_data = trajectory_data[trajectory_data['id'] != highlight_veh_number]

    # Limit the number of unique vehicles plotted
    # unique_ids = trajectory_data['id'].unique()
    # if num_veh_limit > len(unique_ids):
    #     num_veh_limit = len(unique_ids)
    # limited_ids = unique_ids[:num_veh_limit]
    # trajectory_data = trajectory_data[trajectory_data['id'].isin(limited_ids)]

    # Create an interactive figure
    fig = go.Figure()

    # Add HDV trajectories (light gray lines)
    hdv_data = trajectory_data[trajectory_data['type'] == 'HDV']
    for veh_id in hdv_data['id'].unique():
        veh_data = hdv_data[hdv_data['id'] == veh_id]
        fig.add_trace(go.Scatter(
            x=veh_data['time'],
            y=veh_data['distance'],
            mode='lines',
            name=f"HDV {veh_id}",
            line=dict(color='lightgrey', width=1),
            legendgroup='HDV',
            hoverinfo='x+y+name'
        ))

    # Add CAV trajectories (blue lines)
    cav_data = trajectory_data[trajectory_data['type'] == 'CAV']
    for veh_id in cav_data['id'].unique():
        veh_data = cav_data[cav_data['id'] == veh_id]
        fig.add_trace(go.Scatter(
            x=veh_data['time'],
            y=veh_data['distance'],
            mode='lines',
            name=f"CAV {veh_id}",
            line=dict(color='blue', width=1),
            legendgroup='CAV',
            hoverinfo='x+y+name'
        ))

    # Highlight the specific vehicle
    if highlight_veh_number is not None:
        dfs = trajectory_data_full[trajectory_data_full['id'] == highlight_veh_number]
        if not dfs.empty:
            fig.add_trace(go.Scatter(
                x=dfs['time'],
                y=dfs['distance'],
                mode='lines',
                name=f"Highlighted Vehicle {highlight_veh_number}",
                line=dict(color='lightgreen', width=4),
                hoverinfo='x+y+name'
            ))
        else:
            print(f"Warning: Vehicle ID '{highlight_veh_number}' not found in trajectory_data_full.")

    # Update layout for better visuals
    fig.update_layout(
        title="Interactive Vehicle Trajectories",
        xaxis_title="Time",
        yaxis_title="Distance",
        legend_title="Vehicle Types",
        template="plotly_white",
        showlegend=True,
        yaxis=dict(autorange="reversed")
    )

    # Save the plot to an HTML file
    pio.write_html(fig, file=output_html, auto_open=False)

    # Show the interactive plot
    fig.show()


def plot_wb_veh(wfzp_traj, file_folder, scenario_name):
    wfzp_traj = wfzp_traj[wfzp_traj['direction'] == 'WB'].reset_index(drop=True)

    # plot the westbound vehicle input over time
    test = wfzp_traj.groupby('id', as_index=False).first()
    test = test[test['segment'] == '-2801']
    # test = test[(test['time'] >= 300) & (test['time'] <= 3300)].reset_index(drop=True)

    plt.figure(figsize=(24, 10))
    ax = sns.histplot(test['time'], bins=30, kde=True)

    # Add values on top of bars
    for p in ax.patches:
        ax.annotate(f'{int(p.get_height())}',
                    (p.get_x() + p.get_width() / 2, p.get_height()),
                    ha='center', va='bottom', fontsize=18, fontweight='bold')

    plt.xlabel('Timestamp')
    plt.ylabel('Number of Vehicles')
    plt.title('Histogram of Vehicle Inputs')
    # plt.savefig('veh_input_distribution.png', dpi=100, bbox_inches='tight')
    plt.savefig(os.path.join(file_folder, 'veh_input_distribution_{}.png'.format(scenario_name)), dpi=100, bbox_inches='tight')


def plot_std(trajectory_data, sumo_signal, sumoSignalConfig, direction='WB'):
    trajectory_data_wb = trajectory_data[trajectory_data['direction'] == direction].reset_index(drop=True)

    # ['10.21', '27.2','14.15', '14.16', '14.17', '4.103', '4.110', '4.128']
    # plot
    fig = plt.figure()
    fig.set_size_inches(24, 10)
    plt.rcParams.update({'font.size': 24})

    # plot_signal_changes_xml(sumo_signal, sumoSignalConfig, 28985, 1200, direction='WB')
    plot_signal_changes_xml(sumo_signal, sumoSignalConfig, 300, 2400, direction=direction)

    # plot_trajectory(trajectory_data_wb, trajectory_data, 300, 'ego')
    # without highlighting the ego vehicle
    plot_trajectory(trajectory_data_wb, trajectory_data, 300, highlight_veh_number='ego')
    # plot_trajectory(trajectory_data_wb, trajectory_data, 300)

    # plt.show()
    plt.savefig(r'{}\space_time_diagram_{}'.format(file_folder, direction), dpi=100, bbox_inches='tight')

    # plot iterative plot
    plot_trajectory_plotly(trajectory_data_wb, trajectory_data, 900, highlight_veh_number=None, output_html=r'{}\trajectory_plot_{}.html'.format(file_folder, direction))


if __name__ == "__main__":
    # define a reference point coordinates at the leftmost point to calculate the distance over the corridor
    refer_coord = [200, 709]
    relative_time = 28800

    sumoFolder = 'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3'

    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv(os.path.join(sumoFolder, 'sumoSignalConfig_26.csv'), index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)
    sumoSignalConfig['int_name'] = sumoSignalConfig['id'].map(
        {'12': 'Amin', '9': 'I-75 S', '8': 'I-75 N', '10': 'Napier', '3': 'Lifestyle', '2': 'Gunbarrel'})

    # Calculate the relative distance for each point
    sumoSignalConfig['distance'] = np.sqrt((sumoSignalConfig['x'] - refer_coord[0]) ** 2 + (sumoSignalConfig['y'] - refer_coord[1]) ** 2)

    # file_folder = 'MPR50%WithoutDynamics_10Hz_LoopRoute_FixedTime'
    # file_folder = 'EcoDrivingWithoutDynamicsMaxRecall_10Hz_LoopRoute_FixedTime'

    # specify the folder that has the fcd.xml data and signal_result.xml.
    # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/100%_Subscription_1Hz'
    # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/0%_Subscription_1Hz'
    # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/10%_Subscription_1Hz'
    # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime/MPR/20%_Subscription_1Hz'
    # file_folder = r'SUMO_VISSIM_behavior_test'
    # file_folder = r'Shallowford_after_calibration_banleftturn_actuated'
    route_folder = os.path.join(sumoFolder, 'MPR')
    # route_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V2/MPR/0%_Seed100_Subscription_10Hz'

    # read input route file
    # sumo_input_routes = read_route_xml(r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V2/chattCavMpr.rou.xml')
    # route process to remove EB and WB left turn and right turn edges

    folder_list = [f for f in os.listdir(route_folder) if os.path.isdir(os.path.join(route_folder, f))]

    for folder in folder_list:
        file_folder = os.path.join(route_folder, folder)

        # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V2/MPR/100%_Seed100_Subscription_1Hz'
        # file_folder = r'RL_Parth/RL'
        # file_folder = r'RL_Parth/RL_V2'
        # file_folder = r'Shallowford_before_calibration_V3'

        # file_folder = r'Shallowford_after_calibration_banleftturn_actuated'
        trajectory_data = read_trajectory_xml(os.path.join(file_folder, 'fcd.xml'), refer_coord)
        sumo_signal = read_signal_xml(os.path.join(file_folder, 'signal_result.xml'))
        # signal file
        sumo_signal['time'] = sumo_signal['time'].astype(float) - relative_time

        # only keep the WB direction trajectories
        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']

        trajectory_data = trajectory_process(trajectory_data, eb_lanes, wb_lanes, relative_time)

        # plot space-time diagram
        plot_std(trajectory_data, sumo_signal, sumoSignalConfig, 'WB')
        plot_std(trajectory_data, sumo_signal, sumoSignalConfig, 'EB')

        # plot wb vehicle input distribution
        # plot_wb_veh(trajectory_data, file_folder, '0%')

        print('test')