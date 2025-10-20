import ast
import os
import pandas as pd
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import seaborn as sns
import plotly.graph_objects as go
import plotly.io as pio


# Constants used by the interactive plot
LANE_WIDTH_METERS = 3.6  # approximate width per lane for lateral distance estimation
SIGNAL_STATE_COLOR = {
    'G': '#2ecc71',
    'g': '#2ecc71',
    'Y': '#f1c40f',
    'y': '#f1c40f',
    'R': '#e74c3c',
    'r': '#e74c3c'
}
SIGNAL_STATE_LABEL = {
    'G': 'Green',
    'g': 'Green',
    'Y': 'Yellow',
    'y': 'Yellow',
    'R': 'Red',
    'r': 'Red'
}


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

    # trajectory_data = get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True)

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


def _extract_lane_index(lane_value):
    """
    Extracts the numeric lane index from a SUMO lane string like '-2801_2'.
    Returns NaN when parsing is not possible.
    """
    if pd.isna(lane_value):
        return np.nan
    lane_text = str(lane_value)
    if '_' not in lane_text:
        return np.nan
    suffix = lane_text.rsplit('_', 1)[-1]
    try:
        return float(suffix)
    except ValueError:
        return np.nan


def _build_signal_segments(signal_df, signal_config, direction, start_time, end_time):
    """
    Builds contiguous time segments for each signal state within the requested time window.
    """
    segments = []
    config_subset = signal_config[signal_config['approach_direction'] == direction]

    for _, cfg_row in config_subset.iterrows():
        signal_id = str(cfg_row['id'])
        movement_indexes = cfg_row['movement_index']
        if isinstance(movement_indexes, str):
            try:
                movement_indexes = ast.literal_eval(movement_indexes)
            except (ValueError, SyntaxError):
                movement_indexes = []
        if not movement_indexes:
            continue

        target_index = movement_indexes[-1]
        signal_distance = float(cfg_row['distance'])
        intersection_name = cfg_row.get('int_name', signal_id)

        focused_signal = signal_df[signal_df['id'] == signal_id].copy()
        if focused_signal.empty:
            continue

        focused_signal['time'] = focused_signal['time'].astype(float)
        focused_signal = focused_signal.sort_values('time')
        focused_signal['state_char'] = focused_signal['state'].str[target_index:target_index + 1]
        focused_signal = focused_signal[focused_signal['state_char'].isin(SIGNAL_STATE_COLOR.keys())]

        if focused_signal.empty:
            continue

        times = focused_signal['time'].tolist()
        states = focused_signal['state_char'].tolist()

        current_start = times[0]
        current_state = states[0]

        for next_time, next_state in zip(times[1:], states[1:]):
            segments.append({
                'signal_id': signal_id,
                'intersection': intersection_name,
                'state': current_state,
                'start_time': current_start,
                'end_time': next_time,
                'distance': signal_distance
            })
            current_start = next_time
            current_state = next_state

        segments.append({
            'signal_id': signal_id,
            'intersection': intersection_name,
            'state': current_state,
            'start_time': current_start,
            'end_time': end_time,
            'distance': signal_distance
        })

    # Clip segments to requested window to avoid drawing outside the visible range
    clipped_segments = []
    for seg in segments:
        seg_start = max(seg['start_time'], start_time)
        seg_end = min(seg['end_time'], end_time)
        if seg_end <= seg_start:
            continue
        clipped_segments.append({
            **seg,
            'start_time': seg_start,
            'end_time': seg_end
        })

    return clipped_segments


def plot_space_time_matplotlib(trajectory_data, sumo_signal, sumoSignalConfig, direction='WB',
                               start_time=300.0, end_time=2700.0, max_vehicle_lines=None,
                               highlight_vehicle=None, figsize=(24, 10), dpi=100,
                               signal_linewidth=8.0, trajectory_width=1.2):
    """
    Create a Matplotlib space-time diagram that overlays vehicle trajectories with signal states.
    """
    # Filter trajectories for the selected corridor and time window
    filtered = trajectory_data[trajectory_data['direction'] == direction].copy()
    if filtered.empty:
        raise ValueError(f"No trajectory data found for direction '{direction}'.")

    filtered['time'] = filtered['time'].astype(float)
    filtered['distance'] = filtered['distance'].astype(float)
    filtered = filtered[(filtered['time'] >= start_time) & (filtered['time'] <= end_time)]

    if filtered.empty:
        raise ValueError(f"No trajectory data within {start_time}-{end_time} s for '{direction}'.")

    filtered = filtered.sort_values(['id', 'time'])

    # Limit the number of vehicles if requested (keep deterministic order)
    if max_vehicle_lines is not None and max_vehicle_lines > 0:
        unique_ids = filtered['id'].unique()
        keep_ids = set(unique_ids[:max_vehicle_lines])
        if highlight_vehicle is not None:
            keep_ids.add(str(highlight_vehicle))
        filtered = filtered[filtered['id'].isin(keep_ids)]

    fig, ax = plt.subplots(figsize=figsize, dpi=dpi)

    type_color_map = {'HDV': '#95a5a6', 'CAV': '#1f77b4'}
    legend_handles = {}

    # Plot all trajectories
    for veh_id, veh_df in filtered.groupby('id'):
        veh_type = veh_df['type'].iloc[0] if 'type' in veh_df else None
        color = type_color_map.get(veh_type, '#34495e')
        label = f"{veh_type or 'Vehicle'}" if veh_type and veh_type not in legend_handles else None
        line, = ax.plot(
            veh_df['time'],
            veh_df['distance'],
            color=color,
            linewidth=trajectory_width,
            alpha=0.8,
            label=label
        )
        if label and veh_type not in legend_handles:
            legend_handles[veh_type] = line

    # Emphasise highlighted vehicle if provided
    if highlight_vehicle is not None:
        highlight_df = trajectory_data[trajectory_data['id'].astype(str) == str(highlight_vehicle)].copy()
        highlight_df = highlight_df[highlight_df['direction'] == direction]
        highlight_df['time'] = highlight_df['time'].astype(float)
        highlight_df['distance'] = highlight_df['distance'].astype(float)
        highlight_df = highlight_df[(highlight_df['time'] >= start_time) & (highlight_df['time'] <= end_time)]
        if not highlight_df.empty:
            ax.plot(
                highlight_df['time'],
                highlight_df['distance'],
                color='#27ae60',
                linewidth=3.0,
                label=f"Vehicle {highlight_vehicle}"
            )

    # Overlay signal states
    signal_segments = _build_signal_segments(sumo_signal, sumoSignalConfig, direction, start_time, end_time)
    signal_labels_added = set()

    for seg in signal_segments:
        state = seg['state']
        color = SIGNAL_STATE_COLOR.get(state, '#7f8c8d')
        label = SIGNAL_STATE_LABEL.get(state, state)
        show_label = label not in signal_labels_added
        ax.plot(
            [seg['start_time'], seg['end_time']],
            [seg['distance'], seg['distance']],
            color=color,
            linewidth=signal_linewidth,
            solid_capstyle='butt',
            label=f"{label} Signal" if show_label else None
        )
        if show_label:
            signal_labels_added.add(label)

    # Axes formatting
    ax.set_title(f"{direction} Space-Time Diagram")
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Distance Along Corridor (m)")
    ax.set_xlim(start_time, end_time)
    ax.invert_yaxis()
    ax.grid(True, linestyle=':', linewidth=0.6, alpha=0.6)

    # Build combined legend with unique entries
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc='upper right', fontsize=12, frameon=True)

    return fig, ax


def plot_space_time_plotly(trajectory_data, sumo_signal, sumoSignalConfig, direction='WB',
                           start_time=300.0, end_time=3300.0, output_html=None):
    """
    Creates an interactive space-time diagram with Plotly that overlays vehicle trajectories
    and signal phases for the selected corridor direction.
    """
    filtered_traj = trajectory_data[trajectory_data['direction'] == direction].copy()
    if filtered_traj.empty:
        raise ValueError(f"No trajectory data found for direction '{direction}'.")

    filtered_traj['time'] = filtered_traj['time'].astype(float)
    filtered_traj['distance'] = filtered_traj['distance'].astype(float)
    filtered_traj = filtered_traj[(filtered_traj['time'] >= start_time) & (filtered_traj['time'] <= end_time)]

    if filtered_traj.empty:
        raise ValueError(f"No trajectory data within the time window {start_time} - {end_time} for '{direction}'.")

    filtered_traj['speed'] = pd.to_numeric(filtered_traj['speed'], errors='coerce')
    filtered_traj['lane_index'] = filtered_traj['lane'].apply(_extract_lane_index)
    filtered_traj['lateral_distance'] = filtered_traj['lane_index'] * LANE_WIDTH_METERS

    fig = go.Figure()
    type_color_map = {'HDV': '#7f8c8d', 'CAV': '#1f77b4'}

    for vehicle_type, type_df in filtered_traj.groupby('type'):
        color = type_color_map.get(vehicle_type, '#34495e')
        show_legend = True
        for veh_id, veh_df in type_df.groupby('id'):
            veh_df = veh_df.sort_values('time')
            customdata = np.column_stack([
                veh_df['id'].astype(str).to_numpy(),
                veh_df['lane'].astype(str).to_numpy(),
                veh_df['lateral_distance'].to_numpy(),
                veh_df['speed'].to_numpy()
            ])
            fig.add_trace(go.Scatter(
                x=veh_df['time'],
                y=veh_df['distance'],
                mode='lines',
                name=f"{vehicle_type} Trajectory" if show_legend else None,
                legendgroup=f"vehicle-{vehicle_type}",
                line=dict(color=color, width=1.5),
                hovertemplate=(
                    "Time: %{x:.1f}s<br>"
                    "Distance: %{y:.1f} m<br>"
                    "Vehicle: %{customdata[0]}<br>"
                    "Lane: %{customdata[1]}<br>"
                    "Lateral Distance: %{customdata[2]:.1f} m<br>"
                    "Speed: %{customdata[3]:.1f} m/s"
                ),
                customdata=customdata,
                showlegend=show_legend
            ))
            show_legend = False

    signal_segments = _build_signal_segments(sumo_signal, sumoSignalConfig, direction, start_time, end_time)
    legend_shown = set()

    for seg in signal_segments:
        state = seg['state']
        color = SIGNAL_STATE_COLOR.get(state, '#95a5a6')
        label = SIGNAL_STATE_LABEL.get(state, state)
        legend_key = f"signal-{label}"
        fig.add_trace(go.Scatter(
            x=[seg['start_time'], seg['end_time']],
            y=[seg['distance'], seg['distance']],
            mode='lines',
            line=dict(color=color, width=8),
            name=f"{label} Signal" if legend_key not in legend_shown else None,
            legendgroup=legend_key,
            hovertemplate=(
                f"Signal: {seg['intersection']}<br>"
                f"State: {label}<br>"
                "Time: %{x:.1f}s<br>"
                "Distance: %{y:.1f} m"
            ),
            showlegend=legend_key not in legend_shown
        ))
        legend_shown.add(legend_key)

    fig.update_layout(
        title=f"{direction} Space-Time Diagram",
        xaxis_title="Simulation Time (s)",
        yaxis_title="Distance Along Corridor (m)",
        template="plotly_white",
        yaxis=dict(autorange="reversed"),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )

    if output_html:
        pio.write_html(fig, file=output_html, auto_open=False)

    fig.show()


def plot_wb_veh(wfzp_traj, file_folder, scenario_name, input_link='-2801', direction='WB'):
    wfzp_traj = wfzp_traj[wfzp_traj['direction'] == direction].reset_index(drop=True)

    # plot the westbound vehicle input over time
    test = wfzp_traj.groupby('id', as_index=False).first()
    test = test[test['segment'] == input_link]
    # test = test[(test['time'] >= 300) & (test['time'] <= 3300)].reset_index(drop=True)

    plt.figure(figsize=(24, 10))
    ax = sns.histplot(test['time'], bins=4, kde=True)

    # Add values on top of bars
    for p in ax.patches:
        ax.annotate(f'{int(p.get_height())}',
                    (p.get_x() + p.get_width() / 2, p.get_height()),
                    ha='center', va='bottom', fontsize=18, fontweight='bold')

    plt.xlabel('Timestamp')
    plt.ylabel('Number of Vehicles')
    plt.title('Histogram of Vehicle Inputs')
    # plt.savefig('veh_input_distribution.png', dpi=100, bbox_inches='tight')
    plt.savefig(os.path.join(file_folder, 'veh_input_distribution_{}_{}.png'.format(scenario_name, direction)), dpi=100, bbox_inches='tight')


def plot_std(trajectory_data, sumo_signal, sumoSignalConfig, direction='WB'):
    fig, _ = plot_space_time_matplotlib(
        trajectory_data,
        sumo_signal,
        sumoSignalConfig,
        direction=direction,
        start_time=300.0,
        end_time=2700.0,
        max_vehicle_lines=None,
        highlight_vehicle=None
    )
    fig.savefig(r'{}\space_time_diagram_{}'.format(file_folder, direction), dpi=100, bbox_inches='tight')

    # plot iterative plot
    # plot_trajectory_plotly(trajectory_data_wb, trajectory_data, 900, highlight_veh_number=None, output_html=r'{}\trajectory_plot_{}.html'.format(file_folder, direction))


if __name__ == "__main__":
    # define a reference point coordinates at the leftmost point to calculate the distance over the corridor
    # refer_coord = [200, 709]
    refer_coord = [160, 735]
    relative_time = 28800

    sumoFolder = 'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3'
    # sumoFolder = 'Shallowford_before_calibration_V3'

    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv(os.path.join(sumoFolder, 'sumoSignalConfig_26.csv'), index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)
    sumoSignalConfig['int_name'] = sumoSignalConfig['id'].map(
        {'12': 'Amin', '9': 'I-75 S', '8': 'I-75 N', '10': 'Napier', '3': 'Lifestyle', '2': 'Gunbarrel'})

    # Calculate the relative distance for each point (unit: meter)
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
        # file_folder = r'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\100%_Seed100_Subscription_1Hz'
        file_folder = r'C:\Users\hg25079\Documents\GitHub\FIXS\tests\Applications\Ego_Settings_Selection\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\0%_Seed100_Subscription_10Hz'

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
        # plot_std(trajectory_data, sumo_signal, sumoSignalConfig, 'WB')
        # plot_std(trajectory_data, sumo_signal, sumoSignalConfig, 'EB')

        interactive_start = 300.0
        interactive_end = 2700.0
        plot_space_time_plotly(
            trajectory_data,
            sumo_signal,
            sumoSignalConfig,
            direction='WB',
            start_time=interactive_start,
            end_time=interactive_end,
            output_html=os.path.join(file_folder, 'space_time_plot_WB.html')
        )
        # plot_space_time_plotly(
        #     trajectory_data,
        #     sumo_signal,
        #     sumoSignalConfig,
        #     direction='EB',
        #     start_time=interactive_start,
        #     end_time=interactive_end,
        #     output_html=os.path.join(file_folder, 'space_time_plot_EB.html')
        # )

        # # plot wb vehicle input distribution
        # plot_wb_veh(trajectory_data, file_folder, '0%', '-2801', 'WB')
        # plot_wb_veh(trajectory_data, file_folder, '0%', '-312', 'EB')

        # print('test')
