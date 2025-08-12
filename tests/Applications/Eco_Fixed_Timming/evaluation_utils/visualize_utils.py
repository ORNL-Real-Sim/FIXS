import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib as mpl
import seaborn as sns
import pandas as pd
import os
import numpy as np
from evaluation_utils.metadata_utils import compare_distributions
from cav_casestudy.SUMO.VehicleEnergyEvaluation import get_traj_eval
from evaluation_utils.metadata_utils import get_energy_consumption, assign_trip_id
from cav_casestudy.SUMO.spaceTimePlotWithSignals import  read_trajectory_xml, read_signal_xml, trajectory_process, get_exclude_turn_traj
@DeprecationWarning
def plot_system_eval(data_dic_with_dynamics_before_calibration,
                     data_dic_with_dynamics_after_calibration,
                     vtMicroCoeff: pd.DataFrame,
                     df_vehicle_src_coeff: pd.DataFrame,
                     ):

    system_mpg_cpmf_dict = {}

    def get_direction_eval(penetration_rate, data_dic, vtMicroCoeff, df_vehicle_src_coeff, direction):
        trajectory_data = data_dic[penetration_rate][f'trajectory_data_{direction}']
        _, fuel_list, _, _, _, _, dist_list, _ = get_traj_eval(
            trajectory_data, 1, vtMicroCoeff, df_vehicle_src_coeff, veh_no='system')

        mpg_list = []
        for f, d in zip(fuel_list, dist_list):
            dist_miles = d / 1609.34
            fuel_gallon = f / 3.78541
            mpg = dist_miles / fuel_gallon if fuel_gallon > 0 else 0
            mpg_list.append(mpg)

        return mpg_list

    for penetration_rate in data_dic_with_dynamics_before_calibration.keys():

        mpg_dict = {}
        for label, data_dic in [
            ('Before Calibration', data_dic_with_dynamics_before_calibration),
            ('After Calibration', data_dic_with_dynamics_after_calibration)
        ]:
            mpg_dict[f'{label} - WB'] = get_direction_eval(penetration_rate, data_dic, vtMicroCoeff, df_vehicle_src_coeff, 'wb')
            mpg_dict[f'{label} - EB'] = get_direction_eval(penetration_rate, data_dic, vtMicroCoeff, df_vehicle_src_coeff, 'eb')

        system_mpg_cpmf_dict[penetration_rate] = mpg_dict

    def plot_direction_boxplot(direction):
        labels = []
        box_data = []
        means = []
        stds = []
        positions = []
        pos = 1
        color_map = {
            'Before Calibration': '#FF9999',
            'After Calibration': '#99CCFF'
        }

        for penetration_rate in sorted(system_mpg_cpmf_dict.keys()):
            mpg_dict = system_mpg_cpmf_dict[penetration_rate]
            for condition in ['Before Calibration', 'After Calibration']:
                key = f'{condition} - {direction}'
                mpg_values = mpg_dict[key]
                box_data.append(mpg_values)
                labels.append(f'{penetration_rate*100:.0f}%\n{condition}')
                means.append(np.mean(mpg_values))
                stds.append(np.std(mpg_values))
                positions.append(pos)
                pos += 1
            pos += 1  # Space between penetration groups

        # Plot
        fig, ax = plt.subplots(figsize=(14, 6))
        bp = ax.boxplot(box_data, positions=positions, patch_artist=True, widths=0.6)

        # Box styling
        for patch, label in zip(bp['boxes'], labels):
            if 'Before' in label:
                patch.set_facecolor(color_map['Before Calibration'])
            else:
                patch.set_facecolor(color_map['After Calibration'])
            patch.set_alpha(0.7)

        # Error bars
        ax.errorbar(positions, means, yerr=stds, fmt='o', color='black', capsize=5, label='Mean ± Std')

        # Ticks and labels
        ax.set_xticks(positions)
        ax.set_xticklabels(labels, rotation=45, ha='right', fontsize=10)
        ax.set_ylabel('Miles per Gallon (MPG)', fontsize=14)
        ax.set_title(f'System Fuel Efficiency (MPG) – {direction.upper()} Only', fontsize=16)

        ax.set_ylim(0, 50)
        ax.grid(True, linestyle='--', alpha=0.4)
        ax.legend()
        plt.tight_layout()
        plt.show()

    # Generate separate plots
    plot_direction_boxplot('WB')
    plot_direction_boxplot('EB')

def plot_system_eval_four_conditions(data_dic_condition_1, 
                     data_dic_condition_2, 
                     data_dic_condition_3, 
                     data_dic_condition_4,
                     vtMicroCoeff: pd.DataFrame,
                     VehicleSrcCoeff: pd.DataFrame,
                     condition_1_label = 'Before Calibration (With Vehicle Dynamics)',
                     condition_2_label = 'Before Calibration (Without Vehicle Dynamics)',
                     condition_3_label = 'After Calibration (With Vehicle Dynamics)',
                     condition_4_label = 'After Calibration (Without Vehicle Dynamics)',
                     title = 'System Fuel Efficiency (MPG) Before and After Calibration',
                     ):
    """
    Plot the system fuel efficiency (MPG) before and after calibration with four conditions.
    Args:
        data_dic_with_dynamics_before_calibration: dict[float, dict[str, pd.DataFrame]]
        data_dic_without_dynamics_before_calibration: dict[float, dict[str, pd.DataFrame]]
        data_dic_with_dynamics_after_calibration: dict[float, dict[str, pd.DataFrame]]
        data_dic_without_dynamics_after_calibration: dict[float, dict[str, pd.DataFrame]]
        vtMicroCoeff: pd.DataFrame
        VehicleSrcCoeff: pd.DataFrame
        condition_1_label: str
        condition_2_label: str
        condition_3_label: str
        condition_4_label: str
        # Note the condition 1 and condition 2 should be in the same primary category, and condition 1 and condition 3 share the same secondary category
    """
    system_mpg_cpmf_dict = {}
    def get_round_trip_eval(penetration_rate, data_dic, vtMicroCoeff, VehicleSrcCoeff):
        eco_driving_folder = data_dic[penetration_rate]['path']
        refer_coord = [160, 735]
        step_length = 1
        # _, system_fuel_consume_vt_cpmf_liter_wb, system_fuel_consume_vt_micro_liter_wb, _, _, _, _, system_travel_dist2_m_wb, _ = get_traj_eval(path, 1, vtMicroCoeff, df_vehicle_src_coeff, refer_coord=refer_coord_wb,direction='WB',veh_no='system')
        (energy_evaluation_summary, system_fuel_consume_vt_cpmf_liter, system_fuel_consume_vt_micro_liter,
        system_energy_consume_tractive_kj, system_energy_consume_tractive_regen_kj, system_fuel_consume_yunli_icv_liter,
        system_energy_consume_yunli_ev_kj, system_travel_dist2_m, system_travel_time_s) = (get_traj_eval(eco_driving_folder, step_length, vtMicroCoeff, VehicleSrcCoeff, refer_coord, 'Both'))
        
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb + system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb + system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        system_fuel_efficiency_cpmf_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_cpmf_liter/3.78541)
        system_fuel_efficiency_micro_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_micro_liter/3.78541)
        system_fuel_efficiency_tractive_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_tractive_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_icv_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_yunli_icv_liter/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_ev_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_yunli_ev_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)


        return system_fuel_efficiency_cpmf_mpg
    
    for penetration_rate, _ in data_dic_condition_1.items():

        system_fuel_efficiency_cpmf_mpg_condition_1 = get_round_trip_eval(penetration_rate, 
                                                                                               data_dic_condition_1, 
                                                                                               vtMicroCoeff, 
                                                                                               VehicleSrcCoeff)

        system_fuel_efficiency_cpmf_mpg_condition_2 = get_round_trip_eval(penetration_rate, 
                                                                                                  data_dic_condition_2, 
                                                                                                  vtMicroCoeff, 
                                                                                                  VehicleSrcCoeff)
        system_fuel_efficiency_cpmf_mpg_condition_3 = get_round_trip_eval(penetration_rate, 
                                                                                              data_dic_condition_3, 
                                                                                              vtMicroCoeff, 
                                                                                              VehicleSrcCoeff)
        system_fuel_efficiency_cpmf_mpg_condition_4 = get_round_trip_eval(penetration_rate, 
                                                                                                 data_dic_condition_4, 
                                                                                                 vtMicroCoeff, 
                                                                                                 VehicleSrcCoeff)
        system_mpg_cpmf_dict[penetration_rate] = {condition_1_label: system_fuel_efficiency_cpmf_mpg_condition_1,
                                                  condition_2_label: system_fuel_efficiency_cpmf_mpg_condition_2,
                                                  condition_3_label: system_fuel_efficiency_cpmf_mpg_condition_3,
                                                  condition_4_label: system_fuel_efficiency_cpmf_mpg_condition_4}
    
        print(system_mpg_cpmf_dict[penetration_rate])
       
       # Visualization
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    penetration_rates = list(system_mpg_cpmf_dict.keys())

    # Extract MPG values
    mpg_condition_1 = [system_mpg_cpmf_dict[pr][condition_1_label] for pr in penetration_rates]
    mpg_condition_2 = [system_mpg_cpmf_dict[pr][condition_2_label] for pr in penetration_rates]
    mpg_condition_3 = [system_mpg_cpmf_dict[pr][condition_3_label] for pr in penetration_rates]
    mpg_condition_4 = [system_mpg_cpmf_dict[pr][condition_4_label] for pr in penetration_rates]

    # Define Colors primary category
    colors = {
        "Primary Category 1": "red",  
        "Primary Category 2": "blue"    
    }

    # Define Markers secondary category
    markers = {
        "Secondary Category 1": "o",   # Circle
        "Secondary Category 2": "s" # Square
    }

    # Plot Data
    ax.plot(penetration_rates, mpg_condition_1, color=colors["Primary Category 1"], linestyle='--', marker=markers["Secondary Category 1"], markersize=8, linewidth=2, label=condition_1_label)
    ax.plot(penetration_rates, mpg_condition_2, color=colors["Primary Category 1"], linestyle='-', marker=markers["Secondary Category 2"], markersize=8, linewidth=2, label=condition_2_label)
    
    ax.plot(penetration_rates, mpg_condition_3, color=colors["Primary Category 2"], linestyle='--', marker=markers["Secondary Category 1"], markersize=8, linewidth=2, label=condition_3_label)
    ax.plot(penetration_rates, mpg_condition_4, color=colors["Primary Category 2"], linestyle='-', marker=markers["Secondary Category 2"], markersize=8, linewidth=2, label=condition_4_label)

    # Labels and title
    ax.set_xlabel('Penetration Rate (%)', fontsize=14)
    ax.set_ylabel('Miles per Gallon (MPG)', fontsize=14)
    ax.set_title(title, fontsize=16)

    # Grid and legend
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=12, loc='best')

    plt.tight_layout()
    plt.show() 

def plot_system_eval_two_conditions(data_dic_condition_1, 
                     data_dic_condition_2,
                     vtMicroCoeff: pd.DataFrame,
                     VehicleSrcCoeff: pd.DataFrame,
                     condition_1_label = 'With Vehicle Dynamics',
                     condition_2_label = 'Without Vehicle Dynamics',
                     output_dir = r'.\\Results',
                     ):
    """
    Plot the system fuel efficiency (MPG) with two conditions.
    Args:
        data_dic_condition_1: dict[float, dict[str, pd.DataFrame]]
        data_dic_condition_2: dict[float, dict[str, pd.DataFrame]]
        vtMicroCoeff: pd.DataFrame
        VehicleSrcCoeff: pd.DataFrame
        condition_1_label: str
        condition_2_label: str
    """
    system_mpg_cpmf_dict = {}
    def get_round_trip_eval(penetration_rate, data_dic, vtMicroCoeff, VehicleSrcCoeff):
        eco_driving_folder = data_dic[penetration_rate]['path']
        refer_coord = [160, 735]
        step_length = 0.1
        # _, system_fuel_consume_vt_cpmf_liter_wb, system_fuel_consume_vt_micro_liter_wb, _, _, _, _, system_travel_dist2_m_wb, _ = get_traj_eval(path, 1, vtMicroCoeff, df_vehicle_src_coeff, refer_coord=refer_coord_wb,direction='WB',veh_no='system')
        (energy_evaluation_summary, system_fuel_consume_vt_cpmf_liter, system_fuel_consume_vt_micro_liter,
        system_energy_consume_tractive_kj, system_energy_consume_tractive_regen_kj, system_fuel_consume_yunli_icv_liter,
        system_energy_consume_yunli_ev_kj, system_travel_dist_m, system_travel_dist2_m, system_travel_dist_integration_m, system_travel_time_s) = \
        (get_traj_eval(eco_driving_folder, step_length, vtMicroCoeff, VehicleSrcCoeff, refer_coord, 'WB'))
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb + system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb + system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        system_fuel_efficiency_cpmf_mpg = (system_travel_dist_integration_m/1609.34) / (system_fuel_consume_vt_cpmf_liter/3.78541)
        system_fuel_efficiency_micro_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_micro_liter/3.78541)
        system_fuel_efficiency_tractive_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_tractive_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_icv_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_yunli_icv_liter/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_ev_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_yunli_ev_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)


        return system_fuel_efficiency_cpmf_mpg
    
    for penetration_rate, _ in data_dic_condition_1.items():

        system_fuel_efficiency_cpmf_mpg_condition_1 = get_round_trip_eval(
            penetration_rate,
            data_dic_condition_1,
            vtMicroCoeff,
            VehicleSrcCoeff
        )
        print(system_fuel_efficiency_cpmf_mpg_condition_1)
        system_fuel_efficiency_cpmf_mpg_condition_2 = get_round_trip_eval(
            penetration_rate,
            data_dic_condition_2,
            vtMicroCoeff,
            VehicleSrcCoeff
        )
        print(system_fuel_efficiency_cpmf_mpg_condition_2)
        system_mpg_cpmf_dict[penetration_rate] = {condition_1_label: system_fuel_efficiency_cpmf_mpg_condition_1,
                                                  condition_2_label: system_fuel_efficiency_cpmf_mpg_condition_2}
    
       # Visualization
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    penetration_rates = list(system_mpg_cpmf_dict.keys())

    # Extract MPG values
    mpg_condition_1 = [system_mpg_cpmf_dict[pr][condition_1_label] for pr in penetration_rates]
    mpg_condition_2 = [system_mpg_cpmf_dict[pr][condition_2_label] for pr in penetration_rates]

    # Define Colors (Before vs. After Calibration)
    colors = {
        condition_1_label: "red",  
        condition_2_label: "blue"    
    }

    # Define Markers (With vs. Without Dynamics)
    markers = {
        condition_1_label: "o",   # Circle
        condition_2_label: "s" # Square
    }

    # Plot Data
    ax.plot(penetration_rates, mpg_condition_1, color=colors[condition_1_label], linestyle='--', marker=markers[condition_1_label], markersize=8, linewidth=2, label=condition_1_label)
    ax.plot(penetration_rates, mpg_condition_2, color=colors[condition_2_label], linestyle='-', marker=markers[condition_2_label], markersize=8, linewidth=2, label=condition_2_label)

    # Labels and title
    ax.set_xlabel('Penetration Rate (%)', fontsize=14)
    ax.set_ylabel('Miles per Gallon (MPG)', fontsize=14)
    # ax.set_title(f'System Fuel Efficiency (MPG) {condition_1_label} and {condition_2_label}', fontsize=16)

    # Grid and legend
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=12, loc='best')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'system_eval.png'))
    
def plot_ego_eval_two_conditions(data_dic_condition_1, 
                                data_dic_condition_2, 
                                condition_1_label = 'With Vehicle Dynamics',
                                condition_2_label = 'Without Vehicle Dynamics',
                                simulink=True,
                                output_dir = r'.\\Results',
                                ):
    # Extract trip data and labels
    trip_data_condition_1, trip_counts_condition_1, trip_labels_condition_1 = get_energy_consumption(data_dic_condition_1, direction='WB', simulink=simulink)
    trip_data_condition_2, trip_counts_condition_2, trip_labels_condition_2 = get_energy_consumption(data_dic_condition_2,direction='WB', simulink=simulink)
    
    # Calculate global y-axis limits
    # all_trip_data = trip_data_condition_1 + trip_data_condition_2
    # y_min, y_max = min(map(min, all_trip_data)) * 0.9, max(map(max, all_trip_data)) * 1.1  # Adding buffer for clarity
    y_min, y_max = 110, 260
    # use persentile to set the y axis limit
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    # Define function to create boxplots
    def create_boxplot(ax, trip_data, trip_labels, title, trip_counts):
        boxplot = ax.boxplot(
            trip_data,
            tick_labels=trip_labels,
            whis=1.0,
            showfliers=False,
            patch_artist=True,  # Fill the boxes with color
            boxprops=dict(facecolor='lightblue', color='blue'),  # Box properties
            medianprops=dict(color='red', linewidth=2),  # Median line properties
            whiskerprops=dict(color='blue', linewidth=1.5),  # Whisker properties
            capprops=dict(color='blue', linewidth=1.5),  # Cap properties
            flierprops=dict(marker='o', color='red', alpha=0.0)  # Outlier properties
        )
        ax.set_ylabel('Fuel Efficiency (MPGe)', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_ylim(y_min, y_max)  # Set uniform y-axis limits

        # Add sample sizes as annotations
        for i, count in enumerate(trip_counts):
            ax.text(i + 1, y_max * 0.95, f'n={count}', ha='center', va='top', fontweight='bold', fontsize=10)

    # Create four boxplots
    create_boxplot(axes[0], trip_data_condition_1, trip_labels_condition_1, condition_1_label, trip_counts_condition_1)
    create_boxplot(axes[1], trip_data_condition_2, trip_labels_condition_2, condition_2_label, trip_counts_condition_2)


    # Adjust layout and show plot
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'ego_eval.png'))  

def plot_ego_eval_two_conditions_merged(data_dic_condition_1, 
                                data_dic_condition_2, 
                                condition_1_label = 'With Vehicle Dynamics',
                                condition_2_label = 'Without Vehicle Dynamics',
                                simulink=True,
                                output_dir = r'.\\Results',
                                ):
    # Extract trip data and labels
    trip_data_condition_1, trip_counts_condition_1, trip_labels_condition_1 = get_energy_consumption(data_dic_condition_1, direction='WB', simulink=simulink)
    trip_data_condition_2, trip_counts_condition_2, trip_labels_condition_2 = get_energy_consumption(data_dic_condition_2,direction='WB', simulink=simulink)
    
    # Calculate global y-axis limits
    # all_trip_data = trip_data_condition_1 + trip_data_condition_2
    # y_min, y_max = min(map(min, all_trip_data)) * 0.9, max(map(max, all_trip_data)) * 1.1  # Adding buffer for clarity
    y_min, y_max = 110, 280
    # use persentile to set the y axis limit
    fig, ax = plt.subplots(figsize=(12 , 4))
    
    # Define function to create boxplots
    def create_boxplot_with_color_legend(ax,
                                         trip_data_condition_1,
                                         trip_data_condition_2,
                                         trip_labels_condition_1,
                                         trip_labels_condition_2,
                                         condition_1_label='With Dynamics',
                                         condition_2_label='Without Dynamics',
                                         trip_counts_condition_1=None,
                                         trip_counts_condition_2=None,
                                         y_min=0,
                                         y_max=100,
                                         title='Fuel Efficiency Comparison'):
        """
        Plot side-by-side boxplots for each MPR, using only one 'Base' from condition 2.
        """
        condition_1_color = '#1f77b4'
        condition_2_color = '#ff7f0e'

        # Pair MPR labels by index, but skip 'Base' from condition_1
        positions = []
        box_data = []
        box_colors = []
        box_labels = []
        n_annotations = []

        width = 0.4
        base_plotted = False
        index = 1

        for i in range(len(trip_labels_condition_1)):
            label1 = trip_labels_condition_1[i]
            label2 = trip_labels_condition_2[i]

            if label1 == 'Base' and label2 == 'Base':
                # Only plot Base once from condition_2
                if not base_plotted:
                    positions.append(index)
                    box_data.append(trip_data_condition_2[i])
                    box_colors.append(condition_2_color)
                    box_labels.append('Base')
                    n_annotations.append(trip_counts_condition_2[i])
                    index += 1
                    base_plotted = True
                continue

            # Plot condition 1
            positions.append(index - width / 2)
            box_data.append(trip_data_condition_1[i])
            box_colors.append(condition_1_color)
            box_labels.append('')
            n_annotations.append(trip_counts_condition_1[i])

            # Plot condition 2
            positions.append(index + width / 2)
            box_data.append(trip_data_condition_2[i])
            box_colors.append(condition_2_color)
            box_labels.append(trip_labels_condition_1[i])  # show label only once
            n_annotations.append(trip_counts_condition_2[i])

            index += 1

        # Draw boxplots
        boxplot = ax.boxplot(box_data,
                             positions=positions,
                             widths=width,
                             patch_artist=True,
                             whis=1.0,
                             showfliers=False,
                             boxprops=dict(color='gray'),
                             medianprops=dict(color='#ff4c4c', linewidth=2),
                             whiskerprops=dict(color='gray', linewidth=1.5),
                             capprops=dict(color='gray', linewidth=1.5))

        for patch, color in zip(boxplot['boxes'], box_colors):
            patch.set_facecolor(color)

        # Annotate n=...
        for pos, n in zip(positions, n_annotations):
            ax.text(pos, y_max * 0.97, f'n={n}', ha='center', va='top', fontsize=9)

        # X-axis ticks and labels
        xtick_pos = []
        xtick_labels = []
        tick_idx = 1
        for label in trip_labels_condition_1:
            if label == 'Base' and base_plotted:
                xtick_pos.append(tick_idx)
                xtick_labels.append('Base')
                tick_idx += 1
                base_plotted = False
            elif label != 'Base':
                xtick_pos.append(tick_idx)
                xtick_labels.append(label)
                tick_idx += 1

        ax.set_xticks(xtick_pos)
        ax.set_xticklabels(xtick_labels)
        ax.set_xlabel('Penetration Rate (%)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Individual Fuel Efficiency (MPGe)', fontsize=14, fontweight='bold')
        # ax.set_title(title, fontsize=14, fontweight='bold')
        ax.set_ylim(y_min, y_max)
        ax.grid(True, linestyle='--', alpha=0.6)

        # Legend
        legend_patches = [
            mpatches.Patch(color=condition_1_color, label=condition_1_label),
            mpatches.Patch(color=condition_2_color, label=condition_2_label)
        ]
        ax.legend(handles=legend_patches, loc='best', fontsize=10)


    create_boxplot_with_color_legend(ax,
        trip_data_condition_1, 
        trip_data_condition_2,
        trip_labels_condition_1,
        trip_labels_condition_2,
        condition_1_label=condition_1_label,
        condition_2_label=condition_2_label,
        trip_counts_condition_1=trip_counts_condition_1,
        trip_counts_condition_2=trip_counts_condition_2,
        y_min=y_min, y_max=y_max,
        title=f'Individual Vehicle Fuel Efficiency (MPGe)'
    )


    # Adjust layout and show plot
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'ego_eval.png'))  


def plot_ego_eval_two_conditions_compared_to_base(data_dic_condition_1, 
                                data_dic_condition_2, 
                                condition_1_label = 'With Vehicle Dynamics',
                                condition_2_label = 'Without Vehicle Dynamics',
                                simulink=True,
                                output_dir = r'.\\Results',
                                ):
    # Extract trip data and labels
    trip_data_condition_1, _, trip_labels_condition_1 = get_energy_consumption(data_dic_condition_1, 
                                                                                                     direction='WB',
                                                                                                     simulink=simulink,
                                                                                                     separate_trips=False)
    trip_data_condition_2, _, trip_labels_condition_2 = get_energy_consumption(data_dic_condition_2,
                                                                                                     direction='WB', 
                                                                                                     simulink=simulink, 
                                                                                                     separate_trips=False)
 
    # use persentile to set the y axis limit
    fig, ax = plt.subplots(figsize=(12 , 5))
    
    # Define function to create boxplots
    def create_line_plot_with_color_legend(ax,
                                         trip_data_condition_1,
                                         trip_data_condition_2,
                                         trip_labels_condition_1,
                                         trip_labels_condition_2,
                                         condition_1_label='With Dynamics',
                                         condition_2_label='Without Dynamics',
                                         title='Fuel Efficiency Comparison'):
        """
        Plot side-by-side boxplots for each MPR, using only one 'Base' from condition 2.
        """
        
        # build dictionary
        trip_data_condition_1 = dict(zip(trip_labels_condition_1, trip_data_condition_1))
        trip_data_condition_2 = dict(zip(trip_labels_condition_2, trip_data_condition_2))
        condition_1_base_mpge = trip_data_condition_1['Base']
        condition_2_base_mpge = trip_data_condition_2['Base']
        # Draw boxplots
        penetration_rates = list([pr for pr in trip_data_condition_1.keys() if pr != 'Base'])

        # Extract MPG values
        fuel_efficiency_improvement_condition_1 = [((trip_data_condition_1[pr] - condition_1_base_mpge) / trip_data_condition_1[pr]) * 100 for pr in penetration_rates]
        fuel_efficiency_improvement_condition_2 = [((trip_data_condition_2[pr] - condition_2_base_mpge) / trip_data_condition_2[pr]) * 100 for pr in penetration_rates]

        # Define Colors (Before vs. After Calibration)
        colors = {
            condition_1_label: "red",  
            condition_2_label: "blue"    
        }

        # Define Markers (With vs. Without Dynamics)
        markers = {
            condition_1_label: "o",   # Circle
            condition_2_label: "s" # Square
        }

        # Plot Data
        ax.plot(penetration_rates, fuel_efficiency_improvement_condition_1, color=colors[condition_1_label], linestyle='--', marker=markers[condition_1_label], markersize=8, linewidth=2, label=condition_1_label)
        ax.plot(penetration_rates, fuel_efficiency_improvement_condition_2, color=colors[condition_2_label], linestyle='-', marker=markers[condition_2_label], markersize=8, linewidth=2, label=condition_2_label)


        ax.set_xlabel('Market Penetration Rate (%)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Energy Efficiency Improvement (%)', fontsize=14, fontweight='bold')
        # ax.set_title(title, fontsize=14, fontweight='bold')

        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend(fontsize=12, loc='best')
        plt.tight_layout()


    create_line_plot_with_color_legend(ax,
        trip_data_condition_1, 
        trip_data_condition_2,
        trip_labels_condition_1,
        trip_labels_condition_2,
        condition_1_label=condition_1_label,
        condition_2_label=condition_2_label,
        title=f'Fuel Efficiency Improvement (%)'
    )


    # Adjust layout and show plot
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'ego_eval.png'))  


def plot_signal_changes_xml(ax, sumosignalresult, sumoSignalConfig, direction='EB', extra_label=""):
    
    sumoSignalConfig = sumoSignalConfig[sumoSignalConfig['approach_direction'] == direction].reset_index(drop=True)
    if direction == 'WB':
        sumoSignalConfig['distance'] = sumoSignalConfig['distance_wb']
    elif direction == 'EB':
        sumoSignalConfig['distance'] = sumoSignalConfig['distance_eb']
    for sc in sumoSignalConfig.id.unique():
        
        phase_num = sumoSignalConfig[sumoSignalConfig['id'] == sc]['name'].values[0]
        movement_indexes = eval(sumoSignalConfig[sumoSignalConfig['id'] == sc]['movement_index'].values[0])
        x_loc = sumoSignalConfig[sumoSignalConfig['id'] == sc]['distance'].values[0]

        signal_changes_tem = sumosignalresult[sumosignalresult['id'] == sc].reset_index(drop=True)
        signal_changes_tem['target_phase_state'] = signal_changes_tem['state'].str[movement_indexes[0]: movement_indexes[0] + 1]

        # only keep phase changes for the targeted phase
        signal_changes_tem = signal_changes_tem[signal_changes_tem['target_phase_state'] != signal_changes_tem['target_phase_state'].shift()]
        signal_changes_tem['target_phase_state'] = signal_changes_tem['target_phase_state'].map({'G': 'green', 'g': 'green', 'y': 'yellow', 'r': 'red', 's': 'red'})
        
        cnt = 1
        prev_idx = 0
        prev_clr = signal_changes_tem.loc[prev_idx,'target_phase_state']
        
        for index, row in signal_changes_tem.iterrows():
            if index >= 1:
                
                y1 = [x_loc, x_loc]
                x1 = [float(signal_changes_tem.loc[prev_idx,'time']), float(row.time)]
                ax.plot(x1, y1, color=prev_clr, linewidth=7, solid_capstyle='butt')
                c = row.target_phase_state

                prev_idx = index
                prev_clr = c
                cnt += 1
            else:
                continue

    ax.set_title(f'{extra_label}', fontsize=45, fontweight='bold')
    ax.set_xlabel('Simulation Second (s)', fontsize=40, fontweight='bold')
    ax.set_ylabel(f'Distance along {direction} Direction', fontsize=40, fontweight='bold')
    # ax.set_xlim(plot_start_time, plot_start_time + duration_limit)
    ax.set_yticks(sumoSignalConfig['distance'].values)
    ax.set_yticklabels(sumoSignalConfig['int_name'].values, fontsize=30)
    ax.tick_params(axis='both', which='major', labelsize=30)
    # ax.invert_yaxis()
    return ax

def plot_trajectory(ax, trajectory_data, highlight_veh_number='None', plot_start_time=29000, plot_end_time=32400):
    line_width = 1
    trajectory_data = get_exclude_turn_traj(trajectory_data, exclude_turning_traj=True)
    trajectory_data['time'] = trajectory_data['time'].astype(float)
    trajectory_data['distance'] = trajectory_data['distance'].astype(float)
    highlight_veh_data = trajectory_data[trajectory_data['id'] == highlight_veh_number]
    trajectory_data = trajectory_data[trajectory_data['id'] != highlight_veh_number]


    sns.lineplot(ax=ax, data=trajectory_data[trajectory_data['type'] == 'HDV'], x='time', y='distance', hue='id', legend=False, palette=['lightgrey'])
    sns.lineplot(ax=ax, data=trajectory_data[trajectory_data['type'] == 'CAV'], x='time', y='distance', hue='id', legend=False, palette=['grey'])
    
    highlight_veh_data = assign_trip_id(highlight_veh_data, 100)
    # highlight a specific vehicle
    if highlight_veh_number != 'None':
        
        clr = 'green'
        line_width = 4
        for group_name, group in highlight_veh_data.groupby('tripId'):
            ax.plot(group['time'].astype(float), group['distance'].astype(float), color=clr, linewidth=line_width, solid_capstyle='butt')
    ax.set_xlim(0, 3600)
    return ax


def plot_space_time_diagram_two_conditions(data_dic_condition_1,
                                           data_dic_condition_2,
                                           condition_1_label = 'With Vehicle Dynamics',
                                           condition_2_label = 'Without Vehicle Dynamics',
                                           output_dir = r'.\\Results'):
    # plot the west bound first
    refer_coord_eb = [160, 735]
    refer_coord_wb = [1390, 225]
    relative_time = 28800
    wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
    eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']
    for idx, pr in enumerate(data_dic_condition_1.keys()):
        if data_dic_condition_2 is not None:
            fig, axes = plt.subplots(1, 2)
            fig.set_size_inches(48, 10)
        else:
            fig, axes = plt.subplots(1, 1)
            fig.set_size_inches(24, 10)
        path_condition_1 = data_dic_condition_1[pr]['path']
        sumo_signal_condition_1 = read_signal_xml(os.path.join(path_condition_1, 'signal_result.xml'))
        sumo_signal_condition_1['time'] = sumo_signal_condition_1['time'].astype(float) - relative_time
        sumoSignalConfig_condition_1 = data_dic_condition_1[pr]['sumo_signal_config']
        trajectory_data_condition_1 = read_trajectory_xml(os.path.join(path_condition_1, 'fcd.xml'), refer_coord=refer_coord_wb)
        trajectory_data_condition_1 = trajectory_process(trajectory_data_condition_1, eb_lanes, wb_lanes, relative_time)
        # only keep the WB direction trajectories
        trajectory_data_wb_condition_1 = trajectory_data_condition_1[trajectory_data_condition_1['direction'] == 'WB'].reset_index(drop=True)
        if data_dic_condition_2 is None:
            axes = plot_trajectory(axes, trajectory_data_wb_condition_1, highlight_veh_number='ego')
            axes = plot_signal_changes_xml(axes, sumo_signal_condition_1, sumoSignalConfig_condition_1, direction='WB', extra_label=f'{condition_1_label} MPR: {pr:.0f}%')
        else:
            axes[0] = plot_trajectory(axes[0], trajectory_data_wb_condition_1, highlight_veh_number='ego')
            axes[0] = plot_signal_changes_xml(axes[0], sumo_signal_condition_1, sumoSignalConfig_condition_1, direction='WB', extra_label=f'{condition_1_label} MPR: {pr:.0f}%')

        if data_dic_condition_2 is not None:
            path_condition_2 = data_dic_condition_2[pr]['path']
            sumo_signal_condition_2 = read_signal_xml(os.path.join(path_condition_2, 'signal_result.xml'))
            sumo_signal_condition_2['time'] = sumo_signal_condition_2['time'].astype(float) - relative_time
            sumoSignalConfig_condition_2 = data_dic_condition_2[pr]['sumo_signal_config']
            trajectory_data_condition_2 = read_trajectory_xml(os.path.join(path_condition_2, 'fcd.xml'), refer_coord=refer_coord_wb)
            trajectory_data_condition_2 = trajectory_process(trajectory_data_condition_2, eb_lanes, wb_lanes, relative_time)
            # only keep the WB direction trajectories
            trajectory_data_wb_condition_2 = trajectory_data_condition_2[trajectory_data_condition_2['direction'] == 'WB'].reset_index(drop=True)
            axes[1]  = plot_trajectory(axes[1], trajectory_data_wb_condition_2, highlight_veh_number='ego')
            axes[1] = plot_signal_changes_xml(axes[1], sumo_signal_condition_2, sumoSignalConfig_condition_2, direction='WB', extra_label=f'{condition_2_label} MPR: {pr:.0f}%')
    
        plt.tight_layout()
        print(f'space_time_diagram_{pr:.0f}.png')
        plt.savefig(os.path.join(output_dir, f'space_time_diagram_{pr:.0f}.png'))
    
def plot_space_time_diagram_one_condition(data_dic_condition_1,
                                           condition_1_label = 'With Vehicle Dynamics',
                                           output_dir = r'.\\Results'):
    setting_cnt = len(data_dic_condition_1)
    fig, axes = plt.subplots(setting_cnt, 1, figsize=(80, 10 * setting_cnt))
    # plot the west bound first
    refer_coord_eb = [160, 735]
    refer_coord_wb = [1390, 225]
    for idx, pr in enumerate(data_dic_condition_1.keys()):
        path_condition_1 = data_dic_condition_1[pr]['path']
        sumo_signal_condition_1 = read_signal_xml(os.path.join(path_condition_1, 'signal_result.xml'))
        sumoSignalConfig_condition_1 = data_dic_condition_1[pr]['sumo_signal_config']
        trajectory_data_condition_1 = read_trajectory_xml(os.path.join(path_condition_1, 'fcd.xml'), refer_coord=refer_coord_wb)

        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']
        relative_time = 28800
        sumo_signal_condition_1['time'] = sumo_signal_condition_1['time'].astype(float) - relative_time
        
        trajectory_data_condition_1 = trajectory_process(trajectory_data_condition_1, eb_lanes, wb_lanes, relative_time)
        trajectory_data_wb_condition_1 = trajectory_data_condition_1[trajectory_data_condition_1['direction'] == 'WB'].reset_index(drop=True)
        
        axes[idx][0] = plot_trajectory(axes[idx][0], trajectory_data_wb_condition_1, highlight_veh_number='ego')
        axes[idx][0] = plot_signal_changes_xml(axes[idx][0], sumo_signal_condition_1, sumoSignalConfig_condition_1, direction='WB', extra_label=f'PR: {pr:.0f} {condition_1_label}%')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'space_time_diagram_{condition_1_label}.png'))
    
def plot_speed_acc_distribution(data_dic,
                                data_label = 'Without Vehicle Dynamics',
                                output_dir = r'.\\Results',
                                step_length = 0.1,
                                ):
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    # plot the west bound first
    refer_coord_eb = [160, 735]
    refer_coord_wb = [1390, 225]
    
    # penetration_rates = [pr/100 for pr in data_dic.keys()] # convert percentage to decimal
    penetration_rates = list(data_dic.keys())
    color_map = mpl.colormaps.get_cmap('Greens')
    color_map = color_map(np.linspace(0.2, 1.0, len(penetration_rates)))
    color_map = dict(zip(penetration_rates, color_map)) # convert to dict
    color_map[0.0] = 'lightgrey'
    data_dic = dict(sorted(data_dic.items(), key=lambda x: x[0], reverse=True))
    
    for idx, pr in enumerate(data_dic.keys()):
        print(f'plotting speed_acc_distribution_{pr:.0f}')
        path = data_dic[pr]['path']
        trajectory_data = read_trajectory_xml(os.path.join(path, 'fcd.xml'), refer_coord=refer_coord_wb)
        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']
        relative_time = 28800
        trajectory_data['speed'] = trajectory_data['speed'].astype(float)
        trajectory_data['acceleration'] = trajectory_data.groupby('id')['speed'].diff(periods=-1)/step_length
        trajectory_data['acceleration'] = trajectory_data['acceleration'].fillna(0.0)
        
        
        trajectory_data = trajectory_process(trajectory_data, eb_lanes, wb_lanes, relative_time)
        trajectory_data_wb = trajectory_data[trajectory_data['direction'] == 'WB'].reset_index(drop=True)
        pr
        sns.kdeplot(ax=axes[0], 
                    data=trajectory_data_wb[trajectory_data_wb['speed'] > 1e-4], 
                    x='speed', color=color_map[pr], 
                    bw_adjust=1, 
                    label=f'PR: {pr:.0f}%', 
                    fill=False,
                    alpha=.5,
                    linewidth=2,
                    common_norm=False,
                    clip=(0, 100))
        sns.kdeplot(ax=axes[1], 
                    data=trajectory_data_wb[abs(trajectory_data_wb['acceleration']) > 1e-3], 
                    x='acceleration', 
                    color=color_map[pr], 
                    bw_adjust=1, 
                    label=f'PR: {pr:.0f}%', 
                    fill=False,
                    alpha=.5,
                    linewidth=2,
                    common_norm=False,
                    clip=(-4, 3))
        print('finished plotting speed_acc_distribution')
    axes[0].set_title(f'Speed Distribution', fontsize=16, fontweight='bold')
    axes[1].set_title(f'Acceleration Distribution', fontsize=16, fontweight='bold')
    axes[0].set_xlabel('Speed (m/s)', fontsize=14, fontweight='bold')
    axes[1].set_xlabel('Acceleration (m/s^2)', fontsize=14, fontweight='bold')
    axes[0].set_ylabel('Density', fontsize=14, fontweight='bold')
    axes[1].set_ylabel('Density', fontsize=14, fontweight='bold')
    axes[0].legend(fontsize=12, loc='best')
    axes[1].legend(fontsize=12, loc='best')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'speed_acc_distribution_{data_label}.png'))


def plot_speed_acc_distribution_comparison(data_dic_condition_1,
                                data_dic_condition_2,
                                condition_1_label = 'With Vehicle Dynamics',
                                condition_2_label = 'Without Vehicle Dynamics',
                                output_dir = r'.\\Results',
                                step_length = 0.1,
                                ):
    
    
    # plot the west bound first
    refer_coord_eb = [160, 735]
    refer_coord_wb = [1390, 225]
    
    for idx, pr in enumerate(data_dic_condition_1.keys()):
        fig, axes = plt.subplots(2, 1, figsize=(12, 7))
        path_condition_1 = data_dic_condition_1[pr]['path']
        trajectory_data_condition_1 = read_trajectory_xml(os.path.join(path_condition_1, 'fcd.xml'), refer_coord=refer_coord_wb)

        path_condition_2 = data_dic_condition_2[pr]['path']
        trajectory_data_condition_2 = read_trajectory_xml(os.path.join(path_condition_2, 'fcd.xml'), refer_coord=refer_coord_wb)

        wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
        eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313', '-284', '-2841', '-328', '-304']
        relative_time = 28800
        trajectory_data_condition_1['speed'] = trajectory_data_condition_1['speed'].astype(float)
        trajectory_data_condition_1['acceleration'] = trajectory_data_condition_1.groupby('id')['speed'].diff(periods=-1)/step_length
        trajectory_data_condition_1['acceleration'] = trajectory_data_condition_1['acceleration'].fillna(0.0)
        trajectory_data_condition_2['speed'] = trajectory_data_condition_2['speed'].astype(float)
        trajectory_data_condition_2['acceleration'] = trajectory_data_condition_2.groupby('id')['speed'].diff(periods=-1)/step_length
        trajectory_data_condition_2['acceleration'] = trajectory_data_condition_2['acceleration'].fillna(0.0)
        trajectory_data_condition_1 = trajectory_process(trajectory_data_condition_1, eb_lanes, wb_lanes, relative_time)
        trajectory_data_condition_2 = trajectory_process(trajectory_data_condition_2, eb_lanes, wb_lanes, relative_time)

        trajectory_data_wb_condition_1 = trajectory_data_condition_1[trajectory_data_condition_1['direction'] == 'WB'].reset_index(drop=True)
        trajectory_data_wb_condition_2 = trajectory_data_condition_2[trajectory_data_condition_2['direction'] == 'WB'].reset_index(drop=True)

        sns.kdeplot(ax=axes[0], 
                    data=trajectory_data_wb_condition_1, 
                    x='speed', 
                    # color=color_condition_1, 
                    bw_adjust=0.75, 
                    label=f'{condition_1_label}',
                    alpha=1,
                    common_norm=False,
                    clip=(0, 100))
        sns.kdeplot(ax=axes[0], 
                    data=trajectory_data_wb_condition_2, 
                    x='speed', 
                    # color=color_condition_2, 
                    bw_adjust=0.75, 
                    label=f'{condition_2_label}', 
                    alpha=1,
                    common_norm=False,
                    clip=(0, 100))
        sns.kdeplot(ax=axes[1], 
                    data=trajectory_data_wb_condition_1, 
                    x='acceleration', 
                    # color=color_condition_1, 
                    bw_adjust=0.75, 
                    label=f'{condition_1_label}', 
                    alpha=1,
                    common_norm=False,
                    clip=(-200, 200))
        sns.kdeplot(ax=axes[1], 
                    data=trajectory_data_wb_condition_2, 
                    x='acceleration', 
                    # color=color_condition_2, 
                    bw_adjust=0.75, 
                    label=f'{condition_2_label}', 
                    alpha=1,
                    common_norm=False,
                    clip=(-200, 200))
        axes[0].set_title(f'Speed Distribution', fontsize=12, fontweight='bold')
        axes[1].set_title(f'Acceleration Distribution', fontsize=12, fontweight='bold')
        axes[0].set_xlabel('Speed (m/s)', fontsize=12, fontweight='bold')
        axes[1].set_xlabel('Acceleration (m/s^2)', fontsize=12, fontweight='bold')
        axes[0].set_ylabel('Density', fontsize=12, fontweight='bold')
        axes[1].set_ylabel('Density', fontsize=12, fontweight='bold')
        axes[0].legend(fontsize=8, loc='best')
        axes[1].legend(fontsize=8, loc='best')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'speed_acc_comparison_{pr}.png'))
        
def plot_speed_acc_distribution_comparison_ego_upstream(data_dic_condition_1,
                                data_dic_condition_2,
                                condition_1_label = 'Simulink Vehicle Dynamics',
                                condition_2_label = 'SUMO Default Dynamics',
                                output_dir = r'.\\Results',
                                upstream_max_vehicles_count=5,
                                mode = 'ego'
                                ):
    
    assert mode in ['ego', 'upstream']
    speed_distribution_comparison_df = pd.DataFrame()
    acc_distribution_comparison_df = pd.DataFrame()
    speed_distribution_comparison_condition_1_with_ego_df = pd.DataFrame()
    acc_distribution_comparison_condition_1_with_ego_df = pd.DataFrame()
    speed_distribution_comparison_condition_2_with_ego_df = pd.DataFrame()
    acc_distribution_comparison_condition_2_with_ego_df = pd.DataFrame()
    for idx, pr in enumerate(data_dic_condition_1.keys()):
        fig, axes = plt.subplots(1, 2, figsize=(28, 6))
        path_condition_1 = data_dic_condition_1[pr]['path']
        upstream_results_df = pd.read_csv(os.path.join(path_condition_1, f'upstream_results_{pr}.csv'))

        path_condition_2 = data_dic_condition_2[pr]['path']
        upstream_results_df_2 = pd.read_csv(os.path.join(path_condition_2, f'upstream_results_{pr}.csv'))
        speed_condition_1 = []
        speed_condition_2 = []
        acc_condition_1 = []
        acc_condition_2 = []
        if mode == 'ego':
            speed_condition_1 = upstream_results_df['ego_speed']
            acc_condition_1 = upstream_results_df['ego_acc']
            speed_condition_2 = upstream_results_df_2['ego_speed']
            acc_condition_2 = upstream_results_df_2['ego_acc']
        elif mode == 'upstream':
            for vehicle_idx in range(1, upstream_max_vehicles_count+1):
                speed_condition_1.extend(upstream_results_df[f'veh{vehicle_idx}_speed'].values.tolist())
                acc_condition_1.extend(upstream_results_df[f'veh{vehicle_idx}_acc'].values.tolist())
                speed_condition_2.extend(upstream_results_df_2[f'veh{vehicle_idx}_speed'].values.tolist())
                acc_condition_2.extend(upstream_results_df_2[f'veh{vehicle_idx}_acc'].values.tolist())
            speed_condition_1_ego = upstream_results_df['ego_speed']
            acc_condition_1_ego = upstream_results_df['ego_acc']
            speed_condition_2_ego = upstream_results_df_2['ego_speed']
            acc_condition_2_ego = upstream_results_df_2['ego_acc']
            speed_condition_1_ego = pd.DataFrame({'speed': speed_condition_1_ego})
            acc_condition_1_ego = pd.DataFrame({'acceleration': acc_condition_1_ego})
            speed_condition_2_ego = pd.DataFrame({'speed': speed_condition_2_ego})
            acc_condition_2_ego = pd.DataFrame({'acceleration': acc_condition_2_ego})
        
        speed_condition_1 = pd.DataFrame({'speed': speed_condition_1})
        acc_condition_1 = pd.DataFrame({'acceleration': acc_condition_1})
        speed_condition_2 = pd.DataFrame({'speed': speed_condition_2})
        acc_condition_2 = pd.DataFrame({'acceleration': acc_condition_2})
        # filter out the speed and abs(acc) that are too small
        speed_condition_1 = speed_condition_1[speed_condition_1['speed'] > 0]
        acc_condition_1 = acc_condition_1[abs(acc_condition_1['acceleration']) > 1e-3]
        speed_condition_2 = speed_condition_2[speed_condition_2['speed'] > 0]
        acc_condition_2 = acc_condition_2[abs(acc_condition_2['acceleration']) > 1e-3]
        speed_distribution_comparison = compare_distributions(speed_condition_1['speed'], speed_condition_2['speed'])
        acc_distribution_comparison = compare_distributions(acc_condition_1['acceleration'], acc_condition_2['acceleration'])
        if mode == 'upstream':
            speed_distribution_condition_1_comparison_with_ego = compare_distributions(speed_condition_1_ego['speed'],  speed_condition_1['speed'])
            acc_distribution_condition_1_comparison_with_ego = compare_distributions(acc_condition_1_ego['acceleration'],  acc_condition_1['acceleration'])
            speed_distribution_condition_2_comparison_with_ego = compare_distributions(speed_condition_2_ego['speed'],  speed_condition_2['speed'])
            acc_distribution_condition_2_comparison_with_ego = compare_distributions(acc_condition_2_ego['acceleration'],  acc_condition_2['acceleration'])
            speed_distribution_comparison_condition_1_with_ego_df = pd.concat([speed_distribution_comparison_condition_1_with_ego_df, pd.DataFrame({'penetration_rate': pr, **speed_distribution_condition_1_comparison_with_ego}, index=[0])], ignore_index=True)
            acc_distribution_comparison_condition_1_with_ego_df = pd.concat([acc_distribution_comparison_condition_1_with_ego_df, pd.DataFrame({'penetration_rate': pr, **acc_distribution_condition_1_comparison_with_ego}, index=[0])], ignore_index=True)
            speed_distribution_comparison_condition_2_with_ego_df = pd.concat([speed_distribution_comparison_condition_2_with_ego_df, pd.DataFrame({'penetration_rate': pr, **speed_distribution_condition_2_comparison_with_ego}, index=[0])], ignore_index=True)
            acc_distribution_comparison_condition_2_with_ego_df = pd.concat([acc_distribution_comparison_condition_2_with_ego_df, pd.DataFrame({'penetration_rate': pr, **acc_distribution_condition_2_comparison_with_ego}, index=[0])], ignore_index=True)
        speed_distribution_comparison_df = pd.concat([speed_distribution_comparison_df, pd.DataFrame({'penetration_rate': pr, **speed_distribution_comparison}, index=[0])], ignore_index=True)
        acc_distribution_comparison_df = pd.concat([acc_distribution_comparison_df, pd.DataFrame({'penetration_rate': pr, **acc_distribution_comparison}, index=[0])], ignore_index=True)
        sns.kdeplot(ax=axes[0], 
                    data=speed_condition_1, 
                    # color=color_condition_1, 
                    bw_adjust=0.75, 
                    label=f'{condition_1_label}',
                    alpha=.2,
                    fill=True,
                    common_norm=False,
                    clip=(0, 100))
        sns.kdeplot(ax=axes[0], 
                    data=speed_condition_2, 
                    x='speed', 
                    # color=color_condition_2, 
                    bw_adjust=0.75, 
                    label=f'{condition_2_label}', 
                    alpha=.2,
                    fill=True,
                    common_norm=False,
                    clip=(0, 100))
        sns.kdeplot(ax=axes[1], 
                    data=acc_condition_1, 
                    x='acceleration', 
                    # color=color_condition_1, 
                    bw_adjust=0.75, 
                    label=f'{condition_1_label}', 
                    alpha=.2,
                    fill=True,
                    common_norm=False,
                    clip=(-4, 3))
        sns.kdeplot(ax=axes[1], 
                    data=acc_condition_2, 
                    x='acceleration', 
                    # color=color_condition_2, 
                    bw_adjust=0.75, 
                    label=f'{condition_2_label}', 
                    alpha=.2,
                    fill=True,
                    common_norm=False,
                    clip=(-4, 3))
        
        # axes[0].set_title(f'Speed Distribution', fontsize=35, fontweight='bold')
        # axes[1].set_title(f'Acceleration Distribution', fontsize=35, fontweight='bold')
        axes[0].set_xlabel('Speed (m/s)', fontsize=35, fontweight='bold')
        axes[1].set_xlabel('Acceleration (m/s^2)', fontsize=35, fontweight='bold')
        axes[0].set_ylabel('Density', fontsize=35, fontweight='bold')
        axes[1].set_ylabel('Density', fontsize=35, fontweight='bold')
        axes[0].legend(fontsize=25, loc='best')
        axes[1].legend(fontsize=25, loc='best')
        axes[0].set_ylim(0, 0.2)
        axes[1].set_ylim(0, 1.0)
        setting_label = 'Ego Vehicle' if mode == 'ego' else f'Following {upstream_max_vehicles_count} Vehicles'
        fig.suptitle(f'{setting_label} MPR: {pr:.0f}%', fontsize=40, fontweight='bold')
        plt.tight_layout()
        label = 'ego' if mode == 'ego' else f'upstream_{upstream_max_vehicles_count}'
        plt.savefig(os.path.join(output_dir, f'speed_acc_distribution_{label}_{pr}.png'))
    speed_distribution_comparison_df.to_csv(os.path.join(output_dir, f'speed_distribution_comparison_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
    acc_distribution_comparison_df.to_csv(os.path.join(output_dir, f'acc_distribution_comparison_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
    if mode == 'upstream':
        speed_distribution_comparison_condition_1_with_ego_df.to_csv(os.path.join(output_dir, f'speed_distribution_comparison_{condition_1_label}_with_ego_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
        acc_distribution_comparison_condition_1_with_ego_df.to_csv(os.path.join(output_dir, f'acc_distribution_comparison_{condition_1_label}_with_ego_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
        speed_distribution_comparison_condition_2_with_ego_df.to_csv(os.path.join(output_dir, f'speed_distribution_comparison_{condition_2_label}_with_ego_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
        acc_distribution_comparison_condition_2_with_ego_df.to_csv(os.path.join(output_dir, f'acc_distribution_comparison_{condition_2_label}_with_ego_{mode}_{upstream_max_vehicles_count}.csv'), index=False)
def plot_system_eval_two_conditions(data_dic_condition_1, 
                     data_dic_condition_2,
                     vtMicroCoeff: pd.DataFrame,
                     VehicleSrcCoeff: pd.DataFrame,
                     condition_1_label = 'With Vehicle Dynamics',
                     condition_2_label = 'Without Vehicle Dynamics',
                     output_dir = r'.\\Results',
                     veh_no='system'
                     ):
    """
    Plot the system fuel efficiency (MPG) with two conditions.
    Args:
        data_dic_condition_1: dict[float, dict[str, pd.DataFrame]]
        data_dic_condition_2: dict[float, dict[str, pd.DataFrame]]
        vtMicroCoeff: pd.DataFrame
        VehicleSrcCoeff: pd.DataFrame
        condition_1_label: str
        condition_2_label: str
    """
    system_mpg_cpmf_dict = {}
    def get_round_trip_eval(penetration_rate, data_dic, vtMicroCoeff, VehicleSrcCoeff, veh_no='system'):
        eco_driving_folder = data_dic[penetration_rate]['path']
        refer_coord = [160, 735]
        step_length = 0.1
        # _, system_fuel_consume_vt_cpmf_liter_wb, system_fuel_consume_vt_micro_liter_wb, _, _, _, _, system_travel_dist2_m_wb, _ = get_traj_eval(path, 1, vtMicroCoeff, df_vehicle_src_coeff, refer_coord=refer_coord_wb,direction='WB',veh_no='system')
        (energy_evaluation_summary, system_fuel_consume_vt_cpmf_liter, system_fuel_consume_vt_micro_liter,
        system_energy_consume_tractive_kj, system_energy_consume_tractive_regen_kj, system_fuel_consume_yunli_icv_liter,
        system_energy_consume_yunli_ev_kj, system_travel_dist_m, system_travel_dist2_m, system_travel_dist_integration_m, system_travel_time_s) = \
        (get_traj_eval(eco_driving_folder, step_length, vtMicroCoeff, VehicleSrcCoeff, refer_coord, 'WB', veh_no))
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb + system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb + system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_wb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_wb) / 3.78541)
        # system_fuel_efficiency_cpmf_mpg = ((system_travel_dist2_m_eb) / 1609.34) / ((system_fuel_consume_vt_cpmf_liter_eb) / 3.78541)
        system_fuel_efficiency_cpmf_mpg = (system_travel_dist_integration_m/1609.34) / (system_fuel_consume_vt_cpmf_liter/3.78541)
        system_fuel_efficiency_micro_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_vt_micro_liter/3.78541)
        system_fuel_efficiency_tractive_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_tractive_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_icv_mpg = (system_travel_dist2_m/1609.34) / (system_fuel_consume_yunli_icv_liter/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)
        system_fuel_efficiency_yunli_ev_mpg = (system_travel_dist2_m/1609.34) / ((system_energy_consume_yunli_ev_kj/34200)/3.78541)  # (KJ to Liter, 1 Gasoline [Liter] = 1 × 34200 = 34200 Kilojoule)


        return system_fuel_efficiency_cpmf_mpg
    
    for penetration_rate, _ in data_dic_condition_1.items():

        system_fuel_efficiency_cpmf_mpg_condition_1 = get_round_trip_eval(
            penetration_rate,
            data_dic_condition_1,
            vtMicroCoeff,
            VehicleSrcCoeff,
            veh_no
        )
        print(f'{penetration_rate} {condition_1_label}: {system_fuel_efficiency_cpmf_mpg_condition_1}')
        system_fuel_efficiency_cpmf_mpg_condition_2 = get_round_trip_eval(
            penetration_rate,
            data_dic_condition_2,
            vtMicroCoeff,
            VehicleSrcCoeff,
            veh_no
        )
        print(f'{penetration_rate} {condition_2_label}: {system_fuel_efficiency_cpmf_mpg_condition_2}')
        system_mpg_cpmf_dict[penetration_rate] = {condition_1_label: system_fuel_efficiency_cpmf_mpg_condition_1,
                                                  condition_2_label: system_fuel_efficiency_cpmf_mpg_condition_2}
    
       # Visualization
    fig, ax = plt.subplots(1, 1, figsize=(12, 4))
    penetration_rates = list(system_mpg_cpmf_dict.keys())

    # Extract MPG values
    mpg_condition_1 = [system_mpg_cpmf_dict[pr][condition_1_label] for pr in penetration_rates]
    mpg_condition_2 = [system_mpg_cpmf_dict[pr][condition_2_label] for pr in penetration_rates]

    # Define Colors (Before vs. After Calibration)
    colors = {
        condition_1_label: "red",  
        condition_2_label: "blue"    
    }

    # Define Markers (With vs. Without Dynamics)
    markers = {
        condition_1_label: "o",   # Circle
        condition_2_label: "s" # Square
    }

    # Plot Data
    ax.plot(penetration_rates, mpg_condition_1, color=colors[condition_1_label], linestyle='--', marker=markers[condition_1_label], markersize=8, linewidth=2, label=condition_1_label)
    ax.plot(penetration_rates, mpg_condition_2, color=colors[condition_2_label], linestyle='-', marker=markers[condition_2_label], markersize=8, linewidth=2, label=condition_2_label)
    # set x ticks
    
    # Labels and title
    ax.set_xlabel('Market Penetration Rate (%)', fontsize=14, fontweight='bold')
    y_label = 'System Fuel Efficiency (MPG)' if veh_no == 'system' else 'Fuel Efficiency Improvement (%)'
    ax.set_ylabel(y_label, fontsize=14, fontweight='bold')
    # ax.set_title(f'System Fuel Efficiency (MPG) {condition_1_label} and {condition_2_label}', fontsize=16, fontweight='bold')

    # Grid and legend
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=12, loc='best')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'system_eval.png'))
    
    
    


def plot_ego_speed_acc_profile_comparison(data_dic_condition_1, 
                                data_dic_condition_2, 
                                condition_1_label = 'With Vehicle Dynamics',
                                condition_2_label = 'Without Vehicle Dynamics',
                                output_dir = r'.\\Results',
                                direction = 'WB',
                                trip_id = 0.0,
                                include_acceleration = True,
                                mode = 'over_distance'
                                ):
    assert mode in ['over_distance', 'over_time']
    penetration_rates = list(data_dic_condition_1.keys())
    for penetration_rate in penetration_rates:
        if penetration_rate < 0:
            continue
        
        path_condition_1 = data_dic_condition_1[penetration_rate]['path']
        path_condition_2 = data_dic_condition_2[penetration_rate]['path']
        ego_data_wb_condition_1 = pd.read_csv(os.path.join(path_condition_1, 'data_wb_part.csv'))
        ego_data_eb_condition_1 = pd.read_csv(os.path.join(path_condition_1, 'data_eb_part.csv'))
        ego_data_wb_condition_2 = pd.read_csv(os.path.join(path_condition_2, 'data_wb_part.csv'))
        ego_data_eb_condition_2 = pd.read_csv(os.path.join(path_condition_2, 'data_eb_part.csv'))
        tls_wb = data_dic_condition_1[penetration_rate]['sumo_signal_config']
        if direction == 'WB':
            ego_data_condition_1 = ego_data_wb_condition_1
            ego_data_condition_2 = ego_data_wb_condition_2
        elif direction == 'EB':
            ego_data_condition_1 = ego_data_eb_condition_1
            ego_data_condition_2 = ego_data_eb_condition_2
        else:
            raise ValueError(f"Invalid direction: {direction}")
        ego_data_condition_1['tripId'] = ego_data_condition_1['tripId'].astype(int).astype(str)
        ego_data_condition_2['tripId'] = ego_data_condition_2['tripId'].astype(int).astype(str)
        ego_data_condition_1 = ego_data_condition_1[ego_data_condition_1['tripId'] == trip_id]
        ego_data_condition_2 = ego_data_condition_2[ego_data_condition_2['tripId'] == trip_id]
        ego_data_condition_1 = ego_data_condition_1[ego_data_condition_1['id'] == 'ego']
        ego_data_condition_2 = ego_data_condition_2[ego_data_condition_2['id'] == 'ego']
        
        ego_data_condition_1['distance'] = abs(ego_data_condition_1['distance'] - ego_data_condition_1['distance'].iloc[0])
        ego_data_condition_2['distance'] = abs(ego_data_condition_2['distance'] - ego_data_condition_2['distance'].iloc[0])
        ego_data_condition_1['speed'] = ego_data_condition_1['DesiredSpeed']
        ego_data_condition_2['speed'] = ego_data_condition_2['DesiredSpeed']
        if include_acceleration:
            # calculate the acceleration
            ego_data_condition_1['acceleration'] = ego_data_condition_1['speed'].diff() / ego_data_condition_1['Time'].diff()
            ego_data_condition_2['acceleration'] = ego_data_condition_2['speed'].diff() / ego_data_condition_2['Time'].diff()
            # fill the acceleration with the first value
            ego_data_condition_1['acceleration'] = ego_data_condition_1['acceleration'].fillna(ego_data_condition_1['acceleration'].iloc[0])
            ego_data_condition_2['acceleration'] = ego_data_condition_2['acceleration'].fillna(ego_data_condition_2['acceleration'].iloc[0])
        colors = {
        condition_1_label: "red",  
        condition_2_label: "blue"    
        }
        fig, ax = plt.subplots(2 ,1, figsize=(12, 8)) if include_acceleration else plt.subplots(1 ,1, figsize=(12, 4))
        # plot the speed distance profile
        ax_speed = ax[0] if include_acceleration else ax
        if mode == 'over_distance':
            ax_speed.plot(ego_data_condition_1['distance'], ego_data_condition_1['speed'], label=condition_1_label, color=colors[condition_1_label], linewidth=2)
            ax_speed.plot(ego_data_condition_2['distance'], ego_data_condition_2['speed'], label=condition_2_label, color=colors[condition_2_label], linewidth=2)
        elif mode == 'over_time':
            ax_speed.plot(ego_data_condition_1['Time'], ego_data_condition_1['speed'], label=condition_1_label, color=colors[condition_1_label], linewidth=2)
            ax_speed.plot(ego_data_condition_2['Time'], ego_data_condition_2['speed'], label=condition_2_label, color=colors[condition_2_label], linewidth=2)
        # plot vertical lines indicating the tls
        if mode == 'over_distance':
            for tls_id in tls_wb['id'].unique():
                ax_speed.axvline(x=tls_wb[tls_wb['id'] == tls_id]['distance_wb'].values[0], color='grey', linestyle='--', linewidth=1)
                ax_speed.text(tls_wb[tls_wb['id'] == tls_id]['distance_wb'].values[0], 22, tls_wb[tls_wb['id'] == tls_id]['int_name'].values[0], fontsize=10, fontweight='bold', horizontalalignment='center')
        
        ax_speed.set_xlabel('Distance (m)' if mode == 'over_distance' else 'Time (s)', fontsize=14, fontweight='bold')
        ax_speed.set_ylabel('Speed (m/s)', fontsize=14, fontweight='bold')
        ax_speed.set_ylim(0, 25)
        # ax[0].set_title(f'Ego Speed Profile Comparison', fontsize=14, fontweight='bold')
        ax_speed.legend(fontsize=12, loc='upper left')
        # plot the acceleration distance profile
        if include_acceleration:
            if mode == 'over_distance':
                ax[1].plot(ego_data_condition_1['distance'], ego_data_condition_1['acceleration'], label=condition_1_label, color=colors[condition_1_label], linewidth=2)
                ax[1].plot(ego_data_condition_2['distance'], ego_data_condition_2['acceleration'], label=condition_2_label, color=colors[condition_2_label], linewidth=2)
            elif mode == 'over_time':
                ax[1].plot(ego_data_condition_1['Time'], ego_data_condition_1['acceleration'], label=condition_1_label, color=colors[condition_1_label], linewidth=2)
                ax[1].plot(ego_data_condition_2['Time'], ego_data_condition_2['acceleration'], label=condition_2_label, color=colors[condition_2_label], linewidth=2)
            ax[1].set_xlabel('Distance (m)' if mode == 'over_distance' else 'Time (s)', fontsize=14, fontweight='bold')
            ax[1].set_ylabel('Acceleration (m/s^2)', fontsize=14, fontweight='bold')
            # ax[1].set_title(f'Ego Acceleration Profile Comparison', fontsize=14, fontweight='bold')
            ax[1].legend(fontsize=12, loc='lower right')
            plt.suptitle(f'Ego Speed and Acceleration Profile Comparison MPR: {penetration_rate:.0f}%', fontsize=14, fontweight='bold')
        else:
            plt.suptitle(f'Ego Speed Profile Comparison MPR: {penetration_rate:.0f}%', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'ego_speed{'_acc' if include_acceleration else ''}_profile_comparison_{direction}_{penetration_rate}_{mode}.png'))
        
        