import pandas as pd
import xml.etree.ElementTree as ET
from matplotlib import pyplot as plt
import numpy as np
import seaborn as sns
from spaceTimePlotWithSignals import read_signal_xml
import itertools
import os


def get_phase_duration(sumo_signal, sumoSignalConfig):
    phase_duration_summary = []
    for sc in sumoSignalConfig.id.unique():
        print(sc)
        phase_list = sumoSignalConfig[sumoSignalConfig['id'] == sc]['name'].values.tolist()
        for phase in phase_list:
            print(phase)
            movement_indexes = eval(
                sumoSignalConfig[(sumoSignalConfig['id'] == sc) & (sumoSignalConfig['name'] == phase)][
                    'movement_index'].values[0])

            # read the signal changes data for sc and phase
            signal_changes_tem = sumo_signal[
                (sumo_signal['id'] == sc) & (sumo_signal['name'].str.contains(str(phase)))].reset_index(drop=True)
            signal_changes_tem['target_phase_state'] = signal_changes_tem['state'].str[
                                                       movement_indexes[0]: movement_indexes[0] + 1]
            signal_changes_tem = signal_changes_tem[
                signal_changes_tem['target_phase_state'] != signal_changes_tem['target_phase_state'].shift()]
            signal_changes_tem['time'] = signal_changes_tem['time'].astype(float)
            signal_changes_tem['next_time'] = signal_changes_tem['time'].shift(-1)
            signal_changes_tem = signal_changes_tem[signal_changes_tem['next_time'].notna()]
            signal_changes_tem['duration'] = signal_changes_tem['next_time'].astype(float) - signal_changes_tem['time'].astype(float)
            signal_changes_tem['duration'] = signal_changes_tem['duration'].astype(int)
            signal_changes_tem = signal_changes_tem[signal_changes_tem['target_phase_state'] == 'G']
            signal_changes_tem['target_phase'] = phase
            signal_changes_tem = signal_changes_tem[['id', 'target_phase', 'time', 'duration']]
            phase_duration_summary.append(signal_changes_tem)

    phase_duration_summary = pd.concat(phase_duration_summary, ignore_index=True)
    phase_duration_summary = phase_duration_summary.sort_values(by=['id', 'time']).reset_index(drop=True)

    return phase_duration_summary


def plot_phase_duration(phase_duration_summary, sumoSignalConfig):
    for sc in phase_duration_summary.id.unique():
        # simulated signal plot
        # phase_duration_summary = phase_duration_summary[phase_duration_summary['time'] <= 30000]
        phase_duration_summary_tem = phase_duration_summary[phase_duration_summary['id'] == sc]
        phase_duration_summary_tem['cycle'] = np.where(phase_duration_summary_tem['target_phase'] == 2, 1, 0)
        phase_duration_summary_tem['cycle'] = phase_duration_summary_tem['cycle'].cumsum()

        print(sc)
        # plot
        fig = plt.figure()
        fig.set_size_inches(30, 10)
        plt.rcParams.update({'font.size': 20})
        custom_palette = {1: 'orange', 2: 'green', 3: 'red', 4: 'purple', 5: 'gray', 6: 'blue', 7: 'olive', 8: 'pink'}

        sns.barplot(data=phase_duration_summary_tem, x='cycle', y='duration', hue='target_phase', palette=custom_palette)
        # plt.xticks(ticks=range(28985, 33000, 100))  # Show ticks at intervals of 2
        plt.title("Green Duration Bar Plot (Int {})".format(sc))
        plt.xlabel("Signal Cycle")
        plt.ylabel("Green Duration (Seconds)")
        plt.grid(axis='y', linestyle='--', alpha=0.7)

        plt.legend(title="Phase")
        plt.savefig(r'SignalEvaluation\phase_duration_int_{}'.format(sc), dpi=100, bbox_inches='tight')


        # signal setting plot
        sumoSignalConfig_tem = sumoSignalConfig[sumoSignalConfig['id'] == sc]
        sumoSignalConfig_tem = pd.melt(sumoSignalConfig_tem, id_vars=['name'], value_vars=['minDur', 'maxDur'],
                            var_name='min_max', value_name='green_duration')
        sumoSignalConfig_tem['name'] = sumoSignalConfig_tem['name'].astype(int)

        fig = plt.figure()
        fig.set_size_inches(10, 10)
        plt.rcParams.update({'font.size': 20})
        custom_palette = {1: 'orange', 2: 'green', 3: 'red', 4: 'purple', 5: 'gray', 6: 'blue', 7: 'olive', 8: 'pink'}

        sns.barplot(data=sumoSignalConfig_tem, x='min_max', y='green_duration', hue='name', palette=custom_palette)

        # plt.xticks(ticks=range(28985, 33000, 100))  # Show ticks at intervals of 2
        plt.title("Green Duration Settings Bar Plot (Int {})".format(sc))
        plt.xlabel("MinMax")
        plt.ylabel("Green Duration (Seconds)")
        plt.grid(axis='y', linestyle='--', alpha=0.7)

        plt.legend(title="Phase")
        plt.savefig(r'SignalEvaluation\phase_settings_int_{}'.format(sc), dpi=100, bbox_inches='tight')

        print('Done', sc)


if __name__ == "__main__":
    # define a reference point coordinates at the leftmost point to calculate the distance over the corridor
    refer_coord = [230, 705]

    simulation_folder = 'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3'

    # read sumoSignalConfig_26
    sumoSignalConfig = pd.read_csv(os.path.join(simulation_folder, 'sumoSignalConfig.csv'), index_col=0)
    sumoSignalConfig['id'] = sumoSignalConfig['id'].astype(str)

    # Calculate the relative distance for each point
    sumoSignalConfig['distance'] = np.sqrt((sumoSignalConfig['x'] - refer_coord[0]) ** 2 + (sumoSignalConfig['y'] - refer_coord[1]) ** 2)

    sumo_signal = read_signal_xml(os.path.join(simulation_folder, 'signal_result.xml'))

    phase_duration_summary = get_phase_duration(sumo_signal, sumoSignalConfig)

    plot_phase_duration(phase_duration_summary, sumoSignalConfig)

    phase_duration_summary_agg = phase_duration_summary.groupby(by=['id', 'target_phase'], as_index=False).agg({'duration': ['min', 'max', 'median', 'mean', 'sum']})
    phase_duration_summary_agg.to_csv(os.path.join(simulation_folder, 'phase_duration_summary_agg.csv'))
    print('test')
