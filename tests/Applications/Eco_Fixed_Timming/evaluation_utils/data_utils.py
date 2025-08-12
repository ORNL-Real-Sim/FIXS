import os
import typing
import pandas as pd
import numpy as np

def get_data_dics(path: str, verbose: bool = False) -> tuple[dict[float, dict[str,]], dict[float, dict[str, ]]]:
    root_dir = os.getcwd()
    working_dir = os.path.join(root_dir, path)
    seting_dirs = os.listdir(working_dir)

    data_dic_with_dynamics = typing.DefaultDict(dict)
    data_dic_without_dynamics = typing.DefaultDict(dict)
    for idx, setting_dir in enumerate(seting_dirs):

        if verbose:
            print(f'Processing {setting_dir}')
        penetration_rate = float(setting_dir.split('_')[0][:-1])

        sumo_signal_config = pd.read_csv(os.path.join(working_dir, setting_dir, '.\\sumoSignalConfig_26.csv'), index_col=0)
        sumo_signal_config['id'] = sumo_signal_config['id'].astype(str)
        sumo_signal_config['int_name'] = sumo_signal_config['id'].map(
            {'12': 'Amin', '9': 'I-75 S', '8': 'I-75 N', '10': 'Napier', '3': 'Lifestyle', '2': 'Gunbarrel'})
        # Calculate the relative distance for each point
        refer_coord_wb = [1380, 225]
        refer_coord_eb = [230, 705]
        sumo_signal_config['distance_wb'] = np.sqrt((sumo_signal_config['x'] - refer_coord_wb[0]) ** 2 + (sumo_signal_config['y'] - refer_coord_wb[1]) ** 2)
        sumo_signal_config['distance_eb'] = np.sqrt((sumo_signal_config['x'] - refer_coord_eb[0]) ** 2 + (sumo_signal_config['y'] - refer_coord_eb[1]) ** 2)
        if os.path.exists(os.path.join(working_dir, setting_dir, '.\\ego_profile.csv')):
            ego_profile = pd.read_csv(os.path.join(working_dir, setting_dir, '.\\ego_profile.csv'))
            ego_profile['Time'] = ego_profile['Time'].astype(float) - 185
        else:
            ego_profile = None
        
        if '_D' in setting_dir:
            data_dic_with_dynamics[penetration_rate]['ego_profile'] = ego_profile
            data_dic_with_dynamics[penetration_rate]['sumo_signal_config'] = sumo_signal_config
            data_dic_with_dynamics[penetration_rate]['path'] = os.path.join(working_dir, setting_dir)
        else:
            data_dic_without_dynamics[penetration_rate]['ego_profile'] = ego_profile
            data_dic_without_dynamics[penetration_rate]['sumo_signal_config'] = sumo_signal_config
            data_dic_without_dynamics[penetration_rate]['path'] = os.path.join(working_dir, setting_dir)
        # if '_D' not in setting_dir and '_E' not in setting_dir:
        #     # base model
        #     data_dic_with_dynamics[-1.0]['ego_profile'] = ego_profile
        #     data_dic_with_dynamics[-1.0]['sumo_signal_config'] = sumo_signal_config
        #     data_dic_with_dynamics[-1.0]['path'] = os.path.join(working_dir, setting_dir)
        #     data_dic_without_dynamics[-1.0]['ego_profile'] = ego_profile
        #     data_dic_without_dynamics[-1.0]['sumo_signal_config'] = sumo_signal_config
        #     data_dic_without_dynamics[-1.0]['path'] = os.path.join(working_dir, setting_dir)
    return dict(sorted(data_dic_with_dynamics.items())), dict(sorted(data_dic_without_dynamics.items()))