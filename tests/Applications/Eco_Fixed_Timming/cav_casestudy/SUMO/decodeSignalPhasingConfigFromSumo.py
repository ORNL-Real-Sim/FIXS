import os.path

import pandas as pd
import xml.etree.ElementTree as ET
import numpy as np


simulation_folder = 'Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3'
# Parse the XML file
signal_file_path = os.path.join(simulation_folder, 'updated_signal.xml')
tree = ET.parse(signal_file_path)
root = tree.getroot()

# Extract data from XML into a list of dictionaries
data_list = []

for tls in root.findall('tlLogic'):
    for param in tls.findall('phase'):
        print(param.attrib)
        tem_dict = param.attrib
        tem_dict.update(tls.attrib)
        data_list.append(tem_dict)

# Convert the list of dictionaries to a dataframe
sumoSignalConfig = pd.DataFrame(data_list)
sumoSignalConfig['movement_index'] = sumoSignalConfig['state'].apply(lambda x: [i for i, c in enumerate(x) if c == 'G'])
sumoSignalConfig['movement_index_start'] = sumoSignalConfig['movement_index'].str[0]
sumoSignalConfig['movement_index_end'] = sumoSignalConfig['movement_index'].str[-1]

# only keep phase 2 and 6
sumoSignalConfig_26 = sumoSignalConfig[sumoSignalConfig['name'].isin(['2', '6'])].reset_index(drop=True)
sumoSignalConfig_26['movement_index_start_max'] = sumoSignalConfig_26.groupby('id')['movement_index_start'].transform('max')

sumoSignalConfig_26['approach_direction'] = np.where(sumoSignalConfig_26['movement_index_start'] < sumoSignalConfig_26['movement_index_start_max'], 'WB', 'EB')

##################################################
###### parse the signal location information#####
#################################################
# Parse the XML file
net_file_path = os.path.join(simulation_folder, 'chatt.net.xml')

root_net = ET.parse(net_file_path).getroot()

# Extract data from XML into a list of dictionaries
net_data_list = []

tl_ids = ['2', '3', '10', '8', '9', '12']

for int in root_net.findall('junction'):
    print(int.attrib)
    if int.attrib['id'] in tl_ids:
        net_data_list.append(int.attrib)
    else:
        continue

intersection_net = pd.DataFrame(net_data_list)

##############################################################
###### join intersection location with configuration #####
#############################################################
sumoSignalConfig = sumoSignalConfig.join(intersection_net[['id', 'x', 'y']].set_index('id'), how='left', on='id')
sumoSignalConfig.to_csv(os.path.join(simulation_folder, 'sumoSignalConfig.csv'))

sumoSignalConfig_26 = sumoSignalConfig_26.join(intersection_net[['id', 'x', 'y']].set_index('id'), how='left', on='id')
sumoSignalConfig_26.to_csv(os.path.join(simulation_folder, 'sumoSignalConfig_26.csv'))
print('Test')
