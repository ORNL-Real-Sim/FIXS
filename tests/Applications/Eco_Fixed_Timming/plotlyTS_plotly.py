import os
import warnings
warnings.filterwarnings('ignore')
import pandas as pd
import matplotlib as mpl
import numpy as np


OUTPUT_DIR = r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\Results"
os.makedirs(OUTPUT_DIR, exist_ok=True)
SUMO_START_TIME = 28800
# Increase font sizes globally
mpl.rcParams['axes.titlesize'] = 40
mpl.rcParams['axes.labelsize'] = 60
mpl.rcParams['xtick.labelsize'] = 30
mpl.rcParams['ytick.labelsize'] = 45
mpl.rcParams['legend.fontsize'] = 30
mpl.rcParams['figure.titlesize'] = 30
# line width
mpl.rcParams['lines.linewidth'] = 4

def get_data(folder_path: str, verbose: bool = False) -> dict[str, dict[str,]]:

    if verbose:
        print(f"Processing folder: {folder_path}")

    folder = r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\0%_10Hz_baseline_Noego"
    file_path = os.path.join(folder, "sumoSignalConfig_26.csv")
    print(file_path)
    sumo_signal_config = pd.read_csv(file_path, index_col=0)

    sumo_signal_config['id'] = sumo_signal_config['id'].astype(str)
    sumo_signal_config['int_name'] = sumo_signal_config['id'].map(
        {'12': 'Amin', '9': 'I-75 S', '8': 'I-75 N', '10': 'Napier', '3': 'Lifestyle', '2': 'Gunbarrel'})
    # Calculate the relative distance for each point
    refer_coord_wb = [1380, 225]
    refer_coord_eb = [230, 705]
    sumo_signal_config['distance_wb'] = np.sqrt((sumo_signal_config['x'] - refer_coord_wb[0]) ** 2 + (sumo_signal_config['y'] - refer_coord_wb[1]) ** 2)
    sumo_signal_config['distance_eb'] = np.sqrt((sumo_signal_config['x'] - refer_coord_eb[0]) ** 2 + (sumo_signal_config['y'] - refer_coord_eb[1]) ** 2)
    setting_name = os.path.basename(folder_path.rstrip(os.sep))

    data_dic = {
        setting_name: {
            "sumo_signal_config": sumo_signal_config,
            "path": folder_path
        }
    }

    return data_dic

data_dic_full = get_data(r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\0%_10Hz_baseline_Noego", verbose=True)
# data_dic_with_dynamics_after_calibration, data_dic_without_dynamics_after_calibration = get_data_dics('.\\Experiments_Sumo\\Debug')



# data_dic_with_dynamics_after_calibration = {
#     -1.0: data_dic_with_dynamics_after_calibration[-1.0],
#     0.0: data_dic_with_dynamics_after_calibration[0.0],
#     # 10.0: data_dic_with_dynamics_after_calibration[10.0],
#     # 20.0: data_dic_with_dynamics_after_calibration[20.0],
#     # 50.0: data_dic_with_dynamics_after_calibration[50.0],
#     # 100.0: data_dic_with_dynamics_after_calibration[100.0]
# }
# data_dic_without_dynamics_after_calibration = {
#     -1.0: data_dic_without_dynamics_after_calibration[-1.0],
#     0.0: data_dic_without_dynamics_after_calibration[0.0],
#     # 10.0: data_dic_without_dynamics_after_calibration[10.0],
#     # 20.0: data_dic_without_dynamics_after_calibration[20.0],
#     # 50.0: data_dic_without_dynamics_after_calibration[50.0],
#     # 100.0: data_dic_without_dynamics_after_calibration[100.0]
# }



    # 🚨 去掉最外层的 key，只保留 value
data_dic = list(data_dic_full.values())[0]
#----------------------------------------------------------------
#----------------------------------------------------------------


import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib as mpl
import plotly.graph_objects as go
from dash import Dash, dcc, html, Input, Output
import seaborn as sns
import pandas as pd
import os
import numpy as np
# from evaluation_utils.metadata_utils import compare_distributions
from cav_casestudy.SUMO.VehicleEnergyEvaluation import get_traj_eval
from evaluation_utils.metadata_utils import get_energy_consumption, assign_trip_id
from cav_casestudy.SUMO.spaceTimePlotWithSignals import  read_trajectory_xml, read_signal_xml, trajectory_process, get_exclude_turn_traj


def plot_space_time_diagram_two_conditions(data_dic,
                                           condition_label = '0%_With_Noego',
                                           output_dir = r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR\Results",
                                           target_vehicle_id=None):
    """
    生成交互式交通流时空图：
    - 鼠标悬停显示车辆ID、时间、距离
    - 点击轨迹显示车辆进入/离开时间
    - 点击后高亮选中轨迹（蓝色）
    """

    # === 基础参数 ===
    refer_coord_wb = [1390, 225] #Eastern start point
    refer_coord_eb = [160, 735]
    relative_time = 28800
    wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300',
                '-2851', '-285', '-290', '-298', '-295']
    eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313',
                '-284', '-2841', '-328', '-304']

    path_condition = data_dic['path']
    sumo_signal_condition = read_signal_xml(os.path.join(path_condition, 'signal_result.xml'))
    sumo_signal_condition['time'] = sumo_signal_condition['time'].astype(float) - relative_time
    sumoSignalConfig_condition = data_dic['sumo_signal_config']

    trajectory_data = read_trajectory_xml(os.path.join(path_condition, 'fcd.xml'), refer_coord=refer_coord_wb)
    trajectory_data = trajectory_process(trajectory_data, eb_lanes, wb_lanes, relative_time)
    trajectory_data_wb = trajectory_data[trajectory_data['direction'] == 'WB'].reset_index(drop=True)




    # trajectory_data_eb = trajectory_data[trajectory_data['direction'] == 'EB'].reset_index(drop=True)
    # trajectory_data_wb = get_exclude_turn_traj(trajectory_data_wb, exclude_turning_traj=True)

    # === 创建初始图 ===
    fig = go.Figure()

    for veh_id, group in trajectory_data_wb.groupby('id'):
        veh_type = group['type'].iloc[0] if 'type' in group.columns else 'Unknown'
        veh_id_str = str(veh_id).lower()

            # ✅ 识别 ego 车
        if veh_id_str in ['ego']:
            # 使用 assign_trip_id 拆分段落
            group = assign_trip_id(group.copy(), distance_threshold=100)

            for _, seg in group.groupby('tripId'):
                enter_time = seg['time'].min()
                exit_time = seg['time'].max()

                fig.add_trace(go.Scatter(
                    x=seg['time'],
                    y=seg['distance'],
                    mode='lines',
                    name=f"{veh_id}_trip",
                    line=dict(width=3.0, color='green'),
                    hovertemplate=(
                        "<b>EGO Vehicle</b><br>"
                        "<b>Trip:</b> %{customdata[2]}<br>"
                        "<b>Time:</b> %{x:.1f}s<br>"
                        "<b>Distance:</b> %{y:.1f} m<extra></extra>"
                    ),
                    customdata=np.stack([seg['id'], seg['type'], seg['tripId']], axis=-1),
                    meta={'veh_id': veh_id, 'enter': enter_time, 'exit': exit_time},
                    opacity=0.9
                ))

        # ✅ --- 普通车辆逻辑 ---
        else:
            color = 'lightgray' if veh_type == 'HDV' else 'gray'
            line_w = 1.0

            if str(veh_id) == str(target_vehicle_id):  # target vehicle 比灰色略深，更明显
                color = 'black'  
                line_w = 2.0




            enter_time = group['time'].min()
            exit_time = group['time'].max()

            fig.add_trace(go.Scatter(
                x=group['time'],
                y=group['distance'],
                mode='lines',
                name=str(veh_id),
                line=dict(width=line_w, color=color),
                hovertemplate=(
                    "<b>Vehicle ID:</b> %{customdata[0]}<br>"
                    "<b>Time:</b> %{x:.1f}s<br>"
                    "<b>Distance:</b> %{y:.1f} m<br>"
                    "<b>Type:</b> %{customdata[1]}<extra></extra>"
                ),
                customdata=np.stack([group['id'], group['type']], axis=-1),
                meta={'veh_id': veh_id, 'enter': enter_time, 'exit': exit_time},
                opacity=0.6  # 降低透明度，让绿色ego更突出
            ))


    # === 添加信号灯状态线 ===
    sumoSignalConfig_condition = sumoSignalConfig_condition[sumoSignalConfig_condition['approach_direction'] == 'WB']
    sumoSignalConfig_condition['distance'] = sumoSignalConfig_condition['distance_wb']

    for _, row in sumoSignalConfig_condition.iterrows():
        sc_id = row['id']
        x_loc = row['distance']
        movement_indexes = eval(row['movement_index'])

        signal_changes = sumo_signal_condition[sumo_signal_condition['id'] == sc_id].reset_index(drop=True)
        signal_changes['target_phase_state'] = signal_changes['state'].str[movement_indexes[0]: movement_indexes[0] + 1]
        signal_changes = signal_changes[signal_changes['target_phase_state'] != signal_changes['target_phase_state'].shift()]
        signal_changes['target_phase_state'] = signal_changes['target_phase_state'].map(
            {'G': 'green', 'g': 'green', 'y': 'yellow', 'r': 'red', 's': 'red'}
        )

        prev_time = signal_changes['time'].iloc[0]
        prev_color = signal_changes['target_phase_state'].iloc[0]

        for _, row2 in signal_changes.iloc[1:].iterrows():
            fig.add_shape(
                type="line",
                x0=prev_time,
                x1=row2['time'],
                y0=x_loc,
                y1=x_loc,
                line=dict(color=prev_color, width=6)
            )
            prev_time = row2['time']
            prev_color = row2['target_phase_state']

             # === 显示红绿灯名称 ===
    if 'int_name' in sumoSignalConfig_condition.columns:
        signal_labels = sumoSignalConfig_condition[['int_name', 'distance']].drop_duplicates()
        fig.update_yaxes(
            tickvals=signal_labels['distance'],
            ticktext=signal_labels['int_name']
        )   

    fig.update_layout(
        title=f"<b>{condition_label}</b>",
        xaxis=dict(
            title=dict(
                text="Simulation Time [s]",
                font=dict(size=40)     # ✅ 改这里
            ),
            tickfont=dict(size=30),    # ✅ x轴刻度字体
            
            range=[100, trajectory_data_wb['time'].max()]  # ✅ 从300开始    
        
        
        ),
        yaxis=dict(
            title=dict(
                text="Distance along WB [m]",
                font=dict(size=40)     # ✅ 改这里
            ),
            tickfont=dict(size=30)     # ✅ y轴刻度字体
        ),
        template="plotly_white",
        height=1000,
        width=1800,
        hovermode="closest",
        showlegend=False  # ✅ 禁用图例显示
    )

    output_html = os.path.join(output_dir, f"{condition_label.replace(' ', '_')}.html")
    fig.write_html(output_html)
    print(f"✅ Interactive HTML saved: {output_html}")

    # === 启动 Dash App ===
    app = Dash(__name__)
    app.layout = html.Div([
        html.H3(condition_label),
        html.Div([
            dcc.Graph(id='space-time-graph', figure=fig, style={'width': '75vw', 'display': 'inline-block'}),
            html.Div(id='click-info', style={
                'border': '1px solid #ccc', 'padding': '15px',
                'display': 'inline-block', 'verticalAlign': 'top',
                'width': '20vw', 'marginLeft': '10px'
            })
        ])
    ])

    # === 点击事件回调 ===
    @app.callback(
        [Output('space-time-graph', 'figure'),
            Output('click-info', 'children')],
        Input('space-time-graph', 'clickData')
    )
    def highlight_vehicle(clickData):
        updated_fig = fig  # 复制当前图
        if clickData is None:
            # 没点击，恢复初始状态
            for trace in updated_fig.data:
                trace.line.color = 'lightgray' if 'HDV' in trace.name else 'gray'
                trace.line.width = 1.5
                trace.opacity = 0.7
            return updated_fig, "点击任意车辆轨迹以查看详细信息。"

        # 点击数据点
        point = clickData['points'][0]
        trace_idx = point['curveNumber']
        trace_meta = updated_fig.data[trace_idx].meta

        # 高亮选中轨迹
        for i, trace in enumerate(updated_fig.data):
            if i == trace_idx:
                trace.line.color = 'blue'
                trace.line.width = 2
                trace.opacity = 1.0
            else:
                trace.opacity = 0.3
                trace.line.width = 0.8

        # 显示信息框
        info_box = html.Div([
            html.H4("🚗 车辆信息"),
            html.P(f"车辆ID: {trace_meta['veh_id']}"),
            html.P(f"进入时间: {trace_meta['enter']:.1f} s"),
            html.P(f"离开时间: {trace_meta['exit']:.1f} s"),
        ])
        return updated_fig, info_box

    print("✅ 打开浏览器访问 http://127.0.0.1:8050 查看交互式时空图。")
    app.run(debug=True)


 

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

def assign_trip_id(df: pd.DataFrame, distance_threshold: float):
        for veh_id, group in df.groupby('id'):
            group['tripId'] = (abs(group['distance'].diff()) > distance_threshold).cumsum()
            df.loc[df['id'] == veh_id, 'tripId'] = group['tripId']

        return df


plot_space_time_diagram_two_conditions (data_dic,
                                       condition_label = '0%_With_Noego',
                                        output_dir = OUTPUT_DIR,
                                        target_vehicle_id='') 




