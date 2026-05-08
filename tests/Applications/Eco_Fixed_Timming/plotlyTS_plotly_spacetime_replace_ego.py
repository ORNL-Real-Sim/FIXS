import os
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from cav_casestudy.SUMO.spaceTimePlotWithSignals import read_trajectory_xml, trajectory_process, read_signal_xml

def plot_wb_two_vehicle_with_signals(
    folder_466,
    folder_ego,
    output_dir,
    condition_label="4.85 vs Ego (WB with signals)"
):
    refer_coord_wb = [1390, 225]  # WB参考点
    eb_lanes = ['-312', '-293', '-297', '-288', '-2881', '-286', '-302', '-3221', '-322', '-313',
                '-284', '-2841', '-328', '-304']
    wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300',
                '-2851', '-285', '-290', '-298', '-295']
    relative_time = 28800

    # === 读取 4.66 车辆轨迹 ===
    traj_466 = read_trajectory_xml(os.path.join(folder_466, "fcd.xml"), refer_coord=refer_coord_wb)
    traj_466 = trajectory_process(traj_466, eb_lanes, wb_lanes, relative_time)
    traj_466 = traj_466[(traj_466['id'] == 'ego') & (traj_466['direction'] == 'WB')].reset_index(drop=True)

    # === 读取 ego 车辆轨迹 ===
    traj_ego = read_trajectory_xml(os.path.join(folder_ego, "fcd.xml"), refer_coord=refer_coord_wb)
    traj_ego = trajectory_process(traj_ego, eb_lanes, wb_lanes, relative_time)
    traj_ego = traj_ego[(traj_ego['id'].str.lower() == 'ego') & (traj_ego['direction'] == 'WB')].reset_index(drop=True)

    # === 读取信号灯数据 ===
    sumo_signal_config = os.path.join(folder_466, "sumoSignalConfig_26.csv")
    sumo_signal_config = pd.read_csv(sumo_signal_config, index_col=0)
    sumo_signal_config['id'] = sumo_signal_config['id'].astype(str)
    sumo_signal_config['distance_wb'] = np.sqrt((sumo_signal_config['x'] - refer_coord_wb[0])**2 +
                                                (sumo_signal_config['y'] - refer_coord_wb[1])**2)
    sumo_signal_config = sumo_signal_config[sumo_signal_config['approach_direction'] == 'WB']

    sumo_signal_result = read_signal_xml(os.path.join(folder_466, "signal_result.xml"))
    sumo_signal_result['time'] = sumo_signal_result['time'].astype(float) - relative_time

    # === 创建图形 ===
    fig = go.Figure()

    # 画 4.66 车
    fig.add_trace(go.Scatter(
        x=traj_466['time'],
        y=traj_466['distance'],
        mode='lines',
        name="ego without Dyn (WB)",
        line=dict(width=3, color='blue'),
        hovertemplate="Veh 4.85<br>Time=%{x:.1f}s<br>Dist=%{y:.1f}m<extra></extra>"
    ))

    # 画 ego 车
    fig.add_trace(go.Scatter(
        x=traj_ego['time'],
        y=traj_ego['distance'],
        mode='lines',
        name="ego with Dyn (WB)",
        line=dict(width=3, color='orange'),
        hovertemplate="Ego<br>Time=%{x:.1f}s<br>Dist=%{y:.1f}m<extra></extra>"
    ))

    # === 添加信号灯状态线 ===
    for _, row in sumo_signal_config.iterrows():
        sc_id = row['id']
        x_loc = row['distance_wb']
        movement_indexes = eval(row['movement_index'])

        signal_changes = sumo_signal_result[sumo_signal_result['id'] == sc_id].reset_index(drop=True)
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

    # === 更新坐标轴 ===
    if 'int_name' in sumo_signal_config.columns:
        signal_labels = sumo_signal_config[['int_name', 'distance_wb']].drop_duplicates()
        fig.update_yaxes(
            tickvals=signal_labels['distance_wb'],
            ticktext=signal_labels['int_name']
        )

    fig.update_layout(
        title=f"<b>{condition_label}</b>",
        xaxis=dict(
            title=dict(
                text="Simulation Time [s]",
                font=dict(size=50)   # 横坐标标题字体大小
            ),
            tickfont=dict(size=45),  # 横坐标刻度字体大小
            range=[0, 600]
        ),
        yaxis=dict(
            title=dict(
                text="Distance along WB [m]",
                font=dict(size=50)   # 纵坐标标题字体大小
            ),
            tickfont=dict(size=45)   # 纵坐标刻度字体大小
        ),
        template="plotly_white",
        height=900,
        width=1600,
        hovermode="closest",
        showlegend=True
    )


    output_html = os.path.join(output_dir, f"{condition_label.replace(' ', '_')}.html")
    fig.write_html(output_html)
    print(f"✅ Saved WB comparison plot with signals: {output_html}")


# === 调用函数 ===
plot_wb_two_vehicle_with_signals(
    folder_466=r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_10\0%_10Hz_wo_vehDyn_477",
    folder_ego=r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_10\0%_10Hz_w_vehDyn_477",
    output_dir=r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_10\Results",
    condition_label="Without Dyn vs With Dyn"
)
