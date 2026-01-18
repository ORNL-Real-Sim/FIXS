import xml.etree.ElementTree as ET
import pandas as pd
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import gaussian_kde

def extract_vehicle_speed(fcd_file, target_id):
    """
    从fcd.xml文件中提取某一辆车的速度数据
    """
    records = []
    for event, elem in ET.iterparse(fcd_file, events=("end",)):
        if elem.tag == "timestep":
            time = float(elem.get("time"))
            for veh in elem.findall("vehicle"):
                veh_id = veh.get("id")
                if veh_id == target_id:
                    speed = float(veh.get("speed"))
                    records.append([time, veh_id, speed])
            elem.clear()
    df = pd.DataFrame(records, columns=["time", "id", "speed"])
    print(f"✅ {target_id}: {len(df)} 条记录")
    return df


def plot_speed_density_kde(df_vehicle, df_ego, vehicle_id, ego_id, output_dir):
    """
    绘制两辆车的速度密度分布图（使用平滑的KDE）
    """
    plt.figure(figsize=(10, 6))

    speeds_vehicle = df_vehicle["speed"].values
    speeds_ego = df_ego["speed"].values

    # baseline车辆 KDE
    if len(speeds_vehicle) > 1:
        kde_vehicle = gaussian_kde(speeds_vehicle, bw_method='scott')
        speed_range_vehicle = np.linspace(0, max(speeds_vehicle) * 1.1, 1000)
        density_vehicle = kde_vehicle(speed_range_vehicle)
        plt.plot(speed_range_vehicle, density_vehicle,
                 color='blue', linewidth=2, label=f'Vehicle {vehicle_id} (baseline)')

    # ego车辆 KDE
    if len(speeds_ego) > 1:
        kde_ego = gaussian_kde(speeds_ego, bw_method='scott')
        speed_range_ego = np.linspace(0, max(speeds_ego) * 1.1, 1000)
        density_ego = kde_ego(speed_range_ego)
        plt.plot(speed_range_ego, density_ego,
                 color='orange', linewidth=2, linestyle='--', label=f'Ego vehicle')

    # 图表属性
    plt.xlabel("Speed (m/s)", fontsize=16)
    plt.ylabel("Density (KDE)", fontsize=16)
    plt.title("Speed Density Distribution", fontsize=18, fontweight='bold')
    plt.legend(fontsize=14, loc='best')
    plt.grid(True, alpha=0.3)
    plt.xlim(left=0)
    plt.ylim(bottom=0)
    plt.tight_layout()

    # 保存图片
    os.makedirs(output_dir, exist_ok=True)
    output_fig = os.path.join(output_dir, f"{vehicle_id}_vs_{ego_id}_speed_density_kde.png")
    plt.savefig(output_fig, dpi=300, bbox_inches='tight')
    print(f"📊 图片已保存到: {output_fig}")

    plt.show()


# ================= 主程序 =================
if __name__ == "__main__":
    # 两个 fcd.xml 文件路径
    fcd_vehicle = r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\0%_10Hz_wo_vehDyn_baseline\fcd.xml"
    fcd_ego = r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\0%_10Hz_wo_vehDyn_4.85\fcd.xml"

    # 车辆ID
    vehicle_id = "4.85"
    ego_id = "ego"

    # 输出目录
    output_dir = r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\Results"

    # 分别提取速度数据
    df_vehicle = extract_vehicle_speed(fcd_vehicle, vehicle_id)
    df_ego = extract_vehicle_speed(fcd_ego, ego_id)

    # 绘制速度密度分布图
    plot_speed_density_kde(df_vehicle, df_ego, vehicle_id, ego_id, output_dir)
