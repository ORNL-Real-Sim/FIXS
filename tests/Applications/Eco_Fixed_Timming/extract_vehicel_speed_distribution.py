import xml.etree.ElementTree as ET
import pandas as pd
import os
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import gaussian_kde

def extract_two_vehicles_speed(fcd_file, vehicle_id, ego_id):
    """
    从fcd.xml文件中提取两辆车的速度数据
    
    参数:
        fcd_file: fcd.xml文件路径
        vehicle_id: 目标车辆ID
        ego_id: ego车辆ID
    
    返回:
        df_vehicle: 目标车辆数据
        df_ego: ego车辆数据
    """
    results_dir = "Experiments_Sumo/Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR_1/Results"
    os.makedirs(results_dir, exist_ok=True)
    
    records_vehicle = []
    records_ego = []

    # 使用iterparse流式解析大文件
    for event, elem in ET.iterparse(fcd_file, events=("end",)):
        if elem.tag == "timestep":
            time = float(elem.get("time"))
            for veh in elem.findall("vehicle"):
                veh_id = veh.get("id")
                speed = float(veh.get("speed"))
                
                if veh_id == vehicle_id:
                    records_vehicle.append([time, vehicle_id, speed])
                elif veh_id == ego_id:
                    records_ego.append([time, ego_id, speed])
            
            elem.clear()

    df_vehicle = pd.DataFrame(records_vehicle, columns=["time", "id", "speed"])
    df_ego = pd.DataFrame(records_ego, columns=["time", "id", "speed"])
    
    print(f"✅ 数据提取完成:")
    print(f"   - {vehicle_id}: {len(df_vehicle)} 条记录")
    print(f"   - {ego_id}: {len(df_ego)} 条记录")

    return df_vehicle, df_ego


def plot_speed_density_kde(df_vehicle, df_ego, vehicle_id, ego_id, output_dir):
    """
    绘制两辆车的速度密度分布图（使用平滑的KDE）
    
    参数:
        df_vehicle: 目标车辆数据
        df_ego: ego车辆数据
        vehicle_id: 目标车辆ID
        ego_id: ego车辆ID
        output_dir: 输出目录
    """
    plt.figure(figsize=(10, 6))
    
    # 提取速度数据
    speeds_vehicle = df_vehicle["speed"].values
    speeds_ego = df_ego["speed"].values
    
    # 为目标车辆创建KDE
    if len(speeds_vehicle) > 1:
        kde_vehicle = gaussian_kde(speeds_vehicle, bw_method='scott')
        # 创建速度范围用于绘图（从0开始，不会延伸到负值）
        speed_range_vehicle = np.linspace(0, max(speeds_vehicle) * 1.1, 1000)
        density_vehicle = kde_vehicle(speed_range_vehicle)
        
        plt.plot(speed_range_vehicle, density_vehicle, 
                color='blue', linewidth=2, label=f'Vehicle {vehicle_id} (baseline)')
    
    # 为ego车辆创建KDE
    if len(speeds_ego) > 1:
        kde_ego = gaussian_kde(speeds_ego, bw_method='scott')
        # 创建速度范围用于绘图（从0开始，不会延伸到负值）
        speed_range_ego = np.linspace(0, max(speeds_ego) * 1.1, 1000)
        density_ego = kde_ego(speed_range_ego)
        
        plt.plot(speed_range_ego, density_ego, 
                color='orange', linewidth=2, linestyle='--', label=f'Ego vehicle')
    
    # 设置图表属性
    plt.xlabel("Speed (m/s)", fontsize=12)
    plt.ylabel("Density (smoothed KDE)", fontsize=12)
    plt.title("Speed Density Distribution Comparison", fontsize=14, fontweight='bold')
    plt.legend(fontsize=10, loc='best')
    plt.grid(True, alpha=0.3)
    
    # 设置x轴从0开始，确保不会显示负值
    plt.xlim(left=0)
    plt.ylim(bottom=0)
    
    plt.tight_layout()
    
    # 保存图片
    output_fig = os.path.join(output_dir, f"{vehicle_id}_vs_{ego_id}_speed_density_kde.png")
    plt.savefig(output_fig, dpi=300, bbox_inches='tight')
    print(f"📊 图片已保存到: {output_fig}")
    
    plt.show()


# ================= 主程序 =================
if __name__ == "__main__":
    # fcd.xml文件路径
    fcd_file = "Experiments_Sumo/Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR_1/0%_10Hz_wo_vehDyn_29211.2/fcd.xml"
    
    # 车辆ID
    vehicle_id = "4.85"   # 目标车辆
    ego_id = "ego"        # ego车辆
    
    # 输出目录
    output_dir = "Experiments_Sumo/Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR_1/Results"
    
    # 提取速度数据
    df_vehicle, df_ego = extract_two_vehicles_speed(fcd_file, vehicle_id, ego_id)
    
    # 绘制速度密度分布图
    plot_speed_density_kde(df_vehicle, df_ego, vehicle_id, ego_id, output_dir)