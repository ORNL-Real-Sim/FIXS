import xml.etree.ElementTree as ET
import pandas as pd
import os
import matplotlib.pyplot as plt

def extract_vehicle_speed(fcd_file, target_id):
    """
    从 fcd.xml 中提取某一辆车的速度数据。
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
    return df

# ================= 主程序 =================
if __name__ == "__main__":
    # 文件路径
    fcd_466_path = r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\0%_10Hz_wo_vehDyn_baseline\fcd.xml"
    fcd_ego_path = r"C:\Users\yusun\Projects\XIL_Oct\tests\Applications\Eco_Fixed_Timming\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\0%_10Hz_wo_vehDyn_4.85\fcd.xml"

    # 车辆 ID
    vehicle_id = "4.85"
    ego_id = "ego"

    # 提取数据
    df_vehicle = extract_vehicle_speed(fcd_466_path, vehicle_id)
    df_ego = extract_vehicle_speed(fcd_ego_path, ego_id)

    # 画图
    plt.figure(figsize=(12, 7))
    plt.plot(df_vehicle["time"], df_vehicle["speed"], color='blue', linewidth=2.5, label=f'Vehicle {vehicle_id}')
    plt.plot(df_ego["time"], df_ego["speed"], color='orange', linewidth=2.5, label=f'Ego Vehicle')

    plt.xlabel("Time [s]", fontsize=20)
    plt.ylabel("Speed [m/s]", fontsize=20)
    plt.title("Speed-Time Curve", fontsize=24, fontweight='bold')
    plt.legend(fontsize=18)
    plt.grid(True, alpha=0.3)
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    plt.tight_layout()

    # 保存图像
    output_dir = r".\Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_2\Results"
    os.makedirs(output_dir, exist_ok=True)
    output_fig = os.path.join(output_dir, f"{vehicle_id}_vs_{ego_id}_speed.png")
    plt.savefig(output_fig, dpi=300, bbox_inches='tight')
    print(f"📊 图像已保存到: {output_fig}")

    plt.show()
