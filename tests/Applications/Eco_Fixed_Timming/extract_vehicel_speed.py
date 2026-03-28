import xml.etree.ElementTree as ET
import pandas as pd
import os
import matplotlib.pyplot as plt

def extract_two_vehicles_speed(fcd_file, vehicle_id, ego_id):
    """
    Extracting two vehicles (time, id, speed) from fcd.xml and save it to particular folder (Results).

    
    parameters:
        fcd_file: fcd.xml path
        vehicle_id: target vehicle ID
        ego_id: ego vehicle ID
    """
    # output dic
    results_dir = "Experiments_Sumo\Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3\MPR_10\Results"
    os.makedirs(results_dir, exist_ok=True)
    
    # Two output file
    output_file_vehicle = os.path.join(results_dir, f"{vehicle_id}_speed.csv")
    output_file_ego = os.path.join(results_dir, f"{ego_id}_speed.csv")

    records_vehicle = []
    records_ego = []

    # Using iterparse to stream parse large files
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

    # Saving results
    df_vehicle = pd.DataFrame(records_vehicle, columns=["time", "id", "speed"])
    df_ego = pd.DataFrame(records_ego, columns=["time", "id", "speed"])
    
    # df_vehicle.to_csv(output_file_vehicle, index=False)
    # df_ego.to_csv(output_file_ego, index=False)
    
    print(f"✅ Extraction is finished:")
    print(f"   - {vehicle_id}: {len(df_vehicle)} records -> {output_file_vehicle}")
    print(f"   - {ego_id}: {len(df_ego)} records -> {output_file_ego}")

    return df_vehicle, df_ego

# ================= Example =================
if __name__ == "__main__":
    # fcd.xml file path
    fcd_file = "Experiments_Sumo/Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR_10/0%_10Hz_with_vehDyn_SP_test0302/fcd_1.xml"
    
    # Vehcle ID
    vehicle_id = ""   # Target vehicle
    ego_id = "ego"        # ego vehicle
    
    # extract speed data
    df_vehicle, df_ego = extract_two_vehicles_speed(fcd_file, vehicle_id, ego_id)

    # Plot speed curve
    plt.figure(figsize=(10, 6))
    
    # target vehicle_blue
    plt.plot(df_vehicle["time"], df_vehicle["speed"], 
             color='blue', linewidth=2, label=f'Vehicle {vehicle_id}')
    
    # ego vehicle_red
    plt.plot(df_ego["time"], df_ego["speed"], 
             color='orange', linewidth=2, label=f'Ego {ego_id}')
    
    plt.xlabel("Time [s]", fontsize=12)
    plt.ylabel("Speed [m/s]", fontsize=12)
    plt.title(f"Speed-Time Curve Comparison", fontsize=14, fontweight='bold')
    plt.legend(fontsize=10, loc='best')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Save picture
    output_fig = os.path.join(
        "Experiments_Sumo/Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3/MPR_10/Results",
        f"{vehicle_id}_vs_{ego_id}_speed_comparison.png"
    )
    plt.savefig(output_fig, dpi=300, bbox_inches='tight')
    print(f"📊 Saving picture to: {output_fig}")
    
    plt.show()