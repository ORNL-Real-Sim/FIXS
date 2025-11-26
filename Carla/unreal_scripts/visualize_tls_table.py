import matplotlib.pyplot as plt
import math
import pandas as pd

def visualize_tls_positions(tls_data):
    plt.figure(figsize=(10, 8))
    
    for index, entry in tls_data.iterrows():
        x = entry["x"]
        y = entry["y"]
        heading = entry["heading"]
        # Arrow parameters
        heading = math.degrees(heading)
        heading = (heading + 360) % 360
        
        heading = math.radians(heading)
        dx = math.cos(heading) * 1.5  # Arrow length
        dy = math.sin(heading) * 1.5

        # Plot arrow and label
        plt.arrow(x, y, dx, dy, head_width=0.5, head_length=0.7, fc='green', ec='green')
        # plt.text(x, y + 0.5, f"J{entry['junction_id']}-L{entry['link_id']}", fontsize=9, ha='center', color='black')
        # mark the heading in text
        heading = math.degrees(heading)
        plt.text(x, y + 0.5, f"{heading:.2f}", fontsize=9, ha='center', color='black')

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Traffic Light Positions and Headings")
    plt.grid(True)
    plt.axis("equal")
    # plt.show()
    plt.savefig('tls_positions.png')

if __name__ == "__main__":
    tls_table_path = 'test_scenarios/Town01_with_ego_type_as_blueprint/traffic_light_table.csv'
    tls_table = pd.read_csv(tls_table_path)
    visualize_tls_positions(tls_table)