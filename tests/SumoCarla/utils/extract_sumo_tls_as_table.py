import math
import xml.etree.ElementTree as ET
import traci
import os
import pandas as pd

class SumoTLS:
    def __init__(self, id, x, y, z, heading, link):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading
        self.link = link

class SumoTLSGroup:
    def __init__(self, tl_id, x, y, z):
        # this corresponds to the junction id in SUMO
        self.id = tl_id
        self.x = float(x) if x is not None else 0.0
        self.y = float(y) if y is not None else 0.0
        self.z = float(z) if z is not None else 0.0
        self.traffic_lights = {}
    
    def print_tls_info(self):
        for tls in self.traffic_lights.values():
            print(tls.id, tls.x, tls.y, tls.z, tls.heading)
    
def compute_perp_offsets(lane_width, count, idx):
    """
    Given:
      - lane_width: total width of that inLane
      - count: total number of markers to place across this lane
      - idx: zero‐based index (0 .. count-1) for the current marker
    Returns offset_along_normal so that markers run from left edge → right edge.
    If count == 1, returns 0.0 (center).
    """
    if count == 1:
        return 0.0
    span = lane_width
    delta = span / (count * 2.0)
    # center them about 0: from -span/2 to +span/2
    return -span/2 + (2 * idx + 1) * delta

def get_tls_info(tls_ids):
    tls_df = pd.DataFrame(columns=['id', 'state'])
    for tls_id in tls_ids:
        tls_df = tls_df.append({
            'id': tls_id,
            'state': traci.trafficlight.getRedYellowGreenState(tls_id),}, ignore_index=True)
    return tls_df

def parse_sumo_tls(sumo_net_file, sumo_config_file, offset_forward = 2.0, apply_linkwise_offset= True):
    os.system(f'start sumo-gui -c {sumo_config_file} --remote-port 12345 --step-length 1 --begin 0 --end 800')

    traci.init(port=12345, host='127.0.0.1')
    tree = ET.parse(sumo_net_file)
    root = tree.getroot()
    traffic_light_groups = {}
    # get the junctions
    edges = {}
    in_lane_links_total = {}
    in_lane_links_placed = {}
    traffic_light_controllers = {}
    for edge in root.iter('edge'):
        edges[edge.get('id')] = edge

    for controller in root.iter('tlLogic'):
        traffic_light_controllers[controller.get('id')] = controller
    for junction in root.iter('junction'):
        # get the traffic lights
        if junction.get('type') == 'traffic_light':
            junction_id = junction.get('id')
            links = traci.trafficlight.getControlledLinks(junction_id)
            if junction_id not in traffic_light_groups:
                traffic_light_groups[junction_id] = SumoTLSGroup(junction_id, junction.get('x'), junction.get('y'), junction.get('z'))
            
            for idx, linkGroup in enumerate(links):
                for inLane, outLane, viaLane in linkGroup:
                    if inLane not in in_lane_links_total:
                        in_lane_links_total[inLane] = 0
                        in_lane_links_placed.setdefault(inLane, 0)
                    in_lane_links_total[inLane] += 1
            
            for idx, linkGroup in enumerate(links):
                for (inLane, outLane, viaLane) in linkGroup:
                    # 2a) lane‐end geometry & heading
                    shape = traci.lane.getShape(inLane)
                    (x1, y1), (x2, y2) = shape[-2], shape[-1]
                    # the heading is pointing from the first point to the second point (backwards to the lane)
                    heading = math.atan2(y1 - y2, x1 - x2)
                    tx, ty = math.cos(heading), math.sin(heading)
                    # left‐normal:
                    nx, ny = -ty, tx

                    # 2b) fetch total count L and current index i
                    L = in_lane_links_total[inLane]
                    i = in_lane_links_placed[inLane]

                    # 2c) compute perpendicular offset
                    lane_width = traci.lane.getWidth(inLane)
                    if apply_linkwise_offset:
                        d_perp = compute_perp_offsets(lane_width, L, i)
                    else:
                        d_perp = 0.0

                    # or e.g. offset 0.5 to move 0.5m downstream
                    bx, by = offset_forward * tx, -offset_forward * ty

                    # 2d) place the final marker point
                    px = x2 + bx + d_perp * nx
                    py = y2 + by + d_perp * ny

                    # 2e) store it and increment the counter for this lane
                    traffic_light_groups[junction_id].traffic_lights.setdefault(idx, []).append(
                        SumoTLS(idx, px, py, 0, heading, (inLane, outLane, viaLane))
                    )
                in_lane_links_placed[inLane] += 1

    
    return traffic_light_groups

def tls_groups_to_df(traffic_light_groups):
    data = []
    for junction_id, group in traffic_light_groups.items():
        for link_id, link in group.traffic_lights.items():
            for tls in link:
                data.append({
                    'junction_id': junction_id,
                    'link_id': link_id,
                    'x': tls.x,
                    'y': tls.y,
                    'z': tls.z,
                    'heading': tls.heading,
                })
    return pd.DataFrame(data, columns=['junction_id', 'link_id', 'x', 'y', 'z', 'heading'])

if __name__ == "__main__":
    sumo_net_file = 'test_scenarios/Town01_with_ego_type_as_blueprint/Town01.net.xml'
    sumo_config_file = 'test_scenarios/Town01_with_ego_type_as_blueprint/Town01.sumocfg'
    sumo_scnario_path = 'test_scenarios/Town01_with_ego_type_as_blueprint'
    traffic_light_groups = parse_sumo_tls(sumo_net_file, sumo_config_file, apply_linkwise_offset=True)
    traffic_light_groups_df = tls_groups_to_df(traffic_light_groups)
    traffic_light_groups_df.to_csv(os.path.join(sumo_scnario_path, 'traffic_light_table.csv'), index=False)
