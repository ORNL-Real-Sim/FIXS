import math
import xml.etree.ElementTree as ET
import os
import pandas as pd
from shapely.geometry import LineString, Polygon, Point

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

# ------------------------------
# Helpers for 2D/3D coordinates
# ------------------------------
def parse_point_token(token):
    """Parse 'x,y' or 'x,y,z' -> tuple of floats (2D or 3D). Return None if malformed."""
    parts = token.split(',')
    try:
        if len(parts) == 2:
            x, y = map(float, parts)
            return (x, y)
        elif len(parts) == 3:
            x, y, z = map(float, parts)
            return (x, y, z)
    except Exception:
        return None
    return None

def to_xy(seq):
    """Map list of 2D/3D points to list of (x,y)."""
    return [(p[0], p[1]) for p in seq]

def get_z(pt):
    """Get z from a 2D/3D point; default 0.0 if missing."""
    return pt[2] if isinstance(pt, (list, tuple)) and len(pt) >= 3 else 0.0

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

def parse_sumo_tls_from_netxml(
    sumo_net_file,
    offset_forward=0.0,
    apply_linkwise_offset=True,
    extend_back=50.0,
    default_lane_width=3.2
):

    tree = ET.parse(sumo_net_file)
    root = tree.getroot()

    # 1) Junction polygons (for type="traffic_light")
    junction_shapes = {}
    junction_xyz = {}  # (x,y,z) if present on the junction element (strings here; converted in SumoTLSGroup)
    for jn in root.iter('junction'):
        if jn.get('type') == 'traffic_light':
            shape_str = jn.get('shape')
            if shape_str:
                coords_raw = []
                for tok in shape_str.strip().split():
                    pt = parse_point_token(tok)
                    if pt is not None:
                        coords_raw.append(pt)
                # Use only (x, y) for Polygon, ignore z if present
                if len(coords_raw) >= 3:
                    junction_shapes[jn.get('id')] = Polygon(to_xy(coords_raw))
            # store x, y, z if provided (SUMO may omit z)
            x = jn.get('x')
            y = jn.get('y')
            z = jn.get('z')  # may be None
            junction_xyz[jn.get('id')] = (x, y, z)

    # 2) Lane shapes & widths
    lane_shape = {}   # laneId -> list of 2D/3D tuples
    lane_width = {}   # laneId -> float
    for edge in root.iter('edge'):
        for ln in edge.iter('lane'):
            lid = ln.get('id')
            shp = ln.get('shape')
            if shp:
                pts = []
                for tok in shp.strip().split():
                    pt = parse_point_token(tok)
                    if pt is not None:
                        pts.append(pt)
                if len(pts) >= 2:
                    lane_shape[lid] = pts
            w = ln.get('width')
            lane_width[lid] = float(w) if w is not None else default_lane_width

    # 3) Connections by TLS and linkIndex (this replaces traci.trafficlight.getControlledLinks)
    #    build: tls_links[tls_id][linkIndex] -> list of (inLane, outLane, viaLane)
    tls_links = {}
    for conn in root.iter('connection'):
        tls_id = conn.get('tl')
        if not tls_id:
            continue  # not controlled by a traffic light
        link_idx = conn.get('linkIndex')
        if link_idx is None:
            link_idx = "0"

        # from/to lane IDs come as indices; reconstruct lane IDs as "<edge>_<index>"
        from_edge = conn.get('from')
        to_edge   = conn.get('to')
        from_lane_idx = conn.get('fromLane')
        to_lane_idx   = conn.get('toLane')
        via_lane = conn.get('via')  # usually present for internal links

        if from_edge is None or from_lane_idx is None:
            continue
        in_lane = f"{from_edge}_{from_lane_idx}"
        out_lane = f"{to_edge}_{to_lane_idx}" if (to_edge is not None and to_lane_idx is not None) else None

        tls_links.setdefault(tls_id, {})
        tls_links[tls_id].setdefault(int(link_idx), [])
        tls_links[tls_id][int(link_idx)].append((in_lane, out_lane, via_lane))

    # 4) Build groups & compute positions
    traffic_light_groups = {}
    in_lane_links_total = {}
    in_lane_links_placed = {}

    # Pre-count links per inLane for lateral spacing
    for _, linkIndexGroups in tls_links.items():
        for _, triplets in linkIndexGroups.items():
            for inLane, _, _ in triplets:
                in_lane_links_total[inLane] = in_lane_links_total.get(inLane, 0) + 1
                in_lane_links_placed.setdefault(inLane, 0)

    for tls_id, linkIndexGroups in tls_links.items():
        # ensure we have a group object
        if tls_id not in traffic_light_groups:
            jx, jy, jz = junction_xyz.get(tls_id, (None, None, None))
            traffic_light_groups[tls_id] = SumoTLSGroup(tls_id, jx, jy, jz)

        junction_poly = junction_shapes.get(tls_id)

        for link_idx, triplets in sorted(linkIndexGroups.items()):
            for inLane, outLane, viaLane in triplets:
                shape_pts = lane_shape.get(inLane)
                if not shape_pts or len(shape_pts) < 2:
                    # cannot compute heading or intersection
                    continue

                # Use XY for geometry, keep Z from the lane end if present
                shape_xy = to_xy(shape_pts)
                (x1, y1) = shape_xy[-2]
                (x2, y2) = shape_xy[-1]
                end_z = get_z(shape_pts[-1])  # Z from lane end if available

                # last segment heading points "into" the junction
                heading = math.atan2(y1 - y2, x1 - x2)
                tx, ty = math.cos(heading), math.sin(heading)  # tangent (toward lane end)
                nx, ny = -ty, tx                               # left-hand normal

                # extend backwards from lane end (reverse of travel) in XY
                reversed_extension = (x2 - extend_back * tx, y2 - extend_back * ty)

                # IMPORTANT: build a proper XY list for Shapely
                extended_coords_xy = list(shape_xy) + [reversed_extension]
                extended_line = LineString(extended_coords_xy)

                # intersect with junction polygon (if any)
                inter_points = []
                if junction_poly:
                    inter = extended_line.intersection(junction_poly)
                    if isinstance(inter, Point):
                        inter_points = [(inter.x, inter.y)]
                    elif inter.geom_type == 'MultiPoint':
                        inter_points = [(p.x, p.y) for p in inter.geoms]
                    elif inter.geom_type == 'LineString':
                        inter_points = list(inter.coords)

                # pick farthest intersection point from lane end; else, use lane end
                if inter_points:
                    end_pt = Point(shape_xy[-1])
                    intersection_pt = max(inter_points, key=lambda p: Point(p).distance(end_pt))
                    base_x, base_y = intersection_pt
                    base_z = end_z  # no Z from intersection; keep lane end Z
                else:
                    base_x, base_y = x2, y2
                    base_z = end_z

                # If lane had no Z, fall back to junction Z; if missing, 0.0
                if base_z == 0.0:
                    base_z = traffic_light_groups[tls_id].z if traffic_light_groups[tls_id].z is not None else 0.0

                # lateral offset across multiple links from the same inLane
                L = in_lane_links_total.get(inLane, 1)
                i = in_lane_links_placed.get(inLane, 0)
                this_lane_width = lane_width.get(inLane, default_lane_width)
                d_perp = -compute_perp_offsets(this_lane_width, L, i) if apply_linkwise_offset else 0.0

                # forward offset along the tangent
                bx = offset_forward * tx
                by = -offset_forward * ty  # keep your original sign convention

                px = base_x + bx + d_perp * nx
                py = base_y + by + d_perp * ny
                pz = base_z  # keep Z constant (no vertical offset)

                traffic_light_groups[tls_id].traffic_lights.setdefault(link_idx, []).append(
                    SumoTLS(link_idx, px, py, pz, heading, (inLane, outLane, viaLane))
                )
                in_lane_links_placed[inLane] = i + 1

    return traffic_light_groups, junction_shapes

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
    sumo_net_file = 'test_scenarios\MLK\MLK_final_elevation.net.xml'
    output_path = 'test_scenarios\MLK'

    traffic_light_groups, _ = parse_sumo_tls_from_netxml(
        sumo_net_file,
        offset_forward=0.0,
        apply_linkwise_offset=True,
        extend_back=50.0,
        default_lane_width=3.2
    )
    traffic_light_groups_df = tls_groups_to_df(traffic_light_groups)
    traffic_light_groups_df.to_csv(
        os.path.join(output_path, 'traffic_light_table.csv'),
        index=False
    )
