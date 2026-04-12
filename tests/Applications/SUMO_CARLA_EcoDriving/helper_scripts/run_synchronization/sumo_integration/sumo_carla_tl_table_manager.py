import csv
import typing
import traci
import carla
import math

class SumoCarlaTLTableManager:
    def __init__(self, tl_table_path, world, args):
        self.rows, self._tls_table = self.load_tl_table(tl_table_path)
        self._tls_table = self.build_actor_mapping(
            world, self.rows, self._tls_table,
            offset_x=args.offset_x, offset_y=args.offset_y,
            max_match_dist=args.max_match_dist)
        

    def load_tl_table(self, csv_path):
        """
        Expected columns: junction_id, link_id, x, y, z, heading
        Returns:
          rows: list[(junction_id:str, link_id:int, x:float, y:float)]
          table: dict[junction_id][link_id] -> dict(x,y,z,heading, actor=None)
        """
        rows = []
        table = {}
        with open(csv_path, newline="", encoding="utf-8") as file:
            r = csv.DictReader(file)
            required = {"junction_id", "link_id", "x", "y"}
            if not required.issubset(set(r.fieldnames or [])):
                raise ValueError(f"CSV missing required columns. Found: {r.fieldnames}")

            for tl_head in r:
                junc_id = str(tl_head["junction_id"])
                link_id = int(tl_head["link_id"])
                x = float(tl_head["x"])
                y = float(tl_head["y"])
                z = float(tl_head.get("z", 0.0))
                heading = float(tl_head.get("heading", 0.0))

                rows.append((junc_id, link_id, x, y))
                table.setdefault(junc_id, {})[link_id] = {"x": x, "y": y, "z": z, "heading": heading, "actor": None}

        return rows, table
    
    def build_actor_mapping(self, world, rows, table, offset_x=0.0, offset_y=0.0, max_match_dist=50.0):
        """
        For each CARLA TL actor, map to nearest (junction_id, link_id) row in the CSV.
        max_match_dist guards against bad matches (meters in SUMO XY space).
        """
        tls = world.get_actors().filter("traffic.traffic_light*")
        print(f"[CARLA] Found {len(tls)} traffic light actors")

        mapped = 0
        for tl in tls:
            tl.freeze(True)
            loc = tl.get_transform().location
            sx, sy = self.carla_to_sumo_xy(loc, offset_x, offset_y)
            hit = self.find_closest(rows, sx, sy)
            if hit is None:
                continue
            j, k, dist = hit
            if dist > max_match_dist:
                # probably bad match
                continue
            table[j][k]["actor"] = tl
            mapped += 1

        print(f"[MAP] Mapped {mapped}/{len(tls)} CARLA TL actors to SUMO (junction_id, link_id)")
        return table
    
    def apply_sumo_states_to_carla(table):
        """
        For each junction_id in our mapping table:
          - fetch SUMO state string via TraCI
          - apply per-link char to the mapped CARLA actor
        """
        for junction_id, link_map in table.items():
            try:
                state = traci.trafficlight.getRedYellowGreenState(junction_id)
            except traci.TraCIException:
                # junction_id not in SUMO TL list (or SUMO not ready)
                continue
            
            for link_id, entry in link_map.items():
                tl = entry["actor"]
                if tl is None:
                    continue
                if link_id < 0 or link_id >= len(state):
                    continue
                tl.set_state(map_sumo_char_to_carla_state(state[link_id]))


    @staticmethod
    def carla_to_sumo_xy(loc, offset_x=0.0, offset_y=0.0):
        return (loc.x + offset_x, -loc.y + offset_y)