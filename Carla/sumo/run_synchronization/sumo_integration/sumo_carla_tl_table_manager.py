import csv
import math

import carla
import traci


class SumoCarlaTLTableManager(object):
    """
    Maps CARLA traffic light actors to SUMO (junction_id, link_id) rows from a CSV
    and mirrors SUMO signal states into those CARLA actors.
    """
    def __init__(self, tl_table_path, world, offset_x=0.0, offset_y=0.0, max_match_dist=50.0):
        self.rows, self._tls_table = self.load_tl_table(tl_table_path)
        self._tls_table = self.build_actor_mapping(
            world,
            self.rows,
            self._tls_table,
            offset_x=offset_x,
            offset_y=offset_y,
            max_match_dist=max_match_dist)

    @staticmethod
    def load_tl_table(csv_path):
        """
        Expected columns: junction_id, link_id, x, y, z, heading
        Returns:
          rows: list[(junction_id:str, link_id:int, x:float, y:float)]
          table: dict[junction_id][link_id] -> dict(x,y,z,heading, actor=None)
        """
        rows = []
        table = {}
        with open(csv_path, newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            required = {"junction_id", "link_id", "x", "y"}
            if not required.issubset(set(reader.fieldnames or [])):
                raise ValueError("CSV missing required columns. Found: {}".format(reader.fieldnames))

            for tl_head in reader:
                junc_id = str(tl_head["junction_id"])
                link_id = int(tl_head["link_id"])
                x = float(tl_head["x"])
                y = float(tl_head["y"])
                z = float(tl_head.get("z", 0.0))
                heading = float(tl_head.get("heading", 0.0))

                rows.append((junc_id, link_id, x, y))
                table.setdefault(junc_id, {})[link_id] = {
                    "x": x,
                    "y": y,
                    "z": z,
                    "heading": heading,
                    "actor": None
                }

        return rows, table

    @staticmethod
    def carla_to_sumo_xy(loc, offset_x=0.0, offset_y=0.0):
        return (loc.x + offset_x, -loc.y + offset_y)

    @staticmethod
    def find_closest(rows, sx, sy):
        best = None
        best_d2 = float("inf")
        for junc_id, link_id, x, y in rows:
            dx = x - sx
            dy = y - sy
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best = (junc_id, link_id, math.sqrt(d2))
        return best

    @staticmethod
    def map_sumo_char_to_carla_state(char):
        if char in ("r", "u"):
            return carla.TrafficLightState.Red
        if char == "y":
            return carla.TrafficLightState.Yellow
        if char in ("G", "g", "s"):
            return carla.TrafficLightState.Green
        if char in ("O", "o"):
            return carla.TrafficLightState.Off
        return carla.TrafficLightState.Unknown

    def build_actor_mapping(self, world, rows, table, offset_x=0.0, offset_y=0.0, max_match_dist=50.0):
        """
        For each CARLA TL actor, map to the nearest (junction_id, link_id) row in the CSV.
        max_match_dist guards against bad matches in SUMO XY space.
        """
        tls = world.get_actors().filter("traffic.traffic_light*")
        print("[CARLA] Found {} traffic light actors".format(len(tls)))

        mapped = 0
        for tl in tls:
            tl.freeze(True)
            loc = tl.get_transform().location
            sx, sy = self.carla_to_sumo_xy(loc, offset_x, offset_y)
            hit = self.find_closest(rows, sx, sy)
            if hit is None:
                continue

            junc_id, link_id, dist = hit
            if dist > max_match_dist:
                continue

            table[junc_id][link_id]["actor"] = tl
            mapped += 1

        print("[MAP] Mapped {}/{} CARLA TL actors to SUMO (junction_id, link_id)".format(
            mapped, len(tls)))
        return table

    def apply_sumo_states_to_carla(self):
        """
        Fetch the SUMO state string for each junction and apply the per-link
        character to the mapped CARLA traffic light actor.
        """
        for junction_id, link_map in self._tls_table.items():
            try:
                state = traci.trafficlight.getRedYellowGreenState(junction_id)
            except traci.TraCIException:
                continue

            for link_id, entry in link_map.items():
                tl_actor = entry["actor"]
                if tl_actor is None:
                    continue
                if link_id < 0 or link_id >= len(state):
                    continue
                tl_actor.set_state(self.map_sumo_char_to_carla_state(state[link_id]))
