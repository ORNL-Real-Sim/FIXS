from __future__ import annotations

import argparse
import ast
import math
from functools import reduce
from pathlib import Path
from typing import Sequence
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    import seaborn as sns
except ImportError:  # pragma: no cover - optional dependency
    sns = None


_TRAJECTORY_CACHE: dict[tuple[str, str, tuple[float | str, float | str]], dict[str, np.ndarray]] = {}


TIME_WINDOW: tuple[float, float] | None = (29000, 29400)
REFERENCE_COORD = (1390.0, 225.0)
WB_LANES = wb_lanes = ['-2801', '-280', '-307', '-327', '-3271', '-281', '-315', '-3151', '-321', '-300', '-2851', '-285', '-290', '-298', '-295']
SIGNAL_STATE_COLOR = {
    "G": "#2ecc71",
    "g": "#2ecc71",
    "Y": "#f1c40f",
    "y": "#f1c40f",
    "R": "#e74c3c",
    "r": "#e74c3c",
}
SIGNAL_STATE_LABEL = {
    "G": "Green",
    "g": "Green",
    "Y": "Yellow",
    "y": "Yellow",
    "R": "Red",
    "r": "Red",
}

SIMULATION_RESULTS_DIR = Path("simulation_results")
DEFAULT_EXPERIMENT_SETTINGS = ["MachE_HIL_Eco", "MachE_Simulink_Eco_Kp10", "MachE_Simulink_Eco_Kp100", "Sumo_Eco"]
EGO_VEHICLE_ID = "ego"
OUTPUT_ROOT = SIMULATION_RESULTS_DIR / "plots"
SIGNAL_CONFIG_PATH = Path("sumoSignalConfig_26.csv")


def _sort_entry_labels(labels: set[str]) -> list[str]:
    def sort_key(label: str) -> tuple[int, float | str]:
        try:
            return (0, float(label))
        except ValueError:
            return (1, label)

    return sorted(labels, key=sort_key)


def sanitize_entry_label(label: str) -> str:
    return label.replace(".", "_").replace(":", "_").replace(" ", "_")


def create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare ego vehicle trajectories across multiple experiment settings."
    )
    parser.add_argument(
        "--settings",
        nargs="+",
        help=f"Experiment setting folders to include (default: {' '.join(DEFAULT_EXPERIMENT_SETTINGS)})",
    )
    parser.add_argument(
        "--time-window",
        nargs=2,
        type=float,
        metavar=("START", "END"),
        help="Limit plots to the specified time window in seconds (inclusive).",
    )
    parser.add_argument(
        "--no-time-window",
        action="store_true",
        help="Ignore any configured time window and use the full simulation range.",
    )
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=SIMULATION_RESULTS_DIR,
        help=f"Root directory containing simulation results (default: {SIMULATION_RESULTS_DIR})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Directory to write plots (default: <results-dir>/plots).",
    )
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = create_arg_parser()
    args = parser.parse_args(argv)

    if args.time_window and args.no_time_window:
        parser.error("Cannot use --time-window together with --no-time-window.")

    if args.time_window and args.time_window[0] >= args.time_window[1]:
        parser.error("Time window START must be strictly less than END.")

    return args


def discover_simulation_cases(
    results_root: Path,
    settings: list[str] | None = None,
) -> tuple[list[str], dict[str, dict[str, Path]]]:
    if not results_root.exists():
        raise FileNotFoundError(f"Simulation results directory not found: {results_root}")

    if settings:
        setting_dirs: list[Path] = []
        for setting_name in settings:
            candidate = results_root / setting_name
            if candidate.is_dir():
                setting_dirs.append(candidate)
            else:
                print(f"Warning: experiment setting '{setting_name}' missing under {results_root}")
    else:
        setting_dirs = [path for path in results_root.iterdir() if path.is_dir()]

    if not setting_dirs:
        raise RuntimeError(f"No experiment settings discovered under {results_root}")

    entry_maps: dict[str, dict[str, Path]] = {}
    for setting_dir in setting_dirs:
        entries = {
            subdir.name: subdir
            for subdir in setting_dir.iterdir()
            if subdir.is_dir()
        }
        if not entries:
            print(f"Warning: no entry time folders found for setting '{setting_dir.name}'")
        entry_maps[setting_dir.name] = entries

    entry_sets = [set(entry_map.keys()) for entry_map in entry_maps.values() if entry_map]
    if not entry_sets:
        raise RuntimeError("No overlapping entry times found across experiment settings.")

    common_entry_times = reduce(lambda acc, s: acc & s, entry_sets[1:], entry_sets[0])
    if not common_entry_times:
        raise RuntimeError("Experiment settings do not share any common ego entry times.")

    ordered_entries = _sort_entry_labels(common_entry_times)
    cases: dict[str, dict[str, Path]] = {}
    for entry in ordered_entries:
        cases[entry] = {
            setting_name: entry_maps[setting_name][entry]
            for setting_name in entry_maps
            if entry in entry_maps[setting_name]
        }

    if cases:
        representative_settings = list(next(iter(cases.values())).keys())
    else:
        representative_settings = []

    return representative_settings, cases


def read_signal_results(signal_file: Path) -> pd.DataFrame:
    """Load SUMO signal state changes from XML into a DataFrame."""
    if not signal_file.exists():
        raise FileNotFoundError(f"Signal result file not found: {signal_file}")

    try:
        tree = ET.parse(signal_file)
    except ET.ParseError as exc:
        raise ValueError(f"Malformed signal XML '{signal_file}': {exc}") from exc
    root = tree.getroot()
    rows: list[dict[str, str]] = []

    for tls_state in root.findall("tlsState"):
        rows.append(tls_state.attrib)

    if not rows:
        return pd.DataFrame(columns=["id", "time", "state"])

    df = pd.DataFrame(rows)
    if "time" in df.columns:
        df["time"] = pd.to_numeric(df["time"], errors="coerce")
    return df


def load_signal_config(config_path: Path) -> pd.DataFrame:
    """Load signal configuration CSV and ensure required columns are present."""
    if not config_path.exists():
        raise FileNotFoundError(f"Signal configuration file not found: {config_path}")

    config = pd.read_csv(config_path)
    # Drop unnamed index column if present
    unnamed_cols = [col for col in config.columns if col.lower().startswith("unnamed")]
    if unnamed_cols:
        config = config.drop(columns=unnamed_cols)

    if "id" not in config.columns:
        raise ValueError("Signal configuration must include an 'id' column.")

    config["id"] = config["id"].astype(str)

    if "distance" not in config.columns or config["distance"].isna().any():
        if {"x", "y"}.issubset(config.columns):
            config["distance"] = np.sqrt(
                (config["x"] - REFERENCE_COORD[0]) ** 2 + (config["y"] - REFERENCE_COORD[1]) ** 2
            )
        else:
            raise ValueError("Signal configuration must include 'distance' column or 'x' and 'y' coordinates.")

    return config


def build_signal_segments(
    signal_df: pd.DataFrame,
    signal_config: pd.DataFrame,
    direction: str,
    time_window: tuple[float, float],
) -> list[dict[str, float | str]]:
    """Construct time segments for each signal state within the time window."""
    if signal_df.empty:
        return []

    start_time, end_time = time_window
    config_subset = signal_config.copy()
    if "approach_direction" in config_subset.columns:
        config_subset = config_subset[config_subset["approach_direction"] == direction]

    segments: list[dict[str, float | str]] = []
    for _, cfg_row in config_subset.iterrows():
        signal_id = str(cfg_row["id"])
        movement_indexes = cfg_row.get("movement_index")
        if isinstance(movement_indexes, str):
            try:
                movement_indexes = ast.literal_eval(movement_indexes)
            except (ValueError, SyntaxError):
                movement_indexes = []
        if not movement_indexes:
            continue

        target_index = movement_indexes[-1]
        signal_distance = float(cfg_row["distance"])
        intersection_name = cfg_row.get("int_name", signal_id)

        focused_signal = signal_df[signal_df["id"].astype(str) == signal_id].copy()
        if focused_signal.empty:
            continue

        focused_signal["time"] = pd.to_numeric(focused_signal["time"], errors="coerce")
        focused_signal = focused_signal.dropna(subset=["time", "state"])
        if focused_signal.empty:
            continue

        focused_signal = focused_signal.sort_values("time")
        focused_signal["state_char"] = focused_signal["state"].str[target_index : target_index + 1]
        focused_signal = focused_signal[focused_signal["state_char"].isin(SIGNAL_STATE_COLOR.keys())]

        if focused_signal.empty:
            continue

        times = focused_signal["time"].to_numpy(dtype=float)
        states = focused_signal["state_char"].tolist()

        current_start = times[0]
        current_state = states[0]

        for next_time, next_state in zip(times[1:], states[1:]):
            segments.append(
                {
                    "signal_id": signal_id,
                    "intersection": intersection_name,
                    "state": current_state,
                    "start_time": float(current_start),
                    "end_time": float(next_time),
                    "distance": signal_distance,
                }
            )
            current_start = next_time
            current_state = next_state

        segments.append(
            {
                "signal_id": signal_id,
                "intersection": intersection_name,
                "state": current_state,
                "start_time": float(current_start),
                "end_time": float(end_time),
                "distance": signal_distance,
            }
        )

    clipped_segments: list[dict[str, float | str]] = []
    for seg in segments:
        seg_start = max(seg["start_time"], start_time)
        seg_end = min(seg["end_time"], end_time)
        if seg_end <= seg_start:
            continue
        clipped_segments.append(
            {
                **seg,
                "start_time": float(seg_start),
                "end_time": float(seg_end),
            }
        )

    return clipped_segments


def _gaussian_kde(samples: np.ndarray, grid: np.ndarray) -> np.ndarray:
    """Simple Gaussian KDE for fallback plotting when seaborn is unavailable."""
    if samples.size == 0:
        return np.zeros_like(grid)
    std = float(np.std(samples))
    if std < 1e-6:
        std = max(0.1, abs(float(samples[0])) * 0.1)
    bandwidth = 1.06 * std * max(samples.size, 1) ** (-1 / 5)
    bandwidth = max(bandwidth, 1e-3)
    diffs = (grid[:, None] - samples[None, :]) / bandwidth
    density = np.exp(-0.5 * diffs**2).sum(axis=1)
    density /= samples.size * bandwidth * math.sqrt(2.0 * math.pi)
    return density
def extract_vehicle_trajectory(
    fcd_path: Path,
    vehicle_id: str,
    time_window: tuple[float, float] | None = None,
) -> dict[str, np.ndarray]:
    """Parse a SUMO FCD XML and return trajectory arrays for a given vehicle."""
    window_key: tuple[float | str, float | str]
    if time_window is None:
        window_key = ("*", "*")
    else:
        window_key = (float(time_window[0]), float(time_window[1]))
    cache_key = (str(fcd_path), vehicle_id, window_key)
    cached = _TRAJECTORY_CACHE.get(cache_key)
    if cached is not None:
        return cached

    if time_window is None:
        t_min, t_max = -math.inf, math.inf
    else:
        t_min, t_max = time_window
    times: list[float] = []
    speeds: list[float] = []
    distances: list[float] = []
    xs: list[float] = []
    ys: list[float] = []
    lanes: list[str] = []

    current_time: float | None = None
    in_window = False

    try:
        # iterparse keeps memory usage manageable for large FCD exports
        for event, elem in ET.iterparse(fcd_path, events=("start", "end")):
            tag = elem.tag
            if event == "start" and tag == "timestep":
                current_time = float(elem.attrib["time"])
                in_window = t_min <= current_time <= t_max
            elif event == "end" and tag == "timestep":
                elem.clear()
            elif event == "start" and tag == "vehicle":
                if not in_window or elem.attrib.get("id") != vehicle_id or current_time is None:
                    continue

                x_val = float(elem.attrib["x"])
                y_val = float(elem.attrib["y"])
                speed_val = float(elem.attrib["speed"])
                radial_distance = math.hypot(x_val - REFERENCE_COORD[0], y_val - REFERENCE_COORD[1])
                times.append(current_time)
                speeds.append(speed_val)
                distances.append(radial_distance)
                xs.append(x_val)
                ys.append(y_val)
                lanes.append(elem.attrib.get("lane", ""))
            elif event == "end" and tag == "vehicle":
                elem.clear()
    except ET.ParseError as exc:
        print(f"Unable to parse {fcd_path}: {exc}")
        empty_result = {
            "time": np.array([]),
            "speed": np.array([]),
            "distance": np.array([]),
            "x": np.array([]),
            "y": np.array([]),
            "lane": np.array([], dtype=object),
        }
        _TRAJECTORY_CACHE[cache_key] = empty_result
        return empty_result

    # ensure arrays are numpy arrays sorted by time
    if times:
        order = np.argsort(times)
        result = {
            "time": np.array(times)[order],
            "speed": np.array(speeds)[order],
            "distance": np.array(distances)[order],
            "x": np.array(xs)[order],
            "y": np.array(ys)[order],
            "lane": np.array(lanes, dtype=object)[order],
        }
        _TRAJECTORY_CACHE[cache_key] = result
        return result

    empty_result = {
        "time": np.array([]),
        "speed": np.array([]),
        "distance": np.array([]),
        "x": np.array([]),
        "y": np.array([]),
        "lane": np.array([], dtype=object),
    }
    _TRAJECTORY_CACHE[cache_key] = empty_result
    return empty_result


def plot_distance_time(
    ego_data: dict[str, np.ndarray],
    replaced_data: dict[str, np.ndarray],
    time_window: tuple[float, float],
    output_path: Path,
    replaced_vehicle_id: str,
    signal_segments: list[dict[str, float | str]] | None = None,
    signal_linewidth: float = 6.0,
) -> None:
    time_offset = time_window[0]
    ego_time = ego_data["time"] - time_offset
    replaced_time = replaced_data["time"] - time_offset

    plt.figure(figsize=(10, 6))
    plt.plot(replaced_time, replaced_data["distance"], label=f"Vehicle {replaced_vehicle_id} (baseline)")
    plt.plot(ego_time, ego_data["distance"], label="Ego vehicle", linestyle="--")

    if signal_segments:
        seen_states: set[str] = set()
        for segment in signal_segments:
            state = str(segment["state"])
            color = SIGNAL_STATE_COLOR.get(state, "#7f8c8d")
            label = SIGNAL_STATE_LABEL.get(state, state)
            start = segment["start_time"] - time_offset
            end = segment["end_time"] - time_offset
            if end <= 0 or start >= (time_window[1] - time_offset):
                continue
            start = max(start, 0.0)
            end = max(end, start)
            plt.plot(
                [start, end],
                [segment["distance"], segment["distance"]],
                color=color,
                linewidth=signal_linewidth,
                solid_capstyle="butt",
                label=f"{label} Signal" if state not in seen_states else None,
            )
            seen_states.add(state)

    plt.xlabel(f"Time since {time_offset:.0f} s (s)")
    plt.ylabel("Distance from reference (m)")
    plt.xlim(0, max(time_window[1] - time_offset, ego_time.max(initial=0)))
    plt.legend()
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_speed_time(ego_data: dict[str, np.ndarray], replaced_data: dict[str, np.ndarray], time_window: tuple[float, float], output_path: Path, replaced_vehicle_id: str) -> None:
    time_offset = time_window[0]
    ego_time = ego_data["time"] - time_offset
    replaced_time = replaced_data["time"] - time_offset

    plt.figure(figsize=(10, 6))
    plt.plot(replaced_time, replaced_data["speed"], label=f"Vehicle {replaced_vehicle_id} (baseline)")
    plt.plot(ego_time, ego_data["speed"], label="Ego vehicle", linestyle="--")
    plt.xlabel(f"Time since {time_offset:.0f} s (s)")
    plt.ylabel("Speed (m/s)")
    plt.xlim(0, max(time_window[1] - time_offset, ego_time.max(initial=0)))
    plt.legend()
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_speed_distribution(ego_data: dict[str, np.ndarray], replaced_data: dict[str, np.ndarray], output_path: Path, replaced_vehicle_id: str) -> None:
    plt.figure(figsize=(10, 6))
    ego_speed = ego_data["speed"]
    baseline_speed = replaced_data["speed"]

    if sns is not None:
        sns.kdeplot(
            baseline_speed,
            label=f"Vehicle {replaced_vehicle_id} (baseline)",
            linewidth=2.0,
            fill=False,
            common_norm=False,
            warn_singular=False,
        )
        sns.kdeplot(
            ego_speed,
            label="Ego vehicle",
            linewidth=2.0,
            linestyle="--",
            fill=False,
            common_norm=False,
            warn_singular=False,
        )
    else:
        baseline_min = float(baseline_speed.min()) if baseline_speed.size else 0.0
        ego_min = float(ego_speed.min()) if ego_speed.size else 0.0
        baseline_max = float(baseline_speed.max()) if baseline_speed.size else 0.0
        ego_max = float(ego_speed.max()) if ego_speed.size else 0.0
        min_speed = min(baseline_min, ego_min)
        max_speed = max(baseline_max, ego_max)
        if math.isclose(min_speed, max_speed):
            max_speed += 1.0
        grid = np.linspace(min_speed, max_speed, 400)
        baseline_density = _gaussian_kde(baseline_speed, grid)
        ego_density = _gaussian_kde(ego_speed, grid)
        plt.plot(grid, baseline_density, label=f"Vehicle {replaced_vehicle_id} (baseline)", linewidth=2.0)
        plt.plot(grid, ego_density, label="Ego vehicle", linewidth=2.0, linestyle="--")

    plt.xlabel("Speed (m/s)")
    plt.ylabel("Density (smoothed KDE)")
    plt.legend()
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def filter_by_lanes(data: dict[str, np.ndarray], allowed_lanes: list[str]) -> dict[str, np.ndarray]:
    """Keep only data points that belong to the specified lanes."""
    if data["distance"].size == 0:
        return data
    lane_edges = np.array(
        [
            lane.split("_")[0] if isinstance(lane, str) else ""
            for lane in data["lane"]
        ],
        dtype=object,
    )
    mask = np.isin(lane_edges, allowed_lanes)
    return {key: values[mask] for key, values in data.items()}


def trim_to_distance_limit(data: dict[str, np.ndarray], distance_limit: float) -> dict[str, np.ndarray]:
    """Return a copy of the trajectory limited to a maximum distance."""
    if data["distance"].size == 0:
        return data

    mask = data["distance"] <= distance_limit + 1e-6
    if mask.all():
        return data

    return {key: values[mask] for key, values in data.items()}


def plot_distance_time_across_settings(
    data_by_setting: dict[str, dict[str, np.ndarray]],
    time_bounds: tuple[float, float],
    output_path: Path,
    entry_time_label: str,
    signal_segments: list[dict[str, float | str]] | None = None,
    signal_linewidth: float = 4.0,
) -> None:
    if not data_by_setting:
        raise ValueError(f"No data provided for entry {entry_time_label}.")

    start_time, end_time = time_bounds
    if not math.isfinite(start_time) or not math.isfinite(end_time) or start_time >= end_time:
        raise ValueError(f"Invalid time bounds {time_bounds} for entry {entry_time_label}.")

    plt.figure(figsize=(10, 6))
    for setting, data in data_by_setting.items():
        plt.plot(data["time"], data["distance"], label=setting)

    if signal_segments:
        seen_states: set[str] = set()
        for segment in signal_segments:
            state = str(segment["state"])
            color = SIGNAL_STATE_COLOR.get(state, "#7f8c8d")
            label = SIGNAL_STATE_LABEL.get(state, state)
            seg_start = max(float(segment["start_time"]), start_time)
            seg_end = min(float(segment["end_time"]), end_time)
            if seg_end <= seg_start:
                continue
            plt.plot(
                [seg_start, seg_end],
                [float(segment["distance"]), float(segment["distance"])],
                color=color,
                linewidth=signal_linewidth,
                solid_capstyle="butt",
                # label=f"{label} Signal" if state not in seen_states else None,
            )
            seen_states.add(state)

    plt.title(f"Ego Distance vs Time — Entry {entry_time_label}")
    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Distance from reference (m)")
    plt.xlim(start_time, end_time)
    plt.legend(title="Setting")
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_speed_time_across_settings(
    data_by_setting: dict[str, dict[str, np.ndarray]],
    time_bounds: tuple[float, float],
    output_path: Path,
    entry_time_label: str,
) -> None:
    if not data_by_setting:
        raise ValueError(f"No data provided for entry {entry_time_label}.")

    start_time, end_time = time_bounds
    if not math.isfinite(start_time) or not math.isfinite(end_time) or start_time >= end_time:
        raise ValueError(f"Invalid time bounds {time_bounds} for entry {entry_time_label}.")

    plt.figure(figsize=(10, 6))
    for setting, data in data_by_setting.items():
        plt.plot(data["time"], data["speed"], label=setting)

    plt.title(f"Ego Speed vs Time — Entry {entry_time_label}")
    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.xlim(start_time, end_time)
    plt.legend(title="Setting")
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_speed_distribution_across_settings(
    data_by_setting: dict[str, dict[str, np.ndarray]],
    output_path: Path,
    entry_time_label: str,
) -> None:
    plt.figure(figsize=(10, 6))

    non_empty = {setting: data for setting, data in data_by_setting.items() if data["speed"].size > 0}
    if not non_empty:
        plt.text(0.5, 0.5, f"No speed data for entry {entry_time_label}", ha="center", va="center", transform=plt.gca().transAxes)
        plt.savefig(output_path, dpi=300)
        plt.close()
        return

    if sns is not None:
        for setting, data in non_empty.items():
            sns.kdeplot(
                data["speed"],
                label=setting,
                linewidth=2.0,
                fill=False,
                common_norm=False,
                warn_singular=False,
            )
    else:
        min_speed = min(float(np.min(data["speed"])) for data in non_empty.values())
        max_speed = max(float(np.max(data["speed"])) for data in non_empty.values())
        if math.isclose(min_speed, max_speed):
            max_speed += 1.0
        grid = np.linspace(min_speed, max_speed, 400)
        for setting, data in non_empty.items():
            density = _gaussian_kde(data["speed"], grid)
            plt.plot(grid, density, label=setting, linewidth=2.0)

    plt.title(f"Ego Speed Distribution — Entry {entry_time_label}")
    plt.xlabel("Speed (m/s)")
    plt.ylabel("Density (smoothed KDE)")
    plt.legend(title="Setting")
    plt.grid(True, linestyle=":", linewidth=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def plot_ego_across_settings(
    entry_time: str,
    setting_dirs: dict[str, Path],
    time_window: tuple[float, float] | None = TIME_WINDOW,
    allowed_lanes: list[str] | None = None,
    include_signals: bool = False,
    signal_config_path: Path | None = None,
    output_root: Path | None = None,
    direction: str = "WB",
) -> dict[str, object]:
    """
    Generate comparison plots for the ego vehicle across experiment settings.
    """
    allowed_lanes = allowed_lanes or WB_LANES
    output_base = Path(output_root) if output_root is not None else OUTPUT_ROOT
    entry_slug = sanitize_entry_label(entry_time)
    output_dir = output_base / entry_slug
    output_dir.mkdir(parents=True, exist_ok=True)

    data_by_setting: dict[str, dict[str, np.ndarray]] = {}
    for setting_name, case_dir in setting_dirs.items():
        fcd_path = case_dir / "fcd.xml"
        if not fcd_path.exists():
            raise FileNotFoundError(f"FCD file missing for setting '{setting_name}' at {fcd_path}")

        ego_data = extract_vehicle_trajectory(fcd_path, EGO_VEHICLE_ID, time_window)
        ego_data = filter_by_lanes(ego_data, allowed_lanes)
        if ego_data["time"].size == 0:
            raise RuntimeError(f"Ego trajectory missing on selected lanes for setting '{setting_name}'.")
        data_by_setting[setting_name] = ego_data

    sample_counts = {setting: int(data["time"].size) for setting, data in data_by_setting.items()}
    time_mins = [float(np.min(data["time"])) for data in data_by_setting.values() if data["time"].size > 0]
    time_maxs = [float(np.max(data["time"])) for data in data_by_setting.values() if data["time"].size > 0]
    if not time_mins or not time_maxs:
        raise RuntimeError(f"No time samples available across settings for entry {entry_time}.")

    if time_window is not None:
        plot_start, plot_end = time_window
    else:
        plot_start, plot_end = min(time_mins), max(time_maxs)

    if not math.isfinite(plot_start) or not math.isfinite(plot_end) or plot_start >= plot_end:
        raise RuntimeError(
            f"Invalid time bounds derived for entry {entry_time}: start={plot_start}, end={plot_end}"
        )

    time_bounds = (plot_start, plot_end)

    signal_segments: list[dict[str, float | str]] | None = None
    if include_signals:
        signal_config: pd.DataFrame | None = None
        signal_errors: list[str] = []
        config_path = Path(signal_config_path) if signal_config_path is not None else SIGNAL_CONFIG_PATH
        for setting_name, case_dir in setting_dirs.items():
            signal_file = case_dir / "signal_result.xml"
            if not signal_file.exists():
                signal_errors.append(f"{setting_name}: missing signal_result.xml")
                continue
            try:
                signal_df = read_signal_results(signal_file)
                if signal_config is None:
                    signal_config = load_signal_config(config_path)
                signal_segments = build_signal_segments(
                    signal_df,
                    signal_config,
                    direction,
                    time_bounds,
                )
                if signal_segments is not None:
                    break
            except (FileNotFoundError, ValueError) as exc:
                signal_errors.append(f"{setting_name}: {exc}")
                continue

        if signal_segments is None and signal_errors:
            errors_joined = "; ".join(signal_errors)
            print(f"Warning: unable to overlay signals for entry {entry_time}: {errors_joined}")

    distance_plot = output_dir / f"{entry_slug}_distance_time_ego.png"
    speed_plot = output_dir / f"{entry_slug}_speed_time_ego.png"
    distribution_plot = output_dir / f"{entry_slug}_speed_distribution_ego.png"

    plot_distance_time_across_settings(
        data_by_setting,
        time_bounds,
        distance_plot,
        entry_time,
        signal_segments=signal_segments,
    )
    plot_speed_time_across_settings(
        data_by_setting,
        time_bounds,
        speed_plot,
        entry_time,
    )
    plot_speed_distribution_across_settings(
        data_by_setting,
        distribution_plot,
        entry_time,
    )

    return {
        "entry_time": entry_time,
        "settings": list(data_by_setting.keys()),
        "output_dir": output_dir,
        "time_window": time_bounds,
        "plots": {
            "distance_time": distance_plot,
            "speed_time": speed_plot,
            "speed_distribution": distribution_plot,
        },
        "signal_segments_used": signal_segments is not None and len(signal_segments) > 0,
        "sample_counts": sample_counts,
    }


def main(argv: Sequence[str] | None = None) -> None:
    args = parse_args(argv)

    time_window = TIME_WINDOW
    if args.no_time_window:
        time_window = None
    if args.time_window:
        time_window = (float(args.time_window[0]), float(args.time_window[1]))

    results_root = Path(args.results_dir)
    output_root = Path(args.output_dir) if args.output_dir else results_root / "plots"
    settings_override = args.settings if args.settings else DEFAULT_EXPERIMENT_SETTINGS

    try:
        active_settings, cases = discover_simulation_cases(results_root, settings_override)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"Unable to prepare plots: {exc}")
        return

    if not cases:
        print("No matching entry times found to plot.")
        return

    output_root.mkdir(parents=True, exist_ok=True)

    if active_settings:
        print(f"Comparing settings: {', '.join(active_settings)}")
    else:
        print("Comparing settings: none")

    if time_window is None:
        print("Using full simulation time range.")
    else:
        print(f"Using time window: {time_window[0]:.2f} s to {time_window[1]:.2f} s")

    for entry_time, setting_dirs in cases.items():
        try:
            result = plot_ego_across_settings(
                entry_time,
                setting_dirs,
                time_window=time_window,
                allowed_lanes=WB_LANES,
                include_signals=True,
                output_root=output_root,
            )
        except (FileNotFoundError, RuntimeError) as exc:
            print(f"Skipping entry {entry_time}: {exc}")
            continue

        start, end = result["time_window"]
        sample_counts = result.get("sample_counts", {})
        if sample_counts:
            counts_summary = ", ".join(f"{name}={count}" for name, count in sample_counts.items())
        else:
            counts_summary = "n/a"
        print(
            f"Plots saved for entry {entry_time} in {result['output_dir']} "
            f"(samples: {counts_summary}; time {start:.2f}-{end:.2f} s)"
        )


if __name__ == "__main__":
    main()
