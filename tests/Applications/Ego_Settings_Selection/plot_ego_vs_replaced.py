from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import ast
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    import seaborn as sns
except ImportError:  # pragma: no cover - optional dependency
    sns = None


_TRAJECTORY_CACHE: dict[tuple[str, str, tuple[float, float]], dict[str, np.ndarray]] = {}


TIME_WINDOW = (28800, 29500)  # seconds
REFERENCE_COORD = (1390.0, 225.0)
WB_LANES = [
    "-2801",
    "-280",
    "-307",
    "-327",
    "-3271",
    "-281",
    "-315",
    "-3151",
    "-321",
    "-300",
    "-2851",
    "-285",
    "-290",
    "-298",
    "-295",
]
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


def read_signal_results(signal_file: Path) -> pd.DataFrame:
    """Load SUMO signal state changes from XML into a DataFrame."""
    if not signal_file.exists():
        raise FileNotFoundError(f"Signal result file not found: {signal_file}")

    tree = ET.parse(signal_file)
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
BASE_SCENARIO_DIR = Path(
    "Shallowford_after_calibration_banleftturn_AdjustedFixedTime_V3"
) / "MPR"
BASELINE_DIR = BASE_SCENARIO_DIR / "0%_Seed100_Subscription_10Hz"
BASE_FCD_PATH = BASELINE_DIR / "fcd.xml"
SIGNAL_CONFIG_PATH = BASE_SCENARIO_DIR.parent / "sumoSignalConfig_26.csv"
TARGET_ID_LIST = [
    "4.66",
    "4.68",
    "4.72",
    "4.73",
    "4.76",
    "4.77",
    "4.82",
    "4.83",
    "4.84",
    "4.85",
]
EGO_VEHICLE_ID = "ego"
OUTPUT_ROOT = BASE_SCENARIO_DIR / "plots"


def extract_vehicle_trajectory(fcd_path: Path, vehicle_id: str, time_window: tuple[float, float]) -> dict[str, np.ndarray]:
    """Parse a SUMO FCD XML and return trajectory arrays for a given vehicle."""
    cache_key = (str(fcd_path), vehicle_id, time_window)
    cached = _TRAJECTORY_CACHE.get(cache_key)
    if cached is not None:
        return cached

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


def plot_EGO_WITH_REPLACE(
    target_id: str,
    scenario_dir: Path | None = None,
    baseline_fcd_path: Path | None = None,
    output_dir: Path | None = None,
    time_window: tuple[float, float] | None = None,
    allowed_lanes: list[str] | None = None,
    include_signals: bool = True,
    signal_config_path: Path | None = None,
    direction: str = "WB",
) -> dict[str, object]:
    """
    Generate comparison plots for the ego vehicle replacing a target vehicle.
    """
    time_window = time_window or TIME_WINDOW
    allowed_lanes = allowed_lanes or WB_LANES

    scenario_dir = Path(
        scenario_dir if scenario_dir is not None else BASE_SCENARIO_DIR / f"0%_Seed100_Subscription_10Hz_{target_id}"
    )
    baseline_fcd_path = Path(baseline_fcd_path if baseline_fcd_path is not None else BASE_FCD_PATH)
    scenario_fcd_path = scenario_dir / "fcd.xml"

    if not scenario_dir.exists():
        raise FileNotFoundError(f"Scenario directory not found: {scenario_dir}")
    if not scenario_fcd_path.exists():
        raise FileNotFoundError(f"Scenario FCD file missing: {scenario_fcd_path}")
    if not baseline_fcd_path.exists():
        raise FileNotFoundError(f"Baseline FCD file missing: {baseline_fcd_path}")

    output_dir = Path(output_dir if output_dir is not None else OUTPUT_ROOT / target_id.replace(".", "_"))
    output_dir.mkdir(parents=True, exist_ok=True)

    replaced_baseline = extract_vehicle_trajectory(baseline_fcd_path, target_id, time_window)
    if replaced_baseline["time"].size == 0:
        replaced_baseline = extract_vehicle_trajectory(scenario_fcd_path, target_id, time_window)
    replaced_baseline = filter_by_lanes(replaced_baseline, allowed_lanes)
    if replaced_baseline["time"].size == 0:
        raise RuntimeError(f"vehicle {target_id} trajectory missing on selected lanes.")

    ego_data = extract_vehicle_trajectory(scenario_fcd_path, EGO_VEHICLE_ID, time_window)
    ego_data = filter_by_lanes(ego_data, allowed_lanes)
    if ego_data["time"].size == 0:
        raise RuntimeError(f"Ego trajectory missing on selected lanes in {scenario_fcd_path}.")

    distance_limit = float(replaced_baseline["distance"].max()) if replaced_baseline["distance"].size else 0.0
    ego_data = trim_to_distance_limit(ego_data, distance_limit)
    if ego_data["time"].size == 0:
        raise RuntimeError("Ego trajectory exceeds available distance window.")

    signal_segments: list[dict[str, float | str]] | None = None
    if include_signals:
        signal_file = scenario_dir / "signal_result.xml"
        try:
            signal_df = read_signal_results(signal_file)
            config_path = Path(signal_config_path) if signal_config_path is not None else SIGNAL_CONFIG_PATH
            signal_config = load_signal_config(config_path)
            signal_segments = build_signal_segments(signal_df, signal_config, direction, time_window)
        except (FileNotFoundError, ValueError) as exc:
            print(f"Warning: unable to overlay signals for target {target_id}: {exc}")
            signal_segments = None

    distance_plot = output_dir / f"distance_time_ego_vs_{target_id.replace('.', '_')}.png"
    speed_plot = output_dir / f"speed_time_ego_vs_{target_id.replace('.', '_')}.png"
    distribution_plot = output_dir / f"speed_distribution_ego_vs_{target_id.replace('.', '_')}.png"

    plot_distance_time(
        ego_data,
        replaced_baseline,
        time_window,
        distance_plot,
        target_id,
        signal_segments=signal_segments,
    )
    plot_speed_time(
        ego_data,
        replaced_baseline,
        time_window,
        speed_plot,
        target_id,
    )
    plot_speed_distribution(
        ego_data,
        replaced_baseline,
        distribution_plot,
        target_id,
    )

    return {
        "target_id": target_id,
        "output_dir": output_dir,
        "plots": {
            "distance_time": distance_plot,
            "speed_time": speed_plot,
            "speed_distribution": distribution_plot,
        },
        "signal_segments_used": signal_segments is not None and len(signal_segments) > 0,
    }


def main() -> None:
    if not BASELINE_DIR.exists():
        raise FileNotFoundError(f"Baseline directory not found: {BASELINE_DIR}")

    OUTPUT_ROOT.mkdir(exist_ok=True)

    for target_id in TARGET_ID_LIST:
        try:
            result = plot_EGO_WITH_REPLACE(target_id)
        except (FileNotFoundError, RuntimeError) as exc:
            print(f"Skipping {target_id}: {exc}")
            continue

        print(f"Plots saved for target {target_id} in {result['output_dir']}")


if __name__ == "__main__":
    main()
