"""
Pure GLOSA (Green Light Optimal Speed Advisory) eco-driving math for #173.

Reimplemented from the SUMO-Carla GLOSA reference (#120,
tests/SumoCarla/.../glosa_test.py). ONLY the algorithm is reused — the
original is welded to traci (getSpeed / slowDown / getNextTLS /
getNextSwitch). Here it is reduced to pure functions: given a CAV's speed,
its distance to the next signal, and that signal's FIXED timing, return a
recommended desired speed. No sockets, no traci, no I/O — so it is trivially
auditable and reusable.

Sign / unit conventions (SI throughout):
  - distances in meters, speeds in m/s, times in seconds, accels in m/s^2
  - distance_to_signal > 0 means the signal is ahead of the CAV
  - a "switch" is the next change of the CAV's movement signal group

The decision (faithful to #120's adapt_speed):
  - If the CAV can reach the signal BEFORE it next turns red (i.e. it can
    clear on the current/next green), GLOSA computes a single smooth speed
    that arrives just as the window opens/while it is open -> avoid a stop.
  - Otherwise it releases the CAV back toward free-flow (let it approach;
    car-following / the stop logic handles the actual halt).

This module is intentionally dependency-free (stdlib math only).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GlosaParams:
    """Tunable GLOSA constants. Defaults mirror the #120 reference."""
    a_max: float = 2.6        # comfortable accel ceiling (m/s^2)
    d_max: float = 4.5        # comfortable decel ceiling (m/s^2)
    v_max: float = 13.89      # free-flow / speed-limit target (m/s ~= 50 km/h)
    v_min: float = 5.0        # lowest speed GLOSA will advise while creeping (m/s)
    decel: float = 2.5        # nominal decel used for slow-downs (m/s^2)
    range_m: float = 150.0    # only advise within this distance of the signal (m)


# --- Motion utilities (ported verbatim from #120, made pure) ----------------

def distance_at_continuous_accel(speed: float, time: float, a_max: float) -> float:
    """Distance covered accelerating at a_max for `time` from `speed`."""
    return speed * time + a_max * time ** 2 / 2.0


def time_to_junction_at_continuous_accel(distance: float, speed: float, a_max: float) -> float:
    """Time to cover `distance` starting at `speed`, accelerating at a_max."""
    a = a_max
    p_half = speed / a
    return -p_half + math.sqrt(p_half ** 2 + 2.0 * distance / a)


def earliest_arrival(speed: float, distance: float, p: GlosaParams) -> float:
    """Earliest possible arrival time at the signal (accelerate then cruise)."""
    max_speed = max(p.v_max, speed)
    accel_time = min(
        (max_speed - speed) / p.a_max if p.a_max > 0 else 0.0,
        time_to_junction_at_continuous_accel(distance, speed, p.a_max),
    )
    remaining_dist = distance - distance_at_continuous_accel(speed, accel_time, p.a_max)
    remaining_time = remaining_dist / max_speed if max_speed > 0 else float("inf")
    return accel_time + remaining_time


def glosa_maneuver(speed: float, distance: float, time_till_switch: float,
                   p: GlosaParams) -> tuple[float, float]:
    """
    Solve for a single target speed `x` (held, then resume) that places the
    CAV at the signal exactly at `time_till_switch`. Returns (target_speed,
    hold_duration). If no feasible smooth maneuver exists, returns
    (speed, 0.0) — i.e. "no advice, keep going".

    This is the #120 closed-form maneuver, kept verbatim (only parameterized).
    """
    t = time_till_switch
    a = p.a_max
    d = p.d_max
    u = p.v_min
    w = max(p.v_max, speed)
    s = distance
    v = speed
    sign0 = -1
    root_argument = (
        a * d * ((2.0 * d * (s - (w * t))) - ((v - w) ** 2)
                 + (a * ((d * (t ** 2)) + (2.0 * (s - (t * v))))))
    )
    if root_argument < 0:
        return v, 0.0
    x = (((a * (v - (d * t))) + (d * w) - sign0 * math.sqrt(root_argument)) / (d + a))
    y = t - (w - x) / d
    if u <= x <= w and 0 < y < t:
        return x, y
    return v, 0.0


# --- Public API -------------------------------------------------------------

@dataclass
class Advice:
    target_speed: float   # m/s desired speed to push to the DriverModel CAV
    active: bool          # True when GLOSA is actively shaping the speed
    reason: str           # short tag for logging / diagnostics


def recommend_speed(speed: float,
                    distance_to_signal: float,
                    time_to_green_start: float,
                    time_to_green_end: float,
                    p: GlosaParams = GlosaParams()) -> Advice:
    """
    Core eco-driving advisory for one CAV approaching one signal, aiming at the
    next usable GREEN window [time_to_green_start, time_to_green_end].

    Args:
      speed:               current CAV speed (m/s)
      distance_to_signal:  distance to the signal stop line (m); <= 0 -> past it
      time_to_green_start: seconds until that green window OPENS (0.0 if the
                           movement is green right now). Caller derives this from
                           the FIXED signal plan + current sim time.
      time_to_green_end:   seconds until that green window CLOSES (turns red).
      p:                   GLOSA tuning constants

    The three cases (faithful to #120's adapt_speed, made phase-explicit):
      1. Would arrive BEFORE green opens  -> slow to arrive exactly at the open
         (glosa_maneuver targets time_to_green_start) so the CAV rolls through
         instead of stopping and idling. This is the eco win.
      2. Would arrive DURING the green     -> proceed at free-flow ("just go").
      3. Cannot reach before green closes  -> ease toward v_min so the unavoidable
         stop is gentle (decel / energy proxy), not a hard brake.
    """
    if distance_to_signal <= 0.0 or distance_to_signal > p.range_m:
        return Advice(p.v_max, False, "out_of_range")

    ea = earliest_arrival(speed, distance_to_signal, p)

    # Case 1 — too early for the green: shape a smooth slow-down to the open.
    if ea < time_to_green_start:
        if speed > p.v_min:
            target, duration = glosa_maneuver(speed, distance_to_signal,
                                              time_to_green_start, p)
            if duration > 0.0 and target < speed:
                return Advice(target, True, "arrive_at_green")
        # No feasible smooth maneuver (e.g. very close / already slow): creep.
        return Advice(p.v_min, True, "creep_to_green")

    # Case 2 — we land inside the green window: clear it at free-flow.
    if ea <= time_to_green_end:
        return Advice(p.v_max, False, "clear_freeflow")

    # Case 3 — green closes before we can get there: ease down for a soft stop.
    return Advice(p.v_min, True, "cannot_clear_slow")


if __name__ == "__main__":
    # Minimal sanity check (no pytest dependency). Not a substitute for an
    # in-sim run; just guards the math against obvious regressions.
    p = GlosaParams()

    # Out of range (signal far away) -> free-flow, inactive.
    far = recommend_speed(12.0, 500.0, 0.0, 12.0, p)
    assert not far.active and far.target_speed == p.v_max, far

    # Green now, arrives comfortably within the window -> just go.
    clear = recommend_speed(12.0, 120.0, 0.0, 20.0, p)
    print("clear:", clear)
    assert not clear.active and clear.target_speed == p.v_max, clear

    # Red ahead, green opens in 20 s but CAV would arrive in ~7 s -> slow to
    # arrive at the green open (the eco win): a smooth target below current speed.
    early = recommend_speed(12.0, 80.0, 20.0, 35.0, p)
    print("arrive_at_green:", early)
    assert early.active and 0.0 < early.target_speed < 12.0, early

    # Green window essentially closed -> cannot clear -> ease toward v_min.
    cant = recommend_speed(12.0, 120.0, 0.0, 0.5, p)
    print("cannot_clear:", cant)
    assert cant.active and cant.target_speed == p.v_min, cant

    print("eco_driving self-check OK")
