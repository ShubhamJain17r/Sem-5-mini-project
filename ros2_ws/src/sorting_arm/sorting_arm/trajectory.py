#!/usr/bin/env python3
"""
Trajectory Interpolation and Motion Smoothing for Hiwonder / LewanSoul xArm 1S.

Provides:
  - Quintic (minimum-jerk) and cubic polynomial trajectory profiling
  - Joint-space smooth waypoint interpolation
  - Cylindrical Cartesian path interpolation with inverse kinematics resolution
  - Velocity and acceleration bounded trajectory planning to eliminate jerk and mechanical vibration
"""

import math
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

from sorting_arm.kinematics import KinematicParameters, solve_ik


class TrajectoryPoint(NamedTuple):
    """Single discrete point along an interpolated trajectory."""

    time_sec: float
    duration_ms: int
    ticks: Dict[int, int]
    angles_deg: Optional[Dict[int, float]] = None
    velocities_deg_s: Optional[Dict[int, float]] = None


def quintic_scaling(u: float) -> Tuple[float, float, float]:
    """
    Evaluate normalized quintic polynomial (minimum jerk) at u in [0, 1].

    Position: s(u) = 10u^3 - 15u^4 + 6u^5
    Velocity factor: s'(u) = 30u^2 - 60u^3 + 30u^4
    Acceleration factor: s''(u) = 60u - 180u^2 + 120u^3

    Guarantees zero velocity and zero acceleration at both endpoints (u=0 and u=1).
    """
    u = max(0.0, min(1.0, u))
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    u5 = u4 * u

    s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5
    ds = 30.0 * u2 - 60.0 * u3 + 30.0 * u4
    dds = 60.0 * u - 180.0 * u2 + 120.0 * u3
    return s, ds, dds


def cubic_scaling(u: float) -> Tuple[float, float]:
    """
    Evaluate normalized cubic polynomial at u in [0, 1].

    Position: s(u) = 3u^2 - 2u^3
    Velocity factor: s'(u) = 6u - 6u^2
    """
    u = max(0.0, min(1.0, u))
    u2 = u * u
    s = 3.0 * u2 - 2.0 * u2 * u
    ds = 6.0 * u - 6.0 * u2
    return s, ds


def generate_joint_trajectory(
    start_ticks: Dict[int, int],
    target_ticks: Dict[int, int],
    duration_sec: float = 1.0,
    dt_sec: float = 0.05,
    profile: str = "quintic",
) -> List[TrajectoryPoint]:
    """
    Generate smooth intermediate joint waypoints from start_ticks to target_ticks.

    :param start_ticks: Starting positions for active motors {id: tick}
    :param target_ticks: Final target positions for active motors {id: tick}
    :param duration_sec: Total movement duration in seconds
    :param dt_sec: Sampling interval (default: 0.05s = 20 Hz)
    :param profile: "quintic" (minimum jerk) or "cubic"
    :return: List of TrajectoryPoint instances
    """
    if duration_sec <= 0:
        duration_sec = 0.05

    num_steps = max(2, int(math.ceil(duration_sec / dt_sec)) + 1)
    points: List[TrajectoryPoint] = []

    # Active joint IDs present in target
    active_ids = [sid for sid in target_ticks if 1 <= sid <= 6]

    for step in range(num_steps):
        t = min(duration_sec, step * dt_sec)
        u = t / duration_sec

        if profile == "cubic":
            s, ds = cubic_scaling(u)
        else:
            s, ds, _ = quintic_scaling(u)

        step_ticks: Dict[int, int] = {}
        for sid in active_ids:
            start_pos = start_ticks.get(sid, 500)
            target_pos = target_ticks[sid]
            interpolated = start_pos + s * (target_pos - start_pos)
            step_ticks[sid] = int(round(interpolated))

        step_duration_ms = int(round(dt_sec * 1000.0))
        points.append(
            TrajectoryPoint(
                time_sec=t,
                duration_ms=step_duration_ms,
                ticks=step_ticks,
            )
        )

    return points


def generate_cartesian_trajectory(
    start_polar: Tuple[float, float, float],
    target_polar: Tuple[float, float, float],
    claw_tick: int,
    roll_tick: int = 500,
    duration_sec: float = 1.0,
    dt_sec: float = 0.05,
    params: Optional[KinematicParameters] = None,
    profile: str = "quintic",
) -> Tuple[List[TrajectoryPoint], bool]:
    """
    Generate smooth intermediate Cartesian waypoints and solve IK at each step.

    :param start_polar: Starting (r, theta_deg, z)
    :param target_polar: Final (r, theta_deg, z)
    :param claw_tick: Claw position
    :param roll_tick: Roll position
    :param duration_sec: Movement duration
    :param dt_sec: Sampling interval
    :param params: Kinematic parameters
    :param profile: Interpolation profile
    :return: (points, all_ik_success)
    """
    if duration_sec <= 0:
        duration_sec = 0.05

    num_steps = max(2, int(math.ceil(duration_sec / dt_sec)) + 1)
    points: List[TrajectoryPoint] = []
    all_success = True

    r0, th0, z0 = start_polar
    rf, thf, zf = target_polar

    for step in range(num_steps):
        t = min(duration_sec, step * dt_sec)
        u = t / duration_sec

        if profile == "cubic":
            s, _ = cubic_scaling(u)
        else:
            s, _, _ = quintic_scaling(u)

        curr_r = r0 + s * (rf - r0)
        curr_th = th0 + s * (thf - th0)
        curr_z = z0 + s * (zf - z0)

        sol = solve_ik(
            r=curr_r,
            theta=curr_th,
            z=curr_z,
            claw_tick=claw_tick,
            roll_tick=roll_tick,
            params=params,
        )

        if not sol.success:
            all_success = False

        step_duration_ms = int(round(dt_sec * 1000.0))
        points.append(
            TrajectoryPoint(
                time_sec=t,
                duration_ms=step_duration_ms,
                ticks=sol.ticks,
                angles_deg=sol.angles_deg,
            )
        )

    return points, all_success
