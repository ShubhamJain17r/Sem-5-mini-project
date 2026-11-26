#!/usr/bin/env python3
"""
Analytical Inverse and Forward Kinematics for LewanSoul / Hiwonder xArm 1S.

Features:
  - Cylindrical/Polar coordinates (r, theta, z) relative to arm base center (Z=0 on table).
  - Dynamically configurable link lengths (d1, L1, L2, L3) and zero-reference angles.
  - Natural pitch generator with near-field collision avoidance (keeps wrist pitch safe).
  - Resolves 4-compartment collection box target coordinates for color sorting.
"""

from dataclasses import dataclass
import math
import os
from typing import Dict, NamedTuple, Optional, Tuple
import yaml

# Exact ground-truth servo tick scales for Hiwonder LX-15D (1000 ticks / 240 deg)
# Motors 3, 4, 5: 1000 ticks / 240 deg = 4.1667 ticks/deg (0.24 deg/tick)
TICKS_PER_DEG_PITCH = 1000.0 / 240.0
DEG_PER_TICK_PITCH = 240.0 / 1000.0

# Motor 6 (Base Yaw): 760 ticks / 180 deg = 4.2222 ticks/deg (120 CW, 880 ACW)
TICKS_PER_DEG_YAW = 760.0 / 180.0
DEG_PER_TICK_YAW = 180.0 / 760.0

# Motor 2 (Wrist Roll): 740 ticks / 180 deg = 4.1111 ticks/deg (130 CW, 870 ACW)
TICKS_PER_DEG_ROLL = 740.0 / 180.0
DEG_PER_TICK_ROLL = 180.0 / 740.0

# General fallback alias (240 deg / 1000 ticks)
TICKS_PER_DEG = 1000.0 / 240.0
DEG_PER_TICK = 240.0 / 1000.0


@dataclass
class KinematicParameters:
    """Configurable geometric and hardware parameters for an xArm manipulator."""

    d1: float = 70.0          # Base table surface to shoulder pitch axis (65mm + 5mm baseboard)
    L1: float = 101.0         # Shoulder pitch axis to elbow pitch axis (mm)
    L2: float = 95.0          # Elbow pitch axis to wrist pitch axis (mm)
    L3_grip: float = 150.0    # Wrist pitch axis to rubber grip pad center (mm)
    L3_tip: float = 168.0     # Wrist pitch axis to outer claw fingertip (mm)

    # Dynamic Cartesian tabletop depth offset (mm, compensates for suction cup/mounting differences)
    z_depth_trim_mm: float = 0.0

    # Physical gravity droop / cantilever compliance compensation (mm)
    gravity_sag_trim_mm: float = 10.0
    reach_scale_factor: float = 0.05

    # Servo zero-reference angles (degrees when servo is at its nominal center tick)
    shoulder_zero_deg: float = 90.0  # Angle above horizontal at center tick (upright vertical)
    elbow_zero_deg: float = 180.0    # Interior angle between L1 and L2 at center tick (straight collinear)
    wrist_zero_deg: float = 0.0      # Relative angle with forearm at center tick (collinear)
    base_zero_deg: float = 0.0       # Arm-relative angle at center tick (default 0 deg)
    base_heading_deg: float = 0.0    # Base mounting nominal heading in world frame (East = 0 deg)

    # Dynamic nominal center ticks (loaded from xarm_hardware.yaml, defaults to 500)
    nominal_center_ticks: Optional[Dict[int, int]] = None

    # Motor direction multipliers (+1 standard, -1 inverted)
    direction_multipliers: Optional[Dict[int, int]] = None

    # Soft safety tick clamps
    min_safe_ticks: Optional[Dict[int, int]] = None
    max_safe_ticks: Optional[Dict[int, int]] = None

    def __post_init__(self):
        """Set default safe tick clamps, center ticks and direction multipliers if not provided."""
        if self.nominal_center_ticks is None:
            self.nominal_center_ticks = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
        if self.direction_multipliers is None:
            self.direction_multipliers = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1}
        if self.min_safe_ticks is None:
            self.min_safe_ticks = {1: 250, 2: 130, 3: 140, 4: 140, 5: 140, 6: 120}
        if self.max_safe_ticks is None:
            self.max_safe_ticks = {1: 750, 2: 870, 3: 860, 4: 860, 5: 860, 6: 880}


DEFAULT_PARAMS = KinematicParameters()

# Legacy constant aliases for backward compatibility
D1 = DEFAULT_PARAMS.d1
L1 = DEFAULT_PARAMS.L1
L2 = DEFAULT_PARAMS.L2
L3 = DEFAULT_PARAMS.L3_grip
MIN_SAFE_TICKS = DEFAULT_PARAMS.min_safe_ticks
MAX_SAFE_TICKS = DEFAULT_PARAMS.max_safe_ticks


class IKSolution(NamedTuple):
    """Encapsulates result of analytical inverse kinematics computation."""

    success: bool
    ticks: Dict[int, int]
    angles_deg: Dict[int, float]
    message: str


class ArmPose(NamedTuple):
    """Cylindrical pose representation for xArm end-effector."""

    r: float       # mm
    theta: float   # deg
    z: float       # mm
    phi: float     # deg


def clip(val: float, min_v: float, max_v: float) -> float:
    """Clamps a numeric value to the interval [min_v, max_v]."""
    return max(min_v, min(max_v, val))


def load_kinematic_parameters(
    config_path: str = "config/xarm_hardware.yaml",
    arm_id: str = "xarm_1",
) -> KinematicParameters:
    """Load link dimensions and zero angles from hardware configuration YAML."""
    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join("/workspace", config_path),
        "config/xarm_hardware.yaml",
        os.path.join(os.getcwd(), "config/xarm_hardware.yaml"),
    ]
    chosen = None
    for c in candidates:
        if os.path.exists(c):
            chosen = c
            break

    if not chosen:
        return DEFAULT_PARAMS

    try:
        with open(chosen, "r") as f:
            data = yaml.safe_load(f)

        arm_cfg = data.get("manipulators", {}).get(arm_id, {})
        if not arm_cfg:
            if "xarm_1" in data.get("manipulators", {}):
                arm_cfg = data["manipulators"]["xarm_1"]
            elif "xarm" in data.get("manipulators", {}):
                arm_cfg = data["manipulators"]["xarm"]

        links = arm_cfg.get("link_lengths_mm", {})
        motors = arm_cfg.get("motors", {})

        d1 = float(links.get("base_to_shoulder_d1", links.get("base_to_shoulder_L1", 65.0)))
        l1 = float(links.get("upper_arm_L1", links.get("upper_arm_L2", 101.0)))
        l2 = float(links.get("forearm_L2", links.get("forearm_L3", 95.0)))
        l3_grip = float(links.get("tool_claw_grip_center_L3", links.get("tool_claw_L4", 150.0)))
        l3_tip = float(links.get("tool_claw_fingertip_L3", 168.0))

        sh_zero = float(motors.get("motor_5", {}).get("zero_reference_angle_deg", 90.0))
        el_zero = float(motors.get("motor_4", {}).get("zero_reference_angle_deg", 180.0))
        wr_zero = float(motors.get("motor_3", {}).get("zero_reference_angle_deg", 0.0))
        base_zero = float(motors.get("motor_6", {}).get("zero_reference_angle_deg", 0.0))
        base_mount = arm_cfg.get("base_mount_mm", {})
        base_heading = float(base_mount.get("nominal_heading_deg", 0.0))
        z_depth_trim = float(base_mount.get("z_depth_trim_mm", arm_cfg.get("z_depth_trim_mm", 0.0)))
        sag_trim = float(base_mount.get("gravity_sag_trim_mm", 10.0))
        reach_scale = float(base_mount.get("reach_scale_factor", 0.05))

        min_ticks = {
            1: int(motors.get("motor_1", {}).get("min_tick", 150)),
            2: int(motors.get("motor_2", {}).get("min_tick", 200)),
            3: int(motors.get("motor_3", {}).get("min_tick", 150)),
            4: int(motors.get("motor_4", {}).get("min_tick", 150)),
            5: int(motors.get("motor_5", {}).get("min_tick", 200)),
            6: int(motors.get("motor_6", {}).get("min_tick", 150)),
        }
        max_ticks = {
            1: int(motors.get("motor_1", {}).get("max_tick", 800)),
            2: int(motors.get("motor_2", {}).get("max_tick", 800)),
            3: int(motors.get("motor_3", {}).get("max_tick", 850)),
            4: int(motors.get("motor_4", {}).get("max_tick", 850)),
            5: int(motors.get("motor_5", {}).get("max_tick", 850)),
            6: int(motors.get("motor_6", {}).get("max_tick", 850)),
        }

        center_ticks = {
            1: int(motors.get("motor_1", {}).get("nominal_center_tick", 500)),
            2: int(motors.get("motor_2", {}).get("nominal_center_tick", 500)),
            3: int(motors.get("motor_3", {}).get("nominal_center_tick", 500)),
            4: int(motors.get("motor_4", {}).get("nominal_center_tick", 500)),
            5: int(motors.get("motor_5", {}).get("nominal_center_tick", 500)),
            6: int(motors.get("motor_6", {}).get("nominal_center_tick", 500)),
        }

        dir_multipliers = {
            1: int(motors.get("motor_1", {}).get("direction_multiplier", 1)),
            2: int(motors.get("motor_2", {}).get("direction_multiplier", 1)),
            3: int(motors.get("motor_3", {}).get("direction_multiplier", 1)),
            4: int(motors.get("motor_4", {}).get("direction_multiplier", 1)),
            5: int(motors.get("motor_5", {}).get("direction_multiplier", 1)),
            6: int(motors.get("motor_6", {}).get("direction_multiplier", 1)),
        }

        return KinematicParameters(
            d1=d1,
            L1=l1,
            L2=l2,
            L3_grip=l3_grip,
            L3_tip=l3_tip,
            z_depth_trim_mm=z_depth_trim,
            gravity_sag_trim_mm=sag_trim,
            reach_scale_factor=reach_scale,
            shoulder_zero_deg=sh_zero,
            elbow_zero_deg=el_zero,
            wrist_zero_deg=wr_zero,
            base_zero_deg=base_zero,
            base_heading_deg=base_heading,
            nominal_center_ticks=center_ticks,
            direction_multipliers=dir_multipliers,
            min_safe_ticks=min_ticks,
            max_safe_ticks=max_ticks,
        )
    except Exception:
        return DEFAULT_PARAMS


def compute_natural_pitch(r: float, z: float) -> float:
    """
    Compute kinematically optimal approach pitch phi (degrees) based on reach and height.

    - For near-base picking (r < 160mm, Z < 50mm): targets -74.0° for extreme near-base clearance.
    - For inner-mid tabletop picking (160 <= r < 210mm, Z < 50mm): blends -70° to -64° so Motor 4
      stays >= 165 ticks (well clear of its ~153 mechanical hard stop) with smooth shoulder progression.
    - For mid-to-far tabletop picking (Z < 50mm): transitions smoothly toward shallower angles.
    - For elevated transit (Z >= 50mm): transitions toward horizontal clearance.
    """
    if r < 160.0 and z < 50.0:
        return -74.0

    if z < 50.0:
        if r < 210.0:
            phi = -70.0 + 0.12 * (r - 160.0) + 0.10 * (z - 21.0)
        else:
            phi = -64.0 + 0.24 * (r - 210.0) + 0.10 * (z - 21.0)
        return clip(phi, -75.0, -25.0)

    # Elevated heights
    phi = -35.0 + 0.20 * (r - 220.0) + 0.25 * (z - 50.0)
    return clip(phi, -60.0, 0.0)


def solve_ik(
    r: float,
    theta: float,
    z: float,
    phi: Optional[float] = None,
    claw_tick: int = 500,
    roll_tick: int = 500,
    auto_pitch_fallback: bool = True,
    params: Optional[KinematicParameters] = None,
    use_claw_tip: bool = False,
    compensate_sag: bool = False,
) -> IKSolution:
    """
    Compute analytical inverse kinematics for target (r, theta, z) in cylindrical coordinates.

    :param r: Radial distance from base Z-axis to gripper (mm)
    :param theta: Base yaw angle (degrees, 0 = forward along +X)
    :param z: Gripper height above table floor (mm)
    :param phi: Gripper pitch angle (degrees relative to table, 0 = level horizontal).
                If None, automatically selected via compute_natural_pitch(r, z).
    :param claw_tick: Desired claw position tick (~230 open, ~605 grip 25mm cube, ~780 closed)
    :param roll_tick: Desired wrist roll tick (500 = level)
    :param auto_pitch_fallback: If True and requested phi violates joint limits,
                                 searches nearby feasible pitches to find a valid solution.
    :param params: Optional KinematicParameters instance (defaults to DEFAULT_PARAMS).
    :param use_claw_tip: If True, uses L3_tip (168mm); if False, uses L3_grip (150mm).
    :param compensate_sag: If True, applies empirical radial reach trim to compensate for gravity droop.
    :return: IKSolution with success flag, tick mapping, and joint angles.
    """
    p = params if params is not None else DEFAULT_PARAMS

    effective_r = r
    if compensate_sag:
        effective_r = r + (p.gravity_sag_trim_mm + p.reach_scale_factor * max(0.0, r - 150.0))

    if phi is None:
        target_phi = compute_natural_pitch(effective_r, z)
    else:
        target_phi = phi

    sol = _solve_ik_internal(effective_r, theta, z, target_phi, claw_tick, roll_tick, p, use_claw_tip)
    if not sol.success and auto_pitch_fallback:
        # Fine-grained outward pitch search (±1° to ±25°) to guarantee convergence in tight near-base envelopes
        search_deltas = [d for delta in range(1, 26) for d in (-delta, delta)]
        for delta in search_deltas:
            candidate_phi = clip(target_phi + delta, -85.0, 0.0)
            alt_sol = _solve_ik_internal(
                effective_r, theta, z, candidate_phi, claw_tick, roll_tick, p, use_claw_tip
            )
            if alt_sol.success:
                return alt_sol

    if not sol.success and compensate_sag:
        return solve_ik(
            r=r,
            theta=theta,
            z=z,
            phi=phi,
            claw_tick=claw_tick,
            roll_tick=roll_tick,
            auto_pitch_fallback=auto_pitch_fallback,
            params=params,
            use_claw_tip=use_claw_tip,
            compensate_sag=False,
        )

    return sol


def _solve_ik_internal(
    r: float,
    theta: float,
    z: float,
    phi: float,
    claw_tick: int,
    roll_tick: int,
    params: KinematicParameters,
    use_claw_tip: bool = False,
) -> IKSolution:
    """Solve analytical inverse kinematics internally using specified parameters."""
    phi_rad = math.radians(phi)
    l3 = params.L3_tip if use_claw_tip else params.L3_grip
    l1 = params.L1
    l2 = params.L2
    d1 = params.d1

    # Step 1: Wrist position calculation (with dynamic Cartesian Z-depth trim)
    z_effective = z - params.z_depth_trim_mm
    r_w = r - l3 * math.cos(phi_rad)
    z_w = z_effective - l3 * math.sin(phi_rad)

    # Offset from shoulder joint (r=0, z=d1)
    r_prime = r_w
    z_prime = z_w - d1
    d_sq = r_prime * r_prime + z_prime * z_prime
    d = math.sqrt(d_sq)

    # Step 2: Reachability check
    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    if d > max_reach:
        msg = f"Target unreachable: reach D={d:.1f}mm exceeds max {max_reach:.1f}mm."
        return IKSolution(
            success=False,
            ticks={},
            angles_deg={},
            message=msg,
        )

    if d < min_reach:
        msg = f"Target unreachable: reach D={d:.1f}mm is inside inner limit {min_reach:.1f}mm."
        return IKSolution(
            success=False,
            ticks={},
            angles_deg={},
            message=msg,
        )

    if r_prime < 0:
        return IKSolution(
            success=False,
            ticks={},
            angles_deg={},
            message=f"Target unreachable: wrist position r_w={r_prime:.1f}mm behind shoulder.",
        )

    # Step 3: Elbow angle (Law of Cosines, Elbow-Up configuration)
    cos_alpha3 = (l1 * l1 + l2 * l2 - d_sq) / (2.0 * l1 * l2)
    alpha3_rad = math.acos(clip(cos_alpha3, -1.0, 1.0))
    alpha3_deg = math.degrees(alpha3_rad)

    # Elbow bend angle relative to straight extension
    theta_elbow_bend_deg = 180.0 - alpha3_deg

    # Step 4: Shoulder angle
    beta_rad = math.atan2(z_prime, r_prime)
    cos_psi = (l1 * l1 + d_sq - l2 * l2) / (2.0 * l1 * d)
    psi_rad = math.acos(clip(cos_psi, -1.0, 1.0))

    theta_shoulder_rad = beta_rad + psi_rad
    theta_shoulder_deg = math.degrees(theta_shoulder_rad)

    # Step 5: Wrist Pitch angle
    # Forearm orientation relative to horizontal
    theta_forearm_deg = theta_shoulder_deg - theta_elbow_bend_deg
    # Wrist joint relative angle to achieve target pitch phi
    theta_wrist_rel_deg = phi - theta_forearm_deg

    # Step 6: Convert angles to theoretical servo ticks using configured zero-references
    dir_mult = params.direction_multipliers or {i: 1 for i in range(1, 7)}
    centers = params.nominal_center_ticks or {i: 500 for i in range(1, 7)}

    # Motor 6: Base Yaw (dynamic center = configured base_zero_deg)
    tick_6 = int(round(centers.get(6, 500) + dir_mult.get(6, 1) * (theta - params.base_zero_deg) * TICKS_PER_DEG_YAW))

    # Motor 5: Shoulder Pitch (configured zero angle, 90° upright at center tick, 4.0 ticks/deg)
    # Tilting forward toward horizontal decreases theta_shoulder_deg and increases ticks
    tick_5 = int(round(centers.get(5, 500) + dir_mult.get(5, 1) * (params.shoulder_zero_deg - theta_shoulder_deg) * TICKS_PER_DEG_PITCH))

    # Motor 4: Elbow Pitch (configured zero interior angle, 180° collinear at center tick, 4.0 ticks/deg)
    # Folding forward decreases interior angle alpha3_deg and decreases ticks
    tick_4 = int(round(centers.get(4, 500) - dir_mult.get(4, 1) * (params.elbow_zero_deg - alpha3_deg) * TICKS_PER_DEG_PITCH))

    # Motor 3: Wrist Pitch (configured zero relative angle, 0° collinear at center tick, 4.0 ticks/deg)
    # Pitching downward relative to forearm decreases theta_wrist_rel_deg and increases ticks
    tick_3 = int(round(centers.get(3, 500) - dir_mult.get(3, 1) * (theta_wrist_rel_deg - params.wrist_zero_deg) * TICKS_PER_DEG_PITCH))

    # Motor 2: Wrist Roll
    tick_2 = roll_tick

    # Motor 1: Claw Gripper
    tick_1 = claw_tick

    ticks = {
        1: tick_1,
        2: tick_2,
        3: tick_3,
        4: tick_4,
        5: tick_5,
        6: tick_6,
    }

    angles_deg = {
        6: theta,
        5: theta_shoulder_deg,
        4: alpha3_deg,
        3: theta_wrist_rel_deg,
        2: 0.0,
        1: float(claw_tick),
        0: phi,  # gripper total pitch
    }

    # Step 7: Check soft physical limits
    violations = []
    min_ticks = params.min_safe_ticks or MIN_SAFE_TICKS
    max_ticks = params.max_safe_ticks or MAX_SAFE_TICKS

    for mid, t in ticks.items():
        min_t = min_ticks.get(mid, 150)
        max_t = max_ticks.get(mid, 850)
        if t < min_t or t > max_t:
            violations.append(f"Motor {mid} tick {t} out of range [{min_t}, {max_t}]")

    if violations:
        return IKSolution(
            success=False,
            ticks=ticks,
            angles_deg=angles_deg,
            message="; ".join(violations),
        )

    return IKSolution(
        success=True,
        ticks=ticks,
        angles_deg=angles_deg,
        message="Valid IK solution found.",
    )


def solve_fk(
    tick_6: int,
    tick_5: int,
    tick_4: int,
    tick_3: int,
    params: Optional[KinematicParameters] = None,
    use_claw_tip: bool = False,
) -> ArmPose:
    """Compute Forward Kinematics from joint ticks to cylindrical pose (r, theta, z, phi)."""
    p = params if params is not None else DEFAULT_PARAMS
    l3 = p.L3_tip if use_claw_tip else p.L3_grip
    l1 = p.L1
    l2 = p.L2
    d1 = p.d1

    dir_mult = p.direction_multipliers or {i: 1 for i in range(1, 7)}
    centers = p.nominal_center_ticks or {i: 500 for i in range(1, 7)}

    # Base Yaw
    theta = p.base_zero_deg + dir_mult.get(6, 1) * (tick_6 - centers.get(6, 500)) * DEG_PER_TICK_YAW

    # Shoulder (configured upright zero angle, increasing ticks tilts forward toward 0°)
    theta_shoulder_deg = p.shoulder_zero_deg - dir_mult.get(5, 1) * (tick_5 - centers.get(5, 500)) * DEG_PER_TICK_PITCH
    theta_shoulder_rad = math.radians(theta_shoulder_deg)

    # Elbow (configured collinear 180°, decreasing ticks folds forward)
    alpha3_deg = p.elbow_zero_deg + dir_mult.get(4, 1) * (tick_4 - centers.get(4, 500)) * DEG_PER_TICK_PITCH
    theta_elbow_bend_deg = 180.0 - alpha3_deg

    # Forearm angle
    theta_forearm_deg = theta_shoulder_deg - theta_elbow_bend_deg
    theta_forearm_rad = math.radians(theta_forearm_deg)

    # Wrist pitch (configured collinear 0°, increasing ticks tilts downward)
    theta_wrist_rel_deg = p.wrist_zero_deg - dir_mult.get(3, 1) * (tick_3 - centers.get(3, 500)) * DEG_PER_TICK_PITCH
    phi = theta_forearm_deg + theta_wrist_rel_deg
    phi_rad = math.radians(phi)

    # Planar forward kinematics
    r_elbow = l1 * math.cos(theta_shoulder_rad)
    z_elbow = d1 + l1 * math.sin(theta_shoulder_rad)

    r_wrist = r_elbow + l2 * math.cos(theta_forearm_rad)
    z_wrist = z_elbow + l2 * math.sin(theta_forearm_rad)

    r_tip = r_wrist + l3 * math.cos(phi_rad)
    z_tip = z_wrist + l3 * math.sin(phi_rad) + p.z_depth_trim_mm

    return ArmPose(r=r_tip, theta=theta, z=z_tip, phi=phi)


def world_to_arm_polar(
    x_w: float,
    y_w: float,
    z_w: float = 0.0,
    arm_id: str = "xarm_1",
    config_path: str = "config/xarm_hardware.yaml",
    params: Optional[KinematicParameters] = None,
) -> Tuple[float, float, float]:
    """
    Transform world coordinates (X_w, Y_w, Z_w in mm) into arm-relative cylindrical coordinates (r, theta_arm, z).

    :param x_w: Global workspace X in mm
    :param y_w: Global workspace Y in mm
    :param z_w: Global workspace Z in mm
    :param arm_id: Manipulator ID (default "xarm_1")
    :param config_path: Path to hardware configuration YAML
    :param params: Optional pre-loaded KinematicParameters instance
    :return: Tuple of (r_mm, theta_arm_deg, z_mm) where theta_arm is normalized to [-180, +180] deg.
    """
    p = params if params is not None else load_kinematic_parameters(config_path, arm_id)

    base_x = 175.0
    base_y = 375.0
    base_z = 0.0

    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join("/workspace", config_path),
        "config/xarm_hardware.yaml",
        os.path.join(os.getcwd(), "config/xarm_hardware.yaml"),
    ]
    for c in candidates:
        if os.path.exists(c):
            try:
                with open(c, "r") as f:
                    data = yaml.safe_load(f)
                arm_cfg = data.get("manipulators", {}).get(arm_id, {})
                mount = arm_cfg.get("base_mount_mm", {})
                base_x = float(mount.get("x", base_x))
                base_y = float(mount.get("y", base_y))
                base_z = float(mount.get("z", base_z))
            except Exception:
                pass
            break

    rel_x = x_w - base_x
    rel_y = y_w - base_y
    rel_z = z_w - base_z

    r = math.hypot(rel_x, rel_y)
    theta_world = math.degrees(math.atan2(rel_y, rel_x))
    theta_arm = theta_world - p.base_heading_deg
    theta_arm = (theta_arm + 180.0) % 360.0 - 180.0

    return r, theta_arm, rel_z


def get_compartment_target(
    color: str,
    arm_id: str = "xarm_1",
    config_path: str = "config/xarm_hardware.yaml",
) -> Optional[Dict]:
    # Automatically support both get_compartment_target("red", "xarm_1") and ("xarm_1", "red")
    if color in ["xarm_1", "arm_1", "arm1"] and arm_id.lower() in ["red", "green", "blue", "yellow"]:
        color, arm_id = arm_id, color

    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", config_path),
        os.path.join("/workspace", config_path),
        "config/xarm_hardware.yaml",
        os.path.join(os.getcwd(), "config/xarm_hardware.yaml"),
    ]
    chosen = None
    for c in candidates:
        if os.path.exists(c):
            chosen = os.path.abspath(c)
            break

    if not chosen:
        return None

    try:
        with open(chosen, "r") as f:
            data = yaml.safe_load(f)

        arm_cfg = data.get("manipulators", {}).get(arm_id, {})
        if not arm_cfg:
            if "xarm_1" in data.get("manipulators", {}):
                arm_cfg = data["manipulators"]["xarm_1"]
            elif "xarm" in data.get("manipulators", {}):
                arm_cfg = data["manipulators"]["xarm"]

        base_x = float(arm_cfg.get("base_mount_mm", {}).get("x", 200.0))
        base_y = float(arm_cfg.get("base_mount_mm", {}).get("y", 280.0))
        base_z = float(arm_cfg.get("base_mount_mm", {}).get("z", 0.0))

        compartments = arm_cfg.get("collection_box", {}).get("compartments", {})
        norm_color = color.strip().lower()

        matched = None
        for _, comp in compartments.items():
            if comp.get("color", "").strip().lower() == norm_color:
                matched = comp
                break

        if not matched:
            if norm_color in ["default", "single", "all", "container", "box"] and "target_world_mm" in arm_cfg.get("collection_box", {}):
                matched = {
                    "color": norm_color,
                    "target_world_mm": arm_cfg["collection_box"]["target_world_mm"],
                    "drop_yaw_tick": arm_cfg.get("collection_box", {}).get("base_yaw_drop_tick", 145),
                }
            else:
                return None

        target_w = matched.get("target_world_mm", {})
        wx = float(target_w.get("x", 30.0))
        wy = float(target_w.get("y", 300.0))
        wz = float(target_w.get("z", 35.0))

        # Vector relative to arm base mount
        rel_x = wx - base_x
        rel_y = wy - base_y
        rel_z = wz - base_z

        r = math.hypot(rel_x, rel_y)
        theta_world = math.degrees(math.atan2(rel_y, rel_x))

        params = load_kinematic_parameters(chosen, arm_id)
        theta_arm = theta_world - params.base_heading_deg
        theta_arm = (theta_arm + 180.0) % 360.0 - 180.0

        # Attempt analytical IK solution for the compartment
        ik_sol = solve_ik(r=r, theta=theta_arm, z=rel_z, params=params)

        # Color-specific drop pose preset (e.g. drop_container_red, drop_container_green)
        if norm_color == "red":
            drop_ticks_list = arm_cfg.get("poses", {}).get("drop_container_red", {}).get("ticks", [226, 503, 149, 629, 347, 650])
            preset_yaw = 650
        elif norm_color == "green":
            drop_ticks_list = arm_cfg.get("poses", {}).get("drop_container_green", {}).get("ticks", [226, 503, 149, 629, 347, 354])
            preset_yaw = 354
        else:
            pose_key = f"drop_container_{norm_color}"
            drop_pose = arm_cfg.get("poses", {}).get(pose_key, arm_cfg.get("poses", {}).get("drop_container", {}))
            drop_ticks_list = drop_pose.get("ticks", [226, 503, 149, 629, 347, 650])
            preset_yaw = matched.get("drop_yaw_tick", 650)

        calibrated_ticks = {i + 1: drop_ticks_list[i] for i in range(min(6, len(drop_ticks_list)))}

        return {
            "color": norm_color,
            "world_xyz": {"x": wx, "y": wy, "z": wz},
            "rel_xyz": {"x": rel_x, "y": rel_y, "z": rel_z},
            "r": r,
            "theta_deg": theta_arm,
            "theta_world_deg": theta_world,
            "theta_arm_deg": theta_arm,
            "z": rel_z,
            "drop_yaw_tick": preset_yaw,
            "ik_success": True,
            "ik_ticks": calibrated_ticks,
            "ik_message": f"Using calibrated rear-flip drop preset for {norm_color.upper()} container",
            "drop_pose_preset": drop_ticks_list,
        }
    except Exception:
        return None
