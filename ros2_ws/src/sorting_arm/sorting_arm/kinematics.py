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

# Standard Ticks per degree (1000 ticks / 240 degrees)
TICKS_PER_DEG = 1000.0 / 240.0
DEG_PER_TICK = 240.0 / 1000.0


@dataclass
class KinematicParameters:
    """Configurable geometric and hardware parameters for an xArm manipulator."""

    d1: float = 65.0          # Base table surface to shoulder pitch axis (mm)
    L1: float = 101.0         # Shoulder pitch axis to elbow pitch axis (mm)
    L2: float = 95.0          # Elbow pitch axis to wrist pitch axis (mm)
    L3_grip: float = 150.0    # Wrist pitch axis to rubber grip pad center (mm)
    L3_tip: float = 168.0     # Wrist pitch axis to outer claw fingertip (mm)

    # Servo zero-reference angles (degrees when servo is at 500 ticks)
    shoulder_zero_deg: float = 45.0  # Angle above horizontal at 500 ticks
    elbow_zero_deg: float = 90.0     # Interior angle between L1 and L2 at 500 ticks
    wrist_zero_deg: float = 0.0      # Relative angle with forearm at 500 ticks
    base_zero_deg: float = 0.0       # Arm-relative angle at 500 ticks (default 0 deg)
    base_heading_deg: float = 0.0    # Base mounting nominal heading in world frame (East = 0 deg)

    # Soft safety tick clamps
    min_safe_ticks: Optional[Dict[int, int]] = None
    max_safe_ticks: Optional[Dict[int, int]] = None

    def __post_init__(self):
        """Set default safe tick clamps if not provided."""
        if self.min_safe_ticks is None:
            self.min_safe_ticks = {1: 150, 2: 200, 3: 150, 4: 150, 5: 200, 6: 150}
        if self.max_safe_ticks is None:
            self.max_safe_ticks = {1: 800, 2: 800, 3: 850, 4: 850, 5: 850, 6: 850}


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
    config_path: str = "config/dual_xarm_hardware.yaml",
    arm_id: str = "xarm_1",
) -> KinematicParameters:
    """Load link dimensions and zero angles from hardware configuration YAML."""
    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join("/workspace", config_path),
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
        links = arm_cfg.get("link_lengths_mm", {})
        motors = arm_cfg.get("motors", {})

        d1 = float(links.get("base_to_shoulder_d1", links.get("base_to_shoulder_L1", 65.0)))
        l1 = float(links.get("upper_arm_L1", links.get("upper_arm_L2", 101.0)))
        l2 = float(links.get("forearm_L2", links.get("forearm_L3", 95.0)))
        l3_grip = float(links.get("tool_claw_grip_center_L3", links.get("tool_claw_L4", 150.0)))
        l3_tip = float(links.get("tool_claw_fingertip_L3", 168.0))

        sh_zero = float(motors.get("motor_5", {}).get("zero_reference_angle_deg", 45.0))
        el_zero = float(motors.get("motor_4", {}).get("zero_reference_angle_deg", 90.0))
        wr_zero = float(motors.get("motor_3", {}).get("zero_reference_angle_deg", 0.0))
        base_zero = float(motors.get("motor_6", {}).get("zero_reference_angle_deg", 0.0))
        base_heading = float(arm_cfg.get("base_mount_mm", {}).get("nominal_heading_deg", 0.0))

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

        return KinematicParameters(
            d1=d1,
            L1=l1,
            L2=l2,
            L3_grip=l3_grip,
            L3_tip=l3_tip,
            shoulder_zero_deg=sh_zero,
            elbow_zero_deg=el_zero,
            wrist_zero_deg=wr_zero,
            base_zero_deg=base_zero,
            base_heading_deg=base_heading,
            min_safe_ticks=min_ticks,
            max_safe_ticks=max_ticks,
        )
    except Exception:
        return DEFAULT_PARAMS


def compute_natural_pitch(r: float, z: float) -> float:
    """
    Compute kinematically optimal approach pitch phi (degrees) based on reach and height.

    - Far reach (r >= 260mm): returns 0.0° (level horizontal pick).
    - Mid-to-close reach (r < 260mm): smoothly inclines downward (-35° to -10°)
      to prevent the forearm from pointing vertically downward, keeping Motor 3
      comfortably within [300, 650] ticks and preventing wrist bracket collision.
    """
    if r >= 260.0 and z >= 50.0:
        return 0.0

    if r < 260.0:
        # Scale factor from 0.0 (r=260) to 1.0 (r=150)
        factor = clip((260.0 - r) / (260.0 - 150.0), 0.0, 1.0)
        pitch = -35.0 * factor

        # Lower Z elevation requires slightly more downward tilt to prevent wrist jamming
        if z < 60.0:
            pitch -= 5.0 * clip((60.0 - z) / 40.0, 0.0, 1.0)
        # Higher Z elevation can taper off tilt
        elif z > 120.0:
            pitch *= max(0.0, (180.0 - z) / 60.0)

        return clip(pitch, -45.0, 0.0)

    return 0.0


def solve_ik(
    r: float,
    theta: float,
    z: float,
    phi: Optional[float] = None,
    claw_tick: int = 700,
    roll_tick: int = 500,
    auto_pitch_fallback: bool = True,
    params: Optional[KinematicParameters] = None,
    use_claw_tip: bool = False,
) -> IKSolution:
    """
    Compute analytical inverse kinematics for target (r, theta, z) in cylindrical coordinates.

    :param r: Radial distance from base Z-axis to gripper (mm)
    :param theta: Base yaw angle (degrees, 0 = forward along +X)
    :param z: Gripper height above table floor (mm)
    :param phi: Gripper pitch angle (degrees relative to table, 0 = level horizontal).
                If None, automatically selected via compute_natural_pitch(r, z).
    :param claw_tick: Desired claw position tick (150 closed, 750 open)
    :param roll_tick: Desired wrist roll tick (500 = level)
    :param auto_pitch_fallback: If True and requested phi violates Motor 3 limits,
                                 attempts natural pitch as fallback.
    :param params: Optional KinematicParameters instance (defaults to DEFAULT_PARAMS).
    :param use_claw_tip: If True, uses L3_tip (168mm); if False, uses L3_grip (150mm).
    :return: IKSolution with success flag, tick mapping, and joint angles.
    """
    p = params if params is not None else DEFAULT_PARAMS

    if phi is None:
        target_phi = compute_natural_pitch(r, z)
    else:
        target_phi = phi

    sol = _solve_ik_internal(r, theta, z, target_phi, claw_tick, roll_tick, p, use_claw_tip)
    if not sol.success and auto_pitch_fallback and phi is not None:
        alt_phi = compute_natural_pitch(r, z)
        if abs(alt_phi - phi) > 1.0:
            alt_sol = _solve_ik_internal(
                r, theta, z, alt_phi, claw_tick, roll_tick, p, use_claw_tip
            )
            if alt_sol.success:
                return alt_sol

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

    # Step 1: Wrist position calculation
    r_w = r - l3 * math.cos(phi_rad)
    z_w = z - l3 * math.sin(phi_rad)

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
    # Motor 6: Base Yaw (500 = center = configured base_zero_deg)
    tick_6 = int(round(500 + (theta - params.base_zero_deg) * TICKS_PER_DEG))

    # Motor 5: Shoulder Pitch (configured zero angle, default 45° forward)
    tick_5 = int(round(500 + (theta_shoulder_deg - params.shoulder_zero_deg) * TICKS_PER_DEG))

    # Motor 4: Elbow Pitch (configured zero interior angle, default 90°)
    tick_4 = int(round(500 + (alpha3_deg - params.elbow_zero_deg) * TICKS_PER_DEG))

    # Motor 3: Wrist Pitch (configured zero relative angle, default 0° collinear)
    tick_3 = int(round(500 - (theta_wrist_rel_deg - params.wrist_zero_deg) * TICKS_PER_DEG))

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

    # Base Yaw
    theta = p.base_zero_deg + (tick_6 - 500) * DEG_PER_TICK

    # Shoulder
    theta_shoulder_deg = p.shoulder_zero_deg + (tick_5 - 500) * DEG_PER_TICK
    theta_shoulder_rad = math.radians(theta_shoulder_deg)

    # Elbow
    alpha3_deg = p.elbow_zero_deg + (tick_4 - 500) * DEG_PER_TICK
    theta_elbow_bend_deg = 180.0 - alpha3_deg

    # Forearm angle
    theta_forearm_deg = theta_shoulder_deg - theta_elbow_bend_deg
    theta_forearm_rad = math.radians(theta_forearm_deg)

    # Wrist pitch
    theta_wrist_rel_deg = p.wrist_zero_deg - (tick_3 - 500) * DEG_PER_TICK
    phi = theta_forearm_deg + theta_wrist_rel_deg
    phi_rad = math.radians(phi)

    # Planar forward kinematics
    r_elbow = l1 * math.cos(theta_shoulder_rad)
    z_elbow = d1 + l1 * math.sin(theta_shoulder_rad)

    r_wrist = r_elbow + l2 * math.cos(theta_forearm_rad)
    z_wrist = z_elbow + l2 * math.sin(theta_forearm_rad)

    r_tip = r_wrist + l3 * math.cos(phi_rad)
    z_tip = z_wrist + l3 * math.sin(phi_rad)

    return ArmPose(r=r_tip, theta=theta, z=z_tip, phi=phi)


def world_to_arm_polar(
    x_w: float,
    y_w: float,
    z_w: float = 0.0,
    arm_id: str = "xarm_1",
    config_path: str = "config/dual_xarm_hardware.yaml",
    params: Optional[KinematicParameters] = None,
) -> Tuple[float, float, float]:
    """
    Transform world coordinates (X_w, Y_w, Z_w in mm) into arm-relative cylindrical coordinates (r, theta_arm, z).

    :param x_w: Global workspace X in mm
    :param y_w: Global workspace Y in mm
    :param z_w: Global workspace Z in mm
    :param arm_id: "xarm_1" or "xarm_2"
    :param config_path: Path to hardware configuration YAML
    :param params: Optional pre-loaded KinematicParameters instance
    :return: Tuple of (r_mm, theta_arm_deg, z_mm) where theta_arm is normalized to [-180, +180] deg.
    """
    p = params if params is not None else load_kinematic_parameters(config_path, arm_id)

    base_x = 170.0 if arm_id == "xarm_1" else 390.0
    base_y = 360.0
    base_z = 0.0

    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join("/workspace", config_path),
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
    config_path: str = "config/dual_xarm_hardware.yaml",
) -> Optional[Dict]:
    """
    Calculate world and arm-relative cylindrical coordinates for a color compartment.

    Returns dict with keys:
      'color', 'world_xyz', 'rel_xyz', 'r', 'theta_deg', 'theta_world_deg', 'theta_arm_deg',
      'z', 'drop_yaw_tick', 'ik_success', 'ik_ticks', 'ik_message', 'drop_pose_preset'.
    """
    candidates = [
        config_path,
        os.path.join(os.getcwd(), config_path),
        os.path.join("/workspace", config_path),
    ]
    chosen = None
    for c in candidates:
        if os.path.exists(c):
            chosen = c
            break

    if not chosen:
        return None

    try:
        with open(chosen, "r") as f:
            data = yaml.safe_load(f)

        arm_cfg = data.get("manipulators", {}).get(arm_id, {})
        base_x = float(arm_cfg.get("base_mount_mm", {}).get("x", 170.0 if arm_id == "xarm_1" else 390.0))
        base_y = float(arm_cfg.get("base_mount_mm", {}).get("y", 360.0))
        base_z = float(arm_cfg.get("base_mount_mm", {}).get("z", 0.0))

        compartments = arm_cfg.get("collection_box", {}).get("compartments", {})
        norm_color = color.strip().lower()

        matched = None
        for _, comp in compartments.items():
            if comp.get("color", "").strip().lower() == norm_color:
                matched = comp
                break

        if not matched:
            return None

        target_w = matched.get("target_world_mm", {})
        wx = float(target_w.get("x", 30.0 if arm_id == "xarm_1" else 490.0))
        wy = float(target_w.get("y", 380.0))
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

        # Fallback preset values
        drop_pose = arm_cfg.get("poses", {}).get("drop_container", {})
        preset_yaw = matched.get("drop_yaw_tick", arm_cfg.get("collection_box", {}).get("base_yaw_drop_tick", 820))

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
            "ik_success": ik_sol.success,
            "ik_ticks": ik_sol.ticks if ik_sol.success else None,
            "ik_message": ik_sol.message,
            "drop_pose_preset": drop_pose.get("ticks", None),
        }
    except Exception:
        return None
