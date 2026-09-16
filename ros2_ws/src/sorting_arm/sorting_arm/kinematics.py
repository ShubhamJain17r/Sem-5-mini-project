#!/usr/bin/env python3
"""
Analytical Inverse and Forward Kinematics for LewanSoul / Hiwonder xArm 1S.

Coordinates:
  Cylindrical/Polar coordinates (r, theta, z) relative to arm base center (Z=0 on table):
    - r     : radial distance from base vertical axis to fingertip grasp center (mm)
    - theta : base azimuth angle (degrees, 0 = forward along +X)
    - z     : vertical height above table (mm)
    - phi   : gripper pitch angle relative to horizontal (degrees, 0 = parallel to table)

Link Dimensions (Hiwonder xArm 1S):
  - d1 : Base to shoulder pitch axis = 65.0 mm
  - L1 : Upper arm (shoulder to elbow) = 101.0 mm
  - L2 : Forearm (elbow to wrist pitch) = 95.0 mm
  - L3 : Tool / Claw (wrist pitch to fingertip center) = 150.0 mm
"""

import math
from typing import Dict, NamedTuple, Optional

# Link lengths in millimeters
D1 = 65.0    # Base to shoulder
L1 = 101.0   # Upper arm
L2 = 95.0    # Forearm
L3 = 150.0   # Wrist to gripper tip

# Ticks per degree (1000 ticks / 240 degrees)
TICKS_PER_DEG = 1000.0 / 240.0
DEG_PER_TICK = 240.0 / 1000.0

# Physical limits
MIN_SAFE_TICKS = {1: 150, 2: 200, 3: 150, 4: 150, 5: 200, 6: 150}
MAX_SAFE_TICKS = {1: 800, 2: 800, 3: 850, 4: 850, 5: 850, 6: 850}


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


def compute_natural_pitch(r: float, z: float) -> float:
    """
    Compute kinematically optimal approach pitch phi (degrees) based on reach and height.

    - At mid-to-far reach (r >= 250mm), returns 0.0° (level horizontal pick).
    - At close reach (r <= 160mm, z <= 60mm), gently tilts downward (-30° to 0°)
      to keep Motor 3 (Wrist Pitch) comfortably within its physical travel window.
    """
    if r >= 250.0 and z >= 40.0:
        return 0.0

    # Closer range requires downward incline
    if r < 250.0:
        factor = (250.0 - r) / (250.0 - 150.0)
        factor = clip(factor, 0.0, 1.0)
        pitch = -25.0 * factor
        # Higher Z requires less downward tilt
        if z > 100.0:
            pitch *= max(0.0, (160.0 - z) / 60.0)
        return pitch

    return 0.0


def solve_ik(
    r: float,
    theta: float,
    z: float,
    phi: Optional[float] = None,
    claw_tick: int = 700,
    roll_tick: int = 500,
    auto_pitch_fallback: bool = True,
) -> IKSolution:
    """
    Compute analytical inverse kinematics for target (r, theta, z) in cylindrical coordinates.

    :param r: Radial distance from base Z-axis to gripper center (mm)
    :param theta: Base yaw angle (degrees, 0 = forward along +X)
    :param z: Gripper height above table floor (mm)
    :param phi: Gripper pitch angle (degrees relative to table, 0 = level horizontal).
                If None, automatically selected via compute_natural_pitch(r, z).
    :param claw_tick: Desired claw position tick (150 closed, 750 open)
    :param roll_tick: Desired wrist roll tick (500 = level)
    :param auto_pitch_fallback: If True and requested phi violates Motor 3 limits,
                                 attempts natural pitch as fallback.
    :return: IKSolution with success flag, tick mapping, and joint angles.
    """
    if phi is None:
        target_phi = compute_natural_pitch(r, z)
    else:
        target_phi = phi

    sol = _solve_ik_internal(r, theta, z, target_phi, claw_tick, roll_tick)
    if not sol.success and auto_pitch_fallback and phi is not None:
        alt_phi = compute_natural_pitch(r, z)
        if abs(alt_phi - phi) > 1.0:
            alt_sol = _solve_ik_internal(r, theta, z, alt_phi, claw_tick, roll_tick)
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
) -> IKSolution:
    phi_rad = math.radians(phi)

    # Step 1: Wrist position calculation
    r_w = r - L3 * math.cos(phi_rad)
    z_w = z - L3 * math.sin(phi_rad)

    # Offset from shoulder joint (r=0, z=D1)
    r_prime = r_w
    z_prime = z_w - D1
    d_sq = r_prime * r_prime + z_prime * z_prime
    d = math.sqrt(d_sq)

    # Step 2: Reachability check
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

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
            message=f"Target unreachable: wrist position r_w={r_prime:.1f}mm behind shoulder."
        )

    # Step 3: Elbow angle (Law of Cosines, Elbow-Up configuration)
    cos_alpha3 = (L1 * L1 + L2 * L2 - d_sq) / (2.0 * L1 * L2)
    alpha3_rad = math.acos(clip(cos_alpha3, -1.0, 1.0))
    alpha3_deg = math.degrees(alpha3_rad)

    # Elbow bend angle relative to straight extension
    theta_elbow_bend_deg = 180.0 - alpha3_deg

    # Step 4: Shoulder angle
    beta_rad = math.atan2(z_prime, r_prime)
    cos_psi = (L1 * L1 + d_sq - L2 * L2) / (2.0 * L1 * d)
    psi_rad = math.acos(clip(cos_psi, -1.0, 1.0))

    theta_shoulder_rad = beta_rad + psi_rad
    theta_shoulder_deg = math.degrees(theta_shoulder_rad)

    # Step 5: Wrist Pitch angle
    # Forearm orientation relative to horizontal
    theta_forearm_deg = theta_shoulder_deg - theta_elbow_bend_deg
    # Wrist joint relative angle to achieve target pitch phi
    theta_wrist_rel_deg = phi - theta_forearm_deg

    # Step 6: Convert angles to theoretical servo ticks
    # Motor 6: Base Yaw (500 = center = 0 deg)
    tick_6 = int(round(500 + theta * TICKS_PER_DEG))

    # Motor 5: Shoulder Pitch (500 = 45 deg forward, increasing ticks tilts backward/upright)
    tick_5 = int(round(500 + (theta_shoulder_deg - 45.0) * TICKS_PER_DEG))

    # Motor 4: Elbow Pitch (500 = 90 deg interior angle, increasing extends forward)
    tick_4 = int(round(500 + (alpha3_deg - 90.0) * TICKS_PER_DEG))

    # Motor 3: Wrist Pitch (500 = aligned with forearm, increasing tilts downward)
    tick_3 = int(round(500 - theta_wrist_rel_deg * TICKS_PER_DEG))

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
    for mid, t in ticks.items():
        min_t = MIN_SAFE_TICKS.get(mid, 150)
        max_t = MAX_SAFE_TICKS.get(mid, 850)
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
) -> ArmPose:
    """Compute Forward Kinematics from joint ticks to cylindrical pose (r, theta, z, phi)."""
    # Base Yaw
    theta = (tick_6 - 500) * DEG_PER_TICK

    # Shoulder
    theta_shoulder_deg = 45.0 + (tick_5 - 500) * DEG_PER_TICK
    theta_shoulder_rad = math.radians(theta_shoulder_deg)

    # Elbow
    alpha3_deg = 90.0 + (tick_4 - 500) * DEG_PER_TICK
    theta_elbow_bend_deg = 180.0 - alpha3_deg

    # Forearm angle
    theta_forearm_deg = theta_shoulder_deg - theta_elbow_bend_deg
    theta_forearm_rad = math.radians(theta_forearm_deg)

    # Wrist pitch
    theta_wrist_rel_deg = -(tick_3 - 500) * DEG_PER_TICK
    phi = theta_forearm_deg + theta_wrist_rel_deg
    phi_rad = math.radians(phi)

    # Planar forward kinematics
    r_elbow = L1 * math.cos(theta_shoulder_rad)
    z_elbow = D1 + L1 * math.sin(theta_shoulder_rad)

    r_wrist = r_elbow + L2 * math.cos(theta_forearm_rad)
    z_wrist = z_elbow + L2 * math.sin(theta_forearm_rad)

    r_tip = r_wrist + L3 * math.cos(phi_rad)
    z_tip = z_wrist + L3 * math.sin(phi_rad)

    return ArmPose(r=r_tip, theta=theta, z=z_tip, phi=phi)
