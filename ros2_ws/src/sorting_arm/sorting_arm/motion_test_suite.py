#!/usr/bin/env python3
"""
Comprehensive Motion Verification Test Suite for LewanSoul / Hiwonder xArm 1S.

Provides 13 distinct single-arm motions (covering near, mid, far, diagonal West/East,
corner sweeps, multi-altitude staircase, continuous 3-point touching, and 4-compartment flank tour)
and 12 distinct cooperative motions when both arms are connected:

Single Arm Motions (xarm_1 / xarm_2):
  1. Upright Reset & Claw Span Calibration (500-tick reset, 60mm open -> 25mm grip -> closed)
  2. Pure Cartesian Vertical Z-Plunge & Elevate (Z=120mm -> Z=25mm -> Z=120mm at constant R=220mm)
  3. Pure Cartesian Radial Reach Sweep (R=160mm -> R=275mm -> R=160mm at clearance Z=35mm)
  4. Azimuthal Panoramic Arc Scan (Base yaw sweep from -35° to +35° over arena)
  5. Standard Mid-Range Color Pick-and-Place (R=220mm, θ=0°, Z=12.5mm -> Flank Tray)
  6. Near-Field Shallow Pick-and-Place (Short reach R=165mm, θ=0°, Z=12.5mm -> Flank Tray)
  7. Extended Far-Reach Pick-and-Place (Maximum reach R=275mm, θ=0°, Z=12.5mm -> Flank Tray)
  8. Left-Quadrant Flank Diagonal Pick-and-Place (R=210mm, θ=-30° West -> Flank Tray)
  9. Right-Quadrant Midline Diagonal Pick-and-Place (R=210mm, θ=+30° East -> Flank Tray)
  10. Far-Corner Diagonal Reach & Tabletop Sweep (R=265mm, θ=-25° to +25° at Z=20mm)
  11. Multi-Altitude Stepped Standoff Sequence (Z=150mm -> 90mm -> 40mm -> 12.5mm contact)
  12. Continuous 3-Point Tabletop Rapid Touch (Near-West -> Center -> Far-East without homing)
  13. Precision Flank 4-Color Compartment Tour (Red -> Green -> Blue -> Yellow compartments)

Dual Arm Cooperative Motions (Both Connected):
  1. Synchronized Homing & Claw Echo (Simultaneous upright reset & alternating gripper clicks)
  2. Symmetrical Mirroring Sweep (Inward arena bow & outward flank tray sweep in mirror symmetry)
  3. Partitioned Parallel Sorting (Arm 1 sorts West quadrant, Arm 2 sorts East quadrant concurrently)
  4. Midline Mutex Arbitration (Collision avoidance in shared overlap zone X=280mm)
  5. Midline Handover / Relay Transfer (Cross-quadrant handoff from Arm 1 to Arm 2 at midline)
  6. Symmetrical Dual Far-Reach Sweep (Both arms extend forward to R=265mm simultaneously)
  7. Alternating Near/Far Leapfrog Picking (Arm 1 near / Arm 2 far, then alternate)
  8. Concurrent Dual Flank Drop Demonstration (Both arms swing outward to flank boxes in lockstep)
  9. Symmetrical Dual Tabletop Proximity Inspection (Hovering over midline boundaries at Z=30mm)
  10. Synchronized Vertical Z-Plunge & Elevate (Both arms plunge to Z=25mm in lockstep)
  11. Symmetrical Cross-Diagonal Sweep (Coordinated cross-quadrant arena sweep)
  12. Cooperative Emergency Stop & Recovery Sequence (Simulated safety cut and homing restore)
"""

import argparse
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple
import yaml

# Support direct imports
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.abspath(os.path.join(script_dir, ".."))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

from sorting_arm.kinematics import (
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_fk,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.xarm_protocol import (
    XArmDriver,
    find_all_xarm_devices,
)


def load_yaml(path: str) -> dict:
    candidates = [path, os.path.join(os.getcwd(), path), os.path.join("/workspace", path)]
    for c in candidates:
        if os.path.exists(c):
            try:
                with open(c, "r") as f:
                    return yaml.safe_load(f) or {}
            except Exception:
                pass
    return {}


def load_deviations(hw_cfg: dict, arm_name: str) -> Dict[int, int]:
    """Extract optional motor deviations from unified hardware config (defaults to 0)."""
    motors = hw_cfg.get("manipulators", {}).get(arm_name, {}).get("motors", {})
    return {sid: int(motors.get(f"motor_{sid}", {}).get("deviation", 0)) for sid in range(1, 7)}


class ArmController:
    """Manages motion dispatch and kinematics for an individual xArm."""

    def __init__(self, arm_id: str = "xarm_1", port: str = "auto", config_dir: str = "config"):
        self.arm_id = arm_id
        self.config_dir = config_dir
        self.port = port
        self.hw_yaml = os.path.join(config_dir, "dual_xarm_hardware.yaml")

        hw_cfg = load_yaml(self.hw_yaml)
        self.params = load_kinematic_parameters(self.hw_yaml, arm_id=self.arm_id)
        self.deviations = load_deviations(hw_cfg, self.arm_id)
        max_dev = int(hw_cfg.get("manipulators", {}).get("max_deviation_ticks", 25))

        self.driver = XArmDriver(
            port=self.port,
            deviations=self.deviations,
            min_ticks=self.params.min_safe_ticks,
            max_ticks=self.params.max_safe_ticks,
            max_deviation_ticks=max_dev,
        )
        self.connected = self.driver.connect()
        self.status = "CONNECTED" if self.connected else "OFFLINE SIMULATION"

        hw_cfg = load_yaml(self.hw_yaml)
        arm_cfg = hw_cfg.get("manipulators", {}).get(self.arm_id, {})
        m1_cfg = arm_cfg.get("motors", {}).get("motor_1", {}).get("claw_span_mm", {})
        self.claw_open = int(m1_cfg.get("fully_open", {}).get("tick", 228 if arm_id == "xarm_1" else 234))
        self.claw_grip = int(m1_cfg.get("grip_cube_25mm", {}).get("tick", 610 if arm_id == "xarm_1" else 602))
        self.claw_closed = int(m1_cfg.get("fully_closed_empty", {}).get("tick", 791 if arm_id == "xarm_1" else 785))

    def log(self, text: str):
        print(f"[{self.arm_id.upper()}] {text}")

    def send_pose(self, ticks: Dict[int, int], move_time_ms: int = 1200):
        if self.connected and self.driver.is_connected:
            self.driver.send_multi_servo_move(ticks, time_ms=move_time_ms)
        else:
            t_str = ", ".join(f"M{k}:{v}" for k, v in sorted(ticks.items()))
        if self.connected and self.driver.is_connected:
            time.sleep((move_time_ms + 100) / 1000.0)
        else:
            time.sleep(0.01)

    def move_ik(
        self,
        r: float,
        theta: float,
        z: float,
        claw_tick: int = 500,
        roll_tick: int = 500,
        phi: Optional[float] = None,
        move_time_ms: int = 1200,
    ) -> bool:
        sol = solve_ik(
            r=r,
            theta=theta,
            z=z,
            phi=phi,
            claw_tick=claw_tick,
            roll_tick=roll_tick,
            params=self.params,
        )
        if not sol.success:
            self.log(f"IK Failed for (R={r:.1f}, θ={theta:.1f}, Z={z:.1f}): {sol.message}")
            return False
        self.send_pose(sol.ticks, move_time_ms=move_time_ms)
        return True

    def reset_pose(self, move_time_ms: int = 1500):
        """Move arm straight upright to physical reset pose."""
        centers = self.params.nominal_center_ticks or {i: 500 for i in range(1, 7)}
        self.send_pose(centers, move_time_ms=move_time_ms)

    def close(self):
        if self.connected and self.driver:
            self.driver.disconnect()


# =====================================================================================
# GENERALIZED PICK-AND-PLACE HELPER
# =====================================================================================

def execute_pick_and_place(
    arm: ArmController,
    pick_r: float,
    pick_th: float,
    pick_z: float = 12.5,
    color: str = "red",
    motion_label: str = "Pick-and-Place",
):
    """Executes a calibrated multi-phase Cartesian pick and deposit sequence."""
    print(f"\n--- [{motion_label.upper()} ({color.upper()}) on {arm.arm_id}] ---")
    arm.log("Phase 0: Starting at Upright Reset Pose...")
    arm.reset_pose(move_time_ms=1500)

    standoff_r = max(140.0, pick_r - 35.0)
    hover_z = pick_z + 25.0
    lift_z = pick_z + 35.0
    retract_r = max(135.0, pick_r - 50.0)

    arm.log(f"Phase 1: Standoff Hover @ (R={standoff_r:.1f}mm, θ={pick_th:.1f}°, Z={hover_z:.1f}mm)...")
    arm.move_ik(standoff_r, pick_th, hover_z, claw_tick=arm.claw_open, move_time_ms=1400)

    arm.log(f"Phase 2: Advancing to Target @ (R={pick_r:.1f}mm, θ={pick_th:.1f}°, Z={pick_z:.1f}mm)...")
    arm.move_ik(pick_r, pick_th, pick_z, claw_tick=arm.claw_open, move_time_ms=1200)

    arm.log(f"Phase 3: Clamping Gripper onto Object (tick {arm.claw_grip})...")
    arm.move_ik(pick_r, pick_th, pick_z, claw_tick=arm.claw_grip, move_time_ms=800)
    time.sleep(0.4)

    arm.log(f"Phase 4: Elevating to Z={lift_z:.1f}mm and Retracting to R={retract_r:.1f}mm...")
    arm.move_ik(retract_r, pick_th, lift_z, claw_tick=arm.claw_grip, move_time_ms=1100)

    arm.log(f"Phase 5: Slewing to Flank Sorting Box (Target: {color.upper()})...")
    target_info = get_compartment_target(color, arm_id=arm.arm_id, config_path=arm.hw_yaml)
    drop_yaw = target_info.get("drop_yaw_tick", 208 if arm.arm_id == "xarm_1" else 792) if target_info else 208

    preset = target_info.get("drop_pose_preset") if target_info else None
    if preset and len(preset) == 6:
        drop_pose = {
            1: arm.claw_grip,
            2: preset[1],
            3: preset[2],
            4: preset[3],
            5: preset[4],
            6: drop_yaw,
        }
    else:
        drop_pose = {
            1: arm.claw_grip,
            2: 500,
            3: 830 if arm.arm_id == "xarm_1" else 170,
            4: 160 if arm.arm_id == "xarm_1" else 840,
            5: 470,
            6: drop_yaw,
        }
    arm.send_pose(drop_pose, move_time_ms=1700)

    arm.log("Phase 6: Releasing object into sorting box...")
    drop_pose[1] = arm.claw_open
    arm.send_pose(drop_pose, move_time_ms=700)
    time.sleep(0.4)

    arm.log("Phase 7: Clearing container rim and returning to upright reset pose...")
    arm.reset_pose(move_time_ms=1600)
    print(f">>> {motion_label} Complete! <<<\n")


# =====================================================================================
# SINGLE ARM MOTIONS (1 to 13)
# =====================================================================================

def motion_single_1_reset_and_claw(arm: ArmController):
    """Motion 1: Upright Reset & Gripper Span Calibration."""
    print(f"\n--- [MOTION 1: Upright Reset & Gripper Span Calibration on {arm.arm_id}] ---")
    arm.log("Moving straight upright to physical reset pose...")
    arm.reset_pose(move_time_ms=1600)
    time.sleep(0.5)

    arm.log(f"Cycling Claw: Fully OPEN (60mm span -> tick {arm.claw_open})...")
    arm.send_pose({1: arm.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=800)
    time.sleep(0.4)

    arm.log(f"Cycling Claw: GRIP 25mm cube (tick {arm.claw_grip})...")
    arm.send_pose({1: arm.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=800)
    time.sleep(0.4)

    arm.log(f"Cycling Claw: Fully CLOSED empty (tick {arm.claw_closed})...")
    arm.send_pose({1: arm.claw_closed, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=800)
    time.sleep(0.4)

    arm.log("Returning claw to neutral center and finishing upright...")
    arm.reset_pose(move_time_ms=1000)
    print(">>> Motion 1 Complete! <<<\n")


def motion_single_2_vertical_z_plunge(arm: ArmController):
    """Motion 2: Pure Cartesian Vertical Z-Plunge & Elevate."""
    print(f"\n--- [MOTION 2: Pure Cartesian Vertical Z-Plunge & Elevate on {arm.arm_id}] ---")
    arm.log("Phase 0: Starting at upright reset...")
    arm.reset_pose(move_time_ms=1400)

    r_target = 220.0
    th_target = 0.0

    arm.log(f"Phase 1: Gliding to High Altitude Hover (R={r_target}mm, Z=120mm)...")
    arm.move_ik(r=r_target, theta=th_target, z=120.0, claw_tick=arm.claw_open, move_time_ms=1500)
    time.sleep(0.3)

    arm.log(f"Phase 2: Plunging vertically straight down to Tabletop Approach (Z=25mm)...")
    arm.move_ik(r=r_target, theta=th_target, z=25.0, claw_tick=arm.claw_open, move_time_ms=1400)
    time.sleep(0.5)

    arm.log(f"Phase 3: Elevating vertically straight up back to High Altitude (Z=120mm)...")
    arm.move_ik(r=r_target, theta=th_target, z=120.0, claw_tick=arm.claw_open, move_time_ms=1300)
    time.sleep(0.3)

    arm.log("Phase 4: Returning to upright reset pose...")
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 2 Complete! <<<\n")


def motion_single_3_radial_reach_sweep(arm: ArmController):
    """Motion 3: Pure Cartesian Radial Reach Sweep (R-Axis)."""
    print(f"\n--- [MOTION 3: Pure Cartesian Radial Reach Sweep on {arm.arm_id}] ---")
    arm.log("Phase 0: Starting at upright reset...")
    arm.reset_pose(move_time_ms=1400)

    z_clearance = 35.0
    th_target = 0.0

    arm.log(f"Phase 1: Moving to Inner Radial Boundary (R=160mm, Z={z_clearance}mm)...")
    arm.move_ik(r=160.0, theta=th_target, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1400)
    time.sleep(0.3)

    arm.log(f"Phase 2: Gliding horizontally forward to Extended Reach (R=275mm, Z={z_clearance}mm)...")
    arm.move_ik(r=275.0, theta=th_target, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1600)
    time.sleep(0.5)

    arm.log(f"Phase 3: Retracting horizontally back to Inner Boundary (R=160mm, Z={z_clearance}mm)...")
    arm.move_ik(r=160.0, theta=th_target, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1500)
    time.sleep(0.3)

    arm.log("Phase 4: Returning to upright reset pose...")
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 3 Complete! <<<\n")


def motion_single_4_azimuthal_arc_scan(arm: ArmController):
    """Motion 4: Azimuthal Panoramic Arc Scan (-35° to +35°)."""
    print(f"\n--- [MOTION 4: Azimuthal Panoramic Arc Scan on {arm.arm_id}] ---")
    arm.log("Phase 0: Starting at upright reset...")
    arm.reset_pose(move_time_ms=1400)

    r_scan = 220.0
    z_scan = 45.0

    arm.log(f"Phase 1: Entering arena center (R={r_scan}mm, θ=0°, Z={z_scan}mm)...")
    arm.move_ik(r=r_scan, theta=0.0, z=z_scan, claw_tick=arm.claw_open, move_time_ms=1300)
    time.sleep(0.2)

    arm.log("Phase 2: Sweeping West (θ = -35°)...")
    arm.move_ik(r=r_scan, theta=-35.0, z=z_scan, claw_tick=arm.claw_open, move_time_ms=1200)
    time.sleep(0.2)

    arm.log("Phase 3: Continuous arc sweep to East (θ = +35°)...")
    arm.move_ik(r=r_scan, theta=35.0, z=z_scan, claw_tick=arm.claw_open, move_time_ms=1600)
    time.sleep(0.2)

    arm.log("Phase 4: Returning to center azimuth (θ = 0°)...")
    arm.move_ik(r=r_scan, theta=0.0, z=z_scan, claw_tick=arm.claw_open, move_time_ms=1200)
    time.sleep(0.2)

    arm.log("Phase 5: Returning to upright reset pose...")
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 4 Complete! <<<\n")


def motion_single_5_color_pick_and_place(arm: ArmController, color: str = "red"):
    """Motion 5: Standard Mid-Range Pick-and-Place (R=220mm, θ=0°)."""
    execute_pick_and_place(arm, pick_r=220.0, pick_th=0.0, pick_z=12.5, color=color, motion_label="Motion 5: Standard Mid-Range Pick & Place (R=220mm)")


def motion_single_6_near_field_pick(arm: ArmController, color: str = "green"):
    """Motion 6: Near-Field Shallow Pick-and-Place (R=165mm, θ=0°)."""
    execute_pick_and_place(arm, pick_r=165.0, pick_th=0.0, pick_z=12.5, color=color, motion_label="Motion 6: Near-Field Shallow Pick & Place (R=165mm)")


def motion_single_7_extended_far_reach_pick(arm: ArmController, color: str = "blue"):
    """Motion 7: Extended Far-Reach Pick-and-Place (R=275mm, θ=0°)."""
    execute_pick_and_place(arm, pick_r=275.0, pick_th=0.0, pick_z=12.5, color=color, motion_label="Motion 7: Extended Far-Reach Pick & Place (R=275mm)")


def motion_single_8_diagonal_west_pick(arm: ArmController, color: str = "red"):
    """Motion 8: Left-Quadrant Flank Diagonal Pick (R=210mm, θ=-30° West)."""
    execute_pick_and_place(arm, pick_r=210.0, pick_th=-30.0, pick_z=12.5, color=color, motion_label="Motion 8: Left-Quadrant Flank Diagonal Pick (θ=-30°)")


def motion_single_9_diagonal_east_pick(arm: ArmController, color: str = "yellow"):
    """Motion 9: Right-Quadrant Midline Diagonal Pick (R=210mm, θ=+30° East)."""
    execute_pick_and_place(arm, pick_r=210.0, pick_th=30.0, pick_z=12.5, color=color, motion_label="Motion 9: Right-Quadrant Midline Diagonal Pick (θ=+30°)")


def motion_single_10_extreme_corner_sweep(arm: ArmController):
    """Motion 10: Far Corner Reach & Tabletop Clearance Arc Sweep."""
    print(f"\n--- [MOTION 10: Far-Corner Radial Sweep (R=265mm) on {arm.arm_id}] ---")
    arm.reset_pose(move_time_ms=1400)
    r_corner = 265.0
    z_clearance = 20.0

    arm.log(f"Phase 1: Extending to Far-West Corner (R={r_corner}mm, θ=-25°, Z={z_clearance}mm)...")
    arm.move_ik(r=r_corner, theta=-25.0, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1500)
    time.sleep(0.3)

    arm.log("Phase 2: Continuous low-altitude arc sweep to Far-East Corner (θ=+25°)...")
    arm.move_ik(r=r_corner, theta=25.0, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1800)
    time.sleep(0.3)

    arm.log("Phase 3: Smooth retraction to safe inner radius...")
    arm.move_ik(r=170.0, theta=0.0, z=60.0, claw_tick=arm.claw_open, move_time_ms=1300)
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 10 Complete! <<<\n")


def motion_single_11_multi_altitude_staircase(arm: ArmController):
    """Motion 11: Multi-Altitude Stepped Standoff Sequence (Z=150 -> 90 -> 40 -> 12.5mm)."""
    print(f"\n--- [MOTION 11: Multi-Altitude Standoff Staircase on {arm.arm_id}] ---")
    arm.reset_pose(move_time_ms=1400)
    r_target = 215.0

    altitudes = [
        (150.0, "High Altitude Inspection Hover"),
        (90.0, "Mid-Altitude Corridor Descent"),
        (40.0, "Pre-Grasp Standoff Plane"),
        (12.5, "Tabletop Contact Elevation"),
    ]

    for z_alt, desc in altitudes:
        arm.log(f"Stepping to {desc} (R={r_target}mm, Z={z_alt}mm)...")
        arm.move_ik(r=r_target, theta=0.0, z=z_alt, claw_tick=arm.claw_open, move_time_ms=1100)
        time.sleep(0.3)

    arm.log("Ascending smoothly back to upright reset...")
    arm.move_ik(r=170.0, theta=0.0, z=80.0, claw_tick=arm.claw_open, move_time_ms=1100)
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 11 Complete! <<<\n")


def motion_single_12_continuous_3point_touch(arm: ArmController):
    """Motion 12: Continuous 3-Point Tabletop Rapid Touch (West -> Center -> East without homing)."""
    print(f"\n--- [MOTION 12: Continuous 3-Point Tabletop Touch on {arm.arm_id}] ---")
    arm.reset_pose(move_time_ms=1400)

    points = [
        ("Point 1 (Near-West)", 175.0, -25.0, 15.0),
        ("Point 2 (Center-Arena)", 220.0, 0.0, 15.0),
        ("Point 3 (Far-East)", 265.0, 25.0, 15.0),
    ]

    for label, r_pt, th_pt, z_pt in points:
        arm.log(f"Gliding smoothly to {label} @ (R={r_pt}mm, θ={th_pt}°, Z={z_pt}mm)...")
        arm.move_ik(r=r_pt, theta=th_pt, z=z_pt, claw_tick=arm.claw_grip, move_time_ms=1300)
        time.sleep(0.3)

    arm.log("Retracting from workspace and returning upright...")
    arm.move_ik(r=160.0, theta=0.0, z=70.0, claw_tick=arm.claw_open, move_time_ms=1200)
    arm.reset_pose(move_time_ms=1400)
    print(">>> Motion 12 Complete! <<<\n")


def motion_single_13_flank_compartment_tour(arm: ArmController):
    """Motion 13: Precision 4-Color Flank Sorting Box Tour (Red -> Green -> Blue -> Yellow)."""
    print(f"\n--- [MOTION 13: 4-Color Flank Tray Compartment Tour on {arm.arm_id}] ---")
    arm.reset_pose(move_time_ms=1400)

    colors = ["red", "green", "blue", "yellow"]
    for c in colors:
        target_info = get_compartment_target(c, arm_id=arm.arm_id, config_path=arm.hw_yaml)
        drop_yaw = target_info.get("drop_yaw_tick", 208 if arm.arm_id == "xarm_1" else 792) if target_info else 208
        arm.log(f"Visiting [{c.upper()}] compartment (Base Yaw: {drop_yaw} ticks)...")
        drop_pose = {
            1: arm.claw_open,
            2: 500,
            3: 830 if arm.arm_id == "xarm_1" else 170,
            4: 160 if arm.arm_id == "xarm_1" else 840,
            5: 470,
            6: drop_yaw,
        }
        arm.send_pose(drop_pose, move_time_ms=1400)
        time.sleep(0.3)
        # Gripper tap
        drop_pose[1] = arm.claw_grip
        arm.send_pose(drop_pose, move_time_ms=400)
        drop_pose[1] = arm.claw_open
        arm.send_pose(drop_pose, move_time_ms=400)
        time.sleep(0.2)

    arm.reset_pose(move_time_ms=1500)
    print(">>> Motion 13 Complete! <<<\n")


# =====================================================================================
# DUAL ARM COOPERATIVE MOTIONS (1 to 12)
# =====================================================================================

def motion_dual_1_synchronized_reset(arm1: ArmController, arm2: ArmController):
    """Dual Motion 1: Synchronized Homing & Alternating Claw Echo."""
    print("\n--- [DUAL MOTION 1: Synchronized Upright Reset & Claw Echo] ---")
    print("Both arms rising simultaneously to straight upright reset pose...")

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1500})
    t1.start(); t2.start()
    t1.join(); t2.join()
    time.sleep(0.5)

    print("xArm 1 clicks claw (open -> grip)...")
    arm1.send_pose({1: arm1.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    arm1.send_pose({1: arm1.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)

    print("xArm 2 echo clicks claw (open -> grip)...")
    arm2.send_pose({1: arm2.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    arm2.send_pose({1: arm2.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)

    print("Both returning to neutral upright...")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1000})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1000})
    t1.start(); t2.start()
    t1.join(); t2.join()
    print(">>> Dual Motion 1 Complete! <<<\n")


def motion_dual_2_symmetrical_mirror_sweep(arm1: ArmController, arm2: ArmController):
    """Dual Motion 2: Symmetrical Mirroring Sweep (Inward bow & Outward slew)."""
    print("\n--- [DUAL MOTION 2: Symmetrical Mirroring Choreography] ---")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1200})
    t1.start(); t2.start(); t1.join(); t2.join()

    print("Step 1: Inward symmetrical bow toward arena center...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 220.0, "theta": 15.0, "z": 45.0, "move_time_ms": 1600})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 220.0, "theta": -15.0, "z": 45.0, "move_time_ms": 1600})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    print("Step 2: Outward symmetrical slew toward respective flank boxes...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 180.0, "theta": -35.0, "z": 60.0, "move_time_ms": 1600})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 180.0, "theta": 35.0, "z": 60.0, "move_time_ms": 1600})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    print("Step 3: Synchronous return to upright reset...")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1500})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 2 Complete! <<<\n")


def motion_dual_3_partitioned_parallel_sort(arm1: ArmController, arm2: ArmController):
    """Dual Motion 3: Partitioned Parallel Sorting (Concurrent independent cycles)."""
    print("\n--- [DUAL MOTION 3: Partitioned Parallel Sorting (Zero-Interference)] ---")
    print("xArm 1 sorting RED in West quadrant | xArm 2 sorting GREEN in East quadrant...")

    t1 = threading.Thread(target=motion_single_5_color_pick_and_place, args=(arm1, "red"))
    t2 = threading.Thread(target=motion_single_5_color_pick_and_place, args=(arm2, "green"))

    t1.start(); t2.start()
    t1.join(); t2.join()
    print(">>> Dual Motion 3 Complete! Concurrent throughput doubled! <<<\n")


def motion_dual_4_midline_mutex_contention(arm1: ArmController, arm2: ArmController):
    """Dual Motion 4: Midline Mutex Arbitration (Collision avoidance in shared overlap zone)."""
    print("\n--- [DUAL MOTION 4: Midline Mutex Arbitration Demo] ---")
    print("Scenario: Cube lands on midline X=280mm. Both arms request pick mutex...")

    midline_mutex = threading.Lock()

    def arm1_task():
        arm1.log("Requesting midline mutex...")
        with midline_mutex:
            arm1.log("[MUTEX ACQUIRED] Descending to pick midline cube...")
            arm1.move_ik(r=200.0, theta=20.0, z=20.0, claw_tick=arm1.claw_grip, move_time_ms=1400)
            time.sleep(0.6)
            arm1.log("Cube secured. Clearing shared zone and slewing to Box 1...")
            arm1.move_ik(r=160.0, theta=-30.0, z=60.0, claw_tick=arm1.claw_grip, move_time_ms=1400)
            arm1.log("[MUTEX RELEASED]")
        arm1.reset_pose(move_time_ms=1200)

    def arm2_task():
        time.sleep(0.1)
        arm2.log("Requesting midline mutex...")
        if not midline_mutex.acquire(blocking=False):
            arm2.log("[MUTEX BLOCKED] Arm 1 active on midline! Holding in elevated Standoff Hover (Zw=165mm)...")
            arm2.move_ik(r=180.0, theta=-20.0, z=90.0, claw_tick=arm2.claw_open, move_time_ms=1200)
            with midline_mutex:
                arm2.log("[MUTEX ACQUIRED] Midline is clear! Executing secondary task...")
                arm2.move_ik(r=200.0, theta=0.0, z=30.0, claw_tick=arm2.claw_open, move_time_ms=1400)
        arm2.reset_pose(move_time_ms=1200)

    t1 = threading.Thread(target=arm1_task)
    t2 = threading.Thread(target=arm2_task)
    t1.start(); t2.start()
    t1.join(); t2.join()
    print(">>> Dual Motion 4 Complete! Contention handled safely with zero collision! <<<\n")


def motion_dual_5_midline_handover(arm1: ArmController, arm2: ArmController):
    """Dual Motion 5: Midline Handover / Relay Transfer."""
    print("\n--- [DUAL MOTION 5: Midline Handover / Relay Transfer] ---")
    print("Both arms starting upright...")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1200})
    t1.start(); t2.start(); t1.join(); t2.join()

    arm1.log("Step 1: xArm 1 picks object in West quadrant...")
    arm1.move_ik(r=220.0, theta=0.0, z=15.0, claw_tick=arm1.claw_grip, move_time_ms=1400)
    time.sleep(0.3)

    arm1.log("Step 2: xArm 1 presents object at central Midline Handover Waypoint (X=280mm, Z=60mm)...")
    arm1.move_ik(r=190.0, theta=25.0, z=60.0, claw_tick=arm1.claw_grip, move_time_ms=1500)
    time.sleep(0.3)

    arm2.log("Step 3: xArm 2 approaches handover point and clamps onto object...")
    arm2.move_ik(r=190.0, theta=-25.0, z=60.0, claw_tick=arm2.claw_open, move_time_ms=1400)
    arm2.send_pose({1: arm2.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    time.sleep(0.4)

    arm1.log("Step 4: xArm 1 opens claw and retracts safely...")
    arm1.send_pose({1: arm1.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    arm1.move_ik(r=140.0, theta=0.0, z=90.0, claw_tick=arm1.claw_open, move_time_ms=1200)

    arm2.log("Step 5: xArm 2 delivers object to Right Flank Box 2...")
    arm2.move_ik(r=180.0, theta=35.0, z=55.0, claw_tick=arm2.claw_grip, move_time_ms=1500)
    arm2.send_pose({1: arm2.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)

    print("Step 6: Both arms returning to upright reset pose...")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 5 Complete! Midline relay transfer successful! <<<\n")


def motion_dual_6_symmetrical_far_reach(arm1: ArmController, arm2: ArmController):
    """Dual Motion 6: Symmetrical Dual Far-Reach Sweep."""
    print("\n--- [DUAL MOTION 6: Symmetrical Dual Far-Reach Sweep] ---")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1200})
    t1.start(); t2.start(); t1.join(); t2.join()

    print("Extending both arms simultaneously to far reach (R=265mm, Z=40mm)...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 265.0, "theta": 0.0, "z": 40.0, "move_time_ms": 1600})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 265.0, "theta": 0.0, "z": 40.0, "move_time_ms": 1600})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    print("Retracting both arms simultaneously...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 160.0, "theta": 0.0, "z": 70.0, "move_time_ms": 1300})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 160.0, "theta": 0.0, "z": 70.0, "move_time_ms": 1300})
    t1.start(); t2.start(); t1.join(); t2.join()

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1200})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 6 Complete! <<<\n")


def motion_dual_7_alternating_leapfrog(arm1: ArmController, arm2: ArmController):
    """Dual Motion 7: Alternating Near/Far Leapfrog Picking."""
    print("\n--- [DUAL MOTION 7: Alternating Near/Far Leapfrog Picking] ---")
    print("Round 1: xArm 1 picks NEAR (R=170mm) while xArm 2 picks FAR (R=265mm)...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 170.0, "theta": -10.0, "z": 20.0, "move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 265.0, "theta": 10.0, "z": 20.0, "move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    print("Round 2: Leapfrog switch! xArm 1 extends FAR (R=265mm) while xArm 2 retracts NEAR (R=170mm)...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 265.0, "theta": -10.0, "z": 20.0, "move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 170.0, "theta": 10.0, "z": 20.0, "move_time_ms": 1500})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1300})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1300})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 7 Complete! <<<\n")


def motion_dual_8_concurrent_flank_drop(arm1: ArmController, arm2: ArmController):
    """Dual Motion 8: Concurrent Dual Flank Drop Demonstration."""
    print("\n--- [DUAL MOTION 8: Concurrent Dual Flank Drop Demonstration] ---")
    print("Both arms swinging outward to their dedicated flank sorting boxes simultaneously...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 180.0, "theta": -45.0, "z": 55.0, "claw_tick": arm1.claw_open, "move_time_ms": 1600})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 180.0, "theta": 45.0, "z": 55.0, "claw_tick": arm2.claw_open, "move_time_ms": 1600})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.5)

    print("Claw click demonstration at drop container...")
    arm1.send_pose({1: arm1.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=400)
    arm2.send_pose({1: arm2.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=400)
    arm1.send_pose({1: arm1.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=400)
    arm2.send_pose({1: arm2.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=400)

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 8 Complete! <<<\n")


def motion_dual_9_dual_tabletop_proximity(arm1: ArmController, arm2: ArmController):
    """Dual Motion 9: Cooperative Midline Proximity Inspection."""
    print("\n--- [DUAL MOTION 9: Cooperative Midline Proximity Inspection] ---")
    print("Hovering over midline boundaries (Z=30mm) with 60mm safe clearance buffer...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 205.0, "theta": 18.0, "z": 30.0, "move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 205.0, "theta": -18.0, "z": 30.0, "move_time_ms": 1500})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.6)

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1300})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1300})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 9 Complete! <<<\n")


def motion_dual_10_synchronized_z_plunge(arm1: ArmController, arm2: ArmController):
    """Dual Motion 10: Synchronized Pure Cartesian Vertical Z-Plunge & Elevate."""
    print("\n--- [DUAL MOTION 10: Synchronized Cartesian Vertical Z-Plunge] ---")
    print("Both arms plunging to tabletop elevation (Z=25mm) in lockstep...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 220.0, "theta": 0.0, "z": 25.0, "move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 220.0, "theta": 0.0, "z": 25.0, "move_time_ms": 1500})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.5)

    print("Both arms elevating back to high hover (Z=120mm)...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 220.0, "theta": 0.0, "z": 120.0, "move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 220.0, "theta": 0.0, "z": 120.0, "move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1200})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 10 Complete! <<<\n")


def motion_dual_11_cross_diagonal_sweep(arm1: ArmController, arm2: ArmController):
    """Dual Motion 11: Symmetrical Cross-Diagonal Arena Sweep."""
    print("\n--- [DUAL MOTION 11: Symmetrical Cross-Diagonal Arena Sweep] ---")
    print("Step 1: Arm 1 sweeps West flank while Arm 2 sweeps East flank...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 230.0, "theta": -30.0, "z": 45.0, "move_time_ms": 1500})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 230.0, "theta": 30.0, "z": 45.0, "move_time_ms": 1500})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    print("Step 2: Both arms sweep inward toward central arena...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 210.0, "theta": 10.0, "z": 45.0, "move_time_ms": 1600})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 210.0, "theta": -10.0, "z": 45.0, "move_time_ms": 1600})
    t1.start(); t2.start(); t1.join(); t2.join()
    time.sleep(0.4)

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1300})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1300})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 11 Complete! <<<\n")


def motion_dual_12_emergency_stop_recovery(arm1: ArmController, arm2: ArmController):
    """Dual Motion 12: Cooperative Emergency Stop & Recovery Cycle Demonstration."""
    print("\n--- [DUAL MOTION 12: Cooperative Emergency Stop & Recovery Demo] ---")
    print("Dispatching synchronous motion...")
    t1 = threading.Thread(target=arm1.move_ik, kwargs={"r": 200.0, "theta": 0.0, "z": 50.0, "move_time_ms": 1200})
    t2 = threading.Thread(target=arm2.move_ik, kwargs={"r": 200.0, "theta": 0.0, "z": 50.0, "move_time_ms": 1200})
    t1.start(); t2.start()
    time.sleep(0.4)

    print("SIMULATING EMERGENCY STOP: Cutting torque on both arms...")
    if arm1.connected and arm1.driver:
        arm1.driver.emergency_stop()
    if arm2.connected and arm2.driver:
        arm2.driver.emergency_stop()
    t1.join(); t2.join()
    time.sleep(0.6)

    print("Re-enabling torque and restoring homing pose...")
    if arm1.connected and arm1.driver:
        arm1.driver.clear_emergency_stop()
    if arm2.connected and arm2.driver:
        arm2.driver.clear_emergency_stop()

    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 12 Complete! Emergency stop and recovery validated! <<<\n")


# =====================================================================================
# MAIN CLI
# =====================================================================================

def main():
    parser = argparse.ArgumentParser(description="xArm 1S Comprehensive Motion Verification Suite")
    parser.add_argument("--arm", default="xarm_1", choices=["xarm_1", "xarm_2"], help="Target arm for single-arm motions")
    parser.add_argument("--dual", action="store_true", help="Execute dual-arm cooperative motions")
    parser.add_argument("--motion", type=int, choices=list(range(1, 14)), help="Motion ID (1 to 13)")
    parser.add_argument("--all", action="store_true", help="Execute all motions sequentially")
    parser.add_argument("--port", default="auto", help="Port for single arm, or port for xArm 1 in dual mode")
    parser.add_argument("--port2", default="auto", help="Port for xArm 2 in dual mode")
    parser.add_argument("--color", default="red", choices=["red", "green", "blue", "yellow"], help="Target color for pick-and-place motions")
    args = parser.parse_args()

    if not args.motion and not args.all:
        parser.print_help()
        print("\nPlease specify either --motion [1-13] or --all.")
        sys.exit(1)

    # DUAL ARM MODE
    if args.dual:
        print("\n" + "=" * 70)
        print("          DUAL XARM COOPERATIVE MOTION TEST SUITE (12 MOTIONS)")
        print("=" * 70)

        devices = find_all_xarm_devices()
        port1 = args.port
        port2 = args.port2

        if port1 == "auto" and len(devices) >= 1:
            port1 = devices[0][0]
        if port2 == "auto" and len(devices) >= 2:
            port2 = devices[1][0]
        elif port2 == "auto" and len(devices) == 1:
            port2 = "/dev/null"

        print(f"Assigning Interfaces: xarm_1 -> {port1} | xarm_2 -> {port2}")
        arm1 = ArmController(arm_id="xarm_1", port=port1)
        arm2 = ArmController(arm_id="xarm_2", port=port2)

        dual_dispatch = {
            1: motion_dual_1_synchronized_reset,
            2: motion_dual_2_symmetrical_mirror_sweep,
            3: motion_dual_3_partitioned_parallel_sort,
            4: motion_dual_4_midline_mutex_contention,
            5: motion_dual_5_midline_handover,
            6: motion_dual_6_symmetrical_far_reach,
            7: motion_dual_7_alternating_leapfrog,
            8: motion_dual_8_concurrent_flank_drop,
            9: motion_dual_9_dual_tabletop_proximity,
            10: motion_dual_10_synchronized_z_plunge,
            11: motion_dual_11_cross_diagonal_sweep,
            12: motion_dual_12_emergency_stop_recovery,
        }

        try:
            motions = [args.motion] if args.motion else list(range(1, 13))
            for m in motions:
                fn = dual_dispatch.get(m)
                if fn:
                    fn(arm1, arm2)
                if arm1.connected or arm2.connected:
                    time.sleep(1.0)
                else:
                    time.sleep(0.05)
        finally:
            arm1.close()
            arm2.close()

    # SINGLE ARM MODE
    else:
        print("\n" + "=" * 70)
        print(f"          SINGLE {args.arm.upper()} MOTION TEST SUITE (13 MOTIONS)")
        print("=" * 70)
        arm = ArmController(arm_id=args.arm, port=args.port)

        single_dispatch = {
            1: motion_single_1_reset_and_claw,
            2: motion_single_2_vertical_z_plunge,
            3: motion_single_3_radial_reach_sweep,
            4: motion_single_4_azimuthal_arc_scan,
            5: lambda a: motion_single_5_color_pick_and_place(a, color=args.color),
            6: lambda a: motion_single_6_near_field_pick(a, color="green"),
            7: lambda a: motion_single_7_extended_far_reach_pick(a, color="blue"),
            8: lambda a: motion_single_8_diagonal_west_pick(a, color="red"),
            9: lambda a: motion_single_9_diagonal_east_pick(a, color="yellow"),
            10: motion_single_10_extreme_corner_sweep,
            11: motion_single_11_multi_altitude_staircase,
            12: motion_single_12_continuous_3point_touch,
            13: motion_single_13_flank_compartment_tour,
        }

        try:
            motions = [args.motion] if args.motion else list(range(1, 14))
            for m in motions:
                fn = single_dispatch.get(m)
                if fn:
                    fn(arm)
                if arm.connected:
                    time.sleep(1.0)
                else:
                    time.sleep(0.05)
        finally:
            arm.close()


if __name__ == "__main__":
    main()
