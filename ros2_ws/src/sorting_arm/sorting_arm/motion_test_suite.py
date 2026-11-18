#!/usr/bin/env python3
"""
Comprehensive Motion Verification Test Suite for LewanSoul / Hiwonder xArm 1S.

Provides 13 distinct single-arm motions (covering near, mid, far, diagonal West/East,
corner sweeps, multi-altitude staircase, continuous 3-point touching, and 4-compartment flank tour):

Single Arm Motions (xarm_1):
  1. Upright Reset & Claw Span Calibration (500-tick reset, 60mm open -> 25mm grip -> closed)
  2. Pure Cartesian Vertical Z-Plunge & Elevate (Z=120mm -> Z=25mm -> Z=120mm at constant R=220mm)
  3. Pure Cartesian Radial Reach Sweep (R=160mm -> R=275mm -> R=160mm at clearance Z=35mm)
  4. Azimuthal Panoramic Arc Scan (Base yaw sweep from -35° to +35° over arena)
  5. Standard Mid-Range Color Pick-and-Place (R=220mm, θ=0°, Z=12.5mm -> Flank Container)
  6. Near-Field Shallow Pick-and-Place (Short reach R=165mm, θ=0°, Z=12.5mm -> Flank Container)
  7. Extended Far-Reach Pick-and-Place (Maximum reach R=275mm, θ=0°, Z=12.5mm -> Flank Container)
  8. Left-Quadrant Flank Diagonal Pick-and-Place (R=210mm, θ=-30° West -> Flank Container)
  9. Right-Quadrant Midline Diagonal Pick-and-Place (R=210mm, θ=+30° East -> Flank Container)
  10. Far-Corner Diagonal Reach & Tabletop Sweep (R=265mm, θ=-25° to +25° at Z=20mm)
  11. Multi-Altitude Stepped Standoff Sequence (Z=150mm -> 90mm -> 40mm -> 12.5mm contact)
  12. Continuous 3-Point Tabletop Rapid Touch (Near-West -> Center -> Far-East without homing)
  13. Rear Flank Dual Container Boxes Drop Tour (Red @ 650, Green @ 354)
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
        self.hw_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
        candidates = [self.hw_yaml, os.path.join(os.getcwd(), self.hw_yaml), os.path.join("/workspace", self.hw_yaml)]
        for c in candidates:
            if os.path.exists(c):
                self.hw_yaml = c
                break

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
        self.claw_open = int(m1_cfg.get("fully_open", {}).get("tick", 228))
        self.claw_grip = int(m1_cfg.get("grip_cube_25mm", {}).get("tick", 610))
        self.claw_closed = int(m1_cfg.get("fully_closed_empty", {}).get("tick", 791))

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
    drop_yaw = target_info.get("drop_yaw_tick", 145) if target_info else 145

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
            3: 830,
            4: 160,
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
    """Motion 13: Rear Flank Dual Container Boxes Drop Tour (Red @ 650, Green @ 354)."""
    print(f"\n--- [MOTION 13: Rear Flank Dual Container Boxes Drop Tour on {arm.arm_id}] ---")
    arm.reset_pose(move_time_ms=1400)

    # Overhead rear transit tuck
    rear_tuck = {1: arm.claw_open, 2: 503, 3: 504, 4: 350, 5: 501, 6: 502}
    arm.send_pose(rear_tuck, move_time_ms=1200)

    for color_name in ["red", "green"]:
        target_info = get_compartment_target(color_name, arm_id=arm.arm_id, config_path=arm.hw_yaml)
        drop_yaw = target_info.get("drop_yaw_tick", 650 if color_name == "red" else 354) if target_info else (650 if color_name == "red" else 354)
        preset_ticks = target_info.get("drop_pose_preset") if target_info else None

        arm.log(f"Visiting [{color_name.upper()}] Container Box in Rear (Base Yaw: {drop_yaw} ticks)...")
        if preset_ticks and len(preset_ticks) == 6:
            drop_pose = {
                1: arm.claw_open,
                2: preset_ticks[1],
                3: preset_ticks[2],
                4: preset_ticks[3],
                5: preset_ticks[4],
                6: drop_yaw,
            }
        else:
            drop_pose = {
                1: arm.claw_open,
                2: 503,
                3: 704,
                4: 169,
                5: 558,
                6: drop_yaw,
            }

        arm.send_pose(drop_pose, move_time_ms=1500)
        time.sleep(0.4)
        # Gripper tap
        drop_pose[1] = arm.claw_grip
        arm.send_pose(drop_pose, move_time_ms=400)
        time.sleep(0.3)
        drop_pose[1] = arm.claw_open
        arm.send_pose(drop_pose, move_time_ms=400)
        time.sleep(0.3)

    # Retract back through rear tuck to home
    arm.send_pose(rear_tuck, move_time_ms=1100)
    arm.reset_pose(move_time_ms=1500)
    print(">>> Motion 13 Complete! <<<\n")


# =====================================================================================
# MAIN CLI
# =====================================================================================

def main():
    parser = argparse.ArgumentParser(description="xArm 1S Single-Arm Comprehensive Motion Verification Suite")
    parser.add_argument("--arm", default="xarm_1", help="Target manipulator ID (default: xarm_1)")
    parser.add_argument("--motion", type=int, choices=list(range(1, 14)), help="Motion ID (1 to 13)")
    parser.add_argument("--all", action="store_true", help="Execute all 13 motions sequentially")
    parser.add_argument("--port", default="auto", help="Serial or HID port device (default: auto)")
    parser.add_argument("--color", default="red", choices=["red", "green", "blue", "yellow"], help="Target color for pick-and-place motions")
    args = parser.parse_args()

    if not args.motion and not args.all:
        parser.print_help()
        print("\nPlease specify either --motion [1-13] or --all.")
        sys.exit(1)

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
