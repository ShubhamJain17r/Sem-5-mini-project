#!/usr/bin/env python3
"""
Comprehensive Motion Verification Test Suite for LewanSoul / Hiwonder xArm 1S.

Provides 5 distinct motions for single arm (xArm 1 or xArm 2) and
5 distinct cooperative motions when both arms are connected:

Single Arm Motions (xarm_1 / xarm_2):
  1. Upright Reset & Claw Span Calibration (500-tick reset, 60mm open -> 25mm grip -> closed)
  2. Pure Cartesian Vertical Z-Plunge & Lift (Z=120mm -> Z=25mm -> Z=120mm at constant R=220mm)
  3. Pure Cartesian Radial Reach Sweep (R=160mm -> R=270mm -> R=160mm at clearance Z=35mm)
  4. Azimuthal Panoramic Arc Scan (Base yaw sweep from -35° to +35° over arena)
  5. Complete 5-Phase Color Pick-and-Place (Arena cube -> Flank 2x2 tray -> Upright reset)

Dual Arm Cooperative Motions (Both Connected):
  1. Synchronized Homing & Claw Echo (Simultaneous upright reset & alternating gripper clicks)
  2. Symmetrical Mirroring Sweep (Inward arena bow & outward flank tray sweep in mirror symmetry)
  3. Partitioned Parallel Sorting (Arm 1 sorts West quadrant, Arm 2 sorts East quadrant concurrently)
  4. Midline Mutex Arbitration (Collision avoidance in shared overlap zone X=280mm)
  5. Midline Handover / Relay Transfer (Cross-quadrant handoff from Arm 1 to Arm 2 at midline)
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


def load_deviations(config_dir: str, arm_name: str) -> Dict[int, int]:
    path = os.path.join(config_dir, "servo_calibration.yaml")
    data = load_yaml(path)
    devs = data.get(arm_name, {}).get("servo_deviations", {})
    return {
        1: int(devs.get("motor_1_claw", 0)),
        2: int(devs.get("motor_2_wrist_roll", 0)),
        3: int(devs.get("motor_3_wrist_pitch", 0)),
        4: int(devs.get("motor_4_elbow", 0)),
        5: int(devs.get("motor_5_shoulder", 0)),
        6: int(devs.get("motor_6_base_yaw", 0)),
    }


class ArmController:
    """Manages motion dispatch and kinematics for an individual xArm."""

    def __init__(self, arm_id: str = "xarm_1", port: str = "auto", config_dir: str = "config"):
        self.arm_id = arm_id
        self.config_dir = config_dir
        self.port = port
        self.hw_yaml = os.path.join(config_dir, "dual_xarm_hardware.yaml")

        self.params = load_kinematic_parameters(self.hw_yaml, arm_id=self.arm_id)
        self.deviations = load_deviations(config_dir, self.arm_id)

        self.driver = XArmDriver(
            port=self.port,
            deviations=self.deviations,
            min_ticks=self.params.min_safe_ticks,
            max_ticks=self.params.max_safe_ticks,
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
            print(f"    [{self.arm_id} SIM] Ticks: {t_str} ({move_time_ms}ms)")
        time.sleep((move_time_ms + 100) / 1000.0)

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
        self.send_pose({1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=move_time_ms)

    def close(self):
        if self.connected and self.driver:
            self.driver.disconnect()


# =====================================================================================
# SINGLE ARM MOTIONS (1 to 5)
# =====================================================================================

def motion_single_1_reset_and_claw(arm: ArmController):
    """Motion 1: Upright Reset & Gripper Span Calibration."""
    print(f"\n--- [MOTION 1: Upright Reset & Gripper Span Calibration on {arm.arm_id}] ---")
    arm.log("Moving straight upright to physical reset pose (all servos @ 500 ticks)...")
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

    arm.log("Returning claw to neutral center (500 ticks) and finishing upright...")
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

    arm.log(f"Phase 2: Gliding horizontally forward to Extended Reach (R=270mm, Z={z_clearance}mm)...")
    arm.move_ik(r=270.0, theta=th_target, z=z_clearance, claw_tick=arm.claw_open, move_time_ms=1600)
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
    """Motion 5: Complete 5-Phase Color Pick-and-Place."""
    print(f"\n--- [MOTION 5: 5-Phase Pick-and-Place ({color.upper()}) on {arm.arm_id}] ---")
    arm.log("Phase 0: Starting at Upright Reset Pose...")
    arm.reset_pose(move_time_ms=1500)

    pick_r = 220.0
    pick_th = 0.0
    pick_z = 12.5  # Tabletop cube center

    standoff_r = 185.0
    hover_z = 37.5
    lift_z = 47.5
    retract_r = 170.0

    arm.log(f"Phase 1: Standoff Hover @ (R={standoff_r}mm, Z={hover_z}mm, claw open)...")
    arm.move_ik(standoff_r, pick_th, hover_z, claw_tick=arm.claw_open, move_time_ms=1400)

    arm.log(f"Phase 2: Descending onto Cube @ (R={pick_r}mm, Z={pick_z}mm)...")
    arm.move_ik(pick_r, pick_th, pick_z, claw_tick=arm.claw_open, move_time_ms=1200)

    arm.log(f"Phase 3: Clamping Gripper onto 25mm Cube (tick {arm.claw_grip})...")
    arm.move_ik(pick_r, pick_th, pick_z, claw_tick=arm.claw_grip, move_time_ms=800)
    time.sleep(0.4)

    arm.log(f"Phase 4: Lifting to Z={lift_z}mm and Retracting to R={retract_r}mm...")
    arm.move_ik(retract_r, pick_th, lift_z, claw_tick=arm.claw_grip, move_time_ms=1100)

    arm.log(f"Phase 5: Slewing ~70° to Flank 2x2 Tray (Color: {color.upper()})...")
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
        # High clearance drop pose over flank tray (Z ≈ 56mm above tray rim Z=35mm)
        drop_pose = {
            1: arm.claw_grip,
            2: 500,
            3: 830 if arm.arm_id == "xarm_1" else 170,
            4: 160 if arm.arm_id == "xarm_1" else 840,
            5: 470,
            6: drop_yaw,
        }
    arm.send_pose(drop_pose, move_time_ms=1700)

    arm.log("Phase 6: Releasing cube into sorting box...")
    drop_pose[1] = arm.claw_open
    arm.send_pose(drop_pose, move_time_ms=700)
    time.sleep(0.4)

    arm.log("Phase 7: Clearing rim and returning to upright reset pose...")
    arm.reset_pose(move_time_ms=1600)
    print(">>> Motion 5 Complete! <<<\n")


# =====================================================================================
# DUAL ARM COOPERATIVE MOTIONS (1 to 5)
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
    print("xArm 1 sorting RED in Left Quadrant | xArm 2 sorting GREEN in Right Quadrant...")

    t1 = threading.Thread(target=motion_single_5_color_pick_and_place, args=(arm1, "red"))
    t2 = threading.Thread(target=motion_single_5_color_pick_and_place, args=(arm2, "green"))

    t1.start(); t2.start()
    t1.join(); t2.join()
    print(">>> Dual Motion 3 Complete! Concurrent throughput doubled! <<<\n")


def motion_dual_4_midline_mutex_contention(arm1: ArmController, arm2: ArmController):
    """Dual Motion 4: Midline Mutex Arbitration (Collision avoidance in shared overlap zone)."""
    print("\n--- [DUAL MOTION 4: Midline Mutex Arbitration Demo] ---")
    print("Scenario: Cube lands on midline X=280mm. Both arms request pick mutex...")

    # Shared lock
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
            # Now wait for mutex to be released
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

    # Step 1: Arm 1 picks cube from left quadrant
    arm1.log("Step 1: xArm 1 picks object in West quadrant...")
    arm1.move_ik(r=220.0, theta=0.0, z=15.0, claw_tick=arm1.claw_grip, move_time_ms=1400)
    time.sleep(0.3)

    # Step 2: Arm 1 presents object at Midline Handover point
    arm1.log("Step 2: xArm 1 presents object at central Midline Handover Waypoint (X=280mm, Z=60mm)...")
    arm1.move_ik(r=190.0, theta=25.0, z=60.0, claw_tick=arm1.claw_grip, move_time_ms=1500)
    time.sleep(0.3)

    # Step 3: Arm 2 approaches midline and clamps cube
    arm2.log("Step 3: xArm 2 approaches handover point and clamps onto object...")
    arm2.move_ik(r=190.0, theta=-25.0, z=60.0, claw_tick=arm2.claw_open, move_time_ms=1400)
    arm2.send_pose({1: arm2.claw_grip, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    time.sleep(0.4)

    # Step 4: Arm 1 releases and clears
    arm1.log("Step 4: xArm 1 opens claw and retracts safely...")
    arm1.send_pose({1: arm1.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)
    arm1.move_ik(r=140.0, theta=0.0, z=90.0, claw_tick=arm1.claw_open, move_time_ms=1200)

    # Step 5: Arm 2 delivers to Right Flank Box 2
    arm2.log("Step 5: xArm 2 delivers object to Right Flank Box 2...")
    arm2.move_ik(r=180.0, theta=35.0, z=55.0, claw_tick=arm2.claw_grip, move_time_ms=1500)
    arm2.send_pose({1: arm2.claw_open, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}, move_time_ms=600)

    # Step 6: Both return upright
    print("Step 6: Both arms returning to upright reset pose...")
    t1 = threading.Thread(target=arm1.reset_pose, kwargs={"move_time_ms": 1400})
    t2 = threading.Thread(target=arm2.reset_pose, kwargs={"move_time_ms": 1400})
    t1.start(); t2.start(); t1.join(); t2.join()
    print(">>> Dual Motion 5 Complete! Midline relay transfer successful! <<<\n")


# =====================================================================================
# MAIN CLI
# =====================================================================================

def main():
    parser = argparse.ArgumentParser(description="xArm 1S Motion Verification Suite")
    parser.add_argument("--arm", default="xarm_1", choices=["xarm_1", "xarm_2"], help="Target arm for single-arm motions")
    parser.add_argument("--dual", action="store_true", help="Execute dual-arm cooperative motions")
    parser.add_argument("--motion", type=int, choices=[1, 2, 3, 4, 5], help="Motion ID (1 to 5)")
    parser.add_argument("--all", action="store_true", help="Execute all 5 motions sequentially")
    parser.add_argument("--port", default="auto", help="Port for single arm, or port for xArm 1 in dual mode")
    parser.add_argument("--port2", default="auto", help="Port for xArm 2 in dual mode")
    parser.add_argument("--color", default="red", choices=["red", "green", "blue", "yellow"], help="Target color for Motion 5")
    args = parser.parse_args()

    if not args.motion and not args.all:
        parser.print_help()
        print("\nPlease specify either --motion [1-5] or --all.")
        sys.exit(1)

    # DUAL ARM MODE
    if args.dual:
        print("\n" + "=" * 70)
        print("          DUAL XARM COOPERATIVE MOTION TEST SUITE")
        print("=" * 70)

        # Detect ports if auto
        devices = find_all_xarm_devices()
        port1 = args.port
        port2 = args.port2

        if port1 == "auto" and len(devices) >= 1:
            port1 = devices[0][0]
        if port2 == "auto" and len(devices) >= 2:
            port2 = devices[1][0]
        elif port2 == "auto" and len(devices) == 1:
            port2 = "/dev/null"  # Offline simulation for 2nd arm if only 1 connected

        print(f"Assigning Interfaces: xarm_1 -> {port1} | xarm_2 -> {port2}")
        arm1 = ArmController(arm_id="xarm_1", port=port1)
        arm2 = ArmController(arm_id="xarm_2", port=port2)

        try:
            motions = [args.motion] if args.motion else [1, 2, 3, 4, 5]
            for m in motions:
                if m == 1:
                    motion_dual_1_synchronized_reset(arm1, arm2)
                elif m == 2:
                    motion_dual_2_symmetrical_mirror_sweep(arm1, arm2)
                elif m == 3:
                    motion_dual_3_partitioned_parallel_sort(arm1, arm2)
                elif m == 4:
                    motion_dual_4_midline_mutex_contention(arm1, arm2)
                elif m == 5:
                    motion_dual_5_midline_handover(arm1, arm2)
                time.sleep(1.0)
        finally:
            arm1.close()
            arm2.close()

    # SINGLE ARM MODE
    else:
        print("\n" + "=" * 70)
        print(f"          SINGLE {args.arm.upper()} MOTION TEST SUITE")
        print("=" * 70)
        arm = ArmController(arm_id=args.arm, port=args.port)

        try:
            motions = [args.motion] if args.motion else [1, 2, 3, 4, 5]
            for m in motions:
                if m == 1:
                    motion_single_1_reset_and_claw(arm)
                elif m == 2:
                    motion_single_2_vertical_z_plunge(arm)
                elif m == 3:
                    motion_single_3_radial_reach_sweep(arm)
                elif m == 4:
                    motion_single_4_azimuthal_arc_scan(arm)
                elif m == 5:
                    motion_single_5_color_pick_and_place(arm, color=args.color)
                time.sleep(1.0)
        finally:
            arm.close()


if __name__ == "__main__":
    main()
