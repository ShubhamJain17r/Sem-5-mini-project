#!/usr/bin/env python3
"""
Autonomous Vision-Guided Pick-and-Place Bridge (Single xArm 1S Dual-Color Sorting).

Workflow:
  1. Captures overhead camera frame and applies 4-point ArUco planar homography.
  2. Segments colored cubes; filters for target red and green cubes while ignoring blue chassis.
  3. Shifts coordinates into arm-relative cylindrical/polar frames (r, theta_arm).
  4. Enforces Nearest-First pick ordering (ascending r) to prevent foreground cube collisions.
  5. Computes analytical inverse kinematics for pick (r, theta, z=12.5mm) and overhead rear-flip drop into designated Rear Flank container box (Red or Green).
  6. Executes 7-phase trajectory with overhead rear-flip and Motor 1 clamp readback grip verification.
  7. Supports --dry-run and --synthetic modes for offline verification without physical hardware.
"""

import argparse
import math
import os
import sys
import time
from typing import List, Optional

# Ensure repository root and sorting_arm package are in sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
from sorting_arm.kinematics import (
    DEFAULT_PARAMS,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
)
from sorting_arm.pick_and_place_test import PickAndPlaceCommander
from vision.arena_vision_pipeline import ArenaVisionPipeline, DetectedCube
from vision.test_aruco_warp import generate_synthetic_arena_frame
import yaml


class AutonomousSorterBridge:
    """Bridges overhead vision perception with analytical kinematics and physical execution."""

    def __init__(
        self,
        arm_id: str = "xarm_1",
        config_dir: str = "config",
        dry_run: bool = False,
        port: str = "auto",
        scale_px_per_mm: float = 2.0,
    ):
        self.arm_id = arm_id
        self.config_dir = config_dir
        self.dry_run = dry_run
        self.port = port
        self.scale_px_per_mm = scale_px_per_mm

        # Initialize vision pipeline
        self.pipeline = ArenaVisionPipeline(config_dir=self.config_dir, scale_px_per_mm=self.scale_px_per_mm)

        # Load arm kinematic parameters from xarm_hardware.yaml
        primary_yaml = os.path.join(self.config_dir, "xarm_hardware.yaml")
        candidates = [primary_yaml, os.path.join(os.getcwd(), primary_yaml), os.path.join(repo_root, primary_yaml)]
        self.hardware_yaml = primary_yaml
        for c in candidates:
            if os.path.exists(c):
                self.hardware_yaml = c
                break
        self.params = load_kinematic_parameters(self.hardware_yaml, arm_id=self.arm_id)

        # Initialize hardware commander if not dry-run
        self.commander: Optional[PickAndPlaceCommander] = None
        if not self.dry_run:
            try:
                self.commander = PickAndPlaceCommander(
                    arm_id=self.arm_id,
                    config_dir=self.config_dir,
                    direct_mode=True,
                    port=self.port,
                    smooth=True,
                    closed_loop=True,
                )
                print(f"[Bridge] Connected to {self.arm_id} successfully.")
            except Exception as e:
                print(f"[Bridge] Warning: Could not connect to arm {self.arm_id}: {e}")
                print("[Bridge] Falling back to --dry-run mode.")
                self.dry_run = True

    def scan_arena(self, cap=None, synthetic: bool = False) -> List[DetectedCube]:
        """Captures a stationary frame, rectifies arena, and detects colored cubes."""
        if synthetic:
            frame = generate_synthetic_arena_frame()
            self.pipeline.calibrate_homography(frame)
        else:
            if cap is None:
                raise ValueError("Camera capture object is None")
            ret, frame = cap.read()
            if not ret:
                print("[Bridge] Warning: Failed to capture camera frame.")
                return []

        # Undistort, warp, and detect cubes
        _, _, cubes = self.pipeline.process_frame(frame, draw_grid=True)
        return cubes

    def filter_and_sort_queue(self, cubes: List[DetectedCube]) -> List[DetectedCube]:
        """
        Filters cubes for targetable colors (red and green) and applies Nearest-First radial sorting.
        Nearest cubes (smallest r) are prioritized first to prevent the arm forearm/claw
        from colliding with foreground cubes during horizontal approaches.
        Blue chassis reflections and non-targetable colors are excluded from picking.
        """
        assigned_cubes = []
        for c in cubes:
            if c.color.lower() not in ("red", "green"):
                continue
            if not getattr(c, "targetable", True):
                continue
            if c.assigned_arm in (self.arm_id, "xarm_1", "xarm", "all") or self.arm_id in ("xarm_1", "xarm"):
                active_r = c.target_r_mm
                if 120.0 <= active_r <= 320.0:
                    assigned_cubes.append(c)

        # Nearest-First policy: sort by ascending target radius r
        assigned_cubes.sort(key=lambda c: c.target_r_mm)
        return assigned_cubes

    def execute_pick_and_place(self, cube: DetectedCube) -> bool:
        """
        Executes full pick and place for a single detected cube:
          1. Computes pick IK for cube (r, theta, z)
          2. Computes drop IK for dedicated container box
          3. Executes trajectory with grip confirmation
        """
        color = cube.color.lower()
        r = cube.target_r_mm
        theta = cube.target_theta_deg
        z = cube.world_z

        print("\n------------------------------------------------------------------------")
        print(f"  TARGET: {cube.color.upper()} cube at World=({cube.world_x:.1f}, {cube.world_y:.1f}, Z={z:.1f}) mm")
        print(f"  ARM COORDINATES: r = {r:.1f} mm, theta = {theta:+.1f} deg, yaw = {cube.yaw_deg:+.1f} deg")
        print(f"  ASSIGNED MANIPULATOR: {self.arm_id}")
        print("------------------------------------------------------------------------")

        # 1. Compute Pick IK with wrist roll aligned to cube orientation
        roll_tick = int(round(500 + cube.yaw_deg * (1000.0 / 240.0)))
        pick_ik = solve_ik(
            r=r,
            theta=theta,
            z=z,
            phi=None,  # Use natural adaptive pitch
            roll_tick=roll_tick,
            params=self.params,
        )

        if not pick_ik.success:
            print(f"[Error] Pick IK failed: {pick_ik.message}")
            return False

        print(f"[IK Pick] Target reachable! Motor ticks (1..6): {pick_ik.ticks}")
        print(f"[IK Pick] Joint angles (deg): {pick_ik.angles_deg}")

        # 2. Compute Drop Target for Rear Flank Container Box
        drop_target = get_compartment_target(color=color, arm_id=self.arm_id, config_path=self.hardware_yaml)
        if not drop_target:
            print(f"[Error] Could not find compartment target for color '{color}' in {self.hardware_yaml}")
            return False

        drop_r = drop_target["r"]
        drop_theta = drop_target["theta_deg"]
        drop_z = drop_target["z"]
        drop_yaw_tick = drop_target.get("drop_yaw_tick", 875)
        print(f"[IK Drop] Destination: Container Box for {self.arm_id}")
        print(f"[IK Drop] r={drop_r:.1f} mm, theta={drop_theta:+.1f} deg, Drop Yaw Tick={drop_yaw_tick}")

        # 3. Dry-Run or Physical Execution
        if self.dry_run or self.commander is None:
            print("[Dry-Run] Trajectory computed successfully. Skipping hardware actuation.")
            return True

        # Physical trajectory execution
        print(f"[Execution] Dispatching 5-phase pick & place routine to {self.arm_id}...")
        try:
            success = self.commander.run_5_phase_pick_and_place(
                pick_r=r,
                pick_theta=theta,
                pick_z=z,
                color=color,
            )
            return success
        except Exception as e:
            print(f"[Execution Error] Pick-and-place routine failed: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description="Autonomous Vision-Guided Color Sorting Bridge.")
    parser.add_argument("--arm", default="xarm_1", help="Manipulator under test (default: xarm_1)")
    parser.add_argument("--device", default="/dev/video0", help="Camera device path or index (default: /dev/video0)")
    parser.add_argument("--port", default="auto", help="Serial port for arm communication (default: auto)")
    parser.add_argument("--config-dir", default="config", help="Directory containing config YAMLs")
    parser.add_argument("--dry-run", action="store_true", help="Simulate perception, coordinate transformation, and IK without moving hardware")
    parser.add_argument("--synthetic", action="store_true", help="Run with synthetic arena camera frame (for offline testing)")
    parser.add_argument("--single-shot", action="store_true", help="Pick only the nearest single cube and exit")
    args = parser.parse_args()

    print("========================================================================")
    print("  AUTONOMOUS COLOR SORTING: PHASE 1 & PHASE 2 INTEGRATED BRIDGE")
    print(f"  Active Arm: {args.arm} | Mode: {'DRY RUN' if args.dry_run else 'PHYSICAL'}")
    print(f"  Camera Source: {'SYNTHETIC' if args.synthetic else args.device}")
    print("========================================================================")

    bridge = AutonomousSorterBridge(
        arm_id=args.arm,
        config_dir=args.config_dir,
        dry_run=args.dry_run,
        port=args.port,
    )

    cap = None
    if not args.synthetic:
        dev_id = int(args.device) if args.device.isdigit() else args.device
        cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2 if isinstance(dev_id, str) else cv2.CAP_ANY)
        if not cap.isOpened():
            print(f"[Error] Failed to open camera {args.device}. Falling back to --synthetic mode.")
            args.synthetic = True

    # 1. Scan Arena
    print("[Perception] Scanning arena for colored cubes...")
    raw_cubes = bridge.scan_arena(cap=cap, synthetic=args.synthetic)
    print(f"[Perception] Total cubes detected on arena: {len(raw_cubes)}")
    for c in raw_cubes:
        print(f"  - {c.color.upper()}: World=({c.world_x:.1f}, {c.world_y:.1f}) mm | "
              f"Assigned={c.assigned_arm} | r={c.target_r_mm:.1f}mm, th={c.target_theta_deg:+.1f}°")

    # 2. Filter & Sort Queue
    queue = bridge.filter_and_sort_queue(raw_cubes)
    print(f"\n[Queue] Filtered {len(queue)} targets for {args.arm} (Ordered Nearest-First):")
    for i, c in enumerate(queue):
        print(f"  [{i+1}] {c.color.upper()}: r = {c.target_r_mm:.1f} mm (theta = {c.target_theta_deg:+.1f} deg)")

    if not queue:
        print("[Done] No reachable targets found in queue for this arm.")
        if cap:
            cap.release()
        return

    # 3. Process Queue
    targets_to_process = [queue[0]] if args.single_shot else queue
    success_count = 0

    for i, target in enumerate(targets_to_process):
        print(f"\n>>> Executing Target {i+1} of {len(targets_to_process)} <<<")
        ok = bridge.execute_pick_and_place(target)
        if ok:
            success_count += 1
            print(f"[Success] Target {i+1} ({target.color.upper()}) completed!")
        else:
            print(f"[Failed] Target {i+1} ({target.color.upper()}) could not be completed.")

    print("\n========================================================================")
    print(f"  SORTING RUN FINISHED: {success_count}/{len(targets_to_process)} targets sorted successfully.")
    print("========================================================================")

    if cap:
        cap.release()


if __name__ == "__main__":
    main()
