#!/usr/bin/env python3
"""
Single xArm 1S Autonomous Master Execution Pipeline.

Executes end-to-end autonomous color sorting in 4 structured, self-healing stages:
  - Presetup: Validates environment and configuration parameters.
  - Stage 1: Hardware Connectivity & Self-Healing Access (camera & xArm permission loop).
  - Stage 2: Camera Extrinsics & ArUco Marker Orientation Verification (interactive guide).
  - Stage 3: Perception & Central Pickup Zone Filtering (rejects Orange Danger Zone).
  - Stage 4: Autonomous Scheduling & Pick-and-Place Execution (Nearest-First to rear boxes).

Usage:
  python3 tools/run_autonomous_system.py [--dry-run] [--step] [--auto] [--camera 4]
"""

import argparse
import glob
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

# Ensure repository root and sorting_arm package are on sys.path
script_dir = os.path.dirname(os.path.realpath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
import yaml

from sorting_arm.kinematics import (
    DEFAULT_PARAMS,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_fk,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices
from vision.arena_vision_pipeline import ArenaVisionPipeline, DetectedCube


# Colors for terminal output
class TermColor:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'


def log_header(text: str):
    print(f"\n{TermColor.BOLD}{TermColor.CYAN}{'=' * 78}{TermColor.ENDC}")
    print(f"{TermColor.BOLD}{TermColor.CYAN}  {text}{TermColor.ENDC}")
    print(f"{TermColor.BOLD}{TermColor.CYAN}{'=' * 78}{TermColor.ENDC}\n")


def log_info(text: str):
    print(f"{TermColor.GREEN}[INFO]{TermColor.ENDC} {text}")


def log_warn(text: str):
    print(f"{TermColor.WARNING}[WARN]{TermColor.ENDC} {text}")


def log_err(text: str):
    print(f"{TermColor.FAIL}[ERROR]{TermColor.ENDC} {text}")


def check_and_heal_camera(camera_idx: int) -> Tuple[cv2.VideoCapture, int]:
    """
    Attempts to open the specified camera device. If permission is denied or device
    fails to open, prompts the user in the terminal with the exact fix command and
    waits for [Enter] in a self-healing loop.
    """
    while True:
        log_info(f"Probing camera device index {camera_idx} (/dev/video{camera_idx})...")
        cap = cv2.VideoCapture(camera_idx, cv2.CAP_V4L2)
        if cap.isOpened():
            # Flush internal buffer
            for _ in range(5):
                cap.read()
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                log_info(f"Camera opened successfully: {frame.shape[1]}x{frame.shape[0]} px @ /dev/video{camera_idx}")
                return cap, camera_idx
            cap.release()

        # Try discovering other /dev/video* devices if specified index failed
        candidates = sorted(glob.glob("/dev/video*"))
        log_warn(f"Failed to capture frame from /dev/video{camera_idx}. Available video nodes: {candidates}")
        
        # Check permissions on /dev/video*
        perm_issue = False
        for dev in candidates:
            if not os.access(dev, os.R_OK | os.W_OK):
                perm_issue = True
                break

        print(f"\n{TermColor.BOLD}{TermColor.WARNING}--- SELF-HEALING ACTION REQUIRED ---{TermColor.ENDC}")
        if perm_issue or len(candidates) > 0:
            print(f"Read/write access to video device is restricted or device is busy.")
            print(f"Please run the following command in another terminal:")
            print(f"    {TermColor.BOLD}{TermColor.CYAN}sudo chmod 666 /dev/video*{TermColor.ENDC}")
        else:
            print(f"No active camera found. Please plug in the USB camera.")
        
        choice = input(f"\nPress [{TermColor.BOLD}Enter{TermColor.ENDC}] to retry (or enter new camera index [0-9]): ").strip()
        if choice.isdigit():
            camera_idx = int(choice)


def check_and_heal_xarm(preferred_port: str = "auto", dry_run: bool = False) -> Optional[XArmDriver]:
    """
    Attempts to connect to the xArm manipulator over USB HID/Serial.
    If permission denied or device not accessible, prompts user in terminal.
    """
    if dry_run:
        log_info("Dry-run mode active: Skipping physical xArm connection.")
        return None

    while True:
        log_info(f"Searching for xArm 1S USB controller (port={preferred_port})...")
        ports = find_all_xarm_devices()
        
        # Check permissions on /dev/hidraw*
        hid_nodes = sorted(glob.glob("/dev/hidraw*"))
        perm_issue = any(not os.access(h, os.R_OK | os.W_OK) for h in hid_nodes)

        if not ports and perm_issue:
            print(f"\n{TermColor.BOLD}{TermColor.WARNING}--- SELF-HEALING ACTION REQUIRED ---{TermColor.ENDC}")
            print(f"Detected /dev/hidraw nodes, but access is restricted.")
            print(f"Please run the following command in another terminal:")
            print(f"    {TermColor.BOLD}{TermColor.CYAN}sudo chmod 666 /dev/hidraw*{TermColor.ENDC}")
            input(f"\nPress [{TermColor.BOLD}Enter{TermColor.ENDC}] after running the command to retry...")
            continue

        try:
            if preferred_port != "auto":
                target_port = preferred_port
            elif ports:
                target_port = ports[0][0] if isinstance(ports[0], (list, tuple)) else ports[0]
            else:
                target_port = "auto"
            driver = XArmDriver(port=target_port)
            if not driver.connect():
                raise RuntimeError(f"Failed to open connection on port {target_port}")
            log_info(f"Connected to xArm 1S on {driver.port}")
            # Query positions
            curr_pos = driver.get_servo_positions()
            log_info(f"Current servo readbacks: {curr_pos}")
            return driver
        except Exception as e:
            log_warn(f"Connection attempt failed: {e}")
            print(f"\n{TermColor.BOLD}{TermColor.WARNING}--- SELF-HEALING ACTION REQUIRED ---{TermColor.ENDC}")
            print(f"Could not connect to xArm controller.")
            print(f"Ensure the USB cable is securely connected and power switch is ON.")
            print(f"If permission denied, execute in another terminal:")
            print(f"    {TermColor.BOLD}{TermColor.CYAN}sudo chmod 666 /dev/hidraw*{TermColor.ENDC}")
            input(f"\nPress [{TermColor.BOLD}Enter{TermColor.ENDC}] to retry...")


class AutonomousSortingSystem:
    """Master controller managing perception, extrinsics, filtering, and pick-and-place execution."""

    def __init__(self, camera_idx: int = 4, port: str = "auto", dry_run: bool = False, step_mode: bool = False, ignore_marker_angles: bool = False):
        self.camera_idx = camera_idx
        self.port = port
        self.dry_run = dry_run
        self.step_mode = step_mode
        self.ignore_marker_angles = ignore_marker_angles

        self.cap: Optional[cv2.VideoCapture] = None
        self.driver: Optional[XArmDriver] = None
        config_dir = os.path.join(repo_root, "config")
        hw_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
        self.pipeline = ArenaVisionPipeline(config_dir=config_dir)
        self.params = load_kinematic_parameters(hw_yaml, arm_id="xarm_1")

        # Load hardware config poses
        with open(hw_yaml, "r") as f:
            self.hw_config = yaml.safe_load(f)
        self.poses = self.hw_config.get("manipulators", {}).get("xarm_1", {}).get("poses", {})

        # Boundaries of the Central Green Pickup Zone (Rows 7..10, Cols 1..5)
        # In world coordinates (mm):
        # Cols 1..5: X in [50.0, 300.0] mm
        # Rows 7..10: Y in [50.0, 250.0] mm
        self.pickup_zone_min_x = 50.0
        self.pickup_zone_max_x = 300.0
        self.pickup_zone_min_y = 50.0
        self.pickup_zone_max_y = 250.0

    def run_stage_1_hardware_check(self):
        """Stage 1: Verify and heal hardware access (Camera and xArm Manipulator)."""
        log_header("STAGE 1: HARDWARE CONNECTIVITY & SELF-HEALING ACCESS")
        self.cap, self.camera_idx = check_and_heal_camera(self.camera_idx)
        self.driver = check_and_heal_xarm(self.port, self.dry_run)
        log_info("Stage 1 Complete: All required hardware interfaces are active and accessible.")

    def run_stage_2_vision_calibration(self):
        """Stage 2: Camera Extrinsics & Interactive ArUco Alignment Verification."""
        log_header("STAGE 2: CAMERA EXTRINSICS & ARUCO MARKER ALIGNMENT")
        
        while True:
            # Capture clean frame
            for _ in range(8):
                self.cap.read()
            ret, frame = self.cap.read()
            if not ret or frame is None:
                log_warn("Failed to capture frame during calibration. Retrying...")
                time.sleep(0.5)
                continue

            # Verify ArUco Markers
            marker_status = self.pipeline.verify_aruco_markers(frame)
            all_detected = marker_status["all_detected"]
            all_aligned = marker_status["all_aligned"]
            reports = marker_status["reports"]

            print(f"\n{TermColor.BOLD}--- ArUco Marker Diagnostic Status ---{TermColor.ENDC}")
            for mid in [0, 1, 2, 3]:
                rep = reports.get(mid, {})
                status_color = TermColor.GREEN if rep.get("is_oriented_correctly") else TermColor.WARNING
                deg_str = f"{rep.get('measured_angle_deg')}°" if rep.get("measured_angle_deg") is not None else "N/A"
                print(f"  [{status_color}{'OK' if rep.get('is_oriented_correctly') else 'WARN'}{TermColor.ENDC}] "
                      f"Marker {mid} ({rep.get('name')}): Angle={deg_str} | {rep.get('advice')}")

            # Compute Extrinsics
            if all_detected:
                extrinsics = self.pipeline.compute_camera_extrinsics(frame)
                if extrinsics.get("success"):
                    pos = extrinsics["camera_position_world_mm"]
                    print(f"\n{TermColor.BOLD}--- Camera 3D Extrinsics Telemetry ---{TermColor.ENDC}")
                    print(f"  - Position (World mm):  X={pos['x']:.1f}, Y={pos['y']:.1f}, Z={pos['z']:.1f}")
                    print(f"  - Mounting Height Hc:   {extrinsics['height_mm']:.1f} mm")
                    print(f"  - Distance to Center:   {extrinsics['distance_to_arena_center_mm']:.1f} mm")
                    print(f"  - Depression Angle:     {extrinsics['depression_angle_deg']:.1f}°")
                    print(f"  - Viewing Side:         {extrinsics['viewing_side']}")

            if all_detected and (all_aligned or self.ignore_marker_angles):
                if all_aligned:
                    log_info("All 4 ArUco markers are placed and oriented correctly!")
                else:
                    log_info("All 4 ArUco markers detected. Proceeding with current marker orientations.")
                self.pipeline.calibrate_homography(frame, force_update=True)
                self.pipeline.lock_homography(True)
                log_info("Planar homography matrix H successfully calibrated and locked.")
                break
            else:
                print(f"\n{TermColor.BOLD}{TermColor.WARNING}--- ACTION REQUIRED: ADJUST MARKERS / CAMERA ---{TermColor.ENDC}")
                print("Follow the specific adjustment advice printed above.")
                ans = input(f"Press [{TermColor.BOLD}Enter{TermColor.ENDC}] to re-verify (or type 'c' to continue anyway): ").strip().lower()
                if ans in ['c', 'continue', 'proceed', 'skip']:
                    log_warn("User elected to continue with current marker alignment.")
                    self.pipeline.calibrate_homography(frame, force_update=True)
                    self.pipeline.lock_homography(True)
                    break

        log_info("Stage 2 Complete: Camera extrinsics verified and workspace homography locked.")

    def run_stage_3_perception_and_filtering(self) -> List[Tuple[DetectedCube, float, float, Dict[int, int]]]:
        """
        Stage 3: Warp arena, detect colored cubes, filter strictly to Central Green Pickup Zone,
        reject Orange Danger Zone, and verify IK reachability.
        """
        log_header("STAGE 3: PERCEPTION & CENTRAL PICKUP ZONE FILTERING")
        for _ in range(8):
            self.cap.read()
        ret, frame = self.cap.read()
        if not ret:
            log_err("Failed to capture frame for perception.")
            return []

        warped = self.pipeline.warp_arena(frame)
        if warped is None:
            log_err("Arena perspective warping failed.")
            return []

        # Save perception artifact
        cv2.imwrite("artifacts/current_arena_perception.png", warped)

        cubes = self.pipeline.detect_cubes(warped)
        log_info(f"Total raw object contours detected: {len(cubes)}")

        valid_targets = []
        for i, c in enumerate(cubes):
            xw = c.world_x
            yw = c.world_y

            # Check if within Central Green Pickup Zone (Rows 7..10, Cols 1..5)
            in_pickup_zone = (
                self.pickup_zone_min_x <= xw <= self.pickup_zone_max_x and
                self.pickup_zone_min_y <= yw <= self.pickup_zone_max_y
            )

            if not in_pickup_zone:
                log_warn(f"REJECTED: {c.color.upper()} cube at (X={xw:.1f}, Y={yw:.1f} mm) is inside ORANGE DANGER ZONE. Skipping.")
                continue

            # Compute arm polar coordinates
            r, theta_arm, rel_z = world_to_arm_polar(xw, yw, c.world_z, arm_id="xarm_1", params=self.params)
            
            # Solve IK for tabletop grasp
            ik_sol = solve_ik(r=r, theta=theta_arm, z=c.world_z, claw_tick=523, params=self.params)
            if not ik_sol.success:
                log_warn(f"UNREACHABLE: {c.color.upper()} cube at (r={r:.1f}mm, th={theta_arm:.1f}°): {ik_sol.message}")
                continue

            log_info(f"ACCEPTED: {c.color.upper()} cube in Central Pickup Zone at (X={xw:.1f}, Y={yw:.1f}) -> Polar (r={r:.1f}mm, th={theta_arm:.1f}°)")
            valid_targets.append((c, r, theta_arm, ik_sol.ticks))

        log_info(f"Stage 3 Complete: {len(valid_targets)} validated targets ready for autonomous scheduling.")
        return valid_targets

    def run_stage_4_scheduling_and_execution(self, targets: List[Tuple[DetectedCube, float, float, Dict[int, int]]]):
        """
        Stage 4: Nearest-First sorting, trajectory sequencing, and rear container box transfer.
        """
        log_header("STAGE 4: AUTONOMOUS SCHEDULING & PICK-AND-PLACE")
        if not targets:
            log_warn("No valid targets available in Central Pickup Zone to execute.")
            return

        # Sort targets: Nearest-First (smallest radial reach r first)
        targets_sorted = sorted(targets, key=lambda item: item[1])
        log_info(f"Scheduled {len(targets_sorted)} targets with Nearest-First ordering:")
        for idx, (c, r, th, _) in enumerate(targets_sorted):
            print(f"  {idx + 1}. {c.color.upper()} cube at r={r:.1f}mm, theta={th:.1f}° (World: X={c.world_x:.1f}, Y={c.world_y:.1f})")

        # Start from Home Pose
        home_ticks = self.poses.get("home_tucked", {}).get("ticks", [500, 500, 500, 500, 500, 500])
        claw_open_tick = 250
        claw_grip_tick = 520

        if not self.dry_run and self.driver:
            log_info("Moving to Upright Home Pose...")
            self.driver.set_servo_positions(home_ticks, duration_ms=1500)
            time.sleep(1.6)

        for idx, (cube, r, theta_arm, pick_ticks) in enumerate(targets_sorted):
            color = cube.color.lower()
            print(f"\n{TermColor.BOLD}{TermColor.BLUE}>>> Executing Target {idx + 1}/{len(targets_sorted)}: {color.upper()} Cube <<<{TermColor.ENDC}")

            if self.step_mode:
                ans = input(f"Press [{TermColor.BOLD}Enter{TermColor.ENDC}] to execute pick-and-place (or 's' to skip): ").strip().lower()
                if ans in ['s', 'skip']:
                    log_warn(f"Skipping target {idx + 1}")
                    continue

            # Trajectory Waypoints:
            # 1. Standoff Hover: R - 35mm, Z = 50.0mm (use_claw_tip=True to prevent table rubbing)
            standoff_r = max(135.0, r - 35.0)
            standoff_z = 50.0
            standoff_sol = solve_ik(r=standoff_r, theta=theta_arm, z=standoff_z, claw_tick=claw_open_tick, params=self.params, use_claw_tip=True)
            
            # 2. Descent & Advance: R, Z safe tabletop grasp clearance
            grasp_z = max(20.0, cube.world_z)
            grasp_sol = solve_ik(r=r, theta=theta_arm, z=grasp_z, claw_tick=claw_open_tick, params=self.params)

            # 3. Pure Vertical Lift: R, Z = 90.0mm (keeps object elevated cleanly off arena surface)
            lift_z = 90.0
            lift_sol = solve_ik(r=r, theta=theta_arm, z=lift_z, claw_tick=claw_grip_tick, params=self.params)
            if not lift_sol.success:
                lift_sol = solve_ik(r=standoff_r, theta=theta_arm, z=75.0, claw_tick=claw_grip_tick, params=self.params)

            # 4. Rear Drop Target
            drop_target = get_compartment_target(color, arm_id="xarm_1")
            if not drop_target or not drop_target.get("ik_success"):
                log_warn(f"No calibrated drop preset found for {color}; defaulting to Red box.")
                drop_target = get_compartment_target("red", arm_id="xarm_1")

            drop_ticks = drop_target["ik_ticks"]

            if self.dry_run:
                log_info(f"[DRY-RUN] Standoff ticks: {standoff_sol.ticks}")
                log_info(f"[DRY-RUN] Grasp ticks:    {grasp_sol.ticks}")
                log_info(f"[DRY-RUN] Lift ticks:     {lift_sol.ticks}")
                log_info(f"[DRY-RUN] Drop ticks:     {drop_ticks}")
                continue

            # Physical Execution Sequence
            # Step 1: Standoff Hover
            log_info("Phase 1: Transit to Standoff Hover...")
            self.driver.set_servo_positions(list(standoff_sol.ticks.values()), duration_ms=1400)
            time.sleep(1.5)

            # Step 2: Advance to Grasp
            log_info("Phase 2: Advancing to Tabletop Grasp...")
            self.driver.set_servo_positions(list(grasp_sol.ticks.values()), duration_ms=1000)
            time.sleep(1.1)

            # Step 3: Clamp Gripper
            log_info("Phase 3: Clamping Claw Gripper...")
            clamp_ticks = dict(grasp_sol.ticks)
            clamp_ticks[1] = claw_grip_tick
            self.driver.set_servo_positions(list(clamp_ticks.values()), duration_ms=600)
            time.sleep(0.7)

            # Step 4: Pure Vertical Lift
            log_info("Phase 4: Ascending with object...")
            self.driver.set_servo_positions(list(lift_sol.ticks.values()), duration_ms=1000)
            time.sleep(1.1)

            # Step 5: Transfer to Rear Flank Container Box (maintaining firm clamp on object)
            log_info(f"Phase 5: Transferring to {color.upper()} rear container box...")
            transfer_ticks = dict(drop_ticks)
            transfer_ticks[1] = claw_grip_tick
            self.driver.set_servo_positions(list(transfer_ticks.values()), duration_ms=1800)
            time.sleep(1.9)

            # Step 6: Release Gripper into container
            log_info("Phase 6: Releasing object...")
            release_ticks = dict(drop_ticks)
            release_ticks[1] = claw_open_tick
            self.driver.set_servo_positions(list(release_ticks.values()), duration_ms=600)
            time.sleep(0.7)

            # Step 7: Return to Upright Home Pose
            log_info("Phase 7: Returning safely to Upright Home Pose...")
            self.driver.set_servo_positions(home_ticks, duration_ms=1400)
            time.sleep(1.5)

            log_info(f"Target {idx + 1} ({color.upper()}) completed successfully!")

        log_info("Stage 4 Complete: All scheduled targets have been picked and sorted.")

    def run(self):
        """Executes the full 4-stage pipeline."""
        try:
            self.run_stage_1_hardware_check()
            self.run_stage_2_vision_calibration()
            targets = self.run_stage_3_perception_and_filtering()
            self.run_stage_4_scheduling_and_execution(targets)
            log_header("AUTONOMOUS SYSTEM PIPELINE FINISHED SUCCESSFULLY")
        finally:
            if self.cap is not None:
                self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="xArm 1S Autonomous Master Execution Pipeline")
    parser.add_argument("--dry-run", action="store_true", help="Run full perception and kinematics without moving physical servos")
    parser.add_argument("--step", action="store_true", help="Pause for interactive user confirmation between pick targets")
    parser.add_argument("--auto", action="store_true", help="Execute continuously without pausing")
    parser.add_argument("--camera", type=int, default=4, help="Camera device index (default: 4)")
    parser.add_argument("--port", type=str, default="auto", help="xArm controller port (default: auto)")
    parser.add_argument("--ignore-marker-angles", action="store_true", help="Calibrate if all 4 markers are detected, ignoring heading angle warnings")
    args = parser.parse_args()

    system = AutonomousSortingSystem(
        camera_idx=args.camera,
        port=args.port,
        dry_run=args.dry_run,
        step_mode=args.step,
        ignore_marker_angles=args.ignore_marker_angles,
    )
    system.run()


if __name__ == "__main__":
    main()
