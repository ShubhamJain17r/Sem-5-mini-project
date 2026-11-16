#!/usr/bin/env python3
"""
Interactive Pick-and-Place Kinematic Calibration & Verification Script.

Executes the official 5-phase trajectory sequence using analytical polar IK:
  Phase 1: Transit to Radial Standoff (r - 35mm, Z = +37.5mm, Claw open)
  Phase 2: Horizontal Advance into Grasp (r, Z = +12.5mm + z_sag)
  Phase 3: Clamp Gripper (Motor 1 = grip_tick)
  Phase 4: Ascend and Radial Retract (r - 40mm, Z = +47.5mm)
  Phase 5: ~70° Slew to Flank 2x2 Sorting Tray (@ drop_yaw_tick)
  Phase 6: Release Gripper in Sorting Compartment
  Phase 7: Return Home

Can publish to ROS 2 topic (/xarm_1/joint_commands) or run standalone directly over USB.
"""

import argparse
import math
import os
import sys
import time
from typing import Dict, Optional

# Ensure package root is in sys.path for direct execution
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.abspath(os.path.join(script_dir, ".."))
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

import yaml

# Optional ROS 2 import
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    Node = object

from sorting_arm.kinematics import (
    DEFAULT_PARAMS,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
)
from sorting_arm.trajectory import generate_joint_trajectory
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices


class PickAndPlaceCommander(Node if HAS_ROS2 else object):
    """Commands 5-phase pick-and-place trajectories via analytical IK in Direct or ROS 2 mode."""

    def __init__(
        self,
        arm_id: str = "xarm_1",
        config_dir: str = "config",
        direct_mode: bool = True,
        port: str = "auto",
        smooth: bool = True,
        closed_loop: bool = True,
    ):
        self.arm_id = arm_id
        self.config_dir = config_dir
        self.direct_mode = direct_mode
        self.port = port
        self.smooth = smooth
        self.closed_loop = closed_loop
        self.driver = None

        self.hardware_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
        self.params = load_kinematic_parameters(self.hardware_yaml, arm_id=self.arm_id)

        # Load hardware specs
        self.hw_config = self._load_yaml(self.hardware_yaml)
        arm_cfg = self.hw_config.get("manipulators", {}).get(self.arm_id, {})
        motors_cfg = arm_cfg.get("motors", {})
        box_cfg = arm_cfg.get("collection_box", {})

        # Grip / Claw calibration ticks
        m1_cfg = motors_cfg.get("motor_1", {})
        self.claw_open_tick = int(m1_cfg.get("claw_span_mm", {}).get("fully_open", {}).get("tick", 230))
        self.claw_grip_tick = int(m1_cfg.get("claw_span_mm", {}).get("grip_cube_25mm", {}).get("tick", 605))

        # Drop box nominal yaw tick
        self.drop_yaw_tick = int(box_cfg.get("base_yaw_drop_tick", 145))

        # Arena picking elevations (mm)
        arena_cfg = self.hw_config.get("workspace", {}).get("picking_arena", {})
        self.cube_pick_z = float(arena_cfg.get("cube_center_z_mm", 12.5))

        # Load deviations from unified hardware config (default 0)
        self.deviations = {
            sid: int(motors_cfg.get(f"motor_{sid}", {}).get("deviation", 0))
            for sid in range(1, 7)
        }

        if not self.direct_mode and HAS_ROS2:
            super().__init__("pick_and_place_calibrator", namespace=self.arm_id)
            self.cmd_pub = self.create_publisher(JointState, "joint_commands", 10)
            self.log(f"Operating in ROS 2 Topic Mode (publishing to '/{self.arm_id}/joint_commands')")
        else:
            self.direct_mode = True
            target_port = self.port
            if target_port in ["auto", "", None]:
                all_devices = find_all_xarm_devices()
                if len(all_devices) >= 1:
                    target_port = all_devices[0][0]

            self.log(f"Operating in Direct Hardware USB Mode (port: {target_port})")
            self.driver = XArmDriver(
                port=target_port,
                deviations=self.deviations,
                min_ticks=self.params.min_safe_ticks,
                max_ticks=self.params.max_safe_ticks,
            )
            connected = self.driver.connect()
            if connected:
                self.log(f"Successfully connected to xArm on {self.driver.port} ({self.driver.interface_type})")
            else:
                self.log("[WARN] No physical xArm found on USB; running in offline simulation mode.")

        self.log(
            f"Initialized Pick-and-Place Calibrator for {self.arm_id}\n"
            f"  Claw Open Tick: {self.claw_open_tick} | Grip Tick: {self.claw_grip_tick}\n"
            f"  Drop Yaw Tick: {self.drop_yaw_tick} | Cube Pick Z: {self.cube_pick_z} mm\n"
            f"  Smooth Trajectory: {self.smooth} | Closed-Loop Verification: {self.closed_loop}"
        )

    def log(self, text: str):
        """Unified logging output."""
        print(f"[{self.arm_id.upper()}] {text}")

    def _load_yaml(self, path: str) -> dict:
        candidates = [
            path,
            os.path.join(os.getcwd(), path),
            os.path.join("/workspace", path),
        ]
        for c in candidates:
            if os.path.exists(c):
                try:
                    with open(c, "r") as f:
                        return yaml.safe_load(f) or {}
                except Exception:
                    pass
        return {}

    def _load_deviations(self, path: str, arm_name: str) -> Dict[int, int]:
        candidates = [
            path,
            os.path.join(self.config_dir, "servo_calibration.yaml"),
            os.path.join(os.getcwd(), "config/servo_calibration.yaml"),
        ]
        for c in candidates:
            if os.path.exists(c):
                try:
                    with open(c, "r") as f:
                        data = yaml.safe_load(f) or {}
                        devs = data.get(arm_name, {}).get("servo_deviations", {})
                        return {
                            1: int(devs.get("motor_1_claw", 0)),
                            2: int(devs.get("motor_2_wrist_roll", 0)),
                            3: int(devs.get("motor_3_wrist_pitch", 0)),
                            4: int(devs.get("motor_4_elbow", 0)),
                            5: int(devs.get("motor_5_shoulder", 0)),
                            6: int(devs.get("motor_6_base_yaw", 0)),
                        }
                except Exception:
                    pass
        return {i: 0 for i in range(1, 7)}

    def send_pose_ticks(self, ticks: Dict[int, int], move_time_ms: int = 1200) -> bool:
        """Dispatch joint ticks to physical hardware or publish via ROS 2."""
        if self.direct_mode:
            if self.driver and self.driver.is_connected:
                if self.closed_loop:
                    return self.driver.send_motion_and_wait(
                        ticks,
                        time_ms=move_time_ms,
                        tolerance_ticks=10,
                        stall_detection=True,
                    )
                else:
                    ok = self.driver.send_multi_servo_move(ticks, time_ms=move_time_ms)
                    time.sleep((move_time_ms + 150) / 1000.0)
                    return ok
            else:
                tick_str = ", ".join(f"M{k}:{v}" for k, v in sorted(ticks.items()))
                print(f"    -> [SIM] Pose Ticks: {tick_str} (time: {move_time_ms}ms)")
                time.sleep((move_time_ms + 150) / 1000.0)
                return True
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f"motor_{i}" for i in range(1, 7)]
            msg.position = [float(ticks.get(i, 500)) for i in range(1, 7)]
            self.cmd_pub.publish(msg)
            time.sleep((move_time_ms + 150) / 1000.0)
            return True

    def send_ik_target(
        self,
        r: float,
        theta: float,
        z: float,
        claw_tick: int,
        roll_tick: int = 500,
        move_time_ms: int = 1200,
        phi: Optional[float] = None,
    ) -> bool:
        """Solve IK and command joints to reach cylindrical target (r, theta, z)."""
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
            self.log(f"IK Failed for (r={r:.1f}, θ={theta:.1f}, z={z:.1f}): {sol.message}")
            return False

        return self.send_pose_ticks(sol.ticks, move_time_ms=move_time_ms)

    def run_5_phase_pick_and_place(
        self,
        pick_r: float = 220.0,
        pick_theta: float = 0.0,
        pick_z: Optional[float] = None,
        color: str = "red",
        z_sag_mm: float = 0.0,
    ) -> bool:
        """Execute complete 5-phase pick and place sequence."""
        if pick_z is None:
            pick_z = self.cube_pick_z

        # Clamp pick height so arm never dips below ground level (table surface Z=0)
        effective_pick_z = max(10.0, pick_z + z_sag_mm)
        standoff_r = max(130.0, pick_r - 35.0)
        hover_z = effective_pick_z + 25.0
        lift_z = effective_pick_z + 35.0
        retract_r = max(130.0, pick_r - 50.0)

        print("\n" + "=" * 65)
        print(f"  EXECUTING 5-PHASE PICK-AND-PLACE ({color.upper()} CUBE)")
        print(f"  Target: R={pick_r:.1f} mm | θ={pick_theta:.1f}° | Z={effective_pick_z:.1f} mm (sag: {z_sag_mm:+.1f}mm)")
        print("=" * 65)

        # Phase 0: Upright Physical Reset Pose
        print("[Phase 0] Moving to Upright Physical Reset Pose (all servos @ 500 ticks)...")
        home_ticks = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
        self.send_pose_ticks(home_ticks, move_time_ms=1500)

        # Phase 1: Azimuth Slew & Radial Standoff Hover (r - 35mm, hover_z, claw open)
        print(f"[Phase 1] Opening Claw & Standoff Hover @ (R={standoff_r:.1f}mm, Z={hover_z:.1f}mm)...")
        if not self.send_ik_target(standoff_r, pick_theta, hover_z, self.claw_open_tick, move_time_ms=1400):
            return False

        # Phase 2: Horizontal Advance into Grasp (pick_r, pick_z)
        print(f"[Phase 2] Descending & Advancing onto Cube @ (R={pick_r:.1f}mm, Z={effective_pick_z:.1f}mm)...")
        if not self.send_ik_target(pick_r, pick_theta, effective_pick_z, self.claw_open_tick, move_time_ms=1200):
            return False

        # Phase 3: Clamp Gripper on Cube
        print(f"[Phase 3] Clamping Gripper (Motor 1 -> {self.claw_grip_tick} ticks)...")
        if not self.send_ik_target(pick_r, pick_theta, effective_pick_z, self.claw_grip_tick, move_time_ms=800):
            return False
        time.sleep(0.3)

        # Phase 4: Lift & Radial Retract (lift_z, retract_r)
        print(f"[Phase 4] Lifting to Z={lift_z:.1f}mm and Retracting to R={retract_r:.1f}mm...")
        if not self.send_ik_target(retract_r, pick_theta, lift_z, self.claw_grip_tick, move_time_ms=1000):
            return False

        # Phase 5: Slew ~70° to Flank 2x2 Sorting Tray Compartment
        print(f"[Phase 5] Swiveling ~70° to Flank 2x2 Tray (Color: {color.upper()})...")
        target_info = get_compartment_target(color, arm_id=self.arm_id, config_path=self.hardware_yaml)
        drop_yaw = target_info.get("drop_yaw_tick", self.drop_yaw_tick) if target_info else self.drop_yaw_tick

        # High clearance drop pose over flank tray (safe elevation Z ≈ 56mm above tray rim Z=35mm)
        preset_ticks = target_info.get("drop_pose_preset") if target_info else None
        if preset_ticks and len(preset_ticks) == 6:
            drop_pose = {
                1: self.claw_grip_tick,
                2: preset_ticks[1],
                3: preset_ticks[2],
                4: preset_ticks[3],
                5: preset_ticks[4],
                6: drop_yaw,
            }
        else:
            drop_pose = {
                1: self.claw_grip_tick,
                2: 500,
                3: 830,
                4: 160,
                5: 470,
                6: drop_yaw,
            }
        self.send_pose_ticks(drop_pose, move_time_ms=1800)

        # Phase 6: Release Gripper in Box
        print("[Phase 6] Opening Claw to Drop Object...")
        drop_pose[1] = self.claw_open_tick
        self.send_pose_ticks(drop_pose, move_time_ms=700)
        time.sleep(0.4)

        # Phase 7: Retract and Return Home
        print("[Phase 7] Clearing Box Rim and Returning Home...")
        self.send_pose_ticks(home_ticks, move_time_ms=1600)

        print("\n>>> Pick-and-Place Cycle Finished Successfully! <<<\n")
        return True

    def close(self):
        """Clean teardown."""
        if self.direct_mode and self.driver:
            self.driver.emergency_stop()
            self.driver.disconnect()
        elif HAS_ROS2 and not self.direct_mode:
            self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="xArm 1S Kinematic Pick-and-Place Test")
    parser.add_argument("--r", type=float, default=220.0, help="Pick radius (140-300mm, default: 220mm)")
    parser.add_argument("--theta", type=float, default=0.0, help="Pick azimuth (-45 to +45 deg, default: 0)")
    parser.add_argument("--z", type=float, default=None, help="Pick height (default from YAML: 9.53mm)")
    parser.add_argument("--color", default="red", choices=["red", "green", "blue", "yellow"], help="Target drop box color")
    parser.add_argument("--sag", type=float, default=0.0, help="Empirical Z-sag compensation in mm (e.g. +3.0 to lift)")
    parser.add_argument("--arm", default="xarm_1", help="Arm instance (default: xarm_1)")
    parser.add_argument("--port", default="auto", help="USB device port or 'auto'")
    parser.add_argument("--ros", action="store_true", help="Force ROS 2 topic publishing mode")
    parser.add_argument("--direct", action="store_true", help="Force direct USB hardware mode (default)")
    parser.add_argument("--open-loop", action="store_true", help="Disable closed-loop position verification")
    parser.add_argument("--no-smooth", action="store_true", help="Disable trajectory smoothing")
    args = parser.parse_args()

    use_ros = args.ros and HAS_ROS2
    if args.ros and not HAS_ROS2:
        print("[WARN] --ros requested but 'rclpy' is not installed. Falling back to direct USB mode.")

    if use_ros:
        rclpy.init()

    node = PickAndPlaceCommander(
        arm_id=args.arm,
        direct_mode=not use_ros,
        port=args.port,
        smooth=not args.no_smooth,
        closed_loop=not args.open_loop,
    )

    try:
        time.sleep(0.3)
        node.run_5_phase_pick_and_place(
            pick_r=args.r,
            pick_theta=args.theta,
            pick_z=args.z,
            color=args.color,
            z_sag_mm=args.sag,
        )
    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted pick-and-place.")
    finally:
        node.close()
        if use_ros:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
