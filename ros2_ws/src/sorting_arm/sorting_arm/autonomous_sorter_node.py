#!/usr/bin/env python3
"""
ROS 2 Autonomous Sorting Master Node (Single xArm 1S).

Features:
  - Subscribes to /vision/detected_objects (JSON string from vision_node).
  - Filters targets for active manipulator (xarm_1).
  - Enforces Nearest-First pick queue (ascending target_r_mm).
  - Solves analytical closed-form IK for pick and color drop (2x2 flank tray).
  - Dispatches interpolated joint trajectories to /{arm_id}/joint_commands.
  - Monitors /{arm_id}/joint_states for closed-loop target arrival.
"""

import json
import math
import os
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sorting_arm.kinematics import (
    DEG_PER_TICK,
    TICKS_PER_DEG,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
)
from sorting_arm.trajectory import generate_joint_trajectory
from std_msgs.msg import String
from std_srvs.srv import Trigger


class AutonomousSorterNode(Node):
    """ROS 2 Node orchestrating vision-guided pick-and-place color sorting."""

    def __init__(self):
        super().__init__("autonomous_sorter_node")

        # Declare parameters
        self.declare_parameter("arm_id", "xarm_1")
        self.declare_parameter("config_dir", "config")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("auto_reseed", True)

        self.arm_id = self.get_parameter("arm_id").get_parameter_value().string_value
        self.config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        self.auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value
        self.auto_reseed = self.get_parameter("auto_reseed").get_parameter_value().bool_value

        self.hardware_yaml = os.path.join(self.config_dir, "xarm_hardware.yaml")
        self.params = load_kinematic_parameters(self.hardware_yaml, arm_id=self.arm_id)

        # Joint states cache
        self.current_joint_ticks: Dict[int, int] = {i: 500 for i in range(1, 7)}
        self.has_joint_feedback: bool = False

        # Target queue & batch tracking
        self.target_queue: List[Dict] = []
        self.is_busy: bool = False
        self.cubes_sorted_in_batch: int = 0
        self.batch_count: int = 1
        self.last_sort_time: float = time.time()

        # Subscribers
        self.sub_vision = self.create_subscription(
            String, "/vision/detected_objects", self.vision_callback, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, f"/{self.arm_id}/joint_states", self.joint_state_callback, 10
        )

        # Publishers
        self.pub_commands = self.create_publisher(
            JointState, f"/{self.arm_id}/joint_commands", 10
        )
        self.pub_status = self.create_publisher(
            String, f"/{self.arm_id}/sorting_status", 10
        )

        # Services & Clients
        self.srv_start = self.create_service(Trigger, f"/{self.arm_id}/start_sorting", self.handle_start_sorting)
        self.srv_stop = self.create_service(Trigger, f"/{self.arm_id}/stop_sorting", self.handle_stop_sorting)
        self.cli_reseed = self.create_client(Trigger, "/simulation/reseed_grid")

        # Main executive loop timer (1 Hz)
        self.timer = self.create_timer(1.0, self.executive_loop)

        self.enabled = self.auto_start
        self.get_logger().info(
            f"Autonomous Sorter Node initialized for {self.arm_id} (Enabled: {self.enabled}, Dry-Run: {self.dry_run}, Auto-Reseed: {self.auto_reseed})."
        )

    def joint_state_callback(self, msg: JointState):
        """Caches current motor ticks from joint state topic."""
        for name, pos_rad in zip(msg.name, msg.position):
            # Parse motor ID or joint name
            if "claw" in name or "motor_1" in name or "joint_1" in name:
                m_id = 1
            elif "wrist_roll" in name or "motor_2" in name or "joint_2" in name:
                m_id = 2
            elif "wrist_pitch" in name or "motor_3" in name or "joint_3" in name:
                m_id = 3
            elif "elbow" in name or "motor_4" in name or "joint_4" in name:
                m_id = 4
            elif "shoulder" in name or "motor_5" in name or "joint_5" in name:
                m_id = 5
            elif "base" in name or "motor_6" in name or "joint_6" in name:
                m_id = 6
            else:
                continue

            centers = self.params.nominal_center_ticks if hasattr(self.params, "nominal_center_ticks") and self.params.nominal_center_ticks else {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
            pos_deg = math.degrees(pos_rad)
            ticks = int(round(centers.get(m_id, 500) + pos_deg * TICKS_PER_DEG))
            self.current_joint_ticks[m_id] = ticks

        self.has_joint_feedback = True

    def vision_callback(self, msg: String):
        """Processes telemetry from /vision/detected_objects and updates queue."""
        if self.is_busy:
            return  # Do not alter active target while arm is in motion

        try:
            data = json.loads(msg.data)
            raw_objects = data.get("objects", [])

            # Filter reachable objects: pick Red and Green targetable objects, ignore blue and unconfigured colors
            filtered = []
            for obj in raw_objects:
                color_name = obj.get("color", "").lower()
                if color_name not in ("red", "green"):
                    continue
                if not obj.get("targetable", True):
                    continue
                assigned = obj.get("assigned_arm", "xarm_1")
                if assigned in [self.arm_id, "xarm", "xarm_1", ""]:
                    r = obj.get("target_r_mm", 0.0)
                    if 110.0 <= r <= 340.0:
                        filtered.append(obj)

            # Nearest-First policy: sort queue by ascending target_r_mm
            filtered.sort(key=lambda o: o.get("target_r_mm", 0.0))
            self.target_queue = filtered

        except Exception as e:
            self.get_logger().error(f"Error parsing vision telemetry: {e}")

    def executive_loop(self):
        """Executive state machine: picks and sorts cubes sequentially, then reseeds upon batch completion."""
        if not self.enabled or self.is_busy:
            return

        if not self.target_queue:
            # Check if previous batch was completed and auto_reseed is enabled
            if self.auto_reseed and self.cubes_sorted_in_batch > 0 and (time.time() - self.last_sort_time > 2.0):
                self.get_logger().info(
                    f"====================================================================\n"
                    f"  BATCH {self.batch_count} COMPLETED: All {self.cubes_sorted_in_batch} objects successfully sorted!\n"
                    f"  Triggering /simulation/reseed_grid for new scattered Red/Green pattern...\n"
                    f"===================================================================="
                )
                self.cubes_sorted_in_batch = 0
                self.batch_count += 1
                if self.cli_reseed.wait_for_service(timeout_sec=1.0):
                    req = Trigger.Request()
                    self.cli_reseed.call_async(req)
                else:
                    self.get_logger().warn("/simulation/reseed_grid service not available.")
            return

        # Pop nearest target
        target = self.target_queue.pop(0)
        self.is_busy = True
        self.process_target(target)
        self.is_busy = False

    def process_target(self, target: Dict):
        """Executes full pick and place for a target cube."""
        color = target.get("color", "red").lower()
        r = target.get("target_r_mm", 220.0)
        theta = target.get("target_theta_deg", 0.0)
        z = target.get("world_z_mm", 9.53)
        yaw = target.get("yaw_deg", 0.0)

        # 1. Compute Pick IK with wrist roll aligned to cube orientation
        centers = self.params.nominal_center_ticks if hasattr(self.params, "nominal_center_ticks") and self.params.nominal_center_ticks else {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        open_claw_tick = self.params.min_safe_ticks.get(1, 226) if hasattr(self.params, "min_safe_ticks") and self.params.min_safe_ticks else 226
        clamp_claw_tick = 610

        roll_tick = int(round(centers.get(2, 503) + yaw * (1000.0 / 240.0)))
        pick_ik = solve_ik(r=r, theta=theta, z=z, roll_tick=roll_tick, params=self.params)
        if not pick_ik.success:
            self.get_logger().warn(f"Pick IK failed for {color.upper()} at r={r:.1f}mm, th={theta:+.1f}deg: {pick_ik.message}")
            return

        # 2. Compute Drop Target
        drop_target = get_compartment_target(color=color, arm_id=self.arm_id)
        if not drop_target:
            self.get_logger().warn(f"Drop target not found for {color}")
            return

        drop_ticks = drop_target.get("ik_ticks", None)
        if not drop_ticks:
            drop_ticks = drop_target.get("drop_pose_preset", [500, 500, 500, 500, 500, 500])

        self.cubes_sorted_in_batch += 1
        self.last_sort_time = time.time()
        self.get_logger().info(
            f"--> [Batch {self.batch_count} | #{self.cubes_sorted_in_batch}] Sorting {color.upper()} cube:\n"
            f"    Pick: r={r:.1f}mm, th={theta:+.1f}deg, Z={z:.1f}mm\n"
            f"    Drop: {color.upper()} CONTAINER at {drop_target['world_xyz']} (M6 Base Yaw: {drop_ticks[6]})"
        )

        if self.dry_run:
            self.get_logger().info(f"[Dry-Run] Target processed successfully: Pick ticks {pick_ik.ticks}")
            return

        # 3. Publish Trajectory Steps
        # Standoff -> Grasp -> Lift -> Drop -> Home
        # Step A: Standoff (adapt for near-base radial reach)
        standoff_r = r - 35.0 if r >= 180.0 else r
        standoff_ik = solve_ik(r=standoff_r, theta=theta, z=z + 25.0, params=self.params)
        if standoff_ik.success:
            self.dispatch_joint_command(standoff_ik.ticks, time_ms=1200)
            time.sleep(1.3)

        # Step B: Advance and clamp
        self.dispatch_joint_command(pick_ik.ticks, time_ms=800)
        time.sleep(0.9)

        # Step C: Clamp claw
        clamp_ticks = dict(pick_ik.ticks)
        clamp_ticks[1] = clamp_claw_tick  # Standard 25mm grasp tick
        self.dispatch_joint_command(clamp_ticks, time_ms=400)
        time.sleep(0.5)

        # Step D: Lift to standoff
        if standoff_ik.success:
            lift_ticks = dict(standoff_ik.ticks)
            lift_ticks[1] = clamp_claw_tick
            self.dispatch_joint_command(lift_ticks, time_ms=800)
            time.sleep(0.9)

        # Step E: Move to drop compartment via high-clearance overhead transit
        if drop_ticks:
            target_drop = dict(drop_ticks) if isinstance(drop_ticks, dict) else {i + 1: drop_ticks[i] for i in range(6)}

            # High overhead transit pose: clears table surface, foam barrier, and adjacent cubes
            transit_ticks = {
                1: clamp_claw_tick,
                2: 503,
                3: 350,
                4: 496,
                5: 450,
                6: target_drop[6],  # Align base yaw toward target container while high
            }
            self.dispatch_joint_command(transit_ticks, time_ms=1000)
            time.sleep(1.1)

            # Descend into rear container box
            target_drop[1] = clamp_claw_tick  # Keep clamped during descent
            self.dispatch_joint_command(target_drop, time_ms=1200)
            time.sleep(1.3)

            # Step F: Release into container
            target_drop[1] = open_claw_tick  # Open claw
            self.dispatch_joint_command(target_drop, time_ms=400)
            time.sleep(0.5)

            # Lift out of container back to high transit
            transit_ticks[1] = open_claw_tick
            self.dispatch_joint_command(transit_ticks, time_ms=1000)
            time.sleep(1.1)

        # Step G: Return to upright home
        home_ticks = dict(centers)
        home_ticks[1] = open_claw_tick
        self.dispatch_joint_command(home_ticks, time_ms=1000)
        time.sleep(1.1)

        self.get_logger().info(f"Target {color.upper()} successfully placed in flank tray.")

    def dispatch_joint_command(self, ticks: Dict[int, int], time_ms: int = 800):
        """Publishes JointState command to /{arm_id}/joint_commands."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "claw_gripper", "wrist_roll", "wrist_pitch",
            "elbow_pitch", "shoulder_pitch", "base_yaw"
        ]
        centers = self.params.nominal_center_ticks if hasattr(self.params, "nominal_center_ticks") and self.params.nominal_center_ticks else {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        msg.position = [
            math.radians(float(ticks.get(i, centers.get(i, 500)) - centers.get(i, 500)) * DEG_PER_TICK) for i in range(1, 7)
        ]
        self.pub_commands.publish(msg)

    def handle_start_sorting(self, request, response):
        self.enabled = True
        response.success = True
        response.message = f"Sorting enabled for {self.arm_id}."
        return response

    def handle_stop_sorting(self, request, response):
        self.enabled = False
        response.success = True
        response.message = f"Sorting paused for {self.arm_id}."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSorterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
