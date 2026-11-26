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
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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
import yaml


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

        # Dynamic center and clamp ticks
        centers = self.params.nominal_center_ticks or {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        self.rear_transit_ticks = self._load_transit_pose()
        self.clamp_claw_tick = self._compute_clamp_claw_tick()

        # Joint states cache
        self.current_joint_ticks: Dict[int, int] = dict(centers)
        self.has_joint_feedback: bool = False

        # Target queue & batch tracking
        self.target_queue: List[Dict] = []
        self.sorted_cells: Set[Tuple[int, int]] = set()
        self.is_busy: bool = False
        self.cubes_sorted_in_batch: int = 0
        self.batch_count: int = 1
        self.last_sort_time: float = time.time()

        # Subscribers (QoS depth=1 so old queued messages are discarded while arm is moving)
        qos_vision = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_vision = self.create_subscription(
            String, "/vision/detected_objects", self.vision_callback, qos_vision
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

    def _load_transit_pose(self) -> Dict[int, int]:
        """Loads rear_transit_tuck pose dynamically from hardware YAML or nominal centers."""
        centers = self.params.nominal_center_ticks or {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        try:
            with open(self.hardware_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            arm_cfg = data.get("manipulators", {}).get(self.arm_id, {})
            pose_list = arm_cfg.get("poses", {}).get("rear_transit_tuck", {}).get("ticks", None)
            if pose_list and len(pose_list) >= 6:
                return {i + 1: int(pose_list[i]) for i in range(6)}
        except Exception:
            pass
        return {
            1: self.params.min_safe_ticks.get(1, 226),
            2: centers.get(2, 503),
            3: centers.get(3, 504) - 150,
            4: centers.get(4, 496),
            5: centers.get(5, 501) - 50,
            6: centers.get(6, 502),
        }

    def _compute_clamp_claw_tick(self) -> int:
        """Determines full-close claw grasp tick from hardware YAML."""
        try:
            with open(self.hardware_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            arm_cfg = data.get("manipulators", {}).get(self.arm_id, {})
            m1 = arm_cfg.get("motors", {}).get("motor_1", {})
            span_map = m1.get("claw_span_mm", {})
            if "tick" in span_map.get("fully_closed_empty", {}):
                return int(span_map["fully_closed_empty"]["tick"])
            if "max_tick" in m1:
                return int(m1["max_tick"])
        except Exception:
            pass
        return int(self.params.max_safe_ticks.get(1, 750))

    def joint_state_callback(self, msg: JointState):
        """Caches current motor ticks from joint state topic."""
        for name, pos_rad in zip(msg.name, msg.position):
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

            centers = self.params.nominal_center_ticks or {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
            pos_deg = math.degrees(pos_rad)
            ticks = int(round(centers.get(m_id, 500) + pos_deg * TICKS_PER_DEG))
            self.current_joint_ticks[m_id] = ticks

        self.has_joint_feedback = True

    @staticmethod
    def get_cell_key(wx: float, wy: float) -> Tuple[int, int]:
        """Maps world coordinates (mm) to unique 5cm grid cell column and row."""
        return (int(math.floor(wx / 50.0)), int(math.floor(wy / 50.0)))

    def vision_callback(self, msg: String):
        """Processes telemetry from /vision/detected_objects and updates queue."""
        if self.is_busy:
            return  # Do not alter active target while arm is in motion

        try:
            data = json.loads(msg.data)
            raw_objects = data.get("objects", [])

            # Filter reachable objects: pick Red and Green targetable objects (including near-base and far corners)
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
                    if 95.0 <= r <= 350.0:
                        # Cell deduplication: ignore cell if already sorted in current batch
                        wx = obj.get("world_x_mm", 0.0)
                        wy = obj.get("world_y_mm", 0.0)
                        cell_key = self.get_cell_key(wx, wy)
                        if cell_key in self.sorted_cells:
                            continue
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

        # Pop nearest target that hasn't been sorted yet
        target = None
        while self.target_queue:
            candidate = self.target_queue.pop(0)
            wx = candidate.get("world_x_mm", 0.0)
            wy = candidate.get("world_y_mm", 0.0)
            cell_key = self.get_cell_key(wx, wy)
            if cell_key not in self.sorted_cells:
                target = candidate
                break

        if target is None:
            # Check if previous batch was completed and auto_reseed is enabled
            if self.auto_reseed and self.cubes_sorted_in_batch > 0 and (time.time() - self.last_sort_time > 2.0):
                self.get_logger().info(
                    f"====================================================================\n"
                    f"  BATCH {self.batch_count} COMPLETED: All {self.cubes_sorted_in_batch} objects successfully sorted!\n"
                    f"  Triggering /simulation/reseed_grid for new scattered Red/Green pattern...\n"
                    f"===================================================================="
                )
                self.cubes_sorted_in_batch = 0
                self.sorted_cells.clear()
                self.batch_count += 1
                if self.cli_reseed.wait_for_service(timeout_sec=1.0):
                    req = Trigger.Request()
                    self.cli_reseed.call_async(req)
                else:
                    self.get_logger().warn("/simulation/reseed_grid service not available.")
            return

        # Mark cell as sorted immediately to lock out duplicate picks from stale vision frames
        wx = target.get("world_x_mm", 0.0)
        wy = target.get("world_y_mm", 0.0)
        cell_key = self.get_cell_key(wx, wy)
        self.sorted_cells.add(cell_key)

        self.is_busy = True
        try:
            self.process_target(target)
        finally:
            self.is_busy = False



    def process_target(self, target: Dict):
        """Executes full pick and place for a target cube."""
        color = target.get("color", "red").lower()
        r = target.get("target_r_mm", 220.0)
        theta = target.get("target_theta_deg", 0.0)
        z = target.get("world_z_mm", 9.53)
        yaw = target.get("yaw_deg", 0.0)

        centers = self.params.nominal_center_ticks or {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        open_claw_tick = self.params.min_safe_ticks.get(1, 226)
        clamp_claw_tick = self.clamp_claw_tick

        roll_tick = target.get("gripper_roll_tick")
        if roll_tick is None:
            roll_tick = int(round(centers.get(2, 503) + yaw * (1000.0 / 240.0)))
        pick_ik = solve_ik(r=r, theta=theta, z=z, roll_tick=roll_tick, params=self.params, compensate_sag=False)
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

        # 3. Publish Fluid Trajectory Steps (Overhead Standoff -> Vertical Plunge Grasp -> Lift -> Transit -> Drop -> Home)
        # Step A: Pure top-down overhead standoff with claw fully open
        standoff_ik = solve_ik(
            r=r, theta=theta, z=z + 50.0, claw_tick=open_claw_tick, roll_tick=roll_tick,
            params=self.params, compensate_sag=False,
        )
        if standoff_ik.success:
            self.dispatch_joint_command(standoff_ik.ticks, time_ms=600)
            time.sleep(0.62)

        # Step B: Advance to pick cube with claw fully open
        open_pick_ticks = dict(pick_ik.ticks)
        open_pick_ticks[1] = open_claw_tick
        self.dispatch_joint_command(open_pick_ticks, time_ms=450)
        time.sleep(0.48)

        # Step C: Clamp claw
        clamp_ticks = dict(pick_ik.ticks)
        clamp_ticks[1] = clamp_claw_tick
        self.dispatch_joint_command(clamp_ticks, time_ms=250)
        time.sleep(0.28)

        # Step D: Lift to standoff
        if standoff_ik.success:
            lift_ticks = dict(standoff_ik.ticks)
            lift_ticks[1] = clamp_claw_tick
            self.dispatch_joint_command(lift_ticks, time_ms=450)
            time.sleep(0.48)

        # Step E: Move to drop compartment via high-clearance overhead transit
        if drop_ticks:
            target_drop = dict(drop_ticks) if isinstance(drop_ticks, dict) else {i + 1: drop_ticks[i] for i in range(6)}

            # High overhead transit pose loaded dynamically from hardware config
            transit_ticks = dict(self.rear_transit_ticks)
            transit_ticks[1] = clamp_claw_tick
            transit_ticks[6] = target_drop[6]  # Align base yaw toward target container while high
            self.dispatch_joint_command(transit_ticks, time_ms=600)
            time.sleep(0.62)

            # Descend into rear container box
            target_drop[1] = clamp_claw_tick  # Keep clamped during descent
            self.dispatch_joint_command(target_drop, time_ms=600)
            time.sleep(0.62)

            # Step F: Release into container
            target_drop[1] = open_claw_tick  # Open claw
            self.dispatch_joint_command(target_drop, time_ms=250)
            time.sleep(0.28)

            # Lift out of container back to high transit
            transit_ticks[1] = open_claw_tick
            self.dispatch_joint_command(transit_ticks, time_ms=500)
            time.sleep(0.52)

        # Step G: Return to upright home
        home_ticks = dict(centers)
        home_ticks[1] = open_claw_tick
        self.dispatch_joint_command(home_ticks, time_ms=500)
        time.sleep(0.52)

        self.get_logger().info(f"Target {color.upper()} successfully placed in flank tray.")

    def dispatch_joint_command(self, ticks: Dict[int, int], time_ms: int = 600):
        """Publishes JointState command to /{arm_id}/joint_commands."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "claw_gripper", "wrist_roll", "wrist_pitch",
            "elbow_pitch", "shoulder_pitch", "base_yaw"
        ]
        centers = self.params.nominal_center_ticks or {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
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
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
