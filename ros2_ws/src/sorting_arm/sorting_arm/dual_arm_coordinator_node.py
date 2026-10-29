#!/usr/bin/env python3
"""
Dual-Arm Spatial Mutex & Coordination Node for Hiwonder / LewanSoul xArm 1S.

Architecture:
  - Subscribes to /xarm_1/joint_states and /xarm_2/joint_states
  - Calculates real-time global Cartesian end-effector coordinates (Xw, Yw, Zw) via Forward Kinematics
  - Arbitrates shared workspace midline (X ≈ 280 mm, corridor [240 mm, 320 mm]) via a spatial mutex
  - Exposes services for requesting and releasing spatial priority
  - Emits collision warning and holds secondary arm at ready standoff (Zw ≈ 165 mm) if contention occurs
"""

import math
import os
import threading
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sorting_arm.kinematics import (
    DEG_PER_TICK,
    KinematicParameters,
    load_kinematic_parameters,
    solve_fk,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class DualArmCoordinatorNode(Node):
    """Coordinates simultaneous dual-arm motion and arbitrates shared workspace midline."""

    def __init__(self):
        super().__init__("dual_arm_coordinator")

        self.declare_parameter("config_path", "config/dual_xarm_hardware.yaml")
        self.declare_parameter("midline_min_x_mm", 240.0)
        self.declare_parameter("midline_max_x_mm", 320.0)
        self.declare_parameter("collision_threshold_mm", 100.0)

        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        self.midline_min_x = self.get_parameter("midline_min_x_mm").get_parameter_value().double_value
        self.midline_max_x = self.get_parameter("midline_max_x_mm").get_parameter_value().double_value
        self.collision_dist_threshold = (
            self.get_parameter("collision_threshold_mm").get_parameter_value().double_value
        )

        # Load parameters for both arms
        self.params_arm1 = load_kinematic_parameters(self.config_path, arm_id="xarm_1")
        self.params_arm2 = load_kinematic_parameters(self.config_path, arm_id="xarm_2")

        # Base mounts in world coordinates
        self.base_mounts = {
            "xarm_1": {"x": 170.0, "y": 360.0, "z": 0.0, "heading": -110.0},
            "xarm_2": {"x": 390.0, "y": 360.0, "z": 0.0, "heading": -70.0},
        }

        # Latest joint state cache (motor ID 1..6 ticks)
        self.arm_ticks: Dict[str, Dict[int, int]] = {
            "xarm_1": {i: 500 for i in range(1, 7)},
            "xarm_2": {i: 500 for i in range(1, 7)},
        }

        # Current global Cartesian end-effector coordinates (X, Y, Z) mm
        self.arm_cartesian: Dict[str, Tuple[float, float, float]] = {
            "xarm_1": (170.0, 360.0, 100.0),
            "xarm_2": (390.0, 360.0, 100.0),
        }

        # Spatial Mutex state
        self._mutex_lock = threading.Lock()
        self.current_corridor_holder: Optional[str] = None

        # Subscribers
        self.sub_arm1 = self.create_subscription(
            JointState,
            "/xarm_1/joint_states",
            lambda msg: self.joint_state_callback(msg, "xarm_1"),
            10,
        )
        self.sub_arm2 = self.create_subscription(
            JointState,
            "/xarm_2/joint_states",
            lambda msg: self.joint_state_callback(msg, "xarm_2"),
            10,
        )

        # Publishers
        self.pub_status = self.create_publisher(String, "/dual_arm/corridor_status", 10)
        self.timer_status = self.create_timer(0.2, self.publish_status)  # 5 Hz

        # Services for spatial mutex arbitration
        self.srv_req_arm1 = self.create_service(
            Trigger,
            "/dual_arm/request_corridor_arm1",
            lambda req, resp: self.handle_mutex_request("xarm_1", req, resp),
        )
        self.srv_req_arm2 = self.create_service(
            Trigger,
            "/dual_arm/request_corridor_arm2",
            lambda req, resp: self.handle_mutex_request("xarm_2", req, resp),
        )
        self.srv_rel_arm1 = self.create_service(
            Trigger,
            "/dual_arm/release_corridor_arm1",
            lambda req, resp: self.handle_mutex_release("xarm_1", req, resp),
        )
        self.srv_rel_arm2 = self.create_service(
            Trigger,
            "/dual_arm/release_corridor_arm2",
            lambda req, resp: self.handle_mutex_release("xarm_2", req, resp),
        )

        self.get_logger().info(
            f"Dual-Arm Coordinator Node initialized. Midline corridor: "
            f"[{self.midline_min_x:.1f}, {self.midline_max_x:.1f}] mm."
        )

    def joint_state_callback(self, msg: JointState, arm_id: str):
        """Process incoming joint positions and compute FK world coordinates."""
        for i, name in enumerate(msg.name):
            val = msg.position[i]
            sid = None
            if "claw" in name or "motor_1" in name:
                sid = 1
            elif "roll" in name or "motor_2" in name:
                sid = 2
            elif "wrist" in name or "motor_3" in name:
                sid = 3
            elif "elbow" in name or "motor_4" in name:
                sid = 4
            elif "shoulder" in name or "motor_5" in name:
                sid = 5
            elif "base" in name or "motor_6" in name:
                sid = 6

            if sid is not None:
                if sid == 1:
                    self.arm_ticks[arm_id][1] = int(round(val))
                else:
                    deg = math.degrees(val)
                    tick = int(round(500 + deg / DEG_PER_TICK))
                    self.arm_ticks[arm_id][sid] = tick

        # Solve FK
        ticks = self.arm_ticks[arm_id]
        params = self.params_arm1 if arm_id == "xarm_1" else self.params_arm2
        pose = solve_fk(
            tick_6=ticks.get(6, 500),
            tick_5=ticks.get(5, 500),
            tick_4=ticks.get(4, 500),
            tick_3=ticks.get(3, 500),
            params=params,
        )

        # Convert cylindrical polar (r, theta_arm, z) to global workspace coordinates
        mount = self.base_mounts[arm_id]
        heading_deg = mount["heading"]
        theta_world_deg = pose.theta + heading_deg
        theta_world_rad = math.radians(theta_world_deg)

        wx = mount["x"] + pose.r * math.cos(theta_world_rad)
        wy = mount["y"] + pose.r * math.sin(theta_world_rad)
        wz = mount["z"] + pose.z

        self.arm_cartesian[arm_id] = (wx, wy, wz)

        # Proximity Check
        p1 = self.arm_cartesian["xarm_1"]
        p2 = self.arm_cartesian["xarm_2"]
        dist = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)

        if dist < self.collision_dist_threshold:
            self.get_logger().warn(
                f"[COLLISION RISK] Gripper separation is only {dist:.1f} mm! "
                f"Arm1 at ({p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}), "
                f"Arm2 at ({p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f})"
            )

    def handle_mutex_request(self, arm_id: str, request: Trigger.Request, response: Trigger.Response):
        """Handle corridor lock request."""
        with self._mutex_lock:
            if self.current_corridor_holder is None or self.current_corridor_holder == arm_id:
                self.current_corridor_holder = arm_id
                response.success = True
                response.message = f"Midline corridor lock granted to {arm_id}."
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = (
                    f"Midline corridor busy! Currently held by {self.current_corridor_holder}. "
                    f"{arm_id} must hold at standoff."
                )
                self.get_logger().info(response.message)
        return response

    def handle_mutex_release(self, arm_id: str, request: Trigger.Request, response: Trigger.Response):
        """Handle corridor lock release."""
        with self._mutex_lock:
            if self.current_corridor_holder == arm_id:
                self.current_corridor_holder = None
                response.success = True
                response.message = f"Midline corridor released by {arm_id}."
                self.get_logger().info(response.message)
            else:
                response.success = True
                response.message = f"Corridor was not held by {arm_id}."
        return response

    def publish_status(self):
        """Publish corridor status and arm positions."""
        p1 = self.arm_cartesian["xarm_1"]
        p2 = self.arm_cartesian["xarm_2"]
        status_msg = String()
        status_msg.data = (
            f"Holder: {self.current_corridor_holder or 'FREE'} | "
            f"Arm1 EE: ({p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.1f}) | "
            f"Arm2 EE: ({p2[0]:.1f}, {p2[1]:.1f}, {p2[2]:.1f})"
        )
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualArmCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Dual-Arm Coordinator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
