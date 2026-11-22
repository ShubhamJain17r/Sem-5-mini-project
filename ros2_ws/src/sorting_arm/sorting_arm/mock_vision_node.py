#!/usr/bin/env python3
"""
Synthetic Vision Scenario Generator & Publisher Node.

Simulates the output of vision_node for testing autonomous sorting,
nearest-first queueing, and trajectory execution without physical cameras.
"""

import json
import os
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sorting_arm.kinematics import load_kinematic_parameters, world_to_arm_polar
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MockVisionNode(Node):
    """ROS 2 Node publishing synthetic detected objects on /vision/detected_objects."""

    def __init__(self):
        super().__init__("mock_vision_node")

        self.declare_parameter("arm_id", "xarm_1")
        self.declare_parameter("config_dir", "config")
        self.declare_parameter("scenario", "dual_cube")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("loop", False)

        self.arm_id = self.get_parameter("arm_id").get_parameter_value().string_value
        self.config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        self.scenario = self.get_parameter("scenario").get_parameter_value().string_value
        self.publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.loop = self.get_parameter("loop").get_parameter_value().bool_value

        hardware_yaml = os.path.join(self.config_dir, "xarm_hardware.yaml")
        self.params = load_kinematic_parameters(hardware_yaml, arm_id=self.arm_id)

        self.pub_objects = self.create_publisher(String, "/vision/detected_objects", 10)
        self.srv_trigger = self.create_service(Trigger, "/vision/trigger_scan", self.handle_trigger_scan)

        timer_period = 1.0 / max(0.1, self.publish_rate)
        self.timer = self.create_timer(timer_period, self.publish_scenario)

        self.publish_count = 0
        self.max_burst_count = 10 if not self.loop else -1

        self.get_logger().info(
            f"Mock Vision Node started for {self.arm_id} with scenario '{self.scenario}' (Loop: {self.loop})."
        )

    def _get_scenario_cubes(self) -> List[Dict]:
        """Returns cube definitions in world coordinates for the selected scenario."""
        if self.scenario == "single_red":
            return [
                {"color": "red", "x": 175.0, "y": 175.0, "z": 9.53, "yaw": 0.0},
            ]
        elif self.scenario == "single_green":
            return [
                {"color": "green", "x": 175.0, "y": 175.0, "z": 9.53, "yaw": 0.0},
            ]
        elif self.scenario == "four_cubes":
            # 4 cubes at varying radial distances testing Nearest-First sorting
            return [
                {"color": "green", "x": 175.0, "y": 125.0, "z": 9.53, "yaw": 0.0},  # r = 250mm
                {"color": "red", "x": 175.0, "y": 220.0, "z": 9.53, "yaw": 0.0},    # r = 155mm (Nearest)
                {"color": "green", "x": 130.0, "y": 180.0, "z": 9.53, "yaw": 0.0},  # r = 200mm
                {"color": "red", "x": 220.0, "y": 180.0, "z": 9.53, "yaw": 0.0},    # r = 200mm
            ]
        elif self.scenario == "unreachable":
            # Out of physical reach (> 325mm)
            return [
                {"color": "red", "x": 175.0, "y": 25.0, "z": 9.53, "yaw": 0.0},
            ]
        else:  # "dual_cube" (default)
            return [
                {"color": "red", "x": 130.0, "y": 200.0, "z": 9.53, "yaw": 0.0},
                {"color": "green", "x": 220.0, "y": 200.0, "z": 9.53, "yaw": 0.0},
            ]

    def publish_scenario(self):
        """Constructs and publishes JSON detected_objects packet."""
        if not self.loop and self.publish_count >= self.max_burst_count:
            return

        cubes_def = self._get_scenario_cubes()
        objects_data = []

        for c in cubes_def:
            wx = float(c["x"])
            wy = float(c["y"])
            wz = float(c["z"])
            r, theta, _ = world_to_arm_polar(wx, wy, wz, arm_id=self.arm_id, params=self.params)

            objects_data.append({
                "color": c["color"],
                "world_x_mm": wx,
                "world_y_mm": wy,
                "world_z_mm": wz,
                "yaw_deg": float(c.get("yaw", 0.0)),
                "assigned_arm": self.arm_id,
                "target_r_mm": round(r, 2),
                "target_theta_deg": round(theta, 2),
                "targetable": True,
                "confidence": 0.98,
            })

        msg_str = String()
        msg_str.data = json.dumps({
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "total_detected": len(objects_data),
            "homography_locked": True,
            "objects": objects_data,
        })

        self.pub_objects.publish(msg_str)
        self.publish_count += 1

    def handle_trigger_scan(self, request, response):
        """Forces immediate re-burst of scenario targets."""
        self.publish_count = 0
        self.publish_scenario()
        response.success = True
        response.message = f"Scenario '{self.scenario}' published."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
