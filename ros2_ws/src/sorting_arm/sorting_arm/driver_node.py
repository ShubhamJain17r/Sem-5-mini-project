#!/usr/bin/env python3
"""
Hardware Driver Node for Hiwonder / LewanSoul xArm 1S.

Features:
  - Subscribes to /xarm/joint_commands (sensor_msgs/msg/JointState)
  - Publishes /xarm/joint_states at 10 Hz
  - Exposes /xarm/emergency_stop and /xarm/re_enable services (std_srvs/srv/Trigger)
  - Loads physical deviations from servo_calibration.yaml
  - Enforces physical safety limit clamps and prevents brownout current spikes
"""

import math
import os
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from sorting_arm.kinematics import DEG_PER_TICK, TICKS_PER_DEG
from sorting_arm.xarm_protocol import XArmDriver
from std_srvs.srv import Trigger
import yaml

JOINT_NAME_TO_ID = {
    "claw_gripper": 1,
    "wrist_roll": 2,
    "wrist_pitch": 3,
    "elbow_pitch": 4,
    "shoulder_pitch": 5,
    "base_yaw": 6,
    "motor_1": 1,
    "motor_2": 2,
    "motor_3": 3,
    "motor_4": 4,
    "motor_5": 5,
    "motor_6": 6,
    "joint_1": 1,
    "joint_2": 2,
    "joint_3": 3,
    "joint_4": 4,
    "joint_5": 5,
    "joint_6": 6,
}

STANDARD_JOINT_NAMES = [
    "claw_gripper",
    "wrist_roll",
    "wrist_pitch",
    "elbow_pitch",
    "shoulder_pitch",
    "base_yaw",
]


class XArmDriverNode(Node):
    """ROS 2 Node interfacing with physical xArm 1S over USB serial/HID."""

    def __init__(self):
        """Initialize parameters, communication interfaces, and timer loops."""
        super().__init__("xarm_driver_node")

        # Declare parameters
        self.declare_parameter("port", "auto")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("arm_id", "xarm_1")
        self.declare_parameter("config_path", "config/servo_calibration.yaml")
        self.declare_parameter("default_move_time_ms", 120)
        self.declare_parameter("stagger_ms", 0)
        self.declare_parameter("synchronized", True)
        self.declare_parameter("adaptive_timing", True)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.arm_id = self.get_parameter("arm_id").get_parameter_value().string_value
        self.config_path = self.get_parameter("config_path").get_parameter_value().string_value
        move_param = self.get_parameter("default_move_time_ms")
        self.move_time_ms = move_param.get_parameter_value().integer_value
        self.stagger_ms = self.get_parameter("stagger_ms").get_parameter_value().integer_value
        self.synchronized = self.get_parameter("synchronized").get_parameter_value().bool_value
        adaptive_param = self.get_parameter("adaptive_timing")
        self.adaptive_timing = adaptive_param.get_parameter_value().bool_value

        # Load calibration offsets
        deviations = self._load_deviations(self.config_path, self.arm_id)
        self.get_logger().info(f"Loaded deviations for {self.arm_id}: {deviations}")

        # Initialize hardware driver
        self.driver = XArmDriver(
            port=self.port,
            baudrate=self.baudrate,
            deviations=deviations,
        )

        self.get_logger().info(
            f"Attempting to connect to xArm on {self.port} @ {self.baudrate} baud..."
        )
        connected = self.driver.connect()
        if not connected:
            self.get_logger().error(
                f"Failed to connect to xArm on {self.port}. Driver running in offline mode."
            )

        # Subscribers: Use depth 1 QoS to avoid stale buffered commands
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.command_sub = self.create_subscription(
            JointState,
            "/xarm/joint_commands",
            self.joint_command_callback,
            cmd_qos,
        )

        # Publishers
        self.state_pub = self.create_publisher(JointState, "/xarm/joint_states", 10)
        self.state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Services
        self.estop_srv = self.create_service(
            Trigger,
            "/xarm/emergency_stop",
            self.estop_callback,
        )
        self.re_enable_srv = self.create_service(
            Trigger,
            "/xarm/re_enable",
            self.re_enable_callback,
        )

        self.get_logger().info("xArm Driver Node started successfully.")

    def _load_deviations(self, config_file: str, arm_name: str) -> Dict[int, int]:
        candidates = [
            config_file,
            os.path.join(os.getcwd(), config_file),
            os.path.join("/workspace", config_file),
        ]
        chosen = None
        for c in candidates:
            if os.path.exists(c):
                chosen = c
                break

        if not chosen:
            self.get_logger().warn(
                f"Calibration file not found at {config_file}. Using zero deviations."
            )
            return {i: 0 for i in range(1, 7)}

        try:
            with open(chosen, "r") as f:
                data = yaml.safe_load(f)
                devs = data.get(arm_name, {}).get("servo_deviations", {})
                return {
                    1: int(devs.get("motor_1_claw", 0)),
                    2: int(devs.get("motor_2_wrist_roll", 0)),
                    3: int(devs.get("motor_3_wrist_pitch", 0)),
                    4: int(devs.get("motor_4_elbow", 0)),
                    5: int(devs.get("motor_5_shoulder", 0)),
                    6: int(devs.get("motor_6_base_yaw", 0)),
                }
        except Exception as e:
            self.get_logger().warn(f"Error loading calibration YAML: {e}. Using zero deviations.")
            return {i: 0 for i in range(1, 7)}

    def joint_command_callback(self, msg: JointState):
        """Process incoming JointState message and dispatch serial commands."""
        if self.driver.is_estopped:
            self.get_logger().warn("Command dropped: E-Stop is active.")
            return

        if not self.driver.is_connected:
            if not self.driver.connect():
                return

        target_ticks = {}

        for i, name in enumerate(msg.name):
            val = msg.position[i]
            sid = None

            if name in JOINT_NAME_TO_ID:
                sid = JOINT_NAME_TO_ID[name]
            elif name.isdigit():
                sid = int(name)

            if sid is None or sid < 1 or sid > 6:
                continue

            # Convert radian angle or direct ticks to integer ticks
            if abs(val) <= 2.0 * math.pi and sid != 1:
                deg = math.degrees(val)
                tick = int(round(500 + deg * TICKS_PER_DEG))
            else:
                tick = int(round(val))

            target_ticks[sid] = tick

        if not target_ticks:
            return

        # Deadband filter: identify which servos actually changed target
        # Filtering out servos with 0 tick change prevents continuous hunting/buzzing
        changed_ticks = {}
        for sid, tick in target_ticks.items():
            prev = self.driver.current_ticks.get(sid, 500)
            if abs(tick - prev) >= 1:
                changed_ticks[sid] = tick

        # If nothing changed, do not send redundant packet
        if not changed_ticks:
            return

        # Adaptive duration: calculate smooth execution duration based on max displacement
        if self.adaptive_timing:
            max_delta = max(
                abs(tick - self.driver.current_ticks.get(sid, 500))
                for sid, tick in changed_ticks.items()
            )
            # Minimum 70ms for hardware interpolation, + 1.5ms per tick
            move_time = int(min(1500, max(70, 50 + max_delta * 1.5)))
        else:
            move_time = self.move_time_ms

        stagger = 0 if self.synchronized else self.stagger_ms

        self.driver.send_multi_servo_move(
            changed_ticks,
            time_ms=move_time,
            stagger_ms=stagger,
            synchronized=self.synchronized,
        )

    def publish_joint_states(self):
        """Publish current cached joint states to /xarm/joint_states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.arm_id}_base_link"

        msg.name = STANDARD_JOINT_NAMES
        msg.position = []

        for sid in range(1, 7):
            cur_tick = self.driver.current_ticks.get(sid, 500)
            if sid == 1:
                msg.position.append(float(cur_tick))
            else:
                deg = (cur_tick - 500) * DEG_PER_TICK
                rad = math.radians(deg)
                msg.position.append(rad)

        self.state_pub.publish(msg)

    def estop_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Trigger emergency torque cutoff service."""
        success = self.driver.emergency_stop()
        response.success = success
        response.message = "Emergency Stop triggered: all motor torque unloaded."
        self.get_logger().warn(response.message)
        return response

    def re_enable_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Clear emergency stop and re-enable motor control."""
        self.driver.clear_emergency_stop()
        response.success = True
        response.message = "E-Stop cleared: motor torque re-enabled."
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        """Perform safe teardown and disconnection upon node termination."""
        if self.driver:
            self.driver.emergency_stop()
            self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    """Entry point for xArm hardware driver node."""
    rclpy.init(args=args)
    node = XArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down driver node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
