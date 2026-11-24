#!/usr/bin/env python3
"""
ROS 2 Perception Node for Overhead Boom Camera & ArUco Metric Warping.

Features:
  - Captures V4L2 frames from camera (or subscribes to /camera/image_raw).
  - Warps arena to metric grid (2.0 px/mm) using 4-point ArUco planar homography.
  - Detects colored cubes (Red, Green, Blue, Yellow) and converts to arena world coordinates (Xw, Yw).
  - Computes arm polar shifts (r, theta) for both xArm 1 and xArm 2.
  - Publishes detected objects as a structured JSON string to /vision/detected_objects.
  - Publishes annotated telemetry feed to /vision/annotated_image (sensor_msgs/msg/Image).
  - Exposes /vision/trigger_scan service (std_srvs/srv/Trigger) for on-demand stationary scanning.
"""

import json
import os
import sys
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Import perception engine
from vision.arena_vision_pipeline import ArenaVisionPipeline, DetectedCube


class VisionNode(Node):
    """ROS 2 Node publishing arena cube telemetry and annotated visualization."""

    def __init__(self):
        super().__init__("vision_node")

        # Declare parameters
        self.declare_parameter("config_dir", "config")
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("scale_px_per_mm", 2.0)
        self.declare_parameter("use_v4l2_capture", True)

        self.config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        self.camera_device = self.get_parameter("camera_device").get_parameter_value().string_value
        self.publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.scale_px_per_mm = self.get_parameter("scale_px_per_mm").get_parameter_value().double_value
        self.use_v4l2 = self.get_parameter("use_v4l2_capture").get_parameter_value().bool_value

        # Initialize perception engine
        self.pipeline = ArenaVisionPipeline(config_dir=self.config_dir, scale_px_per_mm=self.scale_px_per_mm)

        # Publishers
        qos_telemetry = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub_objects = self.create_publisher(String, "/vision/detected_objects", qos_telemetry)
        self.pub_image = self.create_publisher(Image, "/vision/annotated_image", 5)

        # Service for stationary on-demand scan
        self.srv_scan = self.create_service(Trigger, "/vision/trigger_scan", self.handle_trigger_scan)

        # Setup Camera
        self.cap: Optional[cv2.VideoCapture] = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_cubes: List[DetectedCube] = []

        if self.use_v4l2:
            self._init_camera()

        # Timer loop for continuous telemetry
        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.capture_and_process)

        self.get_logger().info("Vision Node initialized. Publishing on /vision/detected_objects.")

    def _init_camera(self):
        dev = int(self.camera_device) if self.camera_device.isdigit() else self.camera_device
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2 if isinstance(dev, str) else cv2.CAP_ANY)
        if not self.cap.isOpened():
            self.get_logger().warn(f"Could not open camera {self.camera_device}. Awaiting external frames or service.")
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.get_logger().info(f"Connected to camera {self.camera_device} @ 1920x1080.")

    def capture_and_process(self):
        """Captures frame from webcam, processes perception pipeline, and publishes topics."""
        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                return
            self.latest_frame = frame
        elif self.latest_frame is None:
            return

        warped, annotated, cubes = self.pipeline.process_frame(self.latest_frame, draw_grid=True)
        self.latest_cubes = cubes

        # 1. Publish JSON telemetry on /vision/detected_objects
        objects_data = []
        for c in cubes:
            objects_data.append({
                "color": c.color,
                "world_x_mm": c.world_x,
                "world_y_mm": c.world_y,
                "world_z_mm": c.world_z,
                "yaw_deg": c.yaw_deg,
                "assigned_arm": c.assigned_arm,
                "target_r_mm": c.target_r_mm,
                "target_theta_deg": c.target_theta_deg,
                "requires_mutex": c.requires_mutex,
                "r1_mm": c.r1_mm,
                "theta1_deg": c.theta1_deg,
                "r2_mm": c.r2_mm,
                "theta2_deg": c.theta2_deg,
                "confidence": c.confidence,
                "gripper_roll_tick": c.gripper_roll_tick,
            })

        msg_str = String()
        msg_str.data = json.dumps({
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "total_detected": len(objects_data),
            "homography_locked": self.pipeline.H_locked,
            "objects": objects_data,
        })
        self.pub_objects.publish(msg_str)

        # 2. Publish annotated image
        if annotated is not None and self.pub_image.get_subscription_count() > 0:
            img_msg = self._cv2_to_image_msg(annotated)
            if img_msg:
                self.pub_image.publish(img_msg)

    def _cv2_to_image_msg(self, cv_img: np.ndarray) -> Optional[Image]:
        """Converts OpenCV BGR image to sensor_msgs/Image without cv_bridge dependency."""
        try:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "arena_camera"
            msg.height = cv_img.shape[0]
            msg.width = cv_img.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = cv_img.shape[1] * 3
            msg.data = cv_img.tobytes()
            return msg
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return None

    def handle_trigger_scan(self, request, response):
        """Service callback to force a stationary scan and return detected cube count."""
        self.capture_and_process()
        response.success = True
        response.message = f"Arena scanned: {len(self.latest_cubes)} cubes detected."
        return response

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
