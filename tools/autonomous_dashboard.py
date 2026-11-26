#!/usr/bin/env python3
"""
xArm 1S Autonomous System Web Dashboard & Digital Twin Synchronizer.

Provides an interactive browser interface for the 4-stage autonomous sorting pipeline:
  - Background Hardware Watchdog (Camera & xArm serial/HID probing, permissions check)
  - Non-Permanent / Hand-Held Camera Assist HUD (ArUco lock indicators, height & tilt guidance)
  - "Snap & Lock" stationary frame buffer (freeze frame once aligned, put camera down)
  - Golden Zone Perception & Dynamic Wrist Roll orientation analysis
  - Digital Twin Synchronization (mirrors detected physical cubes into RViz2 & 3D render)
  - Dual-mode ROS 2 / Standalone execution with Nearest-First pick-and-place sequencing

Usage:
  python3 tools/autonomous_dashboard.py [--port 8080] [--camera 0] [--dry-run]
"""

import argparse
import glob
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple
import urllib.parse

# Ensure project root and ROS 2 package are importable
script_dir = os.path.dirname(os.path.realpath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, ".."))
ros_pkg_dir = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, ros_pkg_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
import yaml

from sorting_arm.kinematics import (
    DEFAULT_PARAMS,
    DEG_PER_TICK,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices
from tools.render_sim_view import MultiViewRenderer
from vision.align_camera import CameraAligner
from vision.arena_vision_pipeline import ArenaVisionPipeline, DetectedCube

# Optional ROS 2 integration
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    HAS_ROS2 = True
except ImportError:
    rclpy = None
    HAS_ROS2 = False


def build_multiframe_consensus_queue(
    per_frame_cubes: List[List[DetectedCube]],
    params: KinematicParameters,
    cluster_radius_mm: float = 25.0,
) -> List[Dict[str, Any]]:
    """
    Aggregates cube detections across multiple captured frames using spatial clustering
    and median coordinate estimation to reject handheld shake and outlier detections.
    """
    num_frames = len(per_frame_cubes)
    if num_frames == 0:
        return []

    clusters: List[Dict[str, Any]] = []
    for frame_idx, cubes in enumerate(per_frame_cubes):
        for c in cubes:
            matched = None
            best_dist = cluster_radius_mm
            for cl in clusters:
                if cl["color"] != c.color or frame_idx in cl["frame_ids"]:
                    continue
                dist = math.hypot(c.world_x - cl["mean_x"], c.world_y - cl["mean_y"])
                if dist <= best_dist:
                    best_dist = dist
                    matched = cl
            if matched is not None:
                matched["items"].append(c)
                matched["frame_ids"].add(frame_idx)
                n = len(matched["items"])
                matched["mean_x"] = sum(item.world_x for item in matched["items"]) / n
                matched["mean_y"] = sum(item.world_y for item in matched["items"]) / n
            else:
                clusters.append({
                    "color": c.color,
                    "items": [c],
                    "frame_ids": {frame_idx},
                    "mean_x": float(c.world_x),
                    "mean_y": float(c.world_y),
                })

    min_obs = 1 if num_frames == 1 else math.ceil(num_frames / 2.0)
    queue: List[Dict[str, Any]] = []
    for cl in clusters:
        items = cl["items"]
        if len(items) < min_obs:
            continue

        xs = [float(item.world_x) for item in items]
        ys = [float(item.world_y) for item in items]
        zs = [float(item.world_z) for item in items]
        med_x = float(np.median(xs))
        med_y = float(np.median(ys))
        med_z = float(np.median(zs))
        std_mm = float(math.sqrt(float(np.var(xs)) + float(np.var(ys))))

        rolls = [int(item.gripper_roll_tick) for item in items]
        roll_tick = max(set(rolls), key=rolls.count)
        avg_aspect = float(sum(float(getattr(item, "aspect_ratio", 1.0)) for item in items) / len(items))

        r, th_arm, z_rel = world_to_arm_polar(med_x, med_y, med_z, arm_id="xarm_1", params=params)
        queue.append({
            "color": cl["color"],
            "world_x": med_x,
            "world_y": med_y,
            "world_z": med_z,
            "z_rel": float(z_rel),
            "reach_r": float(r),
            "theta_arm": float(th_arm),
            "roll_tick": roll_tick,
            "aspect_ratio": avg_aspect,
            "observations": len(items),
            "total_frames": num_frames,
            "std_mm": round(std_mm, 2),
            "dest_container": "red" if cl["color"] == "red" else "green",
        })

    queue.sort(key=lambda item: item["reach_r"])
    return queue


class LogManager:
    """Thread-safe circular log buffer for live console streaming."""

    def __init__(self, max_entries: int = 150):
        self.max_entries = max_entries
        self.entries: List[Dict[str, str]] = []
        self.lock = threading.Lock()

    def log(self, level: str, msg: str):
        with self.lock:
            entry = {
                "time": time.strftime("%H:%M:%S"),
                "level": level.upper(),
                "msg": msg,
            }
            self.entries.append(entry)
            if len(self.entries) > self.max_entries:
                self.entries.pop(0)
        # Also print to stdout
        prefix = f"[{entry['time']}] [{entry['level']}]"
        print(f"{prefix} {msg}")

    def get_logs(self) -> List[Dict[str, str]]:
        with self.lock:
            return list(self.entries)

    def clear(self):
        with self.lock:
            self.entries.clear()


def get_available_cameras() -> List[Dict[str, Any]]:
    """
    Scans host V4L2 video nodes and enumerates active video capture streams.
    Filters out metadata nodes (stream index 1) and labels internal vs external cameras.
    """
    cameras = []
    sys_nodes = sorted(
        glob.glob("/sys/class/video4linux/video*"),
        key=lambda p: int(os.path.basename(p).replace("video", "")) if os.path.basename(p).replace("video", "").isdigit() else 999
    )
    if sys_nodes:
        for node in sys_nodes:
            bname = os.path.basename(node)
            try:
                idx = int(bname.replace("video", ""))
            except ValueError:
                continue
            name_file = os.path.join(node, "name")
            index_file = os.path.join(node, "index")
            name = "USB Camera"
            stream_idx = 0
            if os.path.exists(name_file):
                try:
                    with open(name_file, "r") as f:
                        name = f.read().strip()
                except Exception:
                    pass
            if os.path.exists(index_file):
                try:
                    with open(index_file, "r") as f:
                        stream_idx = int(f.read().strip())
                except Exception:
                    pass

            # Only stream index 0 is valid video capture; stream index 1 is metadata
            if stream_idx == 0:
                dev_path = f"/dev/{bname}"
                is_integ = any(w in name.lower() for w in ["integrated", "internal", "built-in", "facetime", "webcam"])
                is_acc = os.access(dev_path, os.R_OK | os.W_OK) if os.path.exists(dev_path) else False
                label = f"Camera {idx}: {name} ({dev_path})"
                if is_integ:
                    label += " [Laptop Webcam]"
                else:
                    label += " [Overhead / USB Camera]"
                cameras.append({
                    "index": idx,
                    "device": dev_path,
                    "name": name,
                    "label": label,
                    "is_integrated": is_integ,
                    "accessible": is_acc,
                })

    if not cameras:
        for i in range(4):
            dev = f"/dev/video{i}"
            if os.path.exists(dev):
                cameras.append({
                    "index": i,
                    "device": dev,
                    "name": f"Video Device {i}",
                    "label": f"Camera {i} ({dev})",
                    "is_integrated": False,
                    "accessible": os.access(dev, os.R_OK | os.W_OK),
                })
    return cameras


class AutonomousSystemManager:
    """
    Central controller managing camera capture, perception, digital twin,
    hardware execution, and ROS 2 bridge.
    """

    def __init__(self, camera_idx: int = 0, port: str = "auto", dry_run: bool = False):
        self.camera_idx = camera_idx
        self.requested_port = port
        self.dry_run = dry_run
        self.logger = LogManager()

        # Hardware handles
        self.cap: Optional[cv2.VideoCapture] = None
        self.driver: Optional[XArmDriver] = None
        self.params: KinematicParameters = load_kinematic_parameters()
        self.pipeline = ArenaVisionPipeline(config_dir=os.path.join(repo_root, "config"))
        self.aligner = CameraAligner(config_dir=os.path.join(repo_root, "config"))
        self.sim_renderer = MultiViewRenderer(output_dir=os.path.join(repo_root, "artifacts"))

        # Hardware & Connection State
        self.camera_connected = False
        self.camera_perm_ok = True
        self.xarm_connected = False
        self.xarm_port: Optional[str] = None
        self.xarm_type: Optional[str] = None
        self.xarm_perm_ok = False

        # Frame & Multi-Snapshot Consensus State
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_hud_frame: Optional[np.ndarray] = None
        self.latest_warped: Optional[np.ndarray] = None
        self.latest_sim: Optional[np.ndarray] = None
        self.is_locked = False
        self.locked_frame: Optional[np.ndarray] = None
        self.captured_frames: List[Dict[str, Any]] = []
        self.target_capture_count: int = 4
        self.alignment_quality_pct: int = 0
        self.align_prompts: List[str] = []
        self.frame_lock = threading.Lock()

        # Dynamic cube size & claw open/clamp ticks from config/xarm_hardware.yaml
        self.open_claw_tick = int((self.params.min_safe_ticks or {}).get(1, 250))
        self.clamp_claw_tick, self.cube_center_z_mm = self._load_cube_grasp_config(
            os.path.join(repo_root, "config", "xarm_hardware.yaml")
        )

        # Optical Guidance & Extrinsics
        self.aruco_reports: Dict[int, Any] = {}
        self.all_markers_locked = False
        self.camera_height_mm: Optional[float] = None
        self.depression_angle_deg: Optional[float] = None
        self.distance_to_center_mm: Optional[float] = None

        # Perception & Scheduling
        self.detected_cubes: List[DetectedCube] = []
        self.target_queue: List[Dict[str, Any]] = []
        self.sim_cubes: List[Dict[str, Any]] = []
        self.active_target: Optional[Dict[str, Any]] = None
        self.trajectory_step = "idle"
        self.is_executing = False

        # Current joint ticks
        self.current_joint_ticks = {i: 500 for i in range(1, 7)}
        self.current_joint_ticks[1] = 250  # Open claw

        # ROS 2 Publisher Node
        self.running = True
        self.ros_node: Optional[Any] = None
        self.ros_pub_objects: Optional[Any] = None
        self.ros_pub_joints: Optional[Any] = None
        self.init_ros2()

        # Start background watchdog & video loop
        self.watchdog_thread = threading.Thread(target=self._background_watchdog_loop, daemon=True)
        self.video_thread = threading.Thread(target=self._background_video_loop, daemon=True)
        self.watchdog_thread.start()
        self.video_thread.start()

        self.logger.log("STAGE", "Autonomous System Manager Initialized.")

    def init_ros2(self):
        """Initializes ROS 2 background node if rclpy is available."""
        global HAS_ROS2
        if not HAS_ROS2 or rclpy is None:
            self.logger.log("INFO", "ROS 2 middleware not present; running standalone engine.")
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.ros_node = Node("autonomous_dashboard_bridge")
            self.ros_pub_objects = self.ros_node.create_publisher(String, "/vision/detected_objects", 10)
            self.ros_pub_joints = self.ros_node.create_publisher(JointState, "/xarm_1/joint_states", 10)

            # Spin & publish joint states at 20 Hz for RViz2 digital twin sync
            def ros_spin():
                last_pub = 0.0
                while self.running and rclpy.ok():
                    try:
                        rclpy.spin_once(self.ros_node, timeout_sec=0.04)
                        now = time.time()
                        if now - last_pub >= 0.05:
                            self.publish_joint_states()
                            last_pub = now
                    except Exception:
                        break

            threading.Thread(target=ros_spin, daemon=True).start()
            self.logger.log("SUCCESS", "ROS 2 Bridge initialized: Publishing /vision/detected_objects & /xarm_1/joint_states.")
        except Exception as e:
            self.logger.log("WARN", f"ROS 2 Bridge init skipped: {e}")

    def publish_joint_states(self):
        """Publishes current joint ticks as ROS 2 JointState for RViz2 URDF & ArenaVisualizer."""
        if self.ros_pub_joints is None or self.ros_node is None or rclpy is None or not rclpy.ok():
            return
        try:
            centers = self.params.nominal_center_ticks or {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
            names = ["claw_gripper", "wrist_roll", "wrist_pitch", "elbow_pitch", "shoulder_pitch", "base_yaw"]
            msg = JointState()
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.header.frame_id = "xarm_1_base_link"
            msg.name = names
            msg.position = [
                math.radians((self.current_joint_ticks.get(i + 1, centers.get(i + 1, 500)) - centers.get(i + 1, 500)) * DEG_PER_TICK)
                for i in range(6)
            ]
            self.ros_pub_joints.publish(msg)
        except Exception:
            pass

    def _update_sim_view(self):
        """Renders the 3D Digital Twin frame from current joint ticks and sim_cubes."""
        try:
            cubes_for_render = self.sim_cubes if self.sim_cubes else [
                {"x": c.world_x, "y": c.world_y, "z": c.world_z, "color": c.color}
                for c in self.detected_cubes
            ]
            status_lbl = f"SIMULATION | STEP: {self.trajectory_step.upper()}"
            if self.is_locked and not self.is_executing:
                status_lbl = f"LOCKED ({len(self.captured_frames)}/{self.target_capture_count}f) | READY TO SORT"
            sim_img = self.sim_renderer.render_isometric_view(
                joint_ticks=self.current_joint_ticks,
                cubes=cubes_for_render,
                status_text=status_lbl,
            )
            with self.frame_lock:
                self.latest_sim = sim_img
        except Exception as e:
            self.logger.log("WARN", f"Sim render error: {e}")

    # -------------------------------------------------------------------------
    # Background Hardware Watchdog Loop
    # -------------------------------------------------------------------------
    def _background_watchdog_loop(self):
        """Continuously checks device nodes and permissions every 2 seconds."""
        while self.running:
            try:
                # 1. Probe Video Nodes
                dev_path = f"/dev/video{self.camera_idx}"
                if os.path.exists(dev_path):
                    self.camera_perm_ok = os.access(dev_path, os.R_OK | os.W_OK)
                else:
                    self.camera_perm_ok = False

                # 2. Probe physical xArm connection
                devices = find_all_xarm_devices()
                if devices:
                    self.xarm_port = devices[0][0]
                    self.xarm_type = devices[0][1]
                    self.xarm_connected = True
                    self.xarm_perm_ok = os.access(self.xarm_port, os.R_OK | os.W_OK)
                else:
                    self.xarm_port = None
                    self.xarm_type = None
                    self.xarm_connected = False
                    self.xarm_perm_ok = False
            except Exception as e:
                self.logger.log("WARN", f"Watchdog error: {e}")

            time.sleep(2.0)

    # -------------------------------------------------------------------------
    # Background Video & Optical Guidance Loop
    # -------------------------------------------------------------------------
    def _background_video_loop(self):
        """Continuously grabs frames, runs optical assist & ArUco checks, and updates 3D simulation."""
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                if not self.is_locked:
                    self._open_camera()
                self._update_sim_view()
                time.sleep(0.25)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.camera_connected = False
                self._update_sim_view()
                time.sleep(0.1)
                continue

            self.camera_connected = True

            with self.frame_lock:
                self.latest_frame = frame
                process_frame = self.locked_frame if self.is_locked and self.locked_frame is not None else frame

            # Run live CameraAligner HUD (identical to vision/align_camera.py) + ArUco checks
            try:
                align_centers = self.aligner.detect_markers(process_frame)
                align_metrics = self.aligner.compute_metrics(process_frame.shape, align_centers)
                hud_img = self.aligner.render_hud(process_frame, align_metrics, align_centers)

                marker_status = self.pipeline.verify_aruco_markers(process_frame)
                self.aruco_reports = marker_status.get("reports", {})
                self.all_markers_locked = bool(align_metrics.get("detected", False)) or marker_status.get("all_detected", False)
                self.align_prompts = list(align_metrics.get("prompts", []))

                if align_metrics.get("detected"):
                    if align_metrics.get("is_aligned"):
                        self.alignment_quality_pct = 98
                    elif align_metrics.get("is_close"):
                        self.alignment_quality_pct = 93
                    else:
                        d_err = math.hypot(align_metrics.get("delta_x_mm", 0.0), align_metrics.get("delta_y_mm", 0.0))
                        self.alignment_quality_pct = max(65, min(91, int(round(92 - d_err * 0.4))))

                    extrinsics = self.pipeline.compute_camera_extrinsics(process_frame)
                    if extrinsics.get("success"):
                        self.camera_height_mm = float(extrinsics.get("height_mm", 0.0))
                        self.depression_angle_deg = float(extrinsics.get("depression_angle_deg", 0.0))
                        self.distance_to_center_mm = float(extrinsics.get("distance_to_arena_center_mm", 0.0))
                else:
                    det_cnt = len(align_centers)
                    self.alignment_quality_pct = det_cnt * 18

                with self.frame_lock:
                    self.latest_hud_frame = hud_img

                # Generate Rectified Warped Frame with Golden Zone HUD
                warped, annotated_warped, cubes = self.pipeline.process_frame(process_frame, draw_grid=True)
                if annotated_warped is not None:
                    overlay = annotated_warped.copy()
                    if self.is_locked:
                        cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 40), (16, 185, 129), -1)
                        cv2.putText(
                            overlay,
                            f"LOCKED ({len(self.captured_frames)}/{self.target_capture_count}f) - READY",
                            (20, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA
                        )
                    with self.frame_lock:
                        self.latest_warped = overlay
                        if self.is_locked and not self.sim_cubes:
                            self.detected_cubes = cubes
            except Exception:
                pass

            self._update_sim_view()
            time.sleep(0.04)  # ~25 FPS

    def _open_camera(self):
        """Attempts opening the selected camera device."""
        try:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

            dev_path = f"/dev/video{self.camera_idx}"
            if not os.path.exists(dev_path):
                self.camera_connected = False
                self.logger.log("WARN", f"Device {dev_path} not found on host.")
                return

            self.cap = cv2.VideoCapture(self.camera_idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                # Fallback to default backend
                self.cap = cv2.VideoCapture(self.camera_idx)

            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                for _ in range(3):
                    self.cap.read()
                self.camera_connected = True
                self.logger.log("SUCCESS", f"Camera active on {dev_path}")
            else:
                self.camera_connected = False
                self.logger.log("WARN", f"Failed to capture frames from {dev_path}")
        except Exception as e:
            self.camera_connected = False
            self.logger.log("WARN", f"Failed to open /dev/video{self.camera_idx}: {e}")

    # -------------------------------------------------------------------------
    # Actions & Command Handlers
    # -------------------------------------------------------------------------
    def switch_camera(self, new_idx: int) -> bool:
        """Dynamically switches active camera device index."""
        with self.frame_lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
            self.camera_idx = new_idx
            self.is_locked = False
            self.locked_frame = None
            self.latest_frame = None
            self.latest_warped = None
            self.camera_connected = False

        self._open_camera()
        self.logger.log("INFO", f"Camera switched to index {new_idx} (/dev/video{new_idx})")
        return self.camera_connected

    @staticmethod
    def _load_cube_grasp_config(hw_yaml: str) -> Tuple[int, float]:
        """Loads full-close claw clamp tick and cube center Z from xarm_hardware.yaml."""
        clamp_tick = 750  # Full close so objects are gripped firmly without slipping
        center_z = 22.5   # Default for 45.0 mm tall cube
        try:
            with open(hw_yaml, "r") as f:
                data = yaml.safe_load(f) or {}
            arena_cfg = data.get("workspace", {}).get("picking_arena", {})
            center_z = float(arena_cfg.get("cube_center_z_mm", 22.5))
            m1_cfg = (
                data.get("manipulators", {})
                .get("xarm_1", {})
                .get("motors", {})
                .get("motor_1", {})
            )
            span_map = m1_cfg.get("claw_span_mm", {})
            if "tick" in span_map.get("fully_closed_empty", {}):
                clamp_tick = int(span_map["fully_closed_empty"]["tick"])
            elif "max_tick" in m1_cfg:
                clamp_tick = int(m1_cfg["max_tick"])
        except Exception:
            pass
        return clamp_tick, center_z

    def snap_and_lock(self):
        """Manually captures a frame into the multi-frame buffer and updates consensus (never auto-snaps)."""
        with self.frame_lock:
            if self.latest_frame is None:
                self.logger.log("WARN", "No camera frame available to capture.")
                return False
            frame = self.latest_frame.copy()

        marker_status = self.pipeline.verify_aruco_markers(frame)
        all_ok = bool(marker_status.get("all_detected", False))
        if all_ok:
            self.pipeline.compute_camera_extrinsics(frame)
        warped_cap, ann_cap, cubes = self.pipeline.process_frame(frame, draw_grid=True)

        # Save raw + annotated capture to data/captured_frames/
        cap_dir = os.path.join(repo_root, "data", "captured_frames")
        os.makedirs(cap_dir, exist_ok=True)

        with self.frame_lock:
            self.captured_frames.append({
                "frame": frame,
                "warped": warped_cap,
                "cubes": cubes,
                "all_markers": all_ok,
                "timestamp": time.time(),
            })
            self.locked_frame = frame
            cap_idx = len(self.captured_frames)
            if cap_idx >= self.target_capture_count:
                self.is_locked = True

        try:
            cv2.imwrite(os.path.join(cap_dir, f"capture_{cap_idx}_raw.png"), frame)
            if ann_cap is not None:
                cv2.imwrite(os.path.join(cap_dir, f"capture_{cap_idx}_warped.png"), ann_cap)
        except Exception as e:
            self.logger.log("WARN", f"Could not save captured image #{cap_idx}: {e}")

        self.logger.log(
            "STAGE",
            f"Captured & saved frame {cap_idx}/{self.target_capture_count} "
            f"(Markers: {'4/4' if all_ok else 'Partial'}, Raw cubes: {len(cubes)})."
        )
        self.compute_optimal_consensus()
        self.sync_digital_twin()
        if self.is_locked:
            self.logger.log("SUCCESS", "All 4 frames captured — switched feed to 3D Simulation! You may now remove the camera.")
        return True

    def load_saved_captures(self, cap_dir: Optional[str] = None) -> bool:
        """Loads previously saved capture_*_raw.png images and runs consensus pipeline without needing the camera."""
        if cap_dir is None:
            cap_dir = os.path.join(repo_root, "data", "captured_frames")

        raw_files = sorted(glob.glob(os.path.join(cap_dir, "capture_*_raw.png")))
        if not raw_files:
            self.logger.log("WARN", f"No saved capture_*_raw.png files found in {cap_dir}.")
            return False

        # Optional cached homography fallback (e.g., if markers had HUD overlay on saved frame)
        h_json = os.path.join(cap_dir, "homography.json")
        saved_h = None
        if os.path.exists(h_json):
            try:
                with open(h_json, "r") as f:
                    h_data = json.load(f)
                mat = h_data.get("H_matrix")
                if mat:
                    saved_h = np.array(mat, dtype=np.float32).reshape((3, 3))
            except Exception as e:
                self.logger.log("WARN", f"Could not load saved homography.json: {e}")

        loaded_entries = []
        last_warped_ann = None
        last_frame = None
        for idx, fpath in enumerate(raw_files, start=1):
            frame = cv2.imread(fpath)
            if frame is None:
                continue
            marker_status = self.pipeline.verify_aruco_markers(frame)
            all_ok = bool(marker_status.get("all_detected", False))
            if not all_ok and saved_h is not None:
                self.pipeline.H_matrix = saved_h
                self.pipeline.lock_homography(True)
            else:
                self.pipeline.lock_homography(False)
                self.pipeline.compute_camera_extrinsics(frame)

            warped_cap, ann_cap, cubes = self.pipeline.process_frame(frame, draw_grid=True)
            loaded_entries.append({
                "frame": frame,
                "warped": warped_cap,
                "cubes": cubes,
                "all_markers": all_ok or (saved_h is not None),
                "timestamp": time.time(),
            })
            last_frame = frame
            if ann_cap is not None:
                last_warped_ann = ann_cap
            self.logger.log(
                "INFO",
                f"Loaded {os.path.basename(fpath)} ({idx}/{len(raw_files)}): detected {len(cubes)} cube(s)."
            )

        self.pipeline.lock_homography(False)
        if not loaded_entries:
            self.logger.log("WARN", "Failed to decode any saved capture images.")
            return False

        with self.frame_lock:
            self.captured_frames = loaded_entries
            self.locked_frame = last_frame
            self.latest_frame = last_frame
            if last_warped_ann is not None:
                self.latest_warped = last_warped_ann
            self.is_locked = True
            self.all_markers_locked = True
            self.alignment_quality_pct = 95

        self.compute_optimal_consensus()
        self.sync_digital_twin()
        self.logger.log("SUCCESS", f"Loaded {len(loaded_entries)} saved frame(s) — ready to sort without camera!")
        return True

    def lock_and_simulate(self):
        """Locks current captured consensus (e.g. after 3 frames) and switches feed to 3D Simulation."""
        with self.frame_lock:
            if not self.captured_frames:
                self.logger.log("WARN", "Capture at least 1 frame (Spacebar) before locking.")
                return False
            self.is_locked = True
        self.compute_optimal_consensus()
        self.sync_digital_twin()
        self.logger.log("SUCCESS", f"Locked {len(self.captured_frames)} captured frame(s) — Simulation active!")
        return True

    @staticmethod
    def _is_in_pickup_zone(c: DetectedCube) -> bool:
        """Validates cube is inside the physical arena and reachable by xArm 1S."""
        return (
            25.0 <= c.world_x <= 325.0
            and 25.0 <= c.world_y <= 275.0
            and 110.0 <= c.r_mm <= 325.0
        )

    def compute_optimal_consensus(self):
        """Aggregates all captured frames using spatial clustering and median error correction."""
        with self.frame_lock:
            entries = list(self.captured_frames)

        if not entries:
            self.scan_arena_targets()
            return

        # Prefer frames where all 4 ArUco markers locked; fall back to all if none had 4/4
        valid_entries = [e for e in entries if e["all_markers"]] or entries
        per_frame_cubes = []
        for entry in valid_entries:
            valid_cubes = [
                c for c in entry["cubes"]
                if c.color in ("red", "green") and self._is_in_pickup_zone(c)
            ]
            per_frame_cubes.append(valid_cubes)

        queue = build_multiframe_consensus_queue(per_frame_cubes, self.params)
        self.target_queue = queue
        self.sim_cubes = [
            {
                "id": i,
                "x": t["world_x"],
                "y": t["world_y"],
                "z": t["world_z"],
                "color": t["color"],
                "state": "in_cell",
            }
            for i, t in enumerate(queue)
        ]
        self._update_sim_view()

        if queue:
            self.logger.log(
                "SUCCESS",
                f"Optimal consensus computed from {len(valid_entries)}/{len(entries)} frame(s): "
                f"{len(queue)} cube(s) locked."
            )
            for idx, t in enumerate(queue):
                self.logger.log(
                    "INFO",
                    f"  Target #{idx+1}: {t['color'].upper()} @ "
                    f"(X={t['world_x']:.1f}, Y={t['world_y']:.1f}, Z={t['world_z']:.1f}) -> "
                    f"r={t['reach_r']:.1f}mm ({t['observations']}/{len(valid_entries)} frames, "
                    f"σ={t['std_mm']:.1f}mm)"
                )
        else:
            self.logger.log("WARN", f"No Red/Green cubes detected across {len(entries)} captured frame(s).")

    def unlock_camera(self):
        """Clears captured multi-frame buffer and resumes live camera feed."""
        with self.frame_lock:
            self.is_locked = False
            self.locked_frame = None
            self.captured_frames.clear()
            self.target_queue = []
            self.sim_cubes = []
            self.detected_cubes = []
        self.logger.log("INFO", "Captures cleared; resumed live camera feed.")

    def scan_arena_targets(self):
        """Runs perception on current locked or live frame and builds Nearest-First queue."""
        with self.frame_lock:
            frame = self.locked_frame if (self.is_locked and self.locked_frame is not None) else self.latest_frame

        if frame is None:
            self.logger.log("WARN", "Cannot scan: No frame available.")
            return

        warped, annotated, cubes = self.pipeline.process_frame(frame, draw_grid=True)
        self.detected_cubes = cubes

        # Filter: Reachable Arena & Red/Green targets
        queue = []
        for c in cubes:
            if c.color not in ("red", "green"):
                continue
            if not self._is_in_pickup_zone(c):
                continue

            r, th_arm, _ = world_to_arm_polar(c.world_x, c.world_y, c.world_z, arm_id="xarm_1", params=self.params)
            queue.append({
                "color": c.color,
                "world_x": float(c.world_x),
                "world_y": float(c.world_y),
                "world_z": float(c.world_z),
                "reach_r": float(r),
                "theta_arm": float(th_arm),
                "roll_tick": int(c.gripper_roll_tick),
                "aspect_ratio": float(getattr(c, "aspect_ratio", 1.0)),
                "dest_container": "red" if c.color == "red" else "green",
            })

        # Nearest-First policy
        queue.sort(key=lambda item: item["reach_r"])
        self.target_queue = queue
        self.sim_cubes = [
            {
                "id": i,
                "x": t["world_x"],
                "y": t["world_y"],
                "z": t["world_z"],
                "color": t["color"],
                "state": "in_cell",
            }
            for i, t in enumerate(queue)
        ]
        self._update_sim_view()

        self.logger.log("SUCCESS", f"Scanned arena: {len(queue)} valid targets in Golden Zone.")
        for idx, t in enumerate(queue):
            roll_note = " (90° Roll)" if t["roll_tick"] == 130 else ""
            self.logger.log("INFO", f"  Target #{idx+1}: {t['color'].upper()} @ (X={t['world_x']:.1f}, Y={t['world_y']:.1f}) -> r={t['reach_r']:.1f}mm{roll_note}")

    def sync_digital_twin(self):
        """Mirrors detected physical cubes into RViz simulation and updates digital twin."""
        if not self.target_queue:
            self.logger.log("WARN", "No targets in queue to mirror into digital twin.")
            return

        # 1. Publish to ROS 2 if available
        if self.ros_pub_objects is not None and rclpy is not None:
            try:
                objects_payload = []
                for t in self.target_queue:
                    objects_payload.append({
                        "color": t["color"],
                        "world_x_mm": t["world_x"],
                        "world_y_mm": t["world_y"],
                        "world_z_mm": t["world_z"],
                        "target_r_mm": t["reach_r"],
                        "target_theta_deg": t["theta_arm"],
                        "gripper_roll_tick": t["roll_tick"],
                        "targetable": True,
                    })

                msg = String()
                msg.data = json.dumps({
                    "source": "dashboard",
                    "sync_replace": True,
                    "objects": objects_payload,
                })
                self.ros_pub_objects.publish(msg)
                self.logger.log("SUCCESS", f"Mirrored {len(self.target_queue)} cubes into ROS 2 Digital Twin (/vision/detected_objects).")
            except Exception as e:
                self.logger.log("WARN", f"ROS 2 sync error: {e}")

        # 2. Update multi-view renderer snapshot
        try:
            cubes_sim = [{"x": t["world_x"], "y": t["world_y"], "z": t["world_z"], "color": t["color"]} for t in self.target_queue]
            self.sim_renderer.render_multiview_dashboard(
                joint_ticks=self.current_joint_ticks,
                cubes=cubes_sim,
                status_text="Synchronized with Physical Camera Scan",
            )
            self.logger.log("SUCCESS", "Updated 3D digital twin multi-perspective inspection rendering.")
        except Exception as e:
            self.logger.log("WARN", f"Sim render error: {e}")

    def connect_driver(self) -> bool:
        """Connects to physical xArm if not already connected."""
        if self.dry_run:
            return True
        if self.driver is not None and self.driver.is_connected:
            return True

        devices = find_all_xarm_devices()
        target_port = self.requested_port
        if target_port == "auto" and devices:
            target_port = devices[0][0]

        try:
            hw_data = {}
            hw_yaml = os.path.join(repo_root, "config", "xarm_hardware.yaml")
            if os.path.exists(hw_yaml):
                with open(hw_yaml, "r") as f:
                    hw_data = yaml.safe_load(f) or {}
            arm_cfg = hw_data.get("manipulators", {}).get("xarm_1", {})
            motors_cfg = arm_cfg.get("motors", {})
            deviations = {
                sid: int(motors_cfg.get(f"motor_{sid}", {}).get("deviation", 0))
                for sid in range(1, 7)
            }
            max_dev = int(hw_data.get("manipulators", {}).get("max_deviation_ticks", 25))
            self.driver = XArmDriver(
                port=target_port,
                deviations=deviations,
                min_ticks=self.params.min_safe_ticks,
                max_ticks=self.params.max_safe_ticks,
                max_deviation_ticks=max_dev,
            )
            if self.driver.connect():
                self.xarm_connected = True
                self.xarm_port = target_port
                self.logger.log("SUCCESS", f"Connected to xArm controller on {target_port}")
                return True
        except Exception as e:
            self.logger.log("ERROR", f"Failed connecting to xArm: {e}")

        self.xarm_connected = False
        return False

    def execute_pick_target(self, target: Dict[str, Any]):
        """Executes the 7-phase top-down plunge trajectory for a single target."""
        self.active_target = target
        self.is_executing = True

        color = target["color"]
        r = target["reach_r"]
        theta = target["theta_arm"]
        roll_tick = target["roll_tick"]
        dest = target["dest_container"]

        self.logger.log("STAGE", f"Executing Sort for {color.upper()} Block at r={r:.1f}mm, th={theta:+.1f}°")

        # Load drop container pose
        drop_target = get_compartment_target(color=color, arm_id="xarm_1")
        drop_ticks = drop_target.get("ik_ticks", {1: 250, 2: 500, 3: 450, 4: 250, 5: 650, 6: 650 if color == "red" else 354})

        # Locate matching cube in sim_cubes so simulation animates exact cube pickup/drop
        sim_cube = None
        best_d = 40.0
        for sc in self.sim_cubes:
            if sc.get("state") == "in_cell" and sc.get("color") == color:
                d = math.hypot(sc["x"] - target["world_x"], sc["y"] - target["world_y"])
                if d < best_d:
                    best_d = d
                    sim_cube = sc

        def dispatch(ticks: Dict[int, int], move_time_ms: int = 600):
            if not self.is_executing:
                return
            start_ticks = dict(self.current_joint_ticks)
            if not self.dry_run and self.driver and self.driver.is_connected:
                self.driver.set_servo_positions(ticks, duration_ms=move_time_ms)

            # Interpolate joint ticks at ~20 Hz so Web 3D Digital Twin & RViz2 animate smoothly
            steps = max(1, int(round(move_time_ms / 50.0)))
            dt = (move_time_ms / 1000.0) / steps
            for s in range(1, steps + 1):
                if not self.is_executing:
                    return
                alpha = s / float(steps)
                interp = {
                    mid: int(round(start_ticks.get(mid, 500) + alpha * (ticks.get(mid, start_ticks.get(mid, 500)) - start_ticks.get(mid, 500))))
                    for mid in range(1, 7)
                }
                self.current_joint_ticks = interp
                if sim_cube is not None and sim_cube.get("state") == "grabbed":
                    fk = self.sim_renderer.geom.compute_joint_positions(interp)
                    tx, ty, tz = fk["tool_tip"]
                    sim_cube["x"] = tx
                    sim_cube["y"] = ty
                    sim_cube["z"] = max(22.5, tz)
                self.publish_joint_states()
                self._update_sim_view()
                time.sleep(dt)
            time.sleep(0.08)

        try:
            # Phase 1: Overhead Hover Standoff (Z = 80 mm clears 45 mm cube + 18 mm fingertip)
            self.trajectory_step = "hover"
            standoff_ik = solve_ik(r=r, theta=theta, z=80.0, roll_tick=roll_tick, params=self.params, compensate_sag=False)
            if standoff_ik.success:
                ticks = dict(standoff_ik.ticks)
                ticks[1] = self.open_claw_tick  # Fully open claw before approaching
                self.logger.log("INFO", f"  Phase 1: Overhead Standoff (Z=80mm) -> Ticks {ticks}")
                dispatch(ticks, 750)

            # Phase 2: Pure Vertical Plunge (arm-relative Z centered on 45 mm cube, with 3.5 mm Z-clearance offset)
            base_pick_z = float(target.get("z_rel", float(target.get("world_z", self.cube_center_z_mm)) - 5.0))
            pick_z = max(19.0, base_pick_z + 3.5)
            self.trajectory_step = "plunge"
            pick_ik = solve_ik(r=r, theta=theta, z=pick_z, roll_tick=roll_tick, params=self.params, compensate_sag=False)
            if pick_ik.success:
                ticks = dict(pick_ik.ticks)
                ticks[1] = self.open_claw_tick  # Keep claw fully open for plunge
                self.logger.log("INFO", f"  Phase 2: Vertical Plunge (Z_rel={pick_z:.1f}mm, Roll={roll_tick}) -> Ticks {ticks}")
                dispatch(ticks, 700)

                # Closed-loop potentiometer micro-trim (neutralizes +-5 tick servo deadband/backlash)
                if not self.dry_run and self.driver and self.driver.is_connected:
                    time.sleep(0.10)
                    actual_pos = self.driver.get_servo_positions()
                    if actual_pos:
                        min_safe = self.params.min_safe_ticks or {i: 150 for i in range(1, 7)}
                        max_safe = self.params.max_safe_ticks or {i: 850 for i in range(1, 7)}
                        trim_ticks = dict(ticks)
                        max_err = 0
                        for mid in (3, 4, 5, 6):
                            err = int(ticks[mid]) - int(actual_pos.get(mid, ticks[mid]))
                            if abs(err) >= 2:
                                corr = max(-12, min(12, err))
                                trim_ticks[mid] = max(min_safe.get(mid, 150), min(max_safe.get(mid, 850), ticks[mid] + corr))
                                max_err = max(max_err, abs(err))
                        if max_err >= 2:
                            self.logger.log("INFO", f"  Closed-loop trim (max err {max_err} ticks) -> {trim_ticks}")
                            dispatch(trim_ticks, 220)
                            time.sleep(0.10)
                    else:
                        time.sleep(0.08)

            # Phase 3: Firm Clamping (calibrated tick for 37.5 mm block with settle window)
            self.trajectory_step = "clamp"
            clamp_ticks = dict(self.current_joint_ticks)
            clamp_ticks[1] = self.clamp_claw_tick
            self.logger.log("INFO", f"  Phase 3: Firm Grip Clamping at {self.clamp_claw_tick} ticks")
            dispatch(clamp_ticks, 600)
            if sim_cube is not None:
                sim_cube["state"] = "grabbed"

            # Phase 4: Vertical Extraction Lift (Z = 90 mm)
            self.trajectory_step = "lift"
            lift_ik = solve_ik(r=r, theta=theta, z=90.0, roll_tick=roll_tick, params=self.params, compensate_sag=False)
            if lift_ik.success:
                ticks = dict(lift_ik.ticks)
                ticks[1] = self.clamp_claw_tick
                self.logger.log("INFO", "  Phase 4: Vertical Lift to Z=90mm")
                dispatch(ticks, 600)

            # Phase 5: High Rear-Facing Transit Over Shoulder
            self.trajectory_step = "rear"
            transit_ticks = {1: self.clamp_claw_tick, 2: 500, 3: 350, 4: 500, 5: 450, 6: drop_ticks[6]}
            self.logger.log("INFO", f"  Phase 5: High Overhead Rear Transit -> Base Yaw: {drop_ticks[6]}")
            dispatch(transit_ticks, 700)

            # Phase 6: Container Descent & Release
            self.trajectory_step = "drop"
            target_drop = dict(drop_ticks)
            target_drop[1] = self.clamp_claw_tick
            dispatch(target_drop, 650)

            target_drop[1] = self.open_claw_tick  # Fully open claw
            if sim_cube is not None:
                sim_cube["state"] = "placed"
                sim_cube["x"] = 50.0 if color == "red" else 300.0
                sim_cube["y"] = 550.0
                sim_cube["z"] = 45.0
            self.logger.log("SUCCESS", f"  Phase 6: Released into {dest.upper()} Container Box.")
            dispatch(target_drop, 350)

            # Phase 7: Home Pose Reset
            self.trajectory_step = "home"
            home_ticks = {1: self.open_claw_tick, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
            dispatch(home_ticks, 700)
            self.trajectory_step = "idle"

        except Exception as e:
            self.logger.log("ERROR", f"Execution error: {e}")
        finally:
            self.is_executing = False
            self.active_target = None

    def sort_next(self):
        """Sorts the next target in queue in a background thread."""
        if not self.target_queue:
            self.logger.log("WARN", "Queue is empty. Scan arena first.")
            return
        if self.is_executing:
            self.logger.log("WARN", "Arm is currently busy executing a pick.")
            return

        with self.frame_lock:
            if self.captured_frames and not self.is_locked:
                self.is_locked = True
        self.connect_driver()
        target = self.target_queue.pop(0)
        threading.Thread(target=self.execute_pick_target, args=(target,), daemon=True).start()

    def sort_all(self):
        """Sequentially sorts all items in queue."""
        if not self.target_queue:
            self.logger.log("WARN", "Queue is empty. Scan arena first.")
            return
        if self.is_executing:
            self.logger.log("WARN", "Arm is currently busy.")
            return

        with self.frame_lock:
            if self.captured_frames and not self.is_locked:
                self.is_locked = True

        def run_all():
            self.connect_driver()
            while self.target_queue and self.running:
                target = self.target_queue.pop(0)
                self.execute_pick_target(target)
                time.sleep(0.5)
            self.logger.log("SUCCESS", "All scheduled targets completed successfully!")

        threading.Thread(target=run_all, daemon=True).start()

    def emergency_stop(self):
        """Immediately halts all motion and cuts servo torque."""
        self.logger.log("ERROR", "EMERGENCY STOP TRIGGERED!")
        self.is_executing = False
        self.active_target = None
        self.trajectory_step = "ESTOP"
        if not self.dry_run and self.driver and self.driver.is_connected:
            try:
                self.driver.emergency_stop()
            except Exception:
                pass

    def home_pose(self):
        """Sends arm to neutral tucked home position."""
        self.connect_driver()
        if self.driver is not None:
            self.driver.is_estopped = False
        home_ticks = {1: 250, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
        self.current_joint_ticks = dict(home_ticks)
        if not self.dry_run and self.driver and self.driver.is_connected:
            self.driver.set_servo_positions(home_ticks, duration_ms=750)
        self.logger.log("INFO", "Moved arm to Home Pose.")


# Global Manager Instance
MANAGER: Optional[AutonomousSystemManager] = None


# -----------------------------------------------------------------------------
# HTTP Request Handler & MJPEG Stream Generator
# -----------------------------------------------------------------------------
class DashboardHTTPHandler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        # Silence verbose standard HTTP request logs
        return

    def do_GET(self):
        global MANAGER
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path in ("/", "/index.html"):
            html_path = os.path.join(repo_root, "tools", "dashboard_ui", "index.html")
            try:
                with open(html_path, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            except Exception as e:
                self.send_error(500, f"Error reading index.html: {e}")
            return

        elif path == "/api/cameras":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            cams = get_available_cameras()
            payload = {
                "active_camera_idx": MANAGER.camera_idx if MANAGER else 0,
                "cameras": cams,
            }
            self.wfile.write(json.dumps(payload).encode("utf-8"))
            return

        elif path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            cams = get_available_cameras()
            active_cam = next((c for c in cams if c["index"] == (MANAGER.camera_idx if MANAGER else 0)), None)

            status_payload = {
                "camera_connected": MANAGER.camera_connected if MANAGER else False,
                "camera_idx": MANAGER.camera_idx if MANAGER else 0,
                "camera_perm_ok": MANAGER.camera_perm_ok if MANAGER else True,
                "active_camera": active_cam,
                "available_cameras": cams,
                "xarm_connected": MANAGER.xarm_connected if MANAGER else False,
                "xarm_port": MANAGER.xarm_port if MANAGER else None,
                "xarm_type": getattr(MANAGER, "xarm_type", None) if MANAGER else None,
                "xarm_perm_ok": MANAGER.xarm_perm_ok if MANAGER else False,
                "dry_run": MANAGER.dry_run if MANAGER else False,
                "is_locked": MANAGER.is_locked if MANAGER else False,
                "captured_count": len(MANAGER.captured_frames) if MANAGER else 0,
                "target_capture_count": MANAGER.target_capture_count if MANAGER else 4,
                "alignment_quality_pct": MANAGER.alignment_quality_pct if MANAGER else 0,
                "align_prompts": MANAGER.align_prompts if MANAGER else [],
                "clamp_claw_tick": MANAGER.clamp_claw_tick if MANAGER else 580,
                "cube_center_z_mm": MANAGER.cube_center_z_mm if MANAGER else 22.5,
                "all_markers_locked": MANAGER.all_markers_locked if MANAGER else False,
                "camera_height_mm": MANAGER.camera_height_mm if MANAGER else None,
                "depression_angle_deg": MANAGER.depression_angle_deg if MANAGER else None,
                "aruco_reports": MANAGER.aruco_reports if MANAGER else {},
                "target_queue": MANAGER.target_queue if MANAGER else [],
                "trajectory_step": MANAGER.trajectory_step if MANAGER else "idle",
                "is_executing": MANAGER.is_executing if MANAGER else False,
                "logs": MANAGER.logger.get_logs() if MANAGER else [],
            }
            self.wfile.write(json.dumps(status_payload).encode("utf-8"))
            return

        elif path in ("/stream/camera", "/stream/warped", "/stream/sim"):
            self.handle_mjpeg_stream(path)
            return

        else:
            self.send_error(404, "Endpoint not found.")

    def do_POST(self):
        global MANAGER
        if self.path == "/api/action":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            try:
                data = json.loads(body.decode("utf-8"))
                action = data.get("action")

                res = {"success": True, "message": f"Action {action} processed"}
                if action in ("snap_and_lock", "capture_frame"):
                    MANAGER.snap_and_lock()
                elif action == "load_saved":
                    MANAGER.load_saved_captures()
                elif action == "lock_and_simulate":
                    MANAGER.lock_and_simulate()
                elif action in ("unlock_camera", "clear_captures"):
                    MANAGER.unlock_camera()
                elif action in ("switch_camera", "set_camera"):
                    new_idx = int(data.get("camera_idx", data.get("index", 0)))
                    MANAGER.switch_camera(new_idx)
                    res = {"success": True, "message": f"Switched to camera index {new_idx}"}
                elif action == "scan_arena":
                    MANAGER.scan_arena_targets()
                elif action == "sync_simulation":
                    MANAGER.sync_digital_twin()
                elif action == "sort_next":
                    MANAGER.sort_next()
                elif action == "sort_all":
                    MANAGER.sort_all()
                elif action == "estop":
                    MANAGER.emergency_stop()
                elif action == "home":
                    MANAGER.home_pose()
                elif action == "check_hardware":
                    MANAGER._open_camera()
                    MANAGER.connect_driver()
                else:
                    res = {"success": False, "message": f"Unknown action: {action}"}

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(res).encode("utf-8"))
            except Exception as e:
                self.send_error(500, f"Error processing action: {e}")

    def handle_mjpeg_stream(self, path: str):
        """Generates continuous multipart JPEG stream."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, private")
        self.end_headers()

        while True:
            try:
                frame_to_send = None
                with MANAGER.frame_lock:
                    if path == "/stream/camera":
                        if MANAGER.is_locked and MANAGER.latest_sim is not None:
                            frame_to_send = MANAGER.latest_sim
                        else:
                            frame_to_send = MANAGER.latest_hud_frame if MANAGER.latest_hud_frame is not None else MANAGER.latest_frame
                    elif path == "/stream/warped":
                        frame_to_send = MANAGER.latest_warped
                    elif path == "/stream/sim":
                        frame_to_send = MANAGER.latest_sim

                if frame_to_send is None:
                    # Send blank frame placeholder
                    placeholder = np.full((300, 400, 3), 20, dtype=np.uint8)
                    cv2.putText(placeholder, "AWAITING FRAME...", (50, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 2)
                    frame_to_send = placeholder

                ret, jpeg = cv2.imencode(".jpg", frame_to_send, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b"\r\n")

                time.sleep(0.05)  # 20 FPS
            except (BrokenPipeError, ConnectionResetError):
                break
            except Exception:
                time.sleep(0.1)


def main():
    global MANAGER
    parser = argparse.ArgumentParser(description="xArm 1S Autonomous System Web Dashboard")
    parser.add_argument("--port", type=int, default=8080, help="Web server port (default: 8080)")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--serial_port", type=str, default="auto", help="Serial port (default: auto)")
    parser.add_argument("--dry-run", action="store_true", help="Run in mock simulation mode")
    parser.add_argument("--rviz", action="store_true", help="Also launch RViz2 digital twin mirror (Docker ROS 2 only)")
    parser.add_argument("--load-saved", action="store_true", help="Load saved frames from data/captured_frames/ on startup")
    args = parser.parse_args()

    rviz_proc = None
    if args.rviz and HAS_ROS2:
        try:
            rviz_proc = subprocess.Popen(
                ["ros2", "launch", "sorting_arm", "simulation.launch.py", "mirror_only:=true", "gui:=true", "default_cubes:=false"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("[INFO] Launched RViz2 Digital Twin Mirror in background...")
        except Exception as e:
            print(f"[WARN] Could not launch RViz2 mirror: {e}")

    MANAGER = AutonomousSystemManager(
        camera_idx=args.camera,
        port=args.serial_port,
        dry_run=args.dry_run,
    )
    if args.load_saved:
        MANAGER.load_saved_captures()

    port = args.port
    server = None
    for attempt_port in [port, port + 1, port + 2, 8000, 8081]:
        try:
            server = ThreadingHTTPServer(("0.0.0.0", attempt_port), DashboardHTTPHandler)
            port = attempt_port
            break
        except OSError:
            continue

    if not server:
        print(f"Failed to bind HTTP server to any port near {args.port}.")
        sys.exit(1)

    print("\n" + "=" * 78)
    print("  🚀 xArm 1S Autonomous Dashboard & Mission Control")
    print(f"  👉 Web Dashboard URL: http://localhost:{port}")
    print("  " + "-" * 74)
    print("  ⚠️  STRICT TWO-ENVIRONMENT EXECUTION POLICY:")
    print("     • ANY command related to ROS 2 MUST run in Docker: ./docker/run.sh")
    print("     • ANY Python command MUST run in virtualenv: source venv/bin/activate")
    print("=" * 78 + "\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down dashboard...")
    finally:
        if rviz_proc is not None:
            try:
                rviz_proc.terminate()
            except Exception:
                pass
        if MANAGER:
            MANAGER.running = False
            if MANAGER.cap:
                MANAGER.cap.release()
            if MANAGER.driver:
                MANAGER.driver.disconnect()


if __name__ == "__main__":
    main()
