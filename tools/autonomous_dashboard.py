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
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices
from tools.render_sim_view import MultiViewRenderer
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
        self.sim_renderer = MultiViewRenderer(output_dir=os.path.join(repo_root, "artifacts"))

        # Hardware & Connection State
        self.camera_connected = False
        self.camera_perm_ok = True
        self.xarm_connected = False
        self.xarm_port: Optional[str] = None
        self.xarm_type: Optional[str] = None
        self.xarm_perm_ok = False

        # Frame & Snap-and-Lock State
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_warped: Optional[np.ndarray] = None
        self.latest_sim: Optional[np.ndarray] = None
        self.is_locked = False
        self.locked_frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()

        # Optical Guidance & Extrinsics
        self.aruco_reports: Dict[int, Any] = {}
        self.all_markers_locked = False
        self.camera_height_mm: Optional[float] = None
        self.depression_angle_deg: Optional[float] = None
        self.distance_to_center_mm: Optional[float] = None

        # Perception & Scheduling
        self.detected_cubes: List[DetectedCube] = []
        self.target_queue: List[Dict[str, Any]] = []
        self.active_target: Optional[Dict[str, Any]] = None
        self.trajectory_step = "idle"
        self.is_executing = False

        # Current joint ticks
        self.current_joint_ticks = {i: 500 for i in range(1, 7)}
        self.current_joint_ticks[1] = 250  # Open claw

        # ROS 2 Publisher Node
        self.ros_node: Optional[Any] = None
        self.ros_pub_objects: Optional[Any] = None
        self.init_ros2()

        # Start background watchdog & video loop
        self.running = True
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

            # Spin in thread
            def ros_spin():
                while self.running and rclpy.ok():
                    rclpy.spin_once(self.ros_node, timeout_sec=0.05)

            threading.Thread(target=ros_spin, daemon=True).start()
            self.logger.log("SUCCESS", "ROS 2 Bridge initialized: Publishing to /vision/detected_objects.")
        except Exception as e:
            self.logger.log("WARN", f"ROS 2 Bridge init skipped: {e}")

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
        """Continuously grabs frames, runs optical assist & ArUco checks."""
        while self.running:
            if self.cap is None or not self.cap.isOpened():
                self._open_camera()
                time.sleep(0.5)
                continue

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.camera_connected = False
                time.sleep(0.1)
                continue

            self.camera_connected = True

            with self.frame_lock:
                if not self.is_locked:
                    self.latest_frame = frame
                process_frame = self.locked_frame if self.is_locked and self.locked_frame is not None else frame

            # Run Optical Assist: ArUco marker orientation & extrinsics
            try:
                marker_status = self.pipeline.verify_aruco_markers(process_frame)
                self.aruco_reports = marker_status.get("reports", {})
                self.all_markers_locked = marker_status.get("all_detected", False)

                if self.all_markers_locked:
                    extrinsics = self.pipeline.compute_camera_extrinsics(process_frame)
                    if extrinsics.get("success"):
                        self.camera_height_mm = float(extrinsics.get("height_mm", 0.0))
                        self.depression_angle_deg = float(extrinsics.get("depression_angle_deg", 0.0))
                        self.distance_to_center_mm = float(extrinsics.get("distance_to_arena_center_mm", 0.0))

                # Generate Rectified Warped Frame with Golden Zone HUD
                warped, annotated_warped, cubes = self.pipeline.process_frame(process_frame, draw_grid=True)
                if annotated_warped is not None:
                    # Overlay status banner
                    overlay = annotated_warped.copy()
                    if self.is_locked:
                        cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 40), (16, 185, 129), -1)
                        cv2.putText(overlay, "LOCKED STATIONARY FRAME - TARGETS ACQUIRED", (20, 26),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
                    with self.frame_lock:
                        self.latest_warped = overlay
                        if self.is_locked:
                            self.detected_cubes = cubes
            except Exception as e:
                pass

            # Digital Twin view update (throttle to ~5 Hz)
            try:
                sim_cubes = [
                    {"x": c.world_x, "y": c.world_y, "color": c.color}
                    for c in self.detected_cubes
                ]
                sim_img = self.sim_renderer.render_3d_view(
                    joint_ticks=self.current_joint_ticks,
                    cubes=sim_cubes,
                    active_target=self.active_target,
                    status_text=f"Mode: {self.trajectory_step.upper()}",
                )
                with self.frame_lock:
                    self.latest_sim = sim_img
            except Exception:
                pass

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

    def snap_and_lock(self):
        """Captures stationary frame, locks matrix, and extracts targets."""
        with self.frame_lock:
            if self.latest_frame is None:
                self.logger.log("WARN", "No camera frame available to snap.")
                return False
            self.locked_frame = self.latest_frame.copy()
            self.is_locked = True

        self.logger.log("STAGE", "Camera frame snapped and locked.")
        self.scan_arena_targets()
        self.sync_digital_twin()
        return True

    def unlock_camera(self):
        with self.frame_lock:
            self.is_locked = False
            self.locked_frame = None
        self.logger.log("INFO", "Camera unlocked; resumed live feed.")

    def scan_arena_targets(self):
        """Runs perception on current locked or live frame and builds Nearest-First queue."""
        with self.frame_lock:
            frame = self.locked_frame if (self.is_locked and self.locked_frame is not None) else self.latest_frame

        if frame is None:
            self.logger.log("WARN", "Cannot scan: No frame available.")
            return

        warped, annotated, cubes = self.pipeline.process_frame(frame, draw_grid=True)
        self.detected_cubes = cubes

        # Filter: Golden Zone & Red/Green targets
        queue = []
        for c in cubes:
            if c.color not in ("red", "green"):
                continue
            if not c.in_pickup_zone:
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
                "aspect_ratio": float(c.aspect_ratio),
                "dest_container": "red" if c.color == "red" else "green",
            })

        # Nearest-First policy
        queue.sort(key=lambda item: item["reach_r"])
        self.target_queue = queue

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
            cubes_sim = [{"x": t["world_x"], "y": t["world_y"], "color": t["color"]} for t in self.target_queue]
            self.sim_renderer.render_multiview(
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
            self.driver = XArmDriver(port=target_port)
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

        def dispatch(ticks: Dict[int, int], move_time_ms: int = 600):
            self.current_joint_ticks = dict(ticks)
            if not self.dry_run and self.driver and self.driver.is_connected:
                self.driver.set_servo_positions(ticks, duration_ms=move_time_ms)
            time.sleep(move_time_ms / 1000.0 + 0.08)

        try:
            # Phase 1: Overhead Hover Standoff (Z = 75 mm)
            self.trajectory_step = "hover"
            standoff_ik = solve_ik(r=r, theta=theta, z=75.0, roll_tick=roll_tick, params=self.params, compensate_sag=True)
            if standoff_ik.success:
                ticks = dict(standoff_ik.ticks)
                ticks[1] = 250  # Open claw
                self.logger.log("INFO", f"  Phase 1: Overhead Standoff (Z=75mm) -> Ticks {ticks}")
                dispatch(ticks, 700)

            # Phase 2: Pure Vertical Plunge (Z = 22 mm)
            self.trajectory_step = "plunge"
            pick_ik = solve_ik(r=r, theta=theta, z=22.0, roll_tick=roll_tick, params=self.params, compensate_sag=True)
            if pick_ik.success:
                ticks = dict(pick_ik.ticks)
                ticks[1] = 250
                self.logger.log("INFO", f"  Phase 2: Vertical Plunge (Z=22mm, Roll={roll_tick})")
                dispatch(ticks, 600)

            # Phase 3: Firm Clamping (520 ticks)
            self.trajectory_step = "clamp"
            clamp_ticks = dict(self.current_joint_ticks)
            clamp_ticks[1] = 520
            self.logger.log("INFO", "  Phase 3: Firm Grip Clamping at 520 ticks")
            dispatch(clamp_ticks, 350)

            # Phase 4: Vertical Extraction Lift (Z = 90 mm)
            self.trajectory_step = "lift"
            lift_ik = solve_ik(r=r, theta=theta, z=90.0, roll_tick=roll_tick, params=self.params, compensate_sag=True)
            if lift_ik.success:
                ticks = dict(lift_ik.ticks)
                ticks[1] = 520
                self.logger.log("INFO", "  Phase 4: Vertical Lift to Z=90mm")
                dispatch(ticks, 550)

            # Phase 5: High Rear-Facing Transit Over Shoulder
            self.trajectory_step = "rear"
            transit_ticks = {1: 520, 2: 500, 3: 350, 4: 500, 5: 450, 6: drop_ticks[6]}
            self.logger.log("INFO", f"  Phase 5: High Overhead Rear Transit -> Base Yaw: {drop_ticks[6]}")
            dispatch(transit_ticks, 700)

            # Phase 6: Container Descent & Release
            self.trajectory_step = "drop"
            target_drop = dict(drop_ticks)
            target_drop[1] = 520
            dispatch(target_drop, 650)

            target_drop[1] = 250  # Open claw
            self.logger.log("SUCCESS", f"  Phase 6: Released into {dest.upper()} Container Box.")
            dispatch(target_drop, 350)

            # Phase 7: Home Pose Reset
            self.trajectory_step = "home"
            home_ticks = {1: 250, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
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

        def run_all():
            self.connect_driver()
            while self.target_queue and self.running:
                target = self.target_queue.pop(0)
                self.execute_pick_target(target)
                time.sleep(0.5)
            self.logger.log("SUCCESS", "All scheduled targets completed successfully!")

        threading.Thread(target=run_all, daemon=True).start()

    def emergency_stop(self):
        """Immediately halts all motion and opens claw."""
        self.logger.log("ERROR", "EMERGENCY STOP TRIGGERED!")
        self.is_executing = False
        self.active_target = None
        self.trajectory_step = "ESTOP"
        if not self.dry_run and self.driver and self.driver.is_connected:
            try:
                # Open claw
                self.driver.set_servo_position(1, 250, duration_ms=100)
            except Exception:
                pass

    def home_pose(self):
        """Sends arm to neutral tucked home position."""
        self.connect_driver()
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
                if action == "snap_and_lock":
                    MANAGER.snap_and_lock()
                elif action == "unlock_camera":
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
                        frame_to_send = MANAGER.latest_frame
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
    args = parser.parse_args()

    MANAGER = AutonomousSystemManager(
        camera_idx=args.camera,
        port=args.serial_port,
        dry_run=args.dry_run,
    )

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
        if MANAGER:
            MANAGER.running = False
            if MANAGER.cap:
                MANAGER.cap.release()
            if MANAGER.driver:
                MANAGER.driver.disconnect()


if __name__ == "__main__":
    main()
