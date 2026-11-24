#!/usr/bin/env python3
"""
Arena Grid & Cube Simulation Visualizer Node for xArm 1S in RViz.

Features:
  - Publishes a 5 cm (50 mm) square grid over the 350mm x 600mm baseboard (Marker.LINE_LIST).
  - Displays Column (C0..C6) and Row (R0..R11) cell coordinate markers.
  - Displays Picking Zone boundary and Rear Flank Container Box markers.
  - Tracks the xArm tool_tip in real time via TF and displays a floating HUD + ground shadow.
  - Renders red and green visual cubes with dynamic claw attachment / release.
  - Subscribes to /clicked_point (from RViz Publish Point tool) to spawn cubes interactively.
  - Publishes detected cubes on /vision/detected_objects to trigger autonomous pick-and-place.
"""

from dataclasses import dataclass, field
import json
import math
import os
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sorting_arm.kinematics import (
    DEG_PER_TICK,
    TICKS_PER_DEG,
    load_kinematic_parameters,
    world_to_arm_polar,
)
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class VisualCube:
    """Represents a physical/visual craft cube in the simulation."""
    id: int
    color: str  # "red" or "green"
    x_m: float
    y_m: float
    z_m: float = 0.0127  # Half-height (25.4mm cube resting on table Z=0)
    state: str = "in_cell"  # "in_cell", "grabbed", "placed"
    initial_col: int = 0
    initial_row: int = 0


class ArenaVisualizerNode(Node):
    """ROS 2 Node for visual arena markers, grid, and interactive cubes in RViz."""

    def __init__(self):
        super().__init__("arena_visualizer_node")

        self.declare_parameter("arm_id", "xarm_1")
        self.declare_parameter("config_dir", "config")
        self.declare_parameter("default_cubes", True)
        self.declare_parameter("grid_mode", "full_grid")

        self.arm_id = self.get_parameter("arm_id").get_parameter_value().string_value
        self.config_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        self.spawn_default_cubes = self.get_parameter("default_cubes").get_parameter_value().bool_value
        self.grid_mode = self.get_parameter("grid_mode").get_parameter_value().string_value

        hardware_yaml = os.path.join(self.config_dir, "xarm_hardware.yaml")
        self.params = load_kinematic_parameters(hardware_yaml, arm_id=self.arm_id)

        # TF Buffer & Listener for live tool_tip pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, "/simulation/arena_markers", 10)
        self.pub_cube_markers = self.create_publisher(MarkerArray, "/simulation/cube_markers", 10)
        self.pub_default_markers = self.create_publisher(MarkerArray, "/visualization_marker_array", 10)
        self.pub_vision = self.create_publisher(String, "/vision/detected_objects", 10)

        # Subscribers
        self.sub_clicked = self.create_subscription(
            PointStamped, "/clicked_point", self.handle_clicked_point, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, f"/{self.arm_id}/joint_states", self.handle_joint_states, 10
        )
        self.sub_vision = self.create_subscription(
            String, "/vision/detected_objects", self.handle_vision_telemetry, 10
        )

        # Services
        self.srv_spawn = self.create_service(Trigger, "/simulation/spawn_cube", self.handle_spawn_cube_srv)
        self.srv_clear = self.create_service(Trigger, "/simulation/clear_cubes", self.handle_clear_cubes_srv)
        self.srv_reseed = self.create_service(Trigger, "/simulation/reseed_grid", self.handle_reseed_grid_srv)
        self.srv_populate = self.create_service(Trigger, "/simulation/populate_grid", self.handle_populate_grid_srv)
        self.srv_hard_zones = self.create_service(Trigger, "/simulation/spawn_hard_zones", self.handle_spawn_hard_zones_srv)

        # State tracking
        self.claw_state = "OPEN"
        self.claw_tick = self.params.min_safe_ticks.get(1, 226)
        self.claw_is_clamped = False
        self.gripped_cube_id: Optional[int] = None
        self.current_ee_pos: Optional[Point] = None

        self.cubes: List[VisualCube] = []
        self.next_cube_id = 1
        self.next_color = "red"

        if self.spawn_default_cubes:
            if self.grid_mode == "hard_zones":
                self.populate_hard_zones()
            elif self.grid_mode == "full_grid":
                self.populate_full_grid(randomize=True)
            else:
                # Default dual cubes: Red at (Row 8, Col 2), Green at (Row 8, Col 4)
                self._add_cube("red", 0.125, 0.175)
                self._add_cube("green", 0.225, 0.175)

        # Timer for RViz marker updates (10 Hz)
        self.marker_timer = self.create_timer(0.10, self.update_and_publish_markers)

        # Periodic vision sync (1 Hz) to ensure autonomous_sorter always has latest targets
        self.vision_timer = self.create_timer(1.0, self.publish_detected_objects)

        self.get_logger().info(
            f"Arena Visualizer Node active for {self.arm_id} (Grid Mode: {self.grid_mode}, 5cm Grid, Zero-Text 3D View enabled)."
        )

    def _add_cube(self, color: str, x_m: float, y_m: float) -> VisualCube:
        """Adds a visual cube snapped to grid cell center."""
        col = int(math.floor(x_m / 0.050))
        row = 11 - int(math.floor(y_m / 0.050))
        col = max(0, min(6, col))
        row = max(0, min(11, row))

        cube = VisualCube(
            id=self.next_cube_id,
            color=color.lower(),
            x_m=float(x_m),
            y_m=float(y_m),
            z_m=0.0137,
            state="in_cell",
            initial_col=col,
            initial_row=row,
        )
        self.next_cube_id += 1
        self.cubes.append(cube)
        self.get_logger().info(
            f"Spawned {color.upper()} cube #{cube.id} at ({x_m*1000.0:.1f}mm, {y_m*1000.0:.1f}mm) [Col {col}, Row {row}]."
        )
        return cube

    def populate_full_grid(self, randomize: bool = True) -> Tuple[int, int]:
        """Populates all 45 valid picking cells (Rows 3..10, Cols 0..6, excluding ArUco and xArm mount) with randomly assigned Red and Green craft cubes."""
        import random
        red_cnt = 0
        grn_cnt = 0
        for r in range(3, 11):
            for c in range(7):
                # Exclude ArUco markers: Row 3 Col 0 and Row 3 Col 6
                if (c, r) in [(0, 3), (6, 3)]:
                    continue
                # Exclude xArm base mounting footprint: Cols 2..4 in Rows 3..5
                if c in (2, 3, 4) and r in (3, 4, 5):
                    continue

                if randomize:
                    color = random.choice(["red", "green"])
                else:
                    color = "red" if (r + c) % 2 == 0 else "green"

                if color == "red":
                    red_cnt += 1
                else:
                    grn_cnt += 1

                # Calculate cell coordinates with near-pedestal reachability offsets
                if r == 6 and c == 3:
                    x_m = 0.175
                    y_m = 0.2505
                elif r == 6 and c in (2, 4):
                    x_m = (c + 0.5) * 0.050
                    y_m = 0.2505
                elif r in (3, 4, 5) and c == 1:
                    x_m = 0.052
                    y_m = 0.402 if r == 3 else (0.352 if r == 4 else 0.302)
                elif r in (3, 4, 5) and c == 5:
                    x_m = 0.298
                    y_m = 0.402 if r == 3 else (0.352 if r == 4 else 0.302)
                else:
                    x_m = (c + 0.5) * 0.050
                    y_m = (11 - r + 0.5) * 0.050

                self._add_cube(color, x_m, y_m)

        self.get_logger().info(
            f"Populated 45-cell grid (Rows 3..10, Cols 0..6): {red_cnt} RED, {grn_cnt} GREEN cubes."
        )
        return red_cnt, grn_cnt

    def populate_hard_zones(self) -> Tuple[int, int]:
        """
        Populates high-difficulty picking cells to stress-test kinematics and reach:
          - Near xArm base: Row 6 Col 1 (Red, r=141.4mm), Row 6 Col 5 (Green, r=141.4mm), Row 7 Col 3 (Red, r=150.0mm)
          - Extreme far corners: Row 10 Col 0 (Green, r=335.4mm), Row 10 Col 6 (Red, r=335.4mm)
        """
        hard_cells = [
            ("red", 1, 6),    # Near-base West: X=75mm, Y=275mm (r=141.4mm)
            ("green", 5, 6),  # Near-base East: X=275mm, Y=275mm (r=141.4mm)
            ("red", 3, 7),    # Direct South close: X=175mm, Y=225mm (r=150.0mm)
            ("green", 0, 10), # Far Northwest corner: X=25mm, Y=75mm (r=335.4mm)
            ("red", 6, 10),   # Far Northeast corner: X=325mm, Y=75mm (r=335.4mm)
        ]
        red_cnt = 0
        grn_cnt = 0
        for color, c, r in hard_cells:
            if color == "red":
                red_cnt += 1
            else:
                grn_cnt += 1
            x_m = (c + 0.5) * 0.050
            y_m = (11 - r + 0.5) * 0.050
            self._add_cube(color, x_m, y_m)

        self.get_logger().info(
            f"Populated hard-to-pick zones (Near base & far corners): {red_cnt} RED, {grn_cnt} GREEN cubes."
        )
        return red_cnt, grn_cnt

    def _publish_delete_all_markers(self):
        """Sends DELETEALL to RViz so old markers are flushed immediately."""
        del_array = MarkerArray()
        dm = Marker()
        dm.action = Marker.DELETEALL
        del_array.markers.append(dm)
        self.pub_markers.publish(del_array)
        self.pub_cube_markers.publish(del_array)
        self.pub_default_markers.publish(del_array)

    def handle_clicked_point(self, msg: PointStamped):
        """User clicked in RViz using the 'Publish Point' toolbar tool."""
        x = msg.point.x
        y = msg.point.y

        # Clamp to baseboard bounds (X in [0, 350mm], Y in [0, 600mm])
        if x < 0.0 or x > 0.350 or y < 0.0 or y > 0.600:
            self.get_logger().warn(f"Clicked point ({x:.3f}, {y:.3f}) is outside 350x600mm arena.")
            return

        col = int(math.floor(x / 0.050))
        row = 11 - int(math.floor(y / 0.050))
        col = max(0, min(6, col))
        row = max(0, min(11, row))

        # Guard against clicks on ArUco markers or xArm base mounting footprint
        if (col, row) in [(0, 3), (6, 3), (0, 11), (6, 11)]:
            self.get_logger().warn(f"Cannot place cube on ArUco marker at Col {col}, Row {row}.")
            return
        if col in (2, 3, 4) and row in (3, 4, 5):
            self.get_logger().warn(f"Cannot place cube on xArm mounting footprint at Col {col}, Row {row}.")
            return

        # Snap to cell center
        cell_center_x = (col + 0.5) * 0.050
        cell_center_y = (11 - row + 0.5) * 0.050

        # Alternate colors on successive clicks
        color = self.next_color
        self.next_color = "green" if color == "red" else "red"

        self._add_cube(color, cell_center_x, cell_center_y)
        self.publish_detected_objects()

    def handle_joint_states(self, msg: JointState):
        """Caches claw joint position to detect clamp vs release dynamically."""
        for name, val in zip(msg.name, msg.position):
            if "claw" in name or "motor_1" in name:
                center = self.params.nominal_center_ticks.get(1, 517)
                tick = int(round(center + math.degrees(val) * TICKS_PER_DEG))
                self.claw_tick = tick
                min_t = self.params.min_safe_ticks.get(1, 226)
                max_t = self.params.max_safe_ticks.get(1, 782)
                clamp_thresh = min_t + 0.48 * (max_t - min_t)
                open_thresh = min_t + 0.22 * (max_t - min_t)
                if tick >= clamp_thresh or val > 0.12:
                    self.claw_is_clamped = True
                    self.claw_state = "CLAMPED"
                elif tick <= open_thresh or val < -0.15:
                    self.claw_is_clamped = False
                    self.claw_state = "OPEN"
                break

    def handle_spawn_cube_srv(self, request, response):
        """Service callback to spawn a cube at next available cell."""
        color = self.next_color
        self.next_color = "green" if color == "red" else "red"

        # Find an empty cell in the Golden Zone (Rows 7..9, Cols 2..4) first, fallback to extended arena
        occupied = {(c.initial_col, c.initial_row) for c in self.cubes if c.state == "in_cell"}
        spawned = False
        for r in [8, 9, 7, 10]:
            for c in [3, 2, 4, 1, 5, 0, 6]:
                if (c, r) not in occupied:
                    x = (c + 0.5) * 0.050
                    y = (11 - r + 0.5) * 0.050
                    self._add_cube(color, x, y)
                    spawned = True
                    break
            if spawned:
                break

        if not spawned:
            # Fallback to center of Golden Zone
            self._add_cube(color, 0.175, 0.175)

        self.publish_detected_objects()
        response.success = True
        response.message = f"Spawned {color.upper()} cube."
        return response

    def handle_clear_cubes_srv(self, request, response):
        """Removes all cubes from the arena and clears RViz markers."""
        self.cubes.clear()
        self.gripped_cube_id = None
        self._publish_delete_all_markers()
        self.publish_detected_objects()
        response.success = True
        response.message = "All visual cubes cleared."
        return response

    def handle_reseed_grid_srv(self, request, response):
        """Service callback to clear placed cubes and reseed all 45 cells with a fresh scattered pattern."""
        self.cubes.clear()
        self.gripped_cube_id = None
        self._publish_delete_all_markers()
        r_cnt, g_cnt = self.populate_full_grid(randomize=True)
        self.publish_detected_objects()
        response.success = True
        response.message = f"Reseeded 45 grid cells: {r_cnt} RED, {g_cnt} GREEN."
        self.get_logger().info(response.message)
        return response

    def handle_populate_grid_srv(self, request, response):
        """Service callback to populate grid if currently empty."""
        if not any(c.state == "in_cell" for c in self.cubes):
            self.cubes.clear()
            self.gripped_cube_id = None
            self._publish_delete_all_markers()
            r_cnt, g_cnt = self.populate_full_grid(randomize=True)
            self.publish_detected_objects()
            response.success = True
            response.message = f"Populated 45 cells with {r_cnt} RED and {g_cnt} GREEN cubes."
        else:
            response.success = False
            response.message = "Grid already contains active cubes."
        return response

    def handle_spawn_hard_zones_srv(self, request, response):
        """Service callback to clear grid and spawn cubes in hard-to-pick zones."""
        self.cubes.clear()
        self.gripped_cube_id = None
        self._publish_delete_all_markers()
        r_cnt, g_cnt = self.populate_hard_zones()
        self.publish_detected_objects()
        response.success = True
        response.message = f"Spawned {r_cnt} RED and {g_cnt} GREEN cubes in hard-to-pick zones."
        return response

    def handle_vision_telemetry(self, msg: String):
        """Syncs visual cubes with external vision node telemetry."""
        try:
            data = json.loads(msg.data)
            if data.get("source") == "arena_visualizer":
                return  # Skip self-generated messages

            objs = data.get("objects", [])
            if data.get("sync_replace", False) or data.get("clear_before_sync", False):
                # Keep gripped cube if currently holding one, clear remaining in-cell cubes
                self.cubes = [c for c in self.cubes if c.state != "in_cell"]
                self._publish_delete_all_markers()
                for obj in objs:
                    color = obj.get("color", "red").lower()
                    wx_m = float(obj.get("world_x_mm", 0.0)) / 1000.0
                    wy_m = float(obj.get("world_y_mm", 0.0)) / 1000.0
                    self._add_cube(color, wx_m, wy_m)
                self.get_logger().info(f"Synchronized {len(objs)} real vision cubes into 3D digital twin.")
                return

            for obj in objs:
                color = obj.get("color", "red").lower()
                wx_m = float(obj.get("world_x_mm", 0.0)) / 1000.0
                wy_m = float(obj.get("world_y_mm", 0.0)) / 1000.0
                # Check if cube already exists nearby
                exists = any(
                    math.hypot(c.x_m - wx_m, c.y_m - wy_m) < 0.030 and c.color == color
                    for c in self.cubes
                )
                if not exists:
                    self._add_cube(color, wx_m, wy_m)
        except Exception as e:
            self.get_logger().error(f"Error syncing vision telemetry: {e}")

    def publish_detected_objects(self):
        """Publishes active in-cell cubes to /vision/detected_objects."""
        objects_data = []
        for cube in self.cubes:
            if cube.state != "in_cell":
                continue

            wx = cube.x_m * 1000.0
            wy = cube.y_m * 1000.0
            wz = cube.z_m * 1000.0

            r, theta, _ = world_to_arm_polar(wx, wy, wz, arm_id=self.arm_id, params=self.params)

            objects_data.append({
                "color": cube.color,
                "world_x_mm": round(wx, 2),
                "world_y_mm": round(wy, 2),
                "world_z_mm": round(wz, 2),
                "yaw_deg": 0.0,
                "assigned_arm": self.arm_id,
                "target_r_mm": round(r, 2),
                "target_theta_deg": round(theta, 2),
                "targetable": True,
                "confidence": 0.99,
            })

        msg_str = String()
        msg_str.data = json.dumps({
            "source": "arena_visualizer",
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "total_detected": len(objects_data),
            "homography_locked": True,
            "objects": objects_data,
        })
        self.pub_vision.publish(msg_str)

    def update_and_publish_markers(self):
        """Computes current TF, updates cube attachment state, and publishes MarkerArray."""
        # Query TF for tool_tip
        try:
            t = self.tf_buffer.lookup_transform("world", "tool_tip", rclpy.time.Time())
            ee_x = t.transform.translation.x
            ee_y = t.transform.translation.y
            ee_z = t.transform.translation.z
            self.current_ee_pos = Point(x=ee_x, y=ee_y, z=ee_z)
        except Exception:
            # TF not yet available
            pass

        # Update Cube physics / attachment logic
        if self.current_ee_pos:
            ee_x = self.current_ee_pos.x
            ee_y = self.current_ee_pos.y
            ee_z = self.current_ee_pos.z

            if self.claw_is_clamped and self.gripped_cube_id is None:
                # Find the closest in-cell cube within grasp range
                candidates = []
                for cube in self.cubes:
                    if cube.state == "in_cell":
                        dist = math.hypot(ee_x - cube.x_m, ee_y - cube.y_m, ee_z - cube.z_m)
                        if dist <= 0.035:
                            candidates.append((dist, cube))
                if candidates:
                    candidates.sort(key=lambda item: item[0])
                    best_cube = candidates[0][1]
                    best_cube.state = "grabbed"
                    self.gripped_cube_id = best_cube.id
                    self.get_logger().info(f"Claw gripped {best_cube.color.upper()} cube #{best_cube.id} (dist={candidates[0][0]*1000.0:.1f}mm).")
                    self.publish_detected_objects()

            for cube in self.cubes:
                if cube.state == "grabbed" and cube.id == self.gripped_cube_id:
                    if self.claw_is_clamped:
                        # Follow tool_tip
                        cube.x_m = ee_x
                        cube.y_m = ee_y
                        cube.z_m = max(0.0127, ee_z)
                    else:
                        # Released! Determine drop location
                        cube.state = "placed"
                        self.gripped_cube_id = None
                        # Check if near Red Box (X <= 0.175, Y >= 0.480)
                        if cube.x_m <= 0.175 and cube.y_m >= 0.480:
                            red_count = sum(1 for c in self.cubes if c.state == "placed" and c.x_m <= 0.175)
                            if red_count <= 1:
                                ox, oy, oz = 0.0, 0.0, 0.035
                            else:
                                ox = (((red_count - 2) % 3) - 1) * 0.022
                                oy = ((((red_count - 2) // 3) % 3) - 1) * 0.022
                                oz = 0.035 + (((red_count - 2) // 9) * 0.020)
                            cube.x_m = 0.050 + ox
                            cube.y_m = 0.550 + oy
                            cube.z_m = oz
                            self.get_logger().info(f"Deposited {cube.color.upper()} cube #{cube.id} in RED TRAY (X={cube.x_m*1000:.1f}mm, Y={cube.y_m*1000:.1f}mm).")
                        # Check if near Green Box (X > 0.175, Y >= 0.480)
                        elif cube.x_m > 0.175 and cube.y_m >= 0.480:
                            green_count = sum(1 for c in self.cubes if c.state == "placed" and c.x_m > 0.175)
                            if green_count <= 1:
                                ox, oy, oz = 0.0, 0.0, 0.035
                            else:
                                ox = (((green_count - 2) % 3) - 1) * 0.022
                                oy = ((((green_count - 2) // 3) % 3) - 1) * 0.022
                                oz = 0.035 + (((green_count - 2) // 9) * 0.020)
                            cube.x_m = 0.300 + ox
                            cube.y_m = 0.550 + oy
                            cube.z_m = oz
                            self.get_logger().info(f"Deposited {cube.color.upper()} cube #{cube.id} in GREEN TRAY (X={cube.x_m*1000:.1f}mm, Y={cube.y_m*1000:.1f}mm).")
                        else:
                            cube.z_m = 0.0137
                            self.get_logger().info(f"Released {cube.color.upper()} cube #{cube.id} on table.")
                        self.publish_detected_objects()

        # Construct MarkerArray
        markers = MarkerArray()
        cube_markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1. 5cm Grid Lines (Marker.LINE_LIST)
        grid_marker = Marker()
        grid_marker.header.frame_id = "world"
        grid_marker.header.stamp = now
        grid_marker.ns = "grid_lines"
        grid_marker.id = 1
        grid_marker.type = Marker.LINE_LIST
        grid_marker.action = Marker.ADD
        grid_marker.pose.orientation.w = 1.0
        grid_marker.scale.x = 0.0015  # Line thickness (1.5mm)
        grid_marker.color = ColorRGBA(r=0.25, g=0.55, b=0.85, a=0.65)  # Blueprint cyan

        z_grid = 0.0015
        # Vertical lines (constant X)
        for i in range(8):  # X: 0 to 350mm step 50mm
            x = i * 0.050
            grid_marker.points.append(Point(x=x, y=0.0, z=z_grid))
            grid_marker.points.append(Point(x=x, y=0.600, z=z_grid))

        # Horizontal lines (constant Y)
        for j in range(13):  # Y: 0 to 600mm step 50mm
            y = j * 0.050
            grid_marker.points.append(Point(x=0.0, y=y, z=z_grid))
            grid_marker.points.append(Point(x=0.350, y=y, z=z_grid))

        markers.markers.append(grid_marker)

        # 2. Picking Zone Boundary Outline (Marker.LINE_STRIP across Rows 3..10)
        pz_marker = Marker()
        pz_marker.header.frame_id = "world"
        pz_marker.header.stamp = now
        pz_marker.ns = "zones"
        pz_marker.id = 50
        pz_marker.type = Marker.LINE_STRIP
        pz_marker.action = Marker.ADD
        pz_marker.pose.orientation.w = 1.0
        pz_marker.scale.x = 0.003
        pz_marker.color = ColorRGBA(r=0.1, g=0.8, b=0.3, a=0.8)  # Green border
        pz_marker.points = [
            Point(x=0.000, y=0.050, z=0.003),
            Point(x=0.350, y=0.050, z=0.003),
            Point(x=0.350, y=0.450, z=0.003),
            Point(x=0.000, y=0.450, z=0.003),
            Point(x=0.000, y=0.050, z=0.003),
        ]
        markers.markers.append(pz_marker)

        # 3. xArm Base Mounting Footprint (Marker.LINE_STRIP, Rows 3..5, Cols 2..4)
        arm_base_marker = Marker()
        arm_base_marker.header.frame_id = "world"
        arm_base_marker.header.stamp = now
        arm_base_marker.ns = "zones"
        arm_base_marker.id = 55
        arm_base_marker.type = Marker.LINE_STRIP
        arm_base_marker.action = Marker.ADD
        arm_base_marker.pose.orientation.w = 1.0
        arm_base_marker.scale.x = 0.002
        arm_base_marker.color = ColorRGBA(r=0.95, g=0.55, b=0.1, a=0.85)  # Amber warning border
        arm_base_marker.points = [
            Point(x=0.100, y=0.300, z=0.003),
            Point(x=0.250, y=0.300, z=0.003),
            Point(x=0.250, y=0.450, z=0.003),
            Point(x=0.100, y=0.450, z=0.003),
            Point(x=0.100, y=0.300, z=0.003),
        ]
        markers.markers.append(arm_base_marker)

        # 4. ArUco Markers on table surface (40x40mm thin pads, DICT_4X4_50)
        aruco_pads = [
            (0, 0.025, 0.425),
            (1, 0.325, 0.425),
            (2, 0.325, 0.025),
            (3, 0.025, 0.025),
        ]
        for aid, ax, ay in aruco_pads:
            am = Marker()
            am.header.frame_id = "world"
            am.header.stamp = now
            am.ns = "aruco_markers"
            am.id = 60 + aid
            am.type = Marker.CUBE
            am.action = Marker.ADD
            am.pose.position.x = ax
            am.pose.position.y = ay
            am.pose.position.z = 0.001
            am.pose.orientation.w = 1.0
            am.scale.x = 0.040
            am.scale.y = 0.040
            am.scale.z = 0.0015
            am.color = ColorRGBA(r=0.08, g=0.08, b=0.08, a=0.95)  # Matte dark charcoal
            markers.markers.append(am)

        # 3. Live Tool Tip Table Ground Shadow / Crosshair circle
        if self.current_ee_pos:
            shadow = Marker()
            shadow.header.frame_id = "world"
            shadow.header.stamp = now
            shadow.ns = "tool_hud"
            shadow.id = 101
            shadow.type = Marker.CYLINDER
            shadow.action = Marker.ADD
            shadow.pose.position.x = self.current_ee_pos.x
            shadow.pose.position.y = self.current_ee_pos.y
            shadow.pose.position.z = 0.002
            shadow.pose.orientation.w = 1.0
            shadow.scale.x = 0.030
            shadow.scale.y = 0.030
            shadow.scale.z = 0.001
            shadow.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.65)  # Translucent amber shadow
            markers.markers.append(shadow)

        # 4. Visual Craft Cubes (3D Solid Colored Blocks)
        for cube in self.cubes:
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "craft_cubes"
            m.id = cube.id * 10
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cube.x_m
            m.pose.position.y = cube.y_m
            m.pose.position.z = cube.z_m
            m.pose.orientation.w = 1.0
            m.scale.x = 0.032
            m.scale.y = 0.032
            m.scale.z = 0.0254

            if cube.color == "red":
                m.color = ColorRGBA(r=0.95, g=0.12, b=0.12, a=1.0)
            else:
                m.color = ColorRGBA(r=0.12, g=0.88, b=0.20, a=1.0)
            markers.markers.append(m)
            cube_markers.markers.append(m)

        self.pub_markers.publish(markers)
        self.pub_cube_markers.publish(cube_markers)
        self.pub_default_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ArenaVisualizerNode()
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
