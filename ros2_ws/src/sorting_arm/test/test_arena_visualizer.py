#!/usr/bin/env python3
"""
Unit and integration tests for ArenaVisualizerNode (5cm Grid, HUD, Interactive Cubes).
"""

import json
import math
import time

from geometry_msgs.msg import PointStamped
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from sorting_arm.arena_visualizer_node import ArenaVisualizerNode
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


@pytest.fixture(scope="module")
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_arena_visualizer_grid_and_labels(rclpy_context):
    """Verify that arena visualizer generates 5cm grid lines and coordinate labels."""
    node = ArenaVisualizerNode()
    received_markers = []

    def marker_cb(msg: MarkerArray):
        received_markers.append(msg)

    node.create_subscription(MarkerArray, "/simulation/arena_markers", marker_cb, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Let timer fire
    start = time.time()
    while time.time() - start < 0.3:
        executor.spin_once(timeout_sec=0.05)

    assert len(received_markers) >= 1, "Failed to receive arena markers"
    latest = received_markers[-1]

    # Find grid lines
    grid_markers = [m for m in latest.markers if m.ns == "grid_lines"]
    assert len(grid_markers) == 1
    # 8 vertical lines (16 pts) + 13 horizontal lines (26 pts) = 42 points
    assert len(grid_markers[0].points) == 42
    assert grid_markers[0].type == Marker.LINE_LIST

    # Find column and row labels
    col_labels = [m for m in latest.markers if m.ns == "col_labels"]
    row_labels = [m for m in latest.markers if m.ns == "row_labels"]
    assert len(col_labels) == 7
    assert len(row_labels) == 12

    node.destroy_node()


def test_arena_visualizer_cube_spawning_and_vision_sync(rclpy_context):
    """Verify that clicking in RViz spawns cubes and syncs with /vision/detected_objects."""
    node = ArenaVisualizerNode()
    received_vision = []

    def vision_cb(msg: String):
        received_vision.append(msg)

    node.create_subscription(String, "/vision/detected_objects", vision_cb, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Simulate RViz Publish Point click at Col 2, Row 8 (X=130mm, Y=200mm)
    pt = PointStamped()
    pt.header.frame_id = "world"
    pt.point.x = 0.125
    pt.point.y = 0.225
    pt.point.z = 0.0127
    node.handle_clicked_point(pt)

    # Spin to publish
    start = time.time()
    while time.time() - start < 0.2:
        executor.spin_once(timeout_sec=0.05)

    assert len(received_vision) >= 1
    data = json.loads(received_vision[-1].data)
    assert data["source"] == "arena_visualizer"
    assert data["total_detected"] >= 1

    # Check that cube was snapped to cell center (Col 2 -> X=125mm, Row 8 -> Y=175mm)
    added_cubes = [
        c for c in node.cubes
        if c.initial_col == 2 and (c.initial_row == 7 or c.initial_row == 8)
    ]
    assert len(added_cubes) >= 1

    node.destroy_node()


def test_arena_visualizer_grasp_and_deposit(rclpy_context):
    """Verify cube transitions from in_cell to grabbed to placed upon claw actions."""
    node = ArenaVisualizerNode()
    node.cubes.clear()

    # Place a single red cube at (0.130, 0.200)
    cube = node._add_cube("red", 0.130, 0.200)
    assert cube.state == "in_cell"

    # Simulate tool_tip hovering right above cube
    from geometry_msgs.msg import Point
    node.current_ee_pos = Point(x=0.130, y=0.200, z=0.015)

    # Send claw open joint state (230 ticks)
    open_state = JointState()
    open_state.name = ["claw_gripper"]
    open_state.position = [math.radians((230.0 - 517.0) * (240.0 / 1000.0))]
    node.handle_joint_states(open_state)
    assert not node.claw_is_clamped

    node.update_and_publish_markers()
    assert cube.state == "in_cell"

    # Send claw clamped joint state (610 ticks)
    clamped_state = JointState()
    clamped_state.name = ["claw_gripper"]
    clamped_state.position = [math.radians((610.0 - 517.0) * (240.0 / 1000.0))]
    node.handle_joint_states(clamped_state)
    assert node.claw_is_clamped

    node.update_and_publish_markers()
    assert cube.state == "grabbed"

    # Move tool_tip to Red Container Box (0.050, 0.550, 0.050)
    node.current_ee_pos = Point(x=0.050, y=0.550, z=0.050)
    node.update_and_publish_markers()
    assert cube.state == "grabbed"
    assert cube.x_m == 0.050
    assert cube.y_m == 0.550

    # Release claw
    node.handle_joint_states(open_state)
    assert not node.claw_is_clamped
    node.update_and_publish_markers()
    assert cube.state == "placed"
    assert cube.x_m == 0.050
    assert cube.y_m == 0.550
    assert cube.z_m == 0.035

    node.destroy_node()
