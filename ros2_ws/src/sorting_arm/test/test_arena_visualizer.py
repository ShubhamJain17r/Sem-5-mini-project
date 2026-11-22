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


def test_arena_visualizer_grid_and_zero_text(rclpy_context):
    """Verify that arena visualizer generates 5cm grid lines, zone borders, and ZERO text markers."""
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

    # Verify zero TEXT_VIEW_FACING markers exist (clean, high-performance rendering)
    text_markers = [m for m in latest.markers if m.type == Marker.TEXT_VIEW_FACING]
    assert len(text_markers) == 0, f"Expected 0 text markers, found {len(text_markers)}"

    # Verify picking zone boundary and arm base mounting footprint exist
    zone_markers = [m for m in latest.markers if m.ns == "zones" and m.type == Marker.LINE_STRIP]
    assert len(zone_markers) == 2, f"Expected 2 zone outlines (picking zone + arm base), found {len(zone_markers)}"

    # Verify 4 physical ArUco markers rendered on table
    aruco_markers = [m for m in latest.markers if m.ns == "aruco_markers"]
    assert len(aruco_markers) == 4, f"Expected 4 ArUco markers, found {len(aruco_markers)}"

    node.destroy_node()


def test_arena_visualizer_hard_zones(rclpy_context):
    """Verify that hard zones mode spawns cubes at near-base and far-corner stress coordinates."""
    from std_srvs.srv import Trigger

    node = ArenaVisualizerNode()
    node.cubes.clear()

    # Trigger hard zones spawn
    req = Trigger.Request()
    resp = Trigger.Response()
    node.handle_spawn_hard_zones_srv(req, resp)
    assert resp.success
    assert len(node.cubes) == 5

    # Check near-base coordinates (Row 6 Col 1, Row 6 Col 5, Row 7 Col 3)
    near_base = [c for c in node.cubes if c.initial_row in (6, 7)]
    assert len(near_base) == 3

    # Check far-corner coordinates (Row 10 Col 0, Row 10 Col 6)
    far_corners = [c for c in node.cubes if c.initial_row == 10]
    assert len(far_corners) == 2

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
    while time.time() - start < 1.0:
        executor.spin_once(timeout_sec=0.05)
        if received_vision:
            break

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


def test_arena_visualizer_full_grid_and_reseed(rclpy_context):
    """Verify that full_grid mode populates 45 cells with red/green cubes and reseed works."""
    from std_srvs.srv import Trigger

    node = ArenaVisualizerNode()

    # Initial full grid should have exactly 45 cubes across Rows 3..10 and Cols 0..6
    assert len(node.cubes) == 45
    active_cubes = [c for c in node.cubes if c.state == "in_cell"]
    assert len(active_cubes) == 45

    # Check colors: only red and green
    reds = [c for c in node.cubes if c.color == "red"]
    greens = [c for c in node.cubes if c.color == "green"]
    assert len(reds) + len(greens) == 45
    assert len(reds) > 0
    assert len(greens) > 0

    # Check that NO cube is placed on ArUco markers (Row 3 Cols 0 and 6)
    for c in node.cubes:
        assert not (c.initial_col in (0, 6) and c.initial_row == 3), f"Cube #{c.id} placed on ArUco marker at Col {c.initial_col}, Row {c.initial_row}"

    # Check that NO cube is placed on xArm mounting footprint (Cols 2..4 in Rows 3..5)
    for c in node.cubes:
        assert not (c.initial_col in (2, 3, 4) and c.initial_row in (3, 4, 5)), f"Cube #{c.id} placed on xArm mount at Col {c.initial_col}, Row {c.initial_row}"

    # Check coordinate bounds: Cols 0..6 (0.025 to 0.325m), Rows 3..10 (0.075 to 0.425m)
    for c in node.cubes:
        assert 0.020 <= c.x_m <= 0.330
        assert 0.070 <= c.y_m <= 0.430
        assert c.initial_row in range(3, 11)
        assert 0 <= c.initial_col <= 6

    # Verify cubes exist in upper rows (3..6) as well as lower rows (7..10)
    rows_present = set(c.initial_row for c in node.cubes)
    assert set(range(3, 11)) == rows_present

    # Test reseed service
    req = Trigger.Request()
    resp = Trigger.Response()
    node.handle_reseed_grid_srv(req, resp)
    assert resp.success
    assert "Reseeded 45 grid cells" in resp.message
    assert len(node.cubes) == 45

    node.destroy_node()

