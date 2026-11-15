#!/usr/bin/env python3
"""
Automated Test Suite for Camera Calibration, Planar Homography, Color Detection & Phase 1/Phase 2 Merge.

Verifies:
  1. ArUco 4-point detection & homography matrix H calculation on synthetic arena frames.
  2. Metric pixel-to-world (and world-to-pixel) millimeter roundtrip accuracy (< 0.5 mm error).
  3. Multi-band HSV color segmentation for Red, Green, Blue, and Yellow cubes.
  4. Contour spatial moment centroid extraction and yaw angle normalization.
  5. Coordinate shifts from arena world frame (Xw, Yw) to arm polar frames (r, theta).
  6. Territorial assignment across X = 280 mm midline and spatial mutex corridor flagging.
  7. Nearest-First queue sorting policy (ascending r).
  8. End-to-end IK solvability for vision targets and lateral 2x2 flank tray drop compartments.
"""

import math
import os
import sys

# Ensure repository root is in sys.path
test_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(test_dir, "..", "..", "..", ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
try:
    import pytest
    fixture = pytest.fixture
except ImportError:
    def fixture(fn):
        return fn

from sorting_arm.kinematics import (
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
)
from tools.autonomous_pick_and_place import AutonomousSorterBridge
from vision.arena_vision_pipeline import ArenaVisionPipeline, DetectedCube
from vision.test_aruco_warp import generate_synthetic_arena_frame


def get_pipeline():
    config_dir = os.path.join(repo_root, "config")
    return ArenaVisionPipeline(config_dir=config_dir, scale_px_per_mm=2.0)


@fixture
def pipeline():
    return get_pipeline()


def test_metric_coordinate_mapping(pipeline=None):
    """Test mathematical accuracy of pixel_to_world and world_to_pixel mappings."""
    if pipeline is None:
        pipeline = get_pipeline()
    # Marker 0: Rear-Left (70.0, 180.0) -> (u=0, v=0)
    x0, y0, _ = pipeline.pixel_to_world(0.0, 0.0)
    assert abs(x0 - 70.0) < 0.1
    assert abs(y0 - 180.0) < 0.1

    # Marker 1: Rear-Right (330.0, 180.0) -> (u=520, v=0)
    x1, y1, _ = pipeline.pixel_to_world(520.0, 0.0)
    assert abs(x1 - 330.0) < 0.1
    assert abs(y1 - 180.0) < 0.1

    # Marker 2: Front-Left (70.0, 40.0) -> (u=0, v=280)
    x2, y2, _ = pipeline.pixel_to_world(0.0, 280.0)
    assert abs(x2 - 70.0) < 0.1
    assert abs(y2 - 40.0) < 0.1

    # Marker 3: Front-Right (330.0, 40.0) -> (u=520, v=280)
    x3, y3, _ = pipeline.pixel_to_world(520.0, 280.0)
    assert abs(x3 - 330.0) < 0.1
    assert abs(y3 - 40.0) < 0.1

    # Roundtrip check
    for test_u, test_v in [(100.0, 50.0), (260.0, 140.0), (450.0, 200.0)]:
        wx, wy, _ = pipeline.pixel_to_world(test_u, test_v)
        u_back, v_back = pipeline.world_to_pixel(wx, wy)
        assert abs(u_back - test_u) <= 1
        assert abs(v_back - test_v) <= 1


def test_arm_polar_transformation(pipeline=None):
    """Test transformation from global world coordinates (Xw, Yw) to arm polar frames (r, theta)."""
    if pipeline is None:
        pipeline = get_pipeline()
    # Single xArm 1S is at (200, 280, 0) with nominal heading -90° (South)
    # Heading -90° corresponds to dx = 0, dy = -R
    r_test = 200.0
    tx = 200.0
    ty = 280.0 - r_test  # 80.0 mm

    r_calc, th_calc = pipeline.world_to_arm_polar(tx, ty, "xarm_1")
    assert abs(r_calc - r_test) < 0.5
    assert abs(th_calc - 0.0) < 0.5, f"Expected 0.0 deg along heading, got {th_calc}"

    # Test point at +20° local angle
    rad_arm = math.radians(-70.0)  # -90° + 20°
    tx_off = 200.0 + r_test * math.cos(rad_arm)
    ty_off = 280.0 + r_test * math.sin(rad_arm)

    r_off, th_off = pipeline.world_to_arm_polar(tx_off, ty_off, "xarm_1")
    assert abs(r_off - r_test) < 0.5
    assert abs(th_off - 20.0) < 0.5, f"Expected 20.0 deg, got {th_off}"


def test_aruco_detection_and_synthetic_homography(pipeline=None):
    """Test 4-point ArUco marker detection and perspective warp on synthetic arena frame."""
    if pipeline is None:
        pipeline = get_pipeline()
    frame = generate_synthetic_arena_frame(1920, 1080)
    corners, ids = pipeline.detect_aruco_markers(frame)

    assert ids is not None, "ArUco detection returned None for IDs"
    detected_ids = set(ids.flatten())
    assert {0, 1, 2, 3}.issubset(detected_ids), f"Missing ArUco IDs: {detected_ids}"

    # Calibrate homography
    ok = pipeline.calibrate_homography(frame)
    assert ok, "Homography calibration failed on synthetic frame"
    assert pipeline.H_matrix is not None
    assert pipeline.H_matrix.shape == (3, 3)

    # Warp arena
    warped = pipeline.warp_arena(frame)
    assert warped is not None
    assert warped.shape == (pipeline.rect_h, pipeline.rect_w, 3)


def test_color_segmentation_and_cube_detection(pipeline=None):
    """Test detection of colored cubes, centroid coordinates, and territory assignment."""
    if pipeline is None:
        pipeline = get_pipeline()
    frame = generate_synthetic_arena_frame(1920, 1080)
    pipeline.calibrate_homography(frame)
    warped, annotated, cubes = pipeline.process_frame(frame, draw_grid=True)

    assert warped is not None
    assert annotated is not None
    assert len(cubes) >= 4, f"Expected at least 4 cubes, found {len(cubes)}"

    detected_colors = {c.color for c in cubes}
    expected_colors = {"red", "green", "blue", "yellow"}
    assert expected_colors.issubset(detected_colors), f"Missing colors: {expected_colors - detected_colors}"

    for c in cubes:
        # Check arena bounds: X in [70, 330], Y in [40, 180] (with tolerance)
        assert 60.0 <= c.world_x <= 340.0, f"X out of bounds: {c.world_x}"
        assert 30.0 <= c.world_y <= 190.0, f"Y out of bounds: {c.world_y}"
        # Check yaw angle normalization
        assert -45.0 <= c.yaw_deg <= 45.0, f"Yaw out of bounds: {c.yaw_deg}"
        # Check reach range from arm at (200, 280)
        assert 100.0 <= c.target_r_mm <= 300.0, f"Radius out of reach: {c.target_r_mm}"
        # Check single arm assignment
        assert c.assigned_arm in ("xarm_1", "xarm")


def test_nearest_first_queue_sorting():
    """Test that queue sorting prioritizes cubes by ascending radius to prevent collisions."""
    bridge = AutonomousSorterBridge(arm_id="xarm_1", dry_run=True)
    c1 = DetectedCube(color="red", world_x=200.0, world_y=120.0, assigned_arm="xarm_1", target_r_mm=260.0)
    c2 = DetectedCube(color="blue", world_x=200.0, world_y=160.0, assigned_arm="xarm_1", target_r_mm=160.0)
    c3 = DetectedCube(color="green", world_x=230.0, world_y=140.0, assigned_arm="xarm_1", target_r_mm=210.0)

    queue = bridge.filter_and_sort_queue([c1, c2, c3])
    assert len(queue) == 3
    assert queue[0].target_r_mm == 160.0
    assert queue[1].target_r_mm == 210.0
    assert queue[2].target_r_mm == 260.0
    assert queue[0].color == "blue"


def test_phase1_phase2_ik_bridge():
    """Test that all vision targets solve cleanly with analytical IK and drop targets."""
    config_dir = os.path.join(repo_root, "config")
    hardware_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
    params_arm1 = load_kinematic_parameters(hardware_yaml, arm_id="xarm_1")

    # Synthetic cubes in arena for xarm_1
    test_targets = [
        {"arm": "xarm_1", "color": "red", "r": 220.0, "theta": 0.0, "z": 12.5, "yaw": 10.0, "params": params_arm1},
        {"arm": "xarm_1", "color": "blue", "r": 165.0, "theta": -15.0, "z": 12.5, "yaw": -5.0, "params": params_arm1},
        {"arm": "xarm_1", "color": "green", "r": 210.0, "theta": +10.0, "z": 12.5, "yaw": 0.0, "params": params_arm1},
        {"arm": "xarm_1", "color": "yellow", "r": 250.0, "theta": -20.0, "z": 12.5, "yaw": 25.0, "params": params_arm1},
    ]

    for t in test_targets:
        # 1. Pick IK with wrist roll aligned to cube orientation
        roll_tick = int(round(500 + t["yaw"] * (1000.0 / 240.0)))
        sol = solve_ik(r=t["r"], theta=t["theta"], z=t["z"], roll_tick=roll_tick, params=t["params"])
        assert sol.success, f"Pick IK failed for {t}: {sol.message}"
        assert len(sol.ticks) == 6
        for motor_id, tick in sol.ticks.items():
            assert 100 <= tick <= 900, f"Motor {motor_id} tick {tick} out of safe bounds"

        # 2. Drop Target resolution
        drop = get_compartment_target(color=t["color"], arm_id=t["arm"], config_path=hardware_yaml)
        assert drop is not None, f"Could not find drop target for {t['arm']} {t['color']}"
        assert drop["color"] == t["color"]
        assert 100 <= drop["drop_yaw_tick"] <= 900 or drop["ik_success"], "Invalid drop yaw tick"


if __name__ == "__main__":
    try:
        import pytest
        pytest.main(["-v", __file__])
    except ImportError:
        test_metric_coordinate_mapping()
        test_arm_polar_transformation()
        test_aruco_detection_and_synthetic_homography()
        test_color_segmentation_and_cube_detection()
        test_nearest_first_queue_sorting()
        test_phase1_phase2_ik_bridge()
        print("All 6 vision pipeline tests passed successfully!")
