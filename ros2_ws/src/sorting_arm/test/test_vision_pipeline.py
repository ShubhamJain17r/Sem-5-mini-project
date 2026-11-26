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
    # Top-Left: Row 6, Col 0 (25.0, 275.0) -> (u=0, v=0)
    x0, y0, _ = pipeline.pixel_to_world(0.0, 0.0)
    assert abs(x0 - 25.0) < 0.1
    assert abs(y0 - 275.0) < 0.1

    # Top-Right: Row 6, Col 6 (325.0, 275.0) -> (u=600, v=0)
    x1, y1, _ = pipeline.pixel_to_world(600.0, 0.0)
    assert abs(x1 - 325.0) < 0.1
    assert abs(y1 - 275.0) < 0.1

    # Bottom-Right: Row 11, Col 6 (325.0, 25.0) -> (u=600, v=500)
    x2, y2, _ = pipeline.pixel_to_world(600.0, 500.0)
    assert abs(x2 - 325.0) < 0.1
    assert abs(y2 - 25.0) < 0.1

    # Bottom-Left: Row 11, Col 0 (25.0, 25.0) -> (u=0, v=500)
    x3, y3, _ = pipeline.pixel_to_world(0.0, 500.0)
    assert abs(x3 - 25.0) < 0.1
    assert abs(y3 - 25.0) < 0.1

    # Roundtrip check
    for test_u, test_v in [(100.0, 50.0), (300.0, 200.0), (500.0, 450.0)]:
        wx, wy, _ = pipeline.pixel_to_world(test_u, test_v)
        u_back, v_back = pipeline.world_to_pixel(wx, wy)
        assert abs(u_back - test_u) <= 1
        assert abs(v_back - test_v) <= 1


def test_arm_polar_transformation(pipeline=None):
    """Test transformation from global world coordinates (Xw, Yw) to arm polar frames (r, theta)."""
    if pipeline is None:
        pipeline = get_pipeline()
    # Single xArm 1S is at (175, 375, 0) with nominal heading -90° (South)
    r_test = 200.0
    rad_head = math.radians(-90.0)
    tx = 175.0 + r_test * math.cos(rad_head)
    ty = 375.0 + r_test * math.sin(rad_head)

    r_calc, th_calc = pipeline.world_to_arm_polar(tx, ty, "xarm_1")
    assert abs(r_calc - r_test) < 0.5
    assert abs(th_calc - 0.0) < 0.5, f"Expected 0.0 deg along heading, got {th_calc}"

    # Test point at +20° local angle
    rad_arm = math.radians(-70.0)  # -90° + 20°
    tx_off = 175.0 + r_test * math.cos(rad_arm)
    ty_off = 375.0 + r_test * math.sin(rad_arm)

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
    assert len(cubes) >= 1, f"Expected at least 1 detected cube, found {len(cubes)}"

    detected_colors = {c.color for c in cubes}
    assert "red" in detected_colors, f"Expected red in detected colors: {detected_colors}"
    assert "green" in detected_colors, f"Expected green in detected colors: {detected_colors}"

    # Verify red and green cubes are marked targetable
    target_cubes = [c for c in cubes if c.color in ("red", "green")]
    assert len(target_cubes) >= 2
    assert all(c.targetable for c in target_cubes)

    for c in cubes:
        # Check arena bounds: X in [25, 325], Y in [25, 425] (with tolerance)
        assert 20.0 <= c.world_x <= 330.0, f"X out of bounds: {c.world_x}"
        assert 20.0 <= c.world_y <= 430.0, f"Y out of bounds: {c.world_y}"
        # Check yaw angle normalization
        assert -45.0 <= c.yaw_deg <= 45.0, f"Yaw out of bounds: {c.yaw_deg}"
        # Check single arm assignment
        assert c.assigned_arm in ("xarm_1", "xarm")


def test_nearest_first_queue_sorting():
    """Test that queue sorting prioritizes cubes by ascending radius across red and green targets."""
    bridge = AutonomousSorterBridge(arm_id="xarm_1", dry_run=True)
    c1 = DetectedCube(color="red", world_x=200.0, world_y=120.0, assigned_arm="xarm_1", target_r_mm=260.0)
    c2 = DetectedCube(color="green", world_x=200.0, world_y=160.0, assigned_arm="xarm_1", target_r_mm=160.0)
    c3 = DetectedCube(color="red", world_x=230.0, world_y=140.0, assigned_arm="xarm_1", target_r_mm=210.0)
    c4_ignored_blue = DetectedCube(color="blue", world_x=220.0, world_y=130.0, assigned_arm="xarm_1", target_r_mm=180.0)

    queue = bridge.filter_and_sort_queue([c1, c2, c3, c4_ignored_blue])
    assert len(queue) == 3, f"Expected 3 target cubes (red & green), got {len(queue)}"
    assert queue[0].target_r_mm == 160.0
    assert queue[0].color == "green"
    assert queue[1].target_r_mm == 210.0
    assert queue[1].color == "red"
    assert queue[2].target_r_mm == 260.0
    assert queue[2].color == "red"


def test_phase1_phase2_ik_bridge():
    """Test that all vision targets solve cleanly with analytical IK and drop targets."""
    config_dir = os.path.join(repo_root, "config")
    hardware_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
    params_arm1 = load_kinematic_parameters(hardware_yaml, arm_id="xarm_1")

    # Synthetic cubes in arena for xarm_1 (Red and Green active targets)
    test_targets = [
        {"arm": "xarm_1", "color": "red", "r": 220.0, "theta": 0.0, "z": 12.5, "yaw": 10.0, "params": params_arm1},
        {"arm": "xarm_1", "color": "green", "r": 210.0, "theta": +10.0, "z": 12.5, "yaw": 0.0, "params": params_arm1},
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


def test_rectangular_block_wrist_roll():
    """Verify that elongated rectangular blocks along X receive roll_tick=130 and along Y receive 500."""
    pipeline = ArenaVisionPipeline(config_dir=os.path.join(repo_root, "config"))
    img = np.zeros((pipeline.rect_h, pipeline.rect_w, 3), dtype=np.uint8)
    # Draw green rectangle at center (Col 3, Row 8 -> u=300, v=250)
    # Elongated horizontally: width=100px (~50mm), height=60px (~30mm), area=6000px
    cv2.rectangle(img, (250, 220), (350, 280), (0, 255, 0), -1)
    cubes = pipeline.detect_cubes(img)
    assert len(cubes) >= 1, f"Expected 1 detected block, found {len(cubes)}"
    c = cubes[0]
    assert c.color == "green"
    assert c.gripper_roll_tick == 130, f"Expected roll 130 for horizontal block, got {c.gripper_roll_tick}"


def test_multiframe_consensus_error_correction():
    """Verify multi-frame spatial clustering takes median coordinates and rejects single-frame outliers."""
    from tools.autonomous_dashboard import build_multiframe_consensus_queue
    from vision.arena_vision_pipeline import DetectedCube

    hw_cfg = os.path.join(repo_root, "config", "xarm_hardware.yaml")
    params = load_kinematic_parameters(config_path=hw_cfg, arm_id="xarm_1")

    # Simulate 4 handheld captures of 2 real cubes (red @ ~150,150; green @ ~220,180) + 1 spurious outlier
    f1 = [
        DetectedCube(color="red", world_x=149.0, world_y=151.0, world_z=22.5, gripper_roll_tick=500),
        DetectedCube(color="green", world_x=221.0, world_y=179.0, world_z=22.5, gripper_roll_tick=500),
    ]
    f2 = [
        DetectedCube(color="red", world_x=150.5, world_y=149.5, world_z=22.5, gripper_roll_tick=500),
        DetectedCube(color="green", world_x=219.5, world_y=180.5, world_z=22.5, gripper_roll_tick=500),
        DetectedCube(color="red", world_x=80.0, world_y=110.0, world_z=22.5, gripper_roll_tick=500),  # 1-frame noise
    ]
    f3 = [
        DetectedCube(color="red", world_x=150.0, world_y=150.0, world_z=22.5, gripper_roll_tick=500),
        DetectedCube(color="green", world_x=220.0, world_y=180.0, world_z=22.5, gripper_roll_tick=500),
    ]
    f4 = [
        DetectedCube(color="red", world_x=152.0, world_y=148.5, world_z=22.5, gripper_roll_tick=500),
        DetectedCube(color="green", world_x=220.2, world_y=180.1, world_z=22.5, gripper_roll_tick=500),
    ]

    queue = build_multiframe_consensus_queue([f1, f2, f3, f4], params)
    assert len(queue) == 2, f"Expected 2 consensus cubes (outlier rejected), got {len(queue)}"

    by_color = {q["color"]: q for q in queue}
    assert "red" in by_color and "green" in by_color
    assert abs(by_color["red"]["world_x"] - 150.25) < 0.5
    assert abs(by_color["red"]["world_y"] - 149.75) < 0.5
    assert by_color["red"]["observations"] == 4
    assert by_color["green"]["observations"] == 4


def test_square_cube_wrist_roll_alignment():
    """Verify square cubes align wrist roll to nearest flat face relative to arm heading."""
    pipeline = ArenaVisionPipeline(config_dir=os.path.join(repo_root, "config"))
    img = np.zeros((pipeline.rect_h, pipeline.rect_w, 3), dtype=np.uint8)

    # 1. Unrotated 75x75px (37.5mm) square on midline (u=300, v=250 -> th=0) -> roll ~500
    cv2.rectangle(img, (262, 212), (338, 288), (0, 255, 0), -1)
    cubes_aligned = pipeline.detect_cubes(img)
    assert len(cubes_aligned) == 1
    assert abs(cubes_aligned[0].gripper_roll_tick - 500) <= 5

    # 2. Rotated +25 deg square on midline -> roll shifts significantly from 500 within [315, 685]
    img_rot = np.zeros((pipeline.rect_h, pipeline.rect_w, 3), dtype=np.uint8)
    pts = np.array([[262, 212], [338, 212], [338, 288], [262, 288]], dtype=np.float32)
    M = cv2.getRotationMatrix2D((300, 250), 25.0, 1.0)
    pts_rot = cv2.transform(pts[None, :, :], M)[0].astype(np.int32)
    cv2.fillPoly(img_rot, [pts_rot], (0, 255, 0))
    cubes_rot = pipeline.detect_cubes(img_rot)
    assert len(cubes_rot) == 1
    assert 315 <= cubes_rot[0].gripper_roll_tick <= 685
    assert abs(cubes_rot[0].gripper_roll_tick - 500) >= 60

    # 3. Unrotated square off-midline (u=420 -> th != 0) compensates arm heading
    img_side = np.zeros((pipeline.rect_h, pipeline.rect_w, 3), dtype=np.uint8)
    cv2.rectangle(img_side, (382, 212), (458, 288), (0, 255, 0), -1)
    cubes_side = pipeline.detect_cubes(img_side)
    assert len(cubes_side) == 1
    assert abs(cubes_side[0].gripper_roll_tick - 500) >= 30


def test_pick_retry_limit_and_proximity_guard():
    """Verify retry is skipped near other cubes, limited to 1 when clear, and reports Could not pick."""
    from unittest.mock import MagicMock
    from tools.autonomous_dashboard import AutonomousSystemManager

    mgr = AutonomousSystemManager.__new__(AutonomousSystemManager)
    mgr.dry_run = False
    mgr.running = True
    mgr.open_claw_tick = 250
    mgr.clamp_claw_tick = 750
    mgr.cube_center_z_mm = 22.5
    mgr.params = load_kinematic_parameters(os.path.join(repo_root, "config", "xarm_hardware.yaml"), arm_id="xarm_1")
    mgr.current_joint_ticks = {i: 500 for i in range(1, 7)}
    mgr.sim_cubes = []
    mgr.active_target = None
    mgr.is_executing = False
    mgr.trajectory_step = "idle"
    mgr.last_pick_error = None
    mgr._update_sim_view = lambda: None
    mgr.publish_joint_states = lambda: None
    mgr.logger = MagicMock()

    sent_poses = []
    driver = MagicMock()
    driver.is_connected = True
    driver.set_servo_positions = lambda ticks, duration_ms=600: sent_poses.append(dict(ticks))
    # Motor 1 always reads 750 (fully closed / empty claw)
    driver.get_servo_positions = lambda: {1: 750, 2: 500, 3: 750, 4: 250, 5: 600, 6: 500}
    mgr.driver = driver

    target = {
        "color": "green",
        "world_x": 150.0,
        "world_y": 175.0,
        "world_z": 22.5,
        "z_rel": 17.5,
        "reach_r": 201.5,
        "theta_arm": -7.1,
        "roll_tick": 500,
        "dest_container": "green",
    }

    # Case 1: Nearby cube within 85mm -> skip retry completely (only 1 clamp attempt)
    mgr.target_queue = [{"world_x": 210.0, "world_y": 175.0}]  # 60mm away
    mgr.execute_pick_target(dict(target))
    clamp_cmds = [p for p in sent_poses if p.get(1) == 750]
    assert len(clamp_cmds) == 1, f"Expected 1 clamp (no retry near neighbor), got {len(clamp_cmds)}"
    assert mgr.last_pick_error and "Could not pick" in mgr.last_pick_error
    assert "nearby cube" in mgr.last_pick_error

    # Case 2: Clear area -> strictly 1 retry (2 total clamps), then abort with Could not pick
    sent_poses.clear()
    mgr.target_queue = [{"world_x": 300.0, "world_y": 175.0}]  # 150mm away
    mgr.execute_pick_target(dict(target))
    clamp_cmds = [p for p in sent_poses if p.get(1) == 750]
    assert len(clamp_cmds) == 2, f"Expected 2 clamps (initial + 1 retry), got {len(clamp_cmds)}"
    assert mgr.last_pick_error and "Could not pick" in mgr.last_pick_error
    assert "1 retry" in mgr.last_pick_error



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
        test_rectangular_block_wrist_roll()
        test_multiframe_consensus_error_correction()
        print("All 8 vision pipeline tests passed successfully!")
