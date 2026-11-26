#!/usr/bin/env python3
"""
Unit and Integration Tests for the Simulation and Digital Twin Pipeline.

Verifies:
  1. Mock Driver bringup and 20 Hz JointState publishing.
  2. Trajectory interpolation, speed clamping, and target arrival.
  3. Soft joint limit enforcement.
  4. Emergency stop and recovery service handling.
  5. Mock vision perception scenarios.
  6. End-to-end 7-phase pick-and-place overhead rear-flip simulation.
"""

import json
import math
import os
import sys
import time
import pytest

# Ensure repository root and sorting_arm package are in sys.path
test_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(test_dir, "..", "..", "..", ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from sorting_arm.driver_node import STANDARD_JOINT_NAMES, XArmDriverNode
    from sorting_arm.kinematics import (
        DEG_PER_TICK,
        get_compartment_target,
        load_kinematic_parameters,
        solve_ik,
    )
    from sorting_arm.mock_vision_node import MockVisionNode
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    HAS_RCLPY = True
except ImportError as _err:
    HAS_RCLPY = False
    _IMPORT_ERR = str(_err)

if not HAS_RCLPY:
    pytestmark = pytest.mark.skip(
        reason=f"Requires ROS 2 Humble (rclpy): {_IMPORT_ERR}. Run inside Docker container (./docker/run.sh)."
    )


@pytest.fixture(scope="module")
def rclpy_context():
    """Initialize rclpy context once for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_mock_driver_bringup_and_feedback(rclpy_context):
    """Verify mock driver launches cleanly and publishes valid nominal joint states."""
    node = XArmDriverNode()
    node.sim_mode = True
    node.sim_current_ticks = {1: 230.0, 2: 500.0, 3: 500.0, 4: 500.0, 5: 500.0, 6: 500.0}
    node.sim_target_ticks = dict(node.sim_current_ticks)
    node.sim_step_rates = {i: 0.0 for i in range(1, 7)}
    node.last_sim_time = node.get_clock().now()

    received_states = []

    def state_cb(msg: JointState):
        received_states.append(msg)

    node.create_subscription(JointState, "joint_states", state_cb, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Spin for 150ms to collect state publishes
    start = time.time()
    while time.time() - start < 0.2:
        executor.spin_once(timeout_sec=0.05)

    assert len(received_states) >= 1, "Mock driver failed to publish joint states"
    latest = received_states[-1]
    assert len(latest.name) == 6
    assert latest.name == STANDARD_JOINT_NAMES

    # Claw should be at nominal open
    center_1 = float(node.center_ticks.get(1, 517))
    expected_claw_rad = math.radians((230.0 - center_1) * DEG_PER_TICK)
    assert abs(latest.position[0] - expected_claw_rad) < 0.01

    # Base yaw should be at center
    center_6 = float(node.center_ticks.get(6, 502))
    expected_base_rad = math.radians((500.0 - center_6) * DEG_PER_TICK)
    assert abs(latest.position[5] - expected_base_rad) < 0.01

    node.destroy_node()


def test_simulation_joint_command_interpolation(rclpy_context):
    """Verify mock driver smoothly steps simulated servos toward commanded targets."""
    node = XArmDriverNode()
    node.sim_mode = True
    node.sim_current_ticks = {1: 230.0, 2: 500.0, 3: 500.0, 4: 500.0, 5: 500.0, 6: 500.0}
    node.sim_target_ticks = dict(node.sim_current_ticks)
    node.sim_step_rates = {i: 0.0 for i in range(1, 7)}
    node.last_sim_time = node.get_clock().now()

    # Send command: move base_yaw (Motor 6) from 500 to 600 ticks over 100ms
    cmd = JointState()
    cmd.name = ["base_yaw"]
    center_6 = float(node.center_ticks.get(6, 502))
    target_yaw_rad = math.radians((600.0 - center_6) * DEG_PER_TICK)
    cmd.position = [target_yaw_rad]

    node.joint_command_callback(cmd)
    assert node.sim_target_ticks[6] == 600.0
    assert node.sim_step_rates[6] > 0.0

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Let simulation step for 250ms (duration was 100ms minimum)
    start = time.time()
    while time.time() - start < 0.35:
        executor.spin_once(timeout_sec=0.05)

    # Motor 6 should have arrived at target 600 ticks
    assert abs(node.sim_current_ticks[6] - 600.0) < 1.0, (
        f"Simulated servo did not reach target 600 ticks (actual: {node.sim_current_ticks[6]})"
    )

    node.destroy_node()


def test_simulation_limits_clamping(rclpy_context):
    """Verify mock driver clamps excessive tick commands to configured soft limits."""
    node = XArmDriverNode()
    node.sim_mode = True
    node.sim_current_ticks = {1: 230.0, 2: 500.0, 3: 500.0, 4: 500.0, 5: 500.0, 6: 500.0}
    node.sim_target_ticks = dict(node.sim_current_ticks)
    node.sim_step_rates = {i: 0.0 for i in range(1, 7)}
    node.last_sim_time = node.get_clock().now()

    # Motor 6 max tick is 850. Send command with 980 ticks (> 850)
    cmd = JointState()
    cmd.name = ["base_yaw"]
    cmd.position = [float(980.0)]  # Pass raw ticks directly to test raw-tick compatibility
    node.joint_command_callback(cmd)

    max_t = node.max_ticks.get(6, 850)
    assert node.sim_target_ticks[6] == float(max_t), (
        f"Target tick was not clamped to max limit {max_t}"
    )

    node.destroy_node()


def test_simulation_estop_and_recovery(rclpy_context):
    """Verify E-Stop halts simulation motion immediately and re-enables on reset."""
    node = XArmDriverNode()
    node.sim_mode = True
    node.sim_current_ticks = {1: 230.0, 2: 500.0, 3: 500.0, 4: 500.0, 5: 500.0, 6: 500.0}
    node.sim_target_ticks = dict(node.sim_current_ticks)
    node.sim_step_rates = {i: 0.0 for i in range(1, 7)}
    node.last_sim_time = node.get_clock().now()

    # Trigger E-Stop
    req = Trigger.Request()
    res = Trigger.Response()
    node.estop_callback(req, res)
    assert res.success is True
    assert node.sim_is_estopped is True

    # Attempt to send command while estopped
    cmd = JointState()
    cmd.name = ["base_yaw"]
    center_6 = float(node.center_ticks.get(6, 502))
    cmd.position = [math.radians((700.0 - center_6) * DEG_PER_TICK)]
    node.joint_command_callback(cmd)
    assert node.sim_target_ticks[6] == 500.0, "Command should have been dropped while estopped"

    # Re-enable
    node.re_enable_callback(req, res)
    assert res.success is True
    assert node.sim_is_estopped is False

    # Command now accepted
    node.joint_command_callback(cmd)
    assert node.sim_target_ticks[6] == 700.0

    node.destroy_node()


def test_mock_vision_node_scenarios(rclpy_context):
    """Verify mock vision publishes valid detected objects telemetry with polar coordinates."""
    v_node = MockVisionNode()
    v_node.scenario = "dual_cube"

    received_msgs = []

    def vision_cb(msg: String):
        received_msgs.append(msg)

    v_node.create_subscription(String, "/vision/detected_objects", vision_cb, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(v_node)

    v_node.publish_scenario()

    start = time.time()
    while time.time() - start < 0.2:
        executor.spin_once(timeout_sec=0.05)

    assert len(received_msgs) >= 1
    data = json.loads(received_msgs[-1].data)
    assert "objects" in data
    assert len(data["objects"]) == 2

    colors = {o["color"] for o in data["objects"]}
    assert "red" in colors
    assert "green" in colors

    for obj in data["objects"]:
        assert 120.0 <= obj["target_r_mm"] <= 300.0, "Target distance outside picking arena"
        assert obj["targetable"] is True

    v_node.destroy_node()


def test_end_to_end_trajectory_simulation(rclpy_context):
    """Verify full 7-phase pick-and-place overhead rear-flip trajectory against mock driver."""
    params = load_kinematic_parameters("config/xarm_hardware.yaml", arm_id="xarm_1")

    # Pick target: Red craft cube at r = 210mm, theta = -5°, Z = 9.53mm
    pick_r, pick_theta, pick_z = 210.0, -5.0, 9.53
    pick_sol = solve_ik(r=pick_r, theta=pick_theta, z=pick_z, params=params)
    assert pick_sol.success, f"Pick IK failed: {pick_sol.message}"

    # Drop target for Red cube (Rear Flank Red container box)
    red_target = get_compartment_target(
        "red", arm_id="xarm_1", config_path="config/xarm_hardware.yaml"
    )
    assert red_target is not None
    assert red_target["drop_yaw_tick"] == 650

    # Start mock driver
    driver = XArmDriverNode()
    driver.sim_mode = True
    driver.sim_current_ticks = {1: 230.0, 2: 500.0, 3: 500.0, 4: 500.0, 5: 500.0, 6: 500.0}
    driver.sim_target_ticks = dict(driver.sim_current_ticks)
    driver.sim_step_rates = {i: 0.0 for i in range(1, 7)}
    driver.last_sim_time = driver.get_clock().now()

    executor = SingleThreadedExecutor()
    executor.add_node(driver)

    centers = driver.center_ticks

    def dispatch_and_wait(ticks, move_time_ms=100, timeout_sec=1.5):
        cmd = JointState()
        cmd.name = STANDARD_JOINT_NAMES
        pos_rads = [
            math.radians(float(ticks.get(i, centers.get(i, 500)) - centers.get(i, 500)) * DEG_PER_TICK)
            for i in range(1, 7)
        ]
        cmd.position = pos_rads
        driver.joint_command_callback(cmd)
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            executor.spin_once(timeout_sec=0.02)
            all_arrived = all(
                abs(driver.sim_current_ticks[sid] - driver.sim_target_ticks[sid]) < 0.5
                for sid in range(1, 7)
            )
            if all_arrived:
                break

    # Phase 1: Standoff
    standoff = solve_ik(r=pick_r - 35.0, theta=pick_theta, z=pick_z + 25.0, params=params)
    assert standoff.success
    dispatch_and_wait(standoff.ticks, move_time_ms=70)

    # Phase 2: Pick grasp
    dispatch_and_wait(pick_sol.ticks, move_time_ms=70)
    # Check that driver base yaw matches pick theta
    expected_m6 = pick_sol.ticks[6]
    assert abs(driver.sim_current_ticks[6] - expected_m6) < 2.0

    # Phase 3: Clamp
    clamped_ticks = dict(pick_sol.ticks)
    clamped_ticks[1] = 610
    dispatch_and_wait(clamped_ticks, move_time_ms=50)
    assert abs(driver.sim_current_ticks[1] - 610.0) < 2.0

    # Phase 4: Lift
    lift_ticks = dict(standoff.ticks)
    lift_ticks[1] = 610
    dispatch_and_wait(lift_ticks, move_time_ms=70)

    # Phase 5: Overhead Rear-Flip Transfer to Red container box
    drop_ticks = red_target["ik_ticks"]
    drop_ticks[1] = 610
    dispatch_and_wait(drop_ticks, move_time_ms=80)
    # Red container rear-flip base yaw is 650 ticks
    assert abs(driver.sim_current_ticks[6] - 650.0) < 2.0

    # Phase 6: Release
    drop_ticks[1] = 250
    dispatch_and_wait(drop_ticks, move_time_ms=50)
    assert abs(driver.sim_current_ticks[1] - 250.0) < 2.0

    # Phase 7: Home
    home_ticks = {i: 500 for i in range(1, 7)}
    home_ticks[1] = 250
    dispatch_and_wait(home_ticks, move_time_ms=70)
    for sid in range(2, 7):
        assert abs(driver.sim_current_ticks[sid] - 500.0) < 2.0

    driver.destroy_node()


def test_autonomous_sorter_no_duplicate_picks(rclpy_context):
    """Verify that autonomous sorter never picks the same cell coordinate twice even with stale vision messages."""
    from sorting_arm.autonomous_sorter_node import AutonomousSorterNode
    node = AutonomousSorterNode()
    node.dry_run = True  # Dry run mode skips waiting for real servo motions

    # Simulate vision sending 2 objects at (125mm, 225mm) and (225mm, 225mm)
    msg1 = String()
    msg1.data = json.dumps({
        "source": "vision",
        "objects": [
            {"color": "red", "world_x_mm": 125.0, "world_y_mm": 225.0, "world_z_mm": 13.7, "target_r_mm": 158.1, "target_theta_deg": -18.4, "targetable": True, "assigned_arm": "xarm_1"},
            {"color": "green", "world_x_mm": 225.0, "world_y_mm": 225.0, "world_z_mm": 13.7, "target_r_mm": 158.1, "target_theta_deg": 18.4, "targetable": True, "assigned_arm": "xarm_1"},
        ]
    })
    node.vision_callback(msg1)
    assert len(node.target_queue) == 2

    # Enable and run first executive iteration
    node.enabled = True
    node.executive_loop()

    # Cell (2, 4) should now be in sorted_cells
    assert (2, 4) in node.sorted_cells
    assert node.cubes_sorted_in_batch == 1

    # Simulate arrival of a STALE vision message that still contains both objects
    node.vision_callback(msg1)

    # Only the green object at (4, 4) should remain in queue; red at (2, 4) is filtered out
    assert len(node.target_queue) == 1
    assert node.target_queue[0]["color"] == "green"

    # Run second executive iteration
    node.executive_loop()
    assert (4, 4) in node.sorted_cells
    assert node.cubes_sorted_in_batch == 2

    # If stale message arrives again, target_queue should be empty
    node.vision_callback(msg1)
    assert len(node.target_queue) == 0

    node.destroy_node()


def test_all_28_cells_unique_keys_and_sorting(rclpy_context):
    """Verify that all 28 grid cells generate distinct keys and are queued without skipping."""
    from sorting_arm.autonomous_sorter_node import AutonomousSorterNode
    node = AutonomousSorterNode()
    node.dry_run = True

    # Generate all 28 picking cells: Cols 0..6, Rows 7..10
    objects = []
    expected_keys = set()
    for r in [7, 8, 9, 10]:
        for c in range(7):
            wx = (c + 0.5) * 50.0
            wy = (11 - r + 0.5) * 50.0
            key = AutonomousSorterNode.get_cell_key(wx, wy)
            expected_keys.add(key)
            objects.append({
                "color": "red" if (r + c) % 2 == 0 else "green",
                "world_x_mm": wx,
                "world_y_mm": wy,
                "world_z_mm": 13.7,
                "target_r_mm": math.hypot(wx - 175.0, wy - 375.0),
                "target_theta_deg": 0.0,
                "targetable": True,
                "assigned_arm": "xarm_1",
            })

    # Exactly 28 unique cell keys must exist (none collapsed by rounding)
    assert len(expected_keys) == 28, f"Expected 28 unique keys, got {len(expected_keys)}: {expected_keys}"

    msg = String()
    msg.data = json.dumps({"source": "vision", "objects": objects})
    node.vision_callback(msg)

    # All 28 objects must be admitted into target_queue
    assert len(node.target_queue) == 28

    node.enabled = True
    # Run executive_loop 28 times; every object must be sorted
    for i in range(28):
        node.executive_loop()

    assert node.cubes_sorted_in_batch == 28
    assert len(node.sorted_cells) == 28
    assert node.sorted_cells == expected_keys
    assert len(node.target_queue) == 0

    node.destroy_node()


def test_all_43_cells_extended_arena_sorting(rclpy_context):
    """Verify that all 43 picking cells in Rows 4..10 generate distinct keys and are queued and sorted."""
    from sorting_arm.autonomous_sorter_node import AutonomousSorterNode
    node = AutonomousSorterNode()
    node.dry_run = True

    # Generate all 43 picking cells: Cols 0..6, Rows 4..10, excluding xArm mount
    from sorting_arm.kinematics import world_to_arm_polar
    objects = []
    expected_keys = set()
    for r in range(4, 11):
        for c in range(7):
            if c in (2, 3, 4) and r in (4, 5):
                continue

            if r == 6 and c == 3:
                wx, wy = 175.0, 250.5
            elif r == 6 and c in (2, 4):
                wx, wy = (c + 0.5) * 50.0, 250.5
            elif r in (4, 5) and c == 1:
                wx = 52.0
                wy = 352.0 if r == 4 else 302.0
            elif r in (4, 5) and c == 5:
                wx = 298.0
                wy = 352.0 if r == 4 else 302.0
            else:
                wx = (c + 0.5) * 50.0
                wy = (11 - r + 0.5) * 50.0

            key = AutonomousSorterNode.get_cell_key(wx, wy)
            expected_keys.add(key)
            r_mm, theta_arm, _ = world_to_arm_polar(wx, wy, 13.7, arm_id="xarm_1", params=node.params)
            objects.append({
                "color": "red" if (r + c) % 2 == 0 else "green",
                "world_x_mm": wx,
                "world_y_mm": wy,
                "world_z_mm": 13.7,
                "target_r_mm": r_mm,
                "target_theta_deg": theta_arm,
                "targetable": True,
                "assigned_arm": "xarm_1",
            })

    # Exactly 43 unique cell keys must exist
    assert len(expected_keys) == 43, f"Expected 43 unique keys, got {len(expected_keys)}"
    assert len(objects) == 43

    msg = String()
    msg.data = json.dumps({"source": "vision", "objects": objects})
    node.vision_callback(msg)

    # All 43 objects must be admitted into target_queue
    assert len(node.target_queue) == 43

    node.enabled = True
    # Run executive_loop 43 times; every object must be sorted
    for _ in range(43):
        node.executive_loop()

    assert node.cubes_sorted_in_batch == 43
    assert len(node.sorted_cells) == 43
    assert node.sorted_cells == expected_keys
    assert len(node.target_queue) == 0

    node.destroy_node()



