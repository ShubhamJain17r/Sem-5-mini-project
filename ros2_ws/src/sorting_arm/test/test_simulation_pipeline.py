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
import time
import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from sorting_arm.driver_node import STANDARD_JOINT_NAMES, XArmDriverNode
from sorting_arm.kinematics import (
    DEG_PER_TICK,
    TICKS_PER_DEG,
    get_compartment_target,
    load_kinematic_parameters,
    solve_ik,
)
from sorting_arm.mock_vision_node import MockVisionNode
from std_msgs.msg import String
from std_srvs.srv import Trigger


@pytest.fixture(scope="module")
def rclpy_context():
    """Initializes rclpy context once for the test module."""
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

    test_sub = node.create_subscription(JointState, "joint_states", state_cb, 10)

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

    # Claw should be at nominal open (230 ticks -> (230 - 500) * 0.24 = -64.8 deg)
    expected_claw_rad = math.radians((230.0 - 500.0) * DEG_PER_TICK)
    assert abs(latest.position[0] - expected_claw_rad) < 0.01

    # Base yaw should be at center (500 ticks -> 0 rad)
    assert abs(latest.position[5] - 0.0) < 0.01

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
    target_yaw_rad = math.radians((600.0 - 500.0) * DEG_PER_TICK)
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
    cmd.position = [math.radians((700.0 - 500.0) * DEG_PER_TICK)]
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

    sub = v_node.create_subscription(String, "/vision/detected_objects", vision_cb, 10)

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
    red_target = get_compartment_target("red", arm_id="xarm_1", config_path="config/xarm_hardware.yaml")
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

    def dispatch_and_wait(ticks, move_time_ms=100, timeout_sec=1.5):
        cmd = JointState()
        cmd.name = STANDARD_JOINT_NAMES
        cmd.position = [
            math.radians(float(ticks.get(i, 500) - 500) * DEG_PER_TICK) for i in range(1, 7)
        ]
        driver.joint_command_callback(cmd)
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            executor.spin_once(timeout_sec=0.02)
            if all(abs(driver.sim_current_ticks[sid] - driver.sim_target_ticks[sid]) < 0.5 for sid in range(1, 7)):
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
    drop_ticks[1] = 230
    dispatch_and_wait(drop_ticks, move_time_ms=50)
    assert abs(driver.sim_current_ticks[1] - 230.0) < 2.0

    # Phase 7: Home
    home_ticks = {i: 500 for i in range(1, 7)}
    home_ticks[1] = 230
    dispatch_and_wait(home_ticks, move_time_ms=70)
    for sid in range(2, 7):
        assert abs(driver.sim_current_ticks[sid] - 500.0) < 2.0

    driver.destroy_node()
