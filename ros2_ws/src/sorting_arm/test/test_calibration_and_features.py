#!/usr/bin/env python3
"""
Comprehensive Unit Test Suite for Aligned xArm 2 Calibration, Closed-Loop Safety,
Trajectory Smoothing, Dual-Arm Coordination, and Telemetry Drift Analysis.

All tests run in pure simulation / offline mode (NO hardware connection required).
"""

import os
import shutil
import tempfile

from sorting_arm.kinematics import (
    KinematicParameters,
    load_kinematic_parameters,
    solve_fk,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.trajectory import (
    cubic_scaling,
    generate_cartesian_trajectory,
    generate_joint_trajectory,
    quintic_scaling,
)
from sorting_arm.xarm_protocol import XArmDriver
from tools.telemetry_logger import compute_drift_statistics, run_telemetry_scan
from tools.web_config.server import atomic_write_yaml, load_yaml


def test_xarm2_empirical_parameters_aligned():
    """Verify xArm 2 Motor 3 and Motor 4 empirical boundaries and positive polarities."""
    hw_path = "config/dual_xarm_hardware.yaml"
    params2 = load_kinematic_parameters(config_path=hw_path, arm_id="xarm_2")

    # Both arms must now share direction multiplier +1
    assert params2.direction_multipliers[3] == 1, "Motor 3 polarity must be +1"
    assert params2.direction_multipliers[4] == 1, "Motor 4 polarity must be +1"

    # Upright reset pose (90° upright) should yield ~500 ticks for all pitch joints
    sol = solve_ik(r=200.0, theta=0.0, z=80.0, params=params2)
    assert sol.success
    for sid in [3, 4, 5]:
        assert 130 <= sol.ticks[sid] <= 870, f"Motor {sid} tick {sol.ticks[sid]} out of safe range"


def test_xarm2_fk_ik_consistency():
    """Verify analytical IK and FK roundtrip with aligned polarities."""
    hw_path = "config/dual_xarm_hardware.yaml"
    params2 = load_kinematic_parameters(config_path=hw_path, arm_id="xarm_2")

    test_points = [
        (190.0, 10.0, 70.0),
        (220.0, 0.0, 50.0),
        (240.0, -15.0, 40.0),
    ]
    for r, th, z in test_points:
        sol = solve_ik(r=r, theta=th, z=z, params=params2)
        assert sol.success, f"IK failed for ({r}, {th}, {z}): {sol.message}"

        fk = solve_fk(sol.ticks[6], sol.ticks[5], sol.ticks[4], sol.ticks[3], params=params2)
        assert abs(fk.r - r) < 1.0, f"Radial error {abs(fk.r - r)} too large"
        assert abs(fk.theta - th) < 0.5, f"Azimuth error {abs(fk.theta - th)} too large"
        assert abs(fk.z - z) < 1.0, f"Z error {abs(fk.z - z)} too large"


def test_closed_loop_driver_methods():
    """Verify health check, in-position verification, and stall detection methods."""
    driver = XArmDriver(port="/dev/null")
    # In offline mode
    assert not driver.is_connected
    healthy, msg = driver.check_connection_health()
    assert not healthy
    assert "Not connected" in msg

    # Test in-position arrival simulation
    target = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
    arrived, actuals, errors = driver.wait_in_position(target_ticks=target, tolerance_ticks=10, timeout_sec=0.05)
    # Read failed because disconnected -> returns False
    assert not arrived

    # Test send_motion_and_wait with health check protection
    res = driver.send_motion_and_wait(target, time_ms=100)
    assert not res, "send_motion_and_wait must reject command when disconnected"


def test_trajectory_interpolation_math():
    """Verify quintic and cubic velocity/acceleration boundary conditions."""
    # At u=0: position=0, velocity=0, acceleration=0
    s0, ds0, dds0 = quintic_scaling(0.0)
    assert abs(s0) < 1e-6
    assert abs(ds0) < 1e-6
    assert abs(dds0) < 1e-6

    # At u=1: position=1, velocity=0, acceleration=0
    s1, ds1, dds1 = quintic_scaling(1.0)
    assert abs(s1 - 1.0) < 1e-6
    assert abs(ds1) < 1e-6
    assert abs(dds1) < 1e-6

    # At midpoint u=0.5: position=0.5
    s_mid, ds_mid, _ = quintic_scaling(0.5)
    assert abs(s_mid - 0.5) < 1e-6
    assert ds_mid > 0, "Midpoint velocity must be positive"

    # Joint trajectory generator
    start = {1: 500, 2: 500, 3: 500}
    target = {1: 600, 2: 400, 3: 700}
    points = generate_joint_trajectory(start, target, duration_sec=0.5, dt_sec=0.05)
    assert len(points) == 11
    assert points[0].ticks[1] == 500
    assert points[-1].ticks[1] == 600
    assert points[-1].ticks[2] == 400
    assert points[-1].ticks[3] == 700


def test_cartesian_trajectory_generator():
    """Verify smooth Cartesian path interpolation."""
    p = load_kinematic_parameters("config/dual_xarm_hardware.yaml", arm_id="xarm_1")
    pts, ok = generate_cartesian_trajectory(
        start_polar=(200.0, 0.0, 80.0),
        target_polar=(240.0, 10.0, 30.0),
        claw_tick=500,
        duration_sec=0.4,
        dt_sec=0.05,
        params=p,
    )
    assert ok, "All Cartesian waypoints must resolve valid IK"
    assert len(pts) == 9
    assert all(150 <= pt.ticks[5] <= 850 for pt in pts)


def test_atomic_yaml_saving():
    """Verify atomic YAML write with backup preservation and data integrity."""
    with tempfile.TemporaryDirectory() as tmpdir:
        test_yaml = os.path.join(tmpdir, "test.yaml")
        data1 = {"version": "1.0", "arm": "xarm_1", "ticks": [500, 500]}
        ok, msg = atomic_write_yaml(test_yaml, data1)
        assert ok
        assert os.path.exists(test_yaml)

        # Update
        data2 = {"version": "1.1", "arm": "xarm_1", "ticks": [600, 400]}
        ok, msg = atomic_write_yaml(test_yaml, data2)
        assert ok
        assert os.path.exists(test_yaml + ".bak")

        # Verify content
        loaded = load_yaml(test_yaml)
        assert loaded["version"] == "1.1"
        assert loaded["ticks"] == [600, 400]

        bak_loaded = load_yaml(test_yaml + ".bak")
        assert bak_loaded["version"] == "1.0"


def test_telemetry_drift_analysis():
    """Verify telemetry CSV logging and statistical repeatability analysis."""
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_file = os.path.join(tmpdir, "telemetry_test.csv")
        res = run_telemetry_scan(csv_path=csv_file)
        assert res["scanned_channels"] == 12
        assert os.path.exists(csv_file)

        stats = compute_drift_statistics(csv_file)
        assert "joint_metrics" in stats
        assert "recommendations" in stats
        assert len(stats["recommendations"]) > 0
