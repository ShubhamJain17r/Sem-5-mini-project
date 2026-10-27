from sorting_arm.kinematics import (
    MAX_SAFE_TICKS,
    MIN_SAFE_TICKS,
    KinematicParameters,
    get_compartment_target,
    load_kinematic_parameters,
    solve_fk,
    solve_ik,
    world_to_arm_polar,
)
from sorting_arm.xarm_protocol import XArmDriver


def test_ik_reachable_envelope():
    # Test valid operational coordinates in arena picking zone
    test_points = [
        (200.0, 0.0, 80.0),
        (220.0, 15.0, 60.0),
        (240.0, -20.0, 50.0),
        (260.0, 0.0, 100.0),
    ]
    for r, th, z in test_points:
        sol = solve_ik(r=r, theta=th, z=z)
        assert sol.success, f"Failed at reachable target: r={r}, th={th}, z={z}: {sol.message}"
        for mid, t in sol.ticks.items():
            err_msg = f"Motor {mid} tick {t} out of bounds"
            assert MIN_SAFE_TICKS[mid] <= t <= MAX_SAFE_TICKS[mid], err_msg


def test_ik_out_of_reach():
    # Target too far (> L1 + L2 + L3 = 346mm)
    sol_far = solve_ik(r=400.0, theta=0.0, z=80.0)
    assert not sol_far.success, "Far target should have failed reachability"

    # Target behind shoulder
    sol_behind = solve_ik(r=50.0, theta=0.0, z=80.0)
    assert not sol_behind.success, "Behind-shoulder target should have failed"


def test_fk_ik_roundtrip():
    # Verify mathematical roundtrip consistency
    r, th, z = 220.0, 10.0, 80.0
    sol = solve_ik(r=r, theta=th, z=z)
    assert sol.success

    fk = solve_fk(sol.ticks[6], sol.ticks[5], sol.ticks[4], sol.ticks[3])
    assert abs(fk.r - r) < 1.0, f"Radial error too large: {abs(fk.r - r)}mm"
    assert abs(fk.theta - th) < 0.5, f"Yaw error too large: {abs(fk.theta - th)}deg"
    assert abs(fk.z - z) < 1.0, f"Vertical error too large: {abs(fk.z - z)}mm"


def test_protocol_calibration_and_limits():
    # Tested with arm_1 calibrated deviations: M1: 1, M2: 3, M3: -3, M4: -13, M5: 0, M6: 0
    devs = {1: 1, 2: 3, 3: -3, 4: -13, 5: 0, 6: 0}
    driver = XArmDriver(port="/dev/null", deviations=devs)

    # Motor 4 deviation: tick 500 should become 500 - 13 = 487
    assert driver.apply_deviations_and_limits(4, 500) == 487

    # Motor 3 deviation: tick 500 should become 500 - 3 = 497
    assert driver.apply_deviations_and_limits(3, 500) == 497

    # Limit clamping: tick 900 on Motor 6 should be clamped to 850
    assert driver.apply_deviations_and_limits(6, 900) == 850

    # Limit clamping: tick 50 on Motor 1 should be clamped to 150
    assert driver.apply_deviations_and_limits(1, 50) == 150


def test_load_kinematic_parameters():
    cfg = "config/dual_xarm_hardware.yaml"
    params1 = load_kinematic_parameters(config_path=cfg, arm_id="xarm_1")
    assert params1.d1 == 65.0
    assert params1.L1 == 101.0
    assert params1.L2 == 95.0
    assert params1.L3_grip == 150.0
    assert params1.L3_tip == 168.0
    assert params1.min_safe_ticks[1] == 150
    assert params1.max_safe_ticks[6] == 850

    params2 = load_kinematic_parameters(config_path=cfg, arm_id="xarm_2")
    assert params2.L1 == 101.0
    assert params2.L2 == 95.0


def test_get_compartment_target():
    cfg = "config/dual_xarm_hardware.yaml"
    colors = ["red", "green", "blue", "yellow"]
    for color in colors:
        target1 = get_compartment_target(color, arm_id="xarm_1", config_path=cfg)
        assert target1 is not None, f"Failed to get target for color {color}"
        assert target1["color"] == color
        assert target1["r"] > 0.0
        assert "target_world_mm" in target1 or "world_xyz" in target1
        assert "drop_yaw_tick" in target1
        assert 150 <= target1["drop_yaw_tick"] <= 300

        target2 = get_compartment_target(color, arm_id="xarm_2", config_path=cfg)
        assert target2 is not None, f"Failed to get target for color {color} on arm_2"
        assert target2["color"] == color
        assert 700 <= target2["drop_yaw_tick"] <= 870

    # Invalid color returns None
    assert get_compartment_target("magenta", arm_id="xarm_1") is None


def test_near_field_wrist_pitch_safe():
    # Targets close to shoulder (r <= 220mm, z <= 80mm) previously caused Motor 3
    # to demand ~151 ticks (physical collision with forearm bracket).
    # With adaptive pitch, Motor 3 tick must remain well above 250 ticks.
    close_targets = [
        (200.0, 0.0, 30.0),
        (210.0, 10.0, 40.0),
        (220.0, -10.0, 20.0),
        (220.0, 0.0, 80.0),
    ]
    for r, th, z in close_targets:
        sol = solve_ik(r=r, theta=th, z=z)
        assert sol.success, f"Failed at close target ({r}, {th}, {z}): {sol.message}"
        m3_tick = sol.ticks[3]
        assert m3_tick >= 250, (
            f"Motor 3 tick ({m3_tick}) dangerously close to mechanical limit for ({r}, {th}, {z})"
        )


def test_ik_with_claw_tip():
    # Test IK computation using outer claw tip vs grip center
    sol_grip = solve_ik(r=250.0, theta=0.0, z=50.0, use_claw_tip=False)
    sol_tip = solve_ik(r=250.0, theta=0.0, z=50.0, use_claw_tip=True)
    assert sol_grip.success
    assert sol_tip.success
    # With longer tool length (tip), shoulder/elbow angles must differ to reach same position
    assert sol_grip.ticks[5] != sol_tip.ticks[5]


def test_world_to_arm_polar():
    # xarm_1 is at (170, 360, 0) with nominal heading -110.0 deg
    # Point at (170, 160, 12.5) is dx=0, dy=-200 (directly South): r=200, theta_world=-90 deg
    # theta_arm = -90 - (-110) = +20 deg
    r, th, z = world_to_arm_polar(170.0, 160.0, 12.5, arm_id="xarm_1")
    assert abs(r - 200.0) < 0.1
    assert abs(th - 20.0) < 0.1
    assert abs(z - 12.5) < 0.1

    # If arm heading is custom (e.g. -90 deg South)
    custom_params = KinematicParameters(base_heading_deg=-90.0)
    r, th, z = world_to_arm_polar(170.0, 160.0, 12.5, arm_id="xarm_1", params=custom_params)
    assert abs(r - 200.0) < 0.1
    # Global angle -90° minus heading -90° = 0° local
    assert abs(th - 0.0) < 0.1


def test_base_zero_and_heading_kinematics():
    # Test that configured base_zero_deg offsets tick 6 properly
    p_offset = KinematicParameters(base_zero_deg=30.0)
    # Target theta = 30° should result in tick 500
    sol = solve_ik(r=220.0, theta=30.0, z=80.0, params=p_offset)
    assert sol.success
    assert sol.ticks[6] == 500

    # FK roundtrip with base_zero_deg
    fk = solve_fk(sol.ticks[6], sol.ticks[5], sol.ticks[4], sol.ticks[3], params=p_offset)
    assert abs(fk.theta - 30.0) < 0.5


def test_compartment_target_fallback():
    cfg = "config/dual_xarm_hardware.yaml"
    target = get_compartment_target("red", arm_id="xarm_1", config_path=cfg)
    assert target is not None
    assert "drop_yaw_tick" in target
    assert target["drop_yaw_tick"] == 175
    assert "drop_pose_preset" in target
    assert "theta_arm_deg" in target
    assert "theta_world_deg" in target
    assert "ik_success" in target

