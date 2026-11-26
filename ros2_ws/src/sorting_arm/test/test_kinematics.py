import os
import sys

# Ensure repository root and sorting_arm package are in sys.path
test_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(test_dir, "..", "..", "..", ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

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
    # Tested with arm_1 empirical deviations: M1: 1, M2: -2, M3: -3, M4: -5, M5: -2, M6: -5
    devs1 = {1: 1, 2: -2, 3: -3, 4: -5, 5: -2, 6: -5}
    driver1 = XArmDriver(port="/dev/null", deviations=devs1)

    # Motor 4 deviation: tick 500 should become 500 - 5 = 495
    assert driver1.apply_deviations_and_limits(4, 500) == 495

    # Motor 3 deviation: tick 500 should become 500 - 3 = 497
    assert driver1.apply_deviations_and_limits(3, 500) == 497

    # Motor 1 deviation: tick 500 should become 500 + 1 = 501
    assert driver1.apply_deviations_and_limits(1, 500) == 501

    # Test with custom deviation profile: M1: -2, M2: -4, M3: 2, M4: 3, M5: -3, M6: 2
    devs2 = {1: -2, 2: -4, 3: 2, 4: 3, 5: -3, 6: 2}
    driver2 = XArmDriver(port="/dev/null", deviations=devs2)

    # Motor 4 deviation: tick 500 should become 500 + 3 = 503
    assert driver2.apply_deviations_and_limits(4, 500) == 503

    # Motor 3 deviation: tick 500 should become 500 + 2 = 502
    assert driver2.apply_deviations_and_limits(3, 500) == 502

    # Limit clamping: tick 950 on Motor 6 should be clamped to default 850
    assert driver1.apply_deviations_and_limits(6, 950) == 850

    # Limit clamping with custom limits: min 220, max 880
    custom_limits_min = {1: 220, 2: 200, 3: 110, 4: 135, 5: 120, 6: 100}
    custom_limits_max = {1: 800, 2: 800, 3: 865, 4: 870, 5: 870, 6: 880}
    driver_custom = XArmDriver(
        port="/dev/null",
        deviations=devs1,
        min_ticks=custom_limits_min,
        max_ticks=custom_limits_max,
    )
    # Motor 1 tick 200 should clamp to 220
    assert driver_custom.apply_deviations_and_limits(1, 200) == 220
    # Motor 6 tick 950 should clamp to 880
    assert driver_custom.apply_deviations_and_limits(6, 950) == 880


def test_load_kinematic_parameters():
    cfg = "config/xarm_hardware.yaml"
    params1 = load_kinematic_parameters(config_path=cfg, arm_id="xarm_1")
    assert params1.d1 == 70.0
    assert params1.L1 == 101.0
    assert params1.L2 == 95.0
    assert params1.L3_grip == 150.0
    assert params1.L3_tip == 168.0
    assert 200 <= params1.min_safe_ticks[1] <= 260
    assert 850 <= params1.max_safe_ticks[6] <= 950


def test_get_compartment_target():
    cfg = "config/xarm_hardware.yaml"
    colors = ["red", "green"]
    for color in colors:
        target1 = get_compartment_target(color, arm_id="xarm_1", config_path=cfg)
        assert target1 is not None, f"Failed to get target for color {color}"
        assert target1["color"] == color
        assert target1["r"] > 0.0
        assert "target_world_mm" in target1 or "world_xyz" in target1
        assert "drop_yaw_tick" in target1
        assert 100 <= target1["drop_yaw_tick"] <= 900

    target_red = get_compartment_target("red", arm_id="xarm_1", config_path=cfg)
    target_green = get_compartment_target("green", arm_id="xarm_1", config_path=cfg)
    assert target_red["drop_yaw_tick"] == 650
    assert target_green["drop_yaw_tick"] == 354

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
    # Single xArm 1S is centered at (175, 375, 5.0) with nominal heading -90.0 deg (South down Col 3)
    # Point at (175, 175, 12.5) is dx=0, dy=-200 (directly South): r=200, theta_world=-90 deg
    # rel_z = 12.5 - 5.0 = 7.5 mm
    # theta_arm = -90 - (-90) = 0.0 deg
    r, th, z = world_to_arm_polar(175.0, 175.0, 12.5, arm_id="xarm_1")
    assert abs(r - 200.0) < 0.1
    assert abs(th - 0.0) < 0.1
    assert abs(z - 7.5) < 0.1


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
    cfg = "config/xarm_hardware.yaml"
    target = get_compartment_target("red", arm_id="xarm_1", config_path=cfg)
    assert target is not None
    assert "drop_yaw_tick" in target
    assert 120 <= target["drop_yaw_tick"] <= 900
    assert "drop_pose_preset" in target
    assert "theta_arm_deg" in target
    assert "theta_world_deg" in target
    assert "ik_success" in target


def test_hard_to_pick_zones_ik():
    """Verify analytical IK solves successfully for difficult near-base and far-reach coordinates."""
    cfg = "config/xarm_hardware.yaml"
    params = load_kinematic_parameters(config_path=cfg, arm_id="xarm_1")

    # Hard zone test cases (world_x, world_y, world_z, expected_min_r, expected_max_r)
    hard_points = [
        # Near-base cells (Row 6)
        (75.0, 275.0, 12.5, 140.0, 145.0),   # Row 6 Col 1: West near-base
        (275.0, 275.0, 12.5, 140.0, 145.0),  # Row 6 Col 5: East near-base
        (175.0, 225.0, 12.5, 148.0, 152.0),  # Row 7 Col 3: South close
        # Far-reach corners (Row 10, 97% extension)
        (25.0, 75.0, 12.5, 330.0, 340.0),    # Row 10 Col 0: Northwest corner
        (325.0, 75.0, 12.5, 330.0, 340.0),   # Row 10 Col 6: Northeast corner
    ]

    for wx, wy, wz, min_r, max_r in hard_points:
        r, theta, z = world_to_arm_polar(wx, wy, wz, params=params)
        assert min_r <= r <= max_r, f"Unexpected radial reach {r} for ({wx}, {wy})"

        # Tabletop pick IK
        sol_pick = solve_ik(r=r, theta=theta, z=z, params=params)
        assert sol_pick.success, f"Pick IK failed for hard coordinate ({wx}, {wy}): {sol_pick.message}"

        # Standoff IK
        standoff_r = r - 35.0 if r >= 180.0 else r
        sol_standoff = solve_ik(r=standoff_r, theta=theta, z=z + 25.0, params=params)
        assert sol_standoff.success, f"Standoff IK failed for ({wx}, {wy}): {sol_standoff.message}"

        # Verify all ticks are within hardware limits
        for mid in range(1, 7):
            min_t = params.min_safe_ticks[mid]
            max_t = params.max_safe_ticks[mid]
            assert min_t <= sol_pick.ticks[mid] <= max_t, (
                f"Pick Motor {mid} tick {sol_pick.ticks[mid]} out of [{min_t}, {max_t}]"
            )
            assert min_t <= sol_standoff.ticks[mid] <= max_t, (
                f"Standoff Motor {mid} tick {sol_standoff.ticks[mid]} out of [{min_t}, {max_t}]"
            )


def test_dynamic_tick_adaptation():
    """Verify that kinematics adapts seamlessly when tick limits and nominal centers are changed."""
    # Custom calibration profile with altered centers and boundaries
    custom_params = KinematicParameters(
        nominal_center_ticks={1: 520, 2: 510, 3: 495, 4: 505, 5: 490, 6: 515},
        min_safe_ticks={1: 180, 2: 150, 3: 100, 4: 120, 5: 110, 6: 100},
        max_safe_ticks={1: 820, 2: 850, 3: 880, 4: 870, 5: 860, 6: 910},
    )

    r, theta, z = 220.0, 0.0, 50.0
    sol = solve_ik(r=r, theta=theta, z=z, params=custom_params)
    assert sol.success

    # Base yaw tick should center at 515 for theta=0
    assert sol.ticks[6] == 515

    # FK roundtrip with custom centers
    fk = solve_fk(sol.ticks[6], sol.ticks[5], sol.ticks[4], sol.ticks[3], params=custom_params)
    assert abs(fk.r - r) < 1.0
    assert abs(fk.theta - theta) < 0.5
    assert abs(fk.z - z) < 1.0


def test_sag_compensation():
    """Verify that compensate_sag=True extends effective reach to neutralize cantilever droop."""
    params = KinematicParameters(gravity_sag_trim_mm=10.0, reach_scale_factor=0.05)
    r = 200.0
    sol_nom = solve_ik(r=r, theta=0.0, z=25.0, params=params, compensate_sag=False)
    sol_sag = solve_ik(r=r, theta=0.0, z=25.0, params=params, compensate_sag=True)
    assert sol_nom.success and sol_sag.success
    # Effective r with sag comp is 200 + 10.0 + 0.05 * 50 = 212.5mm
    # Shoulder pitch (Motor 5) tilts further forward (larger tick) for larger reach
    assert sol_sag.ticks[5] > sol_nom.ticks[5]


