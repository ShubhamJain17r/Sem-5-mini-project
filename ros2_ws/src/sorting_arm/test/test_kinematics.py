from sorting_arm.kinematics import solve_ik, solve_fk, MIN_SAFE_TICKS, MAX_SAFE_TICKS
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
