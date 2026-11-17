#!/usr/bin/env python3
"""
xArm 1S FastMCP Server
Model Context Protocol Server for Hardware Access, Telemetry, and Kinematic Manipulation.

Authoritative Workspace & Driver Integration:
- Direct USB HID / Serial UART hardware control.
- Enforces calibrated soft limits and nominal centers from config/xarm_hardware.yaml.
- Real-time potentiometer position telemetry via CMD 0x15 readback.
- Cartesian cylindrical inverse kinematics (r, theta, z).
- Predefined multi-state poses and gripper control.
- Instant emergency stop and torque release.
"""

from __future__ import annotations

import csv
from datetime import datetime
import json
import logging
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, List, Optional

# Setup workspace paths
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = WORKSPACE_ROOT / "config" / "xarm_hardware.yaml"
DATA_DIR = WORKSPACE_ROOT / "data"
CSV_LOG_PATH = DATA_DIR / "telemetry_drift_log.csv"
ROS_PKG_DIR = WORKSPACE_ROOT / "ros2_ws" / "src" / "sorting_arm"

for p in [str(WORKSPACE_ROOT), str(ROS_PKG_DIR)]:
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml
from mcp.server.fastmcp import FastMCP

from sorting_arm.kinematics import (
    KinematicParameters,
    load_kinematic_parameters,
    solve_ik,
    TICKS_PER_DEG,
    DEG_PER_TICK,
)
from sorting_arm.xarm_protocol import (
    MOTOR_NAMES,
    XArmDriver,
    find_xarm_device,
)

# Initialize FastMCP Server
mcp = FastMCP("xArmController")

# Global driver singleton
_DRIVER: Optional[XArmDriver] = None


def load_hardware_config() -> Dict[str, Any]:
    """Load unified hardware configuration."""
    if not CONFIG_PATH.is_file():
        raise FileNotFoundError(f"Hardware configuration not found at {CONFIG_PATH}")
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get_driver() -> XArmDriver:
    """Get or initialize singleton xArm driver."""
    global _DRIVER
    if _DRIVER is not None and _DRIVER.is_connected:
        return _DRIVER

    hw_cfg = load_hardware_config()
    manipulator = hw_cfg.get("manipulators", {}).get("xarm_1", {})
    motors = manipulator.get("motors", {})
    max_dev = int(hw_cfg.get("manipulators", {}).get("max_deviation_ticks", 25))

    min_ticks = {i: int(motors.get(f"motor_{i}", {}).get("min_tick", 150)) for i in range(1, 7)}
    max_ticks = {i: int(motors.get(f"motor_{i}", {}).get("max_tick", 850)) for i in range(1, 7)}
    deviations = {i: int(motors.get(f"motor_{i}", {}).get("deviation", 0)) for i in range(1, 7)}

    dev_path, iface = find_xarm_device()
    if not dev_path:
        dev_path = manipulator.get("communication", {}).get("fallback_serial_device", "/dev/ttyUSB0")

    _DRIVER = XArmDriver(
        port=dev_path,
        baudrate=115200,
        deviations=deviations,
        min_ticks=min_ticks,
        max_ticks=max_ticks,
        max_deviation_ticks=max_dev,
    )

    if not _DRIVER.connect():
        raise RuntimeError(f"Failed to connect to xArm device on {dev_path}")

    return _DRIVER


def record_telemetry(actuals: Dict[int, Optional[int]], targets: Dict[int, int]):
    """Append readings to telemetry_drift_log.csv."""
    os.makedirs(DATA_DIR, exist_ok=True)
    header = [
        "timestamp_iso", "arm_id", "motor_id", "joint_name",
        "target_tick", "actual_tick", "tracking_error_ticks", "deviation_trim", "is_simulated"
    ]
    if not CSV_LOG_PATH.is_file() or CSV_LOG_PATH.stat().st_size == 0:
        with open(CSV_LOG_PATH, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)

    now_iso = datetime.now().isoformat()
    with open(CSV_LOG_PATH, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for sid in range(1, 7):
            act = actuals.get(sid)
            tgt = targets.get(sid, 500)
            err = (act - tgt) if act is not None else ""
            writer.writerow([
                now_iso, "xarm_1", sid, MOTOR_NAMES[sid], tgt,
                act if act is not None else "", err, 0, False
            ])


# =====================================================================================
# MCP TOOLS
# =====================================================================================

@mcp.tool()
def get_joint_telemetry() -> Dict[str, Any]:
    """
    Query real-time potentiometer ticks (CMD 0x15) for all 6 servos on the xArm.
    Returns current target ticks, actual feedback ticks, tracking error, and degree angles.
    """
    driver = get_driver()
    hw_cfg = load_hardware_config()
    motors = hw_cfg.get("manipulators", {}).get("xarm_1", {}).get("motors", {})

    results = {}
    actuals = {}
    targets = {}

    for sid in range(1, 7):
        target = driver.current_ticks.get(sid, 500)
        actual = driver.read_servo_position(sid)
        actuals[sid] = actual
        targets[sid] = target

        m_cfg = motors.get(f"motor_{sid}", {})
        center = int(m_cfg.get("nominal_center_tick", 500))

        entry = {
            "joint_name": MOTOR_NAMES[sid],
            "target_tick": target,
            "actual_tick": actual,
            "tracking_error_ticks": (actual - target) if actual is not None else None,
            "angle_deg_from_center": round((actual - center) * DEG_PER_TICK, 2) if actual is not None else None,
            "min_tick_limit": driver.min_ticks.get(sid),
            "max_tick_limit": driver.max_ticks.get(sid),
        }
        results[f"motor_{sid}"] = entry

    record_telemetry(actuals, targets)
    return {
        "status": "success",
        "device": driver.port,
        "interface": driver.interface_type,
        "is_estopped": driver.is_estopped,
        "joints": results
    }


@mcp.tool()
def move_joint(servo_id: int, target_tick: int, time_ms: int = 1000) -> Dict[str, Any]:
    """
    Move a single xArm joint (Motor 1 to 6) to a target tick within safe soft limits.
    Waits for the motion duration and reads back the actual final potentiometer position.

    :param servo_id: Motor ID (1=Claw, 2=Wrist Roll, 3=Wrist Pitch, 4=Elbow Pitch, 5=Shoulder Pitch, 6=Base Yaw)
    :param target_tick: Target servo tick (clamped to configured limits)
    :param time_ms: Interpolation transit duration in milliseconds (default: 1000)
    """
    if not (1 <= servo_id <= 6):
        return {"status": "error", "message": f"Invalid servo_id {servo_id}. Must be 1 to 6."}

    driver = get_driver()
    min_t = driver.min_ticks.get(servo_id, 150)
    max_t = driver.max_ticks.get(servo_id, 850)
    clamped_target = max(min_t, min(max_t, int(target_tick)))

    ok = driver.send_servo_move(servo_id, clamped_target, time_ms=time_ms)
    if not ok:
        return {"status": "error", "message": "Driver failed to dispatch servo command."}

    time.sleep((time_ms + 100) / 1000.0)
    actual = driver.read_servo_position(servo_id)

    return {
        "status": "success",
        "servo_id": servo_id,
        "joint_name": MOTOR_NAMES[servo_id],
        "requested_tick": target_tick,
        "clamped_target_tick": clamped_target,
        "actual_tick_readback": actual,
        "tracking_error_ticks": (actual - clamped_target) if actual is not None else None,
        "move_time_ms": time_ms
    }


@mcp.tool()
def move_all_joints(joint_ticks: Dict[int, int], time_ms: int = 1500) -> Dict[str, Any]:
    """
    Move multiple xArm joints simultaneously with synchronized interpolation.
    
    :param joint_ticks: Dictionary mapping servo ID (1-6) to target ticks, e.g. {"1": 517, "6": 450}
    :param time_ms: Transit duration in milliseconds (default: 1500)
    """
    driver = get_driver()
    targets = {int(k): int(v) for k, v in joint_ticks.items() if 1 <= int(k) <= 6}
    if not targets:
        return {"status": "error", "message": "No valid joint targets provided (IDs must be 1 to 6)."}

    clamped_targets = {}
    for sid, tick in targets.items():
        min_t = driver.min_ticks.get(sid, 150)
        max_t = driver.max_ticks.get(sid, 850)
        clamped_targets[sid] = max(min_t, min(max_t, tick))

    ok = driver.send_multi_servo_move(clamped_targets, time_ms=time_ms, synchronized=True)
    if not ok:
        return {"status": "error", "message": "Failed to dispatch multi-servo command."}

    time.sleep((time_ms + 150) / 1000.0)

    actuals = {}
    for sid in clamped_targets.keys():
        actuals[sid] = driver.read_servo_position(sid)

    record_telemetry(actuals, driver.current_ticks)

    return {
        "status": "success",
        "commanded_targets": clamped_targets,
        "actual_readbacks": actuals,
        "move_time_ms": time_ms
    }


@mcp.tool()
def move_to_preset_pose(pose_name: str, time_ms: int = 1500) -> Dict[str, Any]:
    """
    Move the xArm to a predefined named pose from config/xarm_hardware.yaml.
    Available poses:
      - 'home_tucked': Safe upright vertical reset pose [517, 503, 504, 496, 501, 502]
      - 'ready_standoff': Hovering safely above arena picking zone (Zw ≈ 165mm)
      - 'drop_container': Over left flank sorting container (Z ≈ 56mm)

    :param pose_name: Name of preset pose
    :param time_ms: Transit duration in milliseconds
    """
    hw_cfg = load_hardware_config()
    poses = hw_cfg.get("manipulators", {}).get("xarm_1", {}).get("poses", {})

    if pose_name not in poses:
        available = list(poses.keys())
        return {"status": "error", "message": f"Pose '{pose_name}' not found. Available: {available}"}

    pose_cfg = poses[pose_name]
    ticks_list = pose_cfg.get("ticks", [])
    if len(ticks_list) != 6:
        return {"status": "error", "message": f"Malformed pose ticks for '{pose_name}': {ticks_list}"}

    targets = {i + 1: ticks_list[i] for i in range(6)}
    transit = time_ms or pose_cfg.get("transit_time_ms", 1500)

    driver = get_driver()
    ok = driver.send_multi_servo_move(targets, time_ms=transit, synchronized=True)
    if not ok:
        return {"status": "error", "message": f"Failed to execute pose '{pose_name}'"}

    time.sleep((transit + 150) / 1000.0)

    actuals = {sid: driver.read_servo_position(sid) for sid in range(1, 7)}
    record_telemetry(actuals, targets)

    return {
        "status": "success",
        "pose_name": pose_name,
        "description": pose_cfg.get("description", ""),
        "commanded_ticks": targets,
        "actual_readbacks": actuals,
        "transit_time_ms": transit
    }


@mcp.tool()
def move_cartesian(
    r_mm: float,
    theta_deg: float,
    z_mm: float,
    claw_tick: Optional[int] = None,
    time_ms: int = 1500,
) -> Dict[str, Any]:
    """
    Move the arm end-effector to a Cartesian cylindrical coordinate (R, Theta, Z) relative to arm base.
    Uses analytical inverse kinematics with natural wrist pitch and reach feasibility checks.

    :param r_mm: Radial reach from base center in millimeters (e.g. 150.0 to 280.0 mm)
    :param theta_deg: Azimuthal yaw angle in degrees (0 = South forward into arena, negative = West, positive = East)
    :param z_mm: Vertical height above tabletop in millimeters (0 = table surface, 150 = hover)
    :param claw_tick: Optional claw gripper tick (defaults to current position or neutral 517)
    :param time_ms: Transit duration in milliseconds
    """
    params = load_kinematic_parameters(str(CONFIG_PATH), arm_id="xarm_1")
    driver = get_driver()

    if claw_tick is None:
        claw_tick = driver.current_ticks.get(1, 517)

    solution = solve_ik(r=r_mm, theta=theta_deg, z=z_mm, params=params, claw_tick=claw_tick)
    if not solution.success:
        return {
            "status": "error",
            "message": f"Inverse Kinematics failed: {solution.message}",
            "requested_cartesian": {"r_mm": r_mm, "theta_deg": theta_deg, "z_mm": z_mm}
        }

    ok = driver.send_multi_servo_move(solution.ticks, time_ms=time_ms, synchronized=True)
    if not ok:
        return {"status": "error", "message": "Failed to send IK joint trajectory to arm."}

    time.sleep((time_ms + 150) / 1000.0)

    actuals = {sid: driver.read_servo_position(sid) for sid in range(1, 7)}
    record_telemetry(actuals, solution.ticks)

    return {
        "status": "success",
        "requested_cartesian": {"r_mm": r_mm, "theta_deg": theta_deg, "z_mm": z_mm},
        "ik_solution_ticks": solution.ticks,
        "ik_joint_angles_deg": solution.angles_deg,
        "actual_readbacks": actuals,
        "move_time_ms": time_ms
    }


@mcp.tool()
def control_gripper(
    action: str,
    custom_tick: Optional[int] = None,
    time_ms: int = 800
) -> Dict[str, Any]:
    """
    Actuate the claw gripper (Motor 1) to preset spans or a custom tick.
    
    :param action: 'open' (56mm span, tick 226), 'half_open' (43mm, tick 517), 'grip_cube_25mm' (tick 630), 'grip_cube_31mm' (tick 588), 'close' (tick 782), or 'custom'
    :param custom_tick: Required if action is 'custom'
    :param time_ms: Move duration in milliseconds (default: 800)
    """
    hw_cfg = load_hardware_config()
    m1_spans = (
        hw_cfg.get("manipulators", {})
        .get("xarm_1", {})
        .get("motors", {})
        .get("motor_1", {})
        .get("claw_span_mm", {})
    )

    action_to_tick = {
        "open": int(m1_spans.get("fully_open", {}).get("tick", 226)),
        "half_open": int(m1_spans.get("nominal_reset", {}).get("tick", 517)),
        "grip_cube_25mm": int(m1_spans.get("grip_cube_25mm", {}).get("tick", 630)),
        "grip_cube_31mm": int(m1_spans.get("grip_cube_31mm", {}).get("tick", 588)),
        "close": int(m1_spans.get("fully_closed_empty", {}).get("tick", 782)),
    }

    if action == "custom":
        if custom_tick is None:
            return {"status": "error", "message": "custom_tick is required when action='custom'"}
        target = int(custom_tick)
    elif action in action_to_tick:
        target = action_to_tick[action]
    else:
        return {"status": "error", "message": f"Unknown action '{action}'. Valid: {list(action_to_tick.keys()) + ['custom']}"}

    return move_joint(servo_id=1, target_tick=target, time_ms=time_ms)


@mcp.tool()
def emergency_stop() -> Dict[str, Any]:
    """
    Immediately cut electrical torque to all 6 servos (CMD 0x14 Multi-Servo Unload).
    Servos will go completely limp and stop executing motion immediately.
    """
    driver = get_driver()
    ok = driver.emergency_stop()
    return {
        "status": "success" if ok else "error",
        "action": "EMERGENCY_STOP",
        "torque_released": ok,
        "is_estopped": driver.is_estopped
    }


@mcp.tool()
def re_enable_servos() -> Dict[str, Any]:
    """
    Re-enable torque on servos by re-holding current physical positions.
    Restores the arm from emergency stop state.
    """
    driver = get_driver()
    driver.is_estopped = False
    actuals = {}
    for sid in range(1, 7):
        pos = driver.read_servo_position(sid)
        if pos is not None:
            driver.send_servo_move(sid, pos, time_ms=100)
            actuals[sid] = pos
    return {
        "status": "success",
        "message": "Torque re-enabled at current positions.",
        "held_positions": actuals,
        "is_estopped": driver.is_estopped
    }


@mcp.tool()
def get_drift_report() -> Dict[str, Any]:
    """
    Analyze recent potentiometer drift log from data/telemetry_drift_log.csv.
    Returns sample count, mean tracking error, and standard deviation per channel.
    """
    if not CSV_LOG_PATH.is_file() or CSV_LOG_PATH.stat().st_size == 0:
        return {"status": "empty", "message": "No telemetry logs recorded yet."}

    channel_errors: Dict[str, List[int]] = {}
    with open(CSV_LOG_PATH, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ch = f"{row.get('arm_id', 'xarm_1')}_M{row.get('motor_id', '')}"
            err_str = row.get("tracking_error_ticks", "")
            if err_str != "":
                try:
                    channel_errors.setdefault(ch, []).append(int(err_str))
                except ValueError:
                    pass

    summary = {}
    for ch, errors in channel_errors.items():
        if errors:
            n = len(errors)
            mean = sum(errors) / n
            var = sum((x - mean) ** 2 for x in errors) / n
            std = var ** 0.5
            summary[ch] = {
                "samples": n,
                "mean_error_ticks": round(mean, 2),
                "std_dev_ticks": round(std, 2),
                "min_error": min(errors),
                "max_error": max(errors),
            }

    return {"status": "success", "channels": summary}


if __name__ == "__main__":
    mcp.run()
