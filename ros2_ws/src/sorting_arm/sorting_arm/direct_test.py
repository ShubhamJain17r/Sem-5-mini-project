#!/usr/bin/env python3
"""
Stage 1 Standalone Hardware & Serial Verification CLI for xArm 1S.

Provides interactive terminal verification:
  - Connect to /dev/ttyUSB0 at 115200 baud
  - Move arm to safe Home Pose
  - Jog individual motors (1 to 6)
  - Verify waypoint execution
  - Real-time potentiometer position query
  - Immediate emergency stop / torque release
"""

import argparse
import os
import sys
import time
from typing import Dict

# Support importing within workspace
pkg_dir = os.path.dirname(os.path.abspath(__file__))
ws_dir = os.path.dirname(pkg_dir)
if ws_dir not in sys.path:
    sys.path.insert(0, ws_dir)

from sorting_arm.xarm_protocol import (  # noqa: E402
    MOTOR_NAMES,
    POSE_HOME_TICKS,
    XArmDriver,
)
import yaml  # noqa: E402


def load_calibration_offsets(config_path: str, arm_name: str = "xarm_1") -> Dict[int, int]:
    """Load calibrated servo deviations from unified hardware config (defaulting to 0)."""
    if not os.path.exists(config_path):
        return {i: 0 for i in range(1, 7)}

    try:
        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}
            motors = data.get("manipulators", {}).get(arm_name, {}).get("motors", {})
            return {
                sid: int(motors.get(f"motor_{sid}", {}).get("deviation", 0))
                for sid in range(1, 7)
            }
    except Exception as e:
        print(f"[WARN] Error reading hardware YAML: {e}. Using 0 offsets.")
        return {i: 0 for i in range(1, 7)}


def print_menu():
    """Display interactive options menu."""
    print("\n" + "=" * 60)
    print("      xArm 1S - Stage 1 Hardware Bringup Test Utility")
    print("=" * 60)
    print("  [1] Move to Home Pose (Safe Tucked Transit)")
    print("  [2] Jog Individual Motor (Motors 1 to 6)")
    print("  [3] Run Multi-Waypoint Reach Test")
    print("  [4] Query Real-time Joint Positions (CMD 0x15)")
    print("  [5] EMERGENCY STOP (Cut torque / Servos Limp)")
    print("  [6] Re-enable Torque")
    print("  [q] Quit")
    print("=" * 60)


def jog_motor_interactive(driver: XArmDriver):
    """Interactively jog single servo."""
    print("\n--- Jog Individual Motor ---")
    for mid in range(1, 7):
        cur = driver.current_ticks.get(mid, 500)
        print(f"  [{mid}] Motor {mid} ({MOTOR_NAMES[mid]}): Current={cur} ticks")

    choice = input("Select Motor (1-6) or [b]ack: ").strip().lower()
    if choice in ["b", ""]:
        return

    try:
        mid = int(choice)
        if mid < 1 or mid > 6:
            print("[ERROR] Invalid Motor ID.")
            return
    except ValueError:
        print("[ERROR] Invalid input.")
        return

    print(f"\nJogging Motor {mid} ({MOTOR_NAMES[mid]}):")
    print("  Controls: [+] +25 ticks | [-] -25 ticks | [p] enter target | [b] back")

    while True:
        cur = driver.current_ticks.get(mid, 500)
        hw_pos = driver.apply_deviations_and_limits(mid, cur)
        cmd = input(f"Motor {mid} (Current: {cur} ticks, HW: {hw_pos}): ").strip().lower()

        if cmd in ["b", "exit"]:
            break
        elif cmd in ["+", "w", "u"]:
            new_tick = cur + 25
            driver.send_servo_move(mid, new_tick, time_ms=300)
            print(f"-> Commanded {new_tick} ticks")
        elif cmd in ["-", "s", "d"]:
            new_tick = cur - 25
            driver.send_servo_move(mid, new_tick, time_ms=300)
            print(f"-> Commanded {new_tick} ticks")
        elif cmd == "p":
            min_t = driver.min_ticks[mid]
            max_t = driver.max_ticks[mid]
            val = input(f"Enter target ticks ({min_t} to {max_t}): ").strip()
            try:
                t = int(val)
                driver.send_servo_move(mid, t, time_ms=800)
                print(f"-> Commanded {t} ticks")
            except ValueError:
                print("[ERROR] Please enter a valid integer.")
        else:
            print("Invalid key. Use +, -, p, or b.")


def run_waypoint_test(driver: XArmDriver):
    """Execute a gentle pick-and-place waypoint sweep."""
    waypoints = [
        ("Home Pose", POSE_HOME_TICKS, 1500),
        ("Approach Pick Area", {1: 700, 2: 500, 3: 520, 4: 500, 5: 600, 6: 420}, 1600),
        ("Grasp Reach", {1: 700, 2: 500, 3: 580, 4: 640, 5: 500, 6: 420}, 1200),
        ("Claw Grip", {1: 250}, 600),
        ("Lift Object", {1: 250, 2: 500, 3: 520, 4: 500, 5: 600, 6: 420}, 1000),
        ("Swing to Drop Area", {1: 250, 2: 500, 3: 500, 4: 380, 5: 620, 6: 800}, 2000),
        ("Release Object", {1: 700}, 600),
        ("Return Home", POSE_HOME_TICKS, 1800),
    ]

    print("\n--- Starting Waypoint Sweep Test ---")
    for name, pose, duration in waypoints:
        if driver.is_estopped:
            print("[ABORT] E-Stop active, stopping waypoint sweep.")
            break
        print(f"Executing: {name} (duration {duration} ms)...")
        driver.send_multi_servo_move(pose, time_ms=duration, stagger_ms=25)
        time.sleep((duration + 100) / 1000.0)

    print("Waypoint test completed.\n")


def query_positions(driver: XArmDriver):
    """Query and print real-time servo positions."""
    print("\n--- Real-Time Potentiometer Readback (CMD 0x15) ---")
    for mid in range(1, 7):
        pos = driver.read_servo_position(mid, timeout_sec=0.15)
        if pos is not None:
            hw_pos = pos + driver.deviations.get(mid, 0)
            print(
                f"  Motor {mid} ({MOTOR_NAMES[mid]}): "
                f"Theoretical={pos} ticks | Hardware={hw_pos} ticks"
            )
        else:
            print(f"  Motor {mid} ({MOTOR_NAMES[mid]}): [No response / timeout]")


def main(args=None):
    """Entry point for stage 1 direct hardware verification CLI."""
    parser = argparse.ArgumentParser(description="xArm 1S Stage 1 Direct Hardware Test")
    parser.add_argument(
        "--port",
        default="auto",
        help="Serial or HID port device path (default: 'auto' probes USB HID and Serial)",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--arm", default="xarm_1", help="Arm identifier (xarm_1 or xarm_2)")
    parser.add_argument(
        "--config",
        default="config/dual_xarm_hardware.yaml",
        help="Path to dual_xarm_hardware.yaml",
    )
    cli_args, _ = parser.parse_known_args()

    config_file = cli_args.config
    if not os.path.isabs(config_file):
        for candidate in [
            os.path.join(os.getcwd(), config_file),
            os.path.join(os.path.dirname(ws_dir), config_file),
            os.path.join("/workspace", config_file),
        ]:
            if os.path.exists(candidate):
                config_file = candidate
                break

    deviations = load_calibration_offsets(config_file, cli_args.arm)
    print(f"[INIT] Loaded deviations for {cli_args.arm}: {deviations}")

    driver = XArmDriver(
        port=cli_args.port,
        baudrate=cli_args.baud,
        deviations=deviations,
    )

    print(f"[INIT] Connecting to xArm on {cli_args.port} at {cli_args.baud} baud...")
    if not driver.connect():
        print(f"[ERROR] Could not open {cli_args.port}.")
        print(
            "Please check:\n"
            "  1. Is USB cable plugged in?\n"
            "  2. Run 'ls -la /dev/ttyUSB*'\n"
            "  3. Are user permissions granted ('sudo chmod 666 /dev/ttyUSB0')?"
        )
        sys.exit(1)

    try:
        while True:
            print_menu()
            choice = input("Enter option: ").strip().lower()

            if choice == "1":
                driver.move_home(time_ms=1500)
            elif choice == "2":
                jog_motor_interactive(driver)
            elif choice == "3":
                run_waypoint_test(driver)
            elif choice == "4":
                query_positions(driver)
            elif choice == "5":
                driver.emergency_stop()
            elif choice == "6":
                driver.clear_emergency_stop()
            elif choice in ["q", "quit", "exit"]:
                print("Exiting test bench...")
                break
            else:
                print("Unknown command. Try again.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Triggering safe stop...")
        driver.emergency_stop()
    finally:
        driver.disconnect()
        print("Serial port closed.")


if __name__ == "__main__":
    main()
