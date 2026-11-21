#!/usr/bin/env python3
"""
Interactive Simulation Cube Commander CLI.

Allows users to spawn Red and Green craft cubes into specific 5cm grid cells,
trigger autonomous picking and placing, clear cubes, and monitor status.

Usage:
  python3 tools/sim_cube_commander.py
"""

import argparse
import json
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import PointStamped
except ImportError:
    print("ROS 2 Python libraries not found. Ensure ROS 2 Humble is sourced.")
    sys.exit(1)


class SimCubeCommander(Node):
    """ROS 2 client node for commanding simulation cubes and sorting actions."""

    def __init__(self, arm_id: str = "xarm_1"):
        super().__init__("sim_cube_commander")
        self.arm_id = arm_id

        # Publishers
        self.pub_vision = self.create_publisher(String, "/vision/detected_objects", 10)
        self.pub_click = self.create_publisher(PointStamped, "/clicked_point", 10)

        # Clients
        self.cli_start_sorting = self.create_client(Trigger, f"/{self.arm_id}/start_sorting")
        self.cli_stop_sorting = self.create_client(Trigger, f"/{self.arm_id}/stop_sorting")
        self.cli_spawn_cube = self.create_client(Trigger, "/simulation/spawn_cube")
        self.cli_clear_cubes = self.create_client(Trigger, "/simulation/clear_cubes")

    def call_trigger(self, client, timeout_sec: float = 2.0) -> bool:
        """Call a std_srvs/Trigger service synchronously."""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            print("  [!] Service not available.")
            return False

        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.done() and future.result() is not None:
            res = future.result()
            print(f"  [+] Response: {res.message} (success={res.success})")
            return res.success
        print("  [!] Service call timed out.")
        return False

    def spawn_cube(self, color: str, col: int, row: int):
        """Spawns a cube at the center of the specified 5cm cell."""
        col = max(0, min(6, col))
        row = max(0, min(11, row))

        x_m = (col + 0.5) * 0.050
        y_m = (11 - row + 0.5) * 0.050

        # Send via /clicked_point so arena_visualizer processes it naturally
        pt = PointStamped()
        pt.header.frame_id = "world"
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = x_m
        pt.point.y = y_m
        pt.point.z = 0.0127
        self.pub_click.publish(pt)
        print(f"  [+] Published cube command: {color.upper()} at Col {col}, Row {row} (X={x_m*1000:.1f}mm, Y={y_m*1000:.1f}mm)")


def print_grid():
    """Prints a text diagram of the 7x12 5cm arena grid with coordinate guide."""
    print("\n" + "=" * 65)
    print("      7x12 ARENA GRID (5 cm / 50 mm SQUARES) — TOP VIEW")
    print("=" * 65)
    print("  Rear Flank (Y = 600mm) ---------------------------------")
    print("  [Row  0]  [RED TRAY]   .   .  (xArm)  .   .  [GREEN TRAY]")
    print("  [Row  1]  [ C0 | C1 ]  .   .  (Base)  .   .  [ C5 | C6 ]")
    print("  [Row  2]       .       .   .  (Mount) .   .       .")
    print("  [Row  3]       .       .   .  ( 175 ) .   .       .")
    print("  [Row  4]       .       .   .  ( 375 ) .   .       .")
    print("  [Row  5]       .       .   .     .    .   .       .")
    print("  [Row  6]       .       .   .     .    .   .       .")
    print("  --------------------------------------------------------")
    print("  PICKING ZONE (Y = 100mm to 300mm):")
    print("  [Row  7]     C0   C1   C2   C3   C4   C5   C6")
    print("  [Row  8]     C0   C1  [R2]  C3  [G4]  C5   C6   <-- Default Dual Cubes")
    print("  [Row  9]     C0   C1   C2   C3   C4   C5   C6")
    print("  [Row 10]     C0   C1   C2   C3   C4   C5   C6")
    print("  --------------------------------------------------------")
    print("  [Row 11]     C0   C1   C2   C3   C4   C5   C6   (Y = 0mm, Front)")
    print("=" * 65)


def run_interactive(commander: SimCubeCommander):
    """Runs interactive menu loop."""
    print_grid()
    while rclpy.ok():
        print("\nCommands:")
        print("  [1] Spawn Red Cube at Col 2, Row 8 (X=125mm, Y=225mm)")
        print("  [2] Spawn Green Cube at Col 4, Row 8 (X=225mm, Y=225mm)")
        print("  [3] Custom Spawn: Select Color (red/green), Column (0..6), Row (7..10)")
        print("  [4] Start Autonomous Sorter (Triggers pick-and-place sequence)")
        print("  [5] Stop / Pause Autonomous Sorter")
        print("  [6] Clear All Visual Cubes")
        print("  [7] Print Arena Grid Map")
        print("  [q] Quit")

        try:
            choice = input("\nEnter selection: ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if choice == "1":
            commander.spawn_cube("red", col=2, row=8)
        elif choice == "2":
            commander.spawn_cube("green", col=4, row=8)
        elif choice == "3":
            try:
                col_in = input("  Enter Color (r for red, g for green) [default: red]: ").strip().lower()
                color = "green" if col_in.startswith("g") else "red"
                col = int(input("  Enter Column (0..6) [recommended 1..5]: ").strip())
                row = int(input("  Enter Row (0..11) [recommended 7..10]: ").strip())
                commander.spawn_cube(color, col=col, row=row)
            except ValueError:
                print("  [!] Invalid number entered.")
        elif choice == "4":
            print("  --> Triggering /{arm_id}/start_sorting...")
            commander.call_trigger(commander.cli_start_sorting)
        elif choice == "5":
            print("  --> Triggering /{arm_id}/stop_sorting...")
            commander.call_trigger(commander.cli_stop_sorting)
        elif choice == "6":
            print("  --> Clearing visual cubes...")
            commander.call_trigger(commander.cli_clear_cubes)
        elif choice == "7":
            print_grid()
        elif choice in ("q", "quit", "exit"):
            break
        else:
            print("  [!] Unknown option.")


def main():
    parser = argparse.ArgumentParser(description="Simulated Cube Commander for xArm 1S RViz Grid")
    parser.add_argument("--arm-id", default="xarm_1", help="xArm instance name (default: xarm_1)")
    parser.add_argument("--spawn-red", action="store_true", help="Spawn red cube and exit")
    parser.add_argument("--spawn-green", action="store_true", help="Spawn green cube and exit")
    parser.add_argument("--col", type=int, default=2, help="Grid column (0..6)")
    parser.add_argument("--row", type=int, default=8, help="Grid row (7..10)")
    parser.add_argument("--start", action="store_true", help="Trigger autonomous sorting start")
    args = parser.parse_args()

    rclpy.init()
    commander = SimCubeCommander(arm_id=args.arm_id)

    if args.spawn_red:
        commander.spawn_cube("red", col=args.col, row=args.row)
        time.sleep(0.5)
    elif args.spawn_green:
        commander.spawn_cube("green", col=args.col, row=args.row)
        time.sleep(0.5)
    elif args.start:
        commander.call_trigger(commander.cli_start_sorting)
    else:
        run_interactive(commander)

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
