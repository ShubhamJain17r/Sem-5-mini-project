#!/usr/bin/env python3
"""
Polar IK Game-Style Keyboard Teleoperation Node.

Controls LewanSoul / Hiwonder xArm 1S in Cylindrical / Polar coordinates (r, theta, z):
  - Left / Right Arrows (or A / E): Rotate Base Yaw (theta)
  - Up / Down Arrows (or W / S): Reach Out / In (r)
  - U / D: Gripper Vertical Height (z)
  - Space: Claw Clamp / Release toggle (250 <-> 750 ticks)
  - H: Return to Safe Home Pose
  - X: Emergency Stop (Torque Cutoff)
  - Q / Ctrl+C: Quit
Publishes to /xarm/joint_commands (sensor_msgs/msg/JointState).
"""

import select
import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sorting_arm.kinematics import IKSolution, solve_ik

STANDARD_JOINT_NAMES = [
    "claw_gripper",
    "wrist_roll",
    "wrist_pitch",
    "elbow_pitch",
    "shoulder_pitch",
    "base_yaw",
]


class NonBlockingTerminal:
    """Context manager for non-blocking single character terminal reads."""

    def __init__(self):
        """Store terminal file descriptor."""
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        """Set terminal into non-blocking cbreak mode with echo disabled."""
        self.old_settings = termios.tcgetattr(self.fd)
        new_settings = termios.tcgetattr(self.fd)
        # Disable canonical mode and echo, preserve output translation (\n -> \r\n)
        new_settings[3] &= ~(termios.ICANON | termios.ECHO)
        new_settings[6][termios.VMIN] = 0
        new_settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, new_settings)
        # Clear screen and hide cursor
        sys.stdout.write("\x1b[2J\x1b[?25l")
        sys.stdout.flush()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Restore terminal settings."""
        if self.old_settings:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        # Restore cursor
        sys.stdout.write("\x1b[?25h\r\n")
        sys.stdout.flush()

    def get_key(self, timeout: float = 0.05) -> Optional[str]:
        """Read a single key press or arrow key escape sequence."""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return None
        c = sys.stdin.read(1)
        if c == "\x1b":  # Escape sequence for arrow keys
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist2:
                c2 = sys.stdin.read(1)
                if c2 == "[":
                    rlist3, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if rlist3:
                        c3 = sys.stdin.read(1)
                        if c3 == "A":
                            return "KEY_UP"
                        elif c3 == "B":
                            return "KEY_DOWN"
                        elif c3 == "C":
                            return "KEY_RIGHT"
                        elif c3 == "D":
                            return "KEY_LEFT"
            return "ESC"
        return c


class PolarTeleopNode(Node):
    """ROS 2 Node providing interactive polar coordinate teleop for xArm."""

    def __init__(self):
        """Initialize teleop state, limits, and command publishers."""
        super().__init__("polar_teleop_node")

        # Publisher to driver
        self.cmd_pub = self.create_publisher(JointState, "/xarm/joint_commands", 10)

        # Current Cylindrical Target State
        self.r = 220.0        # mm (range ~150 to 290)
        self.theta = 0.0      # deg (range -80 to +80)
        self.z = 80.0         # mm (range 20 to 180)
        self.claw_tick = 750  # Open by default
        self.roll_tick = 500  # Level

        # Step sizes
        self.step_r = 5.0      # mm
        self.step_theta = 2.5  # deg
        self.step_z = 5.0      # mm

        self.last_solution: Optional[IKSolution] = None
        self.status_msg = "Ready. Press any control key to jog."

        # Compute initial pose
        self.update_ik()

    def update_ik(self) -> bool:
        """Solve IK for current target and return success."""
        sol = solve_ik(
            r=self.r,
            theta=self.theta,
            z=self.z,
            phi=None,  # Natural auto-pitch keeping wrist safe and as level as possible
            claw_tick=self.claw_tick,
            roll_tick=self.roll_tick,
        )
        self.last_solution = sol
        if sol.success:
            self.status_msg = f"OK: r={self.r:.1f}mm, th={self.theta:.1f}°, z={self.z:.1f}mm"
            return True
        else:
            self.status_msg = f"LIMIT: {sol.message}"
            return False

    def publish_target_joints(self):
        """Publish the current IK ticks to /xarm/joint_commands."""
        if not self.last_solution or not self.last_solution.success:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = STANDARD_JOINT_NAMES

        # [claw, roll, wpitch, elbow, shoulder, base]
        ticks = self.last_solution.ticks
        msg.position = [
            float(ticks[1]),
            float(ticks[2]),
            float(ticks[3]),
            float(ticks[4]),
            float(ticks[5]),
            float(ticks[6]),
        ]
        self.cmd_pub.publish(msg)

    def draw_hud(self):
        """Render live ANSI terminal HUD without flicker or staircase artifacts."""
        reach_pct = (self.r / 300.0) * 100.0
        bar = "#" * int(reach_pct / 5)
        claw_status = "OPEN (Release)" if self.claw_tick >= 500 else "CLAMPED (Grip)"

        lines = [
            "=" * 68,
            "          xArm 1S - Polar IK Keyboard Teleoperation (Stage 2)",
            "=" * 68,
            " Controls:",
            "   [Up/Down Arrow] or [W/S]   : Reach Forward / Backward (r ± 5 mm)",
            "   [Left/Right Arrow] or [A/E]: Rotate Base Yaw (theta ± 2.5°)",
            "   [U] / [D]                 : Gripper Height Up / Down (z ± 5 mm)",
            "   [Space]                   : Claw Grip / Release Toggle",
            "   [H]                       : Safe Home Pose",
            "   [X]                       : Emergency Stop (Torque Cutoff)",
            "   [Q] or [Ctrl+C]           : Quit Teleoperation",
            "-" * 68,
            " Target Polar State:",
            f"   Radial Reach (r) : {self.r:6.1f} mm   [{bar:<20}] ({reach_pct:.1f}%)",
            f"   Base Yaw (theta) : {self.theta:6.1f} deg  (0° = East / Forward)",
            f"   Height (z)       : {self.z:6.1f} mm   (Table mat = 0 mm)",
            f"   Claw State       : {claw_status} [{self.claw_tick} ticks]",
            "-" * 68,
        ]

        if self.last_solution and self.last_solution.success:
            t = self.last_solution.ticks
            a = self.last_solution.angles_deg
            pitch = a.get(0, 0.0)
            lines.extend([
                " Computed Joint Ticks (Theoretical):",
                f"   M6 Base Yaw:     {t[6]:4d} ticks ({a[6]:+5.1f}°)  |  "
                f"M3 Wrist Pitch: {t[3]:4d} ticks (pitch {pitch:+4.1f}°)",
                f"   M5 Shoulder:     {t[5]:4d} ticks ({a[5]:+5.1f}°)  |  "
                f"M2 Wrist Roll:  {t[2]:4d} ticks",
                f"   M4 Elbow Pitch:  {t[4]:4d} ticks ({a[4]:+5.1f}°)  |  "
                f"M1 Claw:        {t[1]:4d} ticks",
            ])
            status_color = "\x1b[32m"  # Green
        else:
            lines.append(" Computed Joint Ticks: [NO SOLUTION - KINEMATIC BOUNDARY]")
            status_color = "\x1b[31m"  # Red

        reset_color = "\x1b[0m"
        lines.extend([
            "-" * 68,
            f" Status: {status_color}{self.status_msg}{reset_color}",
            "=" * 68,
        ])

        # Overwrite in-place starting at top-left, clearing each line to eliminate flicker and staircase
        frame = "\x1b[H" + "".join(f"{line}\x1b[K\r\n" for line in lines)
        sys.stdout.write(frame)
        sys.stdout.flush()

    def run_teleop_loop(self):
        """Loop reading non-blocking keys and streaming commands."""
        self.draw_hud()
        self.publish_target_joints()

        with NonBlockingTerminal() as term:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                key = term.get_key(timeout=0.05)
                if not key:
                    continue

                dirty = False
                prev_r, prev_theta, prev_z = self.r, self.theta, self.z

                # Quit
                if key in ["q", "Q", "\x03"]:  # \x03 is Ctrl+C
                    sys.stdout.write("\r\nExiting Polar Teleoperation...\r\n")
                    sys.stdout.flush()
                    break

                # Radial Reach: In / Out
                elif key in ["KEY_UP", "w", "W"]:
                    self.r = min(300.0, self.r + self.step_r)
                    dirty = True
                elif key in ["KEY_DOWN", "s", "S"]:
                    self.r = max(130.0, self.r - self.step_r)
                    dirty = True

                # Base Yaw: Left / Right
                elif key in ["KEY_LEFT", "a", "A"]:
                    self.theta = min(80.0, self.theta + self.step_theta)
                    dirty = True
                elif key in ["KEY_RIGHT", "e", "E"]:
                    self.theta = max(-80.0, self.theta - self.step_theta)
                    dirty = True

                # Height: Up / Down
                elif key in ["u", "U"]:
                    self.z = min(190.0, self.z + self.step_z)
                    dirty = True
                elif key in ["d", "D"]:
                    self.z = max(15.0, self.z - self.step_z)
                    dirty = True

                # Claw toggle
                elif key == " ":
                    self.claw_tick = 250 if self.claw_tick >= 500 else 750
                    dirty = True

                # Safe Home
                elif key in ["h", "H"]:
                    self.r = 220.0
                    self.theta = 0.0
                    self.z = 80.0
                    self.claw_tick = 750
                    self.roll_tick = 500
                    dirty = True
                    self.status_msg = "Reset to Home Reference Pose."

                # Emergency Stop
                elif key in ["x", "X"]:
                    self.status_msg = "EMERGENCY STOP requested!"
                    self.draw_hud()
                    estop_msg = JointState()
                    estop_msg.header.stamp = self.get_clock().now().to_msg()
                    estop_msg.name = ["estop"]
                    estop_msg.position = [1.0]
                    self.cmd_pub.publish(estop_msg)
                    continue

                if dirty:
                    success = self.update_ik()
                    if not success:
                        self.r, self.theta, self.z = prev_r, prev_theta, prev_z
                        self.update_ik()
                    else:
                        self.publish_target_joints()

                    self.draw_hud()


def main(args=None):
    """Entry point for polar keyboard teleoperation node."""
    rclpy.init(args=args)
    node = PolarTeleopNode()
    try:
        node.run_teleop_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
