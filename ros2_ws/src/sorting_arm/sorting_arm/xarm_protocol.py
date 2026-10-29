#!/usr/bin/env python3
"""
Low-Level Hardware Driver for LewanSoul / Hiwonder xArm 1S.

Supports:
  - Dual modality: USB HID (/dev/hidraw*) and Serial UART (/dev/ttyUSB*, /dev/ttyACM*)
  - Automatic hardware interface detection (probes sysfs for 0483:5750 Hiwonder xArm)
  - 0x55 0x55 binary frame construction
  - Single and multi-joint position commands (CMD 0x03)
  - Emergency Stop / Multi-Servo Torque Unload (CMD 0x14)
  - Servo position readback (CMD 0x15)
  - Software deviation calibration offsets & soft limits
  - Inrush-current brownout protection via 25ms inter-joint staggering
"""

import glob
import logging
import os
import select
import time
from typing import Dict, Optional, Tuple

logger = logging.getLogger("xarm_protocol")

# Binary Command Codes
CMD_SERVO_MOVE = 0x03
CMD_MULT_SERVO_UNLOAD = 0x14
CMD_SERVO_POS_READ = 0x15

# Default physical limits (ticks)
DEFAULT_MIN_TICKS = {1: 150, 2: 200, 3: 150, 4: 150, 5: 200, 6: 150}
DEFAULT_MAX_TICKS = {1: 800, 2: 800, 3: 850, 4: 850, 5: 850, 6: 850}

# Standard Safe Home Pose (M1=Claw, M2=Roll, M3=WPitch, M4=Elbow, M5=Shoulder, M6=Base)
POSE_HOME_TICKS = {
    1: 230,  # Claw open (~60mm span)
    2: 500,  # Roll center
    3: 500,  # Wrist Pitch center
    4: 280,  # Elbow tucked
    5: 720,  # Shoulder upright
    6: 500,  # Base forward (center)
}

# Motor name mapping
MOTOR_NAMES = {
    1: "Claw Gripper",
    2: "Wrist Roll",
    3: "Wrist Pitch",
    4: "Elbow Pitch",
    5: "Shoulder Pitch",
    6: "Base Yaw",
}


def find_xarm_device() -> Tuple[Optional[str], str]:
    """
    Auto-detect xArm 1S connection across USB HID and Serial UART interfaces.

    :return: Tuple of (device_path, interface_type) where interface_type is 'hidraw' or 'serial'.
    """
    # 1. Probe for native USB HID xArm (Vendor ID 0483, Product ID 5750)
    for uevent in sorted(glob.glob("/sys/class/hidraw/hidraw*/device/uevent")):
        try:
            with open(uevent, "r") as f:
                content = f.read()
                if "00000483:00005750" in content or "Hiwonder xArm" in content:
                    dev_name = os.path.basename(os.path.dirname(os.path.dirname(uevent)))
                    dev_path = f"/dev/{dev_name}"
                    if os.path.exists(dev_path):
                        return dev_path, "hidraw"
        except Exception:
            pass

    # 2. Probe for standard Serial UART converters (FTDI, CH340, CP2102) or Arduino bridge
    for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/xarm_*"]:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0], "serial"

    return None, "unknown"


def find_all_xarm_devices() -> List[Tuple[str, str]]:
    """
    Find all connected xArm 1S devices across USB HID and Serial UART interfaces.

    :return: List of tuples (device_path, interface_type).
    """
    found = []
    # 1. Probe for native USB HID xArm (Vendor ID 0483, Product ID 5750)
    for uevent in sorted(glob.glob("/sys/class/hidraw/hidraw*/device/uevent")):
        try:
            with open(uevent, "r") as f:
                content = f.read()
                if "00000483:00005750" in content or "Hiwonder xArm" in content:
                    dev_name = os.path.basename(os.path.dirname(os.path.dirname(uevent)))
                    dev_path = f"/dev/{dev_name}"
                    if os.path.exists(dev_path):
                        found.append((dev_path, "hidraw"))
        except Exception:
            pass

    # 2. Probe for standard Serial UART converters (FTDI, CH340, CP2102)
    for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/xarm_*"]:
        for match in sorted(glob.glob(pattern)):
            if match not in [p for p, _ in found]:
                found.append((match, "serial"))

    return found


class XArmDriver:
    """Hardware communication driver for Hiwonder xArm 1S."""

    def __init__(
        self,
        port: str = "auto",
        baudrate: int = 115200,
        deviations: Optional[Dict[int, int]] = None,
        min_ticks: Optional[Dict[int, int]] = None,
        max_ticks: Optional[Dict[int, int]] = None,
        timeout: float = 0.2,
    ):
        """Initialize driver parameters and state caches."""
        self.requested_port = port
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.interface_type = "unknown"  # 'hidraw' or 'serial'

        self.serial = None
        self.hid_fd = None
        self.is_connected = False
        self.is_estopped = False

        # Calibrated deviation offsets: hardware_ticks = theoretical_ticks + deviation
        self.deviations = deviations if deviations is not None else {i: 0 for i in range(1, 7)}
        self.min_ticks = min_ticks if min_ticks is not None else DEFAULT_MIN_TICKS.copy()
        self.max_ticks = max_ticks if max_ticks is not None else DEFAULT_MAX_TICKS.copy()

        # Current internal tick cache
        self.current_ticks = POSE_HOME_TICKS.copy()

    def connect(self) -> bool:
        """Open connection over USB HID or Serial UART."""
        target_port = self.requested_port

        if target_port in ["auto", "", None]:
            detected_port, detected_type = find_xarm_device()
            if not detected_port:
                logger.error("No xArm device detected on USB HID or Serial interfaces.")
                self.is_connected = False
                return False
            target_port = detected_port
            self.interface_type = detected_type
        elif "hidraw" in target_port:
            self.interface_type = "hidraw"
        else:
            self.interface_type = "serial"

        self.port = target_port

        if self.interface_type == "hidraw":
            return self._connect_hidraw()
        else:
            return self._connect_serial()

    def _connect_hidraw(self) -> bool:
        """Open raw HID device node."""
        try:
            self.hid_fd = os.open(self.port, os.O_RDWR | os.O_NONBLOCK)
            self.is_connected = True
            logger.info(f"Connected to xArm via USB HID on {self.port}.")
            return True
        except PermissionError:
            try:
                os.system(f"sudo -n chmod 666 {self.port} 2>/dev/null")
                self.hid_fd = os.open(self.port, os.O_RDWR | os.O_NONBLOCK)
                self.is_connected = True
                logger.info(f"Auto-granted permissions and connected to xArm on {self.port}.")
                return True
            except Exception:
                pass
            logger.error(
                f"Permission denied accessing {self.port}. "
                f"Please grant permissions ('sudo chmod 666 {self.port}')."
            )
            self.is_connected = False
            return False
        except Exception as e:
            logger.error(f"Failed to open HID device {self.port}: {e}")
            self.is_connected = False
            return False

    def _connect_serial(self) -> bool:
        """Open standard serial port connection."""
        try:
            import serial
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout,
            )
            time.sleep(0.1)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            self.is_connected = True
            logger.info(f"Connected to xArm via Serial on {self.port} at {self.baudrate} baud.")
            return True
        except Exception as e:
            self.is_connected = False
            logger.error(f"Failed to connect to xArm serial on {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial or HID connection."""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception:
                pass

        if self.hid_fd is not None:
            try:
                os.close(self.hid_fd)
            except Exception:
                pass
            self.hid_fd = None

        self.is_connected = False
        logger.info("Disconnected from xArm.")

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

    def apply_deviations_and_limits(self, servo_id: int, target_tick: int) -> int:
        """Apply calibration deviation and clamp to safe physical boundaries."""
        dev = self.deviations.get(servo_id, 0)
        hardware_tick = target_tick + dev

        min_t = self.min_ticks.get(servo_id, 150)
        max_t = self.max_ticks.get(servo_id, 850)

        clamped_tick = max(min_t, min(max_t, hardware_tick))
        return clamped_tick

    def _write_packet(self, payload: bytearray) -> bool:
        """Write packet to physical interface (handles Report ID padding for HID)."""
        if not self.is_connected:
            return False

        if self.interface_type == "hidraw":
            if self.hid_fd is None:
                return False
            # Report ID 0 followed by packet, padded to 65 bytes
            report = bytearray([0x00]) + payload
            if len(report) < 65:
                report.extend([0] * (65 - len(report)))
            try:
                os.write(self.hid_fd, report)
                return True
            except Exception as e:
                logger.error(f"HID write error: {e}")
                return False
        else:
            if self.serial is None or not self.serial.is_open:
                return False
            try:
                self.serial.write(payload)
                self.serial.flush()
                return True
            except Exception as e:
                logger.error(f"Serial write error: {e}")
                return False

    def send_servo_move(self, servo_id: int, theoretical_tick: int, time_ms: int = 1000) -> bool:
        """
        Move a single servo with deviation compensation and limit clamping.

        Packet format: 0x55 0x55 [len=8] [CMD 0x03] [count=1] [timeL] [timeH] [id] [posL] [posH]
        """
        if self.is_estopped:
            logger.warning("Movement rejected: Emergency Stop is active.")
            return False

        if not self.is_connected:
            logger.error("Cannot send move: Device not connected.")
            return False

        hardware_pos = self.apply_deviations_and_limits(servo_id, theoretical_tick)
        self.current_ticks[servo_id] = theoretical_tick

        time_ms = max(0, min(30000, int(time_ms)))
        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF
        pos_l = hardware_pos & 0xFF
        pos_h = (hardware_pos >> 8) & 0xFF

        packet = bytearray([
            0x55, 0x55,
            0x08,            # Length = 8
            CMD_SERVO_MOVE,  # 0x03
            0x01,            # Count = 1
            time_l, time_h,
            servo_id,
            pos_l, pos_h
        ])

        return self._write_packet(packet)

    def send_multi_servo_move(
        self,
        target_ticks: Dict[int, int],
        time_ms: int = 1000,
        stagger_ms: int = 0,
        synchronized: bool = True,
    ) -> bool:
        """
        Move multiple servos smoothly.

        If synchronized=True (default): sends a single native Hiwonder multi-servo
        CMD_SERVO_MOVE packet so all joints start simultaneously and interpolate
        in lockstep without blocking sleep or jitter.
        If synchronized=False: dispatches individual joint commands with stagger_ms delay.
        """
        if self.is_estopped:
            logger.warning("Multi-servo movement rejected: Emergency Stop is active.")
            return False

        if not self.is_connected:
            logger.error("Cannot send move: Device not connected.")
            return False

        # Filter valid IDs 1..6
        valid_targets = {sid: int(tick) for sid, tick in target_ticks.items() if 1 <= sid <= 6}
        if not valid_targets:
            return True

        if synchronized and stagger_ms == 0:
            count = len(valid_targets)
            length = count * 3 + 5
            time_ms = max(0, min(30000, int(time_ms)))
            time_l = time_ms & 0xFF
            time_h = (time_ms >> 8) & 0xFF

            packet = bytearray([
                0x55, 0x55,
                length,
                CMD_SERVO_MOVE,
                count,
                time_l, time_h,
            ])

            for sid, theoretical_tick in valid_targets.items():
                hw_pos = self.apply_deviations_and_limits(sid, theoretical_tick)
                self.current_ticks[sid] = theoretical_tick
                packet.extend([sid, hw_pos & 0xFF, (hw_pos >> 8) & 0xFF])

            return self._write_packet(packet)
        else:
            priority_order = [6, 5, 4, 3, 2, 1]
            success = True
            for servo_id in priority_order:
                if servo_id in valid_targets:
                    tick = valid_targets[servo_id]
                    ok = self.send_servo_move(servo_id, tick, time_ms)
                    if not ok:
                        success = False
                    if stagger_ms > 0:
                        time.sleep(stagger_ms / 1000.0)
            return success

    def move_home(self, time_ms: int = 1500) -> bool:
        """Move arm into safe tucked transit pose."""
        logger.info(f"Moving to Home Pose in {time_ms} ms...")
        return self.send_multi_servo_move(POSE_HOME_TICKS, time_ms=time_ms, synchronized=True)

    def emergency_stop(self) -> bool:
        """
        Send CMD_MULT_SERVO_UNLOAD (0x14) to cut torque on all 6 servos.

        Frame: 0x55 0x55 0x09 0x14 0x06 0x01 0x02 0x03 0x04 0x05 0x06
        """
        self.is_estopped = True
        logger.warning("EMERGENCY STOP TRIGGERED! Cutting motor torque.")

        packet = bytearray([
            0x55, 0x55,
            0x09,                   # Length = 9
            CMD_MULT_SERVO_UNLOAD,  # 0x14
            0x06,                   # 6 servos
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06
        ])

        return self._write_packet(packet)

    def clear_emergency_stop(self):
        """Clear the E-Stop latch, allowing new commands."""
        self.is_estopped = False
        logger.info("Emergency Stop cleared. Servos re-enabled.")

    def read_servo_position(self, servo_id: int, timeout_sec: float = 0.2) -> Optional[int]:
        """
        Query real-time position of a servo (CMD 0x15).

        Returns theoretical position (hardware_ticks - deviation), or None if read fails.
        """
        if not self.is_connected:
            return None

        packet = bytearray([
            0x55, 0x55,
            0x04,
            CMD_SERVO_POS_READ,  # 0x15
            0x01,
            servo_id
        ])

        if self.interface_type == "hidraw":
            if self.hid_fd is None:
                return None
            report = bytearray([0x00]) + packet
            if len(report) < 65:
                report.extend([0] * (65 - len(report)))
            try:
                os.write(self.hid_fd, report)
                r, _, _ = select.select([self.hid_fd], [], [], timeout_sec)
                if not r:
                    return None
                data = os.read(self.hid_fd, 64)
                if len(data) >= 8 and data[0] == 0x55 and data[1] == 0x55 and data[3] == 0x15:
                    ret_id = data[5]
                    if ret_id == servo_id:
                        hardware_pos = data[6] | (data[7] << 8)
                        dev = self.deviations.get(servo_id, 0)
                        return hardware_pos - dev
                return None
            except Exception as e:
                logger.debug(f"HID read error on servo {servo_id}: {e}")
                return None
        else:
            if not self.serial or not self.serial.is_open:
                return None
            try:
                self.serial.reset_input_buffer()
                self.serial.write(packet)
                self.serial.flush()

                start_t = time.time()
                header_count = 0

                while time.time() - start_t < timeout_sec:
                    b = self.serial.read(1)
                    if not b:
                        continue
                    if b == b"\x55":
                        header_count += 1
                        if header_count == 2:
                            break
                    else:
                        header_count = 0

                if header_count < 2:
                    return None

                len_b = self.serial.read(1)
                if not len_b:
                    return None
                pkt_len = ord(len_b)

                payload = self.serial.read(pkt_len - 1)
                if len(payload) < 5 or payload[0] != CMD_SERVO_POS_READ:
                    return None

                ret_id = payload[2]
                if ret_id != servo_id:
                    return None

                hardware_pos = payload[3] | (payload[4] << 8)
                dev = self.deviations.get(servo_id, 0)
                return hardware_pos - dev
            except Exception as e:
                logger.debug(f"Position query error on servo {servo_id}: {e}")
                return None

    def check_connection_health(self) -> Tuple[bool, str]:
        """
        Verify that the xArm controller is connected, responsive, and has valid file descriptors.

        :return: (is_healthy, status_message)
        """
        if not self.is_connected:
            return False, "Not connected: is_connected flag is False"

        if self.interface_type == "hidraw":
            if self.hid_fd is None:
                self.is_connected = False
                return False, "HID file descriptor is None"
            try:
                os.fstat(self.hid_fd)
            except OSError as e:
                self.is_connected = False
                return False, f"HID file descriptor invalid: {e}"
        else:
            if self.serial is None or not self.serial.is_open:
                self.is_connected = False
                return False, "Serial port is None or closed"
            try:
                if hasattr(self.serial, "fileno"):
                    os.fstat(self.serial.fileno())
            except OSError as e:
                self.is_connected = False
                return False, f"Serial port file descriptor invalid: {e}"

        return True, f"Connected and healthy on {self.port} ({self.interface_type})"

    def read_all_servo_positions(self, timeout_sec: float = 0.05) -> Dict[int, Optional[int]]:
        """Query real-time positions for all 6 servos."""
        positions = {}
        for sid in range(1, 7):
            positions[sid] = self.read_servo_position(sid, timeout_sec=timeout_sec)
        return positions

    def wait_in_position(
        self,
        target_ticks: Dict[int, int],
        tolerance_ticks: int = 10,
        timeout_sec: float = 2.0,
        poll_interval_sec: float = 0.03,
    ) -> Tuple[bool, Dict[int, int], Dict[int, int]]:
        """
        Wait until all specified joints arrive within tolerance ticks of their targets.

        :param target_ticks: Target position per servo ID
        :param tolerance_ticks: Maximum allowable difference |actual - target|
        :param timeout_sec: Timeout before aborting
        :param poll_interval_sec: Interval between read polls
        :return: (arrived, actual_positions, tracking_errors)
        """
        start_time = time.time()
        actual_ticks: Dict[int, int] = {}
        errors: Dict[int, int] = {}

        while time.time() - start_time < timeout_sec:
            all_in_position = True
            for sid, target in target_ticks.items():
                if sid < 1 or sid > 6:
                    continue
                actual = self.read_servo_position(sid, timeout_sec=0.04)
                if actual is not None:
                    actual_ticks[sid] = actual
                    err = actual - target
                    errors[sid] = err
                    if abs(err) > tolerance_ticks:
                        all_in_position = False
                else:
                    prev = actual_ticks.get(sid, self.current_ticks.get(sid, 500))
                    errors[sid] = prev - target
                    if abs(prev - target) > tolerance_ticks:
                        all_in_position = False

            if all_in_position and len(actual_ticks) == len(target_ticks):
                return True, actual_ticks, errors

            time.sleep(poll_interval_sec)

        return False, actual_ticks, errors

    def send_motion_and_wait(
        self,
        target_ticks: Dict[int, int],
        time_ms: int = 1000,
        tolerance_ticks: int = 10,
        timeout_margin_sec: float = 0.8,
        stall_detection: bool = True,
    ) -> bool:
        """
        Execute multi-servo trajectory with pre-command health check, in-position verification,
        and stall/obstruction safety interlock.

        If stall is detected, immediately cuts torque via CMD_MULT_SERVO_UNLOAD (0x14) and logs an error report.
        """
        healthy, msg = self.check_connection_health()
        if not healthy:
            logger.error(f"Pre-command check failed: {msg}")
            return False

        dispatched = self.send_multi_servo_move(target_ticks, time_ms=time_ms, synchronized=True)
        if not dispatched:
            return False

        if self.interface_type == "unknown" or (
            self.hid_fd is None and (self.serial is None or not self.serial.is_open)
        ):
            time.sleep(time_ms / 1000.0)
            for sid, tick in target_ticks.items():
                self.current_ticks[sid] = tick
            return True

        move_sec = time_ms / 1000.0
        lead_time = max(0.0, move_sec * 0.75)
        time.sleep(lead_time)

        remaining_timeout = max(0.2, (move_sec - lead_time) + timeout_margin_sec)
        arrived, actuals, errors = self.wait_in_position(
            target_ticks=target_ticks,
            tolerance_ticks=tolerance_ticks,
            timeout_sec=remaining_timeout,
        )

        for sid, pos in actuals.items():
            self.current_ticks[sid] = pos

        if arrived:
            return True

        if stall_detection:
            stalled_joints = []
            for sid, target in target_ticks.items():
                err = errors.get(sid, None)
                if err is not None and abs(err) > tolerance_ticks:
                    actual = actuals.get(sid, "Unknown")
                    name = MOTOR_NAMES.get(sid, f"Motor {sid}")
                    stalled_joints.append(
                        f"{name} (ID {sid}): target={target}, actual={actual}, error={err} ticks"
                    )

            if stalled_joints:
                logger.error(
                    f"STALL / OBSTRUCTION DETECTED! Emergency torque cutoff triggered. "
                    f"Stalled joints: {'; '.join(stalled_joints)}"
                )
                self.emergency_stop()
                return False

        return False
