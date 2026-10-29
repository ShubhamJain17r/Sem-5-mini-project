#!/usr/bin/env python3
"""
Lightweight Web Configuration & Real-Time Diagnostics Server for Dual xArm 1S.

Zero-dependency REST backend using standard library http.server and PyYAML:
  - Serves responsive HTML5 tabular configuration editor and live diagnostics UI
  - Real-time potentiometer querying (CMD 0x15) and tracking error calculation
  - Atomic, validated updates to config/dual_xarm_hardware.yaml and config/servo_calibration.yaml
  - Emergency Stop interlock and automated drift telemetry scanning
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import logging
import os
import shutil
import sys
import time
from typing import Any, Dict, Optional
import urllib.parse

# Ensure workspace packages can be imported
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "../.."))
ros_pkg_dir = os.path.join(project_root, "ros2_ws/src/sorting_arm")
if project_root not in sys.path:
    sys.path.insert(0, project_root)
if ros_pkg_dir not in sys.path:
    sys.path.insert(0, ros_pkg_dir)

import yaml
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("web_config_server")

CONFIG_HARDWARE_PATH = os.path.join(project_root, "config/dual_xarm_hardware.yaml")
CONFIG_CALIB_PATH = os.path.join(project_root, "config/servo_calibration.yaml")
INDEX_HTML_PATH = os.path.join(script_dir, "index.html")

# Global driver cache for arms
ARM_DRIVERS: Dict[str, Optional[XArmDriver]] = {
    "xarm_1": None,
    "xarm_2": None,
}


def get_arm_driver(arm_id: str, auto_reconnect: bool = True) -> Optional[XArmDriver]:
    """Retrieve or initialize driver instance for arm with hotplug auto-detection."""
    global ARM_DRIVERS
    driver = ARM_DRIVERS.get(arm_id)
    devs = load_deviations(CONFIG_CALIB_PATH, arm_id)

    hw = load_yaml(CONFIG_HARDWARE_PATH)
    arm_cfg = hw.get("manipulators", {}).get(arm_id, {})
    comm = arm_cfg.get("communication", {})
    fallback = comm.get("fallback_serial_device", "/dev/ttyUSB0" if arm_id == "xarm_1" else "/dev/ttyUSB1")
    pref_port = comm.get("preferred_port", "auto")

    if driver is None:
        driver = XArmDriver(port=pref_port, deviations=devs)
        ARM_DRIVERS[arm_id] = driver

    if driver.is_connected:
        healthy, _ = driver.check_connection_health()
        if not healthy:
            logger.info(f"{arm_id} connection lost. Disconnecting.")
            driver.disconnect()

    if auto_reconnect and not driver.is_connected:
        devices = find_all_xarm_devices()
        assigned_ports = [
            d.port for aid, d in ARM_DRIVERS.items() if d and d.is_connected and aid != arm_id
        ]

        target_candidate = None
        if os.path.exists(fallback) and fallback not in assigned_ports:
            target_candidate = fallback
        else:
            free_devices = [p for p, _ in devices if p not in assigned_ports]
            if free_devices:
                if arm_id == "xarm_2" and len(free_devices) > 1:
                    target_candidate = free_devices[1]
                else:
                    target_candidate = free_devices[0]

        if target_candidate:
            driver.requested_port = target_candidate
            driver.port = target_candidate
            driver.connect()

    return driver


def load_yaml(path: str) -> Dict[str, Any]:
    """Load YAML file safely."""
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def load_deviations(calib_path: str, arm_id: str) -> Dict[int, int]:
    """Load deviation offsets for specific arm."""
    data = load_yaml(calib_path)
    devs = data.get(arm_id, {}).get("servo_deviations", {})
    return {
        1: int(devs.get("motor_1_claw", 0)),
        2: int(devs.get("motor_2_wrist_roll", 0)),
        3: int(devs.get("motor_3_wrist_pitch", 0)),
        4: int(devs.get("motor_4_elbow", 0)),
        5: int(devs.get("motor_5_shoulder", 0)),
        6: int(devs.get("motor_6_base_yaw", 0)),
    }


def atomic_write_yaml(target_path: str, data: Dict[str, Any]) -> Tuple[bool, str]:
    """Atomically write YAML with backup preservation."""
    try:
        tmp_path = target_path + ".tmp"
        bak_path = target_path + ".bak"

        # Format YAML cleanly
        yaml_str = yaml.dump(data, default_flow_style=False, sort_keys=False)

        with open(tmp_path, "w") as f:
            f.write(yaml_str)
            f.flush()
            os.fsync(f.fileno())

        if os.path.exists(target_path):
            shutil.copy2(target_path, bak_path)

        os.replace(tmp_path, target_path)
        return True, "Success"
    except Exception as e:
        return False, str(e)


class WebConfigRequestHandler(BaseHTTPRequestHandler):
    """HTTP Request handler for Web Configuration & Diagnostics UI."""

    def _set_headers(self, content_type: str = "application/json", status: int = 200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_OPTIONS(self):
        self._set_headers(status=204)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        if path in ["/", "/index.html"]:
            if os.path.exists(INDEX_HTML_PATH):
                with open(INDEX_HTML_PATH, "rb") as f:
                    content = f.read()
                self._set_headers(content_type="text/html; charset=utf-8")
                self.wfile.write(content)
            else:
                self._set_headers(content_type="text/plain", status=404)
                self.wfile.write(b"index.html not found")
            return

        if path == "/api/config":
            self.handle_get_config()
            return

        if path == "/api/status":
            self.handle_get_status()
            return

        self._set_headers(status=404)
        self.wfile.write(json.dumps({"error": "Endpoint not found"}).encode())

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        content_len = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_len) if content_len > 0 else b"{}"

        try:
            payload = json.loads(body.decode("utf-8")) if body else {}
        except Exception:
            payload = {}

        if path == "/api/config":
            self.handle_save_config(payload)
            return

        if path == "/api/estop":
            self.handle_estop()
            return

        if path == "/api/connect":
            self.handle_connect(payload)
            return

        if path == "/api/disconnect":
            self.handle_disconnect(payload)
            return

        if path == "/api/telemetry/scan":
            self.handle_telemetry_scan()
            return

        self._set_headers(status=404)
        self.wfile.write(json.dumps({"error": "Endpoint not found"}).encode())

    def handle_get_config(self):
        """Return combined hardware and calibration configuration."""
        hw = load_yaml(CONFIG_HARDWARE_PATH)
        cal = load_yaml(CONFIG_CALIB_PATH)

        response = {
            "manipulators": hw.get("manipulators", {}),
            "calibrations": cal,
        }
        self._set_headers()
        self.wfile.write(json.dumps(response, indent=2).encode())

    def handle_save_config(self, payload: Dict[str, Any]):
        """Validate parameters and atomically save updates to YAML files."""
        hw_updates = payload.get("manipulators")
        cal_updates = payload.get("calibrations")

        if not hw_updates and not cal_updates:
            self._set_headers(status=400)
            self.wfile.write(json.dumps({"error": "No configuration payload provided"}).encode())
            return

        # Validation rules
        errors = []
        if hw_updates:
            for arm_name in ["xarm_1", "xarm_2"]:
                if arm_name in hw_updates:
                    motors = hw_updates[arm_name].get("motors", {})
                    for mid_str, mcfg in motors.items():
                        min_t = mcfg.get("min_tick")
                        max_t = mcfg.get("max_tick")
                        center_t = mcfg.get("nominal_center_tick")
                        mult = mcfg.get("direction_multiplier")

                        if min_t is not None and max_t is not None:
                            if not (100 <= min_t < max_t <= 900):
                                errors.append(f"{arm_name} {mid_str}: invalid limits [{min_t}, {max_t}]")
                        if center_t is not None:
                            if not (150 <= center_t <= 850):
                                errors.append(f"{arm_name} {mid_str}: center tick {center_t} out of range [150, 850]")
                        if mult is not None and mult not in [-1, 1]:
                            errors.append(f"{arm_name} {mid_str}: direction_multiplier must be +1 or -1")

        if cal_updates:
            for arm_name in ["xarm_1", "xarm_2"]:
                if arm_name in cal_updates:
                    devs = cal_updates[arm_name].get("servo_deviations", {})
                    for k, val in devs.items():
                        if not (-60 <= int(val) <= 60):
                            errors.append(f"{arm_name} deviation {k}={val} exceeds safe range [-60, +60]")

        if errors:
            self._set_headers(status=422)
            self.wfile.write(json.dumps({"error": "Validation failed", "details": errors}).encode())
            return

        # Perform atomic updates
        results = []
        if hw_updates:
            hw_current = load_yaml(CONFIG_HARDWARE_PATH)
            for arm_name, arm_val in hw_updates.items():
                if arm_name in hw_current.get("manipulators", {}):
                    if "motors" in arm_val:
                        for m_key, m_val in arm_val["motors"].items():
                            if m_key in hw_current["manipulators"][arm_name].get("motors", {}):
                                hw_current["manipulators"][arm_name]["motors"][m_key].update(m_val)
                    if "poses" in arm_val:
                        hw_current["manipulators"][arm_name]["poses"].update(arm_val["poses"])
            ok, msg = atomic_write_yaml(CONFIG_HARDWARE_PATH, hw_current)
            results.append(("hardware_yaml", ok, msg))

        if cal_updates:
            cal_current = load_yaml(CONFIG_CALIB_PATH)
            for arm_name, arm_val in cal_updates.items():
                if arm_name in cal_current:
                    if "servo_deviations" in arm_val:
                        cal_current[arm_name]["servo_deviations"].update(arm_val["servo_deviations"])
            ok, msg = atomic_write_yaml(CONFIG_CALIB_PATH, cal_current)
            results.append(("calib_yaml", ok, msg))

        success = all(ok for _, ok, _ in results)
        self._set_headers(status=200 if success else 500)
        self.wfile.write(json.dumps({"success": success, "results": results}).encode())

    def handle_get_status(self):
        """Query real-time status, device nodes, and potentiometer readings for both arms."""
        all_devices = find_all_xarm_devices()
        status = {
            "system_devices": [{"path": p, "type": t} for p, t in all_devices],
            "arms": {},
        }
        for arm_id in ["xarm_1", "xarm_2"]:
            driver = get_arm_driver(arm_id, auto_reconnect=True)
            connected = driver.is_connected if driver else False
            interface = driver.interface_type if driver else "disconnected"
            active_port = driver.port if driver else None
            estopped = driver.is_estopped if driver else False

            # Query potentiometers
            motor_status = {}
            for sid in range(1, 7):
                target = driver.current_ticks.get(sid, 500) if driver else 500
                actual = None
                if driver and connected:
                    actual = driver.read_servo_position(sid, timeout_sec=0.03)

                simulated = actual is None
                display_pos = actual if actual is not None else target
                error = display_pos - target

                motor_status[sid] = {
                    "motor_id": sid,
                    "target_tick": target,
                    "actual_tick": display_pos,
                    "tracking_error": error,
                    "is_simulated": simulated,
                }

            arm_dict = {
                "arm_id": arm_id,
                "connected": connected,
                "port": active_port,
                "interface": interface,
                "estopped": estopped,
                "motors": motor_status,
            }
            status["arms"][arm_id] = arm_dict
            status[arm_id] = arm_dict  # Backwards compatibility

        self._set_headers()
        self.wfile.write(json.dumps(status).encode())

    def handle_connect(self, payload: Dict[str, Any]):
        """Manually trigger connection or assign port for an arm."""
        arm_id = payload.get("arm_id", "xarm_1")
        port = payload.get("port", "auto")
        driver = get_arm_driver(arm_id, auto_reconnect=False)
        if driver:
            if driver.is_connected:
                driver.disconnect()
            driver.requested_port = port
            driver.port = port
            ok = driver.connect()
            self._set_headers()
            self.wfile.write(
                json.dumps(
                    {
                        "success": ok,
                        "connected": ok,
                        "port": driver.port,
                        "arm_id": arm_id,
                        "interface": driver.interface_type,
                    }
                ).encode()
            )
        else:
            self._set_headers(status=400)
            self.wfile.write(json.dumps({"success": False, "error": f"Unknown arm_id {arm_id}"}).encode())

    def handle_disconnect(self, payload: Dict[str, Any]):
        """Manually disconnect arm."""
        arm_id = payload.get("arm_id", "xarm_1")
        driver = get_arm_driver(arm_id, auto_reconnect=False)
        if driver and driver.is_connected:
            driver.disconnect()
        self._set_headers()
        self.wfile.write(json.dumps({"success": True, "arm_id": arm_id}).encode())

    def handle_estop(self):
        """Trigger emergency torque stop across both arms."""
        for arm_id in ["xarm_1", "xarm_2"]:
            driver = get_arm_driver(arm_id)
            if driver and driver.is_connected:
                driver.emergency_stop()

        self._set_headers()
        self.wfile.write(json.dumps({"success": True, "message": "Emergency Stop dispatched to all arms"}).encode())

    def handle_telemetry_scan(self):
        """Trigger an automated multi-servo telemetry scan and log to CSV."""
        from tools.telemetry_logger import run_telemetry_scan
        report = run_telemetry_scan()
        self._set_headers()
        self.wfile.write(json.dumps({"success": True, "report": report}).encode())


def run_server(port: int = 8000):
    """Run local configuration web server."""
    server_address = ("", port)
    httpd = HTTPServer(server_address, WebConfigRequestHandler)
    print(f"\n=======================================================")
    print(f"  Dual xArm Web Config & Diagnostics Server")
    print(f"  Running locally at: http://localhost:{port}")
    print(f"=======================================================\n")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping web server...")
    finally:
        httpd.server_close()


if __name__ == "__main__":
    port = 8000
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        port = int(sys.argv[1])
    run_server(port=port)
