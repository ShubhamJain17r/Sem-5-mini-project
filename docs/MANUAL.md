# Dual xArm 1S — Operator Manual & Debugging Guide

Comprehensive operational manual for running, testing, calibrating, and troubleshooting the Dual xArm 1S color-sorting robotic workcell.

---

## 🎯 Strict Environment Separation Policy

To ensure 100% reproducibility and prevent package conflicts:

| Environment | Purpose | Permitted Operations |
| :--- | :--- | :--- |
| **Host Python `venv`** | Python scripts, web config, telemetry, standalone direct tests | `python3 tools/web_config/server.py`<br>`python3 tools/telemetry_logger.py`<br>`python3 ros2_ws/src/sorting_arm/sorting_arm/*.py`<br>`python3 ros2_ws/src/sorting_arm/test/run_tests.py` |
| **Docker Container** (`./docker/run.sh`) | **ALL** ROS 2 operations | `colcon build`<br>`ros2 launch sorting_arm dual_xarm.launch.py`<br>`ros2 topic echo`<br>`ros2 service call` |

> [!IMPORTANT]
> **ROS 2 commands must NEVER be run directly on the host.**  
> Always execute them through `./docker/run.sh` or inside the Docker interactive shell (`./docker/run.sh bash`).

---

## 🛠️ Initial Setup

### 1. Host Python Virtual Environment (venv) Setup
On your host machine, create and activate the Python virtual environment:
```bash
# Run automated setup script
./setup_venv.sh

# Or manually:
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```
Whenever opening a new terminal for standalone Python scripts, activate the venv:
```bash
source venv/bin/activate
```

### 2. Docker Container Setup (For ROS 2)
Build the ROS 2 Humble development container:
```bash
./docker/build.sh
```
Verify container execution and hardware pass-through:
```bash
./docker/run.sh bash
```

---

## 🔌 Hardware Pre-Flight & Wiring Verification

Before powering on or executing motion commands:
1. **Power Supply**: Ensure both arms receive clean 7.4V (5A per arm) from the dual buck regulator hub. Check that common star ground is bonded.
2. **Controller Switch**: Flip the power toggle switch on each xArm base controller to **ON** (blue LED illuminates).
3. **USB Connections**:
   - `xArm 1` (Left/West arm): Connects to `/dev/ttyUSB0` (or first detected HID/serial node).
   - `xArm 2` (Right/East arm): Connects to `/dev/ttyUSB1` (or second detected HID/serial node).
4. **USB Permissions (Host)**:
   Ensure your host user belongs to the `dialout` and `plugdev` groups:
   ```bash
   sudo usermod -aG dialout,plugdev $USER
   sudo chmod 666 /dev/ttyUSB* /dev/hidraw* 2>/dev/null || true
   ```

---

## 🧪 Phase 1: Standalone Testing with Python (Host venv)

Run these tests in your host `venv` (`source venv/bin/activate`) to verify arm mechanics, limits, and IK before launching ROS 2.

### Step 1: Run Offline Test Suite (No Hardware Connection Required)
Verify kinematics, safety clamps, and trajectory interpolation algorithms:
```bash
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/test/run_tests.py
```
*Expected Result*: All 18 unit tests report `[PASS]` with 0 failures.

### Step 2: Test Single Arm Motion & Gripper
Test communication, upright reset pose, and claw opening/clamping:
```bash
# Test xArm 1
python3 ros2_ws/src/sorting_arm/sorting_arm/direct_test.py --arm xarm_1 --port auto

# Test xArm 2 (with aligned polarity)
python3 ros2_ws/src/sorting_arm/sorting_arm/direct_test.py --arm xarm_2 --port auto
```

### Step 3: Comprehensive Multi-Motion Test Suite (13 Single / 12 Dual Motions)
Execute calibrated single-arm motions or dual-arm cooperative routines:
```bash
# Execute ALL 13 single-arm motions sequentially on xArm 1
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --all

# Execute ALL 13 single-arm motions sequentially on xArm 2
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_2 --all

# Execute individual motion by ID (1 to 13):
# Motion 5: Standard Mid-Range Pick & Place (R=220mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 5

# Motion 6: Near-Field Shallow Pick (R=165mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 6

# Motion 7: Extended Far-Reach Pick (R=275mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 7

# Motion 8: Left-Quadrant Flank Diagonal Pick (R=210mm, θ=-30° West)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 8

# Motion 9: Right-Quadrant Midline Diagonal Pick (R=210mm, θ=+30° East)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 9

# Motion 10: Far-Corner Radial Sweep (R=265mm, θ=-25° to +25°, Z=20mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 10

# Motion 11: Multi-Altitude Standoff Staircase (Z=150 -> 90 -> 40 -> 12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 11

# Motion 12: Continuous 3-Point Tabletop Rapid Touch (West -> Center -> East)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 12

# Motion 13: 4-Color Flank Sorting Box Tour (Red -> Green -> Blue -> Yellow)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 13

# Execute ALL 12 dual-arm cooperative motions sequentially
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --dual --all
```

### Step 4: 5-Phase Pick-and-Place with Trajectory Smoothing & Stall Detection
Execute a full pick cycle with minimum-jerk polynomial interpolation and closed-loop position verification:
```bash
python3 ros2_ws/src/sorting_arm/sorting_arm/pick_and_place_test.py \
    --arm xarm_2 \
    --color red \
    --r 220.0 \
    --theta 0.0 \
    --z 12.5
```
*Features*:
- Dynamically ramps velocities via quintic interpolation (eliminating mechanical shudder).
- Queries real-time potentiometer ticks via command `0x15` to verify target arrival within $\pm 10\text{ ticks}$.
- If an obstacle blocks the claw, triggers emergency torque unload (`0x14`) immediately.

---

## 🌐 Phase 2: Web Configuration & Real-Time Diagnostics Tool

The local browser-based utility lets you inspect joint potentiometers, fine-tune deviations, edit waypoints, and atomically save YAML configs without manual file editing.

### 1. Launching the Web Server (Host venv)
```bash
source venv/bin/activate
python3 tools/web_config/server.py 8000
```
Open **`http://localhost:8000`** in Google Chrome or any modern browser.

### 2. Tabular Motor Configuration Editor
- **Live Table**: Displays Motor 1 to 6 for both `xArm 1` and `xArm 2`:
  - `Min (ticks / deg)` and `Max (ticks / deg)`: Physical travel safety clamps with real-time physical degree conversions ($0.24^\circ/\text{tick}$).
  - `Center (ticks / deg)`: 90° upright center position (nominal ~500 ticks / $120.0^\circ$).
  - `Dev`: Volatile mechanical horn trim offset in ticks ($\text{hardware\_ticks} = \text{theoretical\_ticks} + \text{dev}$).
  - `Dir`: Direction multiplier (`+1` standard).
  - `Live (ticks / deg)`: Continuous real-time potentiometer feedback polled from hardware.
  - `Err`: Current tracking error in ticks ($T_{\text{actual}} - T_{\text{target}}$).
  - `Z-Trim (mm)`: Dynamic Cartesian tabletop height trim in card header to compensate for base mounting/suction cup differences.
- **Inline Editing**: Modify values directly in the table. Degree labels update in real time as numbers are typed.
- **Atomic Save**: Click **"Save Configuration to YAML"** in the top-right corner. The server validates parameters, writes atomically to `dual_xarm_hardware.yaml`, and creates automatic `.bak` backup files.
- 📖 **Comprehensive Step-by-Step Tuning Guide**: Refer to [`docs/08_servo_limits_and_calibration_guide.md`](08_servo_limits_and_calibration_guide.md) for full procedural instructions on finding mechanical limits safely and preventing gear stalls.

### 3. Dynamic Hotplug Port Detection & Manual Controls
- **System Devices Bar**: The header displays all USB HID and Serial devices discovered by Linux in real-time.
- **Connect / Disconnect Toggle**: If an arm is plugged in after starting the server, click **"Connect"** on its card to bind and connect immediately, or rely on the 1.5s background auto-reconnect loop.
- **Emergency Stop Button**: Clicking the red **"Emergency Stop (Torque Cut)"** button dispatches packet `0x14` to instantly unload torque on all 12 motors.

### 4. Automated Potentiometer Drift Telemetry
- Click **"Run Immediate Telemetry Scan"** on Tab 3 to query all 12 joints, append records to `data/telemetry_drift_log.csv`, and review repeatability statistics.

---

## 🤖 Phase 3: Operating with ROS 2 in Docker (Docker Only)

All ROS 2 nodes, drivers, and coordinate arbiters run inside the Docker container.

### Step 1: Build the ROS 2 Workspace in Docker
```bash
./docker/run.sh bash -c "cd /workspace/ros2_ws && colcon build --symlink-install"
```

### Step 2: Launch Simultaneous Dual-Arm Drivers & Spatial Coordinator
```bash
./docker/run.sh ros2 launch sorting_arm dual_xarm.launch.py
```
This single launch file starts:
1. `/xarm_1/xarm_driver_node`: Hardware driver for xArm 1 (`/xarm_1/joint_commands`, `/xarm_1/joint_states`).
2. `/xarm_2/xarm_driver_node`: Hardware driver for xArm 2 (`/xarm_2/joint_commands`, `/xarm_2/joint_states`).
3. `/dual_arm_coordinator`: Real-time forward kinematics tracking, proximity alert, and spatial mutex arbiter for the shared central midline ($X_w \in [240\text{ mm}, 320\text{ mm}]$).

### Step 3: Inspect Real-Time ROS 2 Feedback
In a separate terminal, open a shell inside the running container:
```bash
./docker/run.sh bash
```
Inspect joint states:
```bash
# Echo live joint feedback for Arm 1
ros2 topic echo /xarm_1/joint_states

# Echo live joint feedback for Arm 2
ros2 topic echo /xarm_2/joint_states

# Monitor shared midline corridor lock status and arm end-effector coordinates
ros2 topic echo /dual_arm/corridor_status
```

### Step 4: Spatial Mutex Service Arbitration
When an arm approaches the shared midline picking zone ($X \approx 280\text{ mm}$), it requests corridor priority:
```bash
# Arm 1 requests corridor lock
ros2 service call /dual_arm/request_corridor_arm1 std_srvs/srv/Trigger

# Arm 1 releases corridor lock once clear
ros2 service call /dual_arm/release_corridor_arm1 std_srvs/srv/Trigger
```

### Step 5: ROS 2 Emergency Stop & Re-Enable
Trigger instantaneous motor torque cutoff via ROS 2 services:
```bash
# E-Stop Arm 1
ros2 service call /xarm_1/emergency_stop std_srvs/srv/Trigger

# Clear E-Stop and re-enable Arm 1
ros2 service call /xarm_1/re_enable std_srvs/srv/Trigger
```

---

## 📊 Phase 4: Long-Term Drift Tracking & Repeatability Analysis

To maintain sub-millimeter tabletop picking repeatability without manual paper logging:

### Running Telemetry via CLI (Host venv)
```bash
source venv/bin/activate

# Execute a single multi-joint scan and append to data/telemetry_drift_log.csv:
python3 tools/telemetry_logger.py --scan

# Perform 5 consecutive scans sampled 2 seconds apart:
python3 tools/telemetry_logger.py --samples 5 --interval 2.0

# Print existing historical statistics and maintenance alerts:
python3 tools/telemetry_logger.py --stats-only
```

### Statistical Metrics & Repeatability Formulas
- **Angular Resolution**: 1 servo tick $\approx 0.24^\circ$.
- **Fingertip Displacement at Reach $R = 250\text{ mm}$**:
  $$\Delta s = R \cdot \theta_{\text{rad}} = 250\text{ mm} \times \left(0.24^\circ \times \frac{\pi}{180^\circ}\right) \approx \mathbf{1.05\text{ mm}}$$
- **Sub-Millimeter Repeatability Target**:
  To guarantee fingertip picking error $< 1.0\text{ mm}$, joint position tracking must remain within $\mathbf{\le 1.0\text{ tick}}$!
- **Actionable Maintenance Schedule**:
  | Condition | Diagnostic Meaning | Corrective Action |
  | :--- | :--- | :--- |
  | $\sigma > 3.0\text{ ticks}$ | Joint mechanical backlash / loose horn spline | Tighten M3 center lock screw on servo horn |
  | $|\mu| > 5.0\text{ ticks}$ | Systematic mechanical drift or settling | Recalibrate zero position in xArm PC App or update `dual_xarm_hardware.yaml` |
  | Every 500 cycles / 7 days | Routine scheduled verification | Run 10-sample telemetry scan and inspect CSV drift slope |

---

## 🔍 Comprehensive Debugging & Troubleshooting Matrix

| Symptom / Error | Root Cause | Solution |
| :--- | :--- | :--- |
| **`No xArm device detected on USB HID or Serial interfaces`** | Controller powered off, loose USB cable, or unmapped device node. | 1. Verify 7.4V power switch on arm base is ON.<br>2. Run `ls /dev/ttyUSB* /dev/hidraw*` to confirm device appears.<br>3. Check `dmesg \| tail -n 20` for USB disconnects. |
| **`Permission denied: '/dev/ttyUSB0'` or `'/dev/hidraw*'`** | Current Linux user lacks read/write rights to serial/HID device nodes. | Run `sudo usermod -aG dialout,plugdev $USER` then log out and log back in, or run `sudo chmod 666 /dev/ttyUSB* /dev/hidraw*`. |
| **`STALL / OBSTRUCTION DETECTED! Emergency torque cutoff triggered`** | Joint tracking error remained $> 10\text{ ticks}$ after timeout (mechanical collision, stuck claw, or low voltage). | 1. Inspect physical arm for obstacle or cable entanglement.<br>2. Check 7.4V buck hub output voltage under load (voltage dip below 6.5V causes motor stall).<br>3. Reset via web UI or call `re_enable` service. |
| **`Port busy / [Errno 16] Device or resource busy`** | Another process (e.g. ROS 2 driver node or xArm PC App) is already holding the serial port. | Kill existing process holding port: `fuser -k /dev/ttyUSB0` or disconnect from the web tool before launching ROS 2. |
| **`IK Failed: Target unreachable`** | Requested coordinate lies outside reachable envelope or behind shoulder. | Bounded safe reach is $R \in [130\text{ mm}, 320\text{ mm}]$ and $Z \ge 10.0\text{ mm}$. Ensure target coordinates remain within the $400\times 220\text{ mm}$ arena floor. |
| **Web Config Server reports `Offline (Sim)`** | Server started before USB cable was connected. | Click **"Connect"** button on the arm card, or wait for the 1.5s auto-reconnect cycle to detect the port. |
| **Docker: `cannot connect to X server`** | Host X11 access control is blocking Docker GUI. | Run `xhost +local:root` on your host terminal before launching `./docker/run.sh`. |
| **Arm jerky or sudden torque spikes** | Open-loop commanding with discrete step jumps. | Ensure trajectory interpolation is active (`smooth: True` in `PickAndPlaceCommander` or using `generate_joint_trajectory()`). |
