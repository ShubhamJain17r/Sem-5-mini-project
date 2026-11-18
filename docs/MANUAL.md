# Single xArm 1S — Operator Manual & Debugging Guide

Comprehensive operational manual for running, testing, calibrating, and troubleshooting the Single xArm 1S color-sorting robotic workcell.

---

## 🎯 Strict Environment Separation Policy

To ensure 100% reproducibility and prevent package conflicts:

| Environment | Purpose | Permitted Operations |
| :--- | :--- | :--- |
| **Host Python `venv`** | Python scripts, web config, telemetry, standalone direct tests | `python3 tools/web_config/server.py`<br>`python3 tools/telemetry_logger.py`<br>`python3 ros2_ws/src/sorting_arm/sorting_arm/*.py`<br>`python3 ros2_ws/src/sorting_arm/test/run_tests.py` |
| **Docker Container** (`./docker/run.sh`) | **ALL** ROS 2 operations | `colcon build`<br>`ros2 launch sorting_arm xarm.launch.py`<br>`ros2 topic echo`<br>`ros2 service call` |

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
1. **Power Supply**: Ensure the arm receives clean 7.4V (5A) from the dedicated buck regulator. Check that common star ground with host SBC/PC is bonded.
2. **Controller Switch**: Flip the power toggle switch on the xArm base controller to **ON** (blue LED illuminates).
3. **USB Connection**:
   - `xArm 1`: Connects to `/dev/ttyUSB0` (or first detected HID/serial node).
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
# Run 20 kinematics, driver, and trajectory unit tests
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/test/run_tests.py

# Run automated computer vision & ArUco homography tests
pytest ros2_ws/src/sorting_arm/test/test_vision_pipeline.py -v
```
*Expected Result*: All 26 unit tests report `[PASS]` with 0 failures.

### Step 2: Test Single Arm Motion & Gripper
Test communication, upright reset pose, and claw opening/clamping:
```bash
# Test xArm 1 (auto-detects port)
python3 ros2_ws/src/sorting_arm/sorting_arm/direct_test.py --arm xarm_1 --port auto
```

### Step 3: Comprehensive Multi-Motion Test Suite
Execute calibrated single-arm motions across the workspace:
```bash
# Execute ALL single-arm motions sequentially on xArm 1
PYTHONPATH=ros2_ws/src/sorting_arm:. python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --all

# Execute individual motion by ID (1 to 13):
# Motion 5: Standard Mid-Range Pick & Place (R=200mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 5

# Motion 6: Near-Field Shallow Pick (R=150mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 6

# Motion 7: Extended Far-Reach Pick (R=260mm, θ=0°, Z=12.5mm)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 7

# Motion 13: Rear Flank Dual Container Boxes Drop Tour (Red @ 650, Green @ 354)
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --motion 13
```

### Step 4: Pick-and-Place with Overhead Rear-Flip Transfer
Execute a full pick cycle with minimum-jerk polynomial interpolation and overhead rear-flip drop:
```bash
# Test Red cube pick and drop (Red container @ yaw 650)
python3 ros2_ws/src/sorting_arm/sorting_arm/pick_and_place_test.py --arm xarm_1 --color red

# Test Green cube pick and drop (Green container @ yaw 354)
python3 ros2_ws/src/sorting_arm/sorting_arm/pick_and_place_test.py --arm xarm_1 --color green
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
- **Live Table**: Displays Motor 1 to 6 for `xArm 1`:
  - `Min (ticks / deg)` and `Max (ticks / deg)`: Physical travel safety clamps with real-time physical degree conversions ($0.24^\circ/\text{tick}$).
  - `Center (ticks / deg)`: 90° upright center position (nominal ~500 ticks / $120.0^\circ$).
  - `Dev`: Volatile mechanical horn trim offset in ticks ($\text{hardware\_ticks} = \text{theoretical\_ticks} + \text{dev}$).
  - `Dir`: Direction multiplier (`+1` standard).
  - `Live (ticks / deg)`: Continuous real-time potentiometer feedback polled from hardware.
  - `Err`: Current tracking error in ticks ($T_{\text{actual}} - T_{\text{target}}$).
  - `Z-Trim (mm)`: Dynamic Cartesian tabletop height trim in card header to compensate for base mounting/suction cup differences.
- **Inline Editing**: Modify values directly in the table. Degree labels update in real time as numbers are typed.
- **Atomic Save**: Click **"Save Configuration to YAML"** in the top-right corner. The server validates parameters, writes atomically to `xarm_hardware.yaml`, and creates automatic `.bak` backup files.
- 📖 **Comprehensive Step-by-Step Tuning Guide**: Refer to [`docs/08_servo_limits_and_calibration_guide.md`](08_servo_limits_and_calibration_guide.md) for full procedural instructions on finding mechanical limits safely and preventing gear stalls.

### 3. Dynamic Hotplug Port Detection & Manual Controls
- **System Devices Bar**: The header displays all USB HID and Serial devices discovered by Linux in real-time.
- **Connect / Disconnect Toggle**: If the arm is plugged in after starting the server, click **"Connect"** on its card to bind and connect immediately, or rely on the 1.5s background auto-reconnect loop.
- **Emergency Stop Button**: Clicking the red **"Emergency Stop (Torque Cut)"** button dispatches packet `0x14` to instantly unload torque on all 6 motors.

### 4. Automated Potentiometer Drift Telemetry
- Click **"Run Immediate Telemetry Scan"** on Tab 3 to query all 6 joints, append records to `data/telemetry_drift_log.csv`, and review repeatability statistics.

---

## 📷 Phase 2: Computer Vision Calibration & Autonomous Sorting Bridge (Host venv)

The vision system uses pure OpenCV to rectify the $350 \times 600\text{ mm}$ arena to a $2.0\text{ px/mm}$ metric coordinate grid ($600 \times 800\text{ px}$) with strict color discrimination (Red target, Green distractor, Blue chassis bypass & spatial mask).

### Step 1: Camera Hardware Exposure & White Balance Lock
Ambient lighting changes in a university lab cause fatal HSV color drift. Always disable auto-exposure and auto-white-balance before launching vision scripts:
```bash
# Lock fixed exposure time
v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=156

# Lock fixed color temperature (4500K)
v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0 -c white_balance_temperature=4500
```

### Step 1.5: Camera Centering & Optical Leveling (`align_camera.py`)
Ensure the overhead camera is physically perpendicular and centered over the arena before tightening the boom clamp:
```bash
# Live interactive HUD:
python3 vision/align_camera.py --device /dev/video0

# Or offline synthetic verification:
python3 vision/align_camera.py --synthetic --save-report
```
- **HUD Guidance**: Follow on-screen prompts (`PAN LEFT/RIGHT`, `PAN FORWARD/BACK`, `TILT UP/DOWN`, `ROTATE CW/CCW`).
- Align the **Cyan arena center 'x'** (projective diagonal intersection of ArUco markers 0, 1, 2, 3) with the **Yellow optical center '+'**.
- Verify centering error is $\le 5.0\text{ mm}$, roll tilt is $\le 1.0^\circ$, and pitch/yaw keystoning ratios are within $[0.96, 1.04]$.
- Press **'s'** to save alignment verification to `config/camera_alignment.json` and a snapshot to `data/camera_alignment_snapshot.png`.

### Step 2: Intrinsic Lens Calibration (Undistortion)
Compute camera matrix $K$ and plumb-bob distortion coefficients $D$ using a $9 \times 6$ checkerboard:
```bash
# Live camera calibration:
python3 vision/calibrate_camera.py --device /dev/video0

# Or offline synthetic test:
python3 vision/calibrate_camera.py --synthetic
```
- Controls: Hold checkerboard, press `SPACE` to record sample (collect $\ge 15$ samples), press `c` to compute and atomically update `config/camera_calibration.yaml`.

### Step 3: 4-Point ArUco Planar Homography & Metric Rectification
Detect 4 arena corner ArUco markers (`DICT_4X4_50`, IDs: 0, 1, 2, 3, $50\times 50\text{ mm}$) at $(25, 425), (325, 425), (325, 25), (25, 25)$ in clockwise order and warp the picking arena to $600 \times 800\text{ px}$:
```bash
# Live verification:
python3 vision/test_aruco_warp.py --device /dev/video0

# Or offline synthetic verification:
python3 vision/test_aruco_warp.py --synthetic
```
- Controls:
  - Press `l` to **lock/freeze** the homography matrix $H$ (eliminates camera vibration and arm sweep occlusion).
  - Press `g` to display the $50\text{ mm}$ metric grid and place a physical millimeter ruler on the mat to verify coordinate alignment.
  - Press `s` to save the homography matrix to `config/camera_calibration.yaml`.

### Step 4: Interactive HSV Color Segmentation Tuner
Calibrate upper and lower color bounds under current lab lighting:
```bash
# Live tuner:
python3 vision/tune_hsv.py --device /dev/video0

# Or offline synthetic tuner:
python3 vision/tune_hsv.py --synthetic
```
- Controls: Keys `1` (Red), `2` (Green) to switch active color. Blue and Yellow are bypassed. Adjust the trackbars until target cubes are cleanly masked. Press `s` to export directly to `config/colors.yaml`.

### Step 5: Autonomous Vision-Guided Sorting Bridge (Phase 1 + Phase 2 Merge)
Executes end-to-end vision localization, Nearest-First queue ordering, and closed-form IK pick-and-place into the Rear Flank container box:
```bash
# Offline simulation / dry-run (no hardware required):
python3 tools/autonomous_pick_and_place.py --arm xarm_1 --synthetic --dry-run

# Single-shot pick of the nearest red cube:
python3 tools/autonomous_pick_and_place.py --arm xarm_1 --device /dev/video0 --port auto --single-shot

# Full continuous picking run:
python3 tools/autonomous_pick_and_place.py --arm xarm_1 --device /dev/video0 --port auto
```

---

## 🤖 Phase 3: Operating with ROS 2 in Docker (Docker Only)

All ROS 2 nodes, drivers, and coordinate pipelines run inside the Docker container.

### Step 1: Build the ROS 2 Workspace in Docker
```bash
./docker/run.sh bash -c "cd /workspace/ros2_ws && colcon build --symlink-install"
```

### Step 2: Launch Complete Single-Arm Sorting Workcell
```bash
./docker/run.sh ros2 launch sorting_arm xarm.launch.py auto_start:=true
```
This primary launch file starts:
1. `driver_node`: Hardware driver for xArm 1 (`/joint_commands`, `/joint_states`).
2. `vision_node`: Acquires frames from `/dev/video0`, computes 4-point ArUco homography warp, segments Red/Green/Blue/Yellow cubes, and publishes to `/vision/detected_objects` and `/vision/annotated_image`.
3. `autonomous_sorter`: Subscribes to detected objects, enforces Nearest-First picking order, solves analytical IK, and coordinates pick-and-place into the Left Flank 2×2 tray.

*(For perception-only testing without driving physical hardware, use `ros2 launch sorting_arm vision_sorting.launch.py dry_run:=true`).*

### Step 3: Inspect Real-Time ROS 2 Feedback
In a separate terminal, open a shell inside the running container:
```bash
./docker/run.sh bash
```
Inspect telemetry:
```bash
# Echo live detected cubes (JSON world & polar coordinates)
ros2 topic echo /vision/detected_objects

# Echo live joint feedback for xArm 1
ros2 topic echo /joint_states

# Echo live autonomous sorter status
ros2 topic echo /arm_status
```

### Step 4: ROS 2 Emergency Stop & Re-Enable
Trigger instantaneous motor torque cutoff via ROS 2 services:
```bash
# E-Stop xArm 1
ros2 service call /xarm_driver_node/emergency_stop std_srvs/srv/Trigger

# Clear E-Stop and re-enable xArm 1
ros2 service call /xarm_driver_node/re_enable std_srvs/srv/Trigger
```

---

## 📊 Phase 4: Long-Term Drift Tracking & Repeatability Analysis

To maintain sub-millimeter tabletop picking repeatability without manual paper logging:

### Running Telemetry via CLI (Host venv)
```bash
source venv/bin/activate

# Execute a single multi-joint scan and append to data/telemetry_drift_log.csv:
python3 tools/telemetry_logger.py --scan --arm xarm_1

# Perform 5 consecutive scans sampled 2 seconds apart:
python3 tools/telemetry_logger.py --samples 5 --interval 2.0 --arm xarm_1

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
  | $|\mu| > 5.0\text{ ticks}$ | Systematic mechanical drift or settling | Recalibrate zero position in xArm PC App or update `xarm_hardware.yaml` |
  | Every 500 cycles / 7 days | Routine scheduled verification | Run 10-sample telemetry scan and inspect CSV drift slope |

---

## 🔍 Comprehensive Debugging & Troubleshooting Matrix

| Symptom / Error | Root Cause | Solution |
| :--- | :--- | :--- |
| **`No xArm device detected on USB HID or Serial interfaces`** | Controller powered off, loose USB cable, or unmapped device node. | 1. Verify 7.4V power switch on arm base is ON.<br>2. Run `ls /dev/ttyUSB* /dev/hidraw*` to confirm device appears.<br>3. Check `dmesg \| tail -n 20` for USB disconnects. |
| **`Permission denied: '/dev/ttyUSB0'` or `'/dev/hidraw*'`** | Current Linux user lacks read/write rights to serial/HID device nodes. | Run `sudo usermod -aG dialout,plugdev $USER` then log out and log back in, or run `sudo chmod 666 /dev/ttyUSB* /dev/hidraw*`. |
| **`STALL / OBSTRUCTION DETECTED! Emergency torque cutoff triggered`** | Joint tracking error remained $> 10\text{ ticks}$ after timeout (mechanical collision, stuck claw, or low voltage). | 1. Inspect physical arm for obstacle or cable entanglement.<br>2. Check 7.4V buck regulator output voltage under load (voltage dip below 6.5V causes motor stall).<br>3. Reset via web UI or call `re_enable` service. |
| **`Port busy / [Errno 16] Device or resource busy`** | Another process (e.g. ROS 2 driver node or xArm PC App) is already holding the serial port. | Kill existing process holding port: `fuser -k /dev/ttyUSB0` or disconnect from the web tool before launching ROS 2. |
| **`IK Failed: Target unreachable`** | Requested coordinate lies outside reachable envelope or behind shoulder. | Bounded safe reach is $R \in [100\text{ mm}, 280\text{ mm}]$ and $Z \ge 10.0\text{ mm}$. Ensure target coordinates remain within the $260\times 140\text{ mm}$ arena floor ($X \in [70, 330], Y \in [40, 180]$). |
| **Web Config Server reports `Offline (Sim)`** | Server started before USB cable was connected. | Click **"Connect"** button on the arm card, or wait for the 1.5s auto-reconnect cycle to detect the port. |
| **Docker: `cannot connect to X server`** | Host X11 access control is blocking Docker GUI. | Run `xhost +local:root` on your host terminal before launching `./docker/run.sh`. |
| **Arm jerky or sudden torque spikes** | Open-loop commanding with discrete step jumps. | Ensure trajectory interpolation is active (`smooth: True` in `PickAndPlaceCommander` or using `generate_joint_trajectory()`). |
