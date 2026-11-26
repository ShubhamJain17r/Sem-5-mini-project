# Stepping Stone 8: Servo Limits & Web Portal Calibration Guide

Comprehensive guide for configuring, tuning, and verifying joint travel limits (Min/Max angles), zero-deviation offsets, and Cartesian depth trims using the real-time Web Configuration & Diagnostics Tool (`tools/web_config`).

---

## 🎯 Architectural Philosophy: 100% Dynamic Decoupling

In robotic manipulators, mechanical joint alignments are **volatile**:
- Horn splines can shift if an M3 screw is loosened or retightened.
- Servos experience mechanical settling under load.
- Temporary base mounting (such as silicone vacuum suction cups) introduces cantilever flex and tilt compared to a rigid wood baseboard.

To maintain sub-millimeter tabletop picking repeatability without constantly modifying source code, this architecture enforces strict **Dynamic Decoupling**:

```text
 ┌────────────────────────────────────────────────────────┐
 │       config/xarm_hardware.yaml (Static)               │
 │  - Link lengths (d1, L1, L2, L3)                       │
 │  - Safe travel clamps (min_tick, max_tick)             │
 │  - Calibrated joint centers (nominal_center_tick)      │
 │  - Dynamic Cartesian Z-depth trim (z_depth_trim_mm)    │
 │  - Optional software deviations & max deviation clamp   │
 └──────────────────────────┬─────────────────────────────┘
                            │
                            ▼ Direct USB / ROS 2 & Kinematics
 ┌────────────────────────────────────────────────────────┐
 │        Official Hiwonder xArm PC App / Web Config      │
 │  - Zero-position discovery & EEPROM deviation write   │
 │  - Real-time potentiometer feedback (CMD 0x15)         │
 │  - Live degree conversions & inline deviation trimming │
 │  - Atomic validated YAML writes with .bak preservation │
 └────────────────────────────────────────────────────────┘
```

---

## 📐 Angle to Servo Tick Conversion

Hiwonder / LewanSoul digital bus servos operate on a $240.0^\circ$ physical sweep resolved into $1000$ digital ticks ($0\text{ to }1000$):

$$\text{Angle (degrees)} = \text{Ticks} \times \left(\frac{240.0^\circ}{1000.0}\right) = \text{Ticks} \times \mathbf{0.24^\circ}$$

$$\text{Ticks} = \text{Angle (degrees)} \times \left(\frac{1000.0}{240.0^\circ}\right) \approx \text{Angle} \times \mathbf{4.1667}$$

### Standard Joint Angle Mapping Reference Table

| Motor ID | Joint Name | Physical Range | Nominal Center | Center Angle ($\theta_0$) | Safe Min Tick | Safe Max Tick | Motion Description |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **M1** | Claw Gripper | $0\text{--}60\text{ mm}$ span | $500\text{ ticks}$ | Half-open ($40\text{ mm}$) | $220$ ($60\text{ mm}$ open) | $800$ (Fully clamped) | Decreasing ticks opens claw; increasing ticks clamps. |
| **M2** | Wrist Roll | $\pm 72.0^\circ$ | $500\text{ ticks}$ | $0^\circ$ (Level jaws) | $200$ ($-72.0^\circ$ CW) | $800$ ($+72.0^\circ$ CCW) | Increasing ticks rolls CCW; decreasing rolls CW. |
| **M3** | Wrist Pitch | $-90^\circ\text{ to }+90^\circ$ | $500\text{ ticks}$ | $0^\circ$ (Collinear forearm) | $120$ ($-91.2^\circ$ back) | $865$ ($+87.6^\circ$ down) | Increasing tilts wrist forward/down; decreasing tilts up. |
| **M4** | Elbow Pitch | $0^\circ\text{ to }180^\circ$ | $500\text{ ticks}$ | $180^\circ$ (Straight arm) | $140$ ($0^\circ$ folded) | $875$ ($+90^\circ$ hyperext.) | Decreasing folds forearm forward; increasing straightens. |
| **M5** | Shoulder Pitch | $0^\circ\text{ to }180^\circ$ | $500\text{ ticks}$ | $90^\circ$ (Upright vertical) | $120$ ($180^\circ$ back) | $870$ ($0^\circ$ forward horiz) | Increasing tilts upper arm forward; decreasing tilts back. |
| **M6** | Base Yaw | $\pm 96.5^\circ$ | $502\text{ ticks}$ | $0^\circ$ (Midline South heading) | $100$ ($-96.5^\circ$ CW) | $900$ ($+95.5^\circ$ CCW) | Increasing rotates CCW; decreasing rotates CW. |

---

## 🌐 Setting Min / Max Angles via Web Portal

### 1. Launch the Web Configuration Server
Activate your host virtual environment and launch the server:
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Execute on Host Machine
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================
source venv/bin/activate
python3 tools/web_config/server.py 8000
```
Open **`http://localhost:8000`** in your web browser.

### 2. Live Tabular Interface Features
- **Min / Max (ticks / deg)**: Displays both the raw tick clamp and the live calculated physical angle in degrees.
- **Center (ticks / deg)**: Shows the upright zero reference angle tick (default 502 for Base).
- **Dev**: Software trim added to theoretical commands ($\text{HW} = \text{Theoretical} + \text{Dev}$).
- **Dir**: Rotation direction multiplier (`+1` standard).
- **Live (ticks / deg)**: Continuous real-time potentiometer feedback polled from the physical arm at $1.5\text{ s}$ intervals.
- **Z-Trim (mm)**: Dedicated tabletop height offset to align arm on uneven surfaces or suction cups.

### 3. Step-by-Step Procedure to Safely Find Hardware Limits
To calibrate limits on a joint without stripping gears or overheating motors:

1. **Unload Torque or Move Gently**:
   - Flip the power switch on the arm base, or click **"Emergency Stop"** in the web header to unload torque.
   - Gently rotate the target joint by hand to its physical mechanical stop (e.g. folded forearm or clamped claw).
2. **Observe the Live Column**:
   - Notice the live potentiometer value in the table (e.g. Motor 4 reads `142 (34.1°)`).
3. **Add a 10-Tick Safety Margin**:
   - Never set the soft limit directly against the mechanical hard stop! Always add a $\ge 10\text{ tick}$ ($\approx 2.4^\circ$) buffer.
   - If the hard stop was 142 ticks, enter `152` in the **Min** field.
4. **Repeat for Maximum Limit**:
   - Gently move the joint to the opposite extreme (e.g. fully extended).
   - If the stop reads `882 ticks`, enter `870` in the **Max** field.
5. **Save to YAML**:
   - Click the green **"Save Configuration to YAML"** button in the header.
   - The web server validates that $100 \le \text{Min} < \text{Max} \le 900$, writes atomically to `config/xarm_hardware.yaml`, and creates an automated timestamped `.bak` backup.

---

## 🔧 Resolving Arm Depth Misalignment (`z_depth_trim_mm`)

When operating the xArm on **temporary vacuum suction cups** or adapting to board thickness variations:
- Suction cup compression under cantilever arm reach ($R = 150\text{--}280\text{ mm}$) causes an unavoidable $1\text{--}3\text{ mm}$ base tilt.
- Variations in tabletop flatness can cause the gripper to reach slightly above or below ideal tabletop picking height.

### How to Tune `z_depth_trim_mm`:
1. In the Web Portal under the manipulator header (`xArm 1`), locate the **`Z-Trim`** field.
2. If the arm is hovering $2.0\text{ mm}$ too high above tabletop objects:
   - Increase `Z-Trim` by `+2.0 mm`.
   - In inverse kinematics, `solve_ik()` calculates:
     $$Z_{\text{effective}} = Z_{\text{target}} - Z_{\text{trim}} = 12.5\text{ mm} - 2.0\text{ mm} = 10.5\text{ mm}$$
   - This gently lowers the arm's pick elevation by exactly $2.0\text{ mm}$ in Cartesian space **without warping joint angle ratios**!
3. Click **"Save Configuration to YAML"**.
4. Run `--motion 5` or `--motion 6` to confirm that the claw pad contacts the tabletop with millimeter precision.

---

## 🛡️ Dynamic Deviation Guardrails

Deviation trims in `config/xarm_hardware.yaml` must never exceed realistic mechanical tolerances:
- **Max Deviation Threshold**: `config/xarm_hardware.yaml` defines `max_deviation_ticks: 25`.
- **Driver Clamping**: If an operator accidentally enters `+45` ticks in the web portal or YAML, `XArmDriver.apply_deviations_and_limits()` automatically clamps the applied deviation to $\pm 25\text{ ticks}$ and outputs a terminal warning.
- **Web Validation**: The web server rejects deviation payloads exceeding the configured threshold, preventing runaway joint divergence.

---

## 🚀 Verifying Limits with the Motion Test Suite

After updating limits or deviations in the web portal, verify full-envelope travel:

```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Execute on Host Machine
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================
source venv/bin/activate

# 1. Run offline calibration & dynamic limit unit tests (no physical arm needed):
pytest ros2_ws/src/sorting_arm/test/test_calibration_and_features.py -v

# 2. Verify all single-arm motion test routines on physical xarm_1:
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --all

# 3. Direct pick-and-place end-to-end integration test:
python3 ros2_ws/src/sorting_arm/sorting_arm/pick_and_place_test.py --arm xarm_1
```

*Expected Result*: All trajectories complete smoothly with zero IK boundary violations, zero gear buzzing, and perfect tabletop grasp depth.
