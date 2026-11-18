# Phase-Gate Milestones & Quality Verification Checklist

---

## 🎯 Architecture & Milestone Philosophy

This project is governed by a **Milestone-Driven Phase-Gate Architecture** rather than an inflexible weekly calendar schedule. 

Because university coursework, laboratory access schedules, exams, and hardware debugging introduce unpredictable time variations, **progress to the next phase is strictly gated by physical verification criteria (Gate Passes)**, rather than calendar deadlines. If a task requires extra iterative testing or mechanical re-alignment, the phase remains active until all gate criteria are satisfied.

```text
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 0: Actuation Baseline & Servo Zeroing (Hardware Bringup)         │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 0: Zero Mechanical Deviation & Limits Mapped]
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 1: Single-Arm ROS 2 Serial Driver & Analytical IK Execution      │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 1: Sub-3mm Cartesian Tip Accuracy with Ruler]
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 2: Monocular Vision, Lens Undistortion & Arena Rectification     │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 2: Sub-2mm Centroid Localization & Clean HSV]
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 3: Single-Arm Autonomous Pick-and-Place (Complete Workcell MVP)  │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 3: >= 90% Autonomous Sort Rate on 8 Cubes]
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 4: Full Workcell Assembly, Web Diagnostics & Continuous Benchmark│
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 4: 100% Reliable Continuous Sorting & Telemetry]
```

---

## 📋 Phase 0: Actuation Baseline, Servo Zeroing & Hardware Bringup

**Primary Focus**: Immediate single-arm physical verification, removing mechanical assembly deviations, testing servo limits, and establishing direct control before introducing ROS 2 complexity.  
**Hardware Required**: 1× xArm 1s, 7.4V / 5A DC power supply, host PC / USB connection.  
**Reference Document**: [`docs/05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) (Stepping Stone 5).

### Lab Action Checklist

- [ ] **Power Supply & Ground Check**:
  - [ ] Connect external 7.4V DC supply to xArm base power terminal. Measure voltage with DMM ($7.4\text{V} \pm 0.3\text{V}$).
  - [ ] Check common star ground between power supply and controller.
- [ ] **Manufacturer Baseline Test via xArm App (20 Minutes Max)**:
  - [ ] Connect xArm to PC via official Hiwonder/LewanSoul software.
  - [ ] Verify all 6 servos respond to manual slider commands.
  - [ ] Check for abnormal gear clicking, binding, or excessive heat.
- [x] **Mechanical Deviation Zeroing & EEPROM Hardware Calibration**:
  - [x] Connect xArm 1s directly to PC via USB cable.
  - [x] Open official Hiwonder xArm PC App and reset arm to nominal midpoint ($500\text{ ticks}$).
  - [x] Align joints collinearly / upright (Motor 6 Base, Motor 5 Shoulder, Motor 4 Elbow, Motor 3 Wrist Pitch, Motor 2 Wrist Roll, Motor 1 Claw).
  - [x] Trim deviations using xArm PC App sliders and write permanently into servo EEPROM registers.
  - [x] Record travel limits and nominal centers into unified `config/xarm_hardware.yaml`.
- [x] **Direct Python / USB Driver Teleoperation & Reach Test**:
  - [x] Implement direct USB communication (`direct_test.py` and `xarm_protocol.py`).
  - [x] Motor mapping verified: Motor 1 (Claw), Motor 2 (Roll), Motor 3 (Wrist Pitch), Motor 4 (Elbow), Motor 5 (Shoulder), Motor 6 (Base).
  - [x] Multi-step Pick-and-Place reach test successfully validated across workspace.
- [x] **Claw Grip Characterization (Software Grasp Discrimination)**:
  - [x] Measure position ticks across physical claw travel:
    - xArm 1: Open ($60\text{ mm}$) @ $228\text{ ticks}$, Half-Open ($40\text{ mm}$) @ $501\text{ ticks}$, $25\text{ mm}$ Cube Grasp @ $610\text{ ticks}$, Closed Empty ($0\text{ mm}$) @ $791\text{ ticks}$.
  - [x] Confirm clear discrimination separation gap ($> 170\text{ ticks}$) between $25\text{ mm}$ cube grasp and empty closed state.

> [!IMPORTANT]
> ### 🏁 Gate 0 Exit Criteria (Prerequisites to Enter Phase 1)
> 1. Motor deviations for all 6 servos measured, eliminated in EEPROM, and recorded in unified hardware configuration (`config/xarm_hardware.yaml`).
> 2. Direct USB driver reliably jogs all 6 joints over USB HID / Serial with zero packet dropouts.
> 3. Base servo swing verified to stay within safe tick envelope without hitting mechanical stops.
> 4. Motor 1 position readback reliably distinguishes an empty claw from a clamped object.

---

## 📋 Phase 1: Single-Arm ROS 2 Driver & Analytical IK Integration (✅ COMPLETE)

**Primary Focus**: Migrating control to the ROS 2 environment; implementing closed-form analytical inverse kinematics; verifying sub-3 mm Cartesian positioning and expanding motion coverage.  
**Hardware Required**: 1× xArm 1s, Laptop with Ubuntu 22.04 + ROS 2 Humble, USB-to-UART cable (`/dev/ttyUSB0`), physical ruler, millimeter graph sheet.  
**Reference Document**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2), [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) (Stepping Stone 4), & [`docs/08_servo_limits_and_calibration_guide.md`](08_servo_limits_and_calibration_guide.md) (Stepping Stone 8).

### Lab Action Checklist

- [x] **Docker Container & Isolated ROS 2 Workspace (Stepping Stone 0)**:
  - [x] Build multi-arch container (`docker/Dockerfile`, `docker/build.sh`, `docker/run.sh`).
  - [x] Configure Intel GPU `/dev/dri`, USB serial, and video forwarding.
- [x] **ROS 2 Workspace & Driver Package Build**:
  - [x] Create workspace `ros2_ws/src` and build `sorting_arm` package with `colcon build --symlink-install`.
  - [x] Implement `driver_node.py` and `xarm_protocol.py` with auto HID/serial detection, synchronized multi-servo dispatch, and E-stop service.
  - [x] Implement keyboard polar teleoperation node (`polar_teleop_node.py`).
- [x] **Analytical Inverse Kinematics Verification**:
  - [x] Implement `kinematics.py` closed-form polar solver (cylindrical coordinates $r, \theta, z$).
  - [x] Add near-field wrist pitch safe heuristics and 4-compartment target resolver (`get_compartment_target`).
  - [x] Verify reachability and mathematical roundtrip consistency via automated unit test suite (`ros2_ws/src/sorting_arm/test/test_kinematics.py`).
- [x] **Physical Cartesian Positioning Accuracy Test**:
  - [x] Command arm to discrete grid points via standalone motion suite (`motion_test_suite.py`).
  - [x] Verified single-arm motion test routines sequentially via `--all`.
  - [x] Confirmed grasp repeatability across near ($R=150\text{ mm}$), mid ($R=200\text{ mm}$), and extended far ($R=260\text{ mm}$) envelopes.
- [x] **Dynamic Tabletop Depth & Base Tilt Compensation (`z_depth_trim_mm`)**:
  - [x] Added dynamic Cartesian tabletop depth trim parameter (`z_depth_trim_mm`) in `KinematicParameters` and `xarm_hardware.yaml`.
  - [x] Unified servo parameters into `xarm_hardware.yaml`, eliminating separate volatile calibration files.
- [x] **Expanded Trajectory Execution**:
  - [x] Verified full sequence: Transit $\to$ Multi-Standoff $\to$ Near/Mid/Far Picks $\to$ Flank Tray Tour.
  - [x] Verified smooth minimum-jerk acceleration profiles and zero gear stall.

> [!IMPORTANT]
> ### 🏁 Gate 1 Exit Criteria (Passed ✅ - Proceeding to Phase 2)
> 1. ROS 2 driver controls single xArm 1s reliably over USB serial without communication stalls.
> 2. Analytical IK solver executes in $< 0.1\text{ ms}$ on CPU.
> 3. Measured physical tip positioning error is $\le 3.0\text{ mm}$ in $(X, Y)$ and $\le 2.0\text{ mm}$ in $Z$ across the operating radius ($R = 100\text{--}280\text{ mm}$).
> 4. Single-arm motion test suite executes smoothly without tipping or jerking the baseplate.

---

## 📋 Phase 2: Monocular Vision, Lens Undistortion & Arena Rectification

**Primary Focus**: Setting up overhead boom camera; executing intrinsic lens calibration; tuning robust HSV segmentation; calculating planar homography framing the active picking arena ($260\times 140\text{ mm}$).  
**Hardware Required**: Overhead USB Webcam (Logitech C270 / C920), Scrap aluminum stand ($H \approx 650\text{ mm}$), 4× ArUco markers ($DICT\_4X4\_50$), Checkerboard calibration sheet ($9\times 6$), $25\text{ mm}$ colored cubes.  
**Reference Document**: [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) & [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md) (Stepping Stone 6).

- [x] **Software Perception Architecture & Automated Test Suite**:
  - [x] Implement core 4-point ArUco homography & HSV segmentation pipeline (`vision/arena_vision_pipeline.py`).
  - [x] Implement checkerboard camera calibration script (`vision/calibrate_camera.py`).
  - [x] Implement ArUco verification & metric grid test utility (`vision/test_aruco_warp.py`).
  - [x] Implement real-time 6-trackbar HSV color tuner GUI (`vision/tune_hsv.py`).
  - [x] Implement vision-guided autonomous pick-and-place bridge (`tools/autonomous_pick_and_place.py`).
  - [x] Implement ROS 2 perception and sorter nodes (`vision_node.py` and `autonomous_sorter_node.py`).
  - [x] Verify offline mathematical and perception test suite (`test_vision_pipeline.py`, 6/6 passed).
- [ ] **Camera Mounting & Hardware Exposure Lock (Lab Hardware)**:
  - [ ] Anchor aluminum stand at rear center $(X = 200\text{ mm}, Y = 400\text{ mm})$ with camera nadir over arena center $(X = 200\text{ mm}, Y = 110\text{ mm})$ at $H \approx 650\text{ mm}$.
  - [ ] Lock exposure and white balance in hardware via V4L2:
    ```bash
    v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=156
    v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0 -c white_balance_temperature=4500
    ```
- [ ] **Intrinsic Camera Calibration (Lens Undistortion)**:
  - [ ] Capture 20 images of $9\times 6$ checkerboard across varying sensor angles (`python3 vision/calibrate_camera.py`).
  - [ ] Compute intrinsic matrix $K$ and distortion vector $D$ and save to `config/camera_calibration.yaml`.
- [ ] **4-Point Coplanar ArUco Homography**:
  - [ ] Fix Markers 0, 1, 2, 3 flat on the wooden baseboard at arena corners: $(70, 180), (330, 180), (70, 40), (330, 40)$ ($Z = 0.0\text{ mm}$).
  - [ ] Run `python3 vision/test_aruco_warp.py`. Confirm all 4 markers are detected and lock homography with `l`.
  - [ ] Verify metric grid alignment against physical millimeter ruler ($\le 1.5\text{ mm}$ error).
  - [ ] Save locked homography matrix $H$ with key `s`.
- [ ] **HSV Color Threshold Tuning**:
  - [ ] Run `python3 vision/tune_hsv.py` with test cubes on the arena surface.
  - [ ] Tune upper and lower bounds for Red and Green and save to `config/colors.yaml` with key `s`.
- [ ] **Centroid Extraction & Spatial Accuracy Verification**:
  - [ ] Scatter cubes at known millimeter locations on the active arena.
  - [ ] Compare detected centroids against ground truth and verify spatial error $< 2.0\text{ mm}$.

> [!IMPORTANT]
> ### 🏁 Gate 2 Exit Criteria (Prerequisites to Enter Phase 3)
> 1. Intrinsic lens distortion corrected; rectified arena boundaries are perfectly rectangular ($600 \times 800\text{ px}$ at $2.0\text{ px/mm}$).
> 2. Planar homography matrix locked at startup; zero pixel jitter when table vibrates.
> 3. HSV color discrimination cleanly segments Red and Green targets, and ignores Blue chassis.
> 4. Physical cube localization error is $< 2.0\text{ mm}$ in Cartesian arena coordinates.

---

## 📋 Phase 3: Single-Arm Autonomous Dual-Color (Red & Green) Sorting

**Primary Focus**: Closing the perception-action loop on xArm 1; verifying Nearest-First queue sorting and grip confirmation; sorting Red blocks (to Red box @ 650 ticks) and Green blocks (to Green box @ 354 ticks) via overhead rear-flip transfer while ignoring blue chassis.  
**Hardware Required**: Single calibrated xArm 1s, Camera setup, $7\times 12$ hand-drawn grid arena with foam border, dual Rear Flank container boxes (Red & Green), Red and Green target cubes.  
**Reference Document**: [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) & [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).

### Lab Action Checklist

- [ ] **ROS 2 Vision-to-Arm Pipeline Integration**:
  - [ ] Launch complete system: `ros2 launch sorting_arm xarm.launch.py auto_start:=true`.
  - [ ] Verify `/vision/detected_objects` topic publishes real-time cube lists with `color` and `targetable` flags for both Red and Green.
- [ ] **Nearest-First Corridor Collision Verification**:
  - [ ] Place Cube B ($r = 250\text{ mm}$) directly behind Cube A ($r = 160\text{ mm}$) along the same radial azimuth.
  - [ ] Trigger pick cycle. Confirm `autonomous_sorter` prioritizes Cube A first.
  - [ ] Verify arm picks Cube A, deposits via overhead rear-flip into designated container box, and then picks Cube B with zero forearm collision.
- [ ] **Software Grip Confirmation Integration**:
  - [ ] Verify Motor 1 position feedback executes after clamping.
  - [ ] Intentionally misplace a cube so the claw closes on empty air: confirm arm detects miss, aborts the drop transit, and alerts sorter.
- [ ] **Dual-Color Autonomous Sorting Run**:
  - [ ] Scatter Red and Green target cubes across active arena rows 7..10.
  - [ ] Run autonomous sorting sequence without human intervention.
  - [ ] Verify all Red cubes are picked and deposited into the Red container box at $(50, 550)$ with yaw tick $650$.
  - [ ] Verify all Green cubes are picked and deposited into the Green container box at $(300, 550)$ with yaw tick $354$.
  - [ ] Verify blue arm chassis and reflections are completely ignored.
  - [ ] Record total cycle time and individual pick-to-drop duration.

> [!IMPORTANT]
> ### 🏁 Gate 3 Exit Criteria (Standalone MVP Complete)
> 1. Single arm autonomously sorts both Red and Green cubes with $\ge 90\%$ success rate.
> 2. Nearest-First sorting eliminates all radial approach collisions.
> 3. Grip confirmation detects empty claw and prevents false-positive drop swings.
> 4. Complete system demonstration recorded on video as a baseline project submission.

---

## 📋 Phase 4: Full Workcell Assembly, Web Diagnostics & Continuous Benchmark

**Primary Focus**: Final physical integration on the $350\times 600\text{ mm}$ platform board; wiring management; web portal diagnostic tuning; long-term repeatability telemetry.  
**Hardware Required**: 1× xArm 1s, Single 7.4V/5A power supply, Jetson Orin Nano / RPi 5, Platform Board ($350 \times 600\text{ mm}$) with $7\times 12$ drawn grid, dual Rear Flank container boxes, Red and Green craft cubes.  
**Reference Document**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) and [`docs/08_servo_limits_and_calibration_guide.md`](08_servo_limits_and_calibration_guide.md).

### Lab Action Checklist

- [ ] **Workcell Mechanical & Electrical Assembly**:
  - [ ] Place platform board ($350\times 600\text{ mm}$, $7\times 12$ grid) on workstation.
  - [ ] Mount xArm 1 in Rows 3..5, Cols 2..4 centered at $(X = 175\text{ mm}, Y = 375\text{ mm})$ with heading $-90.0^\circ$ (South).
  - [ ] Place dual container boxes in Rows 0..1: Red box at Cols 0..1 $(X = 50\text{ mm}, Y = 550\text{ mm})$, Green box at Cols 5..6 $(X = 300\text{ mm}, Y = 550\text{ mm})$.
  - [ ] Apply self-adhesive foam weatherstripping tape to create active arena boundary.
  - [ ] Mount rear electronics ($Y \in [500, 600\text{ mm}]$): connect 7.4V/5A buck regulator with star ground and route cables with surface clips.
- [ ] **Web Portal & Drift Telemetry Verification**:
  - [ ] Launch Web Config portal (`python3 tools/web_config/server.py 8000`).
  - [ ] Verify live potentiometer polling on all 6 joints and validate E-stop responsiveness.
  - [ ] Run automated telemetry scan (`python3 tools/telemetry_logger.py --scan --arm xarm_1`) and log baseline statistics.
- [ ] **Full Continuous Picking Benchmark**:
  - [ ] Scatter Red and Green target blocks across the arena.
  - [ ] Launch full system: `ros2 launch sorting_arm xarm.launch.py auto_start:=true`.
  - [ ] Run 3 back-to-back sorting cycles without servo thermal cutoff or communication reset.
- [ ] **Final Polish & Operator Documentation**:
  - [ ] Route and clip all wiring neatly with zero overhang.
  - [ ] Capture high-resolution demo video and finalize project report.

> [!IMPORTANT]
> ### 🏁 Gate 4 Exit Criteria (Final Project Complete)
> 1. Single arm autonomously sorts Red and Green cubes across 3 consecutive runs with 100% completion.
> 2. Zero mechanical binding, zero cable snagging, and clean overhead rear-flip drop motions to yaw ticks 650 and 354.
> 3. Web portal and telemetry logging fully verified with repeatable sub-millimeter positioning accuracy.
> 4. Demonstration video and engineering report finalized for submission.
