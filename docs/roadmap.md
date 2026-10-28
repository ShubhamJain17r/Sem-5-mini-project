# Phase-Gate Milestones & Quality Verification Checklist

---

## 🎯 Architecture & Milestone Philosophy

This project is governed by a **Milestone-Driven Phase-Gate Architecture** rather than an inflexible weekly calendar schedule. 

Because university coursework, laboratory access schedules, exams, and hardware debugging introduce unpredictable time variations, **progress to the next phase is strictly gated by physical verification criteria (Gate Passes)**, rather than calendar deadlines. If a task requires extra iterative testing or mechanical re-alignment, the phase remains active until all gate criteria are satisfied.

```text
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 0: Actuation Baseline & Servo Zeroing (Arduino Hardware Bringup) │
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
    │ Phase 3: Single-Arm Autonomous Pick-and-Place (Complete MVP System)    │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 3: >= 90% Autonomous Sort Rate on 8 Cubes]
    ┌────────────────────────────────────────────────────────────────────────┐
    │ Phase 4: Dual-Arm Mutex Coordination & Pipelined Load Balancing        │
    └───────────────────────────────────┬────────────────────────────────────┘
                                        ▼ [Gate 4: Dual Arms Sorting Concurrently with Zero Deadlock]
```

---

## 📋 Phase 0: Actuation Baseline, Servo Zeroing & Hardware Bringup

**Primary Focus**: Immediate single-arm physical verification, removing mechanical assembly deviations, testing servo limits, and establishing analog jog control via Arduino Uno R4 WiFi before introducing software complexity.  
**Hardware Required**: 1× xArm 1s, Arduino Uno R4 WiFi, 7.4V / 5A DC power supply, Joystick, Potentiometer, Pushbutton, LEDs, common ground jumper wires.  
**Reference Document**: [`docs/05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) (Stepping Stone 5).

### Lab Action Checklist

- [ ] **Power Isolation & Shared Ground Check**:
  - [ ] Connect external 7.4V DC supply to xArm base power terminal. Measure voltage with DMM ($7.4\text{V} \pm 0.3\text{V}$).
  - [ ] Connect Arduino Uno R4 `GND` to xArm controller `GND`.
  - [ ] Verify Arduino is powered safely via USB from laptop (never feed 7.4V into Arduino 5V pin!).
- [ ] **Manufacturer Baseline Test via xArm App (20 Minutes Max)**:
  - [ ] Connect xArm to PC or smartphone via official Hiwonder/LewanSoul software.
  - [ ] Verify all 6 servos respond to manual slider commands.
  - [ ] Check for abnormal gear clicking, binding, or excessive heat.
- [x] **Mechanical Deviation Zeroing (`xarm_zero_calibration.ino`)**:
  - [x] Flash `arduino/xarm_zero_calibration/xarm_zero_calibration.ino` to Arduino Uno R4.
  - [x] Connect Arduino `Pin 0 (RX)` to xArm `TX`, and `Pin 1 (TX)` to xArm `RX`.
  - [x] Open Serial Monitor at `115200 baud`. Observe arm moving to nominal midpoint ($500\text{ ticks}$).
  - [x] Align joints collinearly / upright (Motor 6 Base, Motor 5 Shoulder, Motor 4 Elbow, Motor 3 Wrist Pitch, Motor 2 Wrist Roll, Motor 1 Claw).
  - [x] Trim deviations using `+`, `-`, `>`, `<` keys or xArm PC Calibration App. Export deviation array to `config/servo_calibration.yaml` (xarm_1: M1: 1, M2: -2, M3: -3, M4: -5, M5: -2, M6: -5 | xarm_2: M1: -2, M2: -4, M3: 2, M4: 3, M5: -3, M6: 2).
- [x] **Breadboard-Free Interactive Teleoperation & Reach Test (`xarm_direct_test.ino`)**:
  - [x] Flash `arduino/xarm_direct_test/xarm_direct_test.ino` (9600 baud `Serial1` to xArm, 115200 baud USB `Serial`).
  - [x] Motor mapping verified: Motor 1 (Claw), Motor 2 (Roll), Motor 3 (Wrist Pitch), Motor 4 (Elbow), Motor 5 (Shoulder), Motor 6 (Base).
  - [x] Multi-step Pick-and-Place reach test (`action` and `loop` commands) successfully validated across 120° base yaw swing.
- [x] **Claw Grip Characterization (Software Grasp Discrimination)**:
  - [x] Measure position ticks across physical claw travel:
    - xArm 1: Open ($60\text{ mm}$) @ $228\text{ ticks}$, Half-Open ($40\text{ mm}$) @ $501\text{ ticks}$, $25\text{ mm}$ Cube Grasp @ $610\text{ ticks}$, Closed Empty ($0\text{ mm}$) @ $791\text{ ticks}$.
    - xArm 2: Open ($60\text{ mm}$) @ $234\text{ ticks}$, Half-Open ($40\text{ mm}$) @ $498\text{ ticks}$, $25\text{ mm}$ Cube Grasp @ $602\text{ ticks}$, Closed Empty ($0\text{ mm}$) @ $776\text{ ticks}$.
  - [x] Confirm clear discrimination separation gap ($> 170\text{ ticks}$) between $25\text{ mm}$ cube grasp and empty closed state.

> [!IMPORTANT]
> ### 🏁 Gate 0 Exit Criteria (Prerequisites to Enter Phase 1)
> 1. Motor deviations for all 6 servos measured, eliminated, and recorded in a configuration table.
> 2. Arduino Uno R4 reliably jogs all 6 joints over hardware `Serial1` with zero packet dropouts.
> 3. Base servo swing verified to stay within $\approx 205^\circ \le 240^\circ$ without hitting mechanical stops.
> 4. Motor 6 position readback reliably distinguishes an empty claw from a clamped object.

---

## 📋 Phase 1: Single-Arm ROS 2 Driver & Analytical IK Integration

**Primary Focus**: Migrating control from Arduino to the laptop/PC ROS 2 environment; implementing closed-form analytical inverse kinematics; verifying sub-3 mm Cartesian positioning with a physical ruler.  
**Hardware Required**: 1× xArm 1s, Laptop with Ubuntu 22.04 + ROS 2 Humble, USB-to-UART cable (`/dev/ttyUSB0`), physical ruler, millimeter graph sheet.  
**Reference Document**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2) & [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) (Stepping Stone 4).

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
- [ ] **Physical Cartesian Positioning Accuracy Test**:
  - [ ] Place millimeter grid paper on table in front of arm base.
  - [ ] Command arm to 5 discrete grid points via ROS 2 CLI: `ros2 topic pub --once /arm1/task_goal ...`.
  - [ ] Measure physical claw grasp center position against paper grid with a precision ruler.
  - [ ] Record error vector $(\Delta X, \Delta Y, \Delta Z)$.
- [ ] **Empirical Z-Sag Calibration**:
  - [ ] Measure vertical tip droop at $R = 140, 220, 260, 300\text{ mm}$.
  - [ ] Apply linear compensation factor $k_{\text{sag}} \approx 0.07$ into `arm1_node.py`.
  - [ ] Re-test commanded heights: confirm claw tip height error is $\le 2.0\text{ mm}$ across the entire reach envelope.
- [ ] **5-Phase Trajectory Execution**:
  - [ ] Test the 5-phase trajectory sequence: Transit $\to$ Radial Standoff ($r-30\text{ mm}$) $\to$ Advance Grasp $\to$ Retract & Lift $\to$ Drop Swing ($205^\circ$).
  - [ ] Verify zero jerk and smooth motor acceleration profiles.

> [!IMPORTANT]
> ### 🏁 Gate 1 Exit Criteria (Prerequisites to Enter Phase 2)
> 1. ROS 2 node controls single xArm 1s reliably over USB serial without communication stalls.
> 2. Analytical IK solver executes in $< 0.1\text{ ms}$ on laptop CPU.
> 3. Measured physical tip positioning error is $\le 3.0\text{ mm}$ in $(X, Y)$ and $\le 2.0\text{ mm}$ in $Z$ across the entire operating radius ($R = 120\text{--}320\text{ mm}$).
> 4. 5-phase waypoint trajectory executes smoothly without tipping or jerking the baseplate.

---

## 📋 Phase 2: Monocular Vision, Lens Undistortion & Arena Rectification

**Primary Focus**: Setting up overhead boom camera; executing intrinsic lens calibration; tuning robust HSV segmentation; calculating planar homography exclusively framing the elevated arena.  
**Hardware Required**: Overhead USB Webcam (Logitech C270 / C920), Boom Stand ($65\text{--}75\text{ cm}$), 4× ArUco markers ($DICT\_4X4\_50$), Checkerboard calibration sheet ($9\times 6$), $25\text{ mm}$ colored cubes.  
**Reference Document**: [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) & [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md) (Stepping Stone 6).

### Lab Action Checklist

- [ ] **Camera Mounting & Hardware Exposure Lock**:
  - [ ] Clamp boom stand at $65\text{--}75\text{ cm}$ height centered over right arena $(X = 550\text{ mm}, Y = 300\text{ mm})$.
  - [ ] Lock exposure and white balance in hardware via V4L2:
    ```bash
    v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=156
    v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0 -c white_balance_temperature=4500
    ```
- [ ] **Intrinsic Camera Calibration (Lens Undistortion)**:
  - [ ] Capture 20 images of $9\times 6$ checkerboard across varying sensor angles.
  - [ ] Run `vision/calibrate_camera.py` to compute intrinsic matrix $K$ and distortion vector $D$.
  - [ ] Save parameters to `config/camera_calibration.yaml`. Verify straight lines remain straight after `cv2.undistort()`.
- [ ] **4-Point Coplanar ArUco Homography**:
  - [ ] Fix Markers 0, 1, 2, 3 flat on the wooden baseboard at arena corners ($Z = 0.0\text{ mm}$).
  - [ ] Run `vision/test_aruco_warp.py`. Confirm all 4 markers are detected with zero false IDs.
  - [ ] Rectify arena image to a metric grid ($2.0\text{ px/mm} \implies 780 \times 580\text{ pixels}$).
  - [ ] Confirm lateral flank trays and electronics area are cleanly segregated.
  - [ ] Lock homography matrix $H$ in memory to eliminate camera boom vibration noise.
- [ ] **HSV Color Threshold Tuning**:
  - [ ] Run `vision/tune_hsv.py` with test cubes on the arena surface.
  - [ ] Tune upper and lower bounds for Red (two masks across $0^\circ / 180^\circ$ boundary), Green, Blue, and Yellow.
  - [ ] Save calibrated ranges to `config/colors.yaml`.
- [ ] **Centroid Extraction & Spatial Accuracy Verification**:
  - [ ] Scatter 8 cubes at known millimeter locations on the arena.
  - [ ] Compare OpenCV contour centroid $(X_{\text{vision}}, Y_{\text{vision}})$ against ruler ground truth.
  - [ ] Verify maximum spatial localization error is strictly $< 2.0\text{ mm}$ across all 8 positions.

> [!IMPORTANT]
> ### 🏁 Gate 2 Exit Criteria (Prerequisites to Enter Phase 3)
> 1. Intrinsic lens distortion corrected; rectified arena boundaries are perfectly rectangular.
> 2. Planar homography matrix locked at startup; zero pixel jitter when table vibrates.
> 3. HSV color classification achieves $100\%$ accuracy on all 4 colors under laboratory ambient lighting.
> 4. Physical cube localization error is $< 2.0\text{ mm}$ in Cartesian arena coordinates.

---

## 📋 Phase 3: Single-Arm Autonomous Sorting (Standalone MVP)

**Primary Focus**: Closing the perception-action loop on one arm; verifying Nearest-First queue sorting and grip confirmation; achieving an independently gradable, fully functional autonomous sorting cell.  
**Hardware Required**: Single calibrated xArm 1s, Camera setup, Flat arena with foam border, 2×2 Flank Tray, 8–12 colored craft cubes.  
**Reference Document**: [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) & [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).

### Lab Action Checklist

- [ ] **ROS 2 Vision-to-Arm Pipeline Integration**:
  - [ ] Launch `camera_node`, `vision_node`, `scheduler_node`, and `arm2_node`.
  - [ ] Verify `/detected_objects` topic publishes real-time cube lists.
- [ ] **Nearest-First Corridor Collision Verification**:
  - [ ] Place Cube B ($r = 280\text{ mm}$) directly behind Cube A ($r = 160\text{ mm}$) along the same radial line.
  - [ ] Trigger pick cycle. Confirm `scheduler_node` prioritizes Cube A first.
  - [ ] Verify arm picks Cube A, deposits in box, and then picks Cube B with zero forearm collision.
- [ ] **Software Grip Confirmation Integration**:
  - [ ] Verify Motor 1 position feedback executes after clamping.
  - [ ] Intentionally misplace a cube so the claw closes on empty air: confirm arm detects miss ($P < 220\text{ ticks}$), aborts the drop swing, and alerts scheduler.
- [ ] **Multi-Color End-to-End Sorting Run**:
  - [ ] Scatter 8 cubes (2 Red, 2 Green, 2 Blue, 2 Yellow) across the arm's reach sector.
  - [ ] Run autonomous sorting sequence without human intervention.
  - [ ] Verify all 8 cubes are dropped cleanly into their dedicated color-coded tray compartments.
  - [ ] Record total cycle time and individual pick-to-drop duration.

> [!IMPORTANT]
> ### 🏁 Gate 3 Exit Criteria (Standalone MVP Complete)
> 1. Single arm autonomously sorts 8 scattered cubes with $\ge 90\%$ success rate without human intervention.
> 2. Nearest-First sorting eliminates all radial approach collisions.
> 3. Grip confirmation detects empty claw and prevents false-positive drop swings.
> 4. Complete system demonstration recorded on video as a baseline project submission.

---

## 📋 Phase 4: Dual-Arm Mutex Coordination & Load Balancing (Final Scaled System)

**Primary Focus**: Integrating the second arm (`xarm_1`); mounting rear electronics; establishing spatial crossover mutex for the shared central zone; testing pipelined concurrency.  
**Hardware Required**: 2× xArm 1s, Dual 7.4V/10A power hub, Jetson Orin Nano / RPi 5, Single Store-Cut Baseboard ($560 \times 520\text{ mm}$), 2× 3D-Printed 2×2 Flank Trays, 16–20 wooden cubes.  
**Reference Document**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md), [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md), and [`docs/07_fusion360_cad_modeling_guide.md`](07_fusion360_cad_modeling_guide.md).

### Lab Action Checklist

- [ ] **Dual-Arm Mechanical & Electrical Workcell Assembly**:
  - [ ] Place single pre-cut wooden baseboard ($560\times 520\text{ mm}$, $18\text{ mm}$ MDF) on workstation.
  - [ ] Mount Arm 1 (Left @ $X = 170\text{ mm}, Y = 360\text{ mm}$) and Arm 2 (Right @ $X = 390\text{ mm}, Y = 360\text{ mm}$) flat to the board with wood screws.
  - [ ] Apply self-adhesive foam weatherstripping tape to create $400\times 300\text{ mm}$ arena perimeter.
  - [ ] Mount rear electronics: connect dual buck converters with common ground star point and route cables neatly with surface clips.
  - [ ] Configure Linux udev rules to assign persistent symlinks: `/dev/xarm_arm1` and `/dev/xarm_arm2`.
- [ ] **Spatial Crossover Mutex Testing in Simulation**:
  - [ ] Inject synthetic target cubes in the central cooperative overlap zone ($X \in [220, 340\text{ mm}]$).
  - [ ] Confirm `scheduler_node` grants `acquire_zone` service to one arm while cleanly queueing the other.
  - [ ] Verify 5.0-second watchdog releases the zone if an arm fails.
- [ ] **Zero-Contention Drop Verification**:
  - [ ] Command Arm 1 to drop into Left Flank Tray 1 (~70° outward swing) while Arm 2 simultaneously drops into Right Flank Tray 2 (~70° outward swing).
  - [ ] Confirm both drop trajectories execute simultaneously with zero physical collision and zero software blocking.
- [ ] **Full Dual-Arm Continuous Sorting Benchmark**:
  - [ ] Scatter 16–20 colored craft blocks across the rectangular arena.
  - [ ] Launch full system: `ros2 launch sorting_scheduler system.launch.py`.
  - [ ] Benchmark throughput: compare dual-arm sort time vs. single-arm baseline (target: $\ge 1.7\times$ speedup).
  - [ ] Run 3 back-to-back sorting cycles without servo thermal cutoff or communication reset.
- [ ] **Operator Dashboard & Final Polish**:
  - [ ] Launch Web Dashboard / Foxglove Studio displaying live camera feed, arm states, and queue telemetry.
  - [ ] Route and clip all wiring neatly.

> [!IMPORTANT]
> ### 🏁 Gate 4 Exit Criteria (Final Project Complete)
> 1. Both arms operate concurrently with zero physical collisions across 3 consecutive 16-cube runs.
> 2. Spatial crossover mutex prevents contention in the shared midline zone with $100\%$ determinism.
> 3. Simultaneous Left/Right flank drop motions operate with zero mechanical or software interference.
> 4. High-resolution multi-angle demonstration video and final engineering project report prepared.
