# 🤖 Single xArm 1S Color-Sorting Robot

An autonomous robotic sorting system powered by **ROS 2 Humble**, **OpenCV**, and a single **LewanSoul / Hiwonder xArm 1S** 6-DoF articulated bus-servo manipulator. The system detects randomly scattered colored wooden craft blocks on a flat rectangular arena, computes analytical closed-form inverse kinematics, and sorts objects into a dedicated 4-compartment color-coded flank sorting tray.

---

## 📸 System Architecture & Visual Renders

![Realistic 3D Perspective Render of Workcell](docs/assets/images/workcell_perspective_render.jpg)

![Workcell Multi-View Engineering Blueprint](docs/assets/images/workcell_multiview_blueprint.jpg)

![High-Precision 1:1 Chart Paper Drafting Layout](docs/assets/images/chartpaper_drafting_sheet.png)

![3D CAD Render of 2x2 Quadrant Color Sorting Tray](docs/assets/images/tray_3d_print_cad.png)

---

## 📌 Project Overview

This project automates color-based object sorting using computer vision, analytical kinematics, and minimum-jerk trajectory execution under real-world physical and mechanical constraints:

1. **Central Manipulator Mounting**:
   - Single xArm 1S manipulator centered at $(X = 200.0\text{ mm}, Y = 280.0\text{ mm}, Z = 0.0\text{ mm})$ with nominal heading $-90.0^\circ$ (South, facing $-Y$ directly into the picking arena).
   - Full coverage over the entire picking arena with optimal arm reach ($R \in [130, 260\text{ mm}]$) without joint singularities.
2. **Compact Flat Baseboard ($400.0\text{ mm} \times 400.0\text{ mm} \times 18.0\text{ mm}$)**:
   - Single store-cut MDF or plywood board with zero woodworking cuts required.
   - High mass ($\approx 2.2\text{ kg}$) prevents table slipping, eliminates joint flex, and absorbs servo vibrations.
3. **Flat Rectangular Arena ($Z = 0.0\text{ mm}$, $260.0\text{ mm} \times 140.0\text{ mm}$)**:
   - Centered directly in front of the arm ($X \in [70.0, 330.0\text{ mm}]$, $Y \in [40.0, 180.0\text{ mm}]$). Sits directly flat on the baseboard at $Z = 0.0\text{ mm}$.
   - Self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) forms a compliant retaining perimeter that retains $25\text{ mm}$ cubes without jamming the claw.
   - *Clearance Verified*: $Y = 180\text{ mm}$ leaves a safe $+23\text{ mm}$ buffer before the robot metal baseplate ($Y = 203\text{ mm}$), ensuring 100% collision-free reachability.
4. **Immediate Left Flank 2×2 Sorting Tray ($90^\circ$ Clockwise Swing)**:
   - Dedicated 3D-printed 2×2 quadrant sorting tray ($80\times 80\times 35\text{ mm}$) centered on the Left Flank at $(X = 50.0\text{ mm}, Y = 280.0\text{ mm}, Z = 0.0\text{ mm})$.
   - Outward $90^\circ$ clockwise swing from the arena heading ($r = 150.0\text{ mm}$ to tray center, drop yaw tick $\approx 145$), minimizing transit time.
5. **Table-Coplanar ArUco Markers ($Z = 0.0\text{ mm}$)**:
   - 4 matte vinyl ArUco stickers (`DICT_4X4_50`, IDs 0..3) applied directly flat to the wooden board at the four corners of the arena:
     - Marker 0: $(70.0, 180.0)$ [Rear-Left]
     - Marker 1: $(330.0, 180.0)$ [Rear-Right]
     - Marker 2: $(70.0, 40.0)$ [Front-Left]
     - Marker 3: $(330.0, 40.0)$ [Front-Right]
   - Planar homography rectifies the arena to an exact $520 \times 280\text{ px}$ image at $2.0\text{ px/mm}$ with zero elevation distortion.
6. **Minimal 3D Printing (< 45 g Total Filament)**:
   - Only a single $2\times 2$ quadrant sorting tray unit is 3D-printed ($< 42\text{ g}$ filament). Overhead camera uses a welded scrap aluminum door stand ($700\times 400\text{ mm}$) with a 10" LED ring light.

---

## 📐 Workspace Field Layout

```text
┌────────────────────────────────────────────────────────────────────────┐
│                      ◄─── REAR OF WORKSPACE ───►                       │
│                                                                        │
│                  [Clamp-On Overhead Camera Boom Arm]                   │
│            ┌─────────────────────────────────────────────┐             │
│            │        REAR ELECTRONICS BAY (Y=350..400)    │             │
│            │        (Jetson Orin / RPi 5 + Buck Hub)     │             │
│            └──────────────────────┬──────────────────────┘             │
│                                   │                                    │
│   ┌─────────────────┐             ▼                                    │
│   │  Tray 1 (2x2)   │          xArm 1S                                 │
│   │  (Left Flank)   │      (X=200, Y=280)                              │
│   │  (X=50, Y=280)  │      [Base 140x154]                              │
│   └─────────────────┘             │  +23mm Clearance                   │
│        ▲             ═════════════╪═══════════════════════             │
│  90°   │  ◤ ArUco #0              │                       ArUco #1 ◥   │
│  Swing │  (70, 180)               \      RECTANGULAR      (330, 180)   │
│        ▼                          │     PICKING ARENA     │            │
│                                   │      (260 x 140)      │            │
│                                   │   (Flat Z = 0.0 mm)   │            │
│                                   │     (Foam Border)     │            │
│                                   /                       \            │
│           ◣ ArUco #2 ─────────────┴───────────────────────┴─ ArUco #3 ◢ │
│           (70, 40)                                          (330, 40)  │
│                                                                        │
│                      ◄─── FRONT OF WORKSPACE ───►                      │
│                                   ▲                                    │
│                          (Operator Position)                           │
└────────────────────────────────────────────────────────────────────────┘

  Base Topology:   Single store-cut flat wooden board (400 x 400 x 18 mm MDF)
  Vision Focus:    100% of camera pixels warped onto the 260x140 mm picking arena
  Drop Travel:     Clean 90° clockwise outward swing to Left Flank 2x2 tray (r = 150 mm)
```

---

## 🛠 Hardware Specifications

| Component           | Specification                                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------------------------------- |
| **Manipulator**     | 1× LewanSoul / Hiwonder xArm 1S (Centered at $X=200\text{ mm}, Y=280\text{ mm}$, South heading $-90.0^\circ$)    |
| **Link Lengths**    | Base-to-Shoulder ($d_1$): $65\text{ mm}$ · Upper Arm ($L_1$): $101\text{ mm}$ · Forearm ($L_2$): $95\text{ mm}$ · Tool ($L_3$): $150\text{--}168\text{ mm}$ |
| **Gripper**         | 2-finger parallel claw ($55\text{ mm}$ opening, Motor 2 axial roll, Motor 1 clamp)                              |
| **Compute**         | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (Ubuntu 22.04 LTS, rear electronics bay)                  |
| **Camera**          | Overhead camera with Invicto 10" LED ring light on welded scrap aluminum stand ($700\times 400\text{ mm}$) @ 65 cm nadir |
| **Baseboard**       | Single store-cut MDF board ($400\text{ mm} \times 400\text{ mm} \times 18\text{ mm}$, zero woodworking cuts)   |
| **Picking Arena**   | Flat rectangular surface ($Z = 0.0\text{ mm}$, $260\times 140\text{ mm}$) with $8\text{--}10\text{ mm}$ foam tape border|
| **Collection Bin**  | 1× 3D-printed 2×2 sorting tray ($80\times 80\times 35\text{ mm}$) on Left Flank at $(X=50, Y=280)$              |
| **Electronics Area**| Rear section of board ($Y \approx 350\text{--}400\text{ mm}$), short cable runs with surface wire clips         |
| **Fiducials**       | 4× matte vinyl ArUco stickers (`DICT_4X4_50`, IDs 0..3) coplanar with arena floor at $Z = 0.0\text{ mm}$       |
| **Objects**         | 1-inch ($25.4\text{ mm}$) wooden craft cubes ($14\text{--}16\text{ g}$) in ultra-matte Red, Green, Blue, Yellow  |
| **Power Supply**    | Single 7.4V / 5A DC buck regulator with common star ground                                                      |
| **Communication**   | Direct USB HID / USB-UART serial bus (`/dev/ttyUSB0`) @ 115200 / 9600 baud                                      |

> [!NOTE]
> **Joint Naming vs. Hiwonder Hardware IDs**:
> In mathematical kinematics ($\theta_1\text{--}\theta_6$), joints are indexed from base to claw ($\theta_1 = \text{Base Yaw}, \theta_2 = \text{Shoulder}, \theta_3 = \text{Elbow}, \theta_4 = \text{Wrist Pitch}, \theta_5 = \text{Wrist Roll}, \theta_6 = \text{Claw Clamp}$).
> On the physical Hiwonder xArm 1s bus servo daisy chain, hardware IDs are addressed tip-to-base: **ID 1 = Claw Gripper**, **ID 2 = Wrist Roll**, **ID 3 = Wrist Pitch**, **ID 4 = Elbow Pitch**, **ID 5 = Shoulder Pitch**, **ID 6 = Base Yaw**. All code and configuration files explicitly maintain this mapping.

---

## 💻 Software Stack & Architecture

- **Operating System**: Linux (Ubuntu 22.04 LTS)
- **Middleware**: ROS 2 Humble Hawksbill
- **Computer Vision**: OpenCV (ArUco planar homography, HSV color masking, contour moments)
- **Kinematics**: Analytical closed-form Inverse Kinematics + Distance-to-Pitch LUT
- **Web Dashboard**: HTML5, CSS3, JavaScript REST API

### ROS 2 Graph & Node Roles

```text
[ Overhead Camera ]
       │
       ▼
 [ camera_node ] ───► /camera/image_raw
                           │
                           ▼
                    [ vision_node ] (ArUco Homography + HSV Color Detection)
                           │
                           ▼ /detected_objects
              [ autonomous_sorter_node ] (Nearest-First Radial Queue + State Machine)
                           │
                           ▼ /xarm_1/joint_commands
                  [ xarm_driver_node ] (Serial / HID Protocol Driver)
                           │
                           ▼
                      [ Serial 0 ] (/dev/ttyUSB0)
```

| Node                     | Responsibility                                                                                                                  |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| `camera_node`            | Streams raw video frames from overhead camera to `/camera/image_raw`                                                           |
| `vision_node`            | Detects 4 arena ArUco markers, rectifies arena to metric mm grid, runs HSV thresholding, publishes `/detected_objects`         |
| `autonomous_sorter_node` | Prioritizes targets via Nearest-First ascending radius, computes 5-phase trajectories, handles grip validation & state machine   |
| `driver_node`            | Dispatches servo move packets over USB serial/HID, reads position feedback, monitors communication health & handles E-Stop     |
| `web_config`             | Serves real-time web portal on port 8000 for motor calibration, live potentiometer readouts, and CSV drift telemetry logging     |

---

## 📚 Technical Documentation Index

The project documentation is organized into modular **Stepping-Stone Engineering Specifications** and system integration guides in the [`docs/`](docs/) directory:

### Stepping-Stone Specifications

- 📄 **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](docs/00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container development workflow, Intel GPU acceleration, USB hardware mapping, and ROS 2 Humble package build standards.
- 📄 **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](docs/01_arena_and_layout.md) — Flat store-cut baseboard ($400\times 400\text{ mm}$), central single xArm 1S mounting, rectangular arena ($Z=0\text{ mm}$), foam retaining border, left flank 2x2 tray, and table-coplanar ArUco fiducials.
- 📄 **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](docs/02_kinematics_and_grasp_planning.md) — Complete mathematical derivations, DH parameters, analytical IK formulas ($\theta_1\text{--}\theta_5$), top-down reach limit proof ($158.7\text{ mm}$), dual-regime wrist roll control, 5-phase trajectory waypoint profiling, torque/friction physics, and reference Python IK solver.
- 📄 **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](docs/03_computer_vision_and_calibration.md) — Camera mounting optics, camera alignment and leveling tool (`align_camera.py`), 4-point ArUco planar homography ($2.0\text{ px/mm}$), hardware exposure/WB locks, HSV color segmentation, spatial moments centroiding, and Python perception pipeline.
- 📄 **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](docs/04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, single-arm autonomous sorting pipeline, Nearest-First radial queueing, joint trajectory generation, and production Python nodes.
- 📄 **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](docs/05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol (binary/ASCII), direct USB-HID/UART bringup, xArm PC App servo calibration, and tick-to-angle conversion.
- 📄 **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](docs/06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 6 grip verification, empirical Z-sag compensation, and lens undistortion pipeline.
- 📄 **Stepping Stone 7**: [`docs/07_fusion360_cad_modeling_guide.md`](docs/07_fusion360_cad_modeling_guide.md) — Physical workcell fabrication guide, wood/MDF layered build, 3D renders, cut list, camera mast & non-glare painting.
- 📄 **Stepping Stone 8**: [`docs/08_servo_limits_and_calibration_guide.md`](docs/08_servo_limits_and_calibration_guide.md) — Comprehensive guide for servo min/max angles, physical-to-tick math ($0.24^\circ/\text{tick}$), safe boundary discovery, Web Config Portal integration, and Cartesian Z-depth trim adjustment.

### System Integration & Planning

- 📖 [**`docs/MANUAL.md`**](docs/MANUAL.md): **Comprehensive Operator Manual & Debugging Guide** (Python venv testing, Web Config Server, ROS 2 Docker workflow, and fault troubleshooting).
- 📄 [`docs/implementation_plan.md`](docs/implementation_plan.md): High-level System Architecture Blueprint, coordinate transformations, custom ROS 2 message definitions (`DetectedObject.msg`, `ArmTask.msg`, `ArmStatus.msg`), and pipelined execution rules.
- 📄 [`docs/roadmap.md`](docs/roadmap.md): Milestone-driven Phase-Gate Verification Checklist (Gate 0 to Gate 4) replacing rigid weekly schedules.
- 📄 [`docs/risks.md`](docs/risks.md): Failure Mode and Effects Analysis (FMEA) covering risks R-01 through R-11 (including $240^\circ$ base servo ROM deadzone mitigation), optical coplanarity rules, and pre-flight checklist.

### Technical Drawings & Schematics

- 🖼️ [`docs/assets/images/pcb module.webp`](docs/assets/images/pcb%20module.webp): Physical xArm controller board secondary UART pinout and electrical interfacing diagram.
- 🖼️ [`docs/assets/images/xArm demensions.png`](docs/assets/images/xArm%20demensions.png): Factory link measurements and motor layout drawing for the xArm 1s.

---

## 📂 Project Directory Structure

```text
Sem-5-mini-project/
│
├── config/                         # Unified Configuration YAMLs
│   ├── camera_calibration.yaml     # Overhead camera intrinsics & ArUco corner coordinates
│   ├── colors.yaml                 # Calibrated HSV color bands & morphology thresholds
│   └── xarm_hardware.yaml          # Unified link lengths, mounting origins, joint limits, centers & 4 color compartments
│
├── docker/                         # Docker & Hardware Pass-Through Scripts
│   ├── Dockerfile                  # ROS 2 Humble base image with OpenCV, serial & GUI support
│   ├── build.sh                    # Host UID/GID matching build script
│   ├── run.sh                      # Hardware passthrough container runner
│   └── entrypoint.sh               # Environment overlay auto-source entrypoint
│
├── docs/                           # Stepping-Stone & System Documentation
│   ├── README.md                   # Documentation Index & Architecture Map
│   ├── MANUAL.md                   # 📖 End-to-End Operator Manual & Debugging Guide
│   ├── 00_docker_and_ros2_development_guide.md # Stepping Stone 0: Docker & ROS 2 Workflow
│   ├── 01_arena_and_layout.md      # Stepping Stone 1: Arena & Component Placement
│   ├── 02_kinematics_and_grasp_planning.md # Stepping Stone 2: Kinematics & Grasp Planning
│   ├── 03_computer_vision_and_calibration.md # Stepping Stone 3: Vision & Calibration Pipeline
│   ├── 04_ros2_architecture_and_scheduling.md # Stepping Stone 4: ROS 2 Architecture & Sorter
│   ├── 05_hardware_bringup_and_firmware_control.md # Stepping Stone 5: Low-Level Protocols & Bringup
│   ├── 06_system_testing_diagnostics_and_verification.md # Stepping Stone 6: Testing & Diagnostics
│   ├── 07_fusion360_cad_modeling_guide.md # Stepping Stone 7: Flat Board & 3D Printed Tray Rapid Fabrication
│   ├── assets/                     # CAD Renders, Blueprints, Photos & Layout Diagrams
│   ├── implementation_plan.md      # System architecture & integration blueprint
│   ├── risks.md                    # Risk matrix & failure mode analysis (R-01 to R-11)
│   └── roadmap.md                  # Milestone-based Phase-Gate Checklist (Gate 0 to Gate 4)
│
├── ros2_ws/                        # ROS 2 Humble Workspace (RUN EXCLUSIVELY IN DOCKER)
│   └── src/
│       └── sorting_arm/            # Hardware driver node, analytical polar IK, trajectory & autonomous sorter
│           ├── launch/
│           │   ├── xarm.launch.py  # Full system launch (driver, vision, autonomous sorter)
│           │   ├── xarm_driver.launch.py
│           │   └── vision_sorting.launch.py # Vision perception & autonomous sorter launch
│           ├── sorting_arm/
│           │   ├── direct_test.py      # Standalone USB HID/serial driver test
│           │   ├── driver_node.py      # ROS 2 hardware driver node with E-Stop
│           │   ├── kinematics.py       # Closed-form analytical polar IK solver
│           │   ├── polar_teleop_node.py# Keyboard polar coordinate teleop
│           │   ├── trajectory.py       # Quintic polynomial minimum-jerk trajectory generator
│           │   ├── vision_node.py      # ROS 2 camera capture & ArUco perception node
│           │   ├── autonomous_sorter_node.py # ROS 2 autonomous sorting coordinator
│           │   └── xarm_protocol.py    # Multi-interface serial/HID communication protocol
│           ├── test/
│           │   ├── run_tests.py        # Standalone offline simulation test suite (32 unit tests)
│           │   ├── test_calibration_and_features.py # Hardware & driver unit tests
│           │   ├── test_camera_alignment.py # Interactive camera alignment unit tests
│           │   ├── test_kinematics.py  # 11 analytical polar IK verification tests
│           │   └── test_vision_pipeline.py # 6 ArUco homography & color detection tests
│           ├── package.xml
│           ├── setup.cfg
│           └── setup.py
│
├── tools/                          # Utility & Calibration Tools (RUN IN VENV OR DOCKER)
│   ├── autonomous_pick_and_place.py # End-to-end vision-guided autonomous sorting bridge
│   ├── pick_and_place_test.py      # Trajectory test with closed-loop verification
│   ├── telemetry_logger.py         # Sub-millimeter drift analysis & repeatability logger
│   └── web_config/                 # Real-time Web Configuration & Calibration Tool
│       ├── server.py               # Dynamic port auto-detect HTTP & REST API server (:8000)
│       └── index.html              # Interactive browser dashboard
│
├── vision/                         # Pure OpenCV Perception & Calibration Package
│   ├── __init__.py                 # Package initializer
│   ├── align_camera.py             # Interactive camera centering & optical tilt alignment tool
│   ├── arena_vision_pipeline.py    # Core 4-point ArUco homography & HSV segmentation pipeline
│   ├── calibrate_camera.py         # Interactive checkerboard lens intrinsic calibration
│   ├── test_aruco_warp.py          # ArUco marker detection & metric grid verification
│   └── tune_hsv.py                 # Real-time 6-trackbar HSV threshold calibration GUI
│
├── .devcontainer/                  # VS Code Dev Container configuration
│   └── devcontainer.json           # 1-click isolated ROS 2 environment
├── requirements.txt                # Python host virtual environment dependencies
└── setup_venv.sh                   # One-click host Python virtual environment setup
```

---

## 🚀 Getting Started

You can run this project using the **Project-Isolated Docker Environment** (recommended for Ubuntu 24.04/26.04 hosts and cross-platform Jetson deployment) or **Native Ubuntu 22.04**.

---

### Option A: Project-Isolated Docker (Recommended)

Keeps your host operating system clean, avoids global ROS package conflicts, and runs seamlessly on both **Intel GPU laptops** and **NVIDIA Jetson (ARM64)** boards.

> 📖 **Full Guide**: See [**`DOCKER_GUIDE.md`**](DOCKER_GUIDE.md) for the complete manual, multi-terminal workflow, hardware configuration, and troubleshooting.

#### 1. Prerequisites
- Docker Engine installed (`docker run hello-world` runs without `sudo`)
- USB Webcam (Logitech C270 or C920)
- 1× xArm 1S Robotic Arm connected via USB-UART

#### 2. Build the Project Container
From the repository root:
```bash
./docker/build.sh
```

#### 3. Launch the Container Shell
```bash
./docker/run.sh
```
*(This automatically configures Intel GPU acceleration `/dev/dri`, X11 GUI forwarding for RViz2/OpenCV, and passes through USB serial and video devices).*

#### 4. Build Workspace & Run Nodes (Inside Container)
```bash
# 1. Build ROS 2 workspace
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Run automated analytical kinematics tests
pytest src/sorting_arm/test/test_kinematics.py

# 3. Launch xArm 1S hardware driver node (auto-detects USB HID / serial)
ros2 launch sorting_arm xarm_driver.launch.py

# 4. Or run interactive polar teleoperation node
ros2 run sorting_arm polar_teleop_node

# 5. Full Autonomous Vision-Guided Sorting Pipeline:
ros2 launch sorting_arm xarm.launch.py auto_start:=true
```

---

### Option B: Host Python Virtual Environment (Standalone Python Tools Only)

For running standalone Python scripts (Web Configuration Dashboard, servo calibration, telemetry analysis, direct tests) without spinning up Docker:

> [!WARNING]
> **ROS 2 Execution Policy**: All ROS 2 nodes, launches, and coordination services must run **strictly inside the Docker container** (`./docker/run.sh`). Python commands on the host must be run exclusively inside the dedicated Python virtual environment (`.venv`).

#### 1. Setup Host Virtual Environment
```bash
# Run one-click setup script (creates .venv and installs dependencies)
./setup_venv.sh

# Or activate manually
source .venv/bin/activate
```

#### 2. Run Web Configuration & Calibration Tool
```bash
# Launch dynamic auto-detecting web server on port 8000
python3 tools/web_config/server.py
# Open http://localhost:8000 in your browser
```

#### 3. Run Standalone Tests & Motion Test Suite
```bash
# Run 26 offline simulation unit tests (no hardware required)
python3 ros2_ws/src/sorting_arm/test/run_tests.py

# Run automated computer vision & ArUco homography tests
pytest ros2_ws/src/sorting_arm/test/test_vision_pipeline.py -v

# Run comprehensive 13-motion single-arm trajectory test suite
python3 ros2_ws/src/sorting_arm/sorting_arm/motion_test_suite.py --arm xarm_1 --all

# Run sub-millimeter telemetry drift logger
python3 tools/telemetry_logger.py --arm xarm_1 --scan
```

#### 4. Run Computer Vision Tools & Autonomous Sorting Bridge
```bash
# Test 4-point ArUco planar homography & metric grid (press 'l' lock, 'g' grid, 's' save)
python3 vision/test_aruco_warp.py --synthetic  # or --device /dev/video0

# Tune real-time HSV color thresholds interactively (keys 1:Red, 2:Green, 3:Blue, 4:Yellow, 's' save)
python3 vision/tune_hsv.py --synthetic         # or --device /dev/video0

# Calibrate overhead camera intrinsics with 9x6 checkerboard
python3 vision/calibrate_camera.py --synthetic # or --device /dev/video0

# Run vision-guided autonomous pick-and-place bridge (Phase 1 + Phase 2 merge)
python3 tools/autonomous_pick_and_place.py --arm xarm_1 --synthetic --dry-run
```

For the comprehensive guide, see 📖 [**`docs/MANUAL.md`**](docs/MANUAL.md) and 📄 [**`docs/08_servo_limits_and_calibration_guide.md`**](docs/08_servo_limits_and_calibration_guide.md).

---

## 👨‍💻 Project Team

- **Shubham Jain**
- **Karan Pal**

---

## 📅 Project Status

- [x] **Stepping Stone 0**: Docker & ROS 2 Development Environment (`docs/00_docker_and_ros2_development_guide.md`)
- [x] **Stepping Stone 1**: Arena & Physical Component Placement (`docs/01_arena_and_layout.md`)
- [x] **Stepping Stone 2**: Kinematics & Grasp Planning (`docs/02_kinematics_and_grasp_planning.md`)
- [x] **Stepping Stone 3**: Computer Vision & Calibration (`docs/03_computer_vision_and_calibration.md`)
- [x] **Stepping Stone 4**: ROS 2 Architecture & Single-Arm Pipeline (`docs/04_ros2_architecture_and_scheduling.md`)
- [x] **Stepping Stone 5**: Hardware Bringup & Firmware Control (`docs/05_hardware_bringup_and_firmware_control.md`)
- [x] **Stepping Stone 6**: Testing, Diagnostics & Reach Verification (`docs/06_system_testing_diagnostics_and_verification.md`)
- [x] **Stepping Stone 7**: Flat Board & 3D Printed Tray Rapid Fabrication (`docs/07_fusion360_cad_modeling_guide.md`)
- [x] **Stepping Stone 8**: Servo Limits, Calibration & Z-Trim Web Guide (`docs/08_servo_limits_and_calibration_guide.md`)
- [x] **Phase 0**: Direct USB Hardware Bringup, xArm App Calibration & Reach Testing
- [x] **Phase 1**: Kinematics, Driver, Calibration & 13-Motion Trajectory Test Suite (`ros2_ws/src/sorting_arm/`) — **✅ COMPLETE**
- [x] **Phase 2**: Camera Calibration, ArUco Metric Homography & HSV Segmentation (`vision/` & `vision_node`) — **✅ COMPLETE**
- [x] **Phase 3**: Single-Arm Autonomous Sorting & Live Verification (Gate 3) — **✅ READY**


