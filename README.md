# 🤖 Single xArm 1S Red-Object Picking Robot

An autonomous robotic picking system powered by **ROS 2 Humble**, **OpenCV**, and a single **LewanSoul / Hiwonder xArm 1S** 6-DoF articulated bus-servo manipulator. The system detects red wooden craft blocks on a physical $7 \times 12$ grid arena ($350\text{ mm} \times 600\text{ mm}$), discriminates against green distractor blocks, filters out the blue robot chassis, computes analytical closed-form inverse kinematics, and deposits picked red objects into a dedicated Rear Flank container box.

---

## 📌 Project Overview

This project automates target object picking using classical computer vision, analytical kinematics, and minimum-jerk trajectory execution under real-world physical and mechanical constraints:

1. **Central Manipulator Mounting**:
   - Single xArm 1S manipulator centered at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm}, Z = 0.0\text{ mm})$ [Rows 3..5, Cols 2..4] with nominal heading **$-35.0^\circ$** (South-East).
   - This orientation ensures Motor 6 (travel range $126\text{--}879$ ticks) easily spans both the arena pick envelope ($139\text{--}785$ ticks) and the container drop zone ($875$ ticks) without crossing the servo deadzone.
2. **Physical Hand-Drawn $7 \times 12$ Grid Arena ($350.0\text{ mm} \times 600.0\text{ mm}$)**:
   - 7 columns (Cols 0..6) $\times$ 12 rows (Rows 0..11) of $50.0\text{ mm} \times 50.0\text{ mm}$ square cells drawn directly on the platform.
   - Sits directly flat on the baseboard at $Z = 0.0\text{ mm}$.
   - Self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) forms a compliant retaining perimeter.
3. **Color Discrimination Methodology**:
   - **Target (Red)**: Dual-band HSV segmentation ($[0..10]$ and $[170..180]$) detects red craft cubes. Only red objects are queued, validated, and picked.
   - **Distractor (Green)**: Green cubes ($[38..82]$) are tracked for telemetry and visual overlay, but marked `targetable: false` and strictly excluded from the pick queue.
   - **Chassis Exclusion (Blue)**: The xArm 1S chassis is blue anodized aluminum. Blue detection is completely bypassed to prevent false-positive self-detections and specular reflection artifacts.
   - **Spatial Exclusion Mask**: Rectangular exclusion mask for Rows 3..5, Cols 2..4 ($X \in [100, 250\text{ mm}]$, $Y \in [300, 450\text{ mm}]$) zeros out the robot base area in computer vision.
4. **Dedicated Rear Flank Container Box**:
   - Physical container box placed in Rows 0..1, Columns 5..6 ($100\times 100\text{ mm}$).
   - Drop Target: $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 40.0\text{ mm})$, base yaw tick $875$.
5. **Table-Coplanar ArUco Markers ($50\text{ mm} \times 50\text{ mm}$, $Z = 0.0\text{ mm}$)**:
   - 4 matte vinyl ArUco markers (`DICT_4X4_50`, IDs 0..3) positioned in clockwise order:
     - Marker 0: $(25.0, 425.0\text{ mm})$ [Row 3, Col 0] $\to$ Pixel $(0, 0)$
     - Marker 1: $(325.0, 425.0\text{ mm})$ [Row 3, Col 6] $\to$ Pixel $(600, 0)$
     - Marker 2: $(325.0, 25.0\text{ mm})$ [Row 11, Col 6] $\to$ Pixel $(600, 800)$
     - Marker 3: $(25.0, 25.0\text{ mm})$ [Row 11, Col 0] $\to$ Pixel $(0, 800)$
   - Planar homography rectifies the active arena into a metric $600 \times 800\text{ px}$ image at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$).

---

## 📐 Workspace Field Layout

```text
  Y (Depth: 600 mm, Rows 11 to 0)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Row 0, Y = 600 mm)                      │
   │  │                                  ┌───────────────────────┐ │
   │  │                                  │ CONTAINER BOX (100x100│ │ (X=300, Y=550)
   │  │                                  │ Rows 0..1, Cols 5..6) │ │ Drop Yaw: 875 ticks
   │  │                                  └───────────────────────┘ │
   │  │   [ArUco #0] (50x50mm)      ┌───────────────────┐ [ArUco #1│
   │  │   (25, 425) [Row 3, Col 0]  │   xArm 1S MOUNT   │ (325, 425│
   │  │                             │ (X=175.0, Y=375.0)│ [Row 3, C│
   │  │                             │ Heading: -35.0 deg│          │
   │  │                             └───────────────────┘          │
   │  │                                                            │
   │  │               ACTIVE PICKING ARENA (Rows 7..10)            │
   │  │                     (r = 160..300 mm reach)                │
   │  │               [Red = Target | Green = Distractor]          │
   │  │                                                            │
   │  │   [ArUco #3] (50x50mm)                            [ArUco #2│
   │  │   (25, 25) [Row 11, Col 0]                        (325, 25)│
   │  │                                                   [Row 11, │
   │  │ FRONT OF WORKSPACE (Operator Position @ Row 11, Y = 0 mm)  │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 350 mm, Cols 0 to 6)

  Platform Grid:   7 columns x 12 rows of 50x50 mm squares (350 x 600 mm)
  Robot Mounting:  Rows 3..5, Cols 2..4 (Center: 175.0, 375.0 mm), Heading -35.0°
  Vision Rect:     600 x 800 px metric grid at 2.0 px/mm between ArUco centers
  Drop Motion:     Motor 6 swivel to 875 ticks into Rear Flank container box
```

---

## 🛠 Hardware Specifications

| Component           | Specification                                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------------------------------- |
| **Manipulator**     | 1× LewanSoul / Hiwonder xArm 1S (Mount at $X=175\text{ mm}, Y=375\text{ mm}$, Heading $-35.0^\circ$)             |
| **Link Lengths**    | Base-to-Shoulder ($d_1$): $65\text{ mm}$ · Upper Arm ($L_1$): $101\text{ mm}$ · Forearm ($L_2$): $95\text{ mm}$ · Tool ($L_3$): $150\text{--}168\text{ mm}$ |
| **Gripper**         | 2-finger parallel claw ($55\text{ mm}$ opening, Motor 2 axial roll, Motor 1 clamp)                              |
| **Compute**         | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (Ubuntu 22.04 LTS, rear electronics bay)                  |
| **Camera**          | Overhead camera with Invicto 10" LED ring light on welded scrap aluminum stand @ 65 cm nadir                    |
| **Base Platform**   | Flat platform board with drawn $7\times 12$ grid ($350\text{ mm} \times 600\text{ mm}$, $5\text{ cm}$ cells)    |
| **Picking Arena**   | Rows 7..10 active pick zone ($r \in [160, 300\text{ mm}]$) with compliant foam perimeter                       |
| **Container Box**   | Single physical container box ($100\times 100\times 35\text{ mm}$) at Rear Flank $(X=300, Y=550\text{ mm})$     |
| **Fiducials**       | 4× matte vinyl ArUco markers (`DICT_4X4_50`, IDs 0..3, $50\times 50\text{ mm}$) coplanar at $Z = 0.0\text{ mm}$|
| **Target Objects**  | Red wooden craft blocks ($25.4\text{ mm}$, $14\text{--}16\text{ g}$)                                            |
| **Distractor**      | Green wooden craft blocks (ignored/bypassed by pick scheduler)                                                  |
| **Chassis Color**   | Blue (disabled in vision; excluded via spatial mask to prevent reflection false-positives)                      |
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
- **Computer Vision**: OpenCV (ArUco planar homography, HSV color masking, contour moments, chassis spatial mask)
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
                    [ vision_node ] (ArUco Homography + Red/Green HSV Detection)
                           │
                           ▼ /detected_objects
              [ autonomous_sorter_node ] (Red-Only Queue + Nearest-First State Machine)
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
| `vision_node`            | Detects 4 ArUco markers, rectifies $600\times 800\text{ px}$ metric grid, masks chassis, publishes `/detected_objects`          |
| `autonomous_sorter_node` | Queues target red cubes in Nearest-First order, solves 5-phase trajectories to container box, handles state machine             |
| `driver_node`            | Dispatches servo move packets over USB serial/HID, reads position feedback, monitors communication health & handles E-Stop     |
| `web_config`             | Serves real-time web portal on port 8000 for motor calibration, live potentiometer readouts, and CSV drift telemetry logging     |

---

## 📚 Technical Documentation Index

The project documentation is organized into modular **Stepping-Stone Engineering Specifications** and system integration guides in the [`docs/`](docs/) directory:

### Stepping-Stone Specifications

- 📄 **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](docs/00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container development workflow, Intel GPU acceleration, USB hardware mapping, and ROS 2 Humble package build standards.
- 📄 **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](docs/01_arena_and_layout.md) — Physical $7\times 12$ grid arena ($350\times 600\text{ mm}$), central xArm 1S mount at $(175, 375)$ with heading $-35.0^\circ$, Rear Flank container box, and $50\text{ mm}$ ArUco fiducials.
- 📄 **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](docs/02_kinematics_and_grasp_planning.md) — Complete mathematical derivations, DH parameters, analytical IK formulas ($\theta_1\text{--}\theta_5$), top-down reach limit proof ($158.7\text{ mm}$), dual-regime wrist roll control, 5-phase trajectory waypoint profiling, torque/friction physics, and reference Python IK solver.
- 📄 **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](docs/03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography ($2.0\text{ px/mm}$), **Color Discrimination Methodology** (Red target, Green distractor, Blue chassis exclusion), and spatial masking.
- 📄 **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](docs/04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, single-arm red-object picking pipeline, Nearest-First radial queueing, joint trajectory generation, and production Python nodes.
- 📄 **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](docs/05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol (binary/ASCII), direct USB-HID/UART bringup, xArm PC App servo calibration, and tick-to-angle conversion.
- 📄 **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](docs/06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 1 grip verification, empirical Z-sag compensation, and lens undistortion pipeline.
- 📄 **Stepping Stone 8**: [`docs/08_servo_limits_and_calibration_guide.md`](docs/08_servo_limits_and_calibration_guide.md) — Comprehensive guide for servo min/max angles, physical-to-tick math ($0.24^\circ/\text{tick}$), safe boundary discovery, Web Config Portal integration, and Cartesian Z-depth trim adjustment.

### System Integration & Planning

- 📖 [**`docs/MANUAL.md`**](docs/MANUAL.md): **Comprehensive Operator Manual & Debugging Guide** (Python venv testing, Web Config Server, ROS 2 Docker workflow, and fault troubleshooting).
- 📄 [`docs/implementation_plan.md`](docs/implementation_plan.md): High-level System Architecture Blueprint, coordinate transformations, custom ROS 2 message definitions (`DetectedObject.msg`, `ArmTask.msg`, `ArmStatus.msg`), and pipelined execution rules.
- 📄 [`docs/roadmap.md`](docs/roadmap.md): Milestone-driven Phase-Gate Verification Checklist (Gate 0 to Gate 4).
- 📄 [`docs/risks.md`](docs/risks.md): Failure Mode and Effects Analysis (FMEA) covering risks R-01 through R-11 (including $240^\circ$ base servo ROM deadzone mitigation), optical coplanarity rules, and pre-flight checklist.

---

## 📂 Project Directory Structure

```text
Sem-5-mini-project/
│
├── config/                         # Unified Configuration YAMLs
│   ├── camera_calibration.yaml     # Overhead camera intrinsics & ArUco corner coordinates
│   ├── colors.yaml                 # Calibrated HSV color bands & morphology thresholds
│   └── xarm_hardware.yaml          # Unified link lengths, mounting origins, joint limits & container drop target
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
│   ├── 08_servo_limits_and_calibration_guide.md # Stepping Stone 8: Servo Limits & Calibration
│   ├── arena_bill_of_materials.md  # Master BOM, Hardware Checklist & Fasteners
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
│           │   ├── autonomous_sorter_node.py # ROS 2 autonomous sorting coordinator (red-only)
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
- [x] **Stepping Stone 8**: Servo Limits, Calibration & Z-Trim Web Guide (`docs/08_servo_limits_and_calibration_guide.md`)
- [x] **Phase 0**: Direct USB Hardware Bringup, xArm App Calibration & Reach Testing
- [x] **Phase 1**: Kinematics, Driver, Calibration & 13-Motion Trajectory Test Suite (`ros2_ws/src/sorting_arm/`) — **✅ COMPLETE**
- [x] **Phase 2**: Camera Calibration, ArUco Metric Homography & HSV Segmentation (`vision/` & `vision_node`) — **✅ COMPLETE**
- [x] **Phase 3**: Single-Arm Autonomous Sorting & Live Verification (Gate 3) — **✅ READY**


