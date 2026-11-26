# 🤖 Single xArm 1S Dual-Color (Red & Green) Sorting Robot

An autonomous robotic sorting system powered by **ROS 2 Humble**, **OpenCV**, and a single **LewanSoul / Hiwonder xArm 1S** 6-DoF articulated bus-servo manipulator. The system detects red and green wooden craft blocks on a physical $7 \times 12$ grid arena ($350\text{ mm} \times 600\text{ mm}$), bypasses the blue robot chassis, computes analytical closed-form inverse kinematics, and deposits cubes into symmetric Rear Flank container boxes (Red box @ Rear-Left, Green box @ Rear-Right) using an overhead rear-flip transfer trajectory.

---

> [!CAUTION]
> ## ⚠️ THE ABSOLUTE RULE OF EXECUTION: STRICT TWO-ENVIRONMENT SEPARATION
> This rule is fundamental and must NEVER be violated under any circumstances:
> 
> 1. 🐳 **ANY command related to ROS 2 MUST be executed inside the Docker container** (`./docker/run.sh`).
>    - Examples: `colcon build`, `ros2 launch ...`, `ros2 run ...`, `ros2 topic ...`, `ros2 service ...`, `rviz2`, ROS test suites (`pytest`).
>    - **Why**: ROS 2 Humble packages and system dependencies are containerized to eliminate host operating system library conflicts (especially on Ubuntu 24.04/26.04) and ensure 100% deterministic reproducibility.
> 
> 2. 🐍 **ANY standalone Python command MUST be executed inside the Python virtual environment** (`source venv/bin/activate`).
>    - Examples: `python3 tools/autonomous_dashboard.py`, `python3 tools/run_autonomous_system.py`, `python3 tools/web_config/server.py`, `python3 vision/*.py`.
>    - **Why**: Protects system Python from package corruption and guarantees matching library versions (`opencv-python`, `pyserial`, `pyyaml`).
> 
> 🚫 **NEVER** run ROS 2 commands directly on the host operating system.  
> 🚫 **NEVER** run project Python scripts without activating the Python virtual environment (`venv`).

---

## 📌 Project Overview

This project automates target object sorting using classical computer vision, analytical kinematics, and minimum-jerk trajectory execution under real-world physical and mechanical constraints:

1. **Central Manipulator Mounting**:
   - Single xArm 1S manipulator centered at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm}, Z = 0.0\text{ mm})$ [Rows 3..5, Cols 2..4] with nominal heading **$-90.0^\circ$** (pointing straight South down Column 3 midline).
   - Motor 6 (travel range $100\text{--}900$ ticks, center $502$ ticks) spans the front picking arena ($\pm 26^\circ$) and the rear drop zones ($\pm 35.5^\circ$) with zero risk of hitting mechanical stops.
2. **Overhead Rear-Flip Transfer Trajectory**:
   - Rather than sweeping the arm across the table, the robot lifts the grasped cube vertically to safe transit altitude ($Z \ge 130\text{ mm}$), pitches Motors 5, 4, 3 backward overhead into a rear-facing stance (+Y), aligns Motor 6 over the target container (Red: $650$ ticks, Green: $354$ ticks), and drops from standoff clearance ($Z = 45\text{ mm}$).
3. **Physical Hand-Drawn $7 \times 12$ Grid Arena ($350.0\text{ mm} \times 600.0\text{ mm}$)**:
   - 7 columns (Cols 0..6) $\times$ 12 rows (Rows 0..11) of $50.0\text{ mm} \times 50.0\text{ mm}$ square cells drawn directly on the platform.
   - Sits directly flat on the baseboard at $Z = 0.0\text{ mm}$.
   - Self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) forms a compliant retaining perimeter.
4. **Color Discrimination Methodology**:
   - **Target (Red)**: Dual-band HSV segmentation ($[0..10]$ and $[170..180]$) detects red craft cubes, routed to the Red container box.
   - **Target (Green)**: HSV segmentation ($[38..85]$) detects green craft cubes, routed to the Green container box.
   - **Chassis Exclusion (Blue)**: The xArm 1S chassis is blue anodized aluminum. Blue detection is completely bypassed to prevent false-positive self-detections and specular reflection artifacts.
   - **Spatial Exclusion Mask**: Rectangular exclusion mask for Rows 3..5, Cols 2..4 ($X \in [100, 250\text{ mm}]$, $Y \in [300, 450\text{ mm}]$) zeros out the robot base area in computer vision.
5. **Dual Rear Flank Container Boxes**:
   - **Red Container Box**: Rows 0..1, Cols 0..1 ($100\times 100\text{ mm}$); Drop Target $(X = 50.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, base yaw tick **$650$**.
   - **Green Container Box**: Rows 0..1, Cols 5..6 ($100\times 100\text{ mm}$); Drop Target $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, base yaw tick **$354$**.
6. **Table-Coplanar ArUco Markers ($50\text{ mm} \times 50\text{ mm}$, $Z = 0.0\text{ mm}$)**:
   - 4 matte vinyl ArUco markers (`DICT_4X4_50`, IDs 0..3) positioned at:
     - Marker 0 (ID 2): $(25.0, 275.0\text{ mm})$ [Row 6, Col 0] $\to$ Pixel $(0, 0)$ (Top-Left, pointing West)
     - Marker 1 (ID 0): $(325.0, 275.0\text{ mm})$ [Row 6, Col 6] $\to$ Pixel $(600, 0)$ (Top-Right, pointing West)
     - Marker 2 (ID 3): $(25.0, 25.0\text{ mm})$ [Row 11, Col 0] $\to$ Pixel $(0, 500)$ (Bottom-Left, pointing East)
     - Marker 3 (ID 1): $(325.0, 25.0\text{ mm})$ [Row 11, Col 6] $\to$ Pixel $(600, 500)$ (Bottom-Right, pointing East)
   - Planar homography rectifies the active arena into a metric $600 \times 500\text{ px}$ image at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$).
7. **Manipulator Mounting & Base Elevation**:
   - Centered at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$ ($100\text{ mm}$ behind Row 6 line).
   - Elevated on a $5.0\text{ mm}$ cardboard baseplate ($Z_{\text{base}} = 5.0\text{ mm}$, shoulder pivot height from table $d_1 = 70.0\text{ mm}$).

---

## 📐 Workspace Field Layout

```text
  Y (Depth: 600 mm, Rows 11 to 0)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Row 0, Y = 600 mm)                      │
   │  │ ┌───────────────────────┐        ┌───────────────────────┐ │
   │  │ │   RED CONTAINER BOX   │        │  GREEN CONTAINER BOX  │ │
   │  │ │  (Rows 0..1, Cols 0..1)        │  (Rows 0..1, Cols 5..6) │ │
   │  │ │ Drop Yaw: 650 ticks   │        │ Drop Yaw: 354 ticks   │ │
   │  │ └───────────────────────┘        └───────────────────────┘ │
   │  │                                                            │
   │  │                         ┌───────────────────┐              │
   │  │                         │   xArm 1S MOUNT   │              │
   │  │                         │ (X=175.0, Y=375.0)│              │
   │  │                         │ Z_base = +5.0 mm  │              │
   │  │                         │ Heading: -90° (S) │              │
   │  │                         └───────────────────┘              │
   │  │   [ArUco M0 / ID 2] (50x50mm)      [ArUco M1 / ID 0] (50x50│
   │  │   (25, 275) [Row 6, Col 0]         (325, 275) [Row 6, Col 6│
   │  │   Heading: West (180°)             Heading: West (180°)    │
   │  │ ┌────────────────────────────────────────────────────────┐ │
   │  │ │         GOLDEN PICKUP ZONE (Rows 7..9, Cols 2..4)      │ │
   │  │ │       (Kinematic Sweet Spot: r = 140..260 mm reach)    │ │
   │  │ │   [Top-Down Plunge | Sag Trim | Dynamic Wrist Roll]    │ │
   │  │ └────────────────────────────────────────────────────────┘ │
   │  │   [ArUco M2 / ID 3] (50x50mm)      [ArUco M3 / ID 1] (50x50│
   │  │   (25, 25) [Row 11, Col 0]         (325, 25) [Row 11, Col 6│
   │  │   Heading: East (0°)               Heading: East (0°)      │
   │  │                                                            │
   │  │ FRONT OF WORKSPACE (Operator Position @ Row 11, Y = 0 mm)  │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────> X (Width: 350 mm)

  Platform Grid:   7 columns x 12 rows of 50x50 mm squares (350 x 600 mm)
  Robot Mounting:  Rows 3..5, Cols 2..4 (Center: 175.0, 375.0, 5.0 mm), Heading -90.0° (South)
  Vision Rect:     600 x 500 px metric grid at 2.0 px/mm between ArUco centers (Rows 6..11)
  Drop Motion:     Overhead rear-flip: Red box @ 650 ticks, Green box @ 354 ticks
```

---

## 🛠 Hardware Specifications

| Component           | Specification                                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------------------------------- |
| **Manipulator**     | 1× LewanSoul / Hiwonder xArm 1S (Mount at $X=175\text{ mm}, Y=375\text{ mm}$, Heading $-90.0^\circ$ South)       |
| **Link Lengths**    | Base-to-Shoulder ($d_1$): $65\text{ mm}$ · Upper Arm ($L_1$): $101\text{ mm}$ · Forearm ($L_2$): $95\text{ mm}$ · Tool ($L_3$): $150\text{--}168\text{ mm}$ |
| **Gripper**         | 2-finger parallel claw ($55\text{ mm}$ opening, Motor 2 axial roll, Motor 1 clamp at 520 ticks)                 |
| **Compute**         | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5 / Laptop) (Ubuntu 22.04 LTS, rear electronics bay)         |
| **Camera**          | Overhead camera with Invicto 10" LED ring light on stand @ 65 cm nadir (or hand-held steady scan)               |
| **Base Platform**   | Flat platform board with drawn $7\times 12$ grid ($350\text{ mm} \times 600\text{ mm}$, $5\text{ cm}$ cells)    |
| **Picking Arena**   | Golden Pickup Zone (Rows 7..9, Cols 2..4, $X \in [75, 275\text{ mm}]$, $Y \in [100, 250\text{ mm}]$, $r \in [140, 260\text{ mm}]$) with compliant foam perimeter |
| **Container Boxes** | Dual physical container boxes ($100\times 100\times 35\text{ mm}$ each) at Rear Flank: Red Box $(X=50, Y=550\text{ mm})$, Green Box $(X=300, Y=550\text{ mm})$ |
| **Fiducials**       | 4× matte vinyl ArUco markers (`DICT_4X4_50`, IDs 0..3, $50\times 50\text{ mm}$) coplanar at $Z = 0.0\text{ mm}$|
| **Target Objects**  | Red and Green wooden craft blocks ($25.4\text{ mm}$ cubes / $60\times 35\text{ mm}$ rectangles, $14\text{--}16\text{ g}$) sorted into dedicated rear boxes |
| **Distractor / Bypass** | Blue objects & metallic reflections ignored/bypassed (matches blue xArm chassis)                            |
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
              [ autonomous_sorter_node ] (Dual-Color Queue + Nearest-First State Machine)
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
| `autonomous_sorter_node` | Queues target Red and Green cubes in Nearest-First order, solves 7-phase overhead rear-flip trajectories to dedicated boxes     |
| `driver_node`            | Dispatches servo move packets over USB serial/HID, reads position feedback, monitors communication health & handles E-Stop     |
| `web_config`             | Serves real-time web portal on port 8000 for motor calibration, live potentiometer readouts, and CSV drift telemetry logging     |

---

## 📚 Technical Documentation Index

The project documentation is organized into modular **Stepping-Stone Engineering Specifications** and system integration guides in the [`docs/`](docs/) directory:

### Stepping-Stone Specifications

- 📄 **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](docs/00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container development workflow, Intel GPU acceleration, USB hardware mapping, and ROS 2 Humble package build standards.
- 📄 **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](docs/01_arena_and_layout.md) — Physical $7\times 12$ grid arena ($350\times 600\text{ mm}$), central xArm 1S mount at $(175, 375)$ with heading $-90.0^\circ$ (South), dual Rear Flank container boxes, and $50\text{ mm}$ ArUco fiducials.
- 📄 **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](docs/02_kinematics_and_grasp_planning.md) — Complete mathematical derivations, DH parameters, analytical IK formulas ($\theta_1\text{--}\theta_5$), top-down reach limit proof ($158.7\text{ mm}$), dual-regime wrist roll control, 7-phase overhead rear-flip trajectory waypoint profiling, torque/friction physics, and reference Python IK solver.
- 📄 **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](docs/03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography ($2.0\text{ px/mm}$), **Color Discrimination Methodology** (Red & Green targets, Blue chassis exclusion), and spatial masking.
- 📄 **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](docs/04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, single-arm dual-color sorting pipeline, Nearest-First radial queueing, joint trajectory generation, and production Python nodes.
- 📄 **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](docs/05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol (binary/ASCII), direct USB-HID/UART bringup, xArm PC App servo calibration, and tick-to-angle conversion.
- 📄 **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](docs/06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 1 grip verification, empirical Z-sag compensation, and lens undistortion pipeline.
- 📄 **Stepping Stone 8**: [`docs/08_servo_limits_and_calibration_guide.md`](docs/08_servo_limits_and_calibration_guide.md) — Comprehensive guide for servo min/max angles, physical-to-tick math ($0.24^\circ/\text{tick}$), safe boundary discovery, Web Config Portal integration, and Cartesian Z-depth trim adjustment.
- 📄 **Stepping Stone 9**: [`docs/09_web_dashboard_guide.md`](docs/09_web_dashboard_guide.md) — **Web Mission Control Dashboard & Digital Twin Guide**: Dynamic camera switcher, hardware watchdog, hand-held camera leveling HUD, Snap & Lock stationary frame buffer, and simultaneous RViz digital twin sync.

### System Integration & Planning

- 📖 [**`docs/09_web_dashboard_guide.md`**](docs/09_web_dashboard_guide.md): **Web Mission Control & Digital Twin Operator Guide** (zero-dependency web interface, camera switcher, hotkeys, and troubleshooting).
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
│           │   ├── simulation.launch.py # Full 3D digital twin simulation (mock driver, URDF, RViz2, sorter)
│           │   ├── xarm.launch.py  # Full system launch (driver, vision, autonomous sorter)
│           │   ├── xarm_driver.launch.py
│           │   └── vision_sorting.launch.py # Vision perception & autonomous sorter launch
│           ├── urdf/
│           │   └── xarm_1s.urdf    # Parametric DH workcell URDF (arena grid, containers, xArm links)
│           ├── rviz/
│           │   └── xarm_simulation.rviz # Pre-configured RViz2 3D digital twin scene
│           ├── sorting_arm/
│           │   ├── direct_test.py      # Standalone USB HID/serial driver test
│           │   ├── driver_node.py      # ROS 2 hardware driver node with mock simulation & E-Stop
│           │   ├── kinematics.py       # Closed-form analytical polar IK solver
│           │   ├── mock_vision_node.py # Synthetic vision perception & scenario generator
│           │   ├── polar_teleop_node.py# Keyboard polar coordinate teleop
│           │   ├── trajectory.py       # Quintic polynomial minimum-jerk trajectory generator
│           │   ├── vision_node.py      # ROS 2 camera capture & ArUco perception node
│           │   ├── autonomous_sorter_node.py # ROS 2 autonomous sorting coordinator (dual-color Red & Green)
│           │   └── xarm_protocol.py    # Multi-interface serial/HID communication protocol
│           ├── test/
│           │   ├── run_tests.py        # Standalone offline simulation test suite (38 unit tests)
│           │   ├── test_calibration_and_features.py # Hardware & driver unit tests
│           │   ├── test_camera_alignment.py # Interactive camera alignment unit tests
│           │   ├── test_kinematics.py  # 11 analytical polar IK verification tests
│           │   ├── test_simulation_pipeline.py # 6 automated mock driver & trajectory simulation tests
│           │   └── test_vision_pipeline.py # 6 ArUco homography & color detection tests
│           ├── package.xml
│           ├── setup.cfg
│           └── setup.py
│
├── tools/                          # Utility & Calibration Tools (RUN IN VENV OR DOCKER)
│   ├── autonomous_dashboard.py     # Web-based autonomous mission control & digital twin dashboard (:8080)
│   ├── dashboard_ui/               # Single-page HTML/CSS/JS interface with camera assist HUD
│   ├── run_autonomous_system.py    # 4-stage end-to-end autonomous pipeline (self-healing, ArUco validation, picking)
│   ├── autonomous_pick_and_place.py # End-to-end vision-guided autonomous sorting bridge
│   ├── render_sim_view.py          # Multi-view 3D/elevation digital twin trajectory renderer
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

#### 4. Build Workspace & Run ROS 2 Nodes (Inside Docker Container)
```bash
# ==============================================================================
# [DOCKER CONTAINER] Execute inside Docker container shell (./docker/run.sh)
# Working Directory: /workspace/ros2_ws
# ==============================================================================

# 1. Build ROS 2 workspace
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Run automated analytical kinematics tests (14 tests)
# Note: Both 'pytest' and 'python3 -m pytest' are supported inside Docker
python3 -m pytest src/sorting_arm/test/test_kinematics.py

# 3. Run full ROS 2 mock driver & trajectory simulation pipeline tests (9 tests)
python3 -m pytest src/sorting_arm/test/test_simulation_pipeline.py

# Or run all 50 ROS 2 functional & vision pipeline tests:
python3 -m pytest src/sorting_arm/test/test_kinematics.py src/sorting_arm/test/test_vision_pipeline.py src/sorting_arm/test/test_calibration_and_features.py src/sorting_arm/test/test_camera_alignment.py src/sorting_arm/test/test_simulation_pipeline.py src/sorting_arm/test/test_arena_visualizer.py

# 4. 3D Digital Twin Simulation & RViz2 (NO physical hardware required):
ros2 launch sorting_arm simulation.launch.py gui:=true mock_vision:=true auto_start:=true

# Or run headless simulation in background/CI:
ros2 launch sorting_arm simulation.launch.py gui:=false mock_vision:=true auto_start:=true

# 5. Launch xArm 1S hardware driver node (auto-detects USB HID / serial)
ros2 launch sorting_arm xarm_driver.launch.py

# 6. Or run interactive polar teleoperation node
ros2 run sorting_arm polar_teleop_node

# 7. Full Physical Vision-Guided Sorting Pipeline:
ros2 launch sorting_arm xarm.launch.py auto_start:=true
```

> [!TIP]
> **One-Shot Docker Execution from Host Terminal**:
> You can also run any Docker test directly from the host repository root without opening an interactive container shell:
> ```bash
> # [HOST TERMINAL -> DOCKER] Working Directory: ~/Projects/Sem-5-mini-project
> ./docker/run.sh pytest ros2_ws/src/sorting_arm/test/test_kinematics.py
> ./docker/run.sh pytest ros2_ws/src/sorting_arm/test/test_simulation_pipeline.py
> ```

---

### Option B: Host Python Virtual Environment (Standalone Python Tools Only)

For running standalone Python scripts (Web Configuration Dashboard, Web Mission Control Dashboard, camera calibration, telemetry analysis, direct tests) without spinning up Docker:

> [!CAUTION]
> **Strict Environment Isolation Rule**:
> - **ROS 2 commands (`ros2 ...`, `colcon ...`) MUST NEVER be run directly on the host.**
> - **Python commands on the host MUST ALWAYS be run inside the dedicated Python virtual environment (`source venv/bin/activate`).**

#### 1. Setup & Activate Host Virtual Environment
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Execute on Host Machine
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================

# Run one-click setup script (creates venv/ and installs dependencies)
./setup_venv.sh

# Whenever opening a new terminal session, activate the venv:
source venv/bin/activate
```

#### 2. Run Web Mission Control Dashboard & Digital Twin (Port 8080)
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Execute on Host (source venv/bin/activate)
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================

# Launch Web Mission Control Dashboard (connects to camera, probes xArm, renders twin):
python3 tools/autonomous_dashboard.py --dry-run     # Simulation / Mock Mode
python3 tools/autonomous_dashboard.py               # Live Hardware Mode
# Open http://localhost:8080 in your browser
```

#### 3. Run Terminal Autonomous Pipeline, Tests & Hardware Calibration
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Execute on Host (source venv/bin/activate)
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================

# 1. Run offline unit tests via pytest (36 passed, ROS 2 integration tests gracefully skipped):
pytest ros2_ws/src/sorting_arm/test/

# Or run the standalone zero-dependency test runner:
python3 ros2_ws/src/sorting_arm/test/run_tests.py

# 2. Launch real-time Web Hardware Configuration & Potentiometer Server (Port 8000):
python3 tools/web_config/server.py

# 3. Run 4-stage terminal autonomous sorting pipeline:
python3 tools/run_autonomous_system.py --dry-run --camera 0   # Simulation / Dry-run
python3 tools/run_autonomous_system.py --camera 0             # Live Hardware Execution

# 4. Camera Calibration & ArUco Homography Tools:
python3 vision/test_aruco_warp.py --synthetic      # Test ArUco metric rectification
python3 vision/tune_hsv.py --synthetic             # Interactive 6-trackbar HSV tuner
python3 vision/calibrate_camera.py --synthetic     # Camera lens intrinsics
python3 vision/align_camera.py --synthetic --save-report # Camera leveling HUD check

# 5. Run sub-millimeter telemetry drift logger:
python3 tools/telemetry_logger.py --arm xarm_1 --scan
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


