# 🤖 Dual xArm 1s Color-Sorting Robot

An autonomous dual-arm robotic sorting system powered by **ROS 2 Humble**, **OpenCV**, and two **xArm 1s** articulated bus-servo manipulators. The system detects randomly scattered colored wooden cubes on an elevated continuous stadium arena, coordinates multi-arm task scheduling with zero collision contention, and sorts objects into dedicated color-coded bins.

---

## 📸 System Architecture Schematic

![Dual xArm 1s Color-Sorting Robot — Complete Physical, Optical & Kinematic Architecture](docs/design.png)

---

## 📌 Project Overview

This project automates color-based object sorting using computer vision, inverse kinematics, and multi-robot coordination under real-world physical and mechanical constraints:

1. **Central Collinear Alignment ($X = 360\text{ mm}$)**:
   - **ArUco #0**, **xArm 2 (Front)**, **xArm 1 (Back)**, and **ArUco #2** share a single physical line of symmetry.
   - The overhead camera focuses **100% of its sensor resolution exclusively on the picking arena**, completely excluding the collection boxes to eliminate false-positive color detections.
2. **Elevated Two-Center Stadium Arena (Right Side)**:
   - Spans a continuous $180^\circ\text{--}220^\circ$ elliptical pick envelope across the right flank of both arms.
   - Elevated by **$+20\text{ mm}$** and lined with a high-friction matte silicone mat. This aligns the $25\text{ mm}$ cubes with the natural **$65\text{ mm}$ shoulder height of Motor 2**, allowing horizontal picks in the maximum motor torque band without claw drag on the table.
3. **Safe Reach Boundaries ($d_{\text{edge}} = 5\text{--}25\text{ cm} \iff R = 120\text{--}320\text{ mm}$)**:
   - Operating reach $5\text{--}25\text{ cm}$ from baseplate edge ($12\text{--}32\text{ cm}$ from center, backing off $2.4\text{ cm}$ from straight-arm limit) maintains a healthy $\approx 122^\circ$ elbow angle, eliminating motor stalls, servo jitter, and joint singularities.
4. **Angled Drop Boxes (North-West & South-West) Resolving $240^\circ$ Base Servo ROM**:
   - The xArm 1s base bus servos (LX-15D) have a $240^\circ$ travel limit, creating an unreachable deadzone directly behind the arm ($180^\circ$ due West).
   - Placing **Box 1 in the North-West ($\approx +125^\circ$)** and **Box 2 in the South-West ($\approx -125^\circ$)** constrains total travel to $\approx 205^\circ \le 240^\circ$, eliminating servo stall.
   - Drops occur by gravity with **zero contention** (**no `box_mutex` required**), allowing both arms to drop simultaneously.
5. **Central Electronics & Power Bay**:
   - The left-center flank between the boxes ($X \in [55, 175\text{ mm}], Y \in [215, 385\text{ mm}]$) houses the NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) and dual 7.4V / 10A power hub.
   - Shortens USB and power cable runs to $< 25\text{ cm}$, eliminates wire snagging during $205^\circ$ swings, and provides clean power distribution.
6. **Pipelined Scheduling with Single `arena_mutex`**:
   - While Arm 2 picks on the right, Arm 1 drops in Box 1 (NW).
   - When Arm 2 swings left to drop in Box 2 (SW), Arm 1 immediately acquires the `arena_mutex` to pick on the right, maximizing dual-arm throughput.

---

## 📐 Workspace Field Layout

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                                                                        │
│                              ◄─── BACK OF WORKSPACE ───►                               │
│                                                                                        │
│   ┌─────────────────┐                     ┌────────┐             /───────────────────\ │
│   │  Box 1 (2×2)    │ ◄── 205° NW Drop ── │ xArm 1 │ ──────────►│  Arm 1 Arc Zone    │ │
│   │  (Arm 1 - NW)   │                     │ (Back) │    Pick    │  (Right Arena)     │ │
│   └─────────────────┘                     └────────┘             \                   / │
│                                                │  ◤ ArUco #2      │  CONTINUOUS       ││
│   ┌─────────────────┐             ═════════════╪══════════════════│- TWO-CENTER       ││
│   │  CENTRAL        │               Midline Partition (Y=300mm)   │  STADIUM ARENA    ││
│   │  ELECTRONICS    │                          │                  │  (ELEVATED 20mm)  ││
│   │  BAY: Hub & RPi │                          │                  /                   │ │
│   └─────────────────┘                     ┌────────┐             /                     │
│                                           │ xArm 2 │ ──────────►│  Arm 2 Arc Zone    │ │
│   ┌─────────────────┐ ◄── 205° SW Drop ── │(Front) │    Pick    │  (Right Arena)     │ │
│   │  Box 2 (2×2)    │                     └────────┘             \───────────────────/ │
│   │  (Arm 2 - SW)   │                            ◣ ArUco #0                  ArUco #1 ◢│
│   └─────────────────┘                                                                  │
│                              ◄─── FRONT OF WORKSPACE ───►                              │
│                                           ▲                                            │
│                                  (Operator Position)                                   │
│                                                                                        │
│   ◄─── LEFT (Drop Zone & Electronics) ───►  │  ◄─── RIGHT (Calibrated Arena) ───►      │
│   (Boxes @ Z = 0 mm, Bay @ Z = 0 mm)        │  (4 ArUco markers on Z = 20 mm)       │
└────────────────────────────────────────────────────────────────────────────────────────┘

  Collinear Axis:  ArUco #0, xArm 2, xArm 1, and ArUco #2 lie along X = 360 mm
  Vision Focus:    100% of camera pixels warped onto the picking arena
  Zero Occlusion:  Drop boxes and Central Electronics Bay are completely excluded from vision
```

---

## 🛠 Hardware Specifications

| Component           | Specification                                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------------------------------- |
| **Manipulators**    | 2× xArm 1s (5/6-DOF bus servo arms, Motor 5 shoulder axis @ $65\text{ mm}$)                                     |
| **Link Lengths**    | Base-to-Shoulder ($d_1$): $65\text{ mm}$ · Upper Arm ($L_1$): $101\text{ mm}$ · Forearm ($L_2$): $95\text{ mm}$ · Tool ($L_3$): $150\text{--}168\text{ mm}$ |
| **Gripper**         | 2-finger parallel claw ($55\text{ mm}$ opening, Motor 2 axial roll, Motor 1 clamp)                              |
| **Compute**         | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (Ubuntu 22.04 LTS, mounted in Central Electronics Bay)    |
| **Camera**          | Overhead USB webcam (Logitech C270 / C920) on boom stand @ $65\text{--}75\text{ cm}$                            |
| **Main Baseboard**  | Flat rigid mounting board ($800\text{ mm} \times 600\text{ mm}$)                                                |
| **Picking Arena**   | Raised platform ($+20\text{ mm}$) with matte silicone friction mat ($\mu \approx 0.8$)                          |
| **Collection Bins** | 2× dedicated $2\times 2$ boxes (Box 1 in NW @ $+125^\circ$, Box 2 in SW @ $-125^\circ$, $Z = 0\text{ mm}$)      |
| **Electronics Bay** | Central Flank ($X \in [55, 175\text{ mm}], Y \in [215, 385\text{ mm}]$), short runs ($< 25\text{ cm}$)          |
| **Objects**         | $25\text{ mm}$ wooden cubes spray-painted in 4 high-contrast matte colors                                       |
| **Power Supply**    | Dedicated 7.4V / 5A power supply per arm with common ground                                                     |
| **Communication**   | Direct USB HID / USB-UART serial buses (`/dev/ttyUSB0`, `/dev/ttyUSB1`) @ 115200 / 9600 baud                     |

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
- **Web Dashboard**: HTML5, CSS3, JavaScript, Rosbridge WebSocket

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
                  [ scheduler_node ] (Midline Split + arena_mutex Manager)
                    │             │
       /arm_1/tasks │             │ /arm_2/tasks
                    ▼             ▼
               [ arm_node ]   [ arm_node ]
                 (xArm 1)       (xArm 2)
                    │             │
                    ▼             ▼
              [ Serial 0 ]   [ Serial 1 ]
```

| Node             | Responsibility                                                                                                                  |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `camera_node`    | Streams raw video frames from overhead boom camera to `/camera/image_raw`                                                       |
| `vision_node`    | Detects 4 arena ArUco markers, rectifies elevated arena to mm coordinates, runs HSV thresholding, publishes `/detected_objects` |
| `scheduler_node` | Assigns cubes to Arm 1 (Back) or Arm 2 (Front) via $Y=300\text{ mm}$ midline; serializes pick requests via `arena_mutex`        |
| `arm_node` (x2)  | Computes closed-form IK, executes 5-phase radial pick trajectory, rotates $205^\circ$ to NW/SW box, drops in dedicated bin      |
| `dashboard_node` | Serves real-time web interface via `rosbridge_server` displaying annotated camera feed and arm queue states                     |

---

## 📚 Technical Documentation Index

The project documentation is organized into modular **Stepping-Stone Engineering Specifications** and system integration guides in the [`docs/`](docs/) directory:

### Stepping-Stone Specifications

- 📄 **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](docs/00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container development workflow, Intel GPU acceleration, USB hardware mapping, and ROS 2 Humble package build standards.
- 📄 **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](docs/01_arena_and_layout.md) — Mechanical construction, baseboard dimensions ($800\times 600\text{ mm}$), collinear arm mounting ($X = 360\text{ mm}$), NW/SW drop boxes resolving $240^\circ$ servo ROM, Central Electronics Bay layout, elevated arena platform ($+20\text{ mm}$), barrier profiles, lighting, and ArUco coplanar mounting.
- 📄 **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](docs/02_kinematics_and_grasp_planning.md) — Complete mathematical derivations, DH parameters, analytical IK formulas ($\theta_1\text{--}\theta_5$), top-down reach limit proof ($158.7\text{ mm}$), dual-regime wrist roll control, 5-phase trajectory waypoint profiling, torque/friction physics, and reference Python IK solver.
- 📄 **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](docs/03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography ($2.0\text{ px/mm}$), hardware exposure/WB locks, HSV color segmentation, spatial moments centroiding, and Python perception pipeline.
- 📄 **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](docs/04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, custom interface definitions, dynamic load balancing with hysteresis across $Y = 300\text{ mm}$ midline, spatial crossover mutex state machine, and production Python nodes.
- 📄 **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](docs/05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol (binary/ASCII), Arduino Uno R4 WiFi test bench, servo deviation zeroing, and tick-to-angle conversion.
- 📄 **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](docs/06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 6 grip verification, empirical Z-sag compensation, and lens undistortion pipeline.

### System Integration & Planning

- 📄 [`docs/implementation_plan.md`](docs/implementation_plan.md): High-level System Architecture Blueprint, coordinate transformations, custom ROS 2 message definitions (`DetectedObject.msg`, `ArmTask.msg`, `ArmStatus.msg`), and pipelined execution rules.
- 📄 [`docs/roadmap.md`](docs/roadmap.md): Milestone-driven Phase-Gate Verification Checklist (Gate 0 to Gate 4) replacing rigid weekly schedules.
- 📄 [`docs/risks.md`](docs/risks.md): Failure Mode and Effects Analysis (FMEA) covering risks R-01 through R-11 (including $240^\circ$ base servo ROM deadzone mitigation), optical coplanarity rules, and pre-flight checklist.

### Technical Drawings & Schematics

- 🖼️ [`docs/design.png`](docs/design.png): High-resolution architectural diagram showing baseboard dimensions, collinear axis, NW/SW drop boxes, Central Electronics Bay, and elevated stadium arena.
- 🖼️ [`docs/pcb module.webp`](docs/pcb%20module.webp): Physical xArm controller board secondary UART pinout and electrical interfacing diagram.
- 🖼️ [`docs/xArm demensions.png`](docs/xArm%20demensions.png): Factory link measurements and motor layout drawing for the xArm 1s.

---

## 📂 Project Directory Structure

```text
Sem-5-mini-project/
│
├── arduino/                        # Microcontroller Test & Calibration Firmware
│   ├── xarm_direct_test/           # Breadboard-free teleoperation, reach & pick-and-place testing
│   │   └── xarm_direct_test.ino
│   └── xarm_zero_calibration/      # Interactive servo zeroing & deviation calibration
│       └── xarm_zero_calibration.ino
│
├── config/                         # Configuration YAMLs
│   ├── dual_xarm_hardware.yaml     # Immutable link lengths, mounting origins, joint limits & 4 color compartments
│   └── servo_calibration.yaml      # Measured zero-point deviation trims per joint
│
├── docs/                           # Stepping-Stone & System Documentation
│   ├── README.md                   # Documentation Index & Architecture Map
│   ├── 00_docker_and_ros2_development_guide.md # Stepping Stone 0: Docker & ROS 2 Workflow
│   ├── 01_arena_and_layout.md      # Stepping Stone 1: Arena & Component Placement
│   ├── 02_kinematics_and_grasp_planning.md # Stepping Stone 2: Kinematics & Grasp Planning
│   ├── 03_computer_vision_and_calibration.md # Stepping Stone 3: Vision & Calibration Pipeline
│   ├── 04_ros2_architecture_and_scheduling.md # Stepping Stone 4: ROS 2 & Mutex Scheduling
│   ├── 05_hardware_bringup_and_firmware_control.md # Stepping Stone 5: Low-Level Protocols & Bringup
│   ├── 06_system_testing_diagnostics_and_verification.md # Stepping Stone 6: Testing & Diagnostics
│   ├── design.png                  # High-resolution architectural schematic
│   ├── implementation_plan.md      # System architecture & integration blueprint
│   ├── pcb module.webp             # Controller board pinout diagram
│   ├── risks.md                    # Risk matrix & failure mode analysis (R-01 to R-11)
│   ├── roadmap.md                  # Milestone-based Phase-Gate Checklist (Gate 0 to Gate 4)
│   └── xArm demensions.png         # Physical robot dimension schematic
│
├── ros2_ws/                        # ROS 2 Humble Workspace
│   └── src/
│       └── sorting_arm/            # Hardware driver node, analytical polar IK, & teleop
│           ├── launch/
│           │   └── xarm_driver.launch.py
│           ├── sorting_arm/
│           │   ├── direct_test.py      # Standalone USB HID/serial driver test
│           │   ├── driver_node.py      # ROS 2 hardware driver node with E-Stop
│           │   ├── kinematics.py       # Closed-form analytical polar IK solver
│           │   ├── polar_teleop_node.py# Keyboard polar coordinate teleop
│           │   └── xarm_protocol.py    # Multi-interface serial/HID communication protocol
│           ├── test/
│           │   ├── test_copyright.py
│           │   ├── test_flake8.py
│           │   ├── test_kinematics.py  # Analytical IK and reachability unit tests
│           │   └── test_pep257.py
│           ├── package.xml
│           ├── setup.cfg
│           └── setup.py
│       # (Planned future packages: sorting_interfaces, sorting_vision, sorting_scheduler)
│
├── .devcontainer/                 # VS Code Dev Container configuration
│   └── devcontainer.json           # 1-click isolated ROS 2 environment
│
├── docker/                         # Project-Level Docker Environment
│   ├── Dockerfile                  # Multi-arch (x86_64 / arm64) ROS 2 Humble image
│   ├── build.sh                    # Host UID/GID matching build script
│   ├── run.sh                      # Launch script with Intel GPU, GUI, & hardware pass-through
│   └── entrypoint.sh               # Environment overlay auto-source entrypoint
│
├── DOCKER_GUIDE.md                 # Complete Docker & ROS 2 Humble user manual
├── README.md                       # Project landing page & documentation
└── requirements.txt                # Python dependencies
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
- 2× xArm 1s Robotic Arms connected via USB-UART

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

# 3. Launch xArm 1s hardware driver node (auto-detects USB HID / serial)
ros2 launch sorting_arm xarm_driver.launch.py

# 4. Or run interactive polar teleoperation node
ros2 run sorting_arm polar_teleop_node

# (Future Full Pipeline — Phase 3+):
# ros2 launch sorting_vision vision.launch.py
# ros2 launch sorting_scheduler system.launch.py
```

---

### Option B: Native Installation (Ubuntu 22.04 LTS Only)

#### 1. Prerequisites
- Ubuntu 22.04 LTS with [ROS 2 Humble Desktop](https://docs.ros.org/en/humble/Installation.html)
- Python 3.10+
- USB Webcam (Logitech C270 or C920)
- 2× xArm 1s Robotic Arms connected via USB-UART

#### 2. Installation & Build
```bash
# Install Python dependencies
pip install -r requirements.txt

# Build the ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### 3. Launching the System
```bash
# Run automated kinematics tests
pytest src/sorting_arm/test/test_kinematics.py

# Launch xArm 1s hardware driver node
ros2 launch sorting_arm xarm_driver.launch.py

# Or run polar teleoperation node
ros2 run sorting_arm polar_teleop_node

# (Future Full Pipeline — Phase 3+):
# ros2 launch sorting_vision vision.launch.py
# ros2 launch sorting_scheduler system.launch.py
```

---

## 👨‍💻 Project Team

- **Dhruv Arora**
- **Shubham Jain**
- **Karan Pal**

---

## 📅 Project Status

- [x] Stepping Stone 0: Docker & ROS 2 Development Environment (`docs/00_docker_and_ros2_development_guide.md`)
- [x] Stepping Stone 1: Arena & Physical Component Placement (`docs/01_arena_and_layout.md`)
- [x] Stepping Stone 2: Kinematics & Grasp Planning (`docs/02_kinematics_and_grasp_planning.md`)
- [x] Stepping Stone 5: Hardware Bringup & Firmware Control (`docs/05_hardware_bringup_and_firmware_control.md`)
- [x] Technical Schematic Design (`docs/design.png` & `docs/pcb module.webp`)
- [x] Risk Matrix & $240^\circ$ ROM Mitigation (`docs/risks.md`)
- [x] Phase 0: Arduino Hardware Bringup, Calibration & Reach Testing (`arduino/`)
- [x] Phase 1 Core: ROS 2 Hardware Driver, Closed-Form Polar IK, & Teleop (`ros2_ws/src/sorting_arm/`)
- [ ] Physical Assembly & Arena Mounting _(Week 1)_
- [ ] OpenCV Dedicated Arena Homography & HSV Tuning _(Week 2)_
- [ ] ROS 2 Graph & Mutex Scheduler _(Week 3)_
- [ ] Single Arm Pick & Pitch LUT Calibration _(Weeks 4–6)_
- [ ] Dual Arm Pipelined Integration _(Weeks 7–8)_
- [ ] Live Web Dashboard & Robustness Polish _(Weeks 9–10)_
- [ ] Final Presentation & Demo Video _(Week 11)_
