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
   - Elevated by **$15\text{--}20\text{ mm}$** and lined with a high-friction matte silicone mat. This aligns the $25\text{ mm}$ cubes with the natural **$65\text{ mm}$ shoulder height of Motor 2**, allowing horizontal picks in the maximum motor torque band without claw drag on the table.
3. **Safe Reach Boundaries ($R_{\min} = 100\text{ mm}$, $R_{\max} = 240\text{ mm}$)**:
   - Capping reach at $24\text{ cm}$ (instead of the straight-arm $29.9\text{ cm}$ limit) maintains a healthy $\approx 150^\circ$ elbow angle, eliminating motor stalls, servo jitter, and joint singularities.
4. **Dual Dedicated Collection Boxes (Left Side at $Z = 0\text{ mm}$)**:
   - Each arm has its own dedicated $2\times 2$ sorting box reachable via a smooth $180^\circ$ swing.
   - Drops occur by gravity with **zero contention** (**no `box_mutex` required**), allowing both arms to drop simultaneously.
5. **Pipelined Scheduling with Single `arena_mutex`**:
   - While Arm 2 picks on the right, Arm 1 drops on the left.
   - When Arm 2 swings left to drop, Arm 1 immediately acquires the `arena_mutex` to pick on the right, maximizing dual-arm throughput.

---

## 📐 Workspace Field Layout

```
┌────────────────────────────────────────────────────────────────────────┐
│                                                                        │
│                      ◄─── BACK OF WORKSPACE ───►                       │
│                                                                        │
│   ┌─────────────┐             ┌────────┐         /───────────────────\ │
│   │ Box 1 (2×2) │ ◄── 180° ── │ xArm 1 │ ──────►│  Arm 1 Arc Zone   │ │
│   │  (Arm 1)    │    Drop     │ (Back) │  Pick  │  (180°–220° Right)│ │
│   └─────────────┘             └────────┘         \                   / │
│                                   │  ◤ ArUco #2   │  CONTINUOUS       ││
│                        Midline Partition Line - - │- TWO-CENTER       ││
│                                   │ (Y = 300 mm)  │  STADIUM ARENA    ││
│   ┌─────────────┐             ┌────────┐         /   (ELEVATED 15mm) │ │
│   │ Box 2 (2×2) │ ◄── 180° ── │ xArm 2 │ ──────►│  Arm 2 Arc Zone   │ │
│   │  (Arm 2)    │    Drop     │(Front) │  Pick  │  (180°–220° Right)│ │
│   └─────────────┘             └────────┘         \───────────────────/ │
│                                      ◣ ArUco #0              ArUco #1 ◢│
│                      ◄─── FRONT OF WORKSPACE ───►                      │
│                                ▲                                       │
│                       (Operator Position)                              │
│                                                                        │
│   ◄─── LEFT (Drop Zone) ───►  │  ◄─── RIGHT (Calibrated Arena) ───►    │
│   (Boxes sit at Z = 0 mm)     │  (4 ArUco markers on Z = 15-20mm)      │
└────────────────────────────────────────────────────────────────────────┘

  Collinear Axis:  ArUco #0, xArm 2, xArm 1, and ArUco #2 lie along X = 360 mm
  Vision Focus:    100% of camera pixels warped onto the picking arena
  Zero Occlusion:  Drop boxes are completely excluded from the vision homography
```

---

## 🛠 Hardware Specifications

| Component | Specification |
|---|---|
| **Manipulators** | 2× xArm 1s (5/6-DOF bus servo arms, Motor 2 shoulder axis @ $65\text{ mm}$) |
| **Link Lengths** | Base-to-Shoulder: $65\text{ mm}$ · Upper Arm: $101\text{ mm}$ · Forearm: $95\text{ mm}$ · Tool: $150\text{ mm}$ |
| **Gripper** | 2-finger parallel claw ($55\text{ mm}$ opening, Motor 5 axial roll, Motor 6 clamp) |
| **Compute** | Jetson Nano / Raspberry Pi 5 (Ubuntu 22.04 LTS) |
| **Camera** | Overhead USB webcam (Logitech C270 / C920) on boom stand @ $65\text{--}75\text{ cm}$ |
| **Main Baseboard** | Flat mounting board ($800\text{ mm} \times 600\text{ mm}$) |
| **Picking Arena** | Raised platform ($+15\text{--}20\text{ mm}$) with matte silicone friction mat ($\mu \approx 0.8$) |
| **Collection Bins** | 2× dedicated $2\times 2$ boxes on left flank ($Z = 0\text{ mm}$, cells: Red, Green, Blue, Yellow) |
| **Objects** | $25\text{ mm}$ wooden cubes spray-painted in 4 high-contrast matte colors |
| **Power Supply** | Dedicated 7.4V / 5A power supply per arm with common ground |
| **Communication** | Direct USB-UART serial buses (`/dev/ttyUSB0`, `/dev/ttyUSB1`) @ 115200 baud |

---

## 💻 Software Stack & Architecture

- **Operating System**: Linux (Ubuntu 22.04 LTS)
- **Middleware**: ROS 2 Humble Hawksbill
- **Computer Vision**: OpenCV (ArUco planar homography, HSV color masking, contour moments)
- **Kinematics**: Analytical closed-form Inverse Kinematics + Distance-to-Pitch LUT
- **Web Dashboard**: HTML5, CSS3, JavaScript, Rosbridge WebSocket

### ROS 2 Graph & Node Roles

```
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

| Node | Responsibility |
|---|---|
| `camera_node` | Streams raw video frames from overhead boom camera to `/camera/image_raw` |
| `vision_node` | Detects 4 arena ArUco markers, rectifies elevated arena to mm coordinates, runs HSV thresholding, publishes `/detected_objects` |
| `scheduler_node` | Assigns cubes to Arm 1 (Back) or Arm 2 (Front) via $Y=300\text{ mm}$ midline; serializes pick requests via `arena_mutex` |
| `arm_node` (x2) | Computes closed-form IK, executes 5-phase radial pick trajectory, rotates $180^\circ$ left, drops in dedicated bin |
| `dashboard_node` | Serves real-time web interface via `rosbridge_server` displaying annotated camera feed and arm queue states |

---

## 📚 Technical Documentation Index

Detailed engineering documentation is located in the [`docs/`](docs/) directory:

- 📄 [`docs/implementation_plan.md`](docs/implementation_plan.md): Full technical architecture, coordinate transformations, custom ROS 2 message definitions (`DetectedObject.msg`, `ArmTask.msg`, `ArmStatus.msg`), and pipelined execution rules.
- 📄 [`docs/kinematics_and_grasp_planning.md`](docs/kinematics_and_grasp_planning.md): Complete mathematical derivations, DH parameters, analytical IK formulas ($\theta_1\text{--}\theta_5$), dual-regime wrist roll control, 5-phase trajectory waypoint profiling, torque/friction physics, and reference Python IK solver.
- 📄 [`docs/roadmap.md`](docs/roadmap.md): 11-week college-only development schedule detailing milestones from physical assembly and vision calibration to dual-arm stress testing and final demo.
- 📄 [`docs/risks.md`](docs/risks.md): Failure Mode and Effects Analysis (FMEA) covering risks R-01 through R-10, coplanarity optical rules, servo stall mitigations, and pre-flight checklist.
- 🖼️ [`docs/design.png`](docs/design.png): High-resolution architectural diagram showing baseboard dimensions, collinear axis, elevated arena, and left-side drop boxes.
- 🖼️ [`docs/xArm demensions.png`](docs/xArm%20demensions.png): Factory link measurements and motor layout drawing for the xArm 1s.

---

## 📂 Project Directory Structure

```
Sem-3-mini-project/
│
├── config/                         # Configuration YAMLs
│   ├── colors.yaml                 # Tuned HSV color threshold ranges
│   ├── bin_coordinates.yaml        # Calibrated 2×2 box drop coordinates
│   └── camera_calibration.yaml     # Camera intrinsic and distortion params
│
├── docs/                           # Core Project Documentation
│   ├── design.png                  # High-resolution architectural schematic
│   ├── implementation_plan.md      # Detailed system implementation plan
│   ├── kinematics_and_grasp_planning.md # Complete IK & grasp planning guide
│   ├── risks.md                    # Risk matrix & failure mode analysis
│   ├── roadmap.md                  # 11-week college development roadmap
│   └── xArm demensions.png         # Physical robot dimension schematic
│
├── dashboard/                      # Web Dashboard UI
│   ├── index.html                  # Operator dashboard interface
│   ├── styles.css                  # UI styling & status indicators
│   └── app.js                      # Rosbridge WebSocket client
│
├── ros2_ws/                        # ROS 2 Humble Workspace
│   └── src/
│       ├── sorting_interfaces/     # Custom message definitions
│       │   ├── msg/DetectedObject.msg
│       │   ├── msg/ArmTask.msg
│       │   └── msg/ArmStatus.msg
│       ├── sorting_vision/         # Camera & OpenCV processing nodes
│       ├── sorting_scheduler/      # Task allocation & arena_mutex
│       └── sorting_arm/            # Serial driver & motion planning
│
├── vision/                         # Standalone OpenCV Calibration Scripts
│   ├── calibrate_camera.py         # Intrinsic camera calibration script
│   ├── test_aruco_warp.py          # Dedicated arena homography verification
│   └── tune_hsv.py                 # Interactive HSV color tuning tool
│
├── README.md                       # Project landing page & documentation
└── requirements.txt                # Python dependencies
```

---

## 🚀 Getting Started

### 1. Prerequisites
- Ubuntu 22.04 LTS with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Python 3.10+
- USB Webcam (Logitech C270 or C920)
- 2× xArm 1s Robotic Arms connected via USB-UART

### 2. Installation
```bash
# Clone the repository
git clone <repository-url>
cd Sem-3-mini-project

# Install Python dependencies
pip install -r requirements.txt

# Build the ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launching the System
```bash
# Launch camera and vision pipeline
ros2 launch sorting_vision vision.launch.py

# Launch dual-arm scheduler and controllers
ros2 launch sorting_scheduler system.launch.py
```

---

## 👨‍💻 Project Team

- **Dhruv Arora**
- **Shubham Jain**
- **Karan Pal**

---

## 📅 Project Status

- [x] Workspace & Arena Mechanical Design
- [x] Analytical Kinematics & Grasp Planning
- [x] Optical Layout & ArUco Collinear Framing
- [ ] Physical Assembly & Hardware Mounting *(Week 1)*
- [ ] OpenCV Dedicated Arena Homography *(Week 2)*
- [ ] ROS 2 Graph & Mutex Scheduler *(Week 3)*
- [ ] Single Arm Pick & Pitch LUT Calibration *(Weeks 4–6)*
- [ ] Dual Arm Pipelined Integration *(Weeks 7–8)*
- [ ] Live Web Dashboard & Robustness Polish *(Weeks 9–10)*
- [ ] Final Presentation & Demo Video *(Week 11)*
