# 📚 Project Documentation & Engineering Specifications
### Single xArm 1S Color-Sorting Robot

Welcome to the engineering documentation suite for the autonomous single-arm robotic sorting system. This directory contains detailed specifications covering mechanical layout, kinematics, computer vision, ROS 2 architecture, hardware bringup, and system testing.

---

## 🗺️ Documentation Architecture

The specifications are organized chronologically as modular **Stepping Stones** alongside strategic system blueprints:

```text
docs/
├── README.md                                          # Documentation Suite Index (This File)
├── MANUAL.md                                          # 📖 Comprehensive Operator Manual & Debugging Guide
│
├── 🛠️ Development & Operator Guides
│   ├── MANUAL.md                                      # End-to-End Operator Manual (Python, Server, ROS 2, Debugging)
│   ├── 00_docker_and_ros2_development_guide.md       # Stepping Stone 0: Docker & ROS 2 Humble Development Guide
│   └── ../DOCKER_GUIDE.md                            # Comprehensive Docker & Hardware Pass-Through User Manual
│
├── 🏗️ Physical & Spatial Architecture
│   ├── arena_bill_of_materials.md                     # Master BOM, Hardware Checklist, Fasteners & Drill Bits
│   ├── 01_arena_and_layout.md                         # Stepping Stone 1: Arena Construction, Layout & Dimensions
│   ├── 07_fusion360_cad_modeling_guide.md             # Stepping Stone 7: Physical Workcell Fabrication & Assembly Guide
│   └── assets/                                        # Blueprints & diagrams in assets/images/ & assets/aruco_markers/
│
├── 📐 Kinematics, Grasping & Control
│   └── 02_kinematics_and_grasp_planning.md            # Stepping Stone 2: Analytical IK, Roll Control & 5-Phase Trajectory
│
├── 👁️ Perception & Calibration
│   └── 03_computer_vision_and_calibration.md          # Stepping Stone 3: Optical Framing, ArUco Homography & HSV Segmentation
│
├── 🤖 ROS 2 & System Coordination
│   └── 04_ros2_architecture_and_scheduling.md        # Stepping Stone 4: Computation Graph, Message Registry & Spatial Mutex
│
├── 🔌 Hardware Bringup & Low-Level Control
│   └── 05_hardware_bringup_and_firmware_control.md    # Stepping Stone 5: Binary Serial Protocols, Servo Zeroing & Arduino Rig
│
├── 🧪 Testing & Verification
│   └── 06_system_testing_diagnostics_and_verification.md # Stepping Stone 6: Z-Sag Compensation, Grip Testing & Diagnostics
│
└── 🎯 Project Management & Roadmaps
    ├── implementation_plan.md                         # Master Integration Blueprint & System Overview
    ├── roadmap.md                                     # Phase-Gate Quality Checklist (Gate 0 to Gate 4)
    └── risks.md                                       # Risk Matrix & Failure Mode Mitigation (R-01 to R-11)
```

---

## 📖 Stepping Stone Overview

| Document | Key Topics Covered | Status |
| :--- | :--- | :---: |
| [**`MANUAL.md`**](MANUAL.md) | **End-to-End Operator Manual & Debugging Guide**: Python venv, Web Server, ROS 2 Docker workflow, and fault troubleshooting | ✅ Complete |
| [**`00_docker_and_ros2_development_guide.md`**](00_docker_and_ros2_development_guide.md) | Docker workflow, hybrid VS Code editing, package creation, `colcon` build lifecycle, hardware mapping | ✅ Complete |
| [**`01_arena_and_layout.md`**](01_arena_and_layout.md) | Central single xArm 1S, Zero-Carpentry flat baseboard ($400\times 400\text{ mm}$), rectangular arena ($Z=0\text{ mm}$) with foam border, Left Flank 2×2 sorting tray | ✅ Complete |
| [**`02_kinematics_and_grasp_planning.md`**](02_kinematics_and_grasp_planning.md) | Analytical closed-form IK ($\theta_1\text{--}\theta_5$), dual-regime wrist roll, radial approach, 5-phase trajectory | ✅ Complete |
| [**`03_computer_vision_and_calibration.md`**](03_computer_vision_and_calibration.md) | 4-point ArUco homography warp ($2.0\text{ px/mm}$), exposure lock, HSV color segmentation, cube centroid extraction | ✅ Complete |
| [**`04_ros2_architecture_and_scheduling.md`**](04_ros2_architecture_and_scheduling.md) | Computation graph, `xarm_interfaces`, Nearest-First radial sorting pipeline, autonomous pick-and-place | ✅ Complete |
| [**`05_hardware_bringup_and_firmware_control.md`**](05_hardware_bringup_and_firmware_control.md) | Hiwonder 115200 baud binary packet framing, Arduino Uno R4 test bench, servo deviation zeroing | ✅ Complete |
| [**`06_system_testing_diagnostics_and_verification.md`**](06_system_testing_diagnostics_and_verification.md) | Verification runbooks, empirical Z-sag compensation factor ($k_{\text{sag}} \approx 0.07$), claw grasp discrimination | ✅ Complete |
| [**`07_fusion360_cad_modeling_guide.md`**](07_fusion360_cad_modeling_guide.md) | Zero-carpentry flat board build, 2×2 tray 3D print specs, technical blueprints & 2-hour assembly guide | ✅ Complete |

---

## 🚦 System Blueprints & Governance

* [**`implementation_plan.md`**](implementation_plan.md): Executive summary of the entire physical, electrical, and computational architecture.
* [**`roadmap.md`**](roadmap.md): Milestone-based Phase-Gate Checklist. Guides progression from Phase 0 (Arduino hardware bringup) through Phase 3 (single-arm autonomous sorting).
* [**`risks.md`**](risks.md): Complete failure mode and effects analysis (FMEA) covering 11 critical technical risks and engineering mitigations.

---

## 🚀 Recommended Reading Path

1. **For Software & ROS 2 Developers**:  
   Start with [`00_docker_and_ros2_development_guide.md`](00_docker_and_ros2_development_guide.md) $\to$ [`04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) $\to$ [`02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md).

2. **For Vision, Mechanical & Perception Engineers**:  
   Start with [`01_arena_and_layout.md`](01_arena_and_layout.md) $\to$ [`07_fusion360_cad_modeling_guide.md`](07_fusion360_cad_modeling_guide.md) $\to$ [`03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md).

3. **For Hardware & Firmware Engineers**:  
   Start with [`05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) $\to$ [`06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).
