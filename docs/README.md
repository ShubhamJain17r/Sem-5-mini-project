# 📚 Project Documentation & Engineering Specifications
### Single xArm 1S Dual-Color (Red & Green) Sorting Robot

Welcome to the engineering documentation suite for the autonomous single-arm robotic dual-color (Red & Green) sorting system. This directory contains detailed specifications covering mechanical layout, kinematics, computer vision, ROS 2 architecture, hardware bringup, and system testing.

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
│   └── 01_arena_and_layout.md                         # Stepping Stone 1: 7x12 Arena, Component Placement & Layout
│
├── 📐 Kinematics, Grasping & Control
│   └── 02_kinematics_and_grasp_planning.md            # Stepping Stone 2: Analytical IK, Roll Control & 7-Phase Rear-Flip Trajectory
│
├── 👁️ Perception & Calibration
│   └── 03_computer_vision_and_calibration.md          # Stepping Stone 3: Optical Framing, ArUco Homography & Dual-Color Discrimination
│
├── 🤖 ROS 2 & System Coordination
│   └── 04_ros2_architecture_and_scheduling.md        # Stepping Stone 4: Computation Graph, Message Registry & Sorter
│
├── 🔌 Hardware Bringup & Low-Level Control
│   ├── 05_hardware_bringup_and_firmware_control.md    # Stepping Stone 5: Binary Serial Protocols, Servo Zeroing & Arduino Rig
│   └── 08_servo_limits_and_calibration_guide.md       # Stepping Stone 8: Servo Limits, Angle Conversion & Web Calibration
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
| [**`01_arena_and_layout.md`**](01_arena_and_layout.md) | Central single xArm 1S at $(175, 375)$ heading $-90.0^\circ$ (South), $7\times 12$ hand-drawn grid ($350\times 600\text{ mm}$), dual Rear Flank container boxes | ✅ Complete |
| [**`02_kinematics_and_grasp_planning.md`**](02_kinematics_and_grasp_planning.md) | Analytical closed-form IK ($\theta_1\text{--}\theta_5$), dual-regime wrist roll, radial approach, 7-phase overhead rear-flip trajectory | ✅ Complete |
| [**`03_computer_vision_and_calibration.md`**](03_computer_vision_and_calibration.md) | 4-point ArUco homography warp ($2.0\text{ px/mm}$, $600\times 800\text{ px}$), **Color Discrimination Methodology** (Red & Green targets, Blue chassis exclusion) | ✅ Complete |
| [**`04_ros2_architecture_and_scheduling.md`**](04_ros2_architecture_and_scheduling.md) | Computation graph, `xarm_interfaces`, Nearest-First radial sorting pipeline, autonomous pick-and-place | ✅ Complete |
| [**`05_hardware_bringup_and_firmware_control.md`**](05_hardware_bringup_and_firmware_control.md) | Hiwonder 115200 baud binary packet framing, Arduino Uno R4 test bench, servo deviation zeroing | ✅ Complete |
| [**`06_system_testing_diagnostics_and_verification.md`**](06_system_testing_diagnostics_and_verification.md) | Verification runbooks, empirical Z-sag compensation factor ($k_{\text{sag}} \approx 0.07$), claw grasp discrimination | ✅ Complete |
| [**`08_servo_limits_and_calibration_guide.md`**](08_servo_limits_and_calibration_guide.md) | Comprehensive servo travel boundaries, physical-to-tick math ($0.24^\circ/\text{tick}$), Web Config Portal | ✅ Complete |

---

## 🚦 System Blueprints & Governance

* [**`implementation_plan.md`**](implementation_plan.md): Executive summary of the entire physical, electrical, and computational architecture.
* [**`roadmap.md`**](roadmap.md): Milestone-based Phase-Gate Checklist. Guides progression from Phase 0 (hardware bringup) through Phase 3 (single-arm autonomous sorting).
* [**`risks.md`**](risks.md): Complete failure mode and effects analysis (FMEA) covering 11 critical technical risks and engineering mitigations.

---

## 🚀 Recommended Reading Path

1. **For Software & ROS 2 Developers**:  
   Start with [`00_docker_and_ros2_development_guide.md`](00_docker_and_ros2_development_guide.md) $\to$ [`04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) $\to$ [`02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md).

2. **For Vision & Perception Engineers**:  
   Start with [`01_arena_and_layout.md`](01_arena_and_layout.md) $\to$ [`03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md).

3. **For Hardware & Firmware Engineers**:  
   Start with [`05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) $\to$ [`08_servo_limits_and_calibration_guide.md`](08_servo_limits_and_calibration_guide.md) $\to$ [`06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).
