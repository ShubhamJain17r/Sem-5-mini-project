# Risk Assessment & Mitigation Matrix — Dual xArm 1s Color-Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the dual xArm 1s system, incorporating the **inward-angled V-configuration**, **4-tile modular 3D-printable base ($560\times 520\text{ mm}$)**, **immediate flank drop wells**, **Central Rear Wedge Electronics Bay**, **+25 mm elevated chamfered arena with 12 mm retaining curb**, and **raised curb ArUco fiducials (Hybrid Pocket Method)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | +25 mm elevated arena + high-friction silicone mat + textured claw pads                                         |
| **R-02** | Kinematics | Arm sag & joint flex at extreme reach ($R > 25\text{ cm}$)    |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 320\text{ mm}$) + $65\text{ mm}$ Motor 2 height alignment                           |
| **R-03** | Vision     | Specular reflection & non-coplanar ArUco distortion           |   **High**   |   **Low**   | 4 raised curb pads ($Z = +37\text{ mm}$) with $0.8\text{ mm}$ recessed pockets for matte vinyl stickers         |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure + dedicated 5V diffuse LED ring                                                           |
| **R-05** | Hardware   | Voltage brownout / servo reset under dual-arm torque          | **Critical** | **Medium**  | Separate 7.4V / 5A power supply per arm with shared ground in Central Rear Wedge Bay                            |
| **R-06** | Comms      | USB cable snagging / serial latency jitter                    |   **High**   |   **Low**   | Molded under-deck wiring conduits with direct short USB runs ($< 18\text{ cm}$)                                 |
| **R-07** | Mechanical | Cube bounce off collection box partition rim on drop          |   **Low**    |   **Low**   | Drop wells sit at table level ($Z = 0\text{ mm}$); arm drops inside rim with damping pads                       |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist orientation (locked $\text{Roll} = 0^\circ$)                                                        |
| **R-09** | Multi-Arm  | Deadlock or race condition on shared arena overlap zone       |   **High**   |   **Low**   | Single `arena_mutex` with 5.0-second watchdog timeout; dedicated left/right flank collection wells              |
| **R-10** | Vision     | ArUco marker occlusion by arm link along perimeter            |  **Medium**  |   **Low**   | Markers placed on outer curb vertices clear of arm sweep; homography caching locks matrix                       |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone (Due West)     |   **High**   |   **Low**   | Relocate boxes to immediate lateral flanks; outward swing is only $\approx 65^\circ\text{--}75^\circ$           |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo $240^\circ$ ROM: Immediate Flank Drop Wells (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($0\text{--}1000$ tick range). Directly behind the arm base is an unreachable deadzone.
- **The Immediate Flank Solution**:
  - In the inward-angled V-configuration ($\pm 35.0^\circ$ rest heading), collection wells are placed immediately to the outer flanks of each arm.
  - To drop, the arm rotates outward by only **$\approx 65^\circ\text{--}75^\circ$** from its nominal heading.
  - Total travel from the farthest pick point to drop well is only **$\approx 105^\circ$**, well within the $240^\circ$ physical limit.
  - **Speedup & Reliability**: Drops are $>50\%$ faster with zero risk of servo stall, and independent lateral wells allow simultaneous drops without contention.

### 2.2 Raised Perimeter Curb ArUco Pads & Hybrid Pocket Method (Risk R-03)

- **Previous Vulnerability**: Glossy 3D-printed plastic reflects overhead LED light directly into the camera lens, washing out black ArUco cells. FDM 0.4 mm nozzles also round sharp square corners, degrading OpenCV sub-pixel corner refinement.
- **The Hybrid Solution**:
  - The 4 ArUco fiducial pads are molded into the vertices of the $12\text{ mm}$ elevated retaining curb ($Z = +37\text{ mm}$).
  - Each pad features a precision **$0.8\text{ mm}$ recessed pocket**.
  - High-resolution markers printed on **matte vinyl/paper sticker sheets** press-fit into the pockets, guaranteeing true photometric contrast, zero specular glare, razor-sharp edges, and sub-millimeter CAD ground truth.

### 2.3 Coplanarity Rule (Crucial Optical Physics)

- All 4 ArUco markers are molded at the exact same physical curb elevation ($Z = +37.0\text{ mm}$), ensuring the planar homography matrix $\mathbf{H}$ has zero perspective tilt or distortion.

### 2.4 Elevation Setup: +25 mm Deck with 12 mm Retaining Curb

- Motor 2 (shoulder pitch axis) is **$65\text{ mm}$** above the base. By elevating the arena platform by $+25\text{ mm}$, cube centers sit at $Z = 37.5\text{ mm}$, only $27.5\text{ mm}$ below Motor 2.
- The arm reaches out in its optimal horizontal torque band, while the continuous $12\text{ mm}$ retaining curb prevents scattered cubes from rolling off the platform.

### 2.5 Bounding $R_{\max} = 320\text{ mm}$ (Eliminating Servo Stall)

- Setting the outer reach boundary at **$R_{\max} = 320\text{ mm}$** keeps the elbow joint at $\approx 120^\circ$, preserving high torque and avoiding straight-arm lock singularities.

### 2.6 Modular Stepping-Stone References

- Consult [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Stepping Stone 1) for 4-tile modular 3D-printable base, V-configuration mounting, and flank drop wells.
- Consult [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2) for closed-form IK, reach proofs, dual-regime roll, and 5-phase trajectory profiling.
- Consult [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) for planar homography, HSV segmentation, sub-mm centroiding, and polar mapping.
- Consult [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) (Stepping Stone 4) for distributed ROS 2 Humble node graph, dynamic load balancing across $Y = 300\text{ mm}$ midline, crossover spatial mutex state machine, and Linux udev rules.

\*\* See [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) for detailed mathematical proofs showing why pure top-down picks fail beyond $158\text{ mm}$, the analytical inverse kinematics solver equations ($\theta_1\text{--}\theta_5$), dual-regime wrist roll control, 5-phase trajectory profiling, and torque/friction models.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous sorting:

- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the elevated arena platform ($Z = +20\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the elevated picking arena and completely excludes the left-side drop boxes and Central Electronics Bay.
- [ ] **Friction Check**: Test dragging a cube on the silicone mat—confirm it holds static position when lightly touched.
- [ ] **Boundary Verification**: Jog arm to $R=320\text{ mm}$; confirm claw clears outer barrier comfortably.
- [ ] **Serial Latency**: Ping both arms over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Lighting Lock**: Confirm camera auto-exposure is disabled and HSV masks show clean bounding boxes.
- [ ] **$240^\circ$ ROM Verification**: Manually jog Arm 1 to Box 1 (NW @ $+125^\circ$) and Arm 2 to Box 2 (SW @ $-125^\circ$); verify zero hard-stop contact and zero cable tension.
