# Risk Assessment & Mitigation Matrix — Dual xArm 1s Color-Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the dual xArm 1s system, incorporating the **Zero-Carpentry Flat-Board architecture ($560\times 520\times 18\text{ mm}$)**, **inward-angled V-configuration**, **immediate flank 2×2 sorting trays**, **flat rectangular arena ($Z = 0.0\text{ mm}$) with foam tape border**, and **table-coplanar ArUco fiducials ($Z = 0.0\text{ mm}$)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | 1-inch wooden cubes ($15\text{ g}$ mass) + matte primer/paint + high-friction mat + textured rubber claw pads    |
| **R-02** | Kinematics | Arm sag & joint flex at extreme reach ($R > 25\text{ cm}$)    |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 320\text{ mm}$) + empirical piecewise linear Z-sag compensation in software         |
| **R-03** | Vision     | Specular reflection & non-coplanar ArUco distortion           |   **High**   |   **Low**   | 4 matte vinyl stickers applied flat on wooden board ($Z = 0.0\text{ mm}$ coplanar with floor and cube bases)    |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure (`v4l2-ctl`) + diffuse ambient lighting; wide saturation bands                            |
| **R-05** | Hardware   | Voltage brownout / servo reset under dual-arm torque          | **Critical** | **Medium**  | Dual independent 7.4V / 5A buck regulators with common star ground and 1000 µF low-ESR buffer capacitors        |
| **R-06** | Comms      | USB cable snagging / serial latency jitter                    |   **High**   |   **Low**   | Surface adhesive cable clips with short, direct USB serial runs ($< 18\text{ cm}$) to rear-mounted SBC          |
| **R-07** | Mechanical | Cube bounce off collection tray compartment wall on drop      |   **Low**    |   **Low**   | 3D-printed 2×2 trays with $35\text{ mm}$ wall height; drop executed inside compartment cavity with gravity drop |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist roll orientation (locked $\text{Roll} = 0^\circ$ for far picks) and natural adaptive pitch           |
| **R-09** | Multi-Arm  | Deadlock or race condition on shared arena overlap zone       |   **High**   |   **Low**   | Spatial mutex for crossover zone ($X \in [220, 340\text{ mm}]$); independent outer sectors and private flank trays |
| **R-10** | Vision     | ArUco marker occlusion by arm link along perimeter            |  **Medium**  |   **Low**   | Markers placed at outer arena corners clear of normal arm sweep; homography caching locks matrix                |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone                |   **High**   |   **Low**   | Flank 2×2 trays at $(50, 360)$ and $(510, 360)$ reduce outward drop swing to only $\approx 70.0^\circ$          |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo $240^\circ$ ROM: Immediate Flank 2×2 Trays (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($0\text{--}1000$ tick range). Directly behind the arm base is an unreachable deadzone.
- **The Immediate Flank Solution**:
  - In the V-configuration, 3D-printed 2×2 grid sorting trays are positioned on the immediate lateral flanks of each arm: Left Tray at $(50, 360)$ for Arm 1, Right Tray at $(510, 360)$ for Arm 2.
  - To drop, the arm rotates outward by only **$\approx 70.0^\circ$** from its nominal heading.
  - Total travel from the farthest pick point to drop compartment is well within the $[150, 850]$ safe tick envelope.
  - **Speedup & Reliability**: Drops are $>50\%$ faster with zero risk of servo stall, and independent lateral trays allow concurrent drops without contention.

### 2.2 Coplanar Table-Level ArUco Stickers (Risk R-03)

- **Previous Vulnerability**: Glossy 3D-printed plastic reflects overhead LED light directly into the camera lens, washing out black ArUco cells. Raised curbed markers also introduce parallax errors if cubes rest at a different elevation.
- **The Coplanar Flat Solution**:
  - The 4 ArUco fiducials (`DICT_4X4_50`, IDs 0, 1, 2, 3) are printed on **matte vinyl sticker sheets** ($40\times 40\text{ mm}$) and applied directly flat to the wooden board at the four corners of the arena ($Z = 0.0\text{ mm}$).
  - Because markers share the exact plane of the arena floor and cube bases, OpenCV planar homography calculates a mathematically perfect perspective warp ($H$) with zero elevation-induced parallax error.

### 2.3 Flat Rectangular Arena & Soft Foam Border (Risk R-01, R-08)

- Motor 2 (shoulder pitch axis) is **$65.0\text{ mm}$** above the base. With $25\text{ mm}$ craft cubes resting flat on the board ($Z = 0.0\text{ mm}$), the cube grasp center sits at **$Z = 12.5\text{ mm}$**, requiring a vertical shoulder drop of $\approx 52.5\text{ mm}$.
- Analytical IK solves cleanly for all radii $r \in [130, 300\text{ mm}]$, maintaining high motor torque.
- The soft self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) retains cubes within the active zone without posing any rigid collision hazard to the robotic claw.

### 2.4 Bounding $R_{\max} = 320\text{ mm}$ (Eliminating Servo Stall)

- Setting the outer reach boundary at **$R_{\max} = 320\text{ mm}$** keeps the elbow joint at $\approx 120^\circ$, preserving high torque and avoiding straight-arm lock singularities.

### 2.5 Modular Stepping-Stone References

- Consult [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Stepping Stone 1) for zero-carpentry flat baseboard, V-configuration mounting, and flank 2×2 trays.
- Consult [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2) for closed-form IK, reach proofs, dual-regime roll, and 5-phase trajectory profiling.
- Consult [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) for planar homography, HSV segmentation, sub-mm centroiding, and polar mapping.
- Consult [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) (Stepping Stone 4) for distributed ROS 2 Humble node graph, dynamic load balancing across $X = 280\text{ mm}$ midline, crossover spatial mutex state machine, and Linux udev rules.
- Consult [`docs/07_fusion360_cad_modeling_guide.md`](07_fusion360_cad_modeling_guide.md) (Stepping Stone 7) for detailed wood blueprint, 2×2 tray CAD print parameters, and 2-hour assembly guide.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous sorting:

- [ ] **Flat Baseboard Stability**: Confirm single wooden board ($560\times 520\text{ mm}$) rests dead-flat on workstation without rocking.
- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the baseboard ($Z = 0.0\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the $400\times 300\text{ mm}$ arena and cleanly captures the 4 boundary fiducials.
- [ ] **Friction Check**: Confirm wooden craft blocks hold static position during gripper contact.
- [ ] **Boundary Verification**: Jog arm to $R = 320\text{ mm}$; confirm claw clears outer foam border comfortably.
- [ ] **Serial Latency**: Ping both arms over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Lighting Lock**: Confirm camera auto-exposure is disabled and HSV masks show clean bounding boxes.
- [ ] **Flank Tray Swings**: Manually jog Arm 1 to Left Flank Tray ($r = 120\text{ mm}$, swing $\approx 70^\circ$) and Arm 2 to Right Flank Tray; verify zero binding and zero cable tension.
