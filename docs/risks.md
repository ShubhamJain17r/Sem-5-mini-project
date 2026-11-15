# Risk Assessment & Mitigation Matrix — Single xArm 1s Color-Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the single xArm 1s system, incorporating the **Zero-Carpentry Flat-Board architecture ($400\times 400\times 18\text{ mm}$)**, **central arm mounting at $(200, 280)$ facing South ($-90^\circ$)**, **immediate Left Flank 2×2 sorting tray at $(50, 280)$**, **flat rectangular arena ($260\times 140\text{ mm}$, $Z = 0.0\text{ mm}$) with foam tape border**, and **table-coplanar ArUco fiducials ($Z = 0.0\text{ mm}$)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | 1-inch wooden cubes ($15\text{ g}$ mass) + matte primer/paint + optional silicone mat + textured claw pads       |
| **R-02** | Kinematics | Arm sag & joint flex at reach ($R > 180\text{ mm}$)           |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 280\text{ mm}$) + empirical piecewise linear Z-sag compensation in software         |
| **R-03** | Vision     | Specular reflection & non-coplanar ArUco distortion           |   **High**   |   **Low**   | 4 matte vinyl stickers applied flat on wooden board ($Z = 0.0\text{ mm}$ coplanar with floor and cube bases)    |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure (`v4l2-ctl`) + diffuse ring light illumination; wide saturation bands                     |
| **R-05** | Hardware   | Voltage brownout / servo reset under peak acceleration        | **Critical** |   **Low**   | Dedicated 7.4V / 5A buck regulator with common star ground and 1000 µF low-ESR buffer capacitor                 |
| **R-06** | Comms      | USB cable snagging / serial latency jitter                    |   **High**   |   **Low**   | Surface adhesive cable clips with short, direct USB serial run ($< 18\text{ cm}$) to rear-mounted SBC           |
| **R-07** | Mechanical | Cube bounce off collection tray compartment wall on drop      |   **Low**    |   **Low**   | 3D-printed 2×2 tray with $35\text{ mm}$ wall height; drop executed inside compartment cavity with gravity drop  |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist roll orientation (locked $\text{Roll} = 0^\circ$ for far picks) and natural adaptive pitch           |
| **R-09** | Scheduling | Foreground cube collision during horizontal pick trajectory   |   **High**   |   **Low**   | Deterministic Nearest-First queue sorting policy (ascending radial distance $r_i$ from arm base)                |
| **R-10** | Vision     | ArUco marker occlusion by arm link along perimeter            |  **Medium**  |   **Low**   | Markers placed at outer arena corners clear of normal sweep; homography caching locks matrix on startup         |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone                |   **High**   |   **Low**   | Left Flank 2×2 tray at $(50, 280)$ reduces drop swing to clean $90^\circ$ clockwise (drop yaw tick $\approx 145$)|

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo $240^\circ$ ROM: Immediate Left Flank 2×2 Tray (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($0\text{--}1000$ tick range). Directly behind the arm base is an unreachable deadzone.
- **The Immediate Flank Solution**:
  - The 3D-printed 2×2 grid sorting tray is positioned on the immediate left flank of the arm at $(50.0\text{ mm}, 280.0\text{ mm})$.
  - To drop, the arm rotates clockwise by only **$90.0^\circ$** from its nominal South heading (drop yaw tick $\approx 145$).
  - Total travel from the farthest pick point to drop compartment is well within the $[140, 860]$ safe tick envelope.
  - **Speedup & Reliability**: Drops are fast with zero risk of servo stall or gear strain.

### 2.2 Coplanar Table-Level ArUco Stickers (Risk R-03)

- **Previous Vulnerability**: Glossy 3D-printed plastic reflects overhead LED light directly into the camera lens, washing out black ArUco cells. Raised curbed markers also introduce parallax errors if cubes rest at a different elevation.
- **The Coplanar Flat Solution**:
  - The 4 ArUco fiducials (`DICT_4X4_50`, IDs 0, 1, 2, 3) are printed on **matte vinyl sticker sheets** ($30\times 30\text{ mm}$) and applied directly flat to the wooden board at the four corners of the arena ($Z = 0.0\text{ mm}$): $(70, 180), (330, 180), (70, 40), (330, 40)$.
  - Because markers share the exact plane of the arena floor and cube bases, OpenCV planar homography calculates a mathematically perfect perspective warp ($H$) with zero elevation-induced parallax error ($520\times 280\text{ px}$ grid at $2.0\text{ px/mm}$).

### 2.3 Flat Rectangular Arena & Soft Foam Border (Risk R-01, R-08)

- Motor 5 (shoulder pitch axis) is **$65.0\text{ mm}$** above the base. With $25\text{ mm}$ craft cubes resting flat on the board ($Z = 0.0\text{ mm}$), the cube grasp center sits at **$Z = 12.5\text{ mm}$**, requiring a vertical shoulder drop of $\approx 52.5\text{ mm}$.
- Analytical IK solves cleanly for all radii $r \in [100, 280\text{ mm}]$, maintaining high motor torque.
- The soft self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) retains cubes within the active zone without posing any rigid collision hazard to the robotic claw.

### 2.4 Bounding $R_{\max} = 280\text{ mm}$ (Eliminating Servo Stall)

- Setting the outer reach boundary at **$R_{\max} = 280\text{ mm}$** backs off safely before the $344\text{ mm}$ full extension limit, preserving high torque and avoiding straight-arm lock singularities.

### 2.5 Modular Stepping-Stone References

- Consult [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Stepping Stone 1) for zero-carpentry $400\times 400\text{ mm}$ flat baseboard, arm mount at $(200, 280)$, and Left Flank 2×2 tray.
- Consult [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2) for closed-form IK, reach proofs, dual-regime roll, and 5-phase trajectory profiling.
- Consult [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) for planar homography, HSV segmentation, sub-mm centroiding, and polar mapping.
- Consult [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) (Stepping Stone 4) for single-arm ROS 2 Humble node graph and autonomous sorting pipeline.
- Consult [`docs/07_fusion360_cad_modeling_guide.md`](07_fusion360_cad_modeling_guide.md) (Stepping Stone 7) for detailed wood blueprint, 2×2 tray CAD print parameters, and 2-hour assembly guide.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous sorting:

- [ ] **Flat Baseboard Stability**: Confirm single wooden board ($400\times 400\text{ mm}$) rests dead-flat on workstation without rocking.
- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the baseboard at $(70, 180), (330, 180), (70, 40), (330, 40)$ ($Z = 0.0\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the $260\times 140\text{ mm}$ arena ($520\times 280\text{ px}$) and cleanly captures the 4 boundary fiducials.
- [ ] **Friction Check**: Confirm wooden craft blocks hold static position during gripper contact.
- [ ] **Boundary Verification**: Jog arm to $R = 280\text{ mm}$; confirm claw clears outer foam border comfortably.
- [ ] **Serial Latency**: Ping arm over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Lighting Lock**: Confirm camera auto-exposure is disabled and HSV masks show clean bounding boxes.
- [ ] **Flank Tray Swing**: Manually jog arm to Left Flank Tray ($r = 150\text{ mm}$, swing $90^\circ$ clockwise); verify zero binding and zero cable tension.
