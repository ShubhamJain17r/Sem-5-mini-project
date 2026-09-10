# Risk Assessment & Mitigation Matrix — Dual xArm 1s Color-Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the dual xArm 1s system, incorporating the **two-center stadium arena (RIGHT side)**, **NW / SW dedicated 2×2 boxes (LEFT side)**, **Central Electronics Bay**, **20 mm elevated arena platform**, and **arena-dedicated ArUco framing (collinear with arms)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | 20 mm elevated arena + high-friction silicone mat + textured claw pads                                          |
| **R-02** | Kinematics | Arm sag & joint flex at extreme reach ($R > 25\text{ cm}$)    |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 320\text{ mm}$) + $65\text{ mm}$ Motor 2 height alignment                           |
| **R-03** | Vision     | Non-coplanar ArUco distortion if markers sit at mixed heights |   **High**   |   **Low**   | Mount all 4 ArUco markers directly on the raised arena ($Z = +20\text{ mm}$)                                    |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure + dedicated 5V diffuse LED ring                                                           |
| **R-05** | Hardware   | Voltage brownout / servo reset under dual-arm torque          | **Critical** | **Medium**  | Separate 7.4V / 5A power supply per arm with shared ground in Central Electronics Bay                           |
| **R-06** | Comms      | WiFi latency jitter / packet drops causing arm stutter        |   **High**   | **Medium**  | Direct USB-to-UART serial connections (`/dev/ttyUSB0`, `/dev/ttyUSB1`)                                          |
| **R-07** | Mechanical | Cube bounce off collection box partition rim on drop          |   **Low**    |   **Low**   | Boxes sit at table level ($Z = 0\text{ mm}$); arm drops inside rim with damping pads                            |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist orientation (locked $\text{Roll} = 0^\circ$)                                                        |
| **R-09** | Multi-Arm  | Deadlock or race condition on shared arena access             |   **High**   |   **Low**   | Single `arena_mutex` with 8-second lease timeout; private NW/SW left boxes                                      |
| **R-10** | Vision     | ArUco marker occlusion by arm link along central axis         |  **Medium**  |   **Low**   | Markers 0 & 2 offset from base wiring; homography caching locks matrix                                          |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone (Due West)     |   **High**   |   **Low**   | Relocate Box 1 to NW ($\approx +125^\circ$) and Box 2 to SW ($\approx -125^\circ$); total swing $\le 205^\circ$ |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo $240^\circ$ ROM: Why We Angled Drop Boxes to NW and SW (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($0\text{--}1000$ tick range). With the physical center facing East ($0^\circ$) toward the picking arena, the arm can turn $\pm 120^\circ$. Directly behind the arm ($180^\circ$ due West) is an unreachable deadzone. Attempting a $180^\circ$ drop turn will stall or strip the internal gear train.
- **The NW / SW Solution**:
  - Box 1 is placed in the North-West corner at $\approx +125^\circ$ relative to Arm 1.
  - Box 2 is placed in the South-West corner at $\approx -125^\circ$ relative to Arm 2.
  - Arm 1 rotates CCW by $\approx 205^\circ$ from its southernmost pick angle to drop in Box 1.
  - Arm 2 rotates CW by $\approx 205^\circ$ from its northernmost pick angle to drop in Box 2.
  - Both paths stay safely inside the $240^\circ$ physical limit, guaranteeing zero servo stall.
- **Central Electronics Bay Bonus**: Clearing the central-left area creates space for the NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) and 7.4V power distribution hub directly between the arms, shortening serial and power cables to $< 25\text{ cm}$.

### 2.2 The Collinear Arena ArUco Setup: Why We Moved Markers from Behind the Boxes

- **Previous Vulnerability**: Placing ArUco markers at the far left corners of the baseboard forced the camera to capture a huge $80 \times 60\text{ cm}$ field of view, wasting over 50% of the image resolution on the container boxes and empty space. Furthermore, sorted cubes inside the boxes risked triggering false-positive color detections.
- **The New Design**: Markers 0 and 2 are aligned with the central arm axis along the inner edge of the elevated arena ($X = 360\text{ mm}$), while Markers 1 and 3 are on the outer right boundary ($X = 730\text{ mm}$).
  - **Zero False Detections**: The drop boxes on the left are completely excluded from the camera's warped view.
  - **Double Pixel Density**: 100% of camera pixels are focused on the picking arena.
  - **Lower Camera Mounting**: Boom stand can sit at $65\text{--}75\text{ cm}$ directly over the arena.

### 2.3 Coplanarity Rule (Crucial Optical Physics)

- A 2D homography maps a flat plane ($Z = \text{constant}$). Because the right-side picking arena is **elevated by $+20\text{ mm}$**, all 4 ArUco markers **must sit on that same elevated platform**. If markers sat at different heights, the homography would compute a false tilt and distort cube coordinates by up to $10\text{ mm}$.

### 2.4 The Elevation Setup: Why We Elevated the Arena and NOT the xArm

- **Built-in Height**: Motor 2 (shoulder pitch axis) is already **$65\text{ mm}$** above the base. Raising the arm base would force the arm to point steeply down, reducing horizontal reach and scraping brackets.
- **Elevating the arena by 20 mm**: Cube centers sit at $Z \approx 32.5\text{ mm}$, only $32.5\text{ mm}$ below Motor 2. The arm reaches **straight out horizontally** in its natural power band. Motor torque strain drops by ~30%.

### 2.5 Bounding $R_{\max} = 320\text{ mm}$ (Eliminating Servo Stall)

- Setting the outer curved boundary barrier at **$R_{\max} = 320\text{ mm}$** keeps the elbow joint at $\approx 122^\circ$, retaining full torque margin and eliminating motor stalls.

### 2.6 Modular Stepping-Stone References

- Consult [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) (Stepping Stone 1) for physical assembly, NW/SW boxes, and power bay.
- Consult [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) (Stepping Stone 2) for closed-form IK, reach proofs, dual-regime roll, and 5-phase trajectory profiling.
- Consult [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) (Stepping Stone 3) for planar homography, HSV segmentation, sub-mm centroiding, and polar mapping.

** See [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) for detailed mathematical proofs showing why pure top-down picks fail beyond $158\text{ mm}$, the analytical inverse kinematics solver equations ($\theta_1\text{--}\theta_5$), dual-regime wrist roll control, 5-phase trajectory profiling, and torque/friction models.

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
