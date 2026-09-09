# Risk Assessment & Mitigation Matrix — Dual xArm 1s Color-Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the dual xArm 1s system, incorporating the **two-center stadium arena (RIGHT side)**, **dedicated 2×2 boxes (LEFT side)**, **15–20 mm elevated arena platform**, and **arena-dedicated ArUco framing (collinear with arms)**.

---

## 1. Risk Summary Matrix

| ID | Category | Risk Description | Severity | Probability | Primary Mitigation |
|---|---|---|:---:|:---:|---|
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp | **High** | **Low** | 15–20 mm elevated arena + high-friction silicone mat + textured claw pads |
| **R-02** | Kinematics | Arm sag & joint flex at extreme reach ($R > 25\text{ cm}$) | **High** | **Low** | Bounded radius ($R_{\max} = 240\text{ mm}$) + $65\text{ mm}$ Motor 2 height alignment |
| **R-03** | Vision | Non-coplanar ArUco distortion if markers sit at mixed heights | **High** | **Low** | Mount all 4 ArUco markers directly on the raised arena ($Z = 15\text{--}20\text{ mm}$) |
| **R-04** | Vision | HSV color threshold drift from changing lab lights | **High** | **Medium** | Fixed camera exposure + dedicated 5V diffuse LED ring |
| **R-05** | Hardware | Voltage brownout / servo reset under dual-arm torque | **Critical**| **Medium** | Separate 7.4V / 5A power supply per arm with shared ground |
| **R-06** | Comms | WiFi latency jitter / packet drops causing arm stutter | **High** | **Medium** | Direct USB-to-UART serial connections (`/dev/ttyUSB0`, `/dev/ttyUSB1`) |
| **R-07** | Mechanical | Cube bounce off collection box partition rim on drop | **Low** | **Low** | Boxes sit at table level ($Z = 0\text{ mm}$); arm drops inside rim |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles | **High** | **Low** | Fixed wrist orientation (locked $\text{Roll} = 0^\circ$) |
| **R-09** | Multi-Arm | Deadlock or race condition on shared arena access | **High** | **Low** | Single `arena_mutex` with 8-second lease timeout; private left boxes |
| **R-10** | Vision | ArUco marker occlusion by arm link along central axis | **Medium** | **Low** | Markers 0 & 2 offset from base wiring; homography caching locks matrix |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 The Collinear Arena ArUco Setup: Why We Moved Markers from Behind the Boxes
* **Previous Vulnerability**: Placing ArUco markers at the far left corners of the baseboard forced the camera to capture a huge $80 \times 60\text{ cm}$ field of view, wasting over 50% of the image resolution on the container boxes and empty space. Furthermore, sorted cubes inside the boxes risked triggering false-positive color detections.
* **The New Design**: Markers 0 and 2 are aligned with the central arm axis along the inner edge of the elevated arena, while Markers 1 and 3 are on the outer right boundary.
  * **Zero False Detections**: The drop boxes on the left are completely excluded from the camera's warped view.
  * **Double Pixel Density**: 100% of camera pixels are focused on the picking arena.
  * **Lower Camera Mounting**: Boom stand can sit at $60\text{--}75\text{ cm}$ directly over the arena.

### 2.2 Coplanarity Rule (Crucial Optical Physics)
* A 2D homography maps a flat plane ($Z = \text{constant}$). Because the right-side picking arena is **elevated by $15\text{--}20\text{ mm}$**, all 4 ArUco markers **must sit on that same elevated platform**. If markers sat at different heights, the homography would compute a false tilt and distort cube coordinates by up to $10\text{ mm}$.

### 2.3 The Elevation Setup: Why We Elevated the Arena and NOT the xArm
* **Built-in Height**: Motor 2 (shoulder pitch axis) is already **$65\text{ mm}$** above the base. Raising the arm base would force the arm to point steeply down, reducing horizontal reach and scraping brackets.
* **Elevating the arena by 15–20 mm**: Cube centers sit at $Z \approx 30\text{ mm}$, only $\approx 35\text{ mm}$ below Motor 2. The arm reaches **straight out horizontally** in its natural power band. Motor torque strain drops by ~30%.

### 2.4 Bounding $R_{\max} = 240\text{ mm}$ (Eliminating Servo Stall)
* Setting the outer curved boundary barrier at **$R_{\max} = 240\text{ mm}$** keeps the elbow joint at $\approx 150^\circ$, retaining full torque margin and eliminating motor stalls.

### 2.5 Comprehensive Kinematics, Mechanics & Grasp Planning Reference
* See [`docs/kinematics_and_grasp_planning.md`](file:///d:/Sem-3-mini-project/docs/kinematics_and_grasp_planning.md) for detailed mathematical proofs showing why pure top-down picks fail beyond $158\text{ mm}$, the analytical inverse kinematics solver equations ($\theta_1\text{--}\theta_5$), dual-regime wrist roll control, 5-phase trajectory profiling, and torque/friction models.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous sorting:

- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the elevated arena platform ($Z = 15\text{--}20\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the elevated picking arena and completely excludes the left-side drop boxes.
- [ ] **Friction Check**: Test dragging a cube on the silicone mat—confirm it holds static position when lightly touched.
- [ ] **Boundary Verification**: Jog arm to $R=240\text{ mm}$; confirm claw clears outer barrier comfortably.
- [ ] **Serial Latency**: Ping both arms over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Lighting Lock**: Confirm camera auto-exposure is disabled and HSV masks show clean bounding boxes.
- [ ] **180° Swing Clearance**: Manually jog arm through full 180° swing from right arena to left box; verify zero cable snagging.
