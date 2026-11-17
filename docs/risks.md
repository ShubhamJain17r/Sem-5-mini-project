# Risk Assessment & Mitigation Matrix — Single xArm 1S Red-Object Picking Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the single xArm 1S system, incorporating the **Hand-Drawn $7\times 12$ Grid Arena ($350\times 600\text{ mm}$)**, **central arm mounting at $(175, 375)$ heading $-35.0^\circ$ (South-East)**, **Rear Flank container box at $(300, 550)$**, **color discrimination methodology (Red pick, Green distractor, Blue chassis exclusion)**, and **table-coplanar ArUco fiducials ($50\times 50\text{ mm}$, $Z = 0.0\text{ mm}$)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | 1-inch wooden cubes ($15\text{ g}$ mass) + matte primer/paint + optional silicone mat + textured claw pads       |
| **R-02** | Kinematics | Arm sag & joint flex at reach ($R > 220\text{ mm}$)           |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 300\text{ mm}$) + empirical piecewise linear Z-sag compensation in software         |
| **R-03** | Vision     | Specular reflection & non-coplanar ArUco distortion           |   **High**   |   **Low**   | 4 matte vinyl stickers applied flat on wooden board ($Z = 0.0\text{ mm}$ coplanar with floor and cube bases)    |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure (`v4l2-ctl`) + diffuse ring light illumination; dual-band red thresholding                |
| **R-05** | Hardware   | Voltage brownout / servo reset under peak acceleration        | **Critical** |   **Low**   | Dedicated 7.4V / 5A buck regulator with common star ground and 1000 µF low-ESR buffer capacitor                 |
| **R-06** | Comms      | USB cable snagging / serial latency jitter                    |   **High**   |   **Low**   | Surface adhesive cable clips with short, direct USB serial run to rear-mounted SBC                             |
| **R-07** | Mechanical | Cube bounce off collection container box on drop              |   **Low**    |   **Low**   | Physical box with $35\text{ mm}$ wall height; drop executed inside container cavity $(Z=40\text{ mm})$         |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist roll orientation (locked $\text{Roll} = 0^\circ$ for far picks) and natural adaptive pitch           |
| **R-09** | Scheduling | Foreground cube collision during horizontal pick trajectory   |   **High**   |   **Low**   | Deterministic Nearest-First queue sorting policy (ascending radial distance $r_i$ from arm base)                |
| **R-10** | Vision     | ArUco marker occlusion by arm link along perimeter            |  **Medium**  |   **Low**   | Markers placed at outer arena corners clear of normal sweep; homography caching locks matrix on startup         |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone                |   **High**   |   **Low**   | Arm heading $-35.0^\circ$ maps arena pick ($139\text{--}785$ ticks) and container ($875$ ticks) within safe travel |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo $240^\circ$ ROM: Arm Heading $-35.0^\circ$ (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($126\text{--}879$ ticks). Directly behind the arm base is an unreachable mechanical deadzone.
- **The Angled Mounting Solution**:
  - The arm is mounted with a nominal heading of **$-35.0^\circ$** (South-East).
  - Picking objects across the active arena spans base yaw ticks from $139$ (far left) to $785$ (far right).
  - To drop into the Rear Flank container at $(300.0, 550.0\text{ mm})$, the base swivels to yaw tick **$875$** ($\le 879$ mechanical limit).
  - Because all operations lie strictly in $[139, 875]$, the servo never crosses the hardware deadzone.

### 2.2 Coplanar Table-Level ArUco Stickers (Risk R-03)

- The 4 ArUco fiducials (`DICT_4X4_50`, IDs 0, 1, 2, 3) are printed on **matte vinyl sticker sheets** ($50\times 50\text{ mm}$) and applied directly flat to the platform board at $Z = 0.0\text{ mm}$: $(25, 425), (325, 425), (325, 25), (25, 25)$.
- Because markers share the exact plane of the arena floor and cube bases, OpenCV planar homography calculates a mathematically perfect perspective warp ($H$) with zero elevation-induced parallax error ($600\times 800\text{ px}$ grid at $2.0\text{ px/mm}$).

### 2.3 Hand-Drawn $7\times 12$ Grid Arena & Soft Foam Border (Risk R-01, R-08)

- Motor 5 (shoulder pitch axis) is **$65.0\text{ mm}$** above the base. With $25\text{ mm}$ craft cubes resting flat on the board ($Z = 0.0\text{ mm}$), the cube grasp center sits at **$Z = 12.5\text{ mm}$**, requiring a vertical shoulder drop of $\approx 52.5\text{ mm}$.
- Analytical IK solves cleanly for all radii $r \in [160, 300\text{ mm}]$, maintaining high motor torque.
- Soft self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) retains cubes within the active zone without posing any rigid collision hazard to the robotic claw.

### 2.4 Color Discrimination & Chassis Masking (Risk R-04)

- **Target Pick (Red)**: Dual-band HSV ($[0..10]$ and $[170..180]$) detects red cubes reliably under varying lighting.
- **Distractor (Green)**: Detected for telemetry but excluded from pick queue (`targetable: false`).
- **Chassis Exclusion (Blue)**: Disabled to prevent reflections from the blue anodized aluminum arm frame. Image-space spatial mask over Rows 3..5, Cols 2..4 eliminates arm base false detections.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous picking:

- [ ] **Platform Stability**: Confirm platform board ($350\times 600\text{ mm}$) rests dead-flat on workstation without rocking.
- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the board at $(25, 425), (325, 425), (325, 25), (25, 25)$ ($Z = 0.0\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the active arena ($600\times 800\text{ px}$) and cleanly captures the 4 boundary fiducials.
- [ ] **Color Discrimination Check**: Confirm red cubes show `targetable: true` and green cubes show `targetable: false`; confirm arm chassis produces zero contours.
- [ ] **Boundary Verification**: Jog arm to $R = 300\text{ mm}$; confirm claw clears outer foam border comfortably.
- [ ] **Serial Latency**: Ping arm over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Container Drop Swing**: Manually jog arm to container position (yaw tick $875$); verify zero binding and zero cable tension.
