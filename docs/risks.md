# Risk Assessment & Mitigation Matrix — Single xArm 1S Dual-Color Sorting Robot

This document details the physical, optical, kinematic, and algorithmic failure modes identified for the single xArm 1S system, incorporating the **Hand-Drawn $7\times 12$ Grid Arena ($350\times 600\text{ mm}$)**, **central arm mounting at $(175, 375)$ heading $-90.0^\circ$ (South)**, **dual Rear Flank container boxes (Red @ 650 ticks, Green @ 354 ticks)**, **dual-color sorting methodology (Red & Green targets, Blue chassis exclusion)**, and **table-coplanar ArUco fiducials ($50\times 50\text{ mm}$, $Z = 0.0\text{ mm}$)**.

---

## 1. Risk Summary Matrix

| ID       | Category   | Risk Description                                              |   Severity   | Probability | Primary Mitigation                                                                                              |
| -------- | ---------- | ------------------------------------------------------------- | :----------: | :---------: | --------------------------------------------------------------------------------------------------------------- |
| **R-01** | Mechanical | Cube sliding during horizontal/angled grasp                   |   **High**   |   **Low**   | 1-inch wooden cubes ($15\text{ g}$ mass) + matte primer/paint + optional silicone mat + textured claw pads       |
| **R-02** | Kinematics | Arm sag & joint flex at reach ($R > 220\text{ mm}$)           |   **High**   |   **Low**   | Bounded radius ($R_{\max} = 300\text{ mm}$) + empirical piecewise linear Z-sag compensation in software         |
| **R-03** | Vision     | Specular reflection & non-coplanar ArUco distortion           |   **High**   |   **Low**   | 4 matte vinyl stickers applied flat on wooden board ($Z = 0.0\text{ mm}$ coplanar with floor and cube bases)    |
| **R-04** | Vision     | HSV color threshold drift from changing lab lights            |   **High**   | **Medium**  | Fixed camera exposure (`v4l2-ctl`) + diffuse ring light illumination; dual-band red & green thresholding        |
| **R-05** | Hardware   | Voltage brownout / servo reset under peak acceleration        | **Critical** |   **Low**   | Dedicated 7.4V / 5A buck regulator with common star ground and 1000 µF low-ESR buffer capacitor                 |
| **R-06** | Comms      | USB cable snagging / serial latency jitter                    |   **High**   |   **Low**   | Surface adhesive cable clips with short, direct USB serial run to rear-mounted SBC                             |
| **R-07** | Mechanical | Cube bounce off collection container boxes on drop            |   **Low**    |   **Low**   | Physical boxes with $35\text{ mm}$ wall height; overhead rear-flip drop executed from $Z = 45\text{ mm}$ standoff |
| **R-08** | Mechanical | Table gouging from claw rotation at low angles                |   **High**   |   **Low**   | Fixed wrist roll orientation (locked $\text{Roll} = 0^\circ$ for far picks) and natural adaptive pitch           |
| **R-09** | Scheduling | Foreground cube collision during horizontal pick trajectory   |   **High**   |   **Low**   | Deterministic Nearest-First queue sorting policy (ascending radial distance $r_i$ from arm base)                |
| **R-10** | Vision     | ArUco marker occlusion by arm link along perimeter            |  **Medium**  |   **Low**   | Markers placed at outer arena corners clear of normal sweep; homography caching locks matrix on startup         |
| **R-11** | Hardware   | Base Servo $240^\circ$ ROM Mechanical Deadzone                |   **High**   |   **Low**   | Overhead rear-flip geometry confines all motion to $354\text{--}650$ ticks, well within limits ($100\text{--}900$) |

---

## 2. Key Architecture Decisions & How They Resolved Major Risks

### 2.1 Base Servo Travel & Overhead Rear-Flip Transfer (Risk R-11)

- **The Hardware Constraint**: LewanSoul / Hiwonder LX-15D bus servos have a mechanical travel range of $240^\circ$ ($0\text{--}1000$ ticks, configured limits $100\text{--}900$ ticks). Directly behind the arm base can be an unreachable deadzone if swept along the table.
- **The Overhead Rear-Flip Solution**:
  - The arm is mounted at the exact center of its base length and width at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$ with a nominal heading of **$-90.0^\circ$** (pointing South down Column 3 midline).
  - Front arena picking spans base yaw ticks from $394$ (far right, $-26^\circ$) to $610$ (far left, $+26^\circ$) around center $502$.
  - Rather than sweeping the base across the table, the arm lifts the grasped object to safe transit altitude ($Z \ge 130\text{ mm}$), flips overhead by pitching Motors 5, 4, 3 into a rear-facing (+Y) stance, and slews Motor 6 to the container box:
    - **Red Container Box** (Rear-Left, Rows 0..1, Cols 0..1): base yaw tick **$650$** ($+35.5^\circ$).
    - **Green Container Box** (Rear-Right, Rows 0..1, Cols 5..6): base yaw tick **$354$** ($-35.5^\circ$).
  - Total Motor 6 travel required across the entire sorting cycle is only $\approx 71^\circ$ (ticks $354\text{--}650$). The base servo remains $> 250$ ticks away from its software safety limits ($100$ and $900$), completely eliminating deadzone risk and cable tension!

### 2.2 Coplanar Table-Level ArUco Stickers (Risk R-03)

- The 4 ArUco fiducials (`DICT_4X4_50`, IDs 0, 1, 2, 3) are printed on **matte vinyl sticker sheets** ($50\times 50\text{ mm}$) and applied directly flat to the platform board at $Z = 0.0\text{ mm}$: Marker 0 at $(25, 425)$, Marker 1 at $(325, 425)$, Marker 2 at $(25, 25)$, Marker 3 at $(325, 25)$.
- Because markers share the exact plane of the arena floor and cube bases, OpenCV planar homography calculates a mathematically perfect perspective warp ($H$) with zero elevation-induced parallax error ($600\times 800\text{ px}$ grid at $2.0\text{ px/mm}$).

### 2.3 Hand-Drawn $7\times 12$ Grid Arena & Soft Foam Border (Risk R-01, R-08)

- Motor 5 (shoulder pitch axis) is **$65.0\text{ mm}$** above the base. With $25\text{ mm}$ craft cubes resting flat on the board ($Z = 0.0\text{ mm}$), the cube grasp center sits at **$Z = 12.5\text{ mm}$**, requiring a vertical shoulder drop of $\approx 52.5\text{ mm}$.
- Analytical IK solves cleanly for all radii $r \in [160, 300\text{ mm}]$, maintaining high motor torque.
- Soft self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall) retains cubes within the active zone without posing any rigid collision hazard to the robotic claw.

### 2.4 Color Discrimination & Chassis Masking (Risk R-04)

- **Target Pick 1 (Red)**: Dual-band HSV ($[0..10]$ and $[170..180]$) detects red cubes reliably under varying lighting; routed to Red Container Box at yaw $650$.
- **Target Pick 2 (Green)**: HSV band ($[38..85]$) detects green cubes reliably; routed to Green Container Box at yaw $354$.
- **Chassis Exclusion (Blue)**: Disabled to prevent reflections from the blue anodized aluminum arm frame. Image-space spatial mask over Rows 3..5, Cols 2..4 eliminates arm base false detections.

---

## 3. Pre-Flight Verification Checklist

Before running autonomous picking:

- [ ] **Platform Stability**: Confirm platform board ($350\times 600\text{ mm}$) rests dead-flat on workstation without rocking.
- [ ] **Coplanar Marker Level**: Confirm all 4 ArUco markers sit flat on the board at $(25, 425), (325, 425), (25, 25), (325, 25)$ ($Z = 0.0\text{ mm}$).
- [ ] **Arena ROI Check**: Confirm the homography warp only shows the active arena ($600\times 800\text{ px}$) and cleanly captures the 4 boundary fiducials.
- [ ] **Color Discrimination Check**: Confirm both Red and Green cubes show `targetable: true`; confirm arm chassis produces zero contours.
- [ ] **Boundary Verification**: Jog arm to $R = 300\text{ mm}$; confirm claw clears outer foam border comfortably.
- [ ] **Serial Latency**: Ping arm over USB serial; confirm latency $< 15\text{ ms}$.
- [ ] **Overhead Rear-Flip Test**: Verify motion sequence 13 runs cleanly to Red container ($650$ ticks) and Green container ($354$ ticks) with smooth overhead transit ($Z \ge 130\text{ mm}$).
